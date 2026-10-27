#!/usr/bin/env python3
"""Selectable ORB or XFeat/LighterGlue PnP initialization from RGB-D references."""

import json
import math
import sqlite3
import time
from pathlib import Path
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

from lite3_patrol_nav.visual_matching import solve_candidate, unique_ratio_matches, next_reference_batch


def matrix_yaw(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def appearance_vector(image_bgr):
    small = cv2.resize(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY), (32, 18), interpolation=cv2.INTER_AREA).astype(np.float32)
    small -= float(small.mean())
    norm = float(np.linalg.norm(small))
    return (small/max(norm, 1e-6)).ravel()


class VisualLocalizer(Node):
    def __init__(self):
        super().__init__("rgbd_visual_localizer")
        self.declare_parameter("matching_backend", "orb")
        self.declare_parameter("xfeat_root", "/home/ysc/lite3_patrol_project/third_party/accelerated_features")
        self.declare_parameter("xfeat_dependencies", "/home/ysc/lite3_patrol_project/third_party/xfeat_deps")
        self.declare_parameter("xfeat_top_k", 1024)
        self.declare_parameter("xfeat_candidates", 2)
        self.declare_parameter("database", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("pose_topic", "/visual_localization/pose")
        self.declare_parameter("map_frame", "visual_map")
        self.declare_parameter("processing_rate_hz", 2.0)
        self.declare_parameter("ratio_test", 0.75)
        self.declare_parameter("candidate_count", 4)
        self.declare_parameter("pnp_candidate_count", 4)
        self.declare_parameter("recovery_search_interval_s", 5.0)
        self.declare_parameter("recovery_pnp_candidate_count", 12)
        self.declare_parameter("learned_fallback", True)
        self.declare_parameter("learned_candidate_count", 2)
        self.declare_parameter("learned_required_references", 2)
        self.declare_parameter("max_query_age_s", 1.5)
        self.declare_parameter("max_input_age_s", 1.5)
        self.declare_parameter("learned_dependencies", "/home/ysc/lite3_patrol_project/tools/pnp_lab/deps")
        self.declare_parameter("learned_weights", str(Path.home()/".cache/torch/hub/checkpoints/loftr_outdoor.ckpt"))
        self.declare_parameter("min_matches", 24)
        self.declare_parameter("min_inliers", 24)
        self.declare_parameter("max_reprojection_error", 4.0)
        self.declare_parameter("ambiguity_ratio", 0.90)
        self.declare_parameter("ambiguity_distance_m", 2.0)
        self.declare_parameter("search_strategy", "startup_then_global")
        self.declare_parameter("startup_keyframe_count", 6)
        self.declare_parameter("startup_timeout_s", 8.0)
        self.declare_parameter("confirmation_samples", 3)
        self.declare_parameter("confirmation_max_position_m", 0.50)
        self.declare_parameter("confirmation_max_yaw_rad", 0.35)
        self.declare_parameter("confirmation_max_gap_s", 2.5)
        self.declare_parameter("startup_appearance_fallback", True)
        self.declare_parameter("startup_appearance_max_score", 0.60)
        self.declare_parameter("startup_appearance_top_count", 5)
        self.declare_parameter("startup_appearance_min_support", 3)

        self.backend = str(self.get_parameter("matching_backend").value)
        if self.backend not in ("orb", "xfeat_lighterglue"):
            raise ValueError("matching_backend must be orb or xfeat_lighterglue")
        self.localization_ready = False
        self.visual_paused = False
        database = self.get_parameter("database").value
        if not database:
            raise RuntimeError("database parameter is empty")
        connection = sqlite3.connect("file:{}?mode=ro".format(database), uri=True)
        metadata = {key: json.loads(value) for key, value in connection.execute("SELECT key,value FROM metadata")}
        self.k = np.asarray(metadata["color_k"], dtype=np.float64)
        self.t_base_camera = np.asarray(metadata["base_to_color_optical"], dtype=np.float64)
        self.keyframes = []
        self.reference_size = None
        for row in connection.execute("SELECT id,map_camera,points,descriptors,feature_count,color_jpeg,keypoints FROM keyframes ORDER BY id"):
            keyframe_id, pose_blob, points_blob, descriptor_blob, count, jpeg_blob, pixel_blob = row
            reference = cv2.imdecode(np.frombuffer(jpeg_blob, dtype=np.uint8), cv2.IMREAD_COLOR)
            if reference is None:
                raise RuntimeError("invalid reference JPEG for keyframe {}".format(keyframe_id))
            self.reference_size = (reference.shape[1], reference.shape[0])
            histogram = cv2.calcHist([cv2.cvtColor(reference, cv2.COLOR_BGR2HSV)], [0, 1], None, [30, 32], [0, 180, 0, 256])
            cv2.normalize(histogram, histogram)
            retained = int(count)
            self.keyframes.append({
                "id": keyframe_id,
                "map_camera": np.frombuffer(pose_blob, dtype="<f8").reshape(4, 4).copy(),
                "points": np.frombuffer(points_blob, dtype="<f4").reshape(count, 3)[:retained].copy(),
                "pixels": np.frombuffer(pixel_blob, dtype="<f4").reshape(count, 2)[:retained].copy(),
                "color_jpeg": jpeg_blob,
                "descriptors": np.frombuffer(descriptor_blob, dtype=np.uint8).reshape(count, 32)[:retained].copy(),
                "histogram": histogram,
                "appearance": appearance_vector(reference),
            })
        if self.backend == "xfeat_lighterglue" and connection.execute(
                "SELECT 1 FROM sqlite_master WHERE type='table' AND name='xfeat_features'").fetchone():
            cached = {row[0]: row[1:] for row in connection.execute(
                "SELECT id,keypoints,descriptors,points,feature_count,depth_source FROM xfeat_features")}
            for frame in self.keyframes:
                if frame["id"] not in cached:
                    raise RuntimeError("incomplete XFeat cache")
                frame["xfeat_cache"] = cached[frame["id"]]
                frame["xfeat_metadata"] = metadata["xfeat"]
        connection.close()
        if not self.keyframes:
            raise RuntimeError("visual database contains no keyframes")

        configured_startup_ids = metadata.get("startup_keyframe_ids", [])
        if not configured_startup_ids:
            count = max(1, int(self.get_parameter("startup_keyframe_count").value))
            configured_startup_ids = [frame["id"] for frame in self.keyframes[:count]]
        startup_ids = set(int(value) for value in configured_startup_ids)
        startup_ids.update(
            int(value) for value in metadata.get("startup_region_keyframe_ids", [])
        )
        self.startup_keyframes = [
            frame for frame in self.keyframes if int(frame["id"]) in startup_ids
        ]
        if not self.startup_keyframes:
            self.startup_keyframes = self.keyframes[:1]
        startup_pose = metadata.get("startup_pose_map_base")
        if startup_pose is None:
            fallback = self.startup_keyframes[0]["map_camera"] @ np.linalg.inv(
                self.t_base_camera
            )
            startup_pose = [
                float(fallback[0, 3]), float(fallback[1, 3]),
                float(fallback[2, 3]), matrix_yaw(fallback),
            ]
        self.startup_anchor = np.eye(4, dtype=np.float64)
        anchor_yaw = float(startup_pose[3])
        self.startup_anchor[:3, :3] = np.asarray([
            [math.cos(anchor_yaw), -math.sin(anchor_yaw), 0.0],
            [math.sin(anchor_yaw), math.cos(anchor_yaw), 0.0],
            [0.0, 0.0, 1.0],
        ])
        self.startup_anchor[:3, 3] = np.asarray(startup_pose[:3], dtype=np.float64)

        strategy = str(self.get_parameter("search_strategy").value).strip().lower()
        if strategy not in ("startup_then_global", "startup_only", "global"):
            raise RuntimeError("invalid search_strategy: {}".format(strategy))
        self.search_strategy = strategy
        self.search_mode = "global" if strategy == "global" else "startup"
        self.search_started = 0.0
        self.confirmation_history = deque(
            maxlen=max(1, int(self.get_parameter("confirmation_samples").value))
        )

        self.xfeat_matcher = None
        if self.backend == "xfeat_lighterglue":
            from lite3_patrol_nav.xfeat_matching import XFeatMatcher
            self.get_logger().info("preparing XFeat/LighterGlue reference features before accepting images")
            self.xfeat_matcher = XFeatMatcher(
                self.get_parameter("xfeat_root").value,
                self.get_parameter("xfeat_dependencies").value,
                int(self.get_parameter("xfeat_top_k").value))
            self.xfeat_matcher.prepare(self.keyframes)
            self.xfeat_matcher.warmup()
            self.search_mode = "global"
            self.search_strategy = "global"
        # Both backends only provide the initial pose. Keep expensive ORB/
        # LoFTR inference off the navigation workload once NDT confirms READY.
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/patrol/localization_ready", self.ready_callback, ready_qos)
        self.create_subscription(String, "/patrol/visual_init_state", self.initializer_state_callback, 10)
        self.orb = cv2.ORB_create(nfeatures=900, scaleFactor=1.2, nlevels=8, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last_image_time = 0.0
        self.latest_image = None
        self.last_processed_stamp = None
        self.last_state = "STARTING"
        self.last_diagnostic_time = 0.0
        self.last_recovery_search = 0.0
        self.failed_searches = 0
        self.learned_matcher = None
        self.learned_warmed_this_cycle = False
        self.learned_unavailable = False
        self.last_learned_detail = "not_attempted"
        self.learned_tried_ids = set()
        self.learned_preferred_ids = []
        self.learned_misses = 0
        self.metric_pose_seen = False
        self.confirmation_source = None
        self.last_candidate_time = 0.0
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10
        )
        self.state_publisher = self.create_publisher(String, "/visual_localization/state", 10)
        self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self.image_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        processing_rate = max(
            0.1, float(self.get_parameter("processing_rate_hz").value)
        )
        self.create_timer(1.0 / processing_rate, self.processing_callback)
        self.create_timer(1.0, self.health_callback)
        self.publish_state(
            "READY:keyframes={},startup={},strategy={}".format(
                len(self.keyframes), len(self.startup_keyframes), self.search_strategy
            )
        )
        self.get_logger().info(
            "loaded {} visual keyframes; startup references={}; strategy={}".format(
                len(self.keyframes), len(self.startup_keyframes), self.search_strategy
            )
        )

    def ready_callback(self, message):
        self.localization_ready = bool(message.data)
        if self.localization_ready and not self.visual_paused:
            self.visual_paused = True
            self.latest_image = None
            self.confirmation_history.clear()
            self.publish_state("PAUSED:localization_ready_ndt_odometry_active")
        # Losing READY alone must not trigger a new initialization search.
        # Only the initializer's explicit request resumes the visual frontend.

    def initializer_state_callback(self, message):
        if (not self.visual_paused
                or message.data != "COLLECTING_VISUAL_POSES"):
            return
        self.visual_paused = False
        self.localization_ready = False
        self.confirmation_history.clear()
        self.search_started = 0.0
        self.latest_image = None
        self.last_processed_stamp = None
        self.metric_pose_seen = False
        self.last_candidate_time = 0.0
        self.publish_state("WAITING_FOR_IMAGE:reinitialization")

    def xfeat_solutions(self, color, appearance_ranked):
        query = self.xfeat_matcher.query(color)
        solutions, details = [], []
        frames = self.xfeat_matcher.select_candidates(
            [frame for _, frame in appearance_ranked],
            int(self.get_parameter("xfeat_candidates").value))
        for frame in frames:
            result, detail = self.xfeat_matcher.match(
                frame, query, self.k, int(self.get_parameter("min_matches").value),
                int(self.get_parameter("min_inliers").value),
                float(self.get_parameter("max_reprojection_error").value))
            details.append("{}:raw={},depth={},inliers={},reason={}".format(
                frame["id"], detail["raw_matches"], detail["matches"], detail["inliers"], detail["reason"]))
            if result is not None:
                pose = frame["map_camera"] @ np.linalg.inv(result["transform"]) @ np.linalg.inv(self.t_base_camera)
                solutions.append(dict(keyframe=frame["id"], matches=detail["matches"],
                    inliers=result["inliers"], reprojection=result["reprojection"],
                    pose=pose, source="xfeat_lighterglue_pnp"))
        self.last_learned_detail = "|".join(details)
        self.xfeat_matcher.remember(solutions)
        return solutions

    def publish_state(self, value):
        changed = value != self.last_state
        self.last_state = value
        message = String()
        message.data = value
        self.state_publisher.publish(message)
        # Keep rejection details in logs, even when no state subscriber is up.
        now = time.monotonic()
        if (changed and value.startswith("PAUSED:")) or (value.startswith(("REJECTED:", "ERROR:", "POSE:", "WAITING_FOR_IMAGE")) and now - self.last_diagnostic_time >= 5.0):
            self.get_logger().info(value)
            self.last_diagnostic_time = now

    def health_callback(self):
        if self.visual_paused:
            self.publish_state("PAUSED:localization_ready_ndt_odometry_active")
            return
        if not self.last_image_time:
            self.publish_state("WAITING_FOR_IMAGE")
        elif time.monotonic() - self.last_image_time > 2.0:
            self.publish_state("ERROR:image_timeout")
        else:
            self.publish_state(self.last_state)

    @staticmethod
    def to_bgr(message):
        if message.encoding.lower() not in ("rgb8", "bgr8"):
            raise RuntimeError("unsupported color encoding {}".format(message.encoding))
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = image[:, :message.width*3].reshape(message.height, message.width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if message.encoding.lower() == "rgb8" else image

    def image_callback(self, message):
        if self.visual_paused:
            return
        # Never run expensive matching in the 30 Hz subscription callback.
        # Keeping only the newest frame prevents old camera messages from
        # starving timers and makes relocalization latency bounded.
        self.last_image_time = time.monotonic()
        self.latest_image = message

    def learned_solutions(self, color, appearance_ranked, search_ids):
        if not self.get_parameter("learned_fallback").value or self.learned_unavailable:
            return []
        if self.learned_matcher is None:
            try:
                from lite3_patrol_nav.learned_matching import LearnedMatcher
                self.get_logger().info("loading local LoFTR recovery matcher")
                self.learned_matcher = LearnedMatcher(
                    self.get_parameter("learned_dependencies").value,
                    self.get_parameter("learned_weights").value)
                self.learned_matcher.warmup(color)
            except Exception as error:
                self.learned_matcher = None
                self.learned_unavailable = True
                self.get_logger().warn("LoFTR recovery unavailable: {}".format(error))
                return []
            # Cold loading and CUDA warmup are not live-frame inference.
            # Return to the executor, discard this image, then use a new frame.
            self.learned_warmed_this_cycle = True
            return []
        query = self.learned_matcher.query(color)
        candidates = [frame for _score, frame in appearance_ranked
                      if int(frame["id"]) in search_ids]
        limit = max(2, int(self.get_parameter("learned_candidate_count").value))
        preferred = [frame for frame in candidates if frame["id"] in self.learned_preferred_ids]
        if preferred and int(self.get_parameter("learned_required_references").value) == 1:
            # The NDT-guarded path needs one verified reference. Reusing it
            # keeps live pose latency below the guard's sensor freshness limit.
            limit = 1
        candidates = next_reference_batch(
            candidates, self.learned_preferred_ids, self.learned_tried_ids, limit)
        solutions = []
        details = []
        for frame in candidates:
            result, detail = self.learned_matcher.match(
                frame, query, self.k, int(self.get_parameter("min_matches").value),
                int(self.get_parameter("min_inliers").value),
                float(self.get_parameter("max_reprojection_error").value))
            details.append("{}:{}/{}".format(frame["id"], detail["matches"], detail["inliers"]))
            if result is None:
                continue
            pose = frame["map_camera"] @ np.linalg.inv(result["transform"]) @ np.linalg.inv(self.t_base_camera)
            solutions.append(dict(keyframe=frame["id"], matches=detail["matches"],
                                  inliers=result["inliers"], reprojection=result["reprojection"],
                                  pose=pose, source="learned_pnp"))
        # Standalone use requires independent references. The guarded navigation
        # launcher may accept one PnP reference: NDT must independently confirm
        # that seed before localization_ready can permit any movement.
        consistent = []
        for first in solutions:
            for second in solutions:
                if first is second:
                    continue
                distance = np.linalg.norm(first["pose"][:2, 3]-second["pose"][:2, 3])
                yaw = abs(angle_delta(matrix_yaw(first["pose"]), matrix_yaw(second["pose"])))
                if distance <= 0.30 and yaw <= 0.20:
                    consistent.append(first)
                    break
        if int(self.get_parameter("learned_required_references").value) == 1:
            consistent = solutions
        self.last_learned_detail = "{};agree={}".format("|".join(details), len(consistent))
        if consistent or solutions:
            self.learned_preferred_ids = [item["keyframe"] for item in sorted(
                consistent or solutions, key=lambda item: item["inliers"], reverse=True)[:2]]
            self.learned_misses = 0
        else:
            self.learned_misses += 1
            if self.learned_misses >= 5:
                self.learned_preferred_ids = []
        return consistent

    def processing_callback(self):
        if self.visual_paused:
            return
        message = self.latest_image
        if message is None:
            return
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        self.learned_warmed_this_cycle = False
        input_age = (self.get_clock().now().nanoseconds / 1e9
                     - float(stamp[0]) - stamp[1] / 1e9)
        if (stamp[0] * 1000000000 + stamp[1] <= 0 or input_age < -0.5
                or input_age > float(self.get_parameter("max_input_age_s").value)):
            self.confirmation_history.clear()
            self.publish_state("REJECTED:stale_input_{:.2f}s".format(input_age))
            return
        now = time.monotonic()
        try:
            if self.search_started <= 0.0:
                self.search_started = now
            if (
                self.search_strategy == "startup_then_global"
                and self.search_mode == "startup"
                and now - self.search_started
                >= float(self.get_parameter("startup_timeout_s").value)
            ):
                self.search_mode = "global"
                self.confirmation_history.clear()
                self.get_logger().warn(
                    "no confirmed start-area match; switching to global keyframe search"
                )

            processing_started = time.monotonic()
            self.publish_state("SEARCHING:mode={}".format(self.search_mode))

            color = self.to_bgr(message)
            if (color.shape[1], color.shape[0]) != self.reference_size:
                self.confirmation_history.clear()
                self.publish_state("ERROR:image_size_mismatch:live={}x{},map={}x{}".format(
                    color.shape[1], color.shape[0], *self.reference_size))
                return
            keypoints, query_descriptors = [], None
            if self.backend == "orb":
                gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
                keypoints, query_descriptors = self.orb.detectAndCompute(gray, None)
                if query_descriptors is None or len(query_descriptors) < 30:
                    self.confirmation_history.clear()
                    self.publish_state("REJECTED:too_few_query_features")
                    return
            # A cheap appearance prefilter limits expensive ORB matching to a
            # handful of globally plausible places.
            query_histogram = cv2.calcHist([cv2.cvtColor(color, cv2.COLOR_BGR2HSV)], [0, 1], None, [30, 32], [0, 180, 0, 256])
            cv2.normalize(query_histogram, query_histogram)
            query_appearance = appearance_vector(color)
            search_pool = (
                self.startup_keyframes
                if self.search_mode == "startup"
                else self.keyframes
            )
            appearance_ranked = sorted((
                (
                    0.25 * cv2.compareHist(
                        query_histogram, frame["histogram"], cv2.HISTCMP_BHATTACHARYYA
                    )
                    + 0.75 * (
                        1.0 - float(np.dot(query_appearance, frame["appearance"]))
                    ),
                    frame,
                )
                for frame in self.keyframes
            ), key=lambda value: value[0])
            search_ids = {int(frame["id"]) for frame in search_pool}
            appearance = [
                frame for _score, frame in appearance_ranked
                if int(frame["id"]) in search_ids
            ][:int(self.get_parameter("candidate_count").value)]
            recovery = (
                self.search_mode == "global" and self.failed_searches >= 3
                and now - self.last_recovery_search >= float(
                    self.get_parameter("recovery_search_interval_s").value)
            )
            if recovery:
                # Lighting and moving shadows can put the right place outside
                # the low-resolution appearance shortlist. Periodically use
                # descriptor evidence across the entire map instead.
                appearance = search_pool
                self.last_recovery_search = now

            appearance_coarse = False
            if (
                self.backend == "orb"
                and self.search_mode == "startup"
                and bool(self.get_parameter("startup_appearance_fallback").value)
                and not self.metric_pose_seen
            ):
                top_count = max(
                    1, int(self.get_parameter("startup_appearance_top_count").value)
                )
                top = appearance_ranked[:top_count]
                startup_id_set = {int(frame["id"]) for frame in self.startup_keyframes}
                support = sum(
                    1 for _score, frame in top if int(frame["id"]) in startup_id_set
                )
                appearance_coarse = (
                    bool(top)
                    and float(top[0][0])
                    <= float(self.get_parameter("startup_appearance_max_score").value)
                    and support
                    >= int(self.get_parameter("startup_appearance_min_support").value)
                )

            solutions = []
            max_matches = max_inliers = tried = 0
            best_failed_keyframe = -1
            if self.backend == "xfeat_lighterglue":
                solutions = self.xfeat_solutions(color, appearance_ranked)
            elif appearance_coarse:
                solutions.append({
                    "keyframe": appearance_ranked[0][1]["id"],
                    "matches": 0,
                    "inliers": 0,
                    "reprojection": 10.0,
                    "pose": self.startup_anchor.copy(),
                    "source": "start_appearance",
                })
            else:
                ranked = []
                ratio = float(self.get_parameter("ratio_test").value)
                for keyframe in appearance:
                    good = unique_ratio_matches(
                        self.matcher, keyframe["descriptors"], query_descriptors, ratio)
                    if len(good) > max_matches:
                        max_matches = len(good)
                        best_failed_keyframe = keyframe["id"]
                    if len(good) >= int(self.get_parameter("min_matches").value):
                        ranked.append((len(good), keyframe, good))
                ranked.sort(key=lambda item: item[0], reverse=True)
                candidate_limit = "recovery_pnp_candidate_count" if recovery else "pnp_candidate_count"
                for match_count, keyframe, matches in ranked[:max(
                    1, int(self.get_parameter(candidate_limit).value)
                )]:
                    tried += 1
                    object_points = np.asarray([keyframe["points"][match.queryIdx] for match in matches], np.float32)
                    image_points = np.asarray([keypoints[match.trainIdx].pt for match in matches], np.float32)
                    solution, detail = solve_candidate(
                        object_points, image_points, self.k,
                        int(self.get_parameter("min_inliers").value),
                        float(self.get_parameter("max_reprojection_error").value))
                    if detail["inliers"] > max_inliers:
                        max_inliers = detail["inliers"]
                        best_failed_keyframe = keyframe["id"]
                    if solution is None:
                        continue
                    t_current_key = solution["transform"]
                    t_map_current_camera = keyframe["map_camera"] @ np.linalg.inv(t_current_key)
                    t_map_base = t_map_current_camera @ np.linalg.inv(self.t_base_camera)
                    solutions.append({
                        "keyframe": keyframe["id"], "matches": match_count, "inliers": solution["inliers"],
                        "reprojection": solution["reprojection"], "pose": t_map_base,
                        "source": "pnp",
                    })
            if not solutions and self.backend == "orb":
                solutions = self.learned_solutions(color, appearance_ranked, search_ids)
                if self.learned_warmed_this_cycle:
                    self.latest_image = None
                    self.confirmation_history.clear()
                    self.publish_state("WARMUP_COMPLETE:waiting_for_new_image")
                    return
            if not solutions:
                self.failed_searches += 1
                if now - self.last_candidate_time > float(self.get_parameter("confirmation_max_gap_s").value):
                    self.confirmation_history.clear()
                if self.backend == "xfeat_lighterglue":
                    self.publish_state("REJECTED:xfeat_lighterglue:no_pnp_solution:" + self.last_learned_detail)
                    return
                self.publish_state(
                    "REJECTED:{}:no_pnp_solution:reason={},query={},refs={},tried={},max_matches={},max_inliers={},kf={},recovery={},learned={}".format(
                        self.search_mode, "insufficient_inliers" if tried else "too_few_matches",
                        len(keypoints), len(appearance), tried,
                        max_matches, max_inliers, best_failed_keyframe, int(recovery), self.last_learned_detail)
                )
                return
            self.failed_searches = 0
            solutions.sort(key=lambda item: (item["inliers"], -item["reprojection"]), reverse=True)
            best = solutions[0]
            if best["source"] != "start_appearance" and len(solutions) > 1 and solutions[1]["inliers"] >= best["inliers"]*float(self.get_parameter("ambiguity_ratio").value):
                first, second = best["pose"], solutions[1]["pose"]
                distance = float(np.linalg.norm(first[:2, 3]-second[:2, 3]))
                yaw_difference = abs(angle_delta(matrix_yaw(first), matrix_yaw(second)))
                if distance > float(self.get_parameter("ambiguity_distance_m").value) or yaw_difference > 0.7:
                    self.confirmation_history.clear()
                    self.publish_state("REJECTED:{}:ambiguous_{:.2f}m_{:.2f}rad".format(self.search_mode, distance, yaw_difference))
                    return

            pose = best["pose"]
            yaw = matrix_yaw(pose)
            age = (self.get_clock().now().nanoseconds/1e9
                   - float(message.header.stamp.sec) - message.header.stamp.nanosec/1e9)
            if age < -0.5 or age > float(self.get_parameter("max_query_age_s").value):
                self.confirmation_history.clear()
                self.publish_state("REJECTED:{}:stale_query_{:.2f}s,input_age={:.2f}s,inference={:.2f}s".format(
                    self.search_mode, age, input_age, time.monotonic() - processing_started))
                return
            if best["source"] != "start_appearance":
                self.metric_pose_seen = True
            if (self.confirmation_source != best["source"]
                    or now - self.last_candidate_time > float(self.get_parameter("confirmation_max_gap_s").value)):
                self.confirmation_history.clear()
            self.confirmation_source = best["source"]
            self.last_candidate_time = now
            self.confirmation_history.append((float(pose[0, 3]), float(pose[1, 3]), yaw))
            required = max(1, int(self.get_parameter("confirmation_samples").value))
            if len(self.confirmation_history) < required:
                self.publish_state(
                    "CANDIDATE:{}:{}/{},kf={},inliers={},x={:.2f},y={:.2f},yaw={:.2f}".format(
                        self.search_mode, len(self.confirmation_history), required,
                        best["keyframe"], best["inliers"], pose[0, 3], pose[1, 3], yaw,
                    )
                )
                return
            xs = [value[0] for value in self.confirmation_history]
            ys = [value[1] for value in self.confirmation_history]
            reference_yaw = self.confirmation_history[0][2]
            position_span = max(
                math.hypot(x-reference_x, y-reference_y)
                for x, y in zip(xs, ys)
                for reference_x, reference_y in [(xs[0], ys[0])]
            )
            yaw_span = max(
                abs(angle_delta(value[2], reference_yaw))
                for value in self.confirmation_history
            )
            if (
                position_span
                > float(self.get_parameter("confirmation_max_position_m").value)
                or yaw_span
                > float(self.get_parameter("confirmation_max_yaw_rad").value)
            ):
                last = self.confirmation_history[-1]
                self.confirmation_history.clear()
                self.confirmation_history.append(last)
                self.publish_state(
                    "REJECTED:{}:temporal_inconsistent_{:.2f}m_{:.2f}rad".format(
                        self.search_mode, position_span, yaw_span
                    )
                )
                return

            # Once a start-area solution is temporally confirmed, keep the
            # restricted pool.  This prevents a repetitive corridor farther
            # along the route from stealing the initial-pose estimate.
            if self.search_mode == "startup" and best["source"] != "start_appearance":
                self.search_strategy = "startup_only"
            result = PoseWithCovarianceStamped()
            result.header.stamp = message.header.stamp
            result.header.frame_id = self.get_parameter("map_frame").value
            result.pose.pose.position.x = float(pose[0, 3])
            result.pose.pose.position.y = float(pose[1, 3])
            result.pose.pose.position.z = float(pose[2, 3])
            result.pose.pose.orientation.z = math.sin(yaw/2.0)
            result.pose.pose.orientation.w = math.cos(yaw/2.0)
            sigma_xy = (
                0.45 if best["source"] == "start_appearance"
                else max(0.05, min(0.40, best["reprojection"]*0.025+1.0/math.sqrt(best["inliers"])))
            )
            sigma_yaw = max(0.04, min(0.30, sigma_xy*0.8))
            result.pose.covariance[0] = sigma_xy*sigma_xy
            result.pose.covariance[7] = sigma_xy*sigma_xy
            result.pose.covariance[14] = 0.09
            result.pose.covariance[35] = sigma_yaw*sigma_yaw
            self.pose_publisher.publish(result)
            self.publish_state(
                "POSE:mode={},source={},kf={},inliers={},x={:.2f},y={:.2f},yaw={:.2f},query_age={:.2f}s,input_age={:.2f}s,inference={:.2f}s".format(
                    self.search_mode, best["source"], best["keyframe"], best["inliers"], pose[0, 3], pose[1, 3], yaw,
                    age, input_age, time.monotonic() - processing_started
                )
            )
            elapsed = time.monotonic() - processing_started
            if elapsed > 1.0:
                self.get_logger().warn(
                    "visual frame processing took {:.2f}s in {} mode".format(
                        elapsed, self.search_mode
                    )
                )
        except Exception as error:
            self.get_logger().error("visual localization error: {}".format(error))
            self.publish_state("ERROR:{}".format(type(error).__name__))


def main(args=None):
    rclpy.init(args=args)
    node = VisualLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
