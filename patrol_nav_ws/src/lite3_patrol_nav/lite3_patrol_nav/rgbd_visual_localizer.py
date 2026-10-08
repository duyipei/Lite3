#!/usr/bin/env python3
"""ORB/PnP global visual localizer for a prebuilt RGB-D keyframe database."""

import json
import math
import sqlite3
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


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
        self.declare_parameter("database", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("pose_topic", "/visual_localization/pose")
        self.declare_parameter("map_frame", "visual_map")
        self.declare_parameter("processing_rate_hz", 2.0)
        self.declare_parameter("ratio_test", 0.75)
        self.declare_parameter("candidate_count", 4)
        self.declare_parameter("pnp_candidate_count", 4)
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
        self.declare_parameter("startup_appearance_fallback", True)
        self.declare_parameter("startup_appearance_max_score", 0.60)
        self.declare_parameter("startup_appearance_top_count", 5)
        self.declare_parameter("startup_appearance_min_support", 3)

        database = self.get_parameter("database").value
        if not database:
            raise RuntimeError("database parameter is empty")
        connection = sqlite3.connect("file:{}?mode=ro".format(database), uri=True)
        metadata = {key: json.loads(value) for key, value in connection.execute("SELECT key,value FROM metadata")}
        self.k = np.asarray(metadata["color_k"], dtype=np.float64)
        self.t_base_camera = np.asarray(metadata["base_to_color_optical"], dtype=np.float64)
        self.keyframes = []
        for row in connection.execute("SELECT id,map_camera,points,descriptors,feature_count,color_jpeg FROM keyframes ORDER BY id"):
            keyframe_id, pose_blob, points_blob, descriptor_blob, count, jpeg_blob = row
            reference = cv2.imdecode(np.frombuffer(jpeg_blob, dtype=np.uint8), cv2.IMREAD_COLOR)
            histogram = cv2.calcHist([cv2.cvtColor(reference, cv2.COLOR_BGR2HSV)], [0, 1], None, [30, 32], [0, 180, 0, 256])
            cv2.normalize(histogram, histogram)
            retained = min(int(count), 1000)
            self.keyframes.append({
                "id": keyframe_id,
                "map_camera": np.frombuffer(pose_blob, dtype="<f8").reshape(4, 4).copy(),
                "points": np.frombuffer(points_blob, dtype="<f4").reshape(count, 3)[:retained].copy(),
                "descriptors": np.frombuffer(descriptor_blob, dtype=np.uint8).reshape(count, 32)[:retained].copy(),
                "histogram": histogram,
                "appearance": appearance_vector(reference),
            })
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

        self.orb = cv2.ORB_create(nfeatures=900, scaleFactor=1.2, nlevels=8, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last_image_time = 0.0
        self.latest_image = None
        self.last_processed_stamp = None
        self.last_state = "STARTING"
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10
        )
        self.state_publisher = self.create_publisher(String, "/visual_localization/state", 10)
        self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self.image_callback, qos_profile_sensor_data,
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

    def publish_state(self, value):
        self.last_state = value
        message = String()
        message.data = value
        self.state_publisher.publish(message)

    def health_callback(self):
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
        # Never run expensive matching in the 30 Hz subscription callback.
        # Keeping only the newest frame prevents old camera messages from
        # starving timers and makes relocalization latency bounded.
        self.last_image_time = time.monotonic()
        self.latest_image = message

    def processing_callback(self):
        message = self.latest_image
        if message is None:
            return
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
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
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            keypoints, query_descriptors = self.orb.detectAndCompute(gray, None)
            if query_descriptors is None or len(query_descriptors) < 30:
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

            appearance_coarse = False
            if (
                self.search_mode == "startup"
                and bool(self.get_parameter("startup_appearance_fallback").value)
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
            if appearance_coarse:
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
                    pairs = self.matcher.knnMatch(keyframe["descriptors"], query_descriptors, k=2)
                    good = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < ratio*pair[1].distance]
                    if len(good) >= int(self.get_parameter("min_matches").value):
                        ranked.append((len(good), keyframe, good))
                ranked.sort(key=lambda item: item[0], reverse=True)
                for match_count, keyframe, matches in ranked[:max(
                    1, int(self.get_parameter("pnp_candidate_count").value)
                )]:
                    object_points = np.asarray([keyframe["points"][match.queryIdx] for match in matches], np.float32)
                    image_points = np.asarray([keypoints[match.trainIdx].pt for match in matches], np.float32)
                    ok, rvec, tvec, inlier_ids = cv2.solvePnPRansac(
                        object_points, image_points, self.k, None,
                        iterationsCount=300, reprojectionError=float(self.get_parameter("max_reprojection_error").value),
                        confidence=0.995, flags=cv2.SOLVEPNP_AP3P,
                    )
                    inlier_count = 0 if inlier_ids is None else len(inlier_ids)
                    if not ok or inlier_count < int(self.get_parameter("min_inliers").value):
                        continue
                    rotation, _ = cv2.Rodrigues(rvec)
                    t_current_key = np.eye(4, dtype=np.float64)
                    t_current_key[:3, :3] = rotation
                    t_current_key[:3, 3] = tvec[:, 0]
                    t_map_current_camera = keyframe["map_camera"] @ np.linalg.inv(t_current_key)
                    t_map_base = t_map_current_camera @ np.linalg.inv(self.t_base_camera)
                    projected, _ = cv2.projectPoints(object_points[inlier_ids[:, 0]], rvec, tvec, self.k, None)
                    errors = np.linalg.norm(projected[:, 0, :]-image_points[inlier_ids[:, 0]], axis=1)
                    solutions.append({
                        "keyframe": keyframe["id"], "matches": match_count, "inliers": inlier_count,
                        "reprojection": float(np.median(errors)), "pose": t_map_base,
                        "source": "pnp",
                    })
            if not solutions:
                self.confirmation_history.clear()
                self.publish_state(
                    "REJECTED:{}:no_pnp_solution".format(self.search_mode)
                )
                return
            solutions.sort(key=lambda item: (item["inliers"], -item["reprojection"]), reverse=True)
            best = solutions[0]
            if best["source"] == "pnp" and len(solutions) > 1 and solutions[1]["inliers"] >= best["inliers"]*float(self.get_parameter("ambiguity_ratio").value):
                first, second = best["pose"], solutions[1]["pose"]
                distance = float(np.linalg.norm(first[:2, 3]-second[:2, 3]))
                yaw_difference = abs(angle_delta(matrix_yaw(first), matrix_yaw(second)))
                if distance > float(self.get_parameter("ambiguity_distance_m").value) or yaw_difference > 0.7:
                    self.confirmation_history.clear()
                    self.publish_state("REJECTED:{}:ambiguous_{:.2f}m_{:.2f}rad".format(self.search_mode, distance, yaw_difference))
                    return

            pose = best["pose"]
            yaw = matrix_yaw(pose)
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
            if self.search_mode == "startup":
                self.search_strategy = "startup_only"
            result = PoseWithCovarianceStamped()
            result.header = message.header
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
                "POSE:mode={},source={},kf={},inliers={},x={:.2f},y={:.2f},yaw={:.2f}".format(
                    self.search_mode, best["source"], best["keyframe"], best["inliers"], pose[0, 3], pose[1, 3], yaw
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
