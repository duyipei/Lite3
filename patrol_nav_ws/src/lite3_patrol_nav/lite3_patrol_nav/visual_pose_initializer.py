#!/usr/bin/env python3
"""Guarded visual-map pose injection followed by NDT odometry confirmation."""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hdl_localization.msg import ScanMatchingStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def mean_angle(values):
    return math.atan2(sum(math.sin(v) for v in values), sum(math.cos(v) for v in values))


class VisualPoseInitializer(Node):
    def __init__(self):
        super().__init__("visual_pose_initializer")
        self.declare_parameter("configured", False)
        self.declare_parameter("visual_pose_topic", "/visual_localization/pose")
        self.declare_parameter("visual_map_frame", "visual_map")
        self.declare_parameter("lidar_map_frame", "map")
        self.declare_parameter("map_transform_x", 0.0)
        self.declare_parameter("map_transform_y", 0.0)
        self.declare_parameter("map_transform_z", 0.0)
        self.declare_parameter("map_transform_yaw", 0.0)
        self.declare_parameter("stable_samples", 10)
        self.declare_parameter("stable_duration_s", 0.8)
        self.declare_parameter("max_sample_gap_s", 0.75)
        self.declare_parameter("max_position_std_m", 0.12)
        self.declare_parameter("max_yaw_std_rad", 0.12)
        self.declare_parameter("max_visual_covariance_xy", 0.25)
        self.declare_parameter("ndt_confirmation_samples", 5)
        self.declare_parameter("ndt_settle_s", 1.0)
        self.declare_parameter("ndt_timeout_s", 12.0)
        self.declare_parameter("max_ndt_position_delta_m", 0.50)
        self.declare_parameter("max_ndt_yaw_delta_rad", 0.35)
        self.declare_parameter("auto_initialize", False)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("ready_topic", "/patrol/localization_ready")
        self.declare_parameter("map_min_x", -1.0e6)
        self.declare_parameter("map_max_x", 1.0e6)
        self.declare_parameter("map_min_y", -1.0e6)
        self.declare_parameter("map_max_y", 1.0e6)
        self.declare_parameter("max_ready_odom_step_m", 0.50)
        self.declare_parameter("max_ready_odom_speed_mps", 1.0)
        self.declare_parameter("max_ready_yaw_step_rad", 0.35)
        self.declare_parameter("ready_odom_max_age_s", 0.70)
        self.declare_parameter("ready_visual_max_age_s", 2.0)
        self.declare_parameter("max_ready_visual_position_delta_m", 0.75)
        self.declare_parameter("max_ready_visual_yaw_delta_rad", 0.50)
        self.declare_parameter("ready_visual_bad_samples", 3)
        self.declare_parameter("supervise_visual_after_ready", True)
        self.declare_parameter("ndt_max_matching_error", 0.15)
        self.declare_parameter("ndt_bad_samples", 5)
        self.declare_parameter("sensor_max_age_s", 1.5)

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_publisher = self.create_publisher(
            Bool, self.get_parameter("ready_topic").value, ready_qos
        )
        self.state_publisher = self.create_publisher(String, "/patrol/visual_init_state", 10)
        self.lidar_heartbeat_publisher = self.create_publisher(
            Empty, "/patrol/lidar_heartbeat", 10
        )
        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("initialpose_topic").value, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("visual_pose_topic").value,
            self.visual_pose_callback,
            10,
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 20
        )
        self.create_subscription(ScanMatchingStatus, "/status", self.status_callback, 20)
        self.create_subscription(Odometry, "/patrol/leg_odom_sampled", self.leg_callback, 5)
        self.create_subscription(Bool, "/patrol/armed", self.armed_callback, 10)
        self.create_service(Trigger, "/patrol/request_visual_initialization", self.request_callback)
        self.create_service(Trigger, "/patrol/reset_visual_initialization", self.reset_callback)
        self.create_timer(0.2, self.timer_callback)

        self.samples = deque(maxlen=max(2, int(self.get_parameter("stable_samples").value)))
        self.requested = bool(self.get_parameter("auto_initialize").value)
        self.state = "COLLECTING_VISUAL_POSES" if self.requested else "IDLE"
        self.candidate = None
        self.candidate_time = 0.0
        self.last_visual_time = 0.0
        self.ndt_good_count = 0
        self.last_odom_time = 0.0
        self.last_odom_stamp_s = None
        self.last_odom_pose = None
        self.ready_visual_bad_count = 0
        self.armed = False
        self.last_leg_time = 0.0
        self.last_leg_stamp = 0
        self.last_status_time = 0.0
        self.last_status_stamp = 0
        self.last_visual_stamp = 0
        self.ndt_bad_count = 0
        self.scan_results = {}
        self.last_counted_stamp = 0
        self.initialpose_stamp = 0
        self.latest_odom = None
        self.sample_covariance_xy = 0.0
        self.sample_covariance_yaw = 0.0
        self.set_ready(False)
        if not self.get_parameter("configured").value:
            self.state = "BLOCKED:map_transform_not_configured"
            self.get_logger().error(
                "visual initialization blocked: visual_map -> lidar map transform is not configured"
            )
        else:
            self.get_logger().warn("visual initializer ready; velocity remains independently gated")

    def now_monotonic(self):
        return time.monotonic()

    def set_ready(self, value):
        message = Bool()
        message.data = bool(value)
        self.ready_publisher.publish(message)

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(state)
        self.state = state

    def reset(self, state="IDLE"):
        self.samples.clear()
        self.requested = False
        self.candidate = None
        self.candidate_time = 0.0
        self.ndt_good_count = 0
        self.last_visual_time = 0.0
        self.last_odom_time = 0.0
        self.last_odom_stamp_s = None
        self.last_odom_pose = None
        self.ready_visual_bad_count = 0
        self.scan_results.clear()
        self.latest_odom = None
        self.last_counted_stamp = 0
        self.initialpose_stamp = 0
        self.last_visual_stamp = 0
        self.sample_covariance_xy = 0.0
        self.sample_covariance_yaw = 0.0
        self.ndt_bad_count = 0
        self.set_ready(False)
        self.set_state(state)

    def revoke_ready(self, reason):
        self.samples.clear()
        self.requested = False
        self.candidate = None
        self.candidate_time = 0.0
        self.ndt_good_count = 0
        self.last_odom_time = 0.0
        self.last_odom_stamp_s = None
        self.last_odom_pose = None
        self.ready_visual_bad_count = 0
        self.set_ready(False)
        self.set_state("FAILED:" + reason)
        self.get_logger().error("localization readiness revoked: " + reason)

    def request_callback(self, _request, response):
        if self.armed:
            response.success = False
            response.message = "DISARM before requesting visual initialization"
            return response
        if not self.get_parameter("configured").value:
            response.success = False
            response.message = "visual_map -> lidar map transform is not configured"
            return response
        self.reset("COLLECTING_VISUAL_POSES")
        self.requested = True
        response.success = True
        response.message = "collecting stable visual poses; no motion has been enabled"
        return response

    def reset_callback(self, _request, response):
        if self.armed:
            response.success = False
            response.message = "DISARM before resetting visual initialization"
            return response
        self.reset("IDLE")
        response.success = True
        response.message = "visual initialization reset; localization_ready=false"
        return response

    def covariance_acceptable(self, message):
        limit = float(self.get_parameter("max_visual_covariance_xy").value)
        cov = message.pose.covariance
        return (
            all(math.isfinite(cov[i]) and cov[i] >= 0.0 for i in (0, 7, 35))
            and max(cov[0], cov[7]) <= limit
        )

    @staticmethod
    def stamp_ns(message):
        return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

    def fresh_stamp(self, message):
        stamp = self.stamp_ns(message)
        age = (self.get_clock().now().nanoseconds - stamp) / 1.0e9
        return (
            stamp > 0
            and -0.5 <= age <= float(self.get_parameter("sensor_max_age_s").value)
        )

    @staticmethod
    def valid_pose(message):
        p = message.pose.pose
        q = p.orientation
        values = (p.position.x, p.position.y, p.position.z, q.x, q.y, q.z, q.w)
        return (
            all(math.isfinite(v) for v in values)
            and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 0.1
        )

    def armed_callback(self, message):
        self.armed = bool(message.data)

    def leg_callback(self, message):
        stamp = self.stamp_ns(message)
        if (self.fresh_stamp(message) and self.valid_pose(message)
                and stamp > self.last_leg_stamp):
            self.last_leg_stamp = stamp
            self.last_leg_time = self.now_monotonic()

    def status_callback(self, message):
        stamp = self.stamp_ns(message)
        # NDT status and odometry retain the LiDAR sensor clock so they can be
        # paired exactly by scan.  Their freshness must therefore be measured
        # from callback arrival time, not compared with the host epoch.
        if stamp <= 0 or stamp <= self.last_status_stamp:
            return
        now = self.now_monotonic()
        if now - self.last_status_time > float(self.get_parameter("sensor_max_age_s").value):
            self.ndt_good_count = 0
            if self.state.startswith("READY"):
                self.revoke_ready("ndt_status_stale")
        self.last_status_stamp = stamp
        self.last_status_time = now
        # A heartbeat represents an actual new processed scan, never a timer.
        self.lidar_heartbeat_publisher.publish(Empty())
        good = (
            bool(message.has_converged)
            and math.isfinite(message.matching_error)
            and 0.0 <= message.matching_error <= float(self.get_parameter("ndt_max_matching_error").value)
            and math.isfinite(message.inlier_fraction)
            and 0.0 < message.inlier_fraction <= 1.0
        )
        self.scan_results[stamp] = good
        if len(self.scan_results) > 40:
            del self.scan_results[min(self.scan_results)]
        self.ndt_bad_count = 0 if good else self.ndt_bad_count + 1
        if not good:
            self.ndt_good_count = 0
        if (self.state.startswith("READY")
                and self.ndt_bad_count >= int(self.get_parameter("ndt_bad_samples").value)):
            self.revoke_ready("ndt_matching_failed")
        else:
            self.confirm_ndt()

    def visual_pose_callback(self, message):
        ready = self.state.startswith("READY")
        if not self.requested and not ready:
            return
        if ready and not bool(self.get_parameter("supervise_visual_after_ready").value):
            return
        if not self.valid_pose(message) or not self.fresh_stamp(message):
            return
        stamp = self.stamp_ns(message)
        if stamp <= self.last_visual_stamp:
            return
        self.last_visual_stamp = stamp
        expected_frame = self.get_parameter("visual_map_frame").value
        if message.header.frame_id != expected_frame:
            if ready:
                self.revoke_ready("visual_frame_mismatch")
                return
            self.set_state(
                "REJECTED:visual_frame_{}_expected_{}".format(
                    message.header.frame_id or "empty", expected_frame
                )
            )
            return
        if not self.covariance_acceptable(message):
            if not ready:
                self.set_state("COLLECTING:visual_covariance_too_large")
            return
        now = self.now_monotonic()
        previous_visual_time = self.last_visual_time
        pose = message.pose.pose
        visual_yaw = yaw_from_quaternion(pose.orientation)
        tx = float(self.get_parameter("map_transform_x").value)
        ty = float(self.get_parameter("map_transform_y").value)
        transform_yaw = float(self.get_parameter("map_transform_yaw").value)
        c, s = math.cos(transform_yaw), math.sin(transform_yaw)
        nav_x = tx + c * pose.position.x - s * pose.position.y
        nav_y = ty + s * pose.position.x + c * pose.position.y
        nav_yaw = visual_yaw + transform_yaw
        self.last_visual_time = now

        if ready:
            if self.last_odom_pose is None:
                return
            position_delta = math.hypot(
                nav_x - self.last_odom_pose[0], nav_y - self.last_odom_pose[1]
            )
            yaw_delta = abs(angle_delta(nav_yaw, self.last_odom_pose[2]))
            good = (
                position_delta
                <= float(self.get_parameter("max_ready_visual_position_delta_m").value)
                and yaw_delta
                <= float(self.get_parameter("max_ready_visual_yaw_delta_rad").value)
            )
            self.ready_visual_bad_count = 0 if good else self.ready_visual_bad_count + 1
            if self.ready_visual_bad_count >= int(
                self.get_parameter("ready_visual_bad_samples").value
            ):
                self.revoke_ready(
                    "visual_ndt_disagreement_{:.3f}m_{:.3f}rad".format(
                        position_delta, yaw_delta
                    )
                )
            return

        if self.candidate is not None:
            return
        max_gap = float(self.get_parameter("max_sample_gap_s").value)
        if previous_visual_time and now - previous_visual_time > max_gap:
            self.samples.clear()
            self.sample_covariance_xy = 0.0
            self.sample_covariance_yaw = 0.0
        # Repeated fixed start priors do not become precise measurements.
        cov = message.pose.covariance
        self.sample_covariance_xy = max(self.sample_covariance_xy, cov[0], cov[7])
        self.sample_covariance_yaw = max(self.sample_covariance_yaw, cov[35])
        self.samples.append(
            (now, pose.position.x, pose.position.y, pose.position.z, yaw_from_quaternion(pose.orientation))
        )
        required = int(self.get_parameter("stable_samples").value)
        if len(self.samples) < required:
            self.set_state("COLLECTING:{}/{}".format(len(self.samples), required))
            return
        if self.samples[-1][0] - self.samples[0][0] < float(
            self.get_parameter("stable_duration_s").value
        ):
            self.set_state("COLLECTING:stable_duration_too_short")
            return

        xs = [v[1] for v in self.samples]
        ys = [v[2] for v in self.samples]
        zs = [v[3] for v in self.samples]
        yaws = [v[4] for v in self.samples]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        mean_z = sum(zs) / len(zs)
        mean_yaw = mean_angle(yaws)
        position_std = math.sqrt(
            sum((x - mean_x) ** 2 + (y - mean_y) ** 2 for x, y in zip(xs, ys)) / len(xs)
        )
        yaw_std = math.sqrt(sum(angle_delta(y, mean_yaw) ** 2 for y in yaws) / len(yaws))
        if position_std > float(self.get_parameter("max_position_std_m").value):
            self.set_state("COLLECTING:position_std_{:.3f}m".format(position_std))
            return
        if yaw_std > float(self.get_parameter("max_yaw_std_rad").value):
            self.set_state("COLLECTING:yaw_std_{:.3f}rad".format(yaw_std))
            return

        tx = float(self.get_parameter("map_transform_x").value)
        ty = float(self.get_parameter("map_transform_y").value)
        tz = float(self.get_parameter("map_transform_z").value)
        transform_yaw = float(self.get_parameter("map_transform_yaw").value)
        c, s = math.cos(transform_yaw), math.sin(transform_yaw)
        nav_x = tx + c * mean_x - s * mean_y
        nav_y = ty + s * mean_x + c * mean_y
        nav_z = tz + mean_z
        nav_yaw = mean_yaw + transform_yaw
        self.candidate = (nav_x, nav_y, nav_z, nav_yaw)
        self.candidate_time = now
        self.ndt_good_count = 0

        initial = PoseWithCovarianceStamped()
        initial.header.stamp = self.get_clock().now().to_msg()
        initial.header.frame_id = self.get_parameter("lidar_map_frame").value
        initial.pose.pose.position.x = nav_x
        initial.pose.pose.position.y = nav_y
        initial.pose.pose.position.z = nav_z
        initial.pose.pose.orientation.z = math.sin(nav_yaw / 2.0)
        initial.pose.pose.orientation.w = math.cos(nav_yaw / 2.0)
        initial.pose.covariance[0] = max(position_std ** 2, self.sample_covariance_xy, 0.01)
        initial.pose.covariance[7] = initial.pose.covariance[0]
        initial.pose.covariance[14] = 0.04
        initial.pose.covariance[35] = max(yaw_std ** 2, self.sample_covariance_yaw, 0.01)
        # Establish the post-seed boundary in the LiDAR clock domain.  Using
        # the host-stamped /initialpose here would make every sensor-stamped
        # NDT result appear older than the seed.
        latest_odom_stamp = (
            self.stamp_ns(self.latest_odom) if self.latest_odom is not None else 0
        )
        self.initialpose_stamp = max(self.last_status_stamp, latest_odom_stamp)
        self.scan_results.clear()
        self.latest_odom = None
        self.initialpose_publisher.publish(initial)
        self.set_state(
            "WAITING_NDT:x={:.3f},y={:.3f},yaw={:.3f}".format(nav_x, nav_y, nav_yaw)
        )

    def odom_callback(self, message):
        now = self.now_monotonic()
        stamp_ns = self.stamp_ns(message)
        if stamp_ns <= 0 or not self.valid_pose(message):
            if self.state.startswith("READY"):
                self.revoke_ready("invalid_odometry")
            return
        if (message.header.frame_id != self.get_parameter("lidar_map_frame").value
                or message.child_frame_id != "base_link"):
            if self.state.startswith("READY"):
                self.revoke_ready("odometry_frame_mismatch")
            return
        stamp = message.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if stamp_s <= 0.0:
            stamp_s = None
        pose = message.pose.pose
        odom_yaw = yaw_from_quaternion(pose.orientation)
        current = (pose.position.x, pose.position.y, odom_yaw)
        if self.last_odom_stamp_s is not None and stamp_s <= self.last_odom_stamp_s:
            return
        self.latest_odom = message

        if self.state.startswith("READY"):
            if not (
                float(self.get_parameter("map_min_x").value)
                <= current[0]
                <= float(self.get_parameter("map_max_x").value)
                and float(self.get_parameter("map_min_y").value)
                <= current[1]
                <= float(self.get_parameter("map_max_y").value)
            ):
                self.revoke_ready(
                    "odom_out_of_map_{:.3f}_{:.3f}".format(current[0], current[1])
                )
                return
            if self.last_odom_pose is not None and self.last_odom_time > 0.0:
                distance = math.hypot(
                    current[0] - self.last_odom_pose[0],
                    current[1] - self.last_odom_pose[1],
                )
                yaw_jump = abs(angle_delta(current[2], self.last_odom_pose[2]))
                if distance > float(self.get_parameter("max_ready_odom_step_m").value):
                    self.revoke_ready("odom_step_{:.3f}m".format(distance))
                    return
                # Executor scheduling can deliver queued odometry callbacks in
                # a burst.  Use the sensor timestamp for velocity, never the
                # much shorter callback-arrival interval.  The independent
                # per-message step guard still covers duplicate/non-monotonic
                # stamps.
                if (
                    stamp_s is not None
                    and self.last_odom_stamp_s is not None
                    and stamp_s > self.last_odom_stamp_s
                ):
                    sensor_dt = stamp_s - self.last_odom_stamp_s
                    speed = distance / sensor_dt
                    if speed > float(
                        self.get_parameter("max_ready_odom_speed_mps").value
                    ):
                        self.revoke_ready("odom_speed_{:.3f}mps".format(speed))
                        return
                if yaw_jump > float(
                    self.get_parameter("max_ready_yaw_step_rad").value
                ):
                    self.revoke_ready("odom_yaw_step_{:.3f}rad".format(yaw_jump))
                    return
            self.last_odom_pose = current
            self.last_odom_time = now
            self.last_odom_stamp_s = stamp_s
            return

        self.last_odom_pose = current
        self.last_odom_time = now
        self.last_odom_stamp_s = stamp_s
        self.confirm_ndt()

    def confirm_ndt(self):
        if (self.candidate is None or self.state.startswith(("READY", "FAILED"))
                or self.latest_odom is None):
            return
        now = self.now_monotonic()
        if now - self.candidate_time < float(self.get_parameter("ndt_settle_s").value):
            return
        message = self.latest_odom
        stamp = self.stamp_ns(message)
        if (stamp <= max(self.initialpose_stamp, self.last_counted_stamp)
                or stamp not in self.scan_results):
            return
        self.last_counted_stamp = stamp
        fresh = (
            now - self.last_odom_time <= float(self.get_parameter("sensor_max_age_s").value)
            and now - self.last_status_time <= float(self.get_parameter("sensor_max_age_s").value)
            and now - self.last_leg_time <= float(self.get_parameter("sensor_max_age_s").value)
        )
        if not fresh or not self.scan_results[stamp] or self.ndt_bad_count:
            self.ndt_good_count = 0
            self.set_state("WAITING_NDT:matching_or_leg_odometry_not_ready")
            return
        pose = message.pose.pose
        odom_yaw = yaw_from_quaternion(pose.orientation)
        current = (pose.position.x, pose.position.y, odom_yaw)
        if not (
            float(self.get_parameter("map_min_x").value)
            <= current[0] <= float(self.get_parameter("map_max_x").value)
            and float(self.get_parameter("map_min_y").value)
            <= current[1] <= float(self.get_parameter("map_max_y").value)
        ):
            self.ndt_good_count = 0
            return
        position_delta = math.hypot(pose.position.x - self.candidate[0], pose.position.y - self.candidate[1])
        yaw_delta = abs(angle_delta(odom_yaw, self.candidate[3]))
        if position_delta <= float(self.get_parameter("max_ndt_position_delta_m").value) and yaw_delta <= float(
            self.get_parameter("max_ndt_yaw_delta_rad").value
        ):
            self.ndt_good_count += 1
            self.set_state("WAITING_NDT:accepted_scans={}/{}".format(
                self.ndt_good_count, self.get_parameter("ndt_confirmation_samples").value
            ))
        else:
            self.ndt_good_count = 0
            self.set_state(
                "WAITING_NDT:delta={:.3f}m,{:.3f}rad".format(position_delta, yaw_delta)
            )
        if self.ndt_good_count >= int(self.get_parameter("ndt_confirmation_samples").value):
            self.requested = False
            self.last_odom_pose = current
            self.last_odom_time = now
            self.last_odom_stamp_s = stamp / 1.0e9
            self.ready_visual_bad_count = 0
            self.set_ready(True)
            self.set_state(
                "READY:visual_plus_ndt_confirmed_{:.3f}m_{:.3f}rad".format(
                    position_delta, yaw_delta
                )
            )

    def timer_callback(self):
        if self.state.startswith("READY"):
            now = self.now_monotonic()
            if now - self.last_status_time > float(self.get_parameter("sensor_max_age_s").value):
                self.revoke_ready("ndt_status_stale")
            elif now - self.last_leg_time > float(self.get_parameter("sensor_max_age_s").value):
                self.revoke_ready("leg_odometry_stale")
            elif now - self.last_odom_time > float(
                self.get_parameter("ready_odom_max_age_s").value
            ):
                self.revoke_ready("odometry_stale")
            elif bool(self.get_parameter("supervise_visual_after_ready").value) and now - self.last_visual_time > float(
                self.get_parameter("ready_visual_max_age_s").value
            ):
                self.revoke_ready("visual_pose_stale")
        if self.candidate is not None and not self.state.startswith("READY"):
            if self.now_monotonic() - self.candidate_time > float(
                self.get_parameter("ndt_timeout_s").value
            ):
                self.reset("FAILED:ndt_confirmation_timeout")
        ready = self.state.startswith("READY")
        self.set_ready(ready)
        message = String()
        message.data = self.state
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VisualPoseInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_ready(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
