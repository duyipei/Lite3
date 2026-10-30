#!/usr/bin/env python3
"""Dead-man velocity gate with an explicitly armed multi-goal session."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Header, String
from std_srvs.srv import SetBool, Trigger
from .voa_protocol import VoaExchange, braking_correction


def clamp(value, limit):
    return max(-limit, min(limit, value))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class VelocityGate(Node):
    def __init__(self):
        super().__init__("patrol_velocity_gate")
        self.declare_parameter("max_linear_x", 0.15)
        self.declare_parameter("max_reverse_x", 0.08)
        self.declare_parameter("max_linear_y", 0.05)
        self.declare_parameter("max_angular_z", 0.40)
        self.declare_parameter("freshness_timeout_s", 1.5)
        # Independent of sensor timeout: tolerate two missed 5 Hz commands,
        # then send zero while the bounded Nav2 recovery window runs.
        self.declare_parameter("command_timeout_s", 0.5)
        # One bounded ZERO-speed waiting policy for initial planning and
        # subsequent recovery. The launcher derives it from the actual BT.
        self.declare_parameter("nav_command_wait_timeout_s", 60.0)
        wait_timeout = float(self.get_parameter("nav_command_wait_timeout_s").value)
        if not math.isfinite(wait_timeout) or wait_timeout <= float(self.get_parameter("command_timeout_s").value):
            raise ValueError('command wait must be finite and longer than command freshness')
        self.declare_parameter("max_pose_jump_m", 0.50)
        self.declare_parameter("max_yaw_jump_rad", 0.35)
        self.declare_parameter("output_topic", "/cmd_vel_corrected")
        self.declare_parameter("require_localization_ready", True)
        self.declare_parameter("use_vendor_voa", False)
        self.use_vendor_voa = bool(self.get_parameter("use_vendor_voa").value)
        self.declare_parameter('require_voa_costmap', False)
        self.require_voa_costmap = bool(self.get_parameter('require_voa_costmap').value)
        if self.require_voa_costmap and not self.use_vendor_voa:
            raise ValueError('VOA costmap monitoring requires VOA')
        self.declare_parameter('require_lidar_costmap', False)
        self.require_lidar_costmap = bool(self.get_parameter('require_lidar_costmap').value)
        self.last_lidar_costmap_time = 0.0
        if self.require_lidar_costmap:
            self.create_subscription(Header, '/patrol/lidar_costmap_heartbeat',
                                     self.lidar_costmap_callback, 1)
        self.last_voa_costmap_time = 0.0
        if self.require_voa_costmap:
            self.create_subscription(Header, '/patrol/voa/costmap_heartbeat',
                                     self.voa_costmap_callback, 1)
        self.voa = VoaExchange()
        self.voa_seen = False
        if self.use_vendor_voa:
            voa_qos = QoSProfile(depth=1)
            voa_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.voa_input = self.create_publisher(TwistStamped, '/patrol/voa/input', voa_qos)
            self.create_subscription(TwistStamped, '/patrol/voa/corrected', self.voa_callback, voa_qos)

        self.armed = False
        self.fault = "startup_disarmed"
        self.last_raw = None
        self.last_raw_time = 0.0
        self.last_odom_time = 0.0
        self.last_odom_source_stamp = 0.0
        self.last_lidar_time = 0.0
        self.localization_ready = False
        self.last_ready_time = 0.0
        self.goal_pending = False
        self.goal_active = False
        self.last_pose = None
        self.armed_time = 0.0
        self.goal_started_time = 0.0
        self.publish_count = 0

        output_topic = self.get_parameter("output_topic").value
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.state_publisher = self.create_publisher(String, "/patrol/gate_state", 10)
        self.armed_publisher = self.create_publisher(Bool, "/patrol/armed", 10)
        self.create_subscription(Twist, "/patrol/cmd_vel_raw", self.raw_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 20)
        self.create_subscription(
            Empty, "/patrol/lidar_heartbeat", self.lidar_callback, 10
        )
        self.create_subscription(Bool, "/patrol/localization_ready", self.localization_ready_callback, 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, "/patrol/goal_pending", self.goal_pending_callback, state_qos
        )
        self.create_subscription(
            Bool, "/patrol/goal_active", self.goal_active_callback, state_qos
        )
        self.create_service(SetBool, "/patrol/set_armed", self.arm_callback)
        self.create_service(Trigger, "/patrol/clear_fault", self.clear_fault_callback)
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().warn("velocity gate started DISARMED; non-zero output is blocked")

    def now_monotonic(self):
        return time.monotonic()

    def raw_callback(self, message):
        if not all(math.isfinite(v) for v in (message.linear.x, message.linear.y, message.angular.z)):
            self.last_raw = None
            self.last_raw_time = 0.0
            self.trip("invalid_velocity_command")
            return
        self.last_raw = message
        self.last_raw_time = self.now_monotonic()

    def lidar_callback(self, _message):
        self.last_lidar_time = self.now_monotonic()

    def voa_callback(self, message):
        v = message.twist
        if any(value != 0.0 for value in (v.linear.z, v.angular.x, v.angular.y)):
            self.voa.reset()
            return
        if self.voa.accept(message.header.frame_id, (v.linear.x, v.linear.y, v.angular.z),
                           self.now_monotonic()) and not self.voa_seen:
            self.voa_seen = True
            self.get_logger().info('[VOA] correlated sensor-valid response received; output remains gated')

    def localization_ready_callback(self, message):
        self.last_ready_time = self.now_monotonic()
        was_ready = self.localization_ready
        self.localization_ready = bool(message.data)
        if was_ready and not self.localization_ready and self.armed:
            self.trip("localization_ready_revoked")

    def goal_pending_callback(self, message):
        self.goal_pending = bool(message.data)

    def goal_active_callback(self, message):
        was_active = self.goal_active
        self.goal_active = bool(message.data)
        if self.goal_active != was_active:
            self.voa.reset()
        if self.goal_active and not was_active:
            self.goal_started_time = self.now_monotonic()
            # A command retained from the previous goal must never move the
            # robot while Nav2 is planning the new one.
            self.last_raw = None
            self.last_raw_time = 0.0
        if was_active and not self.goal_active and self.armed:
            # Keep the operator-authorized navigation session armed, but force
            # zero output between goals.  The next RViz goal can then execute
            # immediately without another ARM handshake.
            self.last_raw = None
            self.last_raw_time = 0.0
            self.get_logger().warn(
                "navigation goal finished; ARMED session is idle and the next "
                "RViz goal may start immediately"
            )

    def odom_callback(self, message):
        now = self.now_monotonic()
        pose = message.pose.pose
        q = pose.orientation
        values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if (not all(math.isfinite(v) for v in values)
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .01):
            self.trip('invalid_odometry')
            return
        # NDT output retains this Jetson's monotonic LiDAR source clock.
        if (message.header.frame_id != 'map' or message.child_frame_id != 'base_link'
                or not 0 <= message.header.stamp.nanosec < 1000000000
                or stamp <= self.last_odom_source_stamp
                or not -.05 <= now-stamp <= self.get_parameter('freshness_timeout_s').value):
            return
        self.last_odom_source_stamp = stamp
        current = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        if self.last_pose is not None and self.armed:
            distance = math.hypot(current[0] - self.last_pose[0], current[1] - self.last_pose[1])
            yaw_jump = abs(angle_delta(current[2], self.last_pose[2]))
            if distance > self.get_parameter("max_pose_jump_m").value:
                self.trip("localization_position_jump_{:.3f}m".format(distance))
            elif yaw_jump > self.get_parameter("max_yaw_jump_rad").value:
                self.trip("localization_yaw_jump_{:.3f}rad".format(yaw_jump))
        self.last_pose = current
        self.last_odom_time = now

    def trip(self, reason):
        if self.armed or self.fault != reason:
            self.get_logger().error("gate fault: " + reason)
        self.armed = False
        self.fault = reason

    def clean_disarm(self, reason):
        # Never replace a previously latched sensor/localization fault with a
        # normal goal-finished reason if callbacks arrive in the opposite order.
        if not self.armed:
            return
        self.get_logger().warn("velocity output cleanly DISARMED: " + reason)
        self.armed = False
        self.fault = reason

    def voa_costmap_callback(self, message):
        stamp = message.stamp.sec + message.stamp.nanosec / 1e9
        age = self.now_monotonic() - stamp
        if (message.frame_id != 'map' or message.stamp.sec < 0 or
                not 0 <= message.stamp.nanosec < 1000000000 or
                not -.05 <= age <= .5 or stamp <= self.last_voa_costmap_time):
            return
        if self.last_voa_costmap_time == 0:
            self.get_logger().info('[VOA_COSTMAP] fresh obstacle projection received')
        self.last_voa_costmap_time = stamp

    def lidar_costmap_callback(self, message):
        stamp = message.stamp.sec + message.stamp.nanosec / 1e9
        age = self.now_monotonic() - stamp
        timeout = self.get_parameter('freshness_timeout_s').value
        if (message.frame_id != 'map' or message.stamp.sec < 0 or
                not 0 <= message.stamp.nanosec < 1000000000 or
                not -.05 <= age <= timeout or stamp <= self.last_lidar_costmap_time):
            return
        self.last_lidar_costmap_time = stamp

    def sensor_health(self):
        now = self.now_monotonic()
        # Zero publishers is normal before Nav2 starts; more than one is never
        # valid, including while waiting for the first navigation command.
        if self.count_publishers('/patrol/cmd_vel_raw') > 1:
            return False, 'nav_command_publisher_conflict'
        if self.require_lidar_costmap:
            timeout = self.get_parameter('freshness_timeout_s').value
            if (self.last_lidar_costmap_time <= 0 or
                    not -.05 <= now - self.last_lidar_costmap_time <= timeout):
                return False, 'lidar_costmap_not_ready_or_stale'
            if (self.count_publishers('/patrol/lidar_costmap_heartbeat') != 1 or
                    self.count_publishers('/patrol/costmap_obstacles') != 1):
                return False, 'lidar_costmap_publisher_conflict'
        if self.require_voa_costmap:
            if (self.last_voa_costmap_time <= 0 or
                    not -.05 <= now - self.last_voa_costmap_time <= .5):
                return False, 'voa_costmap_not_ready_or_stale'
            if (self.count_publishers('/patrol/voa/costmap_heartbeat') != 1 or
                    self.count_publishers('/patrol/voa/costmap_obstacles') != 1):
                return False, 'voa_costmap_publisher_conflict'
        if self.use_vendor_voa:
            if not self.voa.healthy(now):
                return False, 'vendor_voa_not_ready_or_stale'
            if (self.count_publishers('/cmd_vel') != 0 or
                    self.count_publishers(self.get_parameter('output_topic').value) != 1 or
                    self.count_publishers('/patrol/voa/corrected') != 1 or
                    self.count_publishers('/patrol/voa/input') != 1):
                return False, 'vendor_voa_command_publisher_conflict'
        timeout = self.get_parameter("freshness_timeout_s").value
        if self.get_parameter("require_localization_ready").value:
            if not self.localization_ready:
                return False, "localization_not_ready"
            if now - self.last_ready_time > timeout:
                return False, "localization_guard_stale"
        if now - self.last_odom_time > timeout:
            return False, "odometry_stale"
        if now - self.last_lidar_time > timeout:
            return False, "lidar_stale"
        return True, "healthy"

    def raw_command_fresh(self):
        now = self.now_monotonic()
        timeout = self.get_parameter("command_timeout_s").value
        if now - self.last_raw_time > timeout:
            return False, "nav_command_stale"
        return True, "healthy"

    def arm_callback(self, request, response):
        if not request.data:
            self.armed = False
            self.voa.reset()
            self.fault = "operator_disarmed"
            response.success = True
            response.message = self.fault
            return response
        healthy, reason = self.sensor_health()
        if self.fault not in (
            "", "startup_disarmed", "operator_disarmed", "goal_finished"
        ):
            response.success = False
            response.message = "clear latched fault first: " + self.fault
        elif not (self.goal_pending or self.goal_active):
            response.success = False
            response.message = "cannot arm: no_pending_or_active_goal"
        elif not healthy:
            response.success = False
            response.message = "cannot arm: " + reason
        elif self.goal_active and not self.raw_command_fresh()[0]:
            response.success = False
            response.message = "cannot arm: active_goal_has_no_fresh_command"
        else:
            self.armed = True
            self.voa.reset()
            self.armed_time = self.now_monotonic()
            self.fault = ""
            response.success = True
            response.message = "ARMED"
            self.get_logger().warn("velocity output ARMED by explicit service call")
        return response

    def clear_fault_callback(self, _request, response):
        self.armed = False
        self.fault = "operator_disarmed"
        self.last_pose = None
        response.success = True
        response.message = "fault cleared; gate remains DISARMED"
        return response

    def timer_callback(self):
        healthy, reason = self.sensor_health()
        raw_fresh, raw_reason = self.raw_command_fresh()
        command = Twist()
        waiting_for_nav_command = False
        idle_session = self.armed and not self.goal_active
        if self.armed and not healthy:
            self.trip(reason)
        elif idle_session:
            # Explicitly publish zero between goals and do not treat the
            # absence of a Nav2 command as stale.  Sensor/localization faults
            # above still disarm and latch as usual.
            pass
        elif self.armed and raw_fresh and self.last_raw is not None:
            command.linear.x = max(
                -self.get_parameter("max_reverse_x").value,
                min(
                    self.get_parameter("max_linear_x").value,
                    self.last_raw.linear.x,
                ),
            )
            command.linear.y = clamp(self.last_raw.linear.y, self.get_parameter("max_linear_y").value)
            command.angular.z = clamp(self.last_raw.angular.z, self.get_parameter("max_angular_z").value)
        elif self.armed and not raw_fresh:
            now = self.now_monotonic()
            waiting_age = now - max(self.goal_started_time, self.last_raw_time)
            if self.goal_active and waiting_age <= self.get_parameter("nav_command_wait_timeout_s").value:
                # Never extend the lifetime of an old velocity command. Keep
                # publishing ZERO while Nav2 waits/replans within this budget.
                waiting_for_nav_command = True
            else:
                self.trip("nav_command_timeout")
        voa_braking = False
        if self.use_vendor_voa:
            original = (command.linear.x, command.linear.y, command.angular.z)
            request = TwistStamped()
            request.header.stamp = self.get_clock().now().to_msg()
            # frame_id is an opaque request token, not a TF frame, on this
            # private protocol. A previous session/request cannot be replayed.
            request.header.frame_id = self.voa.issue(original, self.now_monotonic())
            request.twist = command
            self.voa_input.publish(request)
            corrected = braking_correction(
                original, self.voa.corrected(original, self.now_monotonic()))
            voa_braking = original != (0., 0., 0.) and corrected != original
            command = Twist()
            if self.armed and self.goal_active and healthy and raw_fresh:
                command.linear.x, command.linear.y, command.angular.z = corrected
        self.publisher.publish(command)
        self.publish_count += 1
        if self.publish_count == 1:
            self.get_logger().info("zero-command heartbeat active on " + self.get_parameter("output_topic").value)
        armed_message = Bool()
        armed_message.data = self.armed
        self.armed_publisher.publish(armed_message)
        state = String()
        if self.armed and not self.goal_active:
            state.data = "ARMED:IDLE_READY_FOR_GOAL"
        elif self.armed and waiting_for_nav_command:
            state.data = "ARMED:WAITING_NAV_COMMAND"
        elif self.armed and voa_braking:
            state.data = "ARMED:VOA_BRAKING_OR_WAITING"
        elif self.armed:
            state.data = "ARMED"
        else:
            state.data = "DISARMED:" + (self.fault or raw_reason)
        self.state_publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
