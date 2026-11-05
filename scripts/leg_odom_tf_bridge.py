#!/usr/bin/env python3
"""Publish Lite3 leg odometry in an isolated TF tree for NDT prediction."""

import copy
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class LegOdomTfBridge(Node):
    def __init__(self):
        super().__init__("leg_odom_tf_bridge")
        self.broadcaster = TransformBroadcaster(self)
        self.sampled_publisher = self.create_publisher(
            Odometry, "/patrol/leg_odom_sampled", 2
        )
        self.last_message = time.monotonic()
        self.latest_message = None
        self.last_published_stamp = None
        self.last_source_stamp = 0.0
        self._t22 = None
        self._t22_input = None
        try:
            from lite3_t22_probe import create_probe
            self._t22 = create_probe('leg')
        except Exception as error:
            self.get_logger().warning('[T22] recorder disabled: '+str(error))
        source_qos = QoSProfile(depth=1)
        source_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        source_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Odometry, "/leg_odom2", self.callback, source_qos)
        # /leg_odom2 is about 200 Hz while the LiDAR is 10 Hz.  Publishing the
        # entire stream into /tf can saturate DDS and make scan matching miss
        # sensor frames.  Twenty hertz is sufficient for interpolation.
        self.create_timer(0.05, self.publish_latest)
        self.create_timer(1.0, self.check_freshness)
        self.get_logger().info(
            "publishing robot_odom -> leg_base_link and 20 Hz "
            "/patrol/leg_odom_sampled from /leg_odom2"
        )

    def callback(self, message):
        probe = self._t22
        tracing = probe is not None and probe.active()
        t2_enter_ns = time.monotonic_ns() if tracing else None
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            if tracing:
                probe.count('input.rejected.non_finite')
            self.get_logger().error("discarding non-finite /leg_odom2 pose")
            return
        stamp=message.header.stamp.sec + message.header.stamp.nanosec/1e9
        q=pose.orientation
        validation_now = None
        if (not 0 <= message.header.stamp.nanosec < 1000000000
                or stamp <= self.last_source_stamp
                or not -.05 <= (validation_now := time.monotonic())-stamp <= .5
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
            if tracing:
                reason = ('invalid_stamp' if not 0 <= message.header.stamp.nanosec < 1000000000
                          else 'duplicate' if stamp == self.last_source_stamp
                          else 'out_of_order' if stamp < self.last_source_stamp
                          else 'future' if validation_now-stamp < -.05
                          else 'stale' if validation_now-stamp > .5 else 'quaternion')
                probe.count('input.rejected.'+reason)
            return
        t2_valid_ns = time.monotonic_ns() if tracing else None
        self.last_source_stamp=stamp

        self.latest_message = message
        self.last_message = time.monotonic()
        previous = self._t22_input
        self._t22_input = None
        if tracing:
            source_ns = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            cached_ns = time.monotonic_ns()
            if previous is not None:
                published_ns = (None if self.last_published_stamp is None else
                                self.last_published_stamp[0]*1000000000+self.last_published_stamp[1])
                if previous[0] != published_ns:
                    probe.count('input.overwritten_before_tf')
            self._t22_input = (source_ns, t2_enter_ns, t2_valid_ns, cached_ns)
            probe.count('input.accepted')
            probe.emit('leg_input', source_stamp_ns=source_ns, t2_enter_ns=t2_enter_ns,
                       t2_valid_ns=t2_valid_ns, cached_ns=cached_ns)

    def publish_latest(self):
        probe = self._t22
        tracing = probe is not None and probe.active()
        t3a_ns = time.monotonic_ns() if tracing else None
        if tracing:
            probe.tick(t3a_ns)
        message = self.latest_message
        if message is None:
            return
        source_time=message.header.stamp.sec + message.header.stamp.nanosec/1e9
        if not -.05 <= time.monotonic()-source_time <= .5:
            return
        # /leg_odom2 and the LiDAR use the controller/sensor time domain.  NDT
        # queries this TF at the LiDAR stamp, so the TF must retain the source
        # stamp.  The sampled topic is consumed only by the safety initializer;
        # restamp that copy into graph time so its arrival can be supervised
        # independently without breaking NDT interpolation.
        source_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if source_stamp == self.last_published_stamp:
            return
        graph_stamp = self.get_clock().now().to_msg()
        copy_begin_ns = time.monotonic_ns() if tracing else None
        sampled = copy.deepcopy(message)
        copy_end_ns = time.monotonic_ns() if tracing else None
        sampled.header.stamp = graph_stamp
        sampled_begin_ns = time.monotonic_ns() if tracing else None
        try:
            self.sampled_publisher.publish(sampled)
        except Exception as error:
            if tracing:
                probe.emit('leg_tx_error', stage='sampled_publish',
                           source_stamp_ns=source_stamp[0]*1000000000+source_stamp[1],
                           t3a_ns=t3a_ns, call_begin_ns=sampled_begin_ns,
                           call_return_ns=time.monotonic_ns(), error_type=type(error).__name__)
            raise
        t3b_ns = time.monotonic_ns() if tracing else None
        pose = message.pose.pose
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "robot_odom"
        transform.child_frame_id = "leg_base_link"
        transform.transform.translation.x = float(pose.position.x)
        transform.transform.translation.y = float(pose.position.y)
        transform.transform.translation.z = float(pose.position.z)
        transform.transform.rotation = pose.orientation
        t3c_ns = time.monotonic_ns() if tracing else None
        try:
            self.broadcaster.sendTransform(transform)
        except Exception as error:
            if tracing:
                probe.emit('leg_tx_error', stage='sendTransform',
                           source_stamp_ns=source_stamp[0]*1000000000+source_stamp[1],
                           t3a_ns=t3a_ns, t3c_ns=t3c_ns, t3d_ns=time.monotonic_ns(),
                           error_type=type(error).__name__)
            raise
        t3d_ns = time.monotonic_ns() if tracing else None
        self.last_published_stamp = source_stamp
        if tracing:
            source_ns = source_stamp[0]*1000000000+source_stamp[1]
            previous = self._t22_input
            matched = previous is not None and previous[0] == source_ns
            probe.emit('leg_tx', source_stamp_ns=source_ns,
                       t2_enter_ns=previous[1] if matched else None,
                       t2_valid_ns=previous[2] if matched else None,
                       cached_ns=previous[3] if matched else None,
                       t3a_ns=t3a_ns, copy_begin_ns=copy_begin_ns, copy_end_ns=copy_end_ns,
                       sampled_begin_ns=sampled_begin_ns, t3b_ns=t3b_ns,
                       t3c_ns=t3c_ns, t3d_ns=t3d_ns)

    def check_freshness(self):
        age = time.monotonic() - self.last_message
        if age > 0.5:
            self.get_logger().error(
                "/leg_odom2 is stale for %.2f s; NDT prediction is unavailable" % age
            )


def main():
    rclpy.init()
    node = LegOdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
