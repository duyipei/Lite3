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
            self.get_logger().error("discarding non-finite /leg_odom2 pose")
            return
        stamp=message.header.stamp.sec + message.header.stamp.nanosec/1e9
        q=pose.orientation
        if (not 0 <= message.header.stamp.nanosec < 1000000000
                or stamp <= self.last_source_stamp
                or not -.05 <= time.monotonic()-stamp <= .5
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
            return
        self.last_source_stamp=stamp

        self.latest_message = message
        self.last_message = time.monotonic()

    def publish_latest(self):
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
        sampled = copy.deepcopy(message)
        sampled.header.stamp = graph_stamp
        self.sampled_publisher.publish(sampled)
        pose = message.pose.pose
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "robot_odom"
        transform.child_frame_id = "leg_base_link"
        transform.transform.translation.x = float(pose.position.x)
        transform.transform.translation.y = float(pose.position.y)
        transform.transform.translation.z = float(pose.position.z)
        transform.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(transform)
        self.last_published_stamp = source_stamp

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
