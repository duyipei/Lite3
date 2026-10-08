#!/usr/bin/env python3
"""Relay the map-aligned navigation cloud to a local RViz-friendly topic."""

import argparse
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2


class RvizPointcloudRelay(Node):
    def __init__(self, input_topic, output_topic, parent_pid):
        super().__init__("patrol_rviz_pointcloud_relay")
        output_qos = QoSProfile(depth=1)
        output_qos.history = HistoryPolicy.KEEP_LAST
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.VOLATILE

        self.publisher = self.create_publisher(
            PointCloud2, output_topic, output_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.parent_pid = parent_pid
        self.message_count = 0
        if parent_pid:
            self.create_timer(1.0, self.check_parent)
        self.get_logger().info(
            "relaying {} -> {} with best-effort remote input".format(
                input_topic, output_topic
            )
        )

    def on_cloud(self, message):
        self.publisher.publish(message)
        self.message_count += 1
        if self.message_count == 1:
            self.get_logger().info(
                "first aligned point cloud relayed: frame={} points={}".format(
                    message.header.frame_id, message.width * message.height
                )
            )

    def check_parent(self):
        try:
            os.kill(self.parent_pid, 0)
        except ProcessLookupError:
            self.get_logger().info("navigation session ended; stopping relay")
            rclpy.shutdown()
        except PermissionError:
            pass


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/aligned_points")
    parser.add_argument(
        "--output-topic", default="/patrol/rviz_aligned_points"
    )
    parser.add_argument("--parent-pid", type=int, default=0)
    parsed, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = RvizPointcloudRelay(
        parsed.input_topic, parsed.output_topic, parsed.parent_pid
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
