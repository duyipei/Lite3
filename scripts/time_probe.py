#!/usr/bin/env python3
"""Publish paired Linux realtime/monotonic timestamps for bag diagnostics."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TimeProbe(Node):
    def __init__(self):
        super().__init__("round2a_time_probe", namespace="fast_livo2")
        self.publisher = self.create_publisher(String, "time_probe", 20)
        self.sequence = 0
        self.timer = self.create_timer(0.1, self.publish_sample)

    def publish_sample(self):
        message = String()
        message.data = json.dumps({
            "seq": self.sequence,
            "clock_realtime_ns": time.time_ns(),
            "clock_monotonic_ns": time.monotonic_ns(),
        }, separators=(",", ":"))
        self.publisher.publish(message)
        self.sequence += 1


def main():
    rclpy.init()
    node = TimeProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

