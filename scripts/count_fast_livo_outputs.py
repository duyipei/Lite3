#!/usr/bin/env python3
"""Count isolated FAST-LIVO2 outputs without storing point-cloud payloads."""

import argparse
import json
import signal
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def stamp_ns(message):
    if not hasattr(message, "header"):
        return None
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class OutputCounter(Node):
    def __init__(self):
        super().__init__("round2a_fast_livo_output_counter")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.started_monotonic = time.monotonic()
        self.data = {
            "started_realtime_ns": time.time_ns(),
            "topics": {
                name: {
                    "count": 0,
                    "first_stamp_ns": None,
                    "last_stamp_ns": None,
                    "non_monotonic": 0,
                    "frames": [],
                }
                for name in (
                    "/fast_livo2/odometry",
                    "/fast_livo2/cloud_registered",
                    "/fast_livo2/path",
                    "/fast_livo2/status",
                )
            },
        }
        self.create_subscription(Odometry, "/fast_livo2/odometry", self._callback("/fast_livo2/odometry"), qos)
        self.create_subscription(PointCloud2, "/fast_livo2/cloud_registered", self._callback("/fast_livo2/cloud_registered"), qos)
        self.create_subscription(Path, "/fast_livo2/path", self._callback("/fast_livo2/path"), qos)
        self.create_subscription(String, "/fast_livo2/status", self._callback("/fast_livo2/status"), qos)

    def _callback(self, topic):
        def receive(message):
            entry = self.data["topics"][topic]
            entry["count"] += 1
            current = stamp_ns(message)
            if current is not None:
                if entry["first_stamp_ns"] is None:
                    entry["first_stamp_ns"] = current
                if entry["last_stamp_ns"] is not None and current < entry["last_stamp_ns"]:
                    entry["non_monotonic"] += 1
                entry["last_stamp_ns"] = current
                frame = message.header.frame_id
                if frame and frame not in entry["frames"]:
                    entry["frames"].append(frame)
        return receive

    def finish(self):
        self.data["finished_realtime_ns"] = time.time_ns()
        self.data["elapsed_monotonic_s"] = time.monotonic() - self.started_monotonic


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--duration", type=float, default=170.0)
    args = parser.parse_args()
    rclpy.init()
    node = OutputCounter()
    deadline = time.monotonic() + args.duration
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    while rclpy.ok() and not stop and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    node.finish()
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(node.data, stream, indent=2, sort_keys=True)
        stream.write("\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

