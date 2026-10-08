#!/usr/bin/env python3
"""Collect isolated LIO odometry and summarize static-pose stability."""

import argparse
import csv
import json
import math
import signal
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class OdomCollector(Node):
    def __init__(self):
        super().__init__("lite3_offline_lio_odometry_analyzer")
        self.rows = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry, "/fast_livo2/odometry", self.receive, qos
        )

    def receive(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        self.rows.append(
            {
                "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec),
                "frame_id": message.header.frame_id,
                "child_frame_id": message.child_frame_id,
                "position_m": [float(p.x), float(p.y), float(p.z)],
                "quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
            }
        )


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return ordered[low]
    return ordered[low] * (high - index) + ordered[high] * (index - low)


def yaw_radians(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def circular_mean(values):
    return math.atan2(
        statistics.fmean(math.sin(value) for value in values),
        statistics.fmean(math.cos(value) for value in values),
    )


def summarize(rows, elapsed_s):
    stamps = [row["stamp_ns"] for row in rows]
    if not rows:
        return {"count": 0, "collector_elapsed_s": elapsed_s}
    origin = rows[0]["position_m"]
    displacement = [
        math.sqrt(sum((value - base) ** 2 for value, base in zip(row["position_m"], origin)))
        for row in rows
    ]
    steps = [
        math.sqrt(sum((b - a) ** 2 for a, b in zip(previous["position_m"], current["position_m"])))
        for previous, current in zip(rows, rows[1:])
    ]
    xy_steps = [
        math.hypot(
            current["position_m"][0] - previous["position_m"][0],
            current["position_m"][1] - previous["position_m"][1],
        )
        for previous, current in zip(rows, rows[1:])
    ]
    yaws = [yaw_radians(row["quaternion_xyzw"]) for row in rows]
    stationarity_count = min(200, max(1, len(rows) // 10))
    initial_rows = rows[:stationarity_count]
    final_rows = rows[-stationarity_count:]
    initial_mean = [statistics.fmean(row["position_m"][i] for row in initial_rows) for i in range(3)]
    final_mean = [statistics.fmean(row["position_m"][i] for row in final_rows) for i in range(3)]
    closure = [final_mean[i] - initial_mean[i] for i in range(3)]
    initial_yaw = circular_mean(yaws[:stationarity_count])
    final_yaw = circular_mean(yaws[-stationarity_count:])
    duration_s = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0.0
    return {
        "count": len(rows),
        "collector_elapsed_s": elapsed_s,
        "message_duration_s": duration_s,
        "output_rate_hz": (len(rows) - 1) / duration_s if duration_s > 0 else None,
        "non_monotonic_stamps": sum(b <= a for a, b in zip(stamps, stamps[1:])),
        "frames": sorted({(row["frame_id"], row["child_frame_id"]) for row in rows}),
        "initial_position_m": origin,
        "final_position_m": rows[-1]["position_m"],
        "net_displacement_m": displacement[-1],
        "max_displacement_from_initial_m": max(displacement),
        "trajectory_length_3d_m": sum(steps),
        "trajectory_length_xy_m": sum(xy_steps),
        "stationary_window_samples": stationarity_count,
        "stationary_mean_initial_position_m": initial_mean,
        "stationary_mean_final_position_m": final_mean,
        "closure_xy_m": math.hypot(closure[0], closure[1]),
        "closure_z_m": closure[2],
        "closure_position_3d_m": math.sqrt(sum(value * value for value in closure)),
        "initial_yaw_deg": math.degrees(initial_yaw),
        "final_yaw_deg": math.degrees(final_yaw),
        "closure_yaw_deg": math.degrees(wrap_angle(final_yaw - initial_yaw)),
        "step_m": {
            "median": statistics.median(steps) if steps else None,
            "p95": percentile(steps, 0.95),
            "max": max(steps) if steps else None,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--duration", type=float, default=150.0)
    parser.add_argument("--trajectory-csv")
    args = parser.parse_args()

    rclpy.init()
    node = OdomCollector()
    stop = False
    started = time.monotonic()

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    while (
        rclpy.ok()
        and not stop
        and (args.duration <= 0.0 or time.monotonic() - started < args.duration)
    ):
        rclpy.spin_once(node, timeout_sec=0.25)
    result = summarize(node.rows, time.monotonic() - started)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    if args.trajectory_csv:
        with open(args.trajectory_csv, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_ns", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "yaw_deg"])
            for row in node.rows:
                writer.writerow(
                    [row["stamp_ns"]]
                    + row["position_m"]
                    + row["quaternion_xyzw"]
                    + [math.degrees(yaw_radians(row["quaternion_xyzw"]))]
                )
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
