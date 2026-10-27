#!/usr/bin/env python3
"""Read-only Jetson telemetry: no publishers, services or motion commands."""
import argparse
from collections import defaultdict
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--sensor-detail', action='store_true',
                        help='also receive the two 200 Hz factory streams; prefer while stationary')
    parser.add_argument('--odom-detail', action='store_true',
                        help='include source-stamped odometry samples to diagnose pose/time jumps')
    parser.add_argument('--grid-detail', action='store_true',
                        help='also deserialize full VOA grids; prefer short stationary diagnostics')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 3600:
        parser.error('--seconds must be between 1 and 3600')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data as qos
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64, String
    rclpy.init()
    node = rclpy.create_node('patrol_navigation_audit')
    pending, received, state = {}, {}, {}
    maxima = defaultdict(float)
    counts = defaultdict(int)
    odom_samples = []

    def mark(name):
        now = time.monotonic()
        if name in received:
            maxima[name + '_gap_s'] = max(maxima[name + '_gap_s'], now-received[name])
        received[name] = now
        counts[name] += 1
        return now

    def request(msg):
        now = mark('voa_input')
        pending[msg.header.frame_id] = (now, msg.twist.linear.x, msg.twist.angular.z)
        for key, value in list(pending.items()):
            if now-value[0] > 2:
                del pending[key]

    def response(msg):
        now = mark('voa_response')
        pair = pending.pop(msg.header.frame_id, None)
        if pair is not None:
            latency = now-pair[0]
            maxima['round_trip_s'] = max(maxima['round_trip_s'], latency)
            counts['over_250ms'] += int(latency > .25)
            if pair[1:] != (0., 0.):
                counts['nonzero_requests_answered'] += 1
                counts['voa_zero_proposals'] += int(msg.twist.linear.x == 0 and msg.twist.angular.z == 0)

    def velocity(name, msg):
        mark(name)
        state[name] = [msg.linear.x, msg.angular.z]

    def odometry(name, msg):
        mark(name)
        if name in ('odom', 'leg_sampled'):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            state['pose' if name == 'odom' else 'leg_pose'] = [p.x, p.y, yaw]
            if name == 'odom' and args.odom_detail and len(odom_samples) < 1000:
                stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                values = (p.x, p.y, yaw, msg.twist.twist.linear.x,
                          msg.twist.twist.angular.z)
                odom_samples.append([stamp_ns] + [
                    value if math.isfinite(value) else None for value in values])

    def gate(msg):
        mark('gate')
        state['gate'] = msg.data

    def ultrasound(msg):
        mark('ultrasound')
        state['ultrasound_m'] = msg.data if math.isfinite(msg.data) else None
        counts['invalid_ultrasound'] += int(not math.isfinite(msg.data) or msg.data <= 0)

    specs = [
        (TwistStamped, '/patrol/voa/input', request),
        (TwistStamped, '/patrol/voa/corrected', response),
        (Twist, '/patrol/cmd_vel_raw', lambda m: velocity('nav_velocity', m)),
        (Twist, '/cmd_vel_corrected', lambda m: velocity('output_velocity', m)),
        (String, '/patrol/gate_state', gate),
        (Odometry, '/odom', lambda m: odometry('odom', m)),
        (Odometry, '/patrol/leg_odom_sampled', lambda m: odometry('leg_sampled', m)),
    ]
    if args.grid_detail:
        from grid_map_msgs.msg import GridMap
        specs.append((GridMap, '/patrol/voa/grid_map', lambda m: mark('voa_grid')))
    if args.sensor_detail:
        specs.extend([
            (Odometry, '/leg_odom2', lambda m: odometry('leg_odom', m)),
            (Float64, '/us_publisher/ultrasound_distance', ultrasound),
        ])
    subscriptions = [node.create_subscription(kind, topic, callback, qos)
                     for kind, topic, callback in specs]
    start = last = time.monotonic()
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now-last >= 1:
                record = dict(wall_time=time.time(), elapsed_s=now-start,
                              state=dict(state), counts=dict(counts), maximum=dict(maxima),
                              receive_age_s={key: now-stamp for key, stamp in received.items()})
                if args.odom_detail:
                    record['odom_samples_stamp_ns_x_y_yaw_vx_wz'] = list(odom_samples)
                print(json.dumps(record, allow_nan=False), flush=True)
                counts.clear()
                maxima.clear()
                odom_samples.clear()
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
