#!/usr/bin/env python3
"""Read sensors; optional ZERO-only private VOA probe. No motor topics published."""
import argparse
import json
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--probe', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('--seconds must be in [1, 60]')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64
    from grid_map_msgs.msg import GridMap
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    rclpy.init()
    node = rclpy.create_node('patrol_voa_zero_probe')
    samples = {}
    subscriptions = []
    def callback(topic, message):
        entry = samples.setdefault(topic, {'count': 0})
        entry['count'] += 1
        entry['received'] = time.monotonic()
        if hasattr(message, 'header'):
            entry['stamp'] = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            entry['frame'] = message.header.frame_id
        if hasattr(message, 'data') and isinstance(message.data, float):
            entry['value'] = message.data
    for topic, kind in [('/camera/depth/color/points', PointCloud2), ('/imu/data', Imu),
                        ('/leg_odom2', Odometry), ('/us_publisher/ultrasound_distance', Float64),
                        ('/patrol/voa/grid_map', GridMap)]:
        subscriptions.append(node.create_subscription(kind, topic,
            lambda msg, topic=topic: callback(topic, msg), qos_profile_sensor_data))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    prefix = 'zero_probe_' + uuid.uuid4().hex + '_'
    pending = {}
    responses = []
    nonzero = []
    def response(message):
        sent = pending.pop(message.header.frame_id, None)
        if sent is None or time.monotonic()-sent > .25:
            return
        v = message.twist
        if any(x != 0 for x in (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)):
            nonzero.append(message.header.frame_id)
        else:
            responses.append(time.monotonic())
    subscriptions.append(node.create_subscription(TwistStamped, '/patrol/voa/corrected',
                                                  response, qos_profile_sensor_data))
    publisher = node.create_publisher(TwistStamped, '/patrol/voa/input', qos_profile_sensor_data) if args.probe else None
    start = time.monotonic()
    next_send = start + 2.
    sequence = 0
    conflict = False
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.02)
        now = time.monotonic()
        if publisher and now >= next_send:
            if node.count_publishers('/patrol/voa/input') != 1:
                conflict = True
                break
            sequence += 1
            request = TwistStamped()  # ALL components stay exactly zero.
            request.header.frame_id = prefix + str(sequence)
            request.header.stamp = node.get_clock().now().to_msg()
            pending = {k: v for k, v in pending.items() if now-v <= .25}
            pending[request.header.frame_id] = now
            publisher.publish(request)
            next_send = now + .05
    now = time.monotonic()
    ros_now = node.get_clock().now().nanoseconds / 1e9
    for entry in samples.values():
        entry['arrival_age_s'] = round(now-entry.pop('received'), 3)
        if 'stamp' in entry:
            entry['ros_age_s'] = round(ros_now-entry['stamp'], 3)
            entry['steady_age_s'] = round(now-entry['stamp'], 3)
    try:
        tf = buffer.lookup_transform('base_link', 'camera_depth_optical_frame', Time())
        transform = str(tf.transform)
    except Exception as exc:
        transform = 'MISSING: ' + str(exc)
    report = dict(samples=samples, camera_tf=transform, zero_responses=len(responses),
                  nonzero_responses=len(nonzero), input_conflict=conflict,
                  cmd_vel_publishers=node.count_publishers('/cmd_vel'),
                  corrected_publishers=node.count_publishers('/cmd_vel_corrected'))
    print(json.dumps(report, indent=2))
    good = not conflict and not nonzero and (not args.probe or
        len(responses) >= 20 and now-responses[-1] <= .25)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if good else 1)


if __name__ == '__main__':
    main()
