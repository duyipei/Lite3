#!/usr/bin/env python3
"""Read-only camera stream diagnosis on Jetson; never reset devices or publish."""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--require-depth', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 3 <= args.seconds <= 120:
        parser.error('--seconds must be between 3 and 120')

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    rclpy.init()
    node = rclpy.create_node('patrol_camera_stream_diagnosis')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    topics = [('rgb', '/camera/color/image_raw', Image),
              ('color_info', '/camera/color/camera_info', CameraInfo),
              ('depth', '/camera/depth/image_rect_raw', Image),
              ('cloud', '/camera/depth/color/points', PointCloud2)]
    samples = {key: dict(count=0, advancing_stamps=0, max_gap_s=0.,
                        last_at=None, last_stamp=None) for key, _, _ in topics}
    started = time.monotonic()

    def receive(key, msg):
        now = time.monotonic()
        s = samples[key]
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if s['last_at'] is not None:
            s['max_gap_s'] = max(s['max_gap_s'], now-s['last_at'])
        if stamp > 0 and (s['last_stamp'] is None or stamp > s['last_stamp']):
            s['advancing_stamps'] += 1
        s.update(count=s['count']+1, last_at=now, last_stamp=stamp,
                 frame=msg.header.frame_id, size=[msg.width, msg.height])
        # Factory pointcloud uses sensor uptime, whereas RGB/depth images use
        # Unix time. Report cloud liveness by arrival; don't invent an epoch age.
        s['source_age_s'] = None if key == 'cloud' else time.time()-stamp/1e9

    subscriptions = [node.create_subscription(kind, topic,
                     lambda msg, key=key: receive(key, msg), qos)
                     for key, topic, kind in topics]
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
        now = time.monotonic()
        for key, topic, _ in topics:
            s = samples[key]
            last = s.pop('last_at')
            s['receive_age_s'] = None if last is None else now-last
            s['rate_hz'] = s['count']/(now-started)
            s['receiving'] = (s['advancing_stamps'] >= 3 and last is not None and now-last <= 1.5)
            s['publishers'] = [dict(node=e.node_namespace+'/'+e.node_name,
                                    reliability=str(e.qos_profile.reliability))
                               for e in node.get_publishers_info_by_topic(topic)]
        required = ['rgb', 'color_info']+(['depth', 'cloud'] if args.require_depth else [])
        ok = all(samples[key]['receiving'] for key in required)
        missing = [key for key in required if not samples[key]['receiving']]
        print(json.dumps(dict(status='PASS' if ok else 'FAIL',
                              failed_streams=missing, duration_s=now-started,
                              scope='Observed liveness only; not sustained-load certification',
                              streams=samples), indent=2, allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
