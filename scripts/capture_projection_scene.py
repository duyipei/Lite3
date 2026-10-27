#!/usr/bin/env python3
"""Capture read-only, arrival-paired static scene evidence; no motion publishers."""
import argparse
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from capture_calibration_evidence import matrix


def xyz(msg):
    fields = {f.name: f for f in msg.fields}
    types = {7: 'f4', 8: 'f8'}
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                     'formats': [endian+types[fields[k].datatype] for k in ('x', 'y', 'z')],
                     'offsets': [fields[k].offset for k in ('x', 'y', 'z')], 'itemsize': msg.point_step})
    a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                   strides=(msg.row_step, msg.point_step))
    points = np.stack([a[k].reshape(-1) for k in ('x', 'y', 'z')], axis=1)
    return points[np.isfinite(points).all(axis=1)]


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=20)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('projection_scene_reader')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    queues = {k: deque(maxlen=12) for k in ('image', 'lidar', 'depth', 'depth_image')}
    infos, odom, samples = {}, [], []
    subscriptions = []
    for key, topic, kind in [('image', '/camera/color/image_raw', Image),
                              ('lidar', '/rslidar_points', PointCloud2),
                              ('depth', '/camera/depth/color/points', PointCloud2),
                              ('depth_image', '/camera/depth/image_rect_raw', Image)]:
        subscriptions.append(node.create_subscription(kind, topic, lambda m, k=key: queues[k].append((time.monotonic(), m)), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: infos.update(color=m), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(CameraInfo, '/camera/depth/camera_info', lambda m: infos.update(depth=m), qos_profile_sensor_data))
    def on_odom(m):
        t, q = m.pose.pose.position, m.pose.pose.orientation
        odom.append([stamp(m), t.x, t.y, t.z, q.x, q.y, q.z, q.w])
    subscriptions.append(node.create_subscription(Odometry, '/leg_odom2', on_odom, qos_profile_sensor_data))
    end, next_save = time.monotonic()+args.seconds, time.monotonic()+3
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if time.monotonic() < next_save or not all(queues.values()) or len(infos)<2:
            continue
        at, cloud = queues['lidar'][-1]
        if time.monotonic()-at > .5:
            continue
        it, im = min(queues['image'], key=lambda v: abs(v[0]-at))
        dt, dep = min(queues['depth'], key=lambda v: abs(v[0]-at))
        dit, dim = min(queues['depth_image'], key=lambda v: abs(v[0]-at))
        if max(abs(it-at), abs(dt-at), abs(dit-at)) > .15:
            continue
        ci = infos['color']
        try:
            link_optical = matrix(buffer.lookup_transform('camera_link', ci.header.frame_id, Time()))
            color_depth = matrix(buffer.lookup_transform(ci.header.frame_id, dep.header.frame_id, Time()))
            base_lidar_frame = matrix(buffer.lookup_transform('base_link', cloud.header.frame_id, Time()))
        except Exception:
            continue
        if im.encoding not in ('rgb8', 'bgr8'):
            raise ValueError('unsupported RGB encoding '+im.encoding)
        rgb = np.ndarray((im.height, im.width, 3), dtype=np.uint8, buffer=im.data, strides=(im.step, 3, 1)).copy()
        if im.encoding == 'rgb8':
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        index = len(samples)
        cv2.imwrite(str(args.output / ('image_%02d.png' % index)), rgb)
        if dim.encoding != '16UC1':
            raise ValueError('expected millimetre 16UC1 depth image')
        depth_image = np.ndarray((dim.height, dim.width), dtype='>u2' if dim.is_bigendian else '<u2',
                                 buffer=dim.data, strides=(dim.step, 2)).copy()
        di = infos['depth']
        np.savez_compressed(str(args.output / ('points_%02d.npz' % index)), lidar=xyz(cloud), depth=xyz(dep),
                            link_optical=link_optical, color_depth=color_depth, base_lidar_frame=base_lidar_frame,
                            K=np.array(ci.k).reshape(3, 3), D=np.array(ci.d), depth_image_mm=depth_image,
                            depth_K=np.array(di.k).reshape(3,3), depth_D=np.array(di.d))
        samples.append(dict(index=index, lidar_frame=cloud.header.frame_id, depth_frame=dep.header.frame_id,
                            color_frame=ci.header.frame_id, image_lidar_arrival_delta_s=it-at,
                            depth_lidar_arrival_delta_s=dt-at, image_stamp=stamp(im), lidar_stamp=stamp(cloud),
                            depth_stamp=stamp(dep), distortion_model=ci.distortion_model,
                            depth_image_stamp=stamp(dim), depth_image_arrival_delta_s=dit-at,
                            depth_image_frame=dim.header.frame_id, depth_info_frame=di.header.frame_id,
                            depth_image_size=[dim.width, dim.height], depth_info_size=[di.width, di.height],
                            depth_distortion_model=di.distortion_model))
        next_save = time.monotonic()+2
    report = dict(samples=samples, odometry=odom,
                  pairing='nearest local receive time, <=150ms; NOT proof of hardware timestamp synchronization',
                  counts_buffered={k: len(v) for k, v in queues.items()})
    (args.output / 'capture.json').write_text(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    print('Saved {} samples to {}'.format(len(samples), args.output))
    return 0 if samples else 2


if __name__ == '__main__':
    raise SystemExit(main())
