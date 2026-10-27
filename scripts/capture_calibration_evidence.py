#!/usr/bin/env python3
"""Read-only CameraInfo/image/TF capture. Does not publish or command motion."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from audit_fast_livo_calibration import ROOT, compose, pitch_mount, rigid


def matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    if not np.isclose(x*x+y*y+z*z+w*w, 1, atol=1e-5):
        raise ValueError('invalid TF quaternion')
    return rigid([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], [t.x, t.y, t.z])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--camera-info-topic', default='/camera/color/camera_info')
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    rclpy.init()
    node = rclpy.create_node('calibration_evidence_reader')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    report = {'status': 'LIVE_EVIDENCE_NOT_PROJECTION_VALIDATED', 'runnable': False,
              'camera_info_topic': args.camera_info_topic, 'image_topic': args.image_topic,
              'camera_info_count': 0, 'image_count': 0, 'camera_info_changed': False}

    def camera_info(msg):
        value = dict(width=msg.width, height=msg.height, frame_id=msg.header.frame_id,
                     k=list(msg.k), d=list(msg.d), distortion_model=msg.distortion_model)
        if 'camera_info' in report and report['camera_info'] != value:
            report['camera_info_changed'] = True
        report['camera_info'] = value
        report['camera_info_count'] += 1

    def image(msg):
        report['image'] = dict(width=msg.width, height=msg.height,
                               frame_id=msg.header.frame_id, encoding=msg.encoding)
        report['image_count'] += 1

    subscriptions = [node.create_subscription(CameraInfo, args.camera_info_topic, camera_info, qos_profile_sensor_data),
                     node.create_subscription(Image, args.image_topic, image, qos_profile_sensor_data)]
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    report['tf_frames'] = yaml.safe_load(buffer.all_frames_as_yaml())
    info = report.get('camera_info')
    if info:
        saved = yaml.safe_load((ROOT / 'calibration/camera_pinhole_424x240.yaml').read_text())['/**']['ros__parameters']['camera']
        k = info['k']
        report['saved_intrinsics_match'] = bool(
            (info['width'], info['height']) == (saved['width'], saved['height'])
            and info['distortion_model'] == saved['distortion_model']
            and np.allclose([k[0], k[4], k[2], k[5]], [saved[x] for x in ('fx', 'fy', 'cx', 'cy')], atol=1e-6, rtol=0)
            and len(info['d']) == 5 and np.allclose(info['d'], [saved['d'+str(i)] for i in range(5)], atol=1e-8, rtol=0))
        try:
            tf = buffer.lookup_transform('camera_link', info['frame_id'], Time())
            link_optical = matrix(tf)
            report['T_camera_link_color_optical_live'] = link_optical.tolist()
            base_optical = pitch_mount(.34907, [.25489, 0, .07249]) @ link_optical
            imu_lidar = rigid(np.eye(3), [-.011, -.02329, .04412])
            cl, ic = compose(pitch_mount(np.deg2rad(18), [.187, 0, .4]), base_optical, imu_lidar)
            report['candidate_Rcl'] = cl[:3, :3].reshape(-1).tolist()
            report['candidate_Pcl'] = cl[:3, 3].tolist()
            report['T_imu_color_optical_candidate'] = ic.tolist()
            report['candidate_basis'] = 'Saved factory body mounts plus live RealSense optical TF; raw LiDAR input assumed; projection and timing not validated'
        except Exception as exc:
            report['optical_tf_error'] = str(exc)
    report['capture_complete'] = bool(info and report['image_count'] and not report['camera_info_changed']
                                      and report.get('saved_intrinsics_match')
                                      and report.get('T_camera_link_color_optical_live')
                                      and all(report['image'][k] == info[k] for k in ('width', 'height', 'frame_id')))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({k: v for k, v in report.items() if k not in ('tf_frames',)}, indent=2))
    return 0 if report['capture_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
