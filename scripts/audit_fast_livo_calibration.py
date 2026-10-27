#!/usr/bin/env python3
"""Offline calibration review; emits a report, never a ROS launch configuration.

T_A_B maps coordinates from B to A. Requires numpy and PyYAML, not ROS.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def rigid(rotation, translation):
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    t = np.asarray(translation, dtype=float).reshape(3)
    if not np.isfinite(r).all() or not np.isfinite(t).all():
        raise ValueError('non-finite transform')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError('rotation must be orthonormal with determinant +1')
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = r, t
    return result


def pitch_mount(angle, xyz):
    c, s = np.cos(angle), np.sin(angle)
    return rigid([[c, 0, s], [0, 1, 0], [-s, 0, c]], xyz)


def require_raw_lidar(driver):
    configs = driver['lidar_configs']
    if not configs:
        raise ValueError('no LiDAR configured')
    for config in configs:
        ext = config['extrinsic_parameter']
        if any(not np.isfinite(float(ext[k])) or abs(float(ext[k])) > 1e-9
               for k in ('roll', 'pitch', 'yaw', 'x', 'y', 'z')):
            raise ValueError('FAST-LIVO raw-input profile requires zero driver extrinsics; do not apply the factory mount twice')


def camera_parameters(camera, image_size):
    if (camera['width'], camera['height']) != tuple(image_size):
        raise ValueError('image resolution differs from saved CameraInfo; capture CameraInfo for the actual profile')
    if camera.get('model') != 'Pinhole' or camera.get('distortion_model') != 'plumb_bob':
        raise ValueError('unsupported camera model')
    keys = ('fx', 'fy', 'cx', 'cy', 'd0', 'd1', 'd2', 'd3', 'd4')
    if not all(np.isfinite(float(camera[k])) for k in keys):
        raise ValueError('non-finite camera calibration')
    if camera['fx'] <= 0 or camera['fy'] <= 0 or camera.get('scale', 1) != 1:
        raise ValueError('invalid focal length or unsupported image scaling')
    if camera['d4'] != 0:
        raise ValueError('this FAST-LIVO Pinhole configuration supports only four distortion coefficients')
    return {k: camera[k] for k in ('model', 'width', 'height', 'scale') + keys if k != 'd4'}


def compose(t_base_lidar, t_base_optical, t_imu_lidar):
    t_optical_lidar = np.linalg.inv(t_base_optical) @ t_base_lidar
    t_imu_optical = t_imu_lidar @ np.linalg.inv(t_optical_lidar)
    return t_optical_lidar, t_imu_optical


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver-config', type=Path, default=ROOT / 'config/mid360_jetson_103.json')
    parser.add_argument('--image-size', type=int, nargs=2, default=(424, 240), metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--output', type=Path, required=True, help='new JSON report; refuses to overwrite')
    args = parser.parse_args()
    require_raw_lidar(json.loads(args.driver_config.read_text()))
    camera = yaml.safe_load((ROOT / 'calibration/camera_pinhole_424x240.yaml').read_text())['/**']['ros__parameters']['camera']
    camera = camera_parameters(camera, args.image_size)
    manifest = yaml.safe_load((ROOT / 'calibration/calibration_manifest.yaml').read_text())
    imu = manifest['lidar_imu']['fast_livo2_lidar_to_imu']
    t_imu_lidar = rigid(imu['extrinsic_R'], imu['extrinsic_T'])
    # This unit's saved factory MID360_config.json: degrees, millimetres.
    t_base_lidar = pitch_mount(np.deg2rad(18), [0.187, 0, 0.400])
    # This unit's factory voa_launch.py, also used by voa_camera_tf.py.
    t_base_link = pitch_mount(0.34907, [0.25489, 0, 0.07249])
    # Axis convention only: actual RealSense optical-frame offset is NOT known.
    t_link_optical = rigid([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [0, 0, 0])
    t_base_optical = t_base_link @ t_link_optical
    t_cl, t_ic = compose(t_base_lidar, t_base_optical, t_imu_lidar)
    report = {
        'status': 'GEOMETRY_ONLY_NOT_VALIDATED',
        'runnable': False,
        'convention': 'T_A_B maps B coordinates to A; translations in metres',
        'camera_source': 'calibration/camera_pinhole_424x240.yaml; historical CameraInfo, not live verification',
        'camera': camera,
        'driver_config_checked': str(args.driver_config.resolve()),
        'factory_mount_sources': ['logs/MID360_factory_config_20261022.json', 'scripts/voa_camera_tf.py'],
        'T_base_lidar': t_base_lidar.tolist(),
        'T_base_color_optical_approximate': t_base_optical.tolist(),
        'T_imu_color_optical_approximate': t_ic.tolist(),
        'candidate_extrin_calib': dict(imu, Rcl=t_cl[:3, :3].reshape(-1).tolist(), Pcl=t_cl[:3, 3].tolist()),
        'blockers': [
            'Read live camera_link -> camera_color_optical_frame TF; zero optical translation here is an assumption.',
            'Validate LiDAR-camera projection across scenes and estimate/verify temporal alignment.',
            'Verify live image profile and CameraInfo against saved calibration.',
            'Verify input point coordinates: this report assumes raw LiDAR, not factory-transformed body points.'
        ],
        'upstream': 'https://github.com/DeepRoboticsLab/fast-livo2-deep-robotics',
        'upstream_review_commit': 'f6762c9afc69efbf1b9f170e90c16cf6b7058cb8'
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print('Offline review saved: {}; status={}'.format(args.output, report['status']))


if __name__ == '__main__':
    main()
