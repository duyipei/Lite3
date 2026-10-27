#!/usr/bin/env python3
"""Render captured LiDAR projections; compare with independent RGB-D depth.

Depth residuals are diagnostics, not calibrated 2D landmark reprojection errors.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from audit_fast_livo_calibration import pitch_mount


def project(points, transform, k, d, shape):
    camera = points @ transform[:3, :3].T + transform[:3, 3]
    camera = camera[np.isfinite(camera).all(axis=1) & (camera[:, 2] > .15)]
    if not len(camera):
        return np.zeros(shape, dtype=np.float32)
    pixels = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
    valid = np.isfinite(pixels).all(axis=1) & (pixels[:, 0] >= 0) & (pixels[:, 0] < shape[1]-.5) & (pixels[:, 1] >= 0) & (pixels[:, 1] < shape[0]-.5)
    uv = np.rint(pixels[valid]).astype(int)
    depth = np.full(shape, np.inf, dtype=np.float32)
    np.minimum.at(depth, (uv[:, 1], uv[:, 0]), camera[valid, 2])
    depth[~np.isfinite(depth)] = 0
    return depth


def residuals(lidar, depth):
    mask = (lidar > .3) & (depth > .3) & (lidar < 10) & (depth < 10)
    difference = lidar[mask]-depth[mask]
    return dict(overlap_pixels=int(mask.sum()), median_abs_depth_error_m=float(np.median(abs(difference))) if len(difference) else None,
                median_signed_depth_error_m=float(np.median(difference)) if len(difference) else None,
                p90_abs_depth_error_m=float(np.percentile(abs(difference), 90)) if len(difference) else None,
                fraction_within_0_10m=float(np.mean(abs(difference)<.1)) if len(difference) else None,
                fraction_within_0_20m=float(np.mean(abs(difference)<.2)) if len(difference) else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    args = parser.parse_args()
    capture = json.loads((args.scene/'capture.json').read_text())
    reports, montage = [], []
    for sample in capture['samples']:
        i = sample['index']
        data = np.load(str(args.scene/('points_%02d.npz' % i)))
        image = cv2.imread(str(args.scene/('image_%02d.png' % i)))
        base_optical = pitch_mount(.34907, [.25489, 0, .07249]) @ data['link_optical']
        optical_base = np.linalg.inv(base_optical)
        depth = project(data['depth'], data['color_depth'], data['K'], data['D'], image.shape[:2])
        cloud_depth = depth
        if 'depth_image_mm' in data:
            if sample['depth_image_size'] != sample['depth_info_size'] or sample['depth_image_frame'] != sample['depth_frame']:
                raise ValueError('depth image calibration/frame mismatch')
            if sample['depth_distortion_model'] != 'plumb_bob' or np.any(data['depth_D'] != 0):
                raise ValueError('raw depth deprojection currently requires zero-distortion plumb_bob')
            raw = data['depth_image_mm'].astype(float)*.001
            yy, xx = np.nonzero(raw > 0)
            z = raw[yy,xx]
            dk = data['depth_K']
            points = np.column_stack(((xx-dk[0,2])*z/dk[0,0], (yy-dk[1,2])*z/dk[1,1], z))
            depth = project(points, data['color_depth'], data['K'], data['D'], image.shape[:2])
        factory_mount = pitch_mount(np.deg2rad(18), [.187, 0, .4])
        transforms = {'factory_body_points': optical_base @ data['base_lidar_frame'],
                      'double_factory_mount_negative_control': optical_base @ factory_mount,
                      'inverse_factory_mount_negative_control': optical_base @ np.linalg.inv(factory_mount)}
        row = dict(index=i, lidar_points=len(data['lidar']), depth_points=len(data['depth']), comparisons={},
                   depth_source='raw_depth_image' if 'depth_image_mm' in data else 'published_depth_cloud',
                   depth_projected_pixels=int((depth>0).sum()), cloud_vs_raw_depth=residuals(cloud_depth,depth))
        panels = [image.copy()]
        for name, transform in transforms.items():
            projected = project(data['lidar'], transform, data['K'], data['D'], image.shape[:2])
            stats = residuals(projected, depth)
            stats['projected_pixels'] = int((projected>0).sum())
            row['comparisons'][name] = stats
            overlay = image.copy()
            colors = cv2.applyColorMap(np.uint8(np.clip(projected / 6, 0, 1)*255), cv2.COLORMAP_TURBO)
            mask = projected>0
            overlay[mask] = (.25*overlay[mask] + .75*colors[mask]).astype(np.uint8)
            cv2.imwrite(str(args.scene/('overlay_%02d_%s.png' % (i, name))), overlay)
            if name == 'factory_body_points':
                panels.append(overlay)
                error_overlay = image.copy()
                overlap = (projected>.3) & (depth>.3) & (projected<10) & (depth<10)
                error = abs(projected-depth)
                error_overlay[overlap & (error<.1)] = (0,255,0)
                error_overlay[overlap & (error>=.1) & (error<.2)] = (0,255,255)
                error_overlay[overlap & (error>=.2)] = (0,0,255)
                cv2.imwrite(str(args.scene/('depth_residual_%02d.png' % i)), error_overlay)
        depth_img = cv2.applyColorMap(np.uint8(np.clip(depth/6,0,1)*255), cv2.COLORMAP_TURBO)
        depth_img[depth==0] = 0
        panels.append(depth_img)
        panel = np.concatenate(panels, axis=1)
        panel = cv2.copyMakeBorder(panel, 24, 0, 0, 0, cv2.BORDER_CONSTANT)
        cv2.putText(panel, 'RGB | Factory-body LiDAR projection (0-6m color) | RGB-D depth', (8,17), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
        cv2.imwrite(str(args.scene/('comparison_%02d.png' % i)), panel)
        if len(montage)<4:
            montage.append(panel)
        reports.append(row)
    odom = np.asarray(capture.get('odometry', []))
    motion = {'samples': len(odom)}
    if len(odom):
        motion['max_position_distance_from_first_m'] = float(np.linalg.norm(odom[:,1:4]-odom[0,1:4],axis=1).max())
        motion['max_quaternion_angle_from_first_deg'] = float(np.rad2deg(2*np.arccos(np.clip(abs(odom[:,4:8]@odom[0,4:8]),0,1))).max())
    report = dict(status='STATIC_SCENE_DIAGNOSTIC_NOT_FULL_CALIBRATION_ACCEPTANCE', samples=reports, odometry_motion=motion,
                  limitations=['Arrival-paired frames: timing calibration not verified.', 'No manually identified 2D/3D landmark correspondences: depth residual is NOT pixel reprojection error.',
                               'Depth sensor errors, sparse coverage and occlusions affect residuals.', 'Factory-body input interpretation follows inspected vendor launch/config; not inferred from frame_id alone.'])
    (args.scene/'validation.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    if montage:
        cv2.imwrite(str(args.scene/'projection_contact_sheet.jpg'), np.concatenate(montage,axis=0))
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
