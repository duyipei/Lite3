#!/usr/bin/env python3
"""Read-only synchronized sensor evidence for a stationary ground review.

No map, motion commands, plane fitting or calibration changes. Reproduce the
factory IMU roll/pitch rotation and nominal footprint translation explicitly.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np

from costmap_ground_filter import cloud_xyz
from voa_costmap_bridge import transform_matrix


def leveling_matrix(q, height=.34751):
    x, y, z, w = np.asarray(q, dtype=float)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or abs(norm-1.) > .01:
        raise ValueError('invalid IMU orientation')
    x, y, z, w = (v/norm for v in (x,y,z,w))
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    t = np.eye(4)
    t[:3,:3] = [[cp, sp*sr, sp*cr], [0, cr, -sr], [-sp, cp*sr, cp*cr]]
    t[2,3] = height
    return t, [roll, pitch]


def apply(t, points):
    return points @ t[:3,:3].T + t[:3,3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=30)
    args = p.parse_args()
    if not 5 <= args.seconds <= 120:
        p.error('seconds must be within 5..120')
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node('ground_reference_readonly')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    imu = deque(maxlen=600)
    latest = {}
    counts = {'lidar':0, 'depth':0, 'imu':0, 'leg':0}
    def stamp(m):
        return m.header.stamp.sec+m.header.stamp.nanosec/1e9
    def receive(k,m):
        counts[k] += 1
        if k == 'imu': imu.append(m)
        else: latest[k] = (time.monotonic(), m)
    subs = [node.create_subscription(kind, topic, lambda m,k=k:receive(k,m), qos)
            for k,topic,kind in [('lidar','/rslidar_points',PointCloud2),
                                ('depth','/camera/depth/color/points',PointCloud2),
                                ('imu','/imu/data',Imu),('leg','/leg_odom2',Odometry)]]
    report = {'nominal_footprint_height_m':.34751,
              'scope':'raw evidence only; no automatic calibration or motion',
              'samples':[], 'errors':[], 'counts':counts}
    started = time.monotonic()
    next_save = started+3
    last_stamps = {'lidar':None, 'depth':None}
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now < next_save or not imu: continue
            arrays, row = {}, {'receive_monotonic':now}
            for key in ('lidar','depth'):
                if key not in latest or now-latest[key][0] > .25: continue
                msg = latest[key][1]
                ts = stamp(msg)
                if ts == last_stamps[key]: continue
                im = min(imu,key=lambda m:abs(stamp(m)-ts))
                dt = stamp(im)-ts
                # This hardware uses monotonic sensor stamps for both clouds.
                # Never silently pair different epochs or old IMU samples.
                if abs(dt) > .04:
                    report['errors'].append({'source':key,'reason':'imu_stamp_gap','gap_s':dt})
                    continue
                try:
                    tf = buffer.lookup_transform('base_link',msg.header.frame_id,Time()).transform
                    base = transform_matrix(tf)
                    q = im.orientation
                    level,rp = leveling_matrix([q.x,q.y,q.z,q.w])
                    raw = cloud_xyz(msg)
                    raw = raw[np.isfinite(raw).all(axis=1)]
                    arrays[key+'_raw'] = raw
                    arrays[key+'_T_base_source'] = base
                    arrays[key+'_T_foot_base'] = level
                    arrays[key+'_foot'] = apply(level,apply(base,raw)).astype('<f4')
                    row[key] = {'stamp':ts,'frame':msg.header.frame_id,'imu_dt_s':dt,
                                'imu_rp_rad':rp,'points':len(raw),'source_age_s':now-ts}
                    last_stamps[key] = ts
                except Exception as exc:
                    report['errors'].append({'source':key,'reason':str(exc)})
            if not arrays: continue
            if 'leg' in latest and now-latest['leg'][0] < .25:
                od = latest['leg'][1]; pos=od.pose.pose.position; q=od.pose.pose.orientation
                row['leg_pose'] = [pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w]
                row['leg_stamp'] = stamp(od)
            index = len(report['samples'])
            row['file'] = 'frame_%03d.npz'%index
            np.savez_compressed(args.output/row['file'], **arrays)
            report['samples'].append(row)
            next_save = now+1.5
        report['duration_s'] = time.monotonic()-started
        report['tf_frames_yaml'] = buffer.all_frames_as_yaml()
    finally:
        (args.output/'capture.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'samples':len(report['samples']),'counts':counts,
                      'errors':report['errors'][-8:],'output':str(args.output)}))
    return 0 if report['samples'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
