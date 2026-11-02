#!/usr/bin/env python3
"""Read-only perception, projection and costmap synchronization diagnostics.

No publisher, service, ARM request or navigation goal is created. Costmap topic
arrival cadence does not prove its internal update deadline or a safe path.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=60.)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--nav-config', type=Path,
                   help='legacy compatibility argument; no local collision model is loaded')
    p.add_argument('--perception-only', action='store_true')
    p.add_argument('--save-grids', type=Path, help='save a VOA elevation sample every two seconds')
    args = p.parse_args()
    if args.output.exists() or args.seconds < 5:
        p.error('use a new output path and at least five seconds')
    if args.save_grids:
        args.save_grids.mkdir(parents=True,exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root/'patrol_nav_ws/src/lite3_patrol_nav'))
    from lite3_patrol_nav.voa_grid import decode_grid
    from lite3_patrol_nav.costmap_sync import CostmapSync
    import yaml
    sync_settings=yaml.safe_load((root/'navigation/obstacle_response.yaml').read_text())['planning_sync']
    sync=CostmapSync(sync_settings['costmap_timeout_s'],sync_settings['near_radius_m'],sync_settings['quantization_m'])
    import rclpy
    from grid_map_msgs.msg import GridMap
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.msg import Costmap
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, Header
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init(); node = rclpy.create_node('avoidance_readonly_acceptance')
    streams = {}; errors = []; moving = []; armed = []; saved = [0.]; rejected = []
    pose=[None];sync_samples=[];next_sync=[0.]
    def receive(key, msg):
        now = time.monotonic()
        data = streams.setdefault(key,dict(arrivals=[],ages=[]))
        data['arrivals'].append(now)
        header = msg if key == 'heartbeat' else getattr(msg,'header',None)
        if header is not None and key in ('grid','heartbeat','projection'):
            stamp = header.stamp.sec+header.stamp.nanosec/1e9
            data['ages'].append(now-stamp)
        if key in ('local','global'):
            ros_now=node.get_clock().now().nanoseconds/1e9
            data['ages'].append(ros_now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)
        if key=='projection':sync.receive_cloud(msg,now)
        if key=='local':sync.receive_raw(msg,now,node.get_clock().now().nanoseconds/1e9)
        if key=='odom':
            if msg.header.frame_id=='map':pose[0]=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        if key=='ready' and not msg.data and not args.perception_only:
            errors.append('localization_not_ready')
        if key == 'grid':
            try:
                xyz,resolution = decode_grid(msg)
                if args.save_grids and now-saved[0] >= 2.:
                    np.savez_compressed(args.save_grids/('{:.6f}.npz'.format(stamp)),
                        xyz=xyz,resolution=resolution,stamp=stamp,arrival=now)
                    saved[0] = now
            except Exception as error:
                errors.append(type(error).__name__+':'+str(error))
                rejected.append(dict(arrival=now,source_stamp=stamp,source_age_s=now-stamp,
                                     error=str(error)))
        if key == 'corrected' and any(abs(v)>1e-6 for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
            moving.append(now)
        if key == 'armed' and msg.data:
            armed.append(now)
    sources = {
        'grid': (GridMap,'/patrol/voa/grid_map'),
        'heartbeat': (Header,'/patrol/voa/costmap_heartbeat'),
        'projection': (PointCloud2,'/patrol/voa/costmap_obstacles'),
        'local': (Costmap,'/local_costmap/costmap_raw'),
        'global': (Costmap,'/global_costmap/costmap_raw'),
        'odom': (Odometry,'/odom'),
        'ready': (Bool,'/patrol/localization_ready'),
        'raw': (Twist,'/patrol/cmd_vel_raw'),
        'corrected': (Twist,'/cmd_vel_corrected'),
        'armed': (Bool,'/patrol/armed')}
    qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subs = [node.create_subscription(kind,topic,lambda m,k=k:receive(k,m),qos)
            for k,(kind,topic) in sources.items()]
    started = time.monotonic(); end = started+args.seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if not args.perception_only and now>=next_sync[0] and pose[0] is not None and sync.grid is not None and sync.cloud is not None:
            ok,reason,metrics=sync.check(pose[0],now,node.get_clock().now().nanoseconds/1e9)
            sync_samples.append(dict(ok=ok,reason=reason,**metrics));next_sync[0]=now+.1
    now = time.monotonic(); summary = {}
    for key,data in streams.items():
        arrivals = data['arrivals']; gaps = np.diff(arrivals+[now])
        summary[key] = dict(count=len(arrivals),max_gap_s=float(max(gaps,default=0)),
                            first_arrival_s=arrivals[0]-started)
        if data['ages']:
            summary[key].update(source_age_p95_s=float(np.percentile(data['ages'],95)),
                                source_age_max_s=float(max(data['ages'])))
    failures = list(dict.fromkeys(errors))
    # Match the retained projection heartbeat budget; the retired direct
    # grid-stop layer's 0.25 s deadline is no longer an acceptance condition.
    required = {'grid': .5} if args.perception_only else {
        'grid': .5,'heartbeat':.5,'projection':.5,'local':sync.timeout,'global':1.,
        'corrected':.25,'armed':.5,'odom':.7,'ready':.5}
    for key,deadline in required.items():
        if key not in summary or summary[key]['count'] < 3:
            failures.append(key+':missing_or_insufficient')
        elif summary[key]['max_gap_s'] > deadline:
            failures.append(key+':arrival_gap')
        if summary.get(key,{}).get('source_age_max_s',0) > deadline:
            failures.append(key+':source_stale')
    if moving or armed:
        failures.append('not_a_disarmed_stationary_session')
    if not args.perception_only and (len(sync_samples)<10 or not all(s['ok'] for s in sync_samples[-10:])):
        failures.append('near_obstacle_costmap_sync_not_stable_at_end')
    report = dict(scope='stationary perception only' if args.perception_only else 'stationary timing; dynamic/path acceptance separate',
        passed=not failures,failures=failures,seconds=now-started,streams=summary,
        rejected_grids=rejected,
        costmap_sync=dict(samples=len(sync_samples),waiting=sum(not s['ok'] for s in sync_samples),last=sync_samples[-1] if sync_samples else None,
                          note='raw local costmap spatial correspondence; not plugin provenance or motion certification'),
        nonzero_corrected_count=len(moving),armed_count=len(armed),
        motor_publishers={t:node.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_corrected']})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report,ensure_ascii=False))
    node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
