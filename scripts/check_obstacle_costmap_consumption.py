#!/usr/bin/env python3
"""Read-only cloud/costmap correspondence evidence. No motion or health publishing.

Coverage is necessary but not proof of plugin provenance: pre-existing static
obstacles can occupy the same cells. Dynamic insertion/removal still requires
separate acceptance. Report missing inputs rather than treating them as empty.
"""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
from costmap_ground_filter import cloud_xyz


def coverage(xyz, grid):
    p=grid.info.origin.position;q=grid.info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    d=np.asarray(xyz)[:,:2]-[p.x,p.y]
    c,s=math.cos(yaw),math.sin(yaw)
    xy=d@np.array([[c,-s],[s,c]])
    ij=np.floor(xy/grid.info.resolution).astype(int)
    inside=(ij[:,0]>=0)&(ij[:,0]<grid.info.width)&(ij[:,1]>=0)&(ij[:,1]<grid.info.height)
    a=np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
    values=a[ij[inside,1],ij[inside,0]]
    return dict(input=len(xyz),inside=int(inside.sum()),outside=int((~inside).sum()),
                occupied=int((values>=50).sum()),unknown=int((values<0).sum()),
                below_occupied=int(((values>=0)&(values<50)).sum()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=20)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite evidence')
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('obstacle_costmap_acceptance_readonly')
    latest={};counts={};errors=[]
    def receive(key,msg):
        latest[key]=(msg,time.monotonic());counts[key]=counts.get(key,0)+1
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sources={'lidar':'/patrol/costmap_obstacles','voa':'/patrol/voa/costmap_obstacles'}
    maps={'local':'/local_costmap/costmap','global':'/global_costmap/costmap'}
    subs=[node.create_subscription(PointCloud2,t,lambda m,k=k:receive(k,m),qos_profile_sensor_data) for k,t in sources.items()]
    subs += [node.create_subscription(OccupancyGrid,t,lambda m,k=k:receive(k,m),qos) for k,t in maps.items()]
    end=time.monotonic()+args.seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    now=time.monotonic();pairs={}
    for source in sources:
        for target in maps:
            if source not in latest or target not in latest:continue
            cloud,received=latest[source];grid,grid_received=latest[target]
            try:
                if cloud.header.frame_id!='map' or grid.header.frame_id!='map':raise ValueError('frame mismatch')
                row=coverage(cloud_xyz(cloud),grid)
                row.update(cloud_age_s=now-(cloud.header.stamp.sec+cloud.header.stamp.nanosec/1e9),
                           grid_arrival_age_s=now-grid_received)
                pairs[source+'_'+target]=row
            except Exception as error:errors.append(str(error))
    report=dict(scope='static spatial correspondence; no motion/dynamic-removal certification',counts=counts,pairs=pairs,errors=errors,
                endpoints={k:dict(publishers=node.count_publishers(t),subscribers=node.count_subscribers(t)) for k,t in {**sources,**maps}.items()},
                motor_publishers={t:node.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_corrected']})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
