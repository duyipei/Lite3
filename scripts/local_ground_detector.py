#!/usr/bin/env python3
"""Local ground detector. No map pose or velocity command is published.

Never publishes velocity, localization READY, map-frame obstacle clouds or
safety heartbeats. --active selects the local input consumed by the guarded bridge. Driver E already applied; apply only A and F.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import time
import math
import numpy as np
from local_ground_model import LocalGroundModel
from capture_ground_reference import apply,leveling_matrix
from costmap_ground_filter import cloud_xyz
from voa_costmap_bridge import transform_matrix


def valid_pair(stamp, imu_stamp, q, covariance, now):
    return (all(math.isfinite(x) for x in [stamp,imu_stamp,now,covariance,*q])
            and stamp>0 and -.05<=now-stamp<=.5
            and abs(stamp-imu_stamp)<=.04 and covariance>=0
            and abs(sum(x*x for x in q)-1)<=.01)


def driver_origin(path):
    configs=json.loads(Path(path).read_text())['lidar_configs']
    if len(configs)!=1:raise ValueError('expected one audited Mid360')
    ext=configs[0]['extrinsic_parameter']
    actual=np.array([ext[k] for k in ['roll','pitch','yaw','x','y','z']],float)
    if not np.allclose(actual,[0,18,0,187,0,400],atol=1e-6):
        raise ValueError('driver extrinsics differ from audited E; refuse guessed origin')
    return actual[3:]/1000


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver-config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=60)
    parser.add_argument('--active',action='store_true')
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite evidence')
    origin=driver_origin(args.driver_config)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data as qos
    from rclpy.time import Time
    from sensor_msgs.msg import Imu,PointCloud2,PointField
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    rclpy.init();node=rclpy.create_node('patrol_local_ground_shadow')
    buffer=Buffer();listener=TransformListener(buffer,node)
    imu=deque(maxlen=2400);pending=[None];last=[0.];rows=deque(maxlen=1800);counts={};model=LocalGroundModel()
    prefix='/patrol/lidar_local' if args.active else '/patrol/lidar_local_shadow'
    cloud_pub=node.create_publisher(PointCloud2,prefix+'/obstacles',1)
    status_pub=node.create_publisher(String,prefix+'/status',1)
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
    def reject(reason,s):
        counts[reason]=counts.get(reason,0)+1
        msg=String();msg.data=json.dumps(dict(valid=False,reason=reason,source_stamp=s));status_pub.publish(msg)
    def tick():
        msg=pending[0]
        if msg is None:return
        s=stamp(msg)
        if s<=last[0]:return
        now=time.monotonic()
        if not -.05<=now-s<=.5:
            last[0]=s;reject('stale_cloud',s);return
        if not imu:return
        im=min(imu,key=lambda m:abs(stamp(m)-s));q=im.orientation;quat=[q.x,q.y,q.z,q.w]
        if not valid_pair(s,stamp(im),quat,im.orientation_covariance[0],now):
            reject('invalid_imu_pair',s);return
        if msg.header.frame_id!='rslidar':
            last[0]=s;reject('wrong_source_frame',s);return
        try:
            A=transform_matrix(buffer.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform)
            F,_=leveling_matrix(quat)
            points=apply(F@A,cloud_xyz(msg));laser=apply(F@A,origin[None,:])[0]
            result=model.classify(points,laser)
        except Exception as exc:
            last[0]=s;reject('geometry_error:'+str(exc),s);return
        last[0]=s
        row={k:v for k,v in result.items() if k not in ['points','heights','trusted','keep','source_indices']}
        row.update(stamp=s,age_s=time.monotonic()-s,imu_delta_s=stamp(im)-s,
                   velocity_publishers={t:node.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_corrected','/patrol/cmd_vel_raw']})
        if time.monotonic()-s>.5:
            row.update(valid=False,reason='processing_expired');reject('processing_expired',s)
        if row['valid']:
            xyz=np.asarray(result['points'][result['keep']],dtype='<f4')
            out=PointCloud2();out.header.stamp=msg.header.stamp;out.header.frame_id='robot_foot_print'
            out.height=1;out.width=len(xyz);out.point_step=12;out.row_step=12*len(xyz)
            out.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate('xyz')]
            out.is_bigendian=False;out.is_dense=True;out.data=xyz.tobytes();cloud_pub.publish(out)
            counts['valid']=counts.get('valid',0)+1
        else:counts[row['reason']]=counts.get(row['reason'],0)+1
        rows.append(row);status=String();status.data=json.dumps(row);status_pub.publish(status)
        if len(rows)%30==1:print(json.dumps(row),flush=True)
    subs=[node.create_subscription(Imu,'/imu/data',lambda m:imu.append(m),qos),
          node.create_subscription(PointCloud2,'/rslidar_points',lambda m:pending.__setitem__(0,m),qos)]
    timer=node.create_timer(.1,tick);end=time.monotonic()+args.seconds if args.seconds>0 else float('inf')
    try:
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(scope='local terrain only; no map safety heartbeat or motion',active=args.active,counts=counts,rows=list(rows)),indent=2))
        print(json.dumps(counts),flush=True);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
