#!/usr/bin/env python3
"""Project qualified local terrain into Nav2, preserving original source time.

Local detection never waits for NDT. This map projection requires live READY
and exact map TF or the existing bounded NDT+leg-odometry compensation. Default
outputs are private; --active selects production map cloud and health topics.
"""
import argparse,time
import numpy as np
from costmap_ground_filter import cloud_xyz
from voa_costmap_bridge import Bridge,stamp_seconds,fresh,project_to_map,MAX_AGE

class LocalGroundBridge(Bridge):
    def __init__(self,*args,active=False,**kwargs):
        prefix='/patrol' if active else '/patrol/lidar_local_shadow'
        super().__init__(*args,output_topic=prefix+'/costmap_obstacles',
                         heartbeat_topic=prefix+'/lidar_costmap_heartbeat',**kwargs)
        self.ready=False;self.ready_received=float('-inf')
    def readiness(self,msg):
        self.ready=bool(msg.data);self.ready_received=time.monotonic()
    def publish(self):
        now=time.monotonic()
        if not self.ready or not 0<=now-self.ready_received<=MAX_AGE:return
        for msg in reversed(self.pending):
            stamp=stamp_seconds(msg.header.stamp)
            if stamp<=self.last_published or not fresh(stamp,now):continue
            if msg.header.frame_id!='robot_foot_print':return
            try:xyz=cloud_xyz(msg)
            except (ValueError, TypeError, KeyError, IndexError):continue
            if not np.isfinite(xyz).all() or np.any(np.linalg.norm(xyz[:,:2],axis=1)>4.001):return
            try:pose=self.projection_transform(msg)
            except Exception:continue
            points=project_to_map(xyz,pose)
            out=self.Cloud();out.header.stamp=msg.header.stamp;out.header.frame_id='map'
            out.height=1;out.width=len(points);out.point_step=12;out.row_step=12*len(points)
            out.fields=[self.Field(name=k,offset=i*4,datatype=self.Field.FLOAT32,count=1) for i,k in enumerate('xyz')]
            out.is_bigendian=False;out.is_dense=True;out.data=points.tobytes()
            if not fresh(stamp,time.monotonic()) or time.monotonic()-self.ready_received>MAX_AGE:return
            self.publisher.publish(out);self.heartbeat.publish(out.header);self.last_published=stamp
            if time.monotonic()-self.last_log>5:
                self.node.get_logger().info('local ground map projection: points=%d age=%.3f compensation=%d'%(len(points),time.monotonic()-stamp,self.predictions))
                self.last_log=time.monotonic()
            break

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--active',action='store_true');args=parser.parse_args()
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data as qos
    from sensor_msgs.msg import PointCloud2,PointField
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool,Header
    from tf2_ros import Buffer,TransformListener
    rclpy.init();node=rclpy.create_node('patrol_local_ground_costmap_bridge');buffer=Buffer();listener=TransformListener(buffer,node)
    bridge=LocalGroundBridge(node,buffer,PointCloud2,PointField,Header,Time,.15,active=args.active)
    topic=('/patrol/lidar_local' if args.active else '/patrol/lidar_local_shadow')+'/obstacles'
    subs=[node.create_subscription(PointCloud2,topic,bridge.receive,qos),
          node.create_subscription(Odometry,'/patrol/registration_odom',bridge.registration,1),
          node.create_subscription(Bool,'/patrol/localization_ready',bridge.readiness,1)]
    def publish():
        if node.count_publishers(topic)!=1:return
        bridge.publish()
    timer=node.create_timer(.05,publish)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
