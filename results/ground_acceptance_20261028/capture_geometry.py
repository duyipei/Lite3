import json,time,sys
from pathlib import Path
from collections import deque
import numpy as np,rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,PointCloud2
from rclpy.qos import qos_profile_sensor_data as qos
sys.path.insert(0,'/home/ysc/lite3_patrol_project/scripts')
from costmap_ground_filter import cloud_xyz
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
rclpy.init();n=rclpy.create_node('ndt_geometry_readonly');q={k:{} for k in ('source','aligned','published','candidate')};imus=deque(maxlen=3000);saved=[];last=0

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9

def callback(key,msg):
 d=q[key];d[stamp(msg)]=msg
 while len(d)>60: del d[min(d)]

def pose(m):
 p=m.pose.pose;return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
subs=[]
for key,topic,kind in [('source','/rslidar_points',PointCloud2),('aligned','/aligned_points',PointCloud2),('published','/patrol/registration_odom',Odometry),('candidate','/patrol/ndt_candidate',Odometry)]:subs.append(n.create_subscription(kind,topic,lambda m,k=key:callback(k,m),qos))
subs.append(n.create_subscription(Imu,'/imu/data',lambda m:imus.append(m),qos))
started=time.monotonic()
while time.monotonic()-started<45 and len(saved)<10:
 rclpy.spin_once(n,timeout_sec=.02)
 common=set.intersection(*(set(d) for d in q.values()))
 if not common or not imus or time.monotonic()-last<1.:continue
 s=max(common);im=min(imus,key=lambda m:abs(stamp(m)-s));iq=im.orientation
 if abs(stamp(im)-s) > .04: continue
 index=len(saved)
 np.savez_compressed(out/('frame_%02d.npz'%index),source=cloud_xyz(q['source'][s]),aligned=cloud_xyz(q['aligned'][s]),published=pose(q['published'][s]),candidate=pose(q['candidate'][s]),imu=[iq.x,iq.y,iq.z,iq.w])
 saved.append(dict(stamp=s,imu_delta=stamp(im)-s,published=pose(q['published'][s]),candidate=pose(q['candidate'][s])))
 last=time.monotonic()
(out/'report.json').write_text(json.dumps(saved,indent=2));print(json.dumps(saved));n.destroy_node();rclpy.shutdown()
