"""Capture diagnostic coarse poses and exact raw/IMU stamps. No pose publication."""
import sys,time,re,json
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from sensor_msgs.msg import PointCloud2,Imu
sys.path.insert(0,'/home/ysc/lite3_patrol_project/scripts')
from costmap_ground_filter import cloud_xyz
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
log=Path(sys.argv[2]);seconds=float(sys.argv[3]) if len(sys.argv)>3 else 60
rclpy.init();n=rclpy.create_node('rejected_registration_readonly')
raw={};imu=deque(maxlen=14000);seen=set();rows=[]
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
def source(m):
 raw[stamp(m)]=m
 while len(raw)>600:del raw[min(raw)]
subs=[n.create_subscription(PointCloud2,'/rslidar_points',source,qos),n.create_subscription(Imu,'/imu/data',lambda m:imu.append(m),qos)]
pattern=re.compile(r'refinement diagnostic stamp=([\d.]+) reason=(\w+) dt_ms=([\d.]+) ndt_ms=([\d.]+) source=(\d+) target=(\d+) delta_m=([\d.]+) delta_deg=([\d.]+) score=([^ ]+) overlap=([^ ]+) coarse_xyzq=([^\s]+)')
start=time.monotonic();last=start
while time.monotonic()-start<seconds:
 rclpy.spin_once(n,timeout_sec=.02)
 if time.monotonic()-last<.5 or not raw or not imu:continue
 last=time.monotonic()
 for m in pattern.finditer(log.read_text(errors='replace')):
  st=float(m[1])
  if st in seen:continue
  rs=min(raw,key=lambda t:abs(t-st));im=min(imu,key=lambda x:abs(stamp(x)-st))
  if abs(rs-st)>1e-6 or abs(stamp(im)-st)>.04:continue
  seen.add(st);q=im.orientation;pose=list(map(float,m[11].split(',')))
  name='frame_%02d.npz'%len(rows)
  np.savez_compressed(out/name,source=cloud_xyz(raw[rs]),imu=[q.x,q.y,q.z,q.w],coarse=pose)
  rows.append(dict(file=name,stamp=st,imu_delta=stamp(im)-st,reason=m[2],icp_ms=float(m[3]),ndt_ms=float(m[4]),source_n=int(m[5]),target_n=int(m[6]),delta_m=float(m[7]),delta_deg=float(m[8]),score=list(map(float,m[9].split(','))),overlap=list(map(float,m[10].split(','))),coarse=pose))
(out/'report.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows),flush=True)
n.destroy_node();rclpy.shutdown()
