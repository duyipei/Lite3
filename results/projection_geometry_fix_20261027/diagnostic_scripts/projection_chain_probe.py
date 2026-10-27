import json,time,sys
from pathlib import Path
import numpy as np,rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
rclpy.init();node=rclpy.create_node('projection_chain_readonly');qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
streams={k:[] for k in ('grid','heartbeat','registration','leg_tf','ndt_tf')}
def add(k,h):streams[k].append([time.monotonic(),h.stamp.sec+h.stamp.nanosec/1e9])
def tf(m):
 for t in m.transforms:
  if t.header.frame_id=='robot_odom' and t.child_frame_id=='leg_base_link':add('leg_tf',t.header)
  if t.header.frame_id=='map' and t.child_frame_id=='base_link':add('ndt_tf',t.header)
subs=[node.create_subscription(GridMap,'/patrol/voa/grid_map',lambda m:add('grid',m.header),qos),node.create_subscription(Header,'/patrol/voa/costmap_heartbeat',lambda m:add('heartbeat',m),qos),node.create_subscription(Odometry,'/patrol/registration_odom',lambda m:add('registration',m.header),qos),node.create_subscription(TFMessage,'/tf',tf,qos)]
t=time.monotonic();spin=[]
while time.monotonic()-t<60:
 a=time.monotonic();rclpy.spin_once(node,timeout_sec=.02);spin.append(time.monotonic()-a)
report={'seconds':time.monotonic()-t,'max_spin_s':max(spin),'streams':{}}
for k,v in streams.items():
 a=np.array(v);s={'count':len(a)}
 if len(a)>2:
  s.update(age_percentiles=np.percentile(a[:,0]-a[:,1],[50,95,100]).tolist(),max_gap=float(np.diff(a[:,0]).max()))
 report['streams'][k]=s
h=streams['heartbeat'];events=[]
for prev,cur in zip(h,h[1:]):
 if cur[0]-prev[1]<=.5:continue
 cutoff=prev[1]+.5;e={'held_age_before_next':cur[0]-prev[1],'arrival_gap':cur[0]-prev[0],'previous_source':prev[1],'cutoff':cutoff,'ages_at_cutoff':{}}
 for k,v in streams.items():
  vv=[x for x in v if x[0]<=cutoff]
  if vv:e['ages_at_cutoff'][k]=cutoff-vv[-1][1]
 events.append(e)
report['overruns']=events
out=Path(sys.argv[1]);out.write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(str(out)+'.npz',**{k:np.array(v) for k,v in streams.items()});print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
