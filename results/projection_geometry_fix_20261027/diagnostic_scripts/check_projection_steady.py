import json,time,sys,numpy as np,rclpy
from std_msgs.msg import Header
from rclpy.qos import QoSProfile,ReliabilityPolicy
rclpy.init();n=rclpy.create_node('projection_steady_readonly');samples=[]
def receive(m):
 if m.frame_id=='map':samples.append([time.monotonic(),m.stamp.sec+m.stamp.nanosec/1e9])
s=n.create_subscription(Header,'/patrol/voa/costmap_heartbeat',receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
t=time.monotonic()
while time.monotonic()-t<60:rclpy.spin_once(n,timeout_sec=.02)
a=np.array(samples);report={'count':len(a),'seconds':time.monotonic()-t}
if len(a)>2:
 age=a[:,0]-a[:,1];held=a[1:,0]-a[:-1,1]
 report.update(age_percentiles=np.percentile(age,[0,50,95,100]).tolist(),max_age_before_next=float(held.max()),intervals_over_half_second=int((held>.5).sum()),overdue_duration_s=float(np.maximum(0,held-.5).sum()),max_arrival_gap=float(np.diff(a[:,0]).max()),tail_age=time.monotonic()-a[-1,1])
open(sys.argv[1],'w').write(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
