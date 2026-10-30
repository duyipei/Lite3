import rclpy,time,json,sys,math
from pathlib import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
rclpy.init();n=rclpy.create_node('manual_static_ndt_seed');pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',1);legs=[]
s=n.create_subscription(Odometry,'/leg_odom2',lambda m:legs.append(m),10)
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
if any(n.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_corrected','/patrol/cmd_vel_raw']):raise RuntimeError('velocity publisher conflict')
if len(legs)<20 or pub.get_subscription_count()!=1:raise RuntimeError('missing odometry or ambiguous localization subscriber')
p0=legs[0].pose.pose.position
if any(abs(m.pose.pose.position.x-p0.x)+abs(m.pose.pose.position.y-p0.y)+abs(m.pose.pose.position.z-p0.z)>.005 for m in legs):raise RuntimeError('leg odometry indicates motion')
q0=legs[0].pose.pose.orientation
v0=[q0.x,q0.y,q0.z,q0.w]
for od in legs:
 q=od.pose.pose.orientation;v=[q.x,q.y,q.z,q.w]
 norm=math.sqrt(sum(x*x for x in v));norm0=math.sqrt(sum(x*x for x in v0))
 if not math.isfinite(norm) or norm<.99 or norm>1.01 or norm0<.99 or norm0>1.01:raise RuntimeError('invalid leg quaternion')
 dot=min(1.,abs(sum(a*b for a,b in zip(v,v0))/(norm*norm0)))
 if 2*math.acos(dot)>.01:raise RuntimeError('leg odometry indicates rotation')
d=json.loads(Path(sys.argv[1]).read_text());v=list(map(float,d['pose_xyz_qxyzw']));assert len(v)==7 and all(math.isfinite(x) for x in v) and abs(sum(x*x for x in v[3:])-1)<.001;m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();p=m.pose.pose;p.position.x,p.position.y,p.position.z=v[:3];p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=v[3:];m.pose.covariance[0]=m.pose.covariance[7]=.04;m.pose.covariance[14]=.04;m.pose.covariance[21]=m.pose.covariance[28]=m.pose.covariance[35]=.02
log=Path(sys.argv[2]) if len(sys.argv)>2 else None
count_before=log.read_text(errors='replace').count('initial pose received!!') if log else 0
pub.publish(m)
end=time.monotonic()+15
ack=False
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if log and log.read_text(errors='replace').count('initial pose received!!')>count_before:
  ack=True;break
if log and not ack:raise RuntimeError('initial pose callback did not acknowledge; no retry or motion')
print(json.dumps(d),flush=True);n.destroy_node();rclpy.shutdown()
