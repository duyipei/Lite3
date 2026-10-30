"""Read-only graph, localization and obstacle projection monitor; no commands."""
import rclpy,time,json,sys
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import Bool,Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from hdl_localization.msg import ScanMatchingStatus
from pathlib import Path
rclpy.init();n=rclpy.create_node('ground_acceptance_static_monitor');q=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT);records={};counts={}
def cb(k,m):
 counts[k]=counts.get(k,0)+1; row={'receive':time.monotonic()}
 if hasattr(m,'header'):
  h=m.header;row['stamp']=h.stamp.sec+h.stamp.nanosec/1e9;row['age']=time.monotonic()-row['stamp']
 if k=='ready':row['ready']=m.data
 if k=='ndt':row.update(converged=m.has_converged,error=m.matching_error,inliers=m.inlier_fraction)
 if k in ('metric','leg'):
  p=m.pose.pose;row['pose']=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
 if k=='obstacles':row['points']=m.width*m.height
 if k=='heartbeat':row['stamp']=m.stamp.sec+m.stamp.nanosec/1e9;row['age']=time.monotonic()-row['stamp']
 records.setdefault(k,[]).append(row)
subs=[n.create_subscription(t,topic,lambda m,k=k:cb(k,m),q) for k,topic,t in [('leg','/leg_odom2',Odometry),('ready','/patrol/localization_ready',Bool),('ndt','/status',ScanMatchingStatus),('metric','/patrol/registration_odom',Odometry),('obstacles','/patrol/costmap_obstacles',PointCloud2),('heartbeat','/patrol/lidar_costmap_heartbeat',Header)]]
end=time.monotonic()+15
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
counts_pub={topic:n.count_publishers(topic) for topic in ['/cmd_vel','/cmd_vel_corrected','/patrol/cmd_vel_raw','/patrol/costmap_obstacles','/patrol/lidar_costmap_heartbeat']}
r={'counts':counts,'records':records,'publishers':counts_pub};Path(sys.argv[1]).write_text(json.dumps(r,indent=2));print(json.dumps({'counts':counts,'last':{k:v[-1] for k,v in records.items()},'publishers':counts_pub}));n.destroy_node();rclpy.shutdown()
