import rclpy,time,json,sys
from sensor_msgs.msg import Image,PointCloud2,Imu
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data as qos
from pathlib import Path
rclpy.init();n=rclpy.create_node('static_sensor_acceptance_probe');counts={};last={}
def cb(k,m):
 counts[k]=counts.get(k,0)+1;h=m if k=='heartbeat' else m.header;s=h.stamp.sec+h.stamp.nanosec/1e9;last[k]={'age':(time.time() if k=='rgb' else time.monotonic())-s,'clock':'wall' if k=='rgb' else 'monotonic','frame':h.frame_id}
subs=[n.create_subscription(t,topic,lambda m,k=k:cb(k,m),qos) for k,topic,t in [('rgb','/camera/color/image_raw',Image),('depth','/camera/depth/color/points',PointCloud2),('imu','/imu/data',Imu),('grid','/patrol/voa/grid_map',GridMap),('voa_cloud','/patrol/voa/costmap_obstacles',PointCloud2),('heartbeat','/patrol/voa/costmap_heartbeat',Header)]]
end=time.monotonic()+12
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
result={'counts':counts,'last':last,'velocity_publishers':{t:n.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_corrected','/patrol/cmd_vel_raw']}};Path(sys.argv[1]).write_text(json.dumps(result,indent=2));print(json.dumps(result));n.destroy_node();rclpy.shutdown()
