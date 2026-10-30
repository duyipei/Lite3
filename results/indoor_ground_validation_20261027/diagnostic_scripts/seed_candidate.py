"""One-shot initial pose for stationary calibration test, no motion commands."""
import json,time,rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
p=json.load(open('/tmp/candidate_seed.json'))['pose'];rclpy.init();n=rclpy.create_node('candidate_static_test_seed');pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',1);started=time.monotonic()
while pub.get_subscription_count()==0 and time.monotonic()-started<45:rclpy.spin_once(n,timeout_sec=.1)
if pub.get_subscription_count()!=1:raise RuntimeError('expected exactly one localization consumer')
m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=p[:3];m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=p[3:];m.pose.covariance[0]=m.pose.covariance[7]=.05;m.pose.covariance[35]=.03;pub.publish(m)
for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
print('published diagnostic transformed initial pose',p);n.destroy_node();rclpy.shutdown()
