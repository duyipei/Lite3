import json,time,rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
rclpy.init();n=rclpy.create_node('first_goal_plan_readonly');client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
if not client.wait_for_server(timeout_sec=10):raise RuntimeError('planner unavailable')
g=ComputePathToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=n.get_clock().now().to_msg();g.pose.pose.position.x=3.4961;g.pose.pose.position.y=-.1237;g.pose.pose.orientation.w=1.;g.planner_id='GridBased'
f=client.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=8);handle=f.result()
if not handle or not handle.accepted:raise RuntimeError('planner goal not accepted')
f=handle.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=20);r=f.result()
if not r:handle.cancel_goal_async();raise RuntimeError('planning timeout')
report=dict(status=r.status,poses=len(r.result.path.poses),path=[[p.pose.position.x,p.pose.position.y] for p in r.result.path.poses]);open('/tmp/first_goal_plan_fixed_20261027.json','w').write(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(status=r.status,poses=len(r.result.path.poses))));client.destroy();n.destroy_node();rclpy.shutdown()
