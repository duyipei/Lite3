import rclpy,time
from std_msgs.msg import Bool
rclpy.init();n=rclpy.create_node('static_acceptance_ready_watch');ready=[False]
s=n.create_subscription(Bool,'/patrol/localization_ready',lambda m:ready.__setitem__(0,m.data),1)
end=time.monotonic()+50
while time.monotonic()<end and not ready[0]:rclpy.spin_once(n,timeout_sec=.1)
print('READY' if ready[0] else 'NOT_READY',flush=True);n.destroy_node();rclpy.shutdown();raise SystemExit(0 if ready[0] else 2)
