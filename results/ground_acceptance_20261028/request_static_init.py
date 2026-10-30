"""Wait for isolated localization configuration, then request normal visual init."""
import rclpy,time
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
rclpy.init();n=rclpy.create_node('static_acceptance_init_request')
p=n.create_client(GetParameters,'/patrol_localization/get_parameters')
if not p.wait_for_service(timeout_sec=50):raise RuntimeError('localization service absent')
req=GetParameters.Request();req.names=['fine_registration_enabled'];f=p.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
if not f.done() or not f.result().values[0].bool_value:raise RuntimeError('isolated fine mode not confirmed')
t=n.create_client(Trigger,'/patrol/request_visual_initialization')
if not t.wait_for_service(timeout_sec=5):raise RuntimeError('initializer absent')
f=t.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
if not f.done() or not f.result().success:raise RuntimeError('visual initialization request failed')
print(f.result(),flush=True);n.destroy_node();rclpy.shutdown()
