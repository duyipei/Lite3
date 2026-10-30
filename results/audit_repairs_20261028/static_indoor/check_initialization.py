"""Request normal initializer and observe live status. Never publishes velocity/READY."""
import rclpy,time,json,sys
from pathlib import Path
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
rclpy.init();n=rclpy.create_node('initialization_acceptance_readonly');samples=[];last={};counts={}
def receive(k,m):
 last[k]=m.data;counts[k]=counts.get(k,0)+1
 samples.append(dict(time=time.monotonic(),topic=k,value=m.data))
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
subs=[]
for k,t,kind in [('ready','/patrol/localization_ready',Bool),('guard','/patrol/visual_init_state',String),('visual','/visual_localization/state',String)]:
 subs.append(n.create_subscription(kind,t,lambda m,k=k:receive(k,m),q if k=='ready' else 10))
result={'scope':'normal ORB initializer; production indoor .1m target, no ICP; isolated corrected binary; no motion','status':'NOT_STARTED'}
start=time.monotonic()
try:
 p=n.create_client(GetParameters,'/patrol_localization/get_parameters')
 if not p.wait_for_service(timeout_sec=60):raise RuntimeError('localization service absent')
 req=GetParameters.Request();req.names=['fine_registration_enabled'];f=p.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 if not f.done() or f.result().values[0].bool_value:raise RuntimeError('expected normal optional-ICP-disabled mode')
 client=n.create_client(Trigger,'/patrol/request_visual_initialization')
 if not client.wait_for_service(timeout_sec=20):raise RuntimeError('initializer unavailable')
 f=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=8)
 if not f.done() or not f.result().success:raise RuntimeError('initialization request failed')
 result['request']=f.result().message;result['status']='TIMEOUT';ready_since=None
 end=time.monotonic()+150
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.05)
  publishers={t:n.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_corrected','/patrol/cmd_vel_raw']}
  if any(publishers.values()):raise RuntimeError('unexpected velocity publisher')
  if str(last.get('guard','')).startswith(('FAILED:','BLOCKED:')):
   result['status']='FAILED';break
  if last.get('ready') and str(last.get('guard','')).startswith('READY:'):
   if ready_since is None:ready_since=time.monotonic()
   if time.monotonic()-ready_since>=15:result['status']='READY_HELD_15S';break
  else:ready_since=None

except Exception as exc:result.update(status='ERROR',error=str(exc))
finally:
 result.update(elapsed_s=time.monotonic()-start,last=last,counts=counts,samples=samples,velocity_publishers={t:n.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_corrected','/patrol/cmd_vel_raw']})
 Path(sys.argv[1]).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
 n.destroy_node();rclpy.shutdown()
