import rclpy,time,json,cv2,numpy as np
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile,ReliabilityPolicy
rclpy.init();n=rclpy.create_node('static_camera_diagnostic');counts={'best':0,'reliable':0};last={}
def cb(m,k):
 counts[k]+=1;last[k]=m
subs=[n.create_subscription(Image,'/camera/color/image_raw',lambda m,k=k:cb(m,k),QoSProfile(depth=1,reliability=p)) for k,p in [('best',ReliabilityPolicy.BEST_EFFORT),('reliable',ReliabilityPolicy.RELIABLE)]]
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
for k,m in last.items():
 a=np.ndarray((m.height,m.width,3),dtype=np.uint8,buffer=m.data,strides=(m.step,3,1)).copy()
 if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
 cv2.imwrite('/home/ysc/lite3_patrol_project/results/ground_acceptance_20261028/'+k+'.png',a)
print(json.dumps({'counts':counts,'publishers':[str(x.qos_profile) for x in n.get_publishers_info_by_topic('/camera/color/image_raw')]}));n.destroy_node();rclpy.shutdown()
