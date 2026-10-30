"""Temporary sensor-only A/B relay. Never publishes control or readiness."""
import json,sys,copy,time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
C=np.array(json.load(open(sys.argv[1]))['T_corrected_base_from_nominal_lidar_base'])
assert C.shape==(4,4) and np.isfinite(C).all() and np.allclose(C[3],[0,0,0,1])
assert np.allclose(C[:3,:3].T@C[:3,:3],np.eye(3),atol=1e-6) and np.linalg.norm(C[:3,3])<.4
rclpy.init();n=rclpy.create_node('floor_candidate_sensor_only');q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT);pub=n.create_publisher(PointCloud2,'/patrol/calibration/lidar_candidate',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE));count=[0]
def cb(m):
 if m.header.frame_id!='rslidar':raise ValueError('unexpected source frame')
 # Actual base_link<-rslidar identity was captured and checked before this test.
 out=copy.deepcopy(m);buf=bytearray(m.data);fields={f.name:f for f in m.fields}
 if any(fields[k].datatype!=7 or fields[k].count!=1 for k in ('x','y','z')):raise ValueError('unexpected XYZ type')
 dt=np.dtype({'names':['x','y','z'],'formats':[('>' if m.is_bigendian else '<')+'f4']*3,'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':m.point_step});a=np.ndarray((m.height,m.width),dtype=dt,buffer=buf,strides=(m.row_step,m.point_step));p=np.stack([a[k] for k in ['x','y','z']],axis=-1);p=p@C[:3,:3].T+C[:3,3]
 for i,k in enumerate(['x','y','z']):a[k]=p[:,:,i]
 out.data=bytes(buf);out.header.frame_id='base_link';pub.publish(out);count[0]+=1
 if count[0]==1:n.get_logger().info('first corrected cloud; original source stamp preserved; diagnostic topic only')
s=n.create_subscription(PointCloud2,'/rslidar_points',cb,q)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:print('published',count[0]);n.destroy_node();rclpy.shutdown()
