from pathlib import Path
import sys,unittest,copy,math
from types import SimpleNamespace as N
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import local_ground_costmap_bridge as M
class Cloud:
 def __init__(self):self.header=N()
class Field(N):FLOAT32=7

def cloud(stamp=100.,points=((1.,0.,.4),),frame='robot_foot_print'):
 c=Cloud();c.header=N(stamp=N(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9)),frame_id=frame)
 p=np.asarray(points,dtype='<f4').reshape(-1,3);c.height=1;c.width=len(p);c.point_step=12;c.row_step=12*len(p);c.is_bigendian=False;c.data=p.tobytes()
 c.fields=[Field(name=k,offset=i*4,datatype=7) for i,k in enumerate('xyz')];return c

def tf(x=0.,pitch=0.):return N(translation=N(x=x,y=0.,z=0.),rotation=N(x=0.,y=math.sin(pitch/2),z=0.,w=math.cos(pitch/2)))
class LocalBridgeTests(unittest.TestCase):
 def setUp(self):
  self.out={}
  def publisher(kind,topic,qos):
   self.out[topic]=[];return N(publish=lambda x:self.out[topic].append(copy.deepcopy(x)))
  node=N(create_publisher=publisher,get_logger=lambda:N(info=lambda s:None))
  self.buffer=N(lookup_transform=lambda target,source,t:N(transform=tf(pitch=.3)))
  self.clock=patch.object(M.time,'monotonic',return_value=100.2);self.now=self.clock.start();self.addCleanup(self.clock.stop)
  self.b=M.LocalGroundBridge(node,self.buffer,Cloud,Field,N,N(from_msg=M.stamp_seconds),.15)
  self.heart='/patrol/lidar_local_shadow/lidar_costmap_heartbeat';self.topic='/patrol/lidar_local_shadow/costmap_obstacles'
 def test_no_ready_no_health_and_no_production_topics(self):
  self.b.receive(cloud());self.b.publish();self.assertFalse(self.out[self.heart]);self.assertNotIn('/patrol/costmap_obstacles',self.out)
 def test_geometry_before_flattening_and_original_timestamp(self):
  self.b.readiness(N(data=True));self.b.receive(cloud());self.b.publish()
  out=self.out[self.topic][-1];np.testing.assert_allclose(np.frombuffer(out.data,'<f4'),[math.cos(.3)+.4*math.sin(.3),0,.5],atol=1e-6)
  self.assertEqual(M.stamp_seconds(out.header.stamp),100.)
 def test_stale_replayed_and_revoked_ready_do_not_refresh(self):
  self.b.readiness(N(data=True));self.b.receive(cloud());self.b.publish()
  self.b.publish();self.assertEqual(len(self.out[self.heart]),1)
  self.now.return_value=100.8;self.b.receive(cloud(100.7));self.b.publish();self.assertEqual(len(self.out[self.heart]),1)
  self.b.readiness(N(data=False));self.b.publish();self.assertEqual(len(self.out[self.heart]),1)
 def test_missing_transform_and_wrong_frame_rejected(self):
  self.b.readiness(N(data=True));self.b.receive(cloud(frame='camera_link'));self.b.publish();self.assertFalse(self.out[self.heart])
  self.b.receive(cloud(100.1));self.buffer.lookup_transform=lambda *a:(_ for _ in ()).throw(ValueError('missing TF'))
  self.b.publish();self.assertFalse(self.out[self.heart])
 def test_leg_compensation_requires_fresh_anchor(self):
  self.b.readiness(N(data=True))
  anchor=N(header=N(frame_id='map',stamp=N(sec=100,nanosec=0)),child_frame_id='ndt_registration',pose=N(pose=N(position=N(x=0.,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))))
  self.b.registration(anchor)
  def lookup(target,source,t):
   if target=='map':raise ValueError('NDT pending')
   return N(transform=tf(x=t-100))
  self.buffer.lookup_transform=lookup;self.b.receive(cloud(100.1));self.b.publish()
  np.testing.assert_allclose(np.frombuffer(self.out[self.topic][-1].data,'<f4'),[1.1,0,.5],atol=1e-6)
 def test_nonfinite_and_outside_qualified_range_rejected(self):
  self.b.readiness(N(data=True))
  for t,p in [(100.,((np.nan,0,0),)),(100.1,((5,0,0),))]:
   self.b.receive(cloud(t,p));self.b.publish()
  self.assertFalse(self.out[self.heart])
if __name__=='__main__':unittest.main()
