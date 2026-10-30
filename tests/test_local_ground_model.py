"""Independent known geometry; never uses estimated ground as object truth."""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from local_ground_model import LocalGroundModel

class LocalTerrainTests(unittest.TestCase):
    def setUp(self):
        x,y=np.meshgrid(np.arange(.45,2.45,.025),np.arange(-1.2,1.2,.025))
        self.floor=np.c_[x.ravel(),y.ravel(),np.full(x.size,.25)]
        self.origin=np.array([.187,0,.74751]);self.model=LocalGroundModel()
    def test_flat_ground_and_unmodified_near_threshold_obstacles(self):
        objects=[]
        for height in [.149,.151,.16,.20,1.]:
            o=self.floor[::97].copy();o[:,2]+=height;objects.append(o)
        data=np.vstack([self.floor]+objects);r=self.model.classify(data,self.origin)
        self.assertTrue(r['valid'],r['reason'])
        floor_trusted=r['trusted'][:len(self.floor)]
        self.assertGreater(floor_trusted.mean(),.9)
        self.assertFalse(r['keep'][:len(self.floor)][floor_trusted].any())
        self.assertTrue(r['keep'][:len(self.floor)][~floor_trusted].all())
        offset=len(self.floor)
        for h,pts in zip([.149,.151,.16,.20,1.],objects):
            keep=r['keep'][offset:offset+len(pts)];trusted=r['trusted'][offset:offset+len(pts)]
            if h>=.15:self.assertTrue(keep.all())
            else:self.assertFalse(keep[trusted].any())
            offset+=len(pts)
    def test_slope_and_small_attitude_changes(self):
        for degrees in [-8,0,8]:
            a=np.deg2rad(degrees);c,s=np.cos(a),np.sin(a)
            R=np.array([[c,0,s],[0,1,0],[-s,0,c]])
            points=self.floor@R.T;origin=self.origin@R.T
            r=self.model.classify(points,origin);self.assertTrue(r['valid'],r['reason'])
            self.assertLess(np.percentile(abs(r['heights']),95),1e-5)
    def test_box_top_alone_cannot_become_ground(self):
        top=self.floor.copy();top[:,2]+=.16
        r=self.model.classify(top,self.origin)
        self.assertFalse(r['valid']);self.assertTrue(r['keep'].all())
    def test_steps_and_walls_preserved(self):
        step=self.floor[self.floor[:,0]>1.8].copy();step[:,2]+=.20
        rest=self.floor[self.floor[:,0]<=1.8]
        z,y=np.meshgrid(np.arange(.3,1.4,.03),np.arange(-1.,1.,.03))
        wall=np.c_[np.full(z.size,2.6),y.ravel(),z.ravel()]
        r=self.model.classify(np.vstack([rest,step,wall]),self.origin)
        self.assertTrue(r['valid'],r['reason']);self.assertTrue(r['keep'][len(rest):len(rest)+len(step)].all())
    def test_sparse_and_invalid_input_never_free(self):
        r=self.model.classify(self.floor[:15],self.origin)
        self.assertFalse(r['valid']);self.assertTrue(r['keep'].all())
        with self.assertRaises(ValueError):self.model.classify(self.floor,[np.nan,0,0])
    def test_unsupported_patch_and_drop_are_retained(self):
        extra=np.array([[3.5,0,.25],[1.,0,.10]])
        r=self.model.classify(np.vstack([self.floor,extra]),self.origin)
        self.assertTrue(r['valid']);self.assertTrue(r['keep'][-2:].all())
    def test_wrong_gravity_or_height_fails_closed(self):
        a=np.deg2rad(30);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
        self.assertFalse(self.model.classify(self.floor@R.T,self.origin@R.T)['valid'])
        self.assertFalse(self.model.classify(self.floor,self.origin+[0,0,.3])['valid'])
if __name__=='__main__':unittest.main()

class TemporalAndCalibrationTests(unittest.TestCase):
    def test_source_age_pair_limit_and_quaternion(self):
        from local_ground_shadow import valid_pair
        self.assertTrue(valid_pair(100,100.003,[0,0,0,1],0,100.05))
        self.assertFalse(valid_pair(100,100.041,[0,0,0,1],0,100.05))
        self.assertFalse(valid_pair(100,100,[0,0,0,1],0,100.501))
        self.assertFalse(valid_pair(100,100,[0,0,0,0],0,100.05))
        self.assertFalse(valid_pair(100,100,[0,0,0,1],-1,100.05))
    def test_audited_driver_origin_not_double_transformed(self):
        from local_ground_shadow import driver_origin
        root=Path(__file__).resolve().parents[1]
        np.testing.assert_allclose(driver_origin(root/'logs/MID360_factory_config_20261022.json'),[.187,0,.4])
