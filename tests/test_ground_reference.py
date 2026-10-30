import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_ground_reference import fit_plane, floor_correction
from capture_ground_reference import apply, leveling_matrix


class GroundReferenceTests(unittest.TestCase):
    def test_plane_recovers_floor_despite_sparse_high_returns(self):
        rng=np.random.default_rng(25)
        xy=rng.uniform([.6,-.2],[1.9,.2],(800,2))
        z=.02*xy[:,0]-.01*xy[:,1]-.34+rng.normal(0,.002,800)
        z[:150]+=.2
        plane,stats=fit_plane(np.column_stack([xy,z]))
        expected=np.array([-.02,.01,1.,.34]);expected/=np.linalg.norm(expected[:3])
        np.testing.assert_allclose(plane,expected,atol=.003)
        self.assertLess(stats['inlier_abs_residual_p95_m'],.008)

    def test_ambiguous_narrow_patch_and_wall_rejected(self):
        rng=np.random.default_rng(3)
        for points in (np.column_stack([rng.uniform(.5,2,100),np.zeros(100),np.zeros(100)]),
                       np.column_stack([np.ones(100),rng.uniform(-.3,.3,100),rng.uniform(0,1,100)])):
            with self.assertRaises(ValueError):fit_plane(points)

    def test_correction_preserves_real_obstacle_height_and_locks_unobserved_dofs(self):
        target=np.array([.01,-.02,1.,.33]);target/=np.linalg.norm(target[:3])
        roll,pitch=-.01,.03
        rx=np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
        ry=np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
        true=np.eye(4);true[:3,:3]=ry@rx;true[2,3]=-.2
        source=np.r_[true[:3,:3].T@target[:3],target[3]+target[:3]@true[:3,3]]
        correction,params=floor_correction(source,target)
        np.testing.assert_allclose(correction,true,atol=1e-10)
        floor=np.array([[1.,0.,(-source[3]-source[0])/source[2]]])
        obstacle=floor+.2*source[:3]
        self.assertAlmostEqual(float(apply(correction,floor)@target[:3]+target[3]),0.,places=8)
        self.assertAlmostEqual(float(apply(correction,obstacle)@target[:3]+target[3]),.2,places=8)
        self.assertEqual(params['yaw_deg'],0.)
        np.testing.assert_array_equal(correction[:2,3],[0.,0.])

    def test_factory_leveling_discards_yaw_and_keeps_nominal_translation(self):
        for yaw in (0.,.7,-1.3):
            t,rp=leveling_matrix([0,0,math.sin(yaw/2),math.cos(yaw/2)])
            np.testing.assert_allclose(t[:3,:3],np.eye(3),atol=1e-12)
            np.testing.assert_allclose(apply(t,np.array([[0,0,-.34751]])),[[0,0,0]],atol=1e-12)
        with self.assertRaises(ValueError):leveling_matrix([0,0,0,0])


if __name__=='__main__':unittest.main()
