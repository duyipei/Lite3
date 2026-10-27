"""Scheduler, geometry and frontend lifecycle checks without neural dependencies."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from collections import deque
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'patrol_nav_ws/src/lite3_patrol_nav'))
from lite3_patrol_nav.xfeat_matching import CandidateScheduler, XFeatMatcher

class SchedulerTests(unittest.TestCase):
    def test_changing_rank_still_covers_all_references(self):
        scheduler=CandidateScheduler(); seen=set()
        for i in range(54):
            ranked=[{'id':j} for j in (range(107) if i%2 else reversed(range(107)))]
            seen.update(f['id'] for f in scheduler.select(ranked,2))
        self.assertEqual(seen,set(range(107)))

    def test_preferred_reference_does_not_starve_exploration(self):
        scheduler=CandidateScheduler(); scheduler.remember([dict(keyframe=3,inliers=30)])
        seen=set()
        for _ in range(12):
            selected=scheduler.select([{'id':j} for j in range(10)],2)
            self.assertEqual(selected[0]['id'],3)
            seen.update(f['id'] for f in selected)
        self.assertEqual(seen,set(range(10)))
        for _ in range(3): scheduler.remember([])
        self.assertIsNone(scheduler.preferred)

class FrontendLifecycleTests(unittest.TestCase):
    def setUp(self):
        # Execute the actual lightweight callbacks, without importing ROS or
        # constructing a node. Model inference must never run in these paths.
        tree=ast.parse((ROOT/'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/rgbd_visual_localizer.py').read_text())
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef))
        methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in
                 ('ready_callback','initializer_state_callback','processing_callback','image_callback','health_callback')]
        scope={}; exec(compile(ast.Module(body=methods,type_ignores=[]),'callbacks','exec'),scope)
        self.callbacks=scope
        self.states=[]
        self.node=SimpleNamespace(backend='xfeat_lighterglue',visual_paused=False,
            localization_ready=False,confirmation_history=deque([1]),publish_state=self.states.append,
            latest_image='old',search_started=20,last_processed_stamp=(1,2),metric_pose_seen=True,last_candidate_time=5)

    def test_ready_pauses_and_loss_does_not_resume(self):
        self.callbacks['ready_callback'](self.node,SimpleNamespace(data=True))
        self.assertTrue(self.node.visual_paused)
        self.callbacks['processing_callback'](self.node)  # must return before touching image or ROS
        self.callbacks['ready_callback'](self.node,SimpleNamespace(data=False))
        self.assertTrue(self.node.visual_paused)
        self.callbacks['initializer_state_callback'](self.node,SimpleNamespace(data='FAILED:ndt'))
        self.assertTrue(self.node.visual_paused)
        self.callbacks['initializer_state_callback'](self.node,SimpleNamespace(data='COLLECTING_VISUAL_POSES'))
        self.assertFalse(self.node.visual_paused)
        self.assertIsNone(self.node.latest_image)
        self.assertEqual(len(self.node.confirmation_history),0)

    def test_orb_pauses_inference_and_images_until_explicit_request(self):
        self.node.backend='orb'
        self.callbacks['ready_callback'](self.node,SimpleNamespace(data=True))
        self.assertTrue(self.node.visual_paused)
        self.callbacks['processing_callback'](self.node)
        self.callbacks['image_callback'](self.node,object())
        self.assertIsNone(self.node.latest_image)
        self.callbacks['health_callback'](self.node)
        self.assertTrue(self.states[-1].startswith('PAUSED:'))
        self.callbacks['ready_callback'](self.node,SimpleNamespace(data=False))
        self.callbacks['initializer_state_callback'](self.node,SimpleNamespace(data='FAILED:odometry_stale'))
        self.assertTrue(self.node.visual_paused)
        self.callbacks['initializer_state_callback'](self.node,SimpleNamespace(data='COLLECTING_VISUAL_POSES'))
        self.assertFalse(self.node.visual_paused)
        self.assertIsNone(self.node.latest_image)
        self.assertIsNone(self.node.last_processed_stamp)

    def test_false_readiness_during_startup_does_not_pause(self):
        for backend in ('orb','xfeat_lighterglue'):
            self.node.backend=backend
            self.callbacks['ready_callback'](self.node,SimpleNamespace(data=False))
            self.assertFalse(self.node.visual_paused)

class GeometryTests(unittest.TestCase):
    def test_cached_correspondences_recover_known_pose_and_discard_missing_depth(self):
        cv2.setRNGSeed(7)
        rng=np.random.RandomState(13)
        xyz=rng.uniform([-1.,-.6,2.],[1.,.6,6.],(80,3))
        k=np.array([[302.,0.,213.],[0.,302.,129.],[0.,0.,1.]])
        target=np.array([.15,-.06,.12])
        p1=cv2.projectPoints(xyz,np.array([.04,-.08,.03]),target,k,None)[0][:,0]
        matcher=XFeatMatcher.__new__(XFeatMatcher)
        features={'keypoints':np.zeros((80,2))}
        matcher.references={0:features}
        matcher.model=SimpleNamespace(match_lighterglue=lambda *a,**kw:(np.zeros((80,2)),p1,np.column_stack((np.arange(80),np.arange(80)))))
        xyz[:5]=np.nan
        solution,detail=matcher.match({'id':0,'xfeat_points':xyz},features,k,18,15,4.)
        self.assertEqual(detail['raw_matches'],80)
        self.assertEqual(detail['matches'],75)
        self.assertIsNotNone(solution)
        np.testing.assert_allclose(solution['transform'][:3,3],target,atol=.001)

if __name__=='__main__': unittest.main()
