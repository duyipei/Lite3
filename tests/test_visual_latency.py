"""Run real frontend callbacks with a deterministic inference clock, without ROS/GPU."""
import ast
from collections import deque
import copy
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

from test_visual_initializer_frames import PoseMessage

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'patrol_nav_ws/src/lite3_patrol_nav'))
from lite3_patrol_nav.visual_matching import next_reference_batch


class VisualLatencyTests(unittest.TestCase):
    def setUp(self):
        path = ROOT/'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/rgbd_visual_localizer.py'
        tree = ast.parse(path.read_text())
        cls = next(x for x in tree.body if isinstance(x, ast.ClassDef))
        cls.bases = []
        cls.body = [x for x in cls.body if getattr(x, 'name', '') != '__init__']
        functions = [x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name != 'main']
        scope = dict(math=math, time=time, np=np, cv2=cv2,
                     PoseWithCovarianceStamped=PoseMessage,
                     unique_ratio_matches=lambda *args: [], next_reference_batch=next_reference_batch)
        exec(compile(ast.Module(body=functions+[cls], type_ignores=[]), str(path), 'exec'), scope)
        n = self.node = scope['VisualLocalizer']()
        self.params = dict(max_input_age_s=1.5, max_query_age_s=5.,
                           startup_timeout_s=8., candidate_count=10,
                           recovery_search_interval_s=5., startup_appearance_fallback=False,
                           ratio_test=.8, min_matches=18, min_inliers=15,
                           max_reprojection_error=4., pnp_candidate_count=4,
                           recovery_pnp_candidate_count=12, ambiguity_ratio=.9,
                           ambiguity_distance_m=2., confirmation_max_gap_s=6.,
                           confirmation_samples=1, confirmation_max_position_m=.5,
                           confirmation_max_yaw_rad=.35, map_frame='visual_map',
                           learned_fallback=True, learned_dependencies='', learned_weights='',
                           learned_candidate_count=2, learned_required_references=1)
        n.get_parameter = lambda key: NS(value=self.params[key])
        self.now = 10.
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(self.now*1e9)))
        self.states, self.poses, self.errors = [], [], []
        n.publish_state = self.states.append
        n.get_logger = lambda: NS(info=lambda s: None, warn=lambda s: None, error=self.errors.append)
        n.pose_publisher = NS(publish=lambda msg: self.poses.append(copy.deepcopy(msg)))
        n.visual_paused = False; n.latest_image = None; n.last_processed_stamp = None
        n.search_started = 0.; n.search_strategy = n.search_mode = 'global'
        n.confirmation_history = deque(maxlen=1)
        n.confirmation_source = None; n.last_candidate_time = 0.
        n.failed_searches = 0; n.last_recovery_search = 0.; n.metric_pose_seen = False
        n.backend = 'xfeat_lighterglue'; n.reference_size = (16, 16)
        self.color = np.zeros((16, 16, 3), np.uint8)
        hist = cv2.calcHist([cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)],
                          [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist)
        n.keyframes = [dict(id=1, histogram=hist, appearance=scope['appearance_vector'](self.color),
                            descriptors=np.zeros((40, 32), np.uint8), map_camera=np.eye(4))]
        n.startup_keyframes = n.keyframes
        n.k = n.t_base_camera = np.eye(4)
        n.orb = NS(detectAndCompute=lambda *args: (list(range(40)), np.zeros((40, 32), np.uint8)))
        n.matcher = None
        n.learned_matcher = None; n.learned_unavailable = False
        n.learned_tried_ids = set(); n.learned_preferred_ids = []; n.learned_misses = 0
        n.last_learned_detail = ''
        self.elapsed = 3.5
        def inference(*args):
            self.now += self.elapsed
            return [dict(pose=np.eye(4), source='xfeat_lighterglue_pnp',
                         inliers=30, reprojection=1., keyframe=1, matches=40)]
        n.xfeat_solutions = inference

    def frame(self, stamp=10.):
        ns = round(stamp*1e9)
        return NS(header=NS(stamp=NS(sec=ns//1000000000, nanosec=ns%1000000000),
                            frame_id='camera_color_optical_frame'),
                  encoding='bgr8', height=16, width=16, step=48, data=self.color.tobytes())

    def test_slow_orb_and_xfeat_keep_original_capture_stamp(self):
        n = self.node
        for backend in ('orb', 'xfeat_lighterglue'):
            self.now = 10.; n.last_processed_stamp = None; n.backend = backend
            n.learned_solutions = n.xfeat_solutions
            msg = self.frame(); n.image_callback(msg); n.processing_callback()
            self.assertEqual(self.errors, [])
            self.assertEqual(self.poses[-1].header.stamp.sec, 10)
            self.assertEqual(msg.header.frame_id, 'camera_color_optical_frame')
            self.assertIn('query_age=3.50s', self.states[-1])
        self.assertEqual(len(self.poses), 2)

    def test_old_input_rejected_before_inference(self):
        self.now = 12.
        self.node.image_callback(self.frame()); self.node.processing_callback()
        self.assertEqual(self.poses, [])
        self.assertEqual(self.now, 12.)
        self.assertIn('stale_input', self.states[-1])

    def test_over_budget_result_and_future_stamp_rejected(self):
        self.elapsed = 5.01
        self.node.image_callback(self.frame()); self.node.processing_callback()
        self.assertEqual(self.poses, [])
        self.assertIn('stale_query', self.states[-1])
        self.node.image_callback(self.frame(20.)); self.node.processing_callback()
        self.assertIn('stale_input', self.states[-1])

    def test_cold_loftr_discards_warmup_frame_then_accepts_new_image(self):
        n = self.node; n.backend = 'orb'
        outer = self
        class Matcher:
            def __init__(self, *args): outer.now += 30.
            def warmup(self, color): outer.now += 5.
            def query(self, color): return color
            def match(self, *args):
                outer.now += 2.
                return (dict(transform=np.eye(4), inliers=30, reprojection=1.),
                        dict(matches=40, inliers=30))
        with patch.dict(sys.modules, {'lite3_patrol_nav.learned_matching': NS(LearnedMatcher=Matcher)}):
            n.image_callback(self.frame()); n.processing_callback()
            self.assertEqual(self.errors, [])
            self.assertEqual(self.poses, [])
            self.assertIsNone(n.latest_image)
            self.assertEqual(n.failed_searches, 0)
            self.assertEqual(self.states[-1], 'WARMUP_COMPLETE:waiting_for_new_image')
            n.processing_callback()  # No replay of the warmup image.
            n.image_callback(self.frame(self.now)); n.processing_callback()
        self.assertEqual(self.errors, [])
        self.assertEqual(len(self.poses), 1)
        self.assertEqual(self.poses[0].header.stamp.sec, 45)

    def test_production_frontend_and_both_guards_share_age_budget(self):
        script = (ROOT/'navigation/start_visual_localizer.sh').read_text()
        for map_id in ('indoor_fusion_20260912_001_area_review', 'outdoor_20260921_003_area_review'):
            p = yaml.safe_load((ROOT/'navigation'/map_id/'visual_initialization.yaml').read_text())
            p = p['visual_pose_initializer']['ros__parameters']
            self.assertIn('max_query_age_s:={}'.format(p['initial_visual_max_age_s']), script)
            self.assertGreater(p['max_sample_gap_s'], p['initial_visual_max_age_s'])
            self.assertEqual(p['sensor_max_age_s'], 1.5)
            self.assertEqual(p['ready_odom_max_age_s'], .7)

