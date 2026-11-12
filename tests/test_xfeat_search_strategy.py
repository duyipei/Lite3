"""Exercise actual XFeat search callbacks with fake inference, clocks and ROS."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import test_visual_latency as latency_fixture
import test_visual_initializer_frames as initializer_fixture
from lite3_patrol_nav import xfeat_matching


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav'
DATABASE = ROOT/'results/visual_maps/patrol_map_20261031_01/visual_map.db'


class ConstructorTests(unittest.TestCase):
    def test_invalid_startup_ids_do_not_fall_back_to_an_unconfigured_frame(self):
        tree = ast.parse((PACKAGE/'rgbd_visual_localizer.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        begin = next(i for i, n in enumerate(init.body) if isinstance(n, ast.Assign)
                     and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'configured_startup_ids')
        end = next(i for i, n in enumerate(init.body) if isinstance(n, ast.Assign)
                   and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'startup_pose')
        code = compile(ast.Module(body=init.body[begin:end], type_ignores=[]), 'startup_metadata', 'exec')
        with self.assertRaisesRegex(RuntimeError, 'no configured startup keyframes'):
            exec(code, dict(self=NS(keyframes=[dict(id=0)]),
                            metadata=dict(startup_keyframe_ids=[99], startup_region_keyframe_ids=[100])))

    def test_xfeat_honors_each_strategy_and_startup_metadata_union(self):
        with sqlite3.connect('file:{}?mode=ro'.format(DATABASE), uri=True) as connection:
            metadata = {k: json.loads(v) for k, v in connection.execute('SELECT key,value FROM metadata')}
        expected = set(metadata['startup_keyframe_ids']) | set(metadata['startup_region_keyframe_ids'])
        self.assertEqual(expected, set(range(7)) | set(range(34, 40)))
        for strategy in ('startup_then_global', 'startup_only', 'global'):
            with self.subTest(strategy=strategy):
                overrides = dict(database=str(DATABASE), matching_backend='xfeat_lighterglue',
                                 search_strategy=strategy)

                class FakeNode(initializer_fixture.Node):
                    def declare_parameter(self, key, value):
                        self.params[key] = overrides.get(key, value)

                class FakeMatcher:
                    def __init__(self, *args): pass
                    def prepare(self, frames): pass
                    def warmup(self): pass

                fake = {
                    'rclpy': NS(), 'rclpy.node': NS(Node=FakeNode),
                    'rclpy.qos': NS(qos_profile_sensor_data=NS(),
                        QoSProfile=lambda **kw: NS(), ReliabilityPolicy=NS(RELIABLE=1, BEST_EFFORT=2),
                        DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
                    'geometry_msgs.msg': NS(PoseWithCovarianceStamped=initializer_fixture.PoseMessage),
                    'sensor_msgs.msg': NS(Image=NS), 'std_msgs.msg': NS(String=NS, Bool=NS),
                }
                with patch.dict(sys.modules, fake), patch.object(xfeat_matching, 'XFeatMatcher', FakeMatcher):
                    spec = importlib.util.spec_from_file_location('xfeat_constructor_test', PACKAGE/'rgbd_visual_localizer.py')
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                    node = module.VisualLocalizer()
                self.assertEqual(node.search_strategy, strategy)
                self.assertEqual(node.search_mode, 'global' if strategy == 'global' else 'startup')
                self.assertEqual({f['id'] for f in node.startup_keyframes}, expected)


class SearchTests(unittest.TestCase):
    def setUp(self):
        self.h = latency_fixture.VisualLatencyTests()
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.n = self.h.node
        del self.n.xfeat_solutions  # Use the real callback, not the latency fixture's inference stub.
        self.frames([0, 1, 99], [0, 1])
        self.n.configured_search_strategy = self.n.search_strategy = 'startup_then_global'
        self.n.search_mode = 'startup'
        self.seen, self.remembered, self.results = [], [], {}
        self.delay = 0.
        matcher = self.n.xfeat_matcher = xfeat_matching.XFeatMatcher.__new__(xfeat_matching.XFeatMatcher)
        matcher.scheduler = xfeat_matching.CandidateScheduler()
        matcher.query = lambda color: color
        matcher.match = self.match
        original_remember = matcher.remember

        def remember(solutions):
            self.remembered.append(copy.deepcopy(solutions))
            original_remember(solutions)

        matcher.remember = remember
        self.clock = patch('time.monotonic', lambda: self.h.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def frames(self, ids, startup_ids):
        prototype = self.n.keyframes[0]
        self.n.keyframes = [dict(prototype, id=i) for i in ids]
        self.n.startup_keyframes = [f for f in self.n.keyframes if f['id'] in startup_ids]

    def result(self, keyframe, x=0., inliers=30, yaw=0.):
        pose = np.eye(4)
        pose[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        pose[0, 3] = x
        self.results[keyframe] = dict(transform=np.linalg.inv(pose), inliers=inliers, reprojection=1.)

    def match(self, frame, *args):
        self.seen.append(frame['id'])
        self.h.now += self.delay
        result = self.results.get(frame['id'])
        return result, dict(raw_matches=40, matches=40, inliers=result['inliers'] if result else 0,
                            reason='accepted' if result else 'insufficient_inliers')

    def step(self, now=None, stamp=None):
        if now is not None:
            self.h.now = now
        message = self.h.frame(self.h.now if stamp is None else stamp)
        self.n.image_callback(message)
        self.n.processing_callback()

    def test_startup_success_locks_restricted_search(self):
        self.result(0)
        self.step(10.)
        self.assertEqual(len(self.h.poses), 1)
        self.assertEqual(self.n.search_strategy, 'startup_only')
        self.step(100.)
        self.assertEqual(self.n.search_mode, 'startup')
        self.assertNotIn(99, self.seen)
        self.assertEqual(self.n.xfeat_matcher.scheduler.preferred, 0)

    def test_startup_failure_falls_back_after_time_and_valid_attempts(self):
        for i in range(8):
            self.step(10.+i)
            self.assertEqual(self.n.search_mode, 'startup')
        self.assertEqual(self.n.startup_attempts, 8)
        self.step(18.)
        self.assertEqual(self.n.search_mode, 'global')

    def test_wall_time_alone_does_not_trigger_global(self):
        self.step(10.)
        self.step(100.)
        self.assertEqual(self.n.startup_attempts, 2)
        self.assertEqual(self.n.search_mode, 'startup')

    def test_attempt_count_alone_does_not_trigger_global(self):
        for i in range(12):
            self.step(10.+i/10.)
        self.assertEqual(self.n.search_mode, 'startup')
        self.step(20.)
        self.assertEqual(self.n.search_mode, 'global')

    def test_required_attempts_cover_all_startup_references(self):
        self.frames(list(range(13))+[99], set(range(13)))
        self.h.params.update(startup_min_attempts=1, startup_timeout_s=0.)
        for i in range(7):
            self.step(10.+i)
            self.assertEqual(self.n.search_mode, 'startup')
        self.assertEqual(set(self.seen), set(range(13)))
        self.step(17.)
        self.assertEqual(self.n.search_mode, 'global')

    def test_startup_only_never_falls_back(self):
        self.n.search_strategy = self.n.configured_search_strategy = 'startup_only'
        for i in range(20):
            self.step(10.+i*10.)
        self.assertEqual(self.n.search_mode, 'startup')
        self.assertNotIn(99, self.seen)

    def test_global_immediately_matches_non_startup_reference(self):
        self.n.search_strategy = self.n.configured_search_strategy = self.n.search_mode = 'global'
        self.frames([99, 0, 1], [0, 1])
        self.result(99)
        self.step(10.)
        self.assertIn(99, self.seen)
        self.assertEqual(len(self.h.poses), 1)
        self.assertEqual(self.n.startup_attempts, 0)

    def test_outside_preferred_and_pending_cannot_enter_startup(self):
        scheduler = self.n.xfeat_matcher.scheduler
        scheduler.preferred, scheduler.pending = 99, [99, 0, 1]
        self.result(99, inliers=100)
        self.step(10.)
        self.assertEqual(self.seen, [0, 1])
        self.assertEqual(self.h.poses, [])
        self.assertEqual(self.remembered, [])

    def test_ambiguous_candidates_do_not_promote_preferred(self):
        self.result(0)
        self.result(1, x=3., inliers=29)
        self.step(10.)
        self.assertIn('ambiguous', self.h.states[-1])
        self.assertEqual(self.h.poses, [])
        self.assertEqual(self.remembered, [])
        self.assertIsNone(self.n.xfeat_matcher.scheduler.preferred)

    def test_temporally_inconsistent_candidates_do_not_promote_preferred(self):
        self.h.params['confirmation_samples'] = 2
        self.n.confirmation_history = latency_fixture.deque(maxlen=2)
        self.result(0)
        self.step(10.)
        self.assertTrue(self.h.states[-1].startswith('CANDIDATE:'))
        self.result(0, x=1.)
        self.step(11.)
        self.assertIn('temporal_inconsistent', self.h.states[-1])
        self.assertEqual(self.remembered, [])

    def test_only_fully_confirmed_published_winner_is_remembered(self):
        self.h.params['confirmation_samples'] = 2
        self.n.confirmation_history = latency_fixture.deque(maxlen=2)
        self.result(0)
        self.step(10.)
        self.assertEqual(self.remembered, [])
        self.step(11.)
        self.assertEqual(len(self.h.poses), 1)
        self.assertEqual([s['keyframe'] for s in self.remembered[0]], [0])
        self.assertEqual(self.h.poses[0].header.stamp.sec, 11)

    def test_stale_inference_neither_counts_nor_promotes(self):
        self.result(0)
        self.delay = 6.
        self.step(10.)
        self.assertIn('stale_query', self.h.states[-1])
        self.assertEqual(self.n.startup_attempts, 0)
        self.assertEqual(self.remembered, [])

    def test_no_image_stale_input_and_duplicate_do_not_count(self):
        self.n.processing_callback()
        self.step(20., stamp=10.)
        self.assertEqual(self.n.startup_attempts, 0)
        self.assertEqual(self.seen, [])
        self.step(21.)
        self.step(21.)
        self.assertEqual(self.n.startup_attempts, 1)

    def test_wrong_image_size_does_not_count(self):
        self.n.reference_size = (32, 32)
        self.step(10.)
        self.assertIn('image_size_mismatch', self.h.states[-1])
        self.assertEqual(self.n.startup_attempts, 0)
        self.assertEqual(self.seen, [])

    def test_fallback_clears_confirmation_and_scheduler_state(self):
        self.n.search_started, self.n.startup_attempts = 1., 8
        self.n.confirmation_history.extend([(0., 0., 0.)])
        self.n.confirmation_source = 'xfeat_lighterglue_pnp'
        self.n.last_candidate_time = 19.
        scheduler = self.n.xfeat_matcher.scheduler
        scheduler.preferred, scheduler.pending, scheduler.misses = 1, [1], 2
        self.h.params['confirmation_samples'] = 2
        self.n.confirmation_history = latency_fixture.deque(self.n.confirmation_history, maxlen=2)
        self.result(0)
        self.step(20.)
        self.assertEqual(self.n.search_mode, 'global')
        self.assertTrue(self.h.states[-1].startswith('CANDIDATE:global:1/2'))
        self.assertEqual(self.h.poses, [])
        self.assertIsNone(scheduler.preferred)
        self.assertEqual(scheduler.misses, 0)
        self.assertEqual(self.seen, [0, 1])

    def test_publish_failure_does_not_promote_or_lock_startup(self):
        self.result(0)
        self.n.pose_publisher.publish = lambda msg: (_ for _ in ()).throw(RuntimeError('publish failed'))
        self.step(10.)
        self.assertEqual(self.h.states[-1], 'ERROR:RuntimeError')
        self.assertEqual(self.remembered, [])
        self.assertEqual(self.n.search_strategy, 'startup_then_global')

    def test_explicit_frontend_restart_restores_configured_strategy(self):
        self.result(0)
        self.step(10.)
        self.n.ready_callback(NS(data=True))
        self.n.ready_callback(NS(data=False))
        self.n.initializer_state_callback(NS(data='FAILED:ndt_confirmation_timeout'))
        self.assertTrue(self.n.visual_paused)
        self.n.initializer_state_callback(NS(data='COLLECTING_VISUAL_POSES'))
        self.assertFalse(self.n.visual_paused)
        self.assertEqual(self.n.search_strategy, 'startup_then_global')
        self.assertEqual(self.n.search_mode, 'startup')
        self.assertEqual(self.n.startup_attempts, 0)
        self.assertEqual(list(self.n.confirmation_history), [])
        self.assertEqual(self.n.xfeat_matcher.scheduler.pending, [])
        self.assertIsNone(self.n.xfeat_matcher.scheduler.preferred)


if __name__ == '__main__':
    unittest.main()
