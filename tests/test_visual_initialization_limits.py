"""Check V2's actual initializer and launch limits without ROS or a robot."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import yaml
import test_visual_initializer_frames as initializer_fixture


ROOT = Path(__file__).resolve().parents[1]


class V2InitializationTests(unittest.TestCase):
    def setUp(self):
        self.h = initializer_fixture.InitializerTests()
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.n = self.h.node
        config = ROOT/'navigation/patrol_map_20261031_01_area_v2_review/visual_initialization.yaml'
        self.n.params.update(yaml.safe_load(config.read_text())['visual_pose_initializer']['ros__parameters'])
        self.n.candidate = (0., 0., 0., 0.)
        self.n.candidate_time = 10.
        self.n.state = 'WAITING_NDT:test'
        self.n.latest_leg_pose = (0., 0., 0.)

    def scan(self, stamp, x, yaw=0., good=True):
        n = self.n
        n.now = float(stamp)
        n.last_leg_time = n.now
        n.status_callback(NS(header=NS(stamp=NS(sec=stamp, nanosec=0)),
                             has_converged=good, matching_error=.01, inlier_fraction=.8))
        message = initializer_fixture.PoseMessage()
        message.header.frame_id = 'map'
        message.header.stamp.sec = stamp
        message.pose.pose.position.x = x
        message.pose.pose.orientation.z = math.sin(yaw/2.)
        message.pose.pose.orientation.w = math.cos(yaw/2.)
        n.odom_callback(message)

    def confirm(self, x, yaw=0., good=True):
        for stamp in (20, 21, 22):
            self.scan(stamp, x, yaw, good)

    def test_outer_and_inner_translation_limits_match_without_changing_runtime_limits(self):
        tree = ast.parse((ROOT/'patrol_nav_ws/src/lite3_patrol_nav/launch/patrol_navigation.launch.py').read_text())
        values = {}
        for item in ast.walk(tree):
            if not isinstance(item, ast.Dict):
                continue
            for key, value in zip(item.keys, item.values):
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                    values[key.value] = value.value
        self.assertEqual(self.n.params['max_ndt_position_delta_m'], .75)
        self.assertEqual(values['auto_reanchor_max_translation'], .75)
        self.assertEqual(values['manual_pose_translation_gate'], .50)
        self.assertEqual(values['auto_reanchor_samples'], 15)
        self.assertEqual(values['auto_reanchor_max_yaw'], .26179939)
        self.assertEqual(values['auto_reanchor_max_fitness'], .15)
        self.assertEqual(self.n.params['max_ndt_yaw_delta_rad'], .27)
        self.assertEqual(self.n.stationary_guard.position_limit, .20)
        self.assertTrue(self.n.params['stationary_drift_guard_enabled'])

    def test_0746m_requires_three_distinct_good_scans_then_confirms(self):
        self.scan(20, .746)
        self.scan(20, .746)
        self.scan(21, .746)
        self.assertFalse(self.n.state.startswith('READY:'))
        self.scan(22, .746)
        self.assertTrue(self.n.state.startswith('READY:'))

    def test_exact_075m_boundary_confirms(self):
        self.confirm(.75)
        self.assertTrue(self.n.state.startswith('READY:'))

    def test_over_075m_remains_rejected(self):
        self.confirm(.750001)
        self.assertTrue(self.n.state.startswith('WAITING_NDT:delta='))
        self.assertEqual(self.n.ndt_good_count, 0)

    def test_yaw_limit_remains_enforced(self):
        self.confirm(.1, yaw=.271)
        self.assertFalse(self.n.state.startswith('READY:'))

    def test_bad_ndt_quality_remains_rejected(self):
        self.confirm(.746, good=False)
        self.assertFalse(self.n.state.startswith('READY:'))
        self.assertEqual(self.n.ndt_good_count, 0)

    def test_outer_reference_remains_fixed_seed_even_when_leg_pose_changes(self):
        self.n.latest_leg_pose = (1., 0., 0.)
        self.confirm(1.)
        self.assertFalse(self.n.state.startswith('READY:'))
        self.assertIn('delta=1.000m', self.n.state)

    def test_stationary_drift_protection_still_revokes_ready(self):
        self.confirm(.746)
        self.assertTrue(self.n.state.startswith('READY:'))
        self.scan(23, .947)
        self.assertTrue(self.n.state.startswith('FAILED:stationary_map_drift'))
        self.assertFalse(self.n.outputs['/patrol/localization_ready'][-1].data)


if __name__ == '__main__':
    unittest.main()
