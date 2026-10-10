#!/usr/bin/env python3
"""Offline regression checks only: no ROS, SSH, publishers or robot motion."""
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'turn_runner', ROOT/'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/patrol_runner.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
CONFIG = ROOT/'navigation/outdoor_20260921_003_area_review/nav2_safe.yaml'


def valid_speed(v, w, cfg):
    # Foxy XYThetaIterator::isValidSpeed joint minimum check (not a clamp).
    return not (v*v + 1e-5 < cfg['min_speed_xy']**2
                and abs(w) + 1e-5 < cfg['min_speed_theta'])


class TurningTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load(CONFIG.read_text())
        self.dwb = self.config['controller_server']['ros__parameters']['FollowPath']
        self.target = dict(x=19.6, y=1.9859, yaw=math.pi/2)

    def test_reject_recorded_tiny_stationary_turns(self):
        for v in [0., .015]:
            for w in [-.0266666667, 0., .0266666667]:
                self.assertFalse(valid_speed(v, w, self.dwb))

    def test_keep_pure_turns_both_directions_and_forward(self):
        for w in [-.4, -.10666667, .10666667, .4]:
            self.assertTrue(valid_speed(0., w, self.dwb))
        self.assertTrue(valid_speed(.15, 0., self.dwb))

    def test_remove_variable_sample_count_cost_bias(self):
        for critic in ['PathAlign', 'PathDist', 'GoalAlign', 'GoalDist']:
            self.assertEqual(self.dwb[critic+'.aggregation_type'], 'last')
        # Same per-pose cost, different trajectory sampling counts: sum alone
        # penalizes the faster sample; last does not. Not a Nav2 simulation.
        slow, fast = [4.]*3, [4.]*12
        self.assertLess(sum(slow), sum(fast))
        self.assertEqual(slow[-1], fast[-1])
        self.assertGreater(self.dwb['GoalAlign.forward_point_distance'], .1)

    def test_preserve_collision_and_speed_limits(self):
        self.assertIn('ObstacleFootprint', self.dwb['critics'])
        self.assertIn('Oscillation', self.dwb['critics'])
        self.assertIn('RotateToGoal', self.dwb['critics'])
        self.assertEqual(self.dwb['min_vel_x'], 0.)
        self.assertEqual(self.dwb['max_vel_x'], .15)
        self.assertEqual(self.dwb['max_vel_theta'], .4)
        local = self.config['local_costmap']['local_costmap']['ros__parameters']
        self.assertIn('static_layer', local['plugins'])
        self.assertTrue(local['stvl_obstacle_layer']['enabled'])
        self.assertEqual(local['footprint_padding'], .05)

    def test_generator_overrides_vendor_sum(self):
        with tempfile.TemporaryDirectory(prefix='lite3_turn_test_') as directory:
            src, dst = Path(directory)/'vendor.yaml', Path(directory)/'out.yaml'
            for critic in ['PathAlign', 'PathDist', 'GoalAlign', 'GoalDist']:
                self.dwb[critic+'.aggregation_type'] = 'sum'
            src.write_text(yaml.safe_dump(self.config))
            subprocess.run([sys.executable, str(ROOT/'scripts/make_safe_nav2_config.py'),
                            str(src), str(CONFIG.parent/'patrol_map.yaml'), str(dst)],
                           check=True, capture_output=True)
            generated = yaml.safe_load(dst.read_text())['controller_server']['ros__parameters']['FollowPath']
            expected = yaml.safe_load(CONFIG.read_text())['controller_server']['ros__parameters']['FollowPath']
            for key in ['min_speed_xy', 'min_speed_theta', 'PathAlign.aggregation_type',
                        'PathDist.aggregation_type', 'GoalAlign.aggregation_type',
                        'GoalDist.aggregation_type', 'PathAlign.forward_point_distance',
                        'GoalAlign.forward_point_distance']:
                self.assertEqual(generated[key], expected[key])

    def test_wrong_turn_trace_stops(self):
        watch = runner.TurnProgressWatchdog()
        # Mirrors wp5's clockwise drift away from a goal on the left.
        for t in range(15):
            self.assertIsNone(watch.update(t, (18.964, -.253, math.radians(-8.7-2*t)),
                                           (0., -.0266667), self.target))
        self.assertIn('turn_no_progress:path_bearing', watch.update(
            15, (18.964, -.253, math.radians(-38.7)), (0., -.0266667), self.target))

    def test_productive_turn_does_not_trip(self):
        watch = runner.TurnProgressWatchdog()
        for t in range(20):
            self.assertIsNone(watch.update(t, (18.964, -.253, math.radians(-8.7+4*t)),
                                           (0., .1066667), self.target))

    def test_wraparound_uses_shortest_angle(self):
        watch = runner.TurnProgressWatchdog()
        target = dict(x=0., y=0., yaw=math.radians(-179))
        for t in range(25):
            self.assertIsNone(watch.update(t, (0., 0., math.radians(179)), (0., .1), target))

    def test_stuck_final_yaw_stops(self):
        watch = runner.TurnProgressWatchdog()
        pose = (self.target['x'], self.target['y'], 0.)
        self.assertIsNone(watch.update(0., pose, (0., .1), self.target))
        self.assertIn('final_yaw', watch.update(15., pose, (0., .1), self.target))

    def test_translating_detour_not_judged_by_bearing(self):
        watch = runner.TurnProgressWatchdog()
        for t in range(25):
            self.assertIsNone(watch.update(t, (18., -.3, -1.), (.15, -.1), self.target))

    def test_progress_and_phase_reset(self):
        watch = runner.TurnProgressWatchdog()
        self.assertIsNone(watch.update(0., (19.6, 0., 0.), (0., .1), self.target))
        self.assertIsNone(watch.update(14., (19.6, .2, 0.), (.03, .1), self.target))
        self.assertIsNone(watch.update(20., (19.6, .2, 0.), (0., .1), self.target))
        self.assertIsNone(watch.update(29., (19.6, 1.9, 0.), (0., .1), self.target))


if __name__ == '__main__':
    unittest.main()
