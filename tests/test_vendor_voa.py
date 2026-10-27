"""No ROS/hardware: correction protocol must fail closed."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('voa_protocol', ROOT /
    'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/voa_protocol.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VoaTests(unittest.TestCase):
    def setUp(self):
        self.voa = module.VoaExchange()

    def test_no_response_no_motion(self):
        self.assertFalse(self.voa.healthy(10))
        self.assertEqual(self.voa.corrected((.1, 0, 0), 10), (0, 0, 0))

    def test_valid_correction(self):
        key = self.voa.issue((.1, 0, 0), 10)
        self.assertTrue(self.voa.accept(key, (.06, 0, .2), 10.05))
        self.assertEqual(self.voa.corrected((.1, 0, 0), 10.1), (.06, 0, .2))

    def test_replay_rejected(self):
        key = self.voa.issue((.1, 0, 0), 10)
        self.assertTrue(self.voa.accept(key, (.06, 0, .2), 10.05))
        self.assertFalse(self.voa.accept(key, (.06, 0, .2), 10.06))

    def test_prior_session_rejected(self):
        key = module.VoaExchange().issue((.1, 0, 0), 10)
        self.assertFalse(self.voa.accept(key, (.06, 0, 0), 10.01))

    def test_stale_at_source_not_refreshed_by_arrival(self):
        key = self.voa.issue((.1, 0, 0), 10)
        self.assertFalse(self.voa.accept(key, (.06, 0, 0), 10.3))

    def test_sensor_response_expiry(self):
        key = self.voa.issue((.1, 0, 0), 10)
        self.voa.accept(key, (.06, 0, 0), 10.2)
        self.assertFalse(self.voa.healthy(10.26))

    def test_changed_command_cannot_use_old_correction(self):
        key = self.voa.issue((.1, 0, 0), 10)
        self.voa.accept(key, (.06, 0, 0), 10.02)
        self.assertEqual(self.voa.corrected((0, 0, .3), 10.03), (0, 0, 0))

    def test_zero_input_cannot_generate_motion(self):
        key = self.voa.issue((0, 0, 0), 10)
        self.assertFalse(self.voa.accept(key, (.06, 0, 0), 10.02))
        self.assertFalse(self.voa.healthy(10.03))

    def test_invalid_or_excess_velocity_not_clamped(self):
        for v in ((.2, 0, 0), (0, 0, .5), (0, .01, 0), (-.01, 0, 0), (float('nan'), 0, 0)):
            key = self.voa.issue((.1, 0, 0), 10)
            self.assertFalse(self.voa.accept(key, v, 10.02))

    def test_goal_reset_discards_pending_but_keeps_recent_health(self):
        key = self.voa.issue((0, 0, 0), 10)
        self.voa.accept(key, (0, 0, 0), 10.02)
        old = self.voa.issue((.1, 0, 0), 10.03)
        self.voa.reset()
        self.assertTrue(self.voa.healthy(10.04))
        self.assertFalse(self.voa.accept(old, (.1, 0, 0), 10.05))
        self.assertEqual(self.voa.corrected((.1, 0, 0), 10.06), (0, 0, 0))

    def test_out_of_order_rejected(self):
        first = self.voa.issue((.1, 0, 0), 10)
        second = self.voa.issue((.1, 0, 0), 10.01)
        self.assertTrue(self.voa.accept(second, (0, 0, 0), 10.02))
        self.assertFalse(self.voa.accept(first, (.1, 0, 0), 10.03))

    def test_reset_never_reuses_a_previous_goal_motion_correction(self):
        moving = self.voa.issue((.1, 0, .2), 10.)
        probe = self.voa.issue((0, 0, 0), 10.02)
        self.voa.reset()
        self.assertFalse(self.voa.accept(moving, (.1, 0, .2), 10.03))
        self.assertTrue(self.voa.accept(probe, (0, 0, 0), 10.04))
        self.assertTrue(self.voa.healthy(10.2))
        self.assertEqual(self.voa.corrected((.1, 0, .2), 10.2), (0, 0, 0))
        self.assertFalse(self.voa.healthy(10.28))

    def test_request_queue_bounded_by_time(self):
        for i in range(1000):
            self.voa.issue((0, 0, 0), i / 20.)
        self.assertLessEqual(len(self.voa.pending), 7)


if __name__ == '__main__':
    unittest.main()
