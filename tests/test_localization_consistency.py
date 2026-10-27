import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('consistency', ROOT /
    'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/localization_consistency.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StationaryDriftTests(unittest.TestCase):
    def test_slow_drift_cannot_evade_per_frame_limits(self):
        guard = module.StationaryDriftGuard()
        for step in range(21):
            self.assertIsNone(guard.observe((step*.01, 0, 0), (0, 0, 0)))
        self.assertIn('stationary_map_drift', guard.observe((.21, 0, 0), (0, 0, 0)))

    def test_stationary_noise_does_not_reset_or_trip_anchor(self):
        guard = module.StationaryDriftGuard()
        for step in range(600):
            noise = math.sin(step)*.01
            self.assertIsNone(guard.observe((noise, noise, noise), (noise, noise, noise)))
        self.assertIn('drift', guard.observe((.25, 0, 0), (0, 0, 0)))

    def test_long_moving_route_has_no_fixed_map_leg_anchor(self):
        guard = module.StationaryDriftGuard()
        for step in range(1000):
            x = step*.02
            self.assertIsNone(guard.observe((x*1.05, 0, 0), (x, 0, 0)))

    def test_stationary_yaw_drift_is_rejected(self):
        guard = module.StationaryDriftGuard()
        guard.observe((0, 0, 0), (0, 0, 0))
        self.assertIn('drift', guard.observe((0, 0, .16), (0, 0, 0)))

    def test_real_turn_and_angle_wrap_are_not_drift(self):
        guard = module.StationaryDriftGuard()
        for step in range(400):
            yaw = math.atan2(math.sin(step*.02), math.cos(step*.02))
            self.assertIsNone(guard.observe((0, 0, yaw), (0, 0, yaw)))

    def test_reset_starts_new_stationary_interval(self):
        guard = module.StationaryDriftGuard()
        guard.observe((0, 0, 0), (0, 0, 0))
        guard.reset()
        self.assertIsNone(guard.observe((1, 1, .5), (0, 0, 0)))


class RegistrationReferenceTests(unittest.TestCase):
    def test_default_profile_preserves_existing_pose(self):
        result, q = module.registration_seed((2., 3., .4, .7), (0., 0., 0.), (0., 0., 0.))
        self.assertEqual(result[:3], (2., 3., .4))
        self.assertAlmostEqual(result[3], .7)
        self.assertEqual(q[:2], (0., 0.))

    def test_offset_rotates_with_robot_heading(self):
        result, _ = module.registration_seed((2., 3., 0., math.pi/2),
                                             (.1, -.2, -.7), (0., 0., 0.))
        self.assertAlmostEqual(result[0], 2.2)
        self.assertAlmostEqual(result[1], 3.1)
        self.assertAlmostEqual(result[2], -.7)

    def test_reference_conversion_cancels_factory_preprocessing(self):
        import yaml
        config = yaml.safe_load((ROOT/'navigation/indoor_fusion_20260912_001_area_review/visual_initialization.yaml').read_text())
        params = config['visual_pose_initializer']['ros__parameters']
        pose, q = module.registration_seed((0., 0., 0., 0.),
            params['ndt_seed_reference_xyz'], params['ndt_seed_reference_rpy'])
        pitch = math.radians(18)
        # Transform the live registration origin back through the reference
        # offset; composition must recover the recorded LiDAR->IMU extrinsic.
        x = pose[0]+math.cos(pitch)*.187-math.sin(pitch)*(.4+.34751)
        z = pose[2]+math.sin(pitch)*.187+math.cos(pitch)*(.4+.34751)
        self.assertAlmostEqual(x, -.011, places=7)
        self.assertAlmostEqual(pose[1], -.02329, places=7)
        self.assertAlmostEqual(z, .04412, places=7)
        self.assertAlmostEqual(q[1], -math.sin(pitch/2))
        self.assertAlmostEqual(sum(v*v for v in q), 1.)

    def test_invalid_transform_is_rejected(self):
        with self.assertRaises(ValueError):
            module.registration_seed((0, 0, 0, 0), (float('nan'), 0, 0), (0, 0, 0))
