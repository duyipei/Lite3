import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('voa_camera_tf',
    Path(__file__).resolve().parents[1] / 'scripts/voa_camera_tf.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CameraMountTests(unittest.TestCase):
    def test_equivalent_quaternion_sign(self):
        self.assertTrue(module.matches_mount(module.XYZ, module.QUAT))
        self.assertTrue(module.matches_mount(module.XYZ, tuple(-v for v in module.QUAT)))

    def test_wrong_mount_rejected(self):
        self.assertFalse(module.matches_mount((0, 0, 0), module.QUAT))
        self.assertFalse(module.matches_mount(module.XYZ, (0, 0, 0, 1)))

    def test_invalid_rotation_rejected(self):
        for q in ((0, 0, 0, 0), (float('nan'), 0, 0, 1), (0, 0, 0, 2)):
            self.assertFalse(module.matches_mount(module.XYZ, q))


if __name__ == '__main__':
    unittest.main()
