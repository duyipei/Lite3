"""Run the visual startup script against fake ROS modules; no robot or network."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'navigation/start_visual_localizer.sh'

FAKE_RCLPY = '''import os, time, signal
from sensor_msgs.msg import Image
MODE = os.environ['CAMERA_TEST_MODE']
clock = 0.
time.monotonic = lambda: clock
def init():
    if MODE == 'init_error':
        raise RuntimeError('mock RMW initialization failed')
class Node:
    def create_subscription(self, kind, topic, callback, qos):
        assert topic == '/camera/color/image_raw'
        self.callback = callback
        return object()
    def count_publishers(self, topic):
        return 0 if MODE == 'no_publishers' else 1
    def destroy_node(self):
        print('MOCK_NODE_DESTROYED', flush=True)
def create_node(name):
    assert name == 'patrol_visual_camera_check'
    if MODE == 'noise':
        print('middleware diagnostic before first image', flush=True)
    return Node()
def spin_once(node, timeout_sec):
    global clock
    clock += 10.
    if MODE == 'spin_error':
        raise RuntimeError('mock image subscription failed')
    if MODE not in ('no_frames', 'no_publishers'):
        node.callback(Image())
def shutdown():
    print('MOCK_SHUTDOWN', flush=True)
    if MODE == 'shutdown_hang':
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        time.sleep(100)
'''

FAKE_ROS2 = '''#!/bin/sh
if [ "$1" = node ] && [ "$2" = list ]; then
  exit 0
fi
if [ "$1" = run ]; then
  printf 'VISUAL_STARTED %s\\n' "$*"
  exit 0
fi
exit 7
'''

FAKE_TIMEOUT = '''#!/bin/sh
test "$1" = --kill-after=2s || exit 91
test "$2" = 30s || exit 92
if [ "$CAMERA_TEST_MODE" = outer_timeout ]; then
  exit 124
fi
shift 2
# Shorten only the fake subprocess deadline; production stays at 30+2 s.
exec /usr/bin/timeout --kill-after=0.1s 0.3s "$@"
'''


class VisualCameraStartupTests(unittest.TestCase):
    def run_startup(self, mode):
        with tempfile.TemporaryDirectory(prefix='visual_camera_test_') as directory:
            root = Path(directory)
            (root/'navigation').mkdir()
            (root/'navigation/patrol_env.sh').write_text('true\n')
            (root/'realsense_setup.bash').write_text('true\n')
            database = root/'results/visual_maps/patrol_map_20261031_01/visual_map.db'
            database.parent.mkdir(parents=True)
            database.write_text('mock database; never opened by a real visual node')
            binary = root/'bin'; binary.mkdir()
            for name, content in [('ros2', FAKE_ROS2), ('timeout', FAKE_TIMEOUT)]:
                path = binary/name; path.write_text(content); path.chmod(0o755)
            modules = root/'modules'; modules.mkdir()
            (modules/'rclpy').mkdir()
            (modules/'rclpy/__init__.py').write_text(FAKE_RCLPY)
            (modules/'rclpy/qos.py').write_text(
                'class ReliabilityPolicy: BEST_EFFORT = 1\n'
                'class DurabilityPolicy: VOLATILE = 2\n'
                'class QoSProfile:\n'
                '    def __init__(self, **kw):\n'
                '        assert kw == dict(depth=1, reliability=1, durability=2)\n')
            (modules/'sensor_msgs').mkdir()
            (modules/'sensor_msgs/__init__.py').write_text('')
            (modules/'sensor_msgs/msg.py').write_text(
                'from types import SimpleNamespace as NS\n'
                'class Image:\n'
                '    width=424; height=240\n'
                '    header=NS(frame_id="camera_color_optical_frame")\n')
            source = SCRIPT.read_text().replace(
                'PROJECT=/home/ysc/lite3_patrol_project', 'PROJECT=' + str(root)).replace(
                'source /home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash',
                'source ' + str(root/'realsense_setup.bash'))
            launcher = root/'start.sh'; launcher.write_text(source)
            env = dict(os.environ, CAMERA_TEST_MODE=mode,
                       PATH=str(binary)+':'+os.environ['PATH'],
                       PYTHONPATH=str(modules), PYTHONDONTWRITEBYTECODE='1',
                       LITE3_PATROL_MAP_ID='patrol_map_20261031_01',
                       LITE3_VISUAL_BACKEND='orb')
            env.pop('LITE3_VISUAL_DATABASE', None)
            return subprocess.run(['bash', str(launcher)], env=env, text=True,
                                  capture_output=True, timeout=5)

    def test_real_image_callback_allows_visual_startup(self):
        result = self.run_startup('image')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('RGB Image received: 424x240', result.stdout)
        self.assertIn('MOCK_NODE_DESTROYED', result.stdout)
        self.assertIn('MOCK_SHUTDOWN', result.stdout)
        self.assertIn('VISUAL_STARTED', result.stdout)
        self.assertNotIn('BrokenPipeError', result.stderr)

    def test_diagnostic_text_before_image_cannot_cause_false_no_frames(self):
        result = self.run_startup('noise')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('middleware diagnostic before first image', result.stdout)
        self.assertIn('VISUAL_STARTED', result.stdout)

    def test_publisher_without_frames_still_refuses_startup(self):
        result = self.run_startup('no_frames')
        self.assertEqual(result.returncode, 1)
        self.assertIn('No RGB Image received; publishers=1', result.stderr)
        self.assertNotIn('VISUAL_STARTED', result.stdout)
        self.assertIn('MOCK_NODE_DESTROYED', result.stdout)

    def test_missing_publisher_is_reported(self):
        result = self.run_startup('no_publishers')
        self.assertEqual(result.returncode, 1)
        self.assertIn('No RGB Image received; publishers=0', result.stderr)
        self.assertNotIn('VISUAL_STARTED', result.stdout)

    def test_rmw_errors_are_preserved_and_block_startup(self):
        result = self.run_startup('init_error')
        self.assertEqual(result.returncode, 1)
        self.assertIn('mock RMW initialization failed', result.stderr)
        self.assertNotIn('VISUAL_STARTED', result.stdout)

    def test_subscription_error_is_preserved_and_cleans_up(self):
        result = self.run_startup('spin_error')
        self.assertEqual(result.returncode, 1)
        self.assertIn('mock image subscription failed', result.stderr)
        self.assertIn('MOCK_NODE_DESTROYED', result.stdout)
        self.assertIn('MOCK_SHUTDOWN', result.stdout)
        self.assertNotIn('VISUAL_STARTED', result.stdout)

    def test_outer_timeout_is_not_misreported_as_success(self):
        result = self.run_startup('outer_timeout')
        self.assertEqual(result.returncode, 1)
        self.assertIn('camera check failed (exit 124)', result.stderr)
        self.assertNotIn('VISUAL_STARTED', result.stdout)

    def test_stuck_probe_shutdown_is_bounded_and_blocks_startup(self):
        result = self.run_startup('shutdown_hang')
        self.assertEqual(result.returncode, 1)
        self.assertIn('RGB Image received', result.stdout)
        self.assertIn('camera check failed (exit 137)', result.stderr)
        self.assertNotIn('VISUAL_STARTED', result.stdout)

    def test_visual_node_command_and_parameters_are_unchanged(self):
        before = subprocess.check_output(
            ['git', 'show', 'HEAD:navigation/start_visual_localizer.sh'], cwd=ROOT, text=True)
        marker = 'exec ros2 run lite3_patrol_nav rgbd_visual_localizer'
        self.assertEqual(SCRIPT.read_text().split(marker, 1)[1], before.split(marker, 1)[1])


if __name__ == '__main__':
    unittest.main()
