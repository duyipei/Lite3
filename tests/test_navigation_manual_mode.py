"""Exercise launch sequencing with inert shell commands, never SSH or ROS."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT/'scripts/navigation_start.sh'
TEXT = LAUNCHER.read_text()


def function(name):
    start = TEXT.index(name+'() {')
    return TEXT[start:TEXT.index('\n}\n', start)+3]


class ManualLauncherTests(unittest.TestCase):
    def simulate(self, mode):
        with tempfile.TemporaryDirectory() as tmp:
            preamble = r'''
set -eu
INITIALIZATION_MODE=$1
LITE3_RVIZ_ENABLED=1
LOG_DIR=$2
TRACE=$LOG_DIR/trace.txt
MAP_ID=test_map
VISUAL_MAP_ID=test_visual_map
VISUAL_BACKEND=xfeat_lighterglue
VISUAL_SEARCH_STRATEGY=startup_then_global
DYNAMIC_OBSTACLES=1
GROUND_MODE=map
OBSTACLE_BACKEND=factory_only
NDT_RESOLUTION=1.0
FINE_REGISTRATION=false
DOWNSAMPLE_RESOLUTION=.3
LOCALIZATION_RUNTIME=vendor
USE_VENDOR_VOA=0
SCRIPT_DIR=/never-executed
REMOTE_PROJECT=/never-executed
MAP_DISPLAY_SSH_PID=
RVIZ_SSH_PID=
SSH_BG=(fake_ssh)
SSH=(fake_ssh)
fake_ssh() { printf 'ssh:%s\n' "$*" >> "$TRACE"; }
python3() { printf 'python:%s\n' "$*" >> "$TRACE"; }
sleep() { :; }
kill() { :; }
title() { :; }
die() { echo "$*" >&2; exit 1; }
validate_inputs() { :; }
deploy_runtime() { :; }
static_preflight() { :; }
wait_localization() { wait; }
wait_visual_node() { wait; }
wait_map_display() { wait; }
wait_nav2() { wait; }
wait_goal_bridge() { wait; }
wait_gate_disarmed() { wait; }
show_failure_logs() { :; }
stop_map_display() { printf 'stop_map\n' >> "$TRACE"; }
request_visual_initialization() { printf 'visual_request\n' >> "$TRACE"; }
wait_visual_ready() { printf 'ready:%s\n' "$*" >> "$TRACE"; }
start_rviz() { printf 'rviz\n' >> "$TRACE"; }
stop_visual_frontend_after_initialization() { printf 'stop_visual\n' >> "$TRACE"; }
'''
            script = preamble+'\n'+function('start_initialization_display')+'\n'+function('start_session')+'\nstart_session\nwait\n'
            result = subprocess.run(['bash', '-c', script, 'inert-test', mode, tmp],
                                    stdin=subprocess.DEVNULL, capture_output=True, text=True,
                                    timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            trace = (Path(tmp)/'trace.txt').read_text()
            self.assertEqual((Path(tmp)/'initialization_mode.txt').read_text().strip(), mode)
            return trace

    def test_manual_display_precedes_input_confirmation_and_nav2(self):
        trace = self.simulate('manual')
        self.assertLess(trace.index('start_manual_initialization.sh'), trace.index('rviz\n'))
        self.assertLess(trace.index('rviz\n'), trace.index('ready:0'))
        self.assertLess(trace.index('ready:0'), trace.index('start_nav_isolated.sh'))
        self.assertLess(trace.index('start_nav_isolated.sh'), trace.index('start_gate_disarmed.sh'))
        self.assertNotIn('start_visual_localizer.sh', trace)
        self.assertNotIn('start_visual_initialization.sh', trace)
        self.assertNotIn('visual_request', trace)
        self.assertEqual(trace.count('rviz\n'), 1)

    def test_visual_start_sequence_is_preserved(self):
        trace = self.simulate('visual')
        self.assertLess(trace.index('start_visual_localizer.sh'), trace.index('visual_request'))
        self.assertLess(trace.index('visual_request'), trace.index('ready:0'))
        self.assertLess(trace.index('ready:0'), trace.index('rviz\n'))
        self.assertLess(trace.index('rviz\n'), trace.index('start_nav_isolated.sh'))
        self.assertNotIn('start_manual_initialization.sh', trace)

    def test_manual_mode_requires_rviz_before_any_network_operation(self):
        env = dict(os.environ, LITE3_RVIZ_ENABLED='0')
        result = subprocess.run(['bash', str(LAUNCHER), 'start-manual'], env=env,
                                capture_output=True, text=True, timeout=3)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('手动初始定位需要 RViz', result.stderr)

    def test_invalid_mode_is_rejected_before_any_network_operation(self):
        env = dict(os.environ, LITE3_INITIALIZATION_MODE='invalid')
        result = subprocess.run(['bash', str(LAUNCHER), 'preflight'], env=env,
                                capture_output=True, text=True, timeout=3)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('LITE3_INITIALIZATION_MODE', result.stderr)

    def test_help_lists_both_modes_without_starting_nodes(self):
        result = subprocess.run(['bash', str(LAUNCHER), '--help'],
                                capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 0)
        self.assertIn('start-manual', result.stdout)
        self.assertIn('start-visual', result.stdout)

    def configure_rviz(self, tools):
        code = TEXT.split("<<'PY_MANUAL_RVIZ'\n", 1)[1].split('\nPY_MANUAL_RVIZ', 1)[0]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'display.rviz'
            config = {'Visualization Manager': {'Tools': tools, 'Global Options': {'Fixed Frame': 'map'}}}
            path.write_text(yaml.safe_dump(config))
            result = subprocess.run([sys.executable, '-', str(path)], input=code,
                                    capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 0, result.stderr)
            return yaml.safe_load(path.read_text())

    def test_rviz_pose_tool_is_isolated_and_goal_tool_unchanged(self):
        goal = {'Class': 'rviz_default_plugins/SetGoal', 'Topic': {'Value': '/patrol/goal_request'}}
        cfg = self.configure_rviz([
            {'Class': 'rviz_default_plugins/SetInitialPose', 'Topic': {'Value': '/initialpose'}}, goal])
        tools = cfg['Visualization Manager']['Tools']
        self.assertEqual(tools[0]['Topic']['Value'], '/patrol/manual_initialpose_request')
        self.assertEqual(tools[1], goal)
        self.assertEqual(cfg['Visualization Manager']['Global Options']['Fixed Frame'], 'map')

    def test_rviz_pose_tool_can_be_added_if_missing(self):
        cfg = self.configure_rviz([])
        self.assertEqual(cfg['Visualization Manager']['Tools'][0]['Topic']['Value'],
                         '/patrol/manual_initialpose_request')

    def test_manual_entry_uses_existing_map_limits_and_guard(self):
        entry = (ROOT/'navigation/start_manual_initialization.sh').read_text()
        self.assertIn('--params-file "$CONFIG"', entry)
        self.assertIn('initialization_mode:=manual', entry)
        self.assertIn('visual_pose_initializer', entry)
        self.assertNotIn('ros2 topic pub', entry)
        self.assertNotIn('/patrol/set_armed', entry)


if __name__ == '__main__':
    unittest.main()
