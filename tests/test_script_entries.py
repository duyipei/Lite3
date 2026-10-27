"""Compatibility help checks with all network transport blocked."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml

PROJECT = Path(__file__).resolve().parents[1]


class EntryTests(unittest.TestCase):
    def test_navigation_requests_visual_initialization_only_once(self):
        script = r'''
set -- --help
source "$ENTRY_SCRIPT" >/dev/null
trap - EXIT INT TERM HUP
LOG_DIR=$TEST_LOG_DIR
SSH_BG=(true)
validate_inputs() { :; }
deploy_runtime() { :; }
static_preflight() { :; }
stop_visual_frontend_after_initialization() { echo TEST_VISUAL_RELEASE; }
sleep() { :; }
kill() { return 0; }
wait_localization() { return 0; }
wait_visual_node() { return 0; }
request_visual_initialization() { echo TEST_VISUAL_REQUEST; }
wait_visual_ready() {
  echo "TEST_WAIT_READY:$*"
  if [[ ${2:-} == tracking ]]; then return "$TEST_TRACKING_RESULT"; fi
  return 0
}
show_failure_logs() { :; }
wait_map_display() { return 0; }
start_rviz() { :; }
stop_map_display() { :; }
wait_nav2() { return 0; }
wait_goal_bridge() { return 0; }
wait_gate_disarmed() { return 0; }
python3() { :; }
start_session </dev/null
'''
        with tempfile.TemporaryDirectory(prefix='lite3_tracking_test_') as directory:
            for tracking_result in ('0', '1'):
                result = subprocess.run(
                    ['bash', '-c', script], text=True, capture_output=True, timeout=5,
                    env=dict(os.environ, ENTRY_SCRIPT=str(PROJECT/'scripts/navigation_start.sh'),
                             TEST_LOG_DIR=directory, TEST_TRACKING_RESULT=tracking_result,
                             LITE3_DYNAMIC_OBSTACLES='1'))
                self.assertEqual(result.stdout.count('TEST_VISUAL_REQUEST'), 1)
                self.assertIn('TEST_WAIT_READY:10 tracking', result.stdout)
                self.assertEqual(result.stdout.count('TEST_VISUAL_RELEASE'), 1)
                self.assertLess(result.stdout.index('TEST_VISUAL_RELEASE'),
                                result.stdout.index('TEST_WAIT_READY:10 tracking'))
                self.assertEqual(result.returncode, int(tracking_result), result.stderr)
                self.assertEqual('NAVIGATION READY' in result.stdout, tracking_result == '0')

    def test_cleanup_tolerates_already_stopped_children(self):
        script = r'''
set -- --help
source "$ENTRY_SCRIPT" >/dev/null
trap - EXIT INT TERM HUP
disarm() { :; }
stop_remote_runtime() { :; }
stop_map_display() { set -e; }
LOCALIZATION_SSH_PID=2147483647
stop_local_sessions
echo CLEANUP_COMPLETE
'''
        result = subprocess.run(['bash', '-c', script],
                                env=dict(os.environ, ENTRY_SCRIPT=str(PROJECT/'scripts/navigation_start.sh')),
                                text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('CLEANUP_COMPLETE', result.stdout)

    def test_stationary_profile_does_not_change_navigation_defaults(self):
        with tempfile.TemporaryDirectory(prefix='lite3_init_profile_test_') as directory:
            root = Path(directory)
            (root/'navigation/test_map').mkdir(parents=True)
            original = dict(stable_samples=10, ndt_confirmation_samples=15,
                            ndt_max_matching_error=0.15, max_ndt_position_delta_m=0.70)
            (root/'navigation/test_map/visual_initialization.yaml').write_text(
                yaml.safe_dump({'visual_pose_initializer': {'ros__parameters': original}}))
            (root/'bin').mkdir()
            stub = root/'bin/ros2'
            stub.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
            stub.chmod(0o700)
            (root/'navigation/patrol_env.sh').write_text(
                'export PATH=' + str(root/'bin') + ':"$PATH"\n')
            entry = root/'entry.sh'
            entry.write_text((PROJECT/'navigation/start_visual_initialization.sh').read_text()
                             .replace('PROJECT=/home/ysc/lite3_patrol_project',
                                      'PROJECT=' + str(root), 1))
            for test_only in ('0', '1', 'invalid'):
                env = dict(os.environ, LITE3_PATROL_MAP_ID='test_map',
                           LITE3_RELOCALIZATION_TEST_ONLY=test_only)
                result = subprocess.run(['bash', str(entry)], env=env, text=True,
                                        capture_output=True, timeout=5)
                if test_only == 'invalid':
                    self.assertNotEqual(result.returncode, 0)
                    self.assertNotIn('visual_pose_initializer', result.stdout)
                    continue
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('--params-file', result.stdout)
                profile = root/'navigation/test_map/visual_initialization.yaml'
                self.assertIn(str(profile), result.stdout)
                values = yaml.safe_load(profile.read_text())['visual_pose_initializer']['ros__parameters']
                self.assertEqual(values, original)
                self.assertFalse((root/'logs/relocalization_test/test_map.yaml').exists())
                self.assertNotIn('stable_samples:=', result.stdout)
                self.assertNotIn('ndt_confirmation_samples:=', result.stdout)

    def test_relocalization_only_never_starts_navigation(self):
        script = r'''
set -- --help indoor_fusion_20260912_001_area_review
source "$ENTRY_SCRIPT" >/dev/null
trap - EXIT INT TERM HUP
LOG_DIR=$TEST_LOG_DIR
SSH_BG=(true)
validate_inputs() { :; }
deploy_runtime() { :; }
static_preflight() { :; }
sleep() { :; }
kill() { return 0; }
wait_localization() { return 0; }
wait_visual_node() { return 0; }
request_visual_initialization() { return 0; }
wait_visual_ready() { echo "TEST_WAIT_READY:$1"; return "$TEST_READY_RESULT"; }
show_failure_logs() { :; }
wait_map_display() { echo UNEXPECTED_NAVIGATION; return 1; }
start_rviz() { echo UNEXPECTED_NAVIGATION; return 1; }
start_session relocalize
'''
        with tempfile.TemporaryDirectory(prefix='lite3_relocalization_test_') as directory:
            for ready_result in ('0', '1'):
                env = dict(os.environ, ENTRY_SCRIPT=str(PROJECT/'scripts/navigation_start.sh'),
                           TEST_LOG_DIR=directory, TEST_READY_RESULT=ready_result)
                result = subprocess.run(['bash', '-c', script], env=env, text=True,
                                        capture_output=True, timeout=5)
                self.assertIn('TEST_WAIT_READY:10', result.stdout)
                self.assertNotIn('UNEXPECTED_NAVIGATION', result.stdout)
                self.assertNotIn('NAVIGATION READY', result.stdout)
                if ready_result == '0':
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn('RELOCALIZATION PASSED', result.stdout)
                else:
                    self.assertNotEqual(result.returncode, 0)
                    self.assertNotIn('RELOCALIZATION PASSED', result.stdout)

    def test_visual_map_follows_selected_navigation_map(self):
        env = dict(os.environ)
        env.pop('LITE3_VISUAL_MAP_ID', None)
        for navigation, visual in (
            ('indoor_fusion_20260912_001_area_review', 'indoor_fusion_20260912_001'),
            ('outdoor_20260921_003_area_review', 'outdoor_20260921_003'),
            ('site_fusion_20260909_001_area_v4_review', 'site_fusion_20260909_001'),
        ):
            result = subprocess.run(['bash', str(PROJECT/'scripts/navigation_start.sh'),
                                     '--help', navigation], env=env, text=True,
                                    capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('默认：' + visual, result.stdout)

    def test_explicit_visual_map_override_preserved(self):
        env = dict(os.environ, LITE3_VISUAL_MAP_ID='custom_visual_map')
        result = subprocess.run(['bash', str(PROJECT/'scripts/navigation_start.sh'),
                                 '--help', 'outdoor_20260921_003_area_review'],
                                env=env, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('默认：custom_visual_map', result.stdout)

    def test_legacy_help_matches_canonical(self):
        with tempfile.TemporaryDirectory(prefix='lite3_entry_test_') as directory:
            for name in ('ssh', 'scp'):
                stub = Path(directory) / name
                stub.write_text('#!/bin/sh\necho UNEXPECTED_NETWORK >&2\nexit 73\n')
                stub.chmod(0o700)
            env = dict(os.environ, PATH=directory + os.pathsep + os.environ['PATH'])
            for old, new in (('indoor_nav_start.sh', 'navigation_start.sh'),
                             ('round3a_capture.sh', 'mapping_capture.sh')):
                outputs = []
                for name in (old, new):
                    result = subprocess.run(['bash', str(PROJECT / 'scripts' / name), '--help'],
                                            env=env, text=True, capture_output=True, timeout=5)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertNotIn('UNEXPECTED_NETWORK', result.stderr)
                    outputs.append(result.stdout)
                self.assertEqual(*outputs)


if __name__ == '__main__':
    unittest.main()
