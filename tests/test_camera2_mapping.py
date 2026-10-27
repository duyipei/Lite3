"""Offline command-routing tests. SSH/SCP are mocked; no robot access."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]


class CameraMappingTests(unittest.TestCase):
    def wrapper(self, name, *args, answer='', camera_env=None):
        with tempfile.TemporaryDirectory(prefix='lite3_camera_test_') as directory:
            root = Path(directory)
            shutil.copy2(PROJECT / 'scripts' / name, root / name)
            shutil.copy2(PROJECT / 'scripts/mapping_camera_selection.sh', root)
            stub = root / 'mapping_capture.sh'
            stub.write_text('#!/bin/bash\nprintf "CAMERA=%s ARGS=%s\\n" "${LITE3_MAPPING_CAMERA:-1}" "$*"\n')
            stub.chmod(0o700)
            env = dict(os.environ)
            env.pop('LITE3_MAPPING_CAMERA', None)
            if camera_env is not None:
                env['LITE3_MAPPING_CAMERA'] = camera_env
            return subprocess.run(['bash', str(root / name), *args], env=env,
                                  input=answer, capture_output=True, text=True, timeout=5)

    def test_unified_bridge_entry(self):
        for name in ('mapping_start.sh',):
            result = self.wrapper(name, 'camera2-start')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('ARGS=camera2-start', result.stdout)

    def test_camera2_acquisition_explicit(self):
        result = self.wrapper('mapping_start.sh', 'start', 'outdoor_cam2_test', '2')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('CAMERA=2 ARGS=start outdoor_cam2_test', result.stdout)
        self.assertIn('RAW DATA ONLY', result.stdout)

    def test_camera1_default_preserved(self):
        result = self.wrapper('mapping_start.sh', 'start', 'outdoor_test', answer='\n')
        self.assertEqual(result.returncode, 0)
        self.assertIn('CAMERA=1 ARGS=start outdoor_test', result.stdout)

    def test_invalid_camera_rejected(self):
        result = self.wrapper('mapping_start.sh', 'start', 'outdoor_test', '3')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('ARGS=', result.stdout)

    def test_camera2_preview(self):
        result = self.wrapper('mapping_start.sh', 'video', '2')
        self.assertEqual(result.returncode, 0)
        self.assertIn('CAMERA=2 ARGS=view-video', result.stdout)

    def test_interactive_camera2_unified_entry(self):
        for name in ('mapping_start.sh',):
            result = self.wrapper(name, 'start', 'camera_test', answer=' 2 \n')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('CAMERA=2 ARGS=start camera_test', result.stdout)

    def test_invalid_answer_reprompts(self):
        result = self.wrapper('mapping_start.sh', 'start', 'camera_test', answer='3\n2\n')
        self.assertEqual(result.returncode, 0)
        self.assertIn('输入无效', result.stdout)
        self.assertIn('CAMERA=2 ARGS=', result.stdout)

    def test_cancel_and_eof_do_not_start(self):
        for answer in ('q\n', ''):
            result = self.wrapper('mapping_start.sh', 'start', 'camera_test', answer=answer)
            self.assertNotEqual(result.returncode, 0)
            self.assertNotIn('ARGS=', result.stdout)

    def test_environment_selection_and_cli_precedence(self):
        result = self.wrapper('mapping_start.sh', 'start', 'camera_test', camera_env='2')
        self.assertEqual(result.returncode, 0)
        self.assertIn('CAMERA=2 ARGS=', result.stdout)
        result = self.wrapper('mapping_start.sh', 'start', 'camera_test', '1', camera_env='2')
        self.assertEqual(result.returncode, 0)
        self.assertIn('CAMERA=1 ARGS=', result.stdout)

    def test_auto_run_id_with_selection(self):
        result = self.wrapper('mapping_start.sh', 'start', answer='2\n')
        self.assertEqual(result.returncode, 0)
        self.assertRegex(result.stdout, r'CAMERA=2 ARGS=start mapping_\d{8}_\d{6}')

    def test_old_mapping_entries_removed(self):
        for name in ('outdoor_lidar_visual_mapping.sh', 'indoor_lidar_visual_mapping.sh'):
            self.assertFalse((PROJECT / 'scripts' / name).exists())

    def test_install_and_verify_forwarded(self):
        for args in (('install',), ('verify', 'mapping_test'), ('status',), ('stop',), ('abort',)):
            result = self.wrapper('mapping_start.sh', *args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('ARGS=' + ' '.join(args), result.stdout)
        result = self.wrapper('mapping_start.sh', 'verify')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('ARGS=', result.stdout)

    def bridge(self, fail=False):
        with tempfile.TemporaryDirectory(prefix='lite3_bridge_test_') as directory:
            root = Path(directory)
            for name in ('ssh', 'scp'):
                stub = root / name
                stub.write_text('#!/bin/bash\nprintf "%s %s\\n" "${0##*/}" "$*"\n'
                                + ('exit 73\n' if fail else 'exit 0\n'))
                stub.chmod(0o700)
            key = root / 'fake_key'
            key.touch()
            env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'],
                       LITE3_SSH_KEY=str(key), LITE3_ROBOT_HOST='mock-robot')
            return subprocess.run(['bash', str(PROJECT / 'scripts/mapping_capture.sh'),
                                   'camera2-start'], env=env, text=True,
                                  capture_output=True, timeout=5)

    def test_deploy_then_foreground_bridge(self):
        result = self.bridge()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(sum(line.startswith('scp ') for line in result.stdout.splitlines()), 2)
        self.assertIn('rtsp_camera.py', result.stdout)
        self.assertIn('ssh -t ', result.stdout)
        self.assertIn('ysc@mock-robot', result.stdout)
        self.assertNotIn('sudo', result.stdout)
        self.assertNotIn('date -s', result.stdout)

    def test_connection_failure_stops_deployment(self):
        result = self.bridge(fail=True)
        self.assertEqual(result.returncode, 73)
        self.assertNotIn('scp ', result.stdout)


if __name__ == '__main__':
    unittest.main()
