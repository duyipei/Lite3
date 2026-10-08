"""Exercise shell confirmation using a fake SSH; NEVER connects to hardware."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]


class ConfirmationTests(unittest.TestCase):
    def run_prompt(self, answer):
        with tempfile.TemporaryDirectory(prefix='lite3_prompt_test_') as directory:
            # Block every SSH/SCP attempt even if the confirmation regresses.
            for name in ('ssh', 'scp'):
                executable = Path(directory) / name
                executable.write_text('#!/bin/sh\necho TEST_TRANSPORT_BLOCKED >&2\nexit 73\n')
                executable.chmod(0o700)
            env = dict(os.environ, PATH=directory+os.pathsep+os.environ['PATH'])
            return subprocess.run(['bash', 'scripts/patrol_route.sh', 'run'],
                                  cwd=PROJECT, env=env, input=answer, text=True,
                                  capture_output=True, timeout=10)

    def test_uppercase_explicit_confirmation(self):
        result = self.run_prompt('ARM\n')
        self.assertEqual(result.returncode, 73)
        self.assertIn('[CONFIRMED]', result.stdout)

    def test_lowercase_whitespace(self):
        result = self.run_prompt('  arm  \n')
        self.assertEqual(result.returncode, 73)

    def test_invalid_reprompts_without_connecting(self):
        result = self.run_prompt('yes\n\nN\n')
        self.assertEqual(result.returncode, 1)
        self.assertIn('[WAIT]', result.stdout)
        self.assertIn('[CANCELED]', result.stdout)
        self.assertNotIn('TEST_TRANSPORT_BLOCKED', result.stderr)

    def test_eof_cancels_without_connecting(self):
        result = self.run_prompt('')
        self.assertEqual(result.returncode, 1)
        self.assertIn('[CANCELED]', result.stderr)
        self.assertNotIn('TEST_TRANSPORT_BLOCKED', result.stderr)

    def test_corrected_input_can_confirm(self):
        result = self.run_prompt('typo\nArM\n')
        self.assertEqual(result.returncode, 73)
        self.assertIn('[WAIT]', result.stdout)


if __name__ == '__main__':
    unittest.main()
