"""Archive recovery/integrity checks; never connects to the robot."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('project_archive', ROOT/'tools/jetson_project_archive.py')
archive = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(archive)


class ArchiveTests(unittest.TestCase):
    def make_snapshot(self, directory):
        source = directory/'project'
        source.mkdir()
        (source/'data.bin').write_bytes(bytes(range(256))*1000)
        (source/'outside-link').symlink_to('/etc/hostname')
        os.link(source/'data.bin', source/'hardlink.bin')
        output = directory/'snapshot.tar.gz'
        encoded = io.BytesIO()
        with patch.object(archive, 'REMOTE_ROOT', source.resolve()), \
                patch.object(archive.sys, 'stdout', SimpleNamespace(buffer=encoded)), \
                contextlib.redirect_stderr(io.StringIO()):
            archive.snapshot(source)
        output.write_bytes(encoded.getvalue())
        return source, output

    def test_complete_data_and_links_survive_verified_snapshot(self):
        with tempfile.TemporaryDirectory() as name:
            source, output = self.make_snapshot(Path(name))
            with contextlib.redirect_stdout(io.StringIO()):
                archive.verify(SimpleNamespace(archive=output))
            report = json.loads((output.parent/'verification.json').read_text())
            self.assertTrue(report['verified'])
            entries = json.loads((output.parent/'source_manifest.json').read_text())['entries']
            self.assertEqual(entries['project/outside-link']['kind'], 'symlink')
            self.assertEqual(entries['project/outside-link']['linkname'], '/etc/hostname')
            self.assertEqual(entries['project/hardlink.bin']['kind'], 'hardlink')
            self.assertEqual(report['source_bytes'], (source/'data.bin').stat().st_size)

    def test_corrupt_backup_never_gets_verified_marker(self):
        with tempfile.TemporaryDirectory() as name:
            _, output = self.make_snapshot(Path(name))
            data = bytearray(output.read_bytes())
            data[-8] ^= 1  # gzip CRC, independently of the per-file hashes.
            output.write_bytes(data)
            with self.assertRaises(OSError):
                archive.verify(SimpleNamespace(archive=output))
            self.assertFalse((output.parent/'verification.json').exists())

    def test_existing_backup_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as name:
            output = Path(name)/'snapshot.tar.gz'
            output.write_bytes(b'prior recovery data')
            with patch.object(archive.subprocess, 'Popen') as popen:
                with self.assertRaises(FileExistsError):
                    archive.receive(SimpleNamespace(archive=output))
                popen.assert_not_called()
            self.assertEqual(output.read_bytes(), b'prior recovery data')


if __name__ == '__main__':
    unittest.main()
