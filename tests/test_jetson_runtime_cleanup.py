"""Check backup-before-delete guarantees on disposable local files only."""
import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('runtime_cleanup', ROOT/'tools/jetson_runtime_cleanup.py')
cleanup = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cleanup)


class CleanupTests(unittest.TestCase):
    def test_navigation_boundary_summaries_are_runtime_assets(self):
        for map_id in cleanup.MAPS:
            for name in ('traversability_summary.json', 'patrol_map_summary.json'):
                with self.subTest(map_id=map_id, name=name):
                    self.assertIsNone(cleanup.reason_for('navigation/'+map_id+'/'+name))

    def fixture(self, root):
        paths = ['logs/old_a.log', 'logs/old_b.log', 'config/sensor.yaml']
        for relative in paths:
            p = root/relative
            p.parent.mkdir(exist_ok=True)
            p.write_text('recovery content: '+relative)
        entries = {}
        for relative in paths[:2]:
            p = root/relative
            entries[relative] = dict(kind='file', size=p.stat().st_size,
                                     mtime_ns=p.stat().st_mtime_ns,
                                     sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                                     reason=cleanup.reason_for(relative))
        plan = root/'plan.json'
        plan.write_text(json.dumps(dict(root=str(root), backup_verified=True,
                                        entries=entries, backup='verified-snapshot.tar.gz',
                                        archive_sha256='a'*64)))
        return plan

    def test_changed_file_prevents_every_deletion(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            plan = self.fixture(root)
            (root/'logs/old_b.log').write_text('new data not in the verified backup')
            with patch.object(cleanup, 'ROOT', root), patch.object(cleanup, 'assert_idle'):
                with self.assertRaisesRegex(RuntimeError, 'changed since backup|differs from backup'):
                    cleanup.apply_plan(plan, root/'report.json')
            self.assertTrue((root/'logs/old_a.log').exists())
            self.assertTrue((root/'logs/old_b.log').exists())

    def test_only_verified_files_removed_and_runtime_file_survives(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            plan = self.fixture(root)
            runtime = root/'patrol_nav_ws/install_merged/bin'
            runtime.mkdir(parents=True)
            # Deleting an archived hard link must not remove the installed data.
            os.link(root/'logs/old_a.log', runtime/'module')
            with patch.object(cleanup, 'ROOT', root), patch.object(cleanup, 'assert_idle'), \
                    contextlib.redirect_stdout(io.StringIO()):
                cleanup.apply_plan(plan, root/'report.json')
            self.assertFalse((root/'logs/old_a.log').exists())
            self.assertFalse((root/'logs/old_b.log').exists())
            self.assertTrue((root/'config/sensor.yaml').exists())
            self.assertTrue((runtime/'module').read_text().startswith('recovery content'))
            self.assertEqual(json.loads((root/'report.json').read_text())['new_broken_symlinks'], [])

    def test_runtime_install_cannot_be_injected_into_cleanup_plan(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            plan = self.fixture(root)
            data = json.loads(plan.read_text())
            data['entries']['patrol_nav_ws/install_merged/module.py'] = dict(
                data['entries']['logs/old_a.log'])
            plan.write_text(json.dumps(data))
            with patch.object(cleanup, 'ROOT', root), patch.object(cleanup, 'assert_idle'):
                with self.assertRaisesRegex(ValueError, 'protected/unclassified'):
                    cleanup.apply_plan(plan, root/'report.json')
            self.assertTrue((root/'logs/old_a.log').exists())

    def test_traversal_and_symlink_parents_never_reach_external_files(self):
        with tempfile.TemporaryDirectory() as name, tempfile.TemporaryDirectory() as outside:
            root = Path(name)
            (root/'escape').symlink_to(outside, target_is_directory=True)
            for path in ('../outside', '/etc/passwd', 'escape/data'):
                with self.subTest(path=path), self.assertRaises(ValueError):
                    cleanup.checked_path(root, path)


if __name__ == '__main__':
    unittest.main()
