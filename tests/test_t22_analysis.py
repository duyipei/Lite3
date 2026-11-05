"""Synthetic JSONL and mocked SSH collection tests; no network is accessed."""
import copy
import contextlib
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from t22_fixtures import capture_file, sample_rows, NS_SECOND
from analyze_t22 import analyze, read_capture, stats_ns
from collect_t22 import INVENTORY, collect


class AnalysisTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='t22_analysis_mock_')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def pair(self, **rx_options):
        return [capture_file(self.root, 'leg'), capture_file(self.root, 'voa', **rx_options)]

    def test_matching_stamps_produce_complete_report_with_all_requested_metrics(self):
        result = analyze(self.pair())
        self.assertTrue(result['complete'], result['warnings'])
        self.assertEqual(result['matched_source_stamps'], 1)
        self.assertEqual(result['unmatched_tx'], 0)
        self.assertEqual(result['unmatched_rx'], 0)
        for key in ('T2_enter_to_valid', 'T2_valid_to_T3a', 'cache_to_T3a', 'timer_interval',
                    'deepcopy', 'sampled_publish_call', 'TF_build', 'T3c_to_T3d',
                    'T3d_to_T4a', 'T4b_to_T4c'):
            self.assertEqual(result['metrics'][key]['n'], 1, key)
            self.assertEqual(set(result['metrics'][key]),
                {'n','mean_ms','p50_ms','p95_ms','max_ms','over_20ms','over_50ms','over_100ms','negative_count'})
        self.assertAlmostEqual(result['metrics']['T2_enter_to_valid']['mean_ms'], .001)
        self.assertAlmostEqual(result['metrics']['timer_interval']['mean_ms'], 50.)

    def test_t1_is_explicitly_unmeasured_and_not_zero(self):
        result = analyze(self.pair())['metrics']['T1_to_T2']
        self.assertEqual(result['n'], 0)
        self.assertIsNone(result['mean_ms'])
        self.assertIn('not measured', result['reason'])

    def test_any_missing_role_is_partial_with_warning(self):
        for role in ('leg', 'voa'):
            with self.subTest(role=role):
                result = analyze([capture_file(self.root, role)])
                self.assertFalse(result['complete'])
                self.assertTrue(any('missing role' in v for v in result['warnings']))

    def test_dropped_rows_are_partial(self):
        result = analyze(self.pair(batch=dict(dropped_rows=7)))
        self.assertFalse(result['complete'])
        self.assertEqual(result['dropped_rows'], 7)

    def test_unknown_drop_counter_is_null_instead_of_zero(self):
        result = analyze(self.pair(batch=dict(dropped_rows=None)))
        self.assertFalse(result['complete'])
        self.assertIsNone(result['dropped_rows'])

    def test_recording_not_finished_is_partial(self):
        self.assertFalse(analyze(self.pair(batch=dict(finished=False)))['complete'])

    def test_mismatched_boot_capture_generation_or_window_refuses_merge(self):
        for change in (dict(boot_id='other'), dict(capture_id='other'),
                       dict(enable_id='different'), dict(start_ns=111*NS_SECOND, end_ns=156*NS_SECOND),
                       dict(duration_s=30, end_ns=140*NS_SECOND)):
            with self.subTest(change=change):
                paths = self.pair(metadata=change)
                with self.assertRaisesRegex(ValueError, 'refusing to merge'): analyze(paths)

    def test_duplicate_source_stamps_are_reported_and_excluded(self):
        leg, voa = sample_rows()
        voa.append(copy.deepcopy(voa[0]))
        paths = [capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)]
        result = analyze(paths)
        self.assertFalse(result['complete'])
        self.assertEqual(result['duplicate_source_stamps']['rx'], 1)
        self.assertEqual(result['matched_source_stamps'], 0)

    def test_legitimate_negative_receive_minus_publish_return_is_preserved(self):
        leg, voa = sample_rows()
        voa[0]['t4a_ns'] = leg[2]['t3c_ns']+1000
        paths = [capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)]
        result = analyze(paths)
        self.assertTrue(result['complete'], result['warnings'])
        self.assertEqual(result['metrics']['T3d_to_T4a']['negative_count'], 1)
        self.assertLess(result['metrics']['T3d_to_T4a']['mean_ms'], 0)

    def test_receive_before_publish_entry_is_unreasonable_and_partial(self):
        leg, voa = sample_rows()
        voa[0]['t4a_ns'] = leg[2]['t3c_ns']-1000
        result = analyze([capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)])
        self.assertFalse(result['complete'])
        self.assertTrue(any('receive precedes' in v for v in result['warnings']))

    def test_missing_timestamp_is_not_replaced_with_zero(self):
        leg, voa = sample_rows()
        del leg[2]['t3d_ns']
        result = analyze([capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)])
        self.assertFalse(result['complete'])
        self.assertIsNone(result['metrics']['T3c_to_T3d']['mean_ms'])

    def test_recorder_error_and_truncated_line_make_partial(self):
        paths = self.pair()
        with paths[1].open('a') as handle:
            handle.write(json.dumps(dict(kind='recorder_error', error='disk full'))+'\n')
            handle.write('{"truncated":')
        result = analyze(paths)
        self.assertFalse(result['complete'])
        self.assertEqual(result['recorder_error'], ['disk full'])

    def test_unmatched_source_stamp_and_buffer_exception_are_visible(self):
        leg, voa = sample_rows()
        voa[0]['source_stamp_ns'] += 1
        result = analyze([capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)])
        self.assertEqual((result['unmatched_tx'], result['unmatched_rx']), (1, 1))
        self.assertFalse(result['complete'])
        voa[0]['outcome'] = 'ExtrapolationException'
        result = analyze([capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)])
        self.assertTrue(any('Buffer insertion exception' in v for v in result['warnings']))

    def test_counts_are_cumulative_not_summed_across_batches(self):
        paths = self.pair()
        records = [json.loads(v) for v in paths[1].read_text().splitlines()]
        final = records[1]
        final['counts']['lookup.leg_grid.ExtrapolationException.future'] = 7
        first = dict(final, rows=[], finished=False, written_rows=0, batch_index=1)
        final['batch_index'] = 2
        paths[1].write_text('\n'.join(json.dumps(r) for r in (records[0], first, final))+'\n')
        result = analyze(paths)
        self.assertEqual(result['future_lookup_calls'], 7)

    def test_future_failure_evidence_preserves_actual_grid_age_and_gap(self):
        leg, voa = sample_rows()
        voa.append(dict(kind='lookup_failure_example', stage='leg_grid', direction='future',
                        error_type='ExtrapolationException', grid_stamp_ns=1109064641000,
                        requested_stamp_ns=1109064641000, grid_age_ns=12000000,
                        error='Lookup into the future; latest data is at time 1109.060984',
                        last_set_returned_stamp_ns=1109060984000))
        result = analyze([capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)])
        evidence = result['sampled_failure_examples'][0]
        self.assertAlmostEqual(evidence['shortfall_ms'], 3.657)
        self.assertAlmostEqual(evidence['grid_age_ms'], 12.)

    def test_missing_failure_age_is_null_not_invented(self):
        leg, voa = sample_rows()
        voa.append(dict(kind='lookup_failure_example', error='frame missing'))
        result = analyze([capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)])
        self.assertFalse(result['complete'])
        self.assertIsNone(result['sampled_failure_examples'][0]['grid_age_ms'])

    def test_bad_window_schema_or_missing_batches_is_partial(self):
        for change in (dict(schema_version=True), dict(accepted_ns=110*NS_SECOND), dict(clock='wall time')):
            with self.subTest(change=change):
                self.assertFalse(analyze(self.pair(metadata=change))['complete'])
        path = self.pair()[1]
        path.write_text(path.read_text().splitlines()[0]+'\n')
        self.assertTrue(read_capture(path)['quality']['issues'])

    def test_publish_errors_are_visible_even_if_files_have_final_batches(self):
        leg, voa = sample_rows()
        leg.append(dict(kind='leg_tx_error', stage='sendTransform', error_type='RuntimeError'))
        result = analyze([capture_file(self.root, 'leg', leg), capture_file(self.root, 'voa', voa)])
        self.assertFalse(result['complete'])
        self.assertEqual(result['publish_errors'][0]['stage'], 'sendTransform')

    def test_quantiles_thresholds_and_negative_count(self):
        result = stats_ns([-1000000, 0, 20000000, 21000000, 51000000, 101000000])
        self.assertEqual(result['p50_ms'], 20.5)
        self.assertEqual(result['p95_ms'], 101.)
        self.assertEqual((result['over_20ms'], result['over_50ms'], result['over_100ms']), (3, 2, 1))
        self.assertEqual(result['negative_count'], 1)


class CollectionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='t22_collection_mock_')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.remote = self.root/'mock_remote'
        self.remote.mkdir()
        self.sources = [capture_file(self.remote, role) for role in ('leg', 'voa')]
        self.output = self.root/'desktop_results'
        self.calls = []
        self.corrupt = False

    def runner(self, argv, **options):
        self.calls.append(argv)
        if argv[0] == 'ssh':
            items = []
            for path in self.sources:
                records = [json.loads(v) for v in path.read_text().splitlines()]
                items.append(dict(name=path.name, bytes=path.stat().st_size,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest(), metadata=records[0],
                    finished=records[-1]['finished'], dropped_rows=0, recorder_error=[]))
            return subprocess.CompletedProcess(argv, 0, json.dumps(items), '')
        self.assertEqual(argv[0], 'scp')
        source = self.remote/argv[-2].split(':/tmp/')[1]
        shutil.copyfile(str(source), argv[-1])
        if self.corrupt:
            with Path(argv[-1]).open('a') as handle: handle.write('corruption')
        return subprocess.CompletedProcess(argv, 0, '', '')

    def test_two_files_verified_preserved_and_analyzed(self):
        result = collect('operator-provided-alias', 'test_capture', self.output, runner=self.runner)
        self.assertTrue(result['copy_verified'])
        self.assertTrue(result['analysis_complete'])
        self.assertTrue(all(p.exists() for p in self.sources))
        self.assertEqual(len(self.calls), 3)
        destination = self.output/'test_capture'
        self.assertTrue((destination/'analysis.json').exists())
        self.assertTrue((destination/'collection_manifest.json').exists())
        self.assertFalse(any('ros2' in argument for call in self.calls for argument in call))

    def test_remote_inventory_code_runs_offline_on_isolated_mock_files(self):
        original_glob = Path.glob
        with patch.object(Path, 'glob', lambda path, pattern: original_glob(self.remote, pattern)):
            with patch.object(sys, 'argv', ['mock_inventory','test_capture']):
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    exec(compile(INVENTORY, '<offline inventory>', 'exec'), {})
        results = json.loads(output.getvalue())
        self.assertEqual({r['metadata']['role'] for r in results}, {'leg','voa'})
        for result in results:
            original = self.remote/result['name']
            self.assertEqual(result['sha256'], hashlib.sha256(original.read_bytes()).hexdigest())
        self.assertEqual(sorted(p.name for p in self.remote.iterdir()), sorted(p.name for p in self.sources))
        self.assertFalse(self.calls)

    def test_existing_destination_refused_without_any_connection(self):
        destination = self.output/'test_capture'
        destination.mkdir(parents=True)
        with self.assertRaises(ValueError):
            collect('alias', 'test_capture', self.output, runner=self.runner)
        self.assertFalse(self.calls)

    def test_missing_role_refused_by_default_or_marked_partial_explicitly(self):
        self.sources = self.sources[:1]
        with self.assertRaisesRegex(ValueError, 'both leg and voa'):
            collect('alias', 'test_capture', self.output, runner=self.runner)
        result = collect('alias', 'test_capture', self.output, allow_partial=True, runner=self.runner)
        self.assertFalse(result['analysis_complete'])

    def test_checksum_failure_preserves_partial_files_and_error_record(self):
        self.corrupt = True
        with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
            collect('alias', 'test_capture', self.output, runner=self.runner)
        destination = self.output/'test_capture'
        self.assertTrue((destination/'collection_error.json').exists())
        self.assertTrue(list(destination.glob('*.jsonl')))
        self.assertTrue(all(p.exists() for p in self.sources))

    def test_host_and_id_are_not_shell_commands(self):
        for host in ('', '-oProxyCommand=bad', 'alias;touch /tmp/unwanted', 'alias$(id)'):
            with self.assertRaises(ValueError):
                collect(host, 'test_capture', self.output, runner=self.runner)
        with self.assertRaises(ValueError):
            collect('alias', '../escape', self.output, runner=self.runner)
        self.assertFalse(self.calls)

    def test_unfinished_capture_refused_without_copy(self):
        self.sources[1] = capture_file(self.remote, 'voa', batch=dict(finished=False))
        with self.assertRaisesRegex(ValueError, 'not finished'):
            collect('alias', 'test_capture', self.output, runner=self.runner)
        self.assertEqual(len(self.calls), 1)


if __name__ == '__main__': unittest.main()
