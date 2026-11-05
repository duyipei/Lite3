"""Recorder/control tests: isolated /tmp directories, fake clocks, no ROS."""
import json
import contextlib
import io
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from t22_fixtures import (BOOT, TOKEN, Clock, NS_SECOND, capture_file, header, marker,
                          recorder as R, request, sample_rows)
from t22_capture import CaptureControl


class ProbeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='t22_mock_')
        self.addCleanup(temporary.cleanup)
        self.root, self.clock = Path(temporary.name), Clock()
        self.marker = marker(self.root)
        self.probe = R.Probe('leg', self.root, current_boot=BOOT,
                             clock_ns=self.clock, start_writer=False)
        self.addCleanup(self.close_probe)

    def close_probe(self):
        self.probe.close()
        if self.probe._handle is not None:
            self.probe._handle.close()

    def schedule(self):
        self.clock.advance(1)
        self.req = request(self.root, self.clock)
        self.assertTrue(self.probe.poll_once())
        return self.req

    def activate(self):
        self.schedule()
        self.clock.now = self.req['start_ns']

    def rows(self):
        return list(self.probe._rows)

    def test_default_disabled_has_no_thread_or_result(self):
        (self.root/R.ENABLE).unlink()
        with patch.object(R.threading, 'Thread') as thread:
            self.assertIsNone(R.create_probe('leg', self.root))
            thread.assert_not_called()
        self.assertFalse(list(self.root.glob('*.jsonl')))

    def test_lingering_request_without_enable_does_not_activate(self):
        request(self.root, self.clock)
        (self.root/R.ENABLE).unlink()
        with patch.object(R.threading, 'Thread') as thread:
            self.assertIsNone(R.create_probe('leg', self.root))
            thread.assert_not_called()

    def test_enabled_without_request_waits_without_data(self):
        self.assertTrue(self.probe.poll_once())
        self.probe.emit('ignored', value=1)
        self.assertEqual(self.probe.state, 'waiting')
        self.assertFalse(self.rows())
        self.assertFalse(list(self.root.glob('*.jsonl')))

    def test_scheduled_window_records_only_inside_then_finishes(self):
        self.schedule()
        self.probe.emit('before')
        self.assertFalse(self.rows())
        self.clock.now = self.req['start_ns']
        self.probe.emit('inside', value=7)
        self.assertEqual(self.rows()[0]['kind'], 'inside')
        self.clock.now = self.req['start_ns']+45*NS_SECOND
        self.probe.emit('after')
        self.probe.poll_once()
        path = next(self.root.glob('*.jsonl'))
        records = [json.loads(v) for v in path.read_text().splitlines()]
        self.assertTrue(records[-1]['finished'])
        self.assertEqual([r['kind'] for b in records[1:] for r in b['rows']], ['inside'])
        self.assertFalse(self.probe.active())

    def test_invalid_request_fields_rejected_without_result(self):
        for change in (dict(capture_id='../escape'), dict(start_ns=True), dict(duration_s=True),
                       dict(schema_version=True), dict(enable_id='b'*32), dict(issued_ns=0)):
            with self.subTest(change=change):
                self.clock.advance(1)
                value = request(self.root, self.clock)
                value.update(change)
                R.atomic_json(self.root/R.REQUEST, value)
                self.probe.poll_once()
                self.assertIsNotNone(self.probe.request_error)
                self.assertFalse(self.probe.active())
                self.assertFalse(list(self.root.glob('*.jsonl')))

    def test_duration_out_of_bounds_refused(self):
        for duration in (0, 29, 61, 100000, 45.0):
            with self.subTest(duration=duration):
                value = request(self.root, self.clock, duration=duration)
                with self.assertRaises(ValueError):
                    R.validate_request(value, self.marker, BOOT)

    def test_previous_boot_request_refused(self):
        self.clock.advance(1)
        value = request(self.root, self.clock)
        value['boot_id'] = 'old-boot'
        R.atomic_json(self.root/R.REQUEST, value)
        self.probe.poll_once()
        self.assertIsNotNone(self.probe.request_error)
        self.assertFalse(list(self.root.glob('*.jsonl')))

    def test_preexisting_request_does_not_restart_on_new_node(self):
        request(self.root, self.clock)
        self.clock.advance(1)
        self.assertTrue(self.probe.poll_once())
        self.assertIn('predates', self.probe.request_error)
        self.assertFalse(list(self.root.glob('*.jsonl')))

    def test_duplicate_result_name_safely_fails_without_overwrite(self):
        self.clock.advance(1)
        value = request(self.root, self.clock)
        import os
        existing = self.root/('lite3_t22_test_capture_leg_{}.jsonl'.format(os.getpid()))
        existing.write_text('previous result\n')
        self.assertFalse(self.probe.poll_once())
        self.assertIn('FileExistsError', self.probe.error)
        self.assertEqual(existing.read_text(), 'previous result\n')

    def test_bounded_queue_counts_overflow_without_blocking(self):
        self.activate()
        for i in range(R.CAPACITY+12): self.probe.emit('item', index=i)
        self.assertEqual(len(self.rows()), R.CAPACITY)
        self.assertEqual(self.probe._drop_total(), 12)
        self.probe.poll_once()
        self.assertFalse(self.rows())
        self.probe.emit('item', index=10000)
        self.assertEqual(len(self.rows()), 1)

    def test_recorder_lock_contention_drops_instead_of_waiting(self):
        self.activate()
        with self.probe._lock:
            self.probe.emit('item', index=1)
        self.assertEqual(self.probe._drop_total(), 1)
        self.assertFalse(self.rows())

    def test_callbacks_do_not_perform_json_encoding_or_file_io(self):
        self.activate()
        with patch.object(R.json, 'dumps', side_effect=AssertionError('callback JSON I/O')):
            with patch.object(Path, 'open', side_effect=AssertionError('callback file I/O')):
                self.probe.emit('item', value=1)
                self.probe.tick(self.clock())
        self.assertEqual(len(self.rows()), 2)
        self.assertFalse(self.probe._failed)

    def test_writer_error_disables_recording_without_business_exception(self):
        self.activate()
        self.probe._handle.close()

        class FailedFile:
            def write(self, value): raise OSError('disk full')
            def close(self): pass

        self.probe._handle = FailedFile()
        self.assertFalse(self.probe.poll_once())
        self.probe.emit('still_safe', value=1)
        self.probe.count('still_safe')
        self.assertFalse(self.probe.active())
        status = R.read_json(self.probe.ready_path)
        self.assertIn('disk full', status['recorder_error'])

    def test_clock_exception_is_fail_isolated(self):
        self.activate()
        self.probe.clock_ns = lambda: (_ for _ in ()).throw(RuntimeError('clock error'))
        self.assertFalse(self.probe.active())
        self.probe.emit('ignored')
        self.assertTrue(self.probe._failed)

    def test_cleanup_or_generation_change_stops_diagnostics_only(self):
        for mode in ('remove', 'replace'):
            with self.subTest(mode=mode):
                if mode == 'replace':
                    # This independent probe has not started a capture.
                    marker(self.root)
                    self.probe = R.Probe('leg', self.root, current_boot=BOOT,
                                         clock_ns=self.clock, start_writer=False)
                    changed = dict(self.marker, enable_id='b'*32)
                    R.atomic_json(self.root/R.ENABLE, changed)
                else:
                    self.activate()
                    (self.root/R.ENABLE).unlink()
                self.assertFalse(self.probe.poll_once())
                self.assertEqual(self.probe.state, 'disabled')
                self.assertFalse(self.probe.active())

    def test_failure_categories_count_all_but_examples_bounded_per_stage(self):
        self.activate()
        class ExtrapolationException(Exception): pass
        class LookupException(Exception): pass
        for stage in ('map_grid', 'leg_anchor', 'leg_grid'):
            for i in range(45):
                self.probe.miss(stage, header(self.clock()), header(self.clock()),
                                ExtrapolationException('extrapolation into the future'))
        self.probe.miss('leg_grid', header(self.clock()), header(self.clock()), LookupException('missing frame'))
        self.assertEqual(len(self.rows()), 3*R.MISS_EXAMPLES_PER_SECOND)
        self.assertEqual(self.probe._counts['lookup.leg_grid.ExtrapolationException.future'], 45)
        self.assertEqual(self.probe._counts['lookup.leg_grid.LookupException.other'], 1)

    def test_new_capture_works_after_finished_without_restarting_probes(self):
        self.activate()
        self.probe.emit('first')
        self.clock.now = self.req['start_ns']+45*NS_SECOND
        self.probe.poll_once()
        self.clock.advance(1)
        second = request(self.root, self.clock, name='second')
        self.probe.poll_once()
        self.clock.now = second['start_ns']
        self.probe.emit('second')
        self.clock.now += 45*NS_SECOND
        self.probe.poll_once()
        self.assertEqual(len(list(self.root.glob('*.jsonl'))), 2)
        self.assertEqual(self.probe._counts['first'], 0)
        self.assertEqual(self.probe._counts['second'], 1)

    def test_two_actual_writer_threads_produce_a_matching_complete_mock_capture(self):
        from analyze_t22 import analyze
        probes = [R.Probe(role, self.root, current_boot=BOOT, clock_ns=self.clock)
                  for role in ('leg', 'voa')]
        def shutdown():
            for p in probes: p.close()
            for p in probes: p._thread.join(timeout=3)
        self.addCleanup(shutdown)
        self.clock.advance(1)
        value = request(self.root, self.clock)
        def wait_for(predicate):
            deadline = time.monotonic()+3
            while not predicate():
                if time.monotonic() >= deadline: self.fail('mock writer deadline')
                time.sleep(.01)
        wait_for(lambda: all(p.state == 'scheduled' for p in probes))
        self.clock.now = value['start_ns']
        for p, rows in zip(probes, sample_rows(value['start_ns'])):
            for row in rows:
                p.emit(**row)
        self.clock.now = value['start_ns']+45*NS_SECOND
        wait_for(lambda: all(p.state == 'finished' and p._handle is None for p in probes))
        result = analyze(list(self.root.glob('*.jsonl')))
        self.assertTrue(result['complete'], result['warnings'])


class ControlTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='t22_control_mock_')
        self.addCleanup(temporary.cleanup)
        self.root, self.clock = Path(temporary.name), Clock()
        self.control = CaptureControl(self.root, current_boot=BOOT, clock_ns=self.clock)

    def ready(self):
        value = self.control.enable()['marker']
        for role in ('leg', 'voa'):
            R.atomic_json(self.root/('lite3_t22_ready_'+role+'_1.json'),
                dict(role=role, boot_id=BOOT, enable_id=value['enable_id'], state='waiting',
                     probe_created_ns=self.clock()-1, heartbeat_ns=self.clock(), recorder_error=None))

    def test_enable_is_explicit_and_idempotent(self):
        self.assertFalse(self.control.status()['enabled'])
        first = self.control.enable()
        self.assertTrue(first['enabled'])
        self.assertEqual(self.control.enable()['marker']['enable_id'], first['marker']['enable_id'])
        self.assertFalse(list(self.root.glob('*.jsonl')))

    def test_help_never_instantiates_the_control_or_touches_files(self):
        import t22_capture
        for command in (['help'], ['help','start']):
            with patch.object(t22_capture, 'CaptureControl') as control:
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    self.assertEqual(t22_capture.main(command), 0)
                control.assert_not_called()
                self.assertIn('usage:', output.getvalue())

    def test_start_requires_two_ready_nodes_and_uses_jetson_clock(self):
        self.control.enable()
        with self.assertRaises(ValueError): self.control.start()
        self.ready()
        result = self.control.start(45, capture_id='test_capture')
        self.assertEqual(result['request']['issued_ns'], self.clock())
        self.assertEqual(result['request']['start_ns'], self.clock()+10*NS_SECOND)
        self.assertEqual(self.control.status()['phase'], 'scheduled')
        self.clock.advance(12)
        self.assertEqual(self.control.status()['phase'], 'recording')
        self.clock.advance(45)
        self.assertEqual(self.control.status()['phase'], 'window_ended')

    def test_stale_ready_status_is_not_accepted(self):
        self.ready()
        self.clock.advance(6)
        with self.assertRaises(ValueError): self.control.start()

    def test_invalid_duration_and_id_rejected(self):
        self.ready()
        for duration in (29, 61, True, 45.0):
            with self.assertRaises(ValueError): self.control.start(duration)
        with self.assertRaises(ValueError): self.control.start(capture_id='../other')

    def test_cleanup_refuses_active_capture_and_preserves_results(self):
        self.ready()
        self.control.start(capture_id='test_capture')
        result = capture_file(self.root, 'leg')
        for advance in (0, 11):
            self.clock.advance(advance)
            with self.assertRaises(ValueError): self.control.cleanup()
            with self.assertRaises(ValueError): self.control.start()
        self.assertTrue(self.control.cleanup(force=True)['results_preserved'])
        self.assertTrue(result.exists())
        self.assertTrue(list(self.root.glob('lite3_t22_ready_*.json')))
        self.assertFalse((self.root/R.ENABLE).exists())
        self.assertFalse((self.root/R.REQUEST).exists())

    def test_completed_cleanup_preserves_both_results(self):
        self.ready()
        self.control.start(capture_id='test_capture')
        files = [capture_file(self.root, role, metadata=dict(enable_id=self.control.marker()['enable_id']))
                 for role in ('leg', 'voa')]
        self.clock.advance(56)
        self.control.cleanup()
        self.assertTrue(all(p.exists() for p in files))

    def test_cleanup_waits_for_final_writer_flush_after_window_end(self):
        self.ready()
        self.control.start(capture_id='test_capture')
        self.clock.advance(56)
        with self.assertRaisesRegex(ValueError, 'final files are not finished'):
            self.control.cleanup()
        self.assertTrue((self.root/R.REQUEST).exists())

    def test_old_boot_or_malformed_request_needs_explicit_force_cleanup(self):
        self.ready()
        value = request(self.root, self.clock)
        value['boot_id'] = 'old'
        R.atomic_json(self.root/R.REQUEST, value)
        with self.assertRaises(ValueError): self.control.cleanup()
        self.assertTrue(self.control.status()['errors'])
        self.control.cleanup(force=True)

    def test_existing_capture_id_is_not_reused(self):
        self.ready()
        capture_file(self.root, 'leg')
        with self.assertRaisesRegex(ValueError, 'already has results'):
            self.control.start(capture_id='test_capture')

    def test_status_checks_both_roles_and_matching_window(self):
        self.ready()
        capture_file(self.root, 'leg')
        self.assertFalse(self.control.status('test_capture')['both_finished'])
        capture_file(self.root, 'voa')
        self.assertTrue(self.control.status('test_capture')['both_finished'])
        capture_file(self.root, 'voa', metadata=dict(boot_id='other-boot'))
        self.assertFalse(self.control.status('test_capture')['both_finished'])


if __name__ == '__main__': unittest.main()
