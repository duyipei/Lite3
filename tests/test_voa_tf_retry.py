"""T3 deterministic offline tests: no ROS initialization, DDS, or robot access."""
import ast
import copy
import threading
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

import test_factory_voa_mode as factory
import test_voa_costmap_bridge as fixtures
import test_voa_callback_groups as foxy

M = fixtures.M
ROOT = Path(__file__).resolve().parents[1]


class TransformException(Exception):
    pass


class ExtrapolationException(TransformException):
    pass


class LookupException(TransformException):
    pass


class ConnectivityException(TransformException):
    pass


class Clock:
    def __init__(self, now_ns=100000000000):
        self.now_ns = now_ns

    def monotonic(self):
        return self.now_ns/1e9

    def monotonic_ns(self):
        return self.now_ns

    def advance(self, milliseconds):
        self.now_ns += round(milliseconds*1000000)


class Publisher:
    def __init__(self, clock):
        self.clock, self.messages, self.calls = clock, [], 0
        self.error, self.delay_ms = None, 0

    def publish(self, message):
        self.calls += 1
        if self.error:
            raise self.error
        self.messages.append(copy.deepcopy(message))
        self.clock.advance(self.delay_ms)


class RetryTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        timer = patch.object(M, 'time', self.clock)
        timer.start()
        self.addCleanup(timer.stop)
        self.outputs, self.logs, self.queries = {}, [], []
        self.ready_ns = self.clock.now_ns
        self.lookup_hook = None

        def publisher(kind, topic, qos):
            self.outputs[topic] = Publisher(self.clock)
            return self.outputs[topic]

        self.node = NS(create_publisher=publisher,
                       get_logger=lambda: NS(info=self.logs.append))
        self.bridge = self.make_bridge()

    def make_bridge(self, **settings):
        return M.Bridge(self.node, NS(lookup_transform=self.lookup), fixtures.Cloud,
                        fixtures.Field, NS, NS(from_msg=M.stamp_seconds), .1,
                        tf_exceptions=(TransformException,), **settings)

    def lookup(self, target, source, stamp):
        self.queries.append((target, source, stamp))
        if self.lookup_hook:
            return self.lookup_hook(target, source, stamp)
        if self.clock.now_ns < self.ready_ns:
            raise ExtrapolationException('Lookup would require extrapolation into the future')
        return NS(transform=fixtures.transform())

    @property
    def cloud(self):
        return self.outputs[M.TOPIC]

    @property
    def heartbeat(self):
        return self.outputs[M.HEARTBEAT]

    def enqueue(self, age_ms=0, data=None):
        msg = fixtures.grid(data=data, stamp=self.clock.monotonic()-age_ms/1000)
        self.bridge.receive(msg)
        return self.bridge.pending[-1]

    def future(self, milliseconds=10000):
        self.ready_ns = self.clock.now_ns+round(milliseconds*1000000)

    def recover_after(self, milliseconds):
        c = self.enqueue()
        self.future(milliseconds)
        self.bridge.publish()
        self.assertEqual(c.state, 'PENDING_TF')
        self.assertFalse(self.cloud.messages)
        for _ in range(milliseconds//25):
            self.clock.advance(25)
            self.bridge.publish()
        self.assertEqual(c.state, 'PUBLISHED')
        self.assertEqual(self.bridge.diag_counts['retry_recovered'], 1)
        self.assertEqual(len(self.cloud.messages), 1)
        self.assertEqual(len(self.heartbeat.messages), 1)
        self.assertEqual(c.source_stamp_ns, 100000000000)
        self.assertEqual(self.cloud.messages[0].header.stamp, c.message.header.stamp)
        self.assertTrue(all(q[2] == 100. for q in self.queries))

    def test_01_immediate_tf_publishes_once(self):
        c = self.enqueue()
        self.bridge.publish()
        self.assertEqual((c.state, c.attempt_count), ('PUBLISHED', 1))
        self.assertEqual(len(self.cloud.messages), 1)
        self.assertFalse(self.bridge.pending)

    def test_02_tf_recovers_after_25ms(self):
        self.recover_after(25)

    def test_03_tf_recovers_after_50ms(self):
        self.recover_after(50)

    def test_04_tf_recovers_after_100ms(self):
        self.recover_after(100)

    def test_05_tf_after_150ms_is_not_retried(self):
        c = self.enqueue()
        self.future(175)
        self.bridge.publish()
        for _ in range(8):
            self.clock.advance(25)
            self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(self.bridge.diag_counts['tf_wait_timeout'], 1)
        self.assertFalse(self.cloud.messages)
        self.assertFalse(self.bridge.pending)

    def test_06_tf_never_arrives_attempts_and_deadline_are_bounded(self):
        c = self.enqueue()
        self.future()
        self.bridge.publish()
        deadline = c.retry_deadline_monotonic_ns
        for _ in range(30):
            self.clock.advance(25)
            self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(c.retry_deadline_monotonic_ns, deadline)
        self.assertLessEqual(c.attempt_count, 8)
        self.assertEqual(self.bridge.last_published, 0.)

    def test_07_future_then_interpolatable_reuses_original_stamp(self):
        self.recover_after(50)
        out = self.cloud.messages[0]
        self.assertEqual((out.header.stamp.sec, out.header.stamp.nanosec), (100, 0))
        self.assertNotEqual(M.stamp_seconds(out.header.stamp), self.clock.monotonic())

    def assert_permanent(self, exception):
        c = self.enqueue()
        def broken(*args): raise exception
        self.lookup_hook = broken
        self.bridge.publish()
        self.assertEqual(c.state, 'FAILED_PERMANENTLY')
        count = len(self.queries)
        for _ in range(12):
            self.clock.advance(25)
            self.bridge.publish()
        self.assertEqual(len(self.queries), count)
        self.assertFalse(self.heartbeat.messages)
        return c

    def test_08_past_extrapolation_is_permanent_for_this_frame(self):
        self.assert_permanent(ExtrapolationException('extrapolation into the past'))

    def test_09_missing_frame_does_not_retry_forever(self):
        c = self.assert_permanent(LookupException('base_link does not exist'))
        self.assertEqual(c.last_tf_error_type, 'LookupException')

    def test_connectivity_exception_is_classified(self):
        c = self.assert_permanent(ConnectivityException('different TF trees'))
        self.assertIn('connectivity', c.last_tf_error)

    def test_10_ndt_future_does_not_block_valid_anchor_compensation(self):
        self.bridge.registration(foxy.registration(99.9))
        def lookup(target, source, stamp):
            if target == 'map': raise ExtrapolationException('future NDT')
            return NS(transform=fixtures.transform(x=stamp-99.9))
        self.lookup_hook = lookup
        c = self.enqueue()
        self.bridge.publish()
        self.assertEqual(c.state, 'PUBLISHED')
        self.assertEqual(self.bridge.predictions, 1)
        self.assertEqual(self.bridge.diag_counts['retry_scheduled'], 0)
        self.assertEqual([q[2] for q in self.queries], [100., 99.9, 100.])

    def test_leg_future_can_recover_even_if_exact_ndt_history_is_too_old(self):
        self.bridge.registration(foxy.registration(99.9))
        def lookup(target, source, stamp):
            if target == 'map': raise ExtrapolationException('past NDT history')
            if stamp == 100. and self.clock.now_ns < self.ready_ns:
                raise ExtrapolationException('future leg TF')
            return NS(transform=fixtures.transform())
        self.lookup_hook = lookup
        c = self.enqueue()
        self.future(50)
        self.bridge.publish()
        self.assertEqual(c.state, 'PENDING_TF')
        self.clock.advance(50)
        self.bridge.publish()
        self.assertEqual(c.state, 'PUBLISHED')
        self.assertEqual(self.bridge.diag_counts['retry_recovered'], 1)
        self.assertIn(('robot_odom', 'leg_base_link', 100.), self.queries)

    def test_11_fresh_candidate_gets_maximum_150ms_wait(self):
        c = self.enqueue()
        self.future()
        self.bridge.publish()
        self.assertEqual(c.retry_deadline_monotonic_ns-c.first_tf_failure_monotonic_ns, 150000000)

    def test_12_older_source_automatically_shortens_wait_budget(self):
        c = self.enqueue(age_ms=200)
        self.future()
        self.bridge.publish()
        self.assertEqual(c.retry_deadline_monotonic_ns-c.first_tf_failure_monotonic_ns, 50000000)
        self.clock.advance(50)
        before = len(self.queries)
        self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(len(self.queries), before)
        self.assertEqual(self.bridge.diag_counts['retry_source_budget'], 1)

    def test_clock_sampling_preemption_cannot_extend_source_deadline(self):
        self.clock.advance(200)
        c = self.enqueue(age_ms=200)
        error = M.ProjectionUnavailable([('map_grid', ExtrapolationException('future'),
                                          'future', 100200000000)])
        skewed_reads = NS(monotonic=lambda: 100.2, monotonic_ns=lambda: 100230000000)
        with patch.object(M, 'time', skewed_reads):
            self.bridge._defer_candidate(c, error)
        self.assertEqual(c.retry_deadline_monotonic_ns, 100250000000)

    def test_13_no_remaining_processing_budget_does_not_schedule_retry(self):
        c = self.enqueue(age_ms=260)
        self.future()
        self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(self.bridge.diag_counts['retry_scheduled'], 0)
        self.assertFalse(self.bridge.pending)

    def test_14_recent_receive_does_not_make_old_source_fresh(self):
        c = self.enqueue(age_ms=400)
        self.assertEqual(c.first_seen_monotonic_ns, self.clock.now_ns)
        self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertFalse(self.queries)
        self.assertFalse(self.heartbeat.messages)
        self.assertEqual(self.bridge.diag_counts['source_expired'], 1)

    def test_15_original_safety_constants_and_input_age_limits(self):
        self.assertEqual((M.PUBLISH_MAX_AGE, M.MAX_AGE, M.MAX_PREDICTION), (.35, .5, .5))
        self.bridge.receive(fixtures.grid(stamp=99.49))
        self.bridge.receive(fixtures.grid(stamp=100.051))
        self.assertEqual(self.bridge.diag_counts['grid_age_rejected'], 2)
        self.assertFalse(self.bridge.pending)

    def test_16_reserve_prevents_late_retry_from_starting_projection(self):
        c = self.enqueue(age_ms=220)
        self.future(50)
        self.bridge.publish()
        self.assertEqual(c.retry_deadline_monotonic_ns-self.clock.now_ns, 30000000)
        self.clock.advance(50)
        with patch.object(M, 'grid_obstacles', side_effect=AssertionError('late decode')):
            self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertFalse(self.cloud.messages)

    def test_attempt_limit_applies_in_addition_to_wait_limit(self):
        self.bridge = self.make_bridge(tf_retry_max_attempts=2)
        c = self.enqueue()
        self.future()
        self.bridge.publish()
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual((c.state, c.attempt_count), ('EXPIRED', 2))
        self.assertEqual(self.bridge.diag_counts['tf_attempts_exhausted'], 1)

    def test_maximum_wait_starts_at_first_tf_failure_not_end_of_anchor_scan(self):
        self.bridge.registration(foxy.registration(99.9))
        def delayed_failure(*args):
            self.clock.advance(20)
            raise ExtrapolationException('future TF')
        self.lookup_hook = delayed_failure
        c = self.enqueue()
        self.bridge.publish()
        self.assertEqual(c.first_tf_failure_monotonic_ns, 100020000000)
        self.assertEqual(self.clock.now_ns, 100040000000)
        self.assertEqual(c.retry_deadline_monotonic_ns, 100170000000)

    def test_last_permitted_attempt_can_succeed(self):
        self.bridge = self.make_bridge(tf_retry_max_attempts=2)
        self.recover_after(25)

    def test_17_duplicate_source_is_one_candidate_across_retries(self):
        c = self.enqueue()
        self.future()
        for i in range(3):
            self.bridge.receive(c.message)
            self.bridge.publish()
            self.assertEqual(len(self.bridge.pending), 1)
            self.assertIs(self.bridge.pending[0], c)
            self.clock.advance(25)
        self.assertEqual(c.attempt_count, 3)
        self.assertEqual(self.bridge.diag_counts['grid_accepted'], 1)

    def test_18_successful_frame_never_republishes(self):
        c = self.enqueue()
        self.bridge.publish()
        for _ in range(10):
            self.bridge.receive(c.message)
            self.bridge.publish()
            self.clock.advance(25)
        self.assertEqual(self.cloud.calls, 1)
        self.assertEqual(self.heartbeat.calls, 1)

    def test_19_cache_is_sixteen_with_explicit_eviction(self):
        self.clock.advance(300)
        first = None
        for i in range(64):
            msg = fixtures.grid(stamp=100.+i*.004)
            self.bridge.receive(msg)
            if first is None: first = self.bridge.pending[-1]
            self.assertLessEqual(len(self.bridge.pending), 16)
        self.assertEqual(len(self.bridge.pending), 16)
        self.assertEqual(self.bridge.diag_counts['candidate_evicted'], 48)
        self.assertEqual(first.state, 'EXPIRED')

    def test_20_four_candidate_attempts_per_tick_and_no_duplicate_attempt(self):
        self.clock.advance(100)
        for i in range(16): self.bridge.receive(fixtures.grid(stamp=100.+i*.005))
        self.future()
        self.bridge.publish()
        self.assertEqual(len(self.queries), 4)
        self.assertEqual(len({q[2] for q in self.queries}), 4)
        self.assertEqual(sum(c.attempt_count for c in self.bridge.pending), 4)

    def test_21_old_valid_frame_gets_turn_after_newest_four_miss(self):
        for stamp in sorted(99.95-i/30. for i in range(12)):
            self.bridge.receive(fixtures.grid(stamp=stamp))
        def lookup(target, source, stamp):
            if stamp > 99.8: raise ExtrapolationException('future exact TF')
            return NS(transform=fixtures.transform())
        self.lookup_hook = lookup
        self.bridge.publish()
        self.assertFalse(self.cloud.messages)
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual(len(self.cloud.messages), 1)
        self.assertAlmostEqual(self.bridge.last_published, 99.95-5/30.)
        self.assertLessEqual(len(self.queries), 8)

    def test_22_expired_candidates_are_removed_once(self):
        c = self.enqueue()
        self.clock.advance(351)
        self.bridge.publish()
        self.assertFalse(self.bridge.pending)
        self.bridge.publish()
        self.assertEqual(self.bridge.diag_counts['source_expired'], 1)
        self.assertEqual(c.state, 'EXPIRED')

    def test_23_new_success_supersedes_old_pending_frame(self):
        old = self.enqueue()
        self.future()
        self.bridge.publish()
        self.clock.advance(25)
        newer = self.enqueue()
        self.ready_ns = self.clock.now_ns
        self.bridge.publish()
        self.assertEqual(newer.state, 'PUBLISHED')
        self.assertEqual(old.state, 'EXPIRED')
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual(self.cloud.calls, 1)

    def test_24_candidate_attempt_state_is_independent(self):
        old = self.enqueue()
        self.future()
        self.bridge.publish()
        self.clock.advance(25)
        newer = self.enqueue()
        self.bridge.publish()
        self.assertEqual((old.attempt_count, newer.attempt_count), (2, 1))
        self.assertNotEqual(old.first_seen_monotonic_ns, newer.first_seen_monotonic_ns)
        self.assertNotEqual(old.retry_deadline_monotonic_ns, newer.retry_deadline_monotonic_ns)

    def test_25_26_27_tf_gap_does_not_send_empty_cloud_clear_or_heartbeat(self):
        obstacle = np.zeros((4, 6)); obstacle[0, 0] = .3
        self.enqueue(data=obstacle)
        self.bridge.publish()
        remembered = copy.deepcopy(self.cloud.messages[0])
        old_stamp, old_time = self.bridge.last_published, self.bridge.last_publish_time
        self.clock.advance(25)
        c = self.enqueue()
        self.future(50)
        self.bridge.publish()
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual(c.state, 'PENDING_TF')
        self.assertEqual(self.cloud.calls, 1)
        self.assertEqual(self.heartbeat.calls, 1)
        self.assertEqual(self.cloud.messages[0].data, remembered.data)
        self.assertEqual((self.bridge.last_published, self.bridge.last_publish_time), (old_stamp, old_time))

    def test_28_valid_empty_cloud_still_updates_heartbeat(self):
        self.enqueue()
        self.bridge.publish()
        self.assertEqual(self.cloud.messages[0].width, 0)
        self.assertEqual(self.heartbeat.messages[0].stamp, self.cloud.messages[0].header.stamp)

    def test_unknown_grid_cells_still_produce_conservative_points(self):
        data = np.zeros((4, 6)); data[0, 0] = np.nan
        self.enqueue(data=data)
        self.bridge.publish()
        self.assertEqual(self.cloud.messages[0].width, 1)

    def test_29_old_obstacle_is_not_restamped_or_republished_during_gap(self):
        self.test_25_26_27_tf_gap_does_not_send_empty_cloud_clear_or_heartbeat()
        self.assertEqual(M.stamp_seconds(self.cloud.messages[0].header.stamp), 100.)
        self.assertEqual(len(self.cloud.messages), 1)

    def test_30_real_gate_still_latches_and_stops_after_source_expires(self):
        case = factory.FactoryGateTests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        gate = case.gate
        gate.clock_time = 100.
        gate.last_ready_time = gate.last_odom_time = gate.last_lidar_time = 100.
        self.enqueue()
        self.bridge.publish()
        gate.voa_costmap_callback(self.heartbeat.messages[0])
        gate.goal_pending = True
        self.assertTrue(gate.arm_callback(NS(data=True), NS()).success)  # Mock object, no ROS service.
        gate.goal_active_callback(NS(data=True))
        command = factory.Twist(); command.linear.x = .1
        gate.raw_callback(command)
        gate.timer_callback()
        self.assertGreater(gate.outputs['/cmd_vel_corrected'][-1].linear.x, 0.)
        self.clock.advance(25)
        self.enqueue()
        self.future()
        self.bridge.publish()
        self.clock.advance(486)
        self.bridge.publish()
        gate.clock_time = self.clock.monotonic()
        gate.raw_callback(command)
        gate.timer_callback()
        self.assertFalse(gate.armed)
        self.assertEqual(gate.fault, 'voa_costmap_not_ready_or_stale')
        self.assertEqual(gate.outputs['/cmd_vel_corrected'][-1].linear.x, 0.)
        self.enqueue()
        self.ready_ns = self.clock.now_ns
        self.bridge.publish()
        gate.voa_costmap_callback(self.heartbeat.messages[-1])
        gate.timer_callback()
        self.assertFalse(gate.armed)
        self.assertEqual(gate.outputs['/cmd_vel_corrected'][-1].linear.x, 0.)

    def test_31_32_pending_wait_releases_rlock_without_sleep(self):
        c = self.enqueue()
        self.future()
        self.bridge.publish()
        self.assertEqual(c.state, 'PENDING_TF')
        acquired = []
        def other_thread():
            ok = self.bridge.lock.acquire(timeout=.5)
            acquired.append(ok)
            if ok: self.bridge.lock.release()
        thread = threading.Thread(target=other_thread)
        thread.start(); thread.join(1)
        self.assertEqual(acquired, [True])
        self.assertFalse(thread.is_alive())

    def test_33_same_candidate_once_per_timer_invocation(self):
        self.test_20_four_candidate_attempts_per_tick_and_no_duplicate_attempt()

    def test_34_35_partial_heartbeat_failure_never_resends_or_regresses_cloud(self):
        old = self.enqueue()
        self.clock.advance(25)
        newer = self.enqueue()
        error = RuntimeError('heartbeat publish failed')
        self.heartbeat.error = error
        with self.assertRaises(RuntimeError) as caught: self.bridge.publish()
        self.assertIs(caught.exception, error)
        self.assertEqual(newer.state, 'FAILED_PERMANENTLY')
        self.assertEqual(old.state, 'EXPIRED')
        self.assertEqual(self.bridge.last_published, 0.)
        self.heartbeat.error = None
        self.bridge.publish()
        self.assertEqual(self.cloud.calls, 1)
        self.assertEqual(self.bridge.diag_counts['published'], 0)

    def test_36_cloud_publish_failure_does_not_commit_or_retry(self):
        c = self.enqueue()
        error = RuntimeError('cloud publish failed')
        self.cloud.error = error
        with self.assertRaises(RuntimeError) as caught: self.bridge.publish()
        self.assertIs(caught.exception, error)
        self.assertEqual(c.state, 'FAILED_PERMANENTLY')
        self.cloud.error = None
        self.bridge.publish()
        self.assertEqual(self.cloud.calls, 1)
        self.assertEqual(self.heartbeat.calls, 0)
        self.assertEqual(self.bridge.last_published, 0.)

    def test_37_diagnostic_failure_cannot_turn_future_into_permanent_failure(self):
        def broken_probe(*args): raise RuntimeError('recorder unavailable')
        self.bridge._t22 = NS(miss=broken_probe)
        self.recover_after(50)
        self.assertGreater(self.bridge.diag_counts['diagnostic_errors'], 0)

    def test_unexpected_error_with_future_text_is_not_retried(self):
        c = self.assert_permanent(RuntimeError('future bug, not a TF exception'))
        self.assertEqual(c.last_tf_error_type, 'RuntimeError')
        self.assertIn('future bug', c.last_tf_error)

    def test_processing_that_crosses_source_expiry_never_publishes(self):
        c = self.enqueue(age_ms=100)
        decode = M.grid_obstacles
        def slow(*args, **kwargs):
            self.clock.advance(251)
            return decode(*args, **kwargs)
        with patch.object(M, 'grid_obstacles', side_effect=slow): self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(self.cloud.calls, 0)
        self.assertEqual(self.heartbeat.calls, 0)

    def test_publish_call_crossing_source_expiry_does_not_refresh_heartbeat(self):
        c = self.enqueue(age_ms=200)
        self.cloud.delay_ms = 151
        self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(self.cloud.calls, 1)
        self.assertEqual(self.heartbeat.calls, 0)
        self.assertEqual(self.bridge.last_published, 0.)
        self.bridge.publish()
        self.assertEqual(self.cloud.calls, 1)

    def test_tf_returning_after_fixed_wait_deadline_is_not_projected(self):
        c = self.enqueue()
        self.future()
        self.bridge.publish()
        self.clock.advance(125)
        def delayed_success(*args):
            self.clock.advance(26)
            return NS(transform=fixtures.transform())
        self.lookup_hook = delayed_success
        with patch.object(M, 'grid_obstacles', side_effect=AssertionError('late TF used')):
            self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(self.cloud.calls, 0)

    def test_heartbeat_call_crossing_source_expiry_does_not_commit_success(self):
        c = self.enqueue(age_ms=200)
        self.heartbeat.delay_ms = 151
        self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertEqual(self.bridge.last_published, 0.)
        self.assertEqual(self.bridge.diag_counts['published'], 0)
        self.bridge.publish()
        self.assertEqual((self.cloud.calls, self.heartbeat.calls), (1, 1))

    def test_source_stamp_mutation_is_rejected_instead_of_restamped(self):
        c = self.enqueue()
        c.message.header.stamp.nanosec = 1
        self.bridge.publish()
        self.assertEqual(c.state, 'FAILED_PERMANENTLY')
        self.assertFalse(self.cloud.messages)

    def test_zero_wait_configuration_disables_retry_safely(self):
        self.bridge = self.make_bridge(tf_retry_max_wait_ms=0)
        c = self.enqueue()
        self.future()
        self.bridge.publish()
        self.assertEqual(c.state, 'EXPIRED')
        self.assertFalse(self.bridge.pending)

    def test_invalid_retry_configuration_is_rejected(self):
        for settings in [dict(tf_retry_max_wait_ms=-1), dict(tf_retry_max_wait_ms=float('nan')),
                         dict(tf_retry_max_wait_ms=351), dict(tf_retry_processing_reserve_ms=0),
                         dict(tf_retry_processing_reserve_ms=350), dict(tf_retry_max_attempts=0),
                         dict(tf_retry_max_attempts=65), dict(tf_retry_max_attempts=1.5)]:
            with self.subTest(settings=settings), self.assertRaises(ValueError): self.make_bridge(**settings)

    def test_success_at_150ms_exactly_remains_within_maximum_wait(self):
        self.recover_after(150)


class ConfigurationAndExecutorTests(unittest.TestCase):
    def test_costmap_retention_configuration_is_unchanged(self):
        layer = factory.PROFILE.voa_layer()
        self.assertEqual(layer['voxel_decay'], 1.5)
        self.assertEqual(layer['decay_model'], 0)
        self.assertTrue(layer['voa_grid']['marking'])
        self.assertFalse(layer['voa_grid']['clearing'])
        self.assertEqual(layer['voa_grid']['observation_persistence'], 0.)
        self.assertEqual(layer['voa_grid']['expected_update_rate'], .5)

    def test_wait_path_has_no_sleep_wait_future_or_timeout_argument(self):
        tree = ast.parse((ROOT/'scripts/voa_costmap_bridge.py').read_text())
        for call in [n for n in ast.walk(tree) if isinstance(n, ast.Call)]:
            if isinstance(call.func, ast.Attribute):
                self.assertNotIn(call.func.attr, ('sleep','wait_for_transform_async','wait_for_transform'))
                if call.func.attr == 'lookup_transform':
                    self.assertEqual(len(call.args), 3)
                    self.assertFalse(call.keywords)

    def test_foxy_executor_can_take_tf_between_pending_timer_attempts(self):
        def scenario(e):
            e.bridge.buffer.lookup_hook = lambda: (_ for _ in ()).throw(LookupError('future TF'))
            e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'], fixtures.grid(stamp=100.1)))
            e.finish(e.dispatch(e.timer))
            self.assertEqual(e.bridge.pending[0].state, 'PENDING_TF')
            self.assertTrue(e.bridge.lock.acquire(blocking=False))
            e.bridge.lock.release()
            tf = e.dispatch(e.entities['tf'], NS(transforms=[NS(child_frame_id='leg_base_link')]))
            e.finish(tf)
            self.assertEqual(len(e.bridge.buffer.transforms), 1)
            e.bridge.buffer.lookup_hook = None
            e.clock.now += .025
            e.finish(e.dispatch(e.timer))
            self.assertEqual(len(e.node.publishers[M.TOPIC].messages), 1)
            self.assertEqual(e.bridge.diag_counts['retry_recovered'], 1)
            foxy.ConfigurationTests().check_configuration(e)
        foxy.run_main(scenario)


if __name__ == '__main__':
    unittest.main()
