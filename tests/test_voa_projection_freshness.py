"""VOA projection regressions using fake messages, TF and monotonic time only."""
import ast
import copy
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
import types
import unittest
from unittest.mock import patch

import numpy as np

import test_factory_voa_mode as factory
import test_voa_costmap_bridge as fixtures


ROOT = Path(__file__).resolve().parents[1]
GATE = 'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/velocity_gate.py'
BASELINE = '9b116957dd92d074c0f2b1b61ac72fba511fd9ac'
M = fixtures.M


class TfUnavailable(Exception):
    pass


def grid(stamp, data=None):
    return fixtures.grid(data=data, stamp=round(stamp, 9))


def anchor(stamp):
    return NS(header=NS(frame_id='map', stamp=grid(stamp).header.stamp),
              child_frame_id='ndt_registration',
              pose=NS(pose=NS(position=NS(x=1., y=2., z=.3),
                              orientation=NS(x=0., y=0., z=0., w=1.))))


class ProjectionFreshnessTests(unittest.TestCase):
    def setUp(self):
        self.outputs = {}
        self.logs = []
        self.queries = []
        self.tf_until = 100.2

        def publisher(kind, topic, qos):
            self.outputs[topic] = []
            return NS(publish=lambda msg: self.outputs[topic].append(copy.deepcopy(msg)))

        node = NS(create_publisher=publisher,
                  get_logger=lambda: NS(info=self.logs.append))
        self.bridge = M.Bridge(node, NS(lookup_transform=self.lookup),
                               fixtures.Cloud, fixtures.Field, NS,
                               NS(from_msg=M.stamp_seconds), .1,
                               tf_exceptions=(TfUnavailable,))
        clock = patch.object(M.time, 'monotonic', return_value=100.2)
        self.now = clock.start()
        self.addCleanup(clock.stop)
        ns_clock = patch.object(M.time, 'monotonic_ns', side_effect=lambda: round(self.now.return_value*1e9))
        ns_clock.start()
        self.addCleanup(ns_clock.stop)

    def lookup(self, target, source, stamp):
        self.queries.append((target, stamp))
        if target == 'map' and stamp > self.tf_until:
            raise TfUnavailable('future exact TF')
        return NS(transform=fixtures.transform(x=1.))

    def clouds(self):
        return self.outputs[M.TOPIC]

    def test_deferred_fresh_frame_after_newest_four_tf_misses(self):
        self.now.return_value = 100.
        self.tf_until = 99.8
        for stamp in sorted(99.95-i/30. for i in range(12)):
            self.bridge.receive(grid(stamp))
        self.bridge.publish()
        self.assertEqual(len(self.queries), M.MAX_PENDING_TRIES)
        self.assertFalse(self.clouds())
        self.bridge.publish()
        self.assertTrue(self.clouds())
        self.assertAlmostEqual(self.bridge.last_published, 99.95-5/30., places=8)
        self.assertLessEqual(len(self.queries), 2*M.MAX_PENDING_TRIES)
        self.assertEqual(self.bridge.retry_before, None)

    def test_newest_complete_frame_has_priority(self):
        for stamp in (100., 100.05, 100.1):
            self.bridge.receive(grid(stamp))
        self.bridge.publish()
        self.assertEqual(self.queries, [('map', 100.1)])
        self.assertAlmostEqual(M.stamp_seconds(self.clouds()[0].header.stamp), 100.1)

    def test_all_cached_frames_expired_without_tf_work(self):
        self.bridge.receive(grid(100.))
        self.now.return_value = 100.36
        self.bridge.publish()
        self.assertFalse(self.clouds())
        self.assertFalse(self.queries)
        self.assertEqual(self.bridge.diag_counts['candidate_expired_checks'], 1)

    def test_publish_age_boundary_remains_point_three_five_seconds(self):
        self.bridge.receive(grid(100.))
        self.now.return_value = 100.35
        self.bridge.publish()
        self.assertEqual(len(self.clouds()), 1)
        self.now.return_value = 100.36
        self.bridge.receive(grid(100.001))
        self.bridge.publish()
        self.assertEqual(len(self.clouds()), 1)

    def test_projection_math_thresholds_and_registration_are_unchanged(self):
        baseline = subprocess.check_output(
            ['git', 'show', BASELINE + ':scripts/voa_costmap_bridge.py'], cwd=ROOT, text=True)
        current = (ROOT / 'scripts/voa_costmap_bridge.py').read_text()

        def definitions(source):
            tree = ast.parse(source)
            result = {node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)}
            bridge = next(node for node in tree.body
                          if isinstance(node, ast.ClassDef) and node.name == 'Bridge')
            result['registration'] = next(node for node in bridge.body
                                          if isinstance(node, ast.FunctionDef)
                                          and node.name == 'registration')
            result.update({node.targets[0].id: node.value for node in tree.body
                           if isinstance(node, ast.Assign)
                           and isinstance(node.targets[0], ast.Name)})
            return result

        old, new = definitions(baseline), definitions(current)
        for name in ('stamp_seconds', 'fresh', 'grid_obstacles', 'transform_matrix',
                     'compensated_pose', 'project_to_map', 'registration',
                     'MAX_AGE', 'MAX_PREDICTION', 'PUBLISH_MAX_AGE',
                     'MAX_PENDING_TRIES', 'TOPIC', 'HEARTBEAT'):
            self.assertEqual(ast.dump(new[name]), ast.dump(old[name]), name)

    def test_repeated_and_reversed_stamps_cannot_republish(self):
        for stamp in (100.1, 100.1, 100.05):
            self.bridge.receive(grid(stamp))
        self.bridge.publish()
        self.bridge.publish()
        self.assertEqual(len(self.clouds()), 1)
        self.assertEqual(self.bridge.diag_counts['grid_received'], 3)
        self.assertEqual(self.bridge.diag_counts['grid_accepted'], 1)
        self.assertEqual(self.bridge.diag_counts['grid_duplicate'], 1)
        self.assertEqual(self.bridge.diag_counts['grid_out_of_order'], 1)

    def test_cache_capacity_remains_sixteen(self):
        for index in range(20):
            self.bridge.receive(grid(100.+index*.005))
        self.assertEqual(len(self.bridge.pending), 16)
        self.bridge.publish()
        self.assertAlmostEqual(self.bridge.last_published, 100.095)
        self.assertEqual(len(self.queries), 1)

    def test_fast_input_and_delayed_tf_do_not_starve_older_frames(self):
        next_source = 99.95
        for tick in range(60):
            now = 100.+tick*.025
            self.now.return_value = now
            self.tf_until = now-.2
            while next_source <= now-.05+1e-8:
                self.bridge.receive(grid(next_source))
                next_source += 1/30.
            before = len(self.queries)
            published_before = len(self.clouds())
            self.bridge.publish()
            self.assertLessEqual(len(self.queries)-before, M.MAX_PENDING_TRIES)
            self.assertLessEqual(len(self.bridge.pending), 16)
            if len(self.clouds()) > published_before:
                self.assertLessEqual(now-self.bridge.last_published, M.PUBLISH_MAX_AGE)
            if self.clouds():
                self.assertLessEqual(now-self.bridge.last_published, M.MAX_AGE)
        stamps = [M.stamp_seconds(msg.header.stamp) for msg in self.clouds()]
        self.assertGreater(len(stamps), 8)
        self.assertTrue(all(new > old for old, new in zip(stamps, stamps[1:])))

    def prepare_anchors(self, stamps=(100., 100.1), missing=()):
        self.now.return_value = 100.3
        for stamp in stamps:
            self.bridge.registration(anchor(stamp))

        def lookup(target, source, stamp):
            self.queries.append((target, stamp))
            if target == 'map' or any(abs(stamp-value) < 1e-7 for value in missing):
                raise TfUnavailable('anchor or exact TF absent')
            return NS(transform=fixtures.transform())

        self.bridge.buffer.lookup_transform = lookup
        self.bridge.receive(grid(100.2))

    def test_newest_anchor_valid(self):
        self.prepare_anchors()
        self.bridge.publish()
        self.assertEqual(self.queries,
                         [('map', 100.2), ('robot_odom', 100.1), ('robot_odom', 100.2)])
        self.assertEqual(self.bridge.predictions, 1)
        self.assertEqual(len(self.clouds()), 1)

    def test_latest_anchor_tf_missing_uses_older_valid_anchor(self):
        self.prepare_anchors(missing=(100.1,))
        self.bridge.publish()
        self.assertIn(('robot_odom', 100.), self.queries)
        self.assertEqual(self.bridge.diag_counts['anchor_tf_misses'], 1)
        self.assertEqual(len(self.clouds()), 1)
        self.assertAlmostEqual(self.bridge.last_published, 100.2)

    def test_multiple_anchor_failures_then_valid_anchor(self):
        self.prepare_anchors(stamps=(100., 100.1, 100.15), missing=(100.1, 100.15))
        self.bridge.publish()
        self.assertEqual(self.bridge.diag_counts['anchor_tf_misses'], 2)
        self.assertEqual(len(self.clouds()), 1)

    def test_all_anchors_unavailable_emit_no_health(self):
        self.prepare_anchors(missing=(100., 100.1))
        self.bridge.publish()
        self.assertFalse(self.clouds())
        self.assertFalse(self.outputs[M.HEARTBEAT])
        self.assertEqual(self.bridge.projection_failure, 'anchors_tf_unavailable')
        self.assertEqual(self.bridge.diag_counts['anchors_tf_unavailable'], 1)

    def test_anchor_time_limit_is_unchanged(self):
        self.prepare_anchors(stamps=(100.,))
        self.now.return_value = 100.8
        self.bridge.receive(grid(100.7))
        self.bridge.publish()
        self.assertFalse(self.clouds())
        self.assertEqual(self.bridge.projection_failure, 'no_eligible_anchor')

    def test_motion_limits_fail_without_swallowing_validation_errors(self):
        for value, kwargs in ((.51, {'x': .51}), (.36, {'yaw': .36})):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    M.compensated_pose(fixtures.transform(), fixtures.transform(),
                                       fixtures.transform(**kwargs))

    def test_invalid_compensation_does_not_fall_back_past_math_validation(self):
        self.prepare_anchors()
        for kwargs in ({'x': .51}, {'yaw': .36}):
            with self.subTest(kwargs=kwargs):
                self.queries.clear()

                def lookup(target, source, stamp):
                    self.queries.append((target, stamp))
                    if target == 'map':
                        raise TfUnavailable('exact TF pending')
                    return NS(transform=fixtures.transform(**kwargs if stamp == 100.2 else {}))

                self.bridge.buffer.lookup_transform = lookup
                with self.assertRaises(ValueError):
                    self.bridge.projection_transform(grid(100.2))
                self.assertNotIn(('robot_odom', 100.), self.queries)
                self.assertFalse(self.clouds())

    def test_unexpected_tf_error_is_not_caught_as_anchor_unavailability(self):
        self.prepare_anchors()

        def lookup(target, source, stamp):
            if target == 'map':
                raise TfUnavailable('exact TF pending')
            raise RuntimeError('unexpected lookup bug')

        self.bridge.buffer.lookup_transform = lookup
        with self.assertRaisesRegex(RuntimeError, 'unexpected lookup bug'):
            self.bridge.projection_transform(grid(100.2))

    def test_exact_tf_never_uses_anchor_fallback(self):
        self.bridge.registration(anchor(100.))
        self.bridge.receive(grid(100.1))
        self.bridge.publish()
        self.assertEqual(self.queries, [('map', 100.1)])
        self.assertEqual(self.bridge.predictions, 0)

    def test_projection_tf_queries_have_a_fixed_upper_bound(self):
        self.now.return_value = 100.1
        for index in range(12):
            self.bridge.registration(anchor(99.8+index*.005))
        for index in range(16):
            self.bridge.receive(grid(100.+index*.005))

        def lookup(target, source, stamp):
            self.queries.append((target, stamp))
            if target == 'map' or stamp >= 100.:
                raise TfUnavailable('missing grid-time TF')
            return NS(transform=fixtures.transform())

        self.bridge.buffer.lookup_transform = lookup
        self.bridge.publish()
        self.assertEqual(len(self.queries), M.MAX_PENDING_TRIES*(1+12+1))
        for index in range(M.MAX_PENDING_TRIES):
            self.assertEqual(self.queries.count(('robot_odom', 100.+(15-index)*.005)), 1)
        self.assertFalse(self.clouds())

    def test_processing_that_crosses_deadline_does_not_publish(self):
        self.bridge.receive(grid(100.1))
        decode = M.grid_obstacles

        def slow_decode(*args, **kwargs):
            result = decode(*args, **kwargs)
            self.now.return_value = 100.46
            return result

        with patch.object(M, 'grid_obstacles', side_effect=slow_decode):
            self.bridge.publish()
        self.assertFalse(self.clouds())
        self.assertFalse(self.outputs[M.HEARTBEAT])
        self.assertEqual(self.bridge.diag_counts['processing_expired'], 1)
        self.assertEqual(self.bridge.last_published, 0.)

    def test_receive_age_limits_and_original_header_are_unchanged(self):
        for stamp in (99.69, 100.26):
            self.bridge.receive(grid(stamp))
        self.assertEqual(self.bridge.diag_counts['grid_age_rejected'], 2)
        message = grid(100.1)
        original = copy.deepcopy(message.header)
        self.bridge.receive(message)
        self.bridge.publish()
        self.assertEqual(self.clouds()[0].header.stamp, original.stamp)
        self.assertEqual(message.header, original)

    def test_invalid_grid_stamp_is_diagnosed_without_changing_exception(self):
        message = grid(100.)
        message.header.stamp.nanosec = 1000000000
        with self.assertRaises(ValueError):
            self.bridge.receive(message)
        self.assertEqual(self.bridge.diag_counts['grid_invalid_stamp'], 1)
        self.assertEqual(self.bridge.last_input_result, 'grid_invalid_stamp')
        self.assertFalse(self.bridge.pending)

    def test_empty_and_unknown_frames_keep_existing_semantics(self):
        self.bridge.receive(grid(100.))
        self.bridge.publish()
        self.assertEqual(self.clouds()[-1].width, 0)
        self.assertEqual(len(self.outputs[M.HEARTBEAT]), 1)
        data = np.zeros((4, 6)); data[0, 0] = np.nan
        self.bridge.receive(grid(100.1, data))
        self.bridge.publish()
        self.assertEqual(self.clouds()[-1].width, 1)
        self.assertEqual(len(self.outputs[M.HEARTBEAT]), 2)

    def test_source_expiry_is_logged_even_after_recent_publication(self):
        self.now.return_value = 100.34
        self.bridge.receive(grid(100.))
        self.bridge.publish()
        self.now.return_value = 100.51
        self.bridge.publish()
        warnings = [s for s in self.logs if 'heartbeat_gap ' in s]
        self.assertEqual(len(warnings), 1)
        details = json.loads(warnings[-1].split('heartbeat_gap ', 1)[1])
        self.assertAlmostEqual(details['source_age_s'], .51)
        self.assertAlmostEqual(details['publish_gap_s'], .17)
        self.assertEqual(details['clock'], 'monotonic')
        for index in range(10):
            self.now.return_value = 100.52+index*.01
            self.bridge.publish()
        self.assertEqual(sum('heartbeat_gap ' in s for s in self.logs), 1)

    def test_current_failure_is_not_a_previous_tick_failure(self):
        self.tf_until = 99.
        self.bridge.receive(grid(100.1))
        self.bridge.publish()
        self.assertIn('no_eligible_anchor', self.bridge.last_failure)
        self.tf_until = 100.1
        self.bridge.publish()
        self.assertEqual(self.bridge.last_failure, 'none')
        self.bridge.publish()
        self.assertEqual(self.bridge.last_failure, 'none')

    def test_process_and_publish_durations_are_separate(self):
        self.bridge.receive(grid(100.1))
        decode = M.grid_obstacles

        def timed_decode(*args, **kwargs):
            self.now.return_value += .02
            return decode(*args, **kwargs)

        publish = self.bridge.publisher.publish

        def timed_publish(message):
            self.now.return_value += .01
            publish(message)

        self.bridge.publisher.publish = timed_publish
        with patch.object(M, 'grid_obstacles', side_effect=timed_decode):
            self.bridge.publish()
        self.assertAlmostEqual(self.bridge.last_process_s, .02)
        self.assertAlmostEqual(self.bridge.last_publish_s, .01)
        self.assertAlmostEqual(self.bridge.last_publish_time, 100.23)


class GateHeartbeatDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline_source = subprocess.check_output(
            ['git', 'show', BASELINE + ':' + GATE], cwd=ROOT, text=True)

    def setUp(self):
        # Reuse the existing fake ROS modules; never initialize a ROS context.
        self.fixture = factory.FactoryGateTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.gate = self.fixture.gate
        self.logs = []
        self.gate.get_logger = lambda: NS(info=self.logs.append, warn=self.logs.append,
                                         error=self.logs.append)

    @staticmethod
    def message(stamp, frame='map'):
        return NS(frame_id=frame, stamp=grid(stamp).header.stamp)

    @staticmethod
    def configure(gate):
        gate.now_monotonic = lambda: gate.clock_time
        gate.localization_ready = True
        gate.last_ready_time = gate.last_odom_time = gate.last_lidar_time = 10.
        gate.require_voa_costmap = True
        gate.counts.update({'/patrol/voa/costmap_heartbeat': 1,
                            '/patrol/voa/costmap_obstacles': 1})
        return gate

    def baseline_gate(self):
        module = types.ModuleType('factory_gate_test.baseline_velocity_gate')
        module.__package__ = 'factory_gate_test'
        exec(compile(self.baseline_source, '<git-baseline-velocity-gate>', 'exec'),
             module.__dict__)
        return self.configure(module.VelocityGate())

    def diagnostic(self):
        entries = [line for line in self.logs if 'stale heartbeat ' in line]
        self.assertTrue(entries)
        return json.loads(entries[-1].split('stale heartbeat ', 1)[1])

    def test_uninitialized_heartbeat_is_explicit_and_throttled(self):
        for _ in range(10):
            self.assertEqual(self.gate.sensor_health(),
                             (False, 'voa_costmap_not_ready_or_stale'))
        details = self.diagnostic()
        self.assertFalse(details['initialized'])
        self.assertIsNone(details['source_stamp'])
        self.assertIsNone(details['source_age_s'])
        self.assertIsNone(details['accepted_at'])
        self.assertEqual(len(self.logs), 1)
        self.gate.clock_time += 1.01
        self.gate.sensor_health()
        self.assertEqual(len(self.logs), 2)

    def test_last_source_and_receive_ages_use_monotonic_time(self):
        self.gate.clock_time = 10.2
        self.gate.voa_costmap_callback(self.message(10.))
        self.gate.get_clock = lambda: (_ for _ in ()).throw(AssertionError('ROS clock used'))
        self.gate.clock_time = 10.51
        self.assertFalse(self.gate.sensor_health()[0])
        details = self.diagnostic()
        self.assertTrue(details['initialized'])
        self.assertEqual(details['source_stamp'], 10.)
        self.assertAlmostEqual(details['source_age_s'], .51)
        self.assertAlmostEqual(details['accepted_at'], 10.2)
        self.assertAlmostEqual(details['accepted_receive_age_s'], .31)
        self.assertEqual(details['clock'], 'monotonic')

    def test_first_armed_failure_is_not_hidden_by_startup_log_throttle(self):
        self.gate.sensor_health()  # Logs uninitialized at t=10.
        self.gate.clock_time = 10.01
        self.gate.voa_costmap_callback(self.message(10.01))
        self.gate.goal_pending = True
        self.assertTrue(self.gate.arm_callback(NS(data=True), NS()).success)
        self.gate.clock_time = 10.52
        self.gate.sensor_health()
        details = self.diagnostic()
        self.assertTrue(details['initialized'])
        self.assertAlmostEqual(details['source_age_s'], .51)
        self.assertEqual(sum('stale heartbeat' in s for s in self.logs), 2)

    def test_rejections_are_classified_without_refreshing_valid_source(self):
        self.gate.clock_time = 10.2
        self.gate.voa_costmap_callback(self.message(10.))
        cases = [
            ('invalid_frame', self.message(10.1, 'odom')),
            ('invalid_stamp', NS(frame_id='map', stamp=NS(sec=-1, nanosec=0))),
            ('invalid_stamp', NS(frame_id='map', stamp=NS(sec=10, nanosec=1000000000))),
            ('future', self.message(10.26)),
            ('stale', self.message(9.69)),
            ('duplicate', self.message(10.)),
            ('out_of_order', self.message(9.99)),
        ]
        for reason, message in cases:
            with self.subTest(reason=reason):
                self.gate.voa_costmap_callback(message)
                self.assertEqual(self.gate.voa_heartbeat_last_rejection, reason)
                self.assertEqual(self.gate.last_voa_costmap_time, 10.)
                self.assertEqual(self.gate.voa_heartbeat_last_accepted_at, 10.2)
                self.assertTrue(self.gate.sensor_health()[0])
        self.assertEqual(sum(self.gate.voa_heartbeat_rejected.values()), len(cases))
        self.gate.clock_time = 10.51
        self.gate.sensor_health()
        details = self.diagnostic()
        self.assertEqual(details['received'], 8)
        self.assertEqual(details['rejected']['invalid_stamp'], 2)
        self.assertEqual(details['last_rejection'], 'out_of_order')

    def test_high_rate_accepted_heartbeats_do_not_add_per_frame_warnings(self):
        for index in range(100):
            self.gate.clock_time = 10.+index*.001
            self.gate.voa_costmap_callback(self.message(self.gate.clock_time))
            self.assertTrue(self.gate.sensor_health()[0])
        self.assertEqual(self.gate.voa_heartbeat_received, 100)
        self.assertFalse(any('stale heartbeat' in line for line in self.logs))

    def test_callback_acceptance_matches_baseline_at_age_boundaries(self):
        for age in (-.051, -.05, -.049, 0., .499, .5, .500001):
            with self.subTest(age=age):
                old = self.baseline_gate()
                new = self.configure(self.fixture.module.VelocityGate())
                old.clock_time = new.clock_time = 10.+age
                message = self.message(10.)
                old.voa_costmap_callback(message)
                new.voa_costmap_callback(message)
                self.assertEqual(new.last_voa_costmap_time, old.last_voa_costmap_time)
                self.assertEqual(new.sensor_health(), old.sensor_health())

    def test_rejection_sequence_matches_baseline(self):
        old = self.baseline_gate()
        new = self.gate
        cases = [self.message(10.), self.message(10.), self.message(9.99),
                 self.message(10.1), self.message(10.26), self.message(9.69),
                 self.message(10.15, 'odom'),
                 NS(frame_id='map', stamp=NS(sec=-1, nanosec=0)),
                 NS(frame_id='map', stamp=NS(sec=10, nanosec=1000000000))]
        old.clock_time = new.clock_time = 10.2
        for message in cases:
            old.voa_costmap_callback(message)
            new.voa_costmap_callback(message)
            self.assertEqual(new.last_voa_costmap_time, old.last_voa_costmap_time)
            self.assertEqual(new.sensor_health(), old.sensor_health())

    def test_zero_output_latch_and_no_automatic_rearm_match_baseline(self):
        old = self.baseline_gate()
        new = self.gate
        for gate in (old, new):
            gate.voa_costmap_callback(self.message(10.))
            gate.goal_pending = True
            self.assertTrue(gate.arm_callback(NS(data=True), NS()).success)
            gate.goal_active_callback(NS(data=True))
            command = factory.Twist(); command.linear.x = .1
            gate.raw_callback(command)
            gate.timer_callback()
            gate.clock_time = 10.51
            gate.last_ready_time = gate.last_odom_time = gate.last_lidar_time = 10.51
            gate.timer_callback()
            gate.voa_costmap_callback(self.message(10.51))
            gate.timer_callback()
            self.assertFalse(gate.armed)
            self.assertEqual(gate.fault, 'voa_costmap_not_ready_or_stale')
            self.assertFalse(gate.arm_callback(NS(data=True), NS()).success)
        for topic in ('/cmd_vel_corrected', '/patrol/armed', '/patrol/gate_state'):
            self.assertEqual([vars(msg) for msg in new.outputs[topic]],
                             [vars(msg) for msg in old.outputs[topic]])

    def test_safety_control_methods_and_health_return_paths_are_unchanged(self):
        def methods(source):
            node = next(n for n in ast.parse(source).body
                        if isinstance(n, ast.ClassDef) and n.name == 'VelocityGate')
            return {n.name: n for n in node.body if isinstance(n, ast.FunctionDef)}

        old = methods(self.baseline_source)
        new = methods((ROOT / GATE).read_text())
        for name, method in old.items():
            if name not in ('__init__', 'voa_costmap_callback', 'sensor_health'):
                self.assertEqual(ast.dump(new[name]), ast.dump(method), name)
        # The health method differs only by one diagnostic call in its existing
        # stale branch. All predicates, their order and return values match.
        class RemoveDiagnostic(ast.NodeTransformer):
            def visit_Expr(self, node):
                if (isinstance(node.value, ast.Call)
                        and isinstance(node.value.func, ast.Attribute)
                        and node.value.func.attr == '_log_voa_costmap_stale'):
                    return None
                return node

        health = RemoveDiagnostic().visit(new['sensor_health'])
        self.assertEqual(ast.dump(health), ast.dump(old['sensor_health']))
        original_check = next(n.test for n in old['voa_costmap_callback'].body
                              if isinstance(n, ast.If))
        current_check = next(n.test for n in new['voa_costmap_callback'].body
                             if isinstance(n, ast.If))
        self.assertEqual(ast.dump(current_check), ast.dump(original_check))


if __name__ == '__main__':
    unittest.main()
