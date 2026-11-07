"""T4.1 offline TF query, failure-priority and T3 integration tests. No ROS."""
from collections import Counter
import copy
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

import test_voa_costmap_bridge as fixtures
from test_voa_tf_retry import (
    Clock, ConnectivityException, ExtrapolationException, LookupException,
    TransformException,
)

M = fixtures.M


def stamp_ns(stamp):
    return stamp.sec*1000000000 + stamp.nanosec


def anchor(stamp, transform=None):
    transform = fixtures.transform(x=1., y=2.) if transform is None else transform
    return NS(header=NS(frame_id='map', stamp=fixtures.grid(stamp=stamp).header.stamp),
              child_frame_id='ndt_registration',
              pose=NS(pose=NS(position=transform.translation, orientation=transform.rotation)))


class FakeTF:
    def __init__(self):
        self.exact = ExtrapolationException('extrapolation into the past: NDT history')
        self.legs = {}
        self.calls = []

    def lookup_transform(self, target, source, stamp):
        # A three-argument signature also rejects new blocking timeout arguments.
        requested_ns = round(stamp*1000000000)
        self.calls.append((target, source, requested_ns))
        value = (self.exact if target == 'map' else
                 self.legs.get(requested_ns, LookupException('missing leg frame')))
        if isinstance(value, Exception):
            raise value
        return value

    def leg(self, stamp, transform=None, error=None):
        self.legs[stamp_ns(fixtures.grid(stamp=stamp).header.stamp)] = (
            error if error is not None else NS(transform=transform or fixtures.transform()))

    def grid_calls(self, stamp=100.):
        return self.calls.count(('robot_odom', 'leg_base_link', round(stamp*1000000000)))


class ProjectionTFDedupTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(100050000000)
        clock_patch = patch.object(M, 'time', self.clock)
        clock_patch.start()
        self.addCleanup(clock_patch.stop)
        self.tf = FakeTF()
        self.outputs = {}

        def publisher(kind, topic, qos):
            self.outputs[topic] = []
            return NS(publish=lambda message: self.outputs[topic].append(copy.deepcopy(message)))

        node = NS(create_publisher=publisher, get_logger=lambda: NS(info=lambda text: None))
        self.bridge = M.Bridge(node, self.tf, fixtures.Cloud, fixtures.Field, NS,
                               NS(from_msg=M.stamp_seconds), .1,
                               tf_exceptions=(TransformException,))
        self.msg = fixtures.grid(stamp=100.)

    def anchors(self, stamps=(99.8, 99.9, 99.95)):
        for stamp in stamps:
            self.bridge.registration(anchor(stamp))
            self.tf.leg(stamp)
        self.tf.leg(100.)

    def failure(self):
        with self.assertRaises(M.ProjectionUnavailable) as caught:
            self.bridge.projection_transform(self.msg)
        return caught.exception

    def test_exact_ndt_success_returns_original_object_without_leg_queries(self):
        expected = fixtures.transform(yaw=.2, pitch=.1, x=3., y=-2.)
        self.tf.exact = NS(transform=expected)
        self.anchors()
        self.assertIs(self.bridge.projection_transform(self.msg), expected)
        self.assertEqual(self.tf.calls, [('map', 'base_link', 100000000000)])
        self.assertEqual(self.bridge.predictions, 0)

    def test_latest_qualified_anchor_keeps_priority_and_anchor_first_order(self):
        self.anchors()
        result = self.bridge.projection_transform(self.msg)
        np.testing.assert_allclose(result, M.transform_matrix(fixtures.transform(x=1., y=2.)))
        self.assertEqual(self.tf.calls, [('map', 'base_link', 100000000000),
                                        ('robot_odom', 'leg_base_link', 99950000000),
                                        ('robot_odom', 'leg_base_link', 100000000000)])

    def test_grid_future_is_queried_once_across_all_anchors(self):
        self.anchors()
        self.tf.leg(100., error=ExtrapolationException('extrapolation into the future: leg grid'))
        error = self.failure()
        self.assertTrue(error.retryable)
        self.assertEqual((error.category, error.error_type), ('future', 'ExtrapolationException'))
        self.assertTrue(str(error).startswith('leg_grid:'))
        self.assertEqual(self.tf.grid_calls(), 1)
        self.assertEqual(len(self.tf.calls), 5)
        self.assertEqual(self.bridge.projection_failure, 'anchors_tf_unavailable')
        # This existing counter counts failed anchor paths, including reuse.
        self.assertEqual(self.bridge.diag_counts['anchor_tf_misses'], 3)

    def test_new_call_requeries_after_previous_failure_and_recovers(self):
        self.anchors()
        self.tf.leg(100., error=ExtrapolationException('future leg grid'))
        self.failure()
        self.tf.leg(100., transform=fixtures.transform(x=.1))
        result = self.bridge.projection_transform(self.msg)
        np.testing.assert_allclose(result[:3, 3], [1.1, 2., 0.])
        self.assertEqual(self.tf.grid_calls(), 2)
        self.assertEqual(self.bridge.projection_failure, 'none')

    def test_successful_lookup_is_not_cached_across_calls(self):
        self.anchors()
        first = self.bridge.projection_transform(self.msg)
        self.tf.leg(100., transform=fixtures.transform(x=.2))
        second = self.bridge.projection_transform(self.msg)
        self.assertEqual(self.tf.grid_calls(), 2)
        self.assertAlmostEqual(second[0, 3]-first[0, 3], .2)

    def test_tf_arriving_during_anchor_loop_is_retried_on_next_call(self):
        self.anchors()
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        original_lookup = self.tf.lookup_transform

        def lookup(target, source, stamp):
            try:
                return original_lookup(target, source, stamp)
            except ExtrapolationException:
                if target == 'robot_odom' and stamp == 100.:
                    self.tf.leg(100.)  # New TF arrives after this failed query.
                raise

        self.tf.lookup_transform = lookup
        self.failure()
        self.assertEqual(self.tf.grid_calls(), 1)
        self.bridge.projection_transform(self.msg)
        self.assertEqual(self.tf.grid_calls(), 2)

    def test_no_anchor_performs_only_exact_query(self):
        error = self.failure()
        self.assertFalse(error.retryable)
        self.assertEqual(error.category, 'past')
        self.assertEqual(self.bridge.projection_failure, 'no_eligible_anchor')
        self.assertEqual(self.tf.calls, [('map', 'base_link', 100000000000)])

    def test_out_of_window_anchors_do_not_trigger_leg_queries(self):
        self.bridge.anchors.extend([anchor(99.49), anchor(100.01)])
        self.failure()
        self.assertEqual(self.bridge.projection_failure, 'no_eligible_anchor')
        self.assertEqual(len(self.tf.calls), 1)

    def test_anchor_at_half_second_boundary_remains_eligible(self):
        self.bridge.anchors.append(anchor(99.5))
        self.tf.leg(99.5)
        self.tf.leg(100.)
        self.bridge.projection_transform(self.msg)
        self.assertIn(('robot_odom', 'leg_base_link', 99500000000), self.tf.calls)

    def test_multiple_anchor_failures_fall_back_to_older_qualified_anchor(self):
        self.anchors()
        self.tf.leg(99.95, error=LookupException('newest anchor missing'))
        self.tf.leg(99.9, error=ConnectivityException('middle anchor disconnected'))
        self.bridge.projection_transform(self.msg)
        self.assertEqual([call[2] for call in self.tf.calls],
                         [100000000000, 99950000000, 99900000000, 99800000000, 100000000000])
        self.assertEqual(self.tf.grid_calls(), 1)
        self.assertEqual(self.bridge.diag_counts['anchor_tf_misses'], 2)

    def test_all_anchor_failures_do_not_query_unused_grid_leg_or_change_category(self):
        self.anchors()
        for stamp in (99.8, 99.9, 99.95):
            self.tf.leg(stamp, error=ExtrapolationException('past anchor history'))
        self.tf.leg(100., error=ExtrapolationException('future grid must not be queried'))
        error = self.failure()
        self.assertFalse(error.retryable)
        self.assertEqual(error.category, 'past')
        self.assertEqual(self.tf.grid_calls(), 0)
        self.assertTrue(str(error).startswith('leg_anchor:'))

    def test_expected_grid_exception_categories_are_preserved(self):
        self.anchors()
        for failure, category, retryable in [
                (ExtrapolationException('extrapolation into the future'), 'future', True),
                (ExtrapolationException('extrapolation into the past'), 'past', False),
                (LookupException('frame not found'), 'lookup', False),
                (ConnectivityException('disconnected frame trees'), 'connectivity', False),
                (TransformException('unclassified TF failure'), 'tf_other', False)]:
            with self.subTest(category=category):
                self.tf.calls.clear()
                self.tf.leg(100., error=failure)
                error = self.failure()
                self.assertEqual((error.category, error.retryable), (category, retryable))
                self.assertEqual(error.error_type, type(failure).__name__)
                self.assertIs(error.__cause__, failure)
                self.assertEqual(self.tf.grid_calls(), 1)

    def test_anchor_future_has_priority_over_cached_grid_past(self):
        self.anchors()
        future = ExtrapolationException('future middle anchor')
        past = ExtrapolationException('past grid')
        self.tf.leg(99.9, error=future)
        self.tf.leg(100., error=past)
        error = self.failure()
        self.assertTrue(error.retryable)
        self.assertTrue(str(error).startswith('leg_anchor:ExtrapolationException:future:'))
        self.assertIs(error.__cause__, past)  # Existing last-path chaining is retained.
        self.assertEqual(self.tf.grid_calls(), 1)

    def test_cached_grid_future_keeps_last_logical_future_path_priority(self):
        self.anchors()
        self.tf.leg(99.9, error=ExtrapolationException('future middle anchor'))
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        error = self.failure()
        self.assertTrue(str(error).startswith('leg_grid:ExtrapolationException:future:'))
        self.assertEqual(self.tf.grid_calls(), 1)

    def test_exact_future_remains_retryable_when_all_anchors_are_past(self):
        self.anchors()
        self.tf.exact = ExtrapolationException('future NDT')
        for stamp in (99.8, 99.9, 99.95):
            self.tf.leg(stamp, error=ExtrapolationException('past anchor'))
        error = self.failure()
        self.assertTrue(error.retryable)
        self.assertTrue(str(error).startswith('map_grid:ExtrapolationException:future:'))
        self.assertEqual(self.tf.grid_calls(), 0)

    def test_zero_age_anchor_success_shares_its_lookup_with_grid(self):
        self.anchors((99.9, 100.))
        self.bridge.projection_transform(self.msg)
        self.assertEqual(self.tf.grid_calls(), 1)
        self.assertEqual(len(self.tf.calls), 2)

    def test_zero_age_anchor_failure_is_reused_on_older_anchor_grid_path(self):
        self.anchors((99.9, 100.))
        self.tf.leg(100., error=ExtrapolationException('future shared time'))
        error = self.failure()
        self.assertEqual(self.tf.grid_calls(), 1)
        self.assertTrue(str(error).startswith('leg_grid:ExtrapolationException:future:'))
        self.assertEqual(self.bridge.diag_counts['anchor_tf_misses'], 2)

    def test_one_nanosecond_difference_does_not_alias_anchor_and_grid(self):
        self.anchors((99.999999999,))
        self.bridge.projection_transform(self.msg)
        self.assertIn(('robot_odom', 'leg_base_link', 99999999999), self.tf.calls)
        self.assertEqual(self.tf.grid_calls(), 1)
        self.assertEqual(len(self.tf.calls), 3)

    def test_failure_diagnostic_is_emitted_only_for_actual_grid_lookup(self):
        self.anchors()
        probe = self.bridge._t22 = Mock()
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        self.failure()
        self.assertEqual([call.args[0] for call in probe.miss.call_args_list],
                         ['map_grid', 'leg_grid'])
        self.assertEqual(self.bridge.diag_counts['anchor_tf_misses'], 3)

    def test_twelve_anchors_require_at_most_fourteen_queries_per_candidate(self):
        stamps = [99.8+index*.005 for index in range(12)]
        self.anchors(stamps)
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        self.failure()
        self.assertEqual(len(self.tf.calls), 14)
        self.assertEqual(self.tf.grid_calls(), 1)
        self.assertEqual(len(self.bridge.anchors), 12)

    def test_four_candidates_require_at_most_fifty_six_queries_per_tick(self):
        self.anchors([99.8+index*.005 for index in range(12)])
        for stamp in [100.+index*.01 for index in range(4)]:
            self.tf.leg(stamp, error=ExtrapolationException('future grid'))
            self.bridge.receive(fixtures.grid(stamp=stamp))
        self.bridge.publish()
        self.assertEqual(len(self.tf.calls), 56)
        counts = Counter(call[2] for call in self.tf.calls if call[0] == 'robot_odom')
        self.assertTrue(all(counts[100000000000+index*10000000] == 1 for index in range(4)))
        self.assertFalse(self.outputs[M.TOPIC])
        self.assertFalse(self.outputs[M.HEARTBEAT])

    def test_t3_retries_requery_without_extending_deadline_then_publish_once(self):
        self.anchors()
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        header = copy.deepcopy(self.msg.header)
        self.bridge.receive(self.msg)
        candidate = self.bridge.pending[-1]
        self.bridge.publish()
        deadline = candidate.retry_deadline_monotonic_ns
        first_failure = candidate.first_tf_failure_monotonic_ns
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual(candidate.retry_deadline_monotonic_ns, deadline)
        self.assertEqual(candidate.first_tf_failure_monotonic_ns, first_failure)
        self.tf.leg(100.)
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual(candidate.state, 'PUBLISHED')
        self.assertEqual(candidate.attempt_count, 3)
        self.assertEqual(self.tf.grid_calls(), 3)
        self.assertEqual(self.bridge.diag_counts['retry_recovered'], 1)
        self.assertEqual(self.outputs[M.TOPIC][0].header.stamp, header.stamp)
        self.assertEqual(self.outputs[M.TOPIC][0].header.frame_id, 'map')
        self.assertEqual(self.outputs[M.HEARTBEAT][0], self.outputs[M.TOPIC][0].header)
        self.assertEqual(self.msg.header, header)
        self.bridge.receive(self.msg)  # Replayed source is not a second observation.
        calls = len(self.tf.calls)
        self.bridge.publish()
        self.assertEqual(len(self.tf.calls), calls)
        self.assertEqual(len(self.outputs[M.TOPIC]), 1)

    def test_t3_wait_deadline_expires_even_if_tf_is_now_available(self):
        self.anchors()
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        self.bridge.receive(self.msg)
        candidate = self.bridge.pending[-1]
        self.bridge.publish()
        calls = len(self.tf.calls)
        self.tf.leg(100.)
        self.clock.advance(175)
        self.bridge.publish()
        self.assertEqual(candidate.state, 'EXPIRED')
        self.assertEqual(self.bridge.diag_counts['tf_wait_timeout'], 1)
        self.assertEqual(len(self.tf.calls), calls)
        self.assertFalse(self.outputs[M.HEARTBEAT])

    def test_old_source_shortens_wait_without_moving_source_budget(self):
        self.anchors()
        self.clock.advance(150)  # Source is already 200 ms old.
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        self.bridge.receive(self.msg)
        candidate = self.bridge.pending[-1]
        self.bridge.publish()
        self.assertEqual(candidate.retry_deadline_monotonic_ns, 100250000000)
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual(candidate.retry_deadline_monotonic_ns, 100250000000)
        self.clock.advance(25)
        self.bridge.publish()
        self.assertEqual(candidate.state, 'EXPIRED')
        self.assertEqual(self.bridge.diag_counts['retry_source_budget'], 1)
        self.assertFalse(self.outputs[M.HEARTBEAT])

    def test_expired_candidate_does_no_tf_work(self):
        self.bridge.receive(self.msg)
        self.clock.advance(301)
        self.bridge.publish()
        self.assertFalse(self.tf.calls)
        self.assertFalse(self.outputs[M.TOPIC])

    def test_original_source_header_is_preserved_on_failure(self):
        self.anchors()
        header = copy.deepcopy(self.msg.header)
        self.tf.leg(100., error=ExtrapolationException('future grid'))
        self.failure()
        self.assertEqual(self.msg.header, header)

    def test_unexpected_exceptions_propagate_at_each_query_stage(self):
        self.anchors()
        for stage in ('map_grid', 'leg_anchor', 'leg_grid'):
            with self.subTest(stage=stage):
                self.tf.exact = ExtrapolationException('past NDT')
                self.tf.leg(99.95)
                self.tf.leg(100.)
                unexpected = RuntimeError('unexpected TF implementation failure')
                if stage == 'map_grid': self.tf.exact = unexpected
                else: self.tf.leg(99.95 if stage == 'leg_anchor' else 100., error=unexpected)
                with self.assertRaises(RuntimeError) as caught:
                    self.bridge.projection_transform(self.msg)
                self.assertIs(caught.exception, unexpected)

    def test_motion_bounds_still_raise_without_trying_an_older_anchor(self):
        self.anchors()
        for transform in (fixtures.transform(x=.51), fixtures.transform(yaw=.36)):
            with self.subTest(transform=transform):
                self.tf.calls.clear()
                self.tf.leg(100., transform=transform)
                with self.assertRaises(ValueError): self.bridge.projection_transform(self.msg)
                self.assertNotIn(('robot_odom', 'leg_base_link', 99900000000), self.tf.calls)

    def test_rigid_compensation_matches_independent_matrix_reference(self):
        def pose_and_matrix(xyz, rpy):
            roll, pitch, yaw = rpy
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            cy, sy = math.cos(yaw/2), math.sin(yaw/2)
            transform = NS(translation=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
                           rotation=NS(x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy,
                                       z=cr*cp*sy-sr*sp*cy, w=cr*cp*cy+sr*sp*sy))
            c, s = np.cos(rpy), np.sin(rpy)
            rx = np.array([[1, 0, 0], [0, c[0], -s[0]], [0, s[0], c[0]]])
            ry = np.array([[c[1], 0, s[1]], [0, 1, 0], [-s[1], 0, c[1]]])
            rz = np.array([[c[2], -s[2], 0], [s[2], c[2], 0], [0, 0, 1]])
            matrix = np.eye(4)
            matrix[:3, :3], matrix[:3, 3] = rz @ ry @ rx, xyz
            return transform, matrix

        for anchor_stamp, anchor_rpy in [(99.5, (0., 0., 0.)), (99.8, (.12, -.18, .7)),
                                         (99.95, (-.09, .16, -1.1))]:
            with self.subTest(anchor_stamp=anchor_stamp, rpy=anchor_rpy):
                a, ma = pose_and_matrix((3., -2., .4), anchor_rpy)
                l0, m0 = pose_and_matrix((1., 2., .3), (.04, -.06, .45))
                l1, m1 = pose_and_matrix((1.12, 1.93, .35), (.06, -.05, .51))
                self.bridge.anchors.clear()
                self.bridge.anchors.append(anchor(anchor_stamp, a))
                self.tf.leg(anchor_stamp, l0)
                self.tf.leg(100., l1)
                result = self.bridge.projection_transform(self.msg)
                np.testing.assert_allclose(result, ma @ np.linalg.inv(m0) @ m1, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
