"""Real initializer with fake ROS: no robot, services, or ROS processes."""
import copy
import importlib.util
import math
import sys
import types
import unittest
from unittest.mock import patch

import test_visual_initializer_frames as fixtures

NS = types.SimpleNamespace


class ManualNode(fixtures.Node):
    def __init__(self, name):
        super().__init__(name)
        self.publishers, self.subscriptions, self.services, self.warnings = {}, [], [], []

    def declare_parameter(self, key, value):
        super().declare_parameter(key, value)
        if key == 'initialization_mode':
            self.params[key] = 'manual'
        elif key == 'configured':
            self.params[key] = False

    def count_publishers(self, topic):
        return self.publishers.get(topic, 0)

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append(topic)

    def create_service(self, kind, topic, callback):
        self.services.append(topic)

    def get_logger(self):
        return NS(info=lambda s: None, warn=self.warnings.append, error=self.warnings.append)


class ManualInitializationTests(unittest.TestCase):
    def setUp(self):
        fake = {
            'rclpy': NS(), 'rclpy.node': NS(Node=ManualNode),
            'rclpy.qos': NS(QoSProfile=lambda **kw: NS(),
                DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1)),
            'geometry_msgs.msg': NS(PoseWithCovarianceStamped=fixtures.PoseMessage),
            'nav_msgs.msg': NS(Odometry=fixtures.PoseMessage),
            'hdl_localization.msg': NS(ScanMatchingStatus=NS),
            'std_msgs.msg': NS(Bool=NS, Empty=NS, String=NS),
            'std_srvs.srv': NS(Trigger=NS),
            'manual_initializer_test': NS(__path__=[str(fixtures.PACKAGE)]),
        }
        p = patch.dict(sys.modules, fake)
        p.start(); self.addCleanup(p.stop)
        spec = importlib.util.spec_from_file_location(
            'manual_initializer_test.visual_pose_initializer',
            fixtures.PACKAGE/'visual_pose_initializer.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.node = n = module.VisualPoseInitializer()
        n.now_monotonic = lambda: n.now
        n.latest_leg_pose = (0., 0., 0.)
        n.last_leg_time = n.now
        n.manual_quiet_since = n.now-1.1
        n.params.update(ndt_confirmation_samples=3, ndt_settle_s=0.)

    def pose(self, x=1., y=2., stamp=10):
        msg = fixtures.PoseMessage()
        msg.header.frame_id = 'map'; msg.header.stamp.sec = stamp
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        return msg

    def seed(self):
        self.node.manual_pose_callback(self.pose())
        self.assertEqual(len(self.node.outputs['/initialpose']), 1)

    def scan(self, stamp, good=True, x=1., y=2.):
        n = self.node
        n.now = float(stamp); n.last_leg_time = n.now
        n.status_callback(NS(header=NS(stamp=NS(sec=stamp, nanosec=0)),
                             has_converged=good, matching_error=.01, inlier_fraction=.8))
        msg = self.pose(x, y, stamp)
        n.odom_callback(msg)

    def ready(self):
        self.seed()
        for stamp in (11, 12, 13):
            self.scan(stamp)
        self.assertTrue(self.node.state.startswith('READY:manual_plus_ndt_confirmed'))

    def test_manual_subscribes_only_isolated_pose_and_needs_no_visual_configuration(self):
        n = self.node
        self.assertIn('/patrol/manual_initialpose_request', n.subscriptions)
        self.assertNotIn('/visual_localization/pose', n.subscriptions)
        self.assertNotIn('/patrol/request_visual_initialization', n.services)
        self.assertEqual(n.state, 'WAITING_MANUAL_INITIALPOSE')
        self.assertFalse(n.outputs['/patrol/localization_ready'][-1].data)

    def test_map_pose_is_not_transformed_as_a_visual_imu_pose(self):
        n = self.node
        n.seed_reference_xyz = [5., 6., -.72]
        n.seed_reference_rpy = [0., -.3, 0.]
        n.params.update(map_transform_x=99., map_transform_y=99.)
        self.seed()
        msg = n.outputs['/initialpose'][-1]
        self.assertEqual((msg.pose.pose.position.x, msg.pose.pose.position.y,
                          msg.pose.pose.position.z), (1., 2., 0.))
        self.assertEqual(msg.pose.pose.orientation.y, 0.)
        self.assertEqual(msg.pose.pose.orientation.w, 1.)

    def test_operator_clock_is_separate_from_scan_clock(self):
        n = self.node
        n.manual_pose_callback(self.pose(stamp=1794220000))
        self.assertEqual(n.outputs['/initialpose'][-1].header.stamp.sec, 10)
        self.assertEqual(n.initialpose_stamp, 10000000000)
        self.assertEqual(n.last_manual_stamp, 1794220000000000000)

    def test_click_does_not_publish_ready(self):
        self.seed()
        self.assertFalse(self.node.outputs['/patrol/localization_ready'][-1].data)
        self.assertTrue(self.node.state.startswith('WAITING_NDT:manual'))

    def test_distinct_accepted_scans_are_required(self):
        self.seed()
        for stamp in (11, 11, 12):
            self.scan(stamp)
            self.assertFalse(self.node.state.startswith('READY'))
        self.scan(13)
        self.assertTrue(self.node.state.startswith('READY:manual_plus_ndt_confirmed'))

    def test_preseed_scans_cannot_confirm_new_manual_pose(self):
        self.seed()
        self.scan(9)
        self.assertEqual(self.node.ndt_good_count, 0)

    def test_bad_ndt_resets_confirmation(self):
        self.seed(); self.scan(11); self.scan(12, good=False)
        self.assertEqual(self.node.ndt_good_count, 0)
        self.scan(13); self.scan(14)
        self.assertFalse(self.node.state.startswith('READY'))
        self.scan(15)
        self.assertTrue(self.node.state.startswith('READY'))

    def test_position_disagreement_remains_blocked(self):
        self.seed()
        for stamp in (11, 12, 13):
            self.scan(stamp, x=2.)
        self.assertFalse(self.node.state.startswith('READY'))
        self.assertEqual(self.node.ndt_good_count, 0)

    def test_timeout_never_republishes_or_reinitializes_automatically(self):
        self.seed(); self.node.now = 30.
        self.node.timer_callback()
        self.assertEqual(self.node.state, 'FAILED:ndt_confirmation_timeout')
        self.assertIsNone(self.node.candidate)
        self.assertEqual(len(self.node.outputs['/initialpose']), 1)

    def test_manual_ready_does_not_require_a_camera(self):
        self.ready(); n = self.node
        n.last_visual_time = 0.; n.params['supervise_visual_after_ready'] = True
        n.timer_callback()
        self.assertTrue(n.outputs['/patrol/localization_ready'][-1].data)

    def test_ready_pose_cannot_be_overwritten_by_another_click(self):
        self.ready(); n = self.node; state = n.state
        n.manual_pose_callback(self.pose(x=3., stamp=20))
        self.assertEqual(len(n.outputs['/initialpose']), 1)
        self.assertEqual(n.state, state)

    def test_armed_input_is_rejected(self):
        self.node.armed_callback(NS(data=True))
        self.node.manual_pose_callback(self.pose())
        self.assertEqual(self.node.outputs['/initialpose'], [])

    def test_gate_unknown_is_not_assumed_disarmed(self):
        n = self.node; n.publishers['/patrol/armed'] = 1
        n.manual_pose_callback(self.pose())
        self.assertEqual(n.outputs['/initialpose'], [])

    def test_fresh_explicit_disarmed_gate_can_initialize(self):
        n = self.node; n.publishers['/patrol/armed'] = 1
        n.armed_callback(NS(data=False)); self.seed()

    def test_stale_disarmed_confirmation_is_rejected(self):
        n = self.node; n.publishers['/patrol/armed'] = 1
        n.last_armed_time = 8.
        n.manual_pose_callback(self.pose())
        self.assertEqual(n.outputs['/initialpose'], [])

    def test_active_navigation_or_public_velocity_publishers_are_rejected(self):
        for topic in ('/patrol/cmd_vel_raw', '/cmd_vel'):
            with self.subTest(topic=topic):
                self.node.publishers = {topic: 1}
                self.node.manual_pose_callback(self.pose())
                self.assertEqual(self.node.outputs['/initialpose'], [])

    def test_pre_nav2_bootstrap_requires_an_observed_quiet_interval(self):
        n = self.node; n.manual_quiet_since = None
        n.manual_pose_callback(self.pose())
        self.assertEqual(n.outputs['/initialpose'], [])
        n.now = 11.01; n.last_leg_time = n.now
        self.seed()

    def test_graph_error_fails_closed(self):
        def broken(_topic):
            raise RuntimeError('graph unavailable')
        self.node.count_publishers = broken
        self.node.manual_pose_callback(self.pose())
        self.assertEqual(self.node.outputs['/initialpose'], [])

    def test_missing_or_stale_leg_odometry_blocks_seed(self):
        n = self.node
        for pose, last_time in ((None, n.now), ((0., 0., 0.), 8.)):
            n.latest_leg_pose, n.last_leg_time = pose, last_time
            n.manual_pose_callback(self.pose())
            self.assertEqual(n.outputs['/initialpose'], [])

    def test_invalid_frame_pose_stamp_covariance_and_bounds_are_rejected(self):
        variants = []
        msg = self.pose(); msg.header.frame_id = 'visual_map'; variants.append(msg)
        msg = self.pose(); msg.pose.pose.position.x = float('nan'); variants.append(msg)
        msg = self.pose(); msg.pose.pose.orientation.w = 0.; variants.append(msg)
        msg = self.pose(stamp=0); variants.append(msg)
        msg = self.pose(); msg.header.stamp.nanosec = 1000000000; variants.append(msg)
        msg = self.pose(); msg.pose.covariance[7] = -1.; variants.append(msg)
        msg = self.pose(x=2000000.); variants.append(msg)
        for msg in variants:
            self.node.manual_pose_callback(msg)
            self.assertEqual(self.node.outputs['/initialpose'], [])

    def test_duplicate_request_cannot_extend_confirmation_deadline(self):
        self.seed(); before = self.node.candidate_time
        self.node.now = 10.5; self.node.manual_pose_callback(self.pose())
        self.assertEqual(len(self.node.outputs['/initialpose']), 1)
        self.assertEqual(self.node.candidate_time, before)

    def test_new_manual_click_can_correct_an_unconfirmed_seed(self):
        self.seed(); self.node.now = 10.5
        self.node.manual_pose_callback(self.pose(x=1.2, stamp=11))
        self.assertEqual(len(self.node.outputs['/initialpose']), 2)
        self.assertEqual(self.node.candidate[0], 1.2)
        self.assertFalse(self.node.outputs['/patrol/localization_ready'][-1].data)

    def test_publish_failure_does_not_confirm_initialization(self):
        def broken(_msg):
            raise RuntimeError('publish failed')
        self.node.initialpose_publisher.publish = broken
        self.node.manual_pose_callback(self.pose())
        self.assertEqual(self.node.state, 'FAILED:manual_initialpose_publish')
        self.assertIsNone(self.node.candidate)

    def test_visual_input_is_ignored_in_manual_mode(self):
        self.node.visual_pose_callback(fixtures.PoseMessage())
        self.assertEqual(self.node.outputs['/initialpose'], [])

    def test_arm_during_confirmation_locks_manual_seed(self):
        self.seed(); n = self.node
        n.armed_callback(NS(data=True)); n.armed_callback(NS(data=False))
        n.manual_pose_callback(self.pose(stamp=11))
        self.assertEqual(len(n.outputs['/initialpose']), 1)
        self.assertTrue(n.manual_seed_locked)
        self.assertTrue(n.state.startswith('FAILED:'))

    def test_stationary_drift_guard_still_revokes_manual_ready(self):
        self.ready(); n = self.node
        n.params['stationary_drift_guard_enabled'] = True
        for step in range(1, 24):
            n.now = 13+step*.1; n.last_leg_time = n.now
            msg = self.pose(x=1+step*.01)
            ns = round(n.now*1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
            n.odom_callback(msg)
            if n.state.startswith('FAILED'): break
        self.assertIn('FAILED:stationary_map_drift', n.state)
        self.assertFalse(n.outputs['/patrol/localization_ready'][-1].data)

    def test_ndt_liveness_guard_still_revokes_manual_ready(self):
        self.ready(); n = self.node; n.now = 16.
        n.timer_callback()
        self.assertTrue(n.state.startswith('FAILED:'))
        self.assertFalse(n.outputs['/patrol/localization_ready'][-1].data)


if __name__ == '__main__':
    unittest.main()
