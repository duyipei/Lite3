"""Exercise the real initializer's publishing and drift revocation without ROS."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
import yaml
from unittest.mock import patch

NS = types.SimpleNamespace
PACKAGE = Path(__file__).resolve().parents[1]/'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav'


class PoseMessage:
    def __init__(self):
        self.header = NS(stamp=NS(sec=10, nanosec=0), frame_id='visual_map')
        self.pose = NS(pose=NS(position=NS(x=0., y=0., z=0.),
                              orientation=NS(x=0., y=0., z=0., w=1.)),
                       covariance=[0.]*36)
        self.child_frame_id = 'base_link'


class Node:
    def __init__(self, name):
        self.params, self.outputs, self.now = {}, {}, 10.
    def declare_parameter(self, key, value):
        self.params[key] = True if key == 'configured' else value
    def get_parameter(self, key): return NS(value=self.params[key])
    def create_publisher(self, kind, topic, qos):
        self.outputs[topic] = []
        return NS(publish=lambda msg: self.outputs[topic].append(copy.deepcopy(msg)))
    def create_subscription(self, *args): pass
    def create_service(self, *args): pass
    def create_timer(self, *args): pass
    def get_logger(self): return NS(info=lambda s: None, warn=lambda s: None, error=lambda s: None)
    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.now*1e9),
                  to_msg=lambda: NS(sec=int(self.now), nanosec=0)))


class InitializerTests(unittest.TestCase):
    def setUp(self):
        fake = {
            'rclpy': NS(), 'rclpy.node': NS(Node=Node),
            'rclpy.qos': NS(QoSProfile=lambda **kw: NS(),
                DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1)),
            'geometry_msgs.msg': NS(PoseWithCovarianceStamped=PoseMessage),
            'nav_msgs.msg': NS(Odometry=PoseMessage),
            'hdl_localization.msg': NS(ScanMatchingStatus=NS),
            'std_msgs.msg': NS(Bool=NS, Empty=NS, String=NS),
            'std_srvs.srv': NS(Trigger=NS),
            'initializer_test': NS(__path__=[str(PACKAGE)]),
        }
        self.patch = patch.dict(sys.modules, fake)
        self.patch.start(); self.addCleanup(self.patch.stop)
        spec = importlib.util.spec_from_file_location('initializer_test.visual_pose_initializer',
                                                       PACKAGE/'visual_pose_initializer.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        self.node = module.VisualPoseInitializer()
        self.node.now_monotonic = lambda: self.node.now

    def test_initialpose_contains_full_reference_rotation(self):
        n = self.node
        n.seed_reference_xyz = [.04, -.02, -.72]
        n.seed_reference_rpy = [0., -math.pi/10, 0.]
        n.params.update(stable_samples=2, stable_duration_s=.8, max_sample_gap_s=2.)
        n.requested = True
        for stamp in (10, 11):
            n.now = stamp
            msg = PoseMessage(); msg.header.stamp.sec = stamp
            n.visual_pose_callback(msg)
        published = n.outputs['/initialpose'][-1]
        self.assertEqual(published.header.frame_id, 'map')
        self.assertAlmostEqual(published.pose.pose.position.z, -.72)
        self.assertAlmostEqual(published.pose.pose.orientation.y, -math.sin(math.pi/20))
        self.assertAlmostEqual(published.pose.pose.orientation.w, math.cos(math.pi/20))
        self.assertTrue(n.state.startswith('WAITING_NDT'))

    def leg_history(self, begin, end, moving=False, rotating=False):
        n = self.node
        for tick in range(round(begin*10), round(end*10)+1):
            n.now = tick/10
            msg = PoseMessage()
            msg.header.stamp.sec, tenth = divmod(tick, 10)
            msg.header.stamp.nanosec = tenth*100000000
            # A temporary excursion must be detected even if it returns.
            offset = .1 if moving and tick == round((begin+end)*5) else 0.
            yaw = .1 if rotating and tick == round((begin+end)*5) else 0.
            msg.pose.pose.position.x = offset
            msg.pose.pose.orientation.z = math.sin(yaw/2)
            msg.pose.pose.orientation.w = math.cos(yaw/2)
            n.leg_callback(msg)

    def test_slow_stationary_initialization_still_needs_ndt(self):
        n = self.node
        n.params.update(initial_visual_max_age_s=5., max_sample_gap_s=6.,
                        stable_samples=4, stable_duration_s=2.)
        n.requested = True
        for stamp in (10, 14, 18, 22):
            self.leg_history(stamp-.1, stamp+3.5)
            msg = PoseMessage(); msg.header.stamp.sec = stamp
            n.visual_pose_callback(msg)
        self.assertEqual(len(n.outputs['/initialpose']), 1)
        self.assertTrue(n.state.startswith('WAITING_NDT'))
        self.assertFalse(n.outputs['/patrol/localization_ready'][-1].data)

    def test_delayed_pose_without_motion_history_rejected(self):
        n = self.node
        n.params['initial_visual_max_age_s'] = 5.
        n.requested = True; n.now = 13.
        n.visual_pose_callback(PoseMessage())
        self.assertIn('missing_leg_history', n.state)
        self.assertEqual(len(n.samples), 0)

    def test_delayed_pose_rejects_motion_and_temporary_rotation(self):
        n = self.node
        n.params['initial_visual_max_age_s'] = 5.
        n.requested = True
        for moving, rotating in ((True, False), (False, True)):
            n.visual_leg_history.clear(); n.last_leg_stamp = 0
            self.leg_history(9.9, 13., moving, rotating)
            n.visual_pose_callback(PoseMessage())
            self.assertIn('robot_moved', n.state)
            self.assertEqual(len(n.samples), 0)

    def test_delayed_pose_rejects_gaps_stale_odometry_and_arm(self):
        n = self.node
        n.params['initial_visual_max_age_s'] = 5.
        self.leg_history(9.9, 13.)
        original = list(n.visual_leg_history)
        n.visual_leg_history.clear()
        n.visual_leg_history.extend([original[0], original[-1]])
        self.assertEqual(n.initial_visual_timing_reason(PoseMessage()), 'delayed_visual_leg_gap')
        n.visual_leg_history.clear(); n.visual_leg_history.extend(original)
        n.now = 13.6
        self.assertEqual(n.initial_visual_timing_reason(PoseMessage()), 'delayed_visual_leg_gap')
        n.now = 13.; n.has_been_armed = True
        self.assertEqual(n.initial_visual_timing_reason(PoseMessage()), 'delayed_visual_after_arm')

    def test_visual_age_limit_and_future_clock_are_bounded(self):
        n = self.node
        n.params['initial_visual_max_age_s'] = 5.
        self.leg_history(9.9, 15.)
        self.assertIsNone(n.initial_visual_timing_reason(PoseMessage()))
        for now in (15.001, 9.49):
            n.now = now
            self.assertEqual(n.initial_visual_timing_reason(PoseMessage()), 'visual_stamp_out_of_window')

    def test_initial_visual_allowance_does_not_relax_ready_or_leg_age(self):
        n = self.node
        n.params['initial_visual_max_age_s'] = 5.
        n.now = 13.; n.state = 'READY:test'; n.requested = True
        n.visual_pose_callback(PoseMessage())
        n.leg_callback(PoseMessage())
        self.assertEqual(n.last_visual_time, 0.)
        self.assertEqual(n.last_leg_time, 0.)
        self.assertEqual(n.state, 'READY:test')

    def test_burst_arrivals_cannot_invent_stable_capture_duration(self):
        n = self.node
        n.params.update(initial_visual_max_age_s=5., stable_samples=2,
                        stable_duration_s=2., max_sample_gap_s=6.)
        n.requested = True
        msg = PoseMessage(); n.visual_pose_callback(msg)
        self.leg_history(9.9, 13.)
        msg = PoseMessage(); msg.header.stamp.nanosec = 100000000
        n.visual_pose_callback(msg)
        self.assertEqual(n.state, 'COLLECTING:stable_duration_too_short')
        self.assertEqual(n.outputs['/initialpose'], [])

    def production_seed(self):
        root = PACKAGE.parents[3]
        config = yaml.safe_load((root/'navigation/outdoor_20260921_003_area_review/visual_initialization.yaml').read_text())
        n = self.node
        n.params.update(config['visual_pose_initializer']['ros__parameters'])
        n.requested = True
        for stamp in (10, 11, 12):
            n.now = stamp
            msg = PoseMessage(); msg.header.stamp.sec = stamp
            n.visual_pose_callback(msg)
            self.assertEqual(n.outputs['/initialpose'], [])
        n.now = 13
        msg = PoseMessage(); msg.header.stamp.sec = 13
        n.visual_pose_callback(msg)
        self.assertEqual(len(n.outputs['/initialpose']), 1)
        return n

    def accepted_scan(self, stamp, good=True):
        n = self.node
        n.now = float(stamp)
        n.last_leg_time = n.now
        n.status_callback(NS(header=NS(stamp=NS(sec=stamp, nanosec=0)),
                             has_converged=good, matching_error=.01, inlier_fraction=.8))
        msg = PoseMessage(); msg.header.frame_id = 'map'; msg.header.stamp.sec = stamp
        n.odom_callback(msg)

    def test_shorter_production_flow_still_requires_distinct_good_ndt_scans(self):
        n = self.production_seed()
        for stamp in (20, 20, 21):
            self.accepted_scan(stamp)
            self.assertFalse(n.state.startswith('READY'))
        self.accepted_scan(22)
        self.assertTrue(n.state.startswith('READY:'))
        self.assertEqual(len(n.outputs['/initialpose']), 1)

    def test_bad_ndt_scan_restarts_short_confirmation(self):
        n = self.production_seed()
        self.accepted_scan(20)
        self.accepted_scan(21, good=False)
        for stamp in (22, 23):
            self.accepted_scan(stamp)
            self.assertFalse(n.state.startswith('READY'))
        self.accepted_scan(24)
        self.assertTrue(n.state.startswith('READY:'))

    def test_accumulated_stationary_drift_revokes_ready(self):
        n = self.node
        n.params['stationary_drift_guard_enabled'] = True
        n.state = 'READY:test'
        for step in range(23):
            n.now = 10+step*.1
            n.latest_leg_pose = (0., 0., 0.)
            n.last_leg_time = n.now
            msg = PoseMessage(); msg.header.frame_id = 'map'
            ns = round(n.now*1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
            msg.pose.pose.position.x = step*.01
            n.odom_callback(msg)
            if n.state.startswith('FAILED'):
                break
        self.assertIn('FAILED:stationary_map_drift', n.state)
        self.assertFalse(n.outputs['/patrol/localization_ready'][-1].data)

    def prepare_timing(self):
        n = self.node
        n.params.update(startup_timing_grace_s=3.0, supervise_visual_after_ready=False)
        n.state = 'READY:test'
        n.last_odom_time = 9.0
        n.last_status_time = n.last_leg_time = 10.0
        return n

    def test_startup_gap_withholds_ready_without_losing_seed(self):
        n = self.prepare_timing()
        n.timer_callback()
        self.assertEqual(n.state, 'READY:test')
        self.assertFalse(n.outputs['/patrol/localization_ready'][-1].data)
        self.assertTrue(n.outputs['/patrol/visual_init_state'][-1].data.startswith('WAITING_STARTUP_TIMING:'))
        n.last_odom_time = n.now
        n.timer_callback()
        self.assertTrue(n.outputs['/patrol/localization_ready'][-1].data)
        self.assertEqual(len(n.outputs['/initialpose']), 0)

    def test_startup_grace_is_bounded(self):
        n = self.prepare_timing()
        n.last_odom_time = 6.9
        n.timer_callback()
        self.assertEqual(n.state, 'FAILED:odometry_stale')

    def test_after_any_arm_original_timeout_remains_latched(self):
        for disarmed in (False, True):
            n = self.prepare_timing()
            n.armed_callback(NS(data=True))
            if disarmed:
                n.armed_callback(NS(data=False))
            n.timer_callback()
            self.assertEqual(n.state, 'FAILED:odometry_stale')
            n.last_odom_time = n.now
            n.timer_callback()
            self.assertFalse(n.outputs['/patrol/localization_ready'][-1].data)

    def test_startup_status_gap_does_not_bypass_bad_matching(self):
        n = self.prepare_timing()
        n.last_status_time = 8.0
        n.status_callback(NS(header=NS(stamp=NS(sec=10,nanosec=0)),
                             has_converged=False,matching_error=1.0,inlier_fraction=.1))
        self.assertEqual(n.state, 'READY:test')
        n.ndt_bad_count = n.params['ndt_bad_samples']-1
        n.now=10.1
        n.status_callback(NS(header=NS(stamp=NS(sec=10,nanosec=100000000)),
                             has_converged=False,matching_error=1.0,inlier_fraction=.1))
        self.assertEqual(n.state, 'FAILED:ndt_matching_failed')

    def test_old_source_scan_does_not_refresh_status_or_odom(self):
        n=self.node
        before=n.last_status_time
        n.status_callback(NS(header=NS(stamp=NS(sec=1,nanosec=0)),
                             has_converged=True,matching_error=.01,inlier_fraction=.8))
        self.assertEqual(n.last_status_time,before)
        self.assertEqual(n.outputs['/patrol/lidar_heartbeat'],[])
        old=PoseMessage();old.header.frame_id='map';old.header.stamp.sec=1
        n.odom_callback(old)
        self.assertIsNone(n.latest_odom)

    def test_low_error_low_overlap_is_not_good_registration(self):
        n=self.node
        n.status_callback(NS(header=NS(stamp=NS(sec=10,nanosec=0)),
                             has_converged=True,matching_error=.001,inlier_fraction=.001))
        self.assertFalse(n.scan_results[10000000000])
