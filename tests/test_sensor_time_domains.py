"""Exercise production clock checks with independent source/elapsed clocks."""
import copy
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace as NS

import pytest

from lite3_patrol_nav import sensor_time as clocks
import test_t22_instrumentation as leg_fixture
import test_manual_initialization as manual_fixture
import test_visual_initializer_frames as init_fixture
import test_vendor_voa_gate as gate_fixture
import test_voa_tf_retry as retry_fixture


ROOT = Path(__file__).resolve().parents[1]


def test_clock_uses_real_ros_api_once_without_inferring_offset(monkeypatch):
    created, raw = [], [100250000000]

    class Clock:
        def __init__(self, *, clock_type):
            created.append(clock_type)
        def now(self):
            return NS(nanoseconds=raw[0])

    clocks._steady_clock.cache_clear()
    monkeypatch.setitem(sys.modules, 'rclpy.clock', NS(Clock=Clock, ClockType=NS(STEADY_TIME='ROS_STEADY')))
    try:
        assert clocks.sensor_time() == 100.25
        raw[0] += 1000000
        assert clocks.sensor_time() == 100.251
        assert created == ['ROS_STEADY']
    finally:
        clocks._steady_clock.cache_clear()


def test_clock_initialization_error_does_not_silently_use_another_clock(monkeypatch):
    def broken(**kwargs):
        raise RuntimeError('clock unavailable')
    clocks._steady_clock.cache_clear()
    monkeypatch.setitem(sys.modules, 'rclpy.clock', NS(Clock=broken, ClockType=NS(STEADY_TIME=3)))
    try:
        with pytest.raises(RuntimeError, match='clock unavailable'):
            clocks.sensor_time()
    finally:
        clocks._steady_clock.cache_clear()


@pytest.mark.parametrize('offset', [.25, -.25, 3.0])
def test_leg_source_age_uses_ros_clock_and_tf_stamp_is_preserved(monkeypatch, offset):
    module = leg_fixture.load_leg((ROOT/'scripts/leg_odom_tf_bridge.py').read_text())
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    raw = [100.+offset]
    module.sensor_time = lambda: raw[0]
    node = module.LegOdomTfBridge()
    msg = leg_fixture.odometry(raw[0]-.01)
    before = copy.deepcopy(msg)
    node.callback(msg)
    node.publish_latest()
    assert len(node.transforms) == len(node.samples) == 1
    assert node.transforms[0].header.stamp == before.header.stamp
    assert msg == before
    node.publish_latest()
    assert len(node.transforms) == 1
    raw[0] += .501
    node.callback(leg_fixture.odometry(before.header.stamp.sec+before.header.stamp.nanosec/1e9))
    node.publish_latest()
    assert len(node.transforms) == 1


@pytest.mark.parametrize('age', [-.051, .501])
def test_leg_really_future_or_stale_source_still_rejected(monkeypatch, age):
    module = leg_fixture.load_leg((ROOT/'scripts/leg_odom_tf_bridge.py').read_text())
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    module.sensor_time = lambda: 100.25
    node = module.LegOdomTfBridge()
    node.callback(leg_fixture.odometry(100.25-age))
    node.publish_latest()
    assert node.latest_message is None
    assert not node.transforms and not node.samples


def bind_source_clock(monkeypatch, node, raw):
    method = type(node).now_monotonic
    monkeypatch.setitem(method.__globals__, 'sensor_time', lambda: raw[0])
    node.now_monotonic = MethodType(method, node)


def test_manual_seed_can_confirm_ndt_in_source_domain_despite_wall_and_python_offset(monkeypatch):
    case = manual_fixture.ManualInitializationTests()
    case.setUp()
    try:
        node = case.node
        raw = [10.25]
        bind_source_clock(monkeypatch, node, raw)
        node.last_leg_time = raw[0]
        node.manual_pose_callback(case.pose(stamp=1794900000))
        assert len(node.outputs['/initialpose']) == 1
        assert node.initialpose_stamp == 10250000000
        assert not node.outputs['/patrol/localization_ready'][-1].data
        for second in (11, 12, 13):
            raw[0] = second+.25
            node.last_leg_time = raw[0]
            status = NS(header=NS(stamp=NS(sec=second, nanosec=240000000)),
                        has_converged=True, matching_error=.01, inlier_fraction=.8)
            odom = case.pose(stamp=second)
            odom.header.stamp.nanosec = 240000000
            node.status_callback(status)
            node.odom_callback(odom)
        assert node.state.startswith('READY:manual_plus_ndt_confirmed')
    finally:
        case.doCleanups()


def test_visual_seed_still_needs_distinct_quality_checked_ndt_scans(monkeypatch):
    case = init_fixture.InitializerTests()
    case.setUp()
    try:
        node, raw = case.node, [10.25]
        bind_source_clock(monkeypatch, node, raw)
        node.requested = True
        node.params.update(stable_samples=4, stable_duration_s=2.,
                           max_sample_gap_s=6., ndt_confirmation_samples=3, ndt_settle_s=0.)
        for second in (10, 11, 12, 13):
            raw[0] = second+.25
            node.now = second
            image_pose = init_fixture.PoseMessage()
            image_pose.header.stamp.sec = second
            node.visual_pose_callback(image_pose)
        assert len(node.outputs['/initialpose']) == 1
        assert not node.outputs['/patrol/localization_ready'][-1].data
        for second in (14, 15, 16):
            raw[0] = second+.25
            node.last_leg_time = raw[0]
            status = NS(header=NS(stamp=NS(sec=second, nanosec=240000000)),
                        has_converged=True, matching_error=.01, inlier_fraction=.8)
            odom = init_fixture.PoseMessage()
            odom.header.frame_id = 'map'
            odom.header.stamp = copy.deepcopy(status.header.stamp)
            node.status_callback(status)
            node.odom_callback(odom)
        assert node.state.startswith('READY:visual_plus_ndt_confirmed')
    finally:
        case.doCleanups()


def test_gate_uses_same_source_clock_and_original_half_second_timeout(monkeypatch):
    case = gate_fixture.GateTests()
    case.setUp()
    try:
        node, raw = case.gate, [10.25]
        bind_source_clock(monkeypatch, node, raw)
        stamp = NS(sec=10, nanosec=240000000)
        node.voa_costmap_callback(NS(stamp=stamp, frame_id='map'))
        assert node.last_voa_costmap_time == 10.24
        assert not node.armed
        raw[0] = 10.741
        assert raw[0]-node.last_voa_costmap_time > .5
        node.voa_costmap_callback(NS(stamp=stamp, frame_id='map'))
        assert node.voa_heartbeat_last_rejection == 'stale'
        odom = init_fixture.PoseMessage()
        odom.header.frame_id = 'map'
        odom.header.stamp.sec, odom.header.stamp.nanosec = 10, 730000000
        node.odom_callback(odom)
        assert node.last_odom_source_stamp == 10.73
        assert not node.armed
    finally:
        case.doCleanups()


@pytest.mark.parametrize('offset', [.25, -.25, 3.0])
def test_voa_retry_deadline_converts_domains_and_retains_original_stamp(monkeypatch, offset):
    case = retry_fixture.RetryTests()
    case.setUp()
    try:
        module = retry_fixture.M
        monkeypatch.setattr(module, 'sensor_time', lambda: case.clock.monotonic()+offset)
        msg = retry_fixture.fixtures.grid(stamp=case.clock.monotonic()+offset-.2)
        before = copy.deepcopy(msg.header.stamp)
        case.bridge.receive(msg)
        case.future()
        case.bridge.publish()
        candidate = case.bridge.pending[0]
        assert candidate.state == 'PENDING_TF'
        assert candidate.retry_deadline_monotonic_ns-case.clock.now_ns == 50000000
        fixed = candidate.retry_deadline_monotonic_ns
        case.clock.advance(25)
        case.ready_ns = case.clock.now_ns
        case.bridge.publish()
        assert candidate.retry_deadline_monotonic_ns == fixed
        assert case.bridge.diag_counts['retry_recovered'] == 1
        output = case.outputs[module.TOPIC].messages[-1]
        assert output.header.stamp == before
        assert msg.header.stamp == before
        diagnostics = case.bridge._projection_diagnostics(case.clock.monotonic())
        assert diagnostics['source_age_s'] == pytest.approx(.225)
        assert diagnostics['publish_gap_s'] == 0
    finally:
        case.doCleanups()
