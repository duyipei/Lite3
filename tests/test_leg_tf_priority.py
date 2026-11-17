"""Execute the actual leg bridge with mock ROS messages and deterministic time.

No ROS initialization, DDS, robot, mode switch or motion command is used.
The delay matrix compares the saved Git baseline with the current source.
"""
import ast
import copy
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest

import test_t22_instrumentation as F


ROOT = F.ROOT
BASELINE = subprocess.check_output(['git', 'show',
    'b9f81142f6dee3a43a9c62582b3890dc2db92ac5:scripts/leg_odom_tf_bridge.py'],
    cwd=ROOT, text=True)


class Clock:
    def __init__(self): self.ns = 100000000000
    def monotonic(self): return self.ns/1e9
    def monotonic_ns(self): return self.ns
    def advance(self, ms): self.ns += round(ms*1000000)
    def graph(self):
        stamp = 1793426540000000000+self.ns-100000000000
        return NS(sec=stamp//1000000000, nanosec=stamp % 1000000000)


def make_node(*, baseline=False, diagnostics=False, delay_ms=0):
    clock = Clock()
    class Node(F.FakeNode):
        def __init__(self, name):
            super().__init__(name)
            self.log, self.graph_reads, self.tf_entries, self.sample_entries = [], 0, [], []
        def get_logger(self):
            return NS(**{level: (lambda text, level=level: self.log.append((level, text)))
                         for level in ('info', 'warning', 'error')})
        def get_clock(self):
            self.graph_reads += 1
            return NS(now=lambda: NS(to_msg=clock.graph))
        def create_publisher(self, *args):
            publisher = super().create_publisher(*args)
            def publish(value):
                self.sample_entries.append(clock.ns)
                clock.advance(delay_ms)
                return publisher.publish(value)
            return NS(publish=publish)
    class Broadcaster(F.FakeBroadcaster):
        def sendTransform(self, value):
            self.node.tf_entries.append(clock.ns)
            return super().sendTransform(value)
    source = BASELINE if baseline else (ROOT/'scripts/leg_odom_tf_bridge.py').read_text()
    with patch.object(F, 'FakeNode', Node), patch.object(F, 'FakeBroadcaster', Broadcaster):
        module = F.load_leg(source)
    module.time = clock
    with patch.object(F.R, 'create_probe', return_value=None):
        node = module.LegOdomTfBridge()
    if diagnostics: node._t22 = F.MemoryProbe()
    return node, module, clock


def source():
    value = F.odometry()
    value.child_frame_id = 'original_leg'
    value.twist = NS(twist=NS(linear=NS(x=.1, y=.02, z=0.), angular=NS(x=0., y=0., z=.3)))
    value.pose.covariance = list(range(36))
    return value


def test_normal_order_raw_stamp_full_auxiliary_content_and_no_duplicate():
    node, _, clock = make_node(diagnostics=True)
    value = source()
    before = copy.deepcopy(value)
    graph = clock.graph()
    node.callback(value)
    node.publish_latest()
    node.publish_latest()
    assert node.calls == ['sendTransform', 'sampled_publish']
    assert node.last_published_stamp == node.last_sampled_stamp == (99, 990000000)
    assert value == before
    assert node.transforms[0].header.stamp == before.header.stamp
    assert node.transforms[0].header.frame_id == 'robot_odom'
    assert node.transforms[0].child_frame_id == 'leg_base_link'
    assert node.transforms[0].transform.rotation == before.pose.pose.orientation
    expected = copy.deepcopy(before)
    expected.header.stamp = graph
    assert node.samples == [expected]
    assert node.graph_reads == 1


def test_no_message_and_no_new_stamp_do_not_send_any_extra_tf():
    node, _, clock = make_node()
    node.publish_latest()
    assert not node.calls
    node.callback(source())
    node.publish_latest()
    for _ in range(5):
        clock.advance(50)
        node.publish_latest()
    assert len(node.transforms) == len(node.samples) == 1


@pytest.mark.parametrize('stamp', [99.99, 99.98])
def test_duplicate_and_reverse_input_cannot_replace_latest(stamp):
    node, _, _ = make_node()
    value = source()
    node.callback(value)
    node.callback(F.odometry(stamp))
    assert node.latest_message is value
    node.publish_latest()
    assert len(node.transforms) == 1


@pytest.mark.parametrize('kind', ['stale', 'future', 'nanosec', 'position', 'quaternion'])
def test_invalid_source_still_cannot_publish(kind):
    node, _, _ = make_node(diagnostics=True)
    value = source()
    if kind == 'stale': value = F.odometry(99.499)
    if kind == 'future': value = F.odometry(100.051)
    if kind == 'nanosec': value.header.stamp.nanosec = 1000000000
    if kind == 'position': value.pose.pose.position.x = float('nan')
    if kind == 'quaternion': value.pose.pose.orientation.w = 2.
    node.callback(value)
    node.publish_latest()
    assert not node.calls and node.last_published_stamp is None


def test_auxiliary_failure_retry_keeps_original_graph_stamp_without_tf_or_copy_repeat():
    node, module, clock = make_node(diagnostics=True)
    value = source()
    node.callback(value)
    failure = ValueError('sample publish failed')
    node.sample_error = failure
    original_copy = copy.deepcopy
    copied = []
    def observe_copy(obj, *args, **kwargs):
        if obj is value: copied.append(clock.ns)
        return original_copy(obj, *args, **kwargs)
    with patch.object(module.copy, 'deepcopy', side_effect=observe_copy):
        node.publish_latest()
        assert node.last_published_stamp == (99, 990000000)
        assert node.last_sampled_stamp is None
        first_graph = copy.copy(node._pending_sampled[0])
        assert len(node.transforms) == 1 and node._sampled_failed
        clock.advance(50)
        node.sample_error = None
        node.publish_latest()
        node.publish_latest()
    assert node.last_sampled_stamp == node.last_published_stamp
    assert len(node.transforms) == len(node.samples) == len(copied) == 1
    assert node.graph_reads == 1
    assert node.samples[0].header.stamp == first_graph
    assert not node._sampled_failed and node._pending_sampled is None
    assert any(level == 'error' for level, _ in node.log)
    assert any('recovered' in text for _, text in node.log)
    rows = node._t22.rows
    assert len([r for r in rows if r['kind'] == 'leg_tx']) == 1
    assert len([r for r in rows if r['kind'] == 'leg_tx_error']) == 1


def test_copy_failure_keeps_sent_tf_and_fixed_graph_time_then_recovers():
    node, module, clock = make_node()
    value = source()
    node.callback(value)
    original = copy.deepcopy
    def broken_copy(obj, *args, **kwargs):
        if obj is value: raise RuntimeError('copy failed')
        return original(obj, *args, **kwargs)
    with patch.object(module.copy, 'deepcopy', side_effect=broken_copy):
        node.publish_latest()
    assert len(node.transforms) == 1 and not node.samples
    first_graph = node._pending_sampled[0]
    clock.advance(50)
    node.publish_latest()
    assert node.samples[0].header.stamp == first_graph
    assert len(node.transforms) == 1 and node.graph_reads == 1


def test_auxiliary_failure_new_source_supersedes_old_retry():
    node, _, clock = make_node()
    node.sample_error = RuntimeError('aux fail')
    node.callback(source())
    node.publish_latest()
    clock.advance(50)
    value = F.odometry(100.04)
    value.pose.pose.position.x = 8.
    node.callback(value)
    node.sample_error = None
    node.publish_latest()
    assert len(node.transforms) == 2 and len(node.samples) == 1
    assert node.samples[0].pose.pose.position.x == 8.
    assert node.last_sampled_stamp == (100, 40000000)


def test_expired_auxiliary_retry_cannot_refresh_graph_time_or_initializer_data():
    node, _, clock = make_node()
    node.sample_error = RuntimeError('aux fail')
    node.callback(source())
    node.publish_latest()
    clock.advance(501)
    node.sample_error = None
    node.publish_latest()
    node.check_freshness()
    assert not node.samples and len(node.transforms) == 1 and node.graph_reads == 1
    assert node.last_sampled_stamp is None
    assert any('freshness is not refreshed' in text for _, text in node.log)


def test_tf_failure_propagates_without_marking_success_or_sending_auxiliary():
    node, _, _ = make_node(diagnostics=True)
    node.callback(source())
    failure = RuntimeError('send failed')
    node.tf_error = failure
    with pytest.raises(RuntimeError) as caught: node.publish_latest()
    assert caught.value is failure
    assert node.last_published_stamp is node.last_sampled_stamp is None
    assert not node.samples and not node.transforms and not node.graph_reads
    assert node._t22.rows[-1]['stage'] == 'sendTransform'
    node.tf_error = None
    node.publish_latest()
    assert len(node.transforms) == len(node.samples) == 1


def test_t22_order_boundaries_and_diagnostics_do_not_change_outputs():
    plain, _, _ = make_node()
    traced, _, _ = make_node(diagnostics=True)
    for node in (plain, traced):
        node.callback(source())
        node.publish_latest()
    assert plain.calls == traced.calls
    assert plain.transforms == traced.transforms and plain.samples == traced.samples
    row = next(r for r in traced._t22.rows if r['kind'] == 'leg_tx')
    keys = ['t3a_ns', 'tf_build_begin_ns', 't3c_ns', 't3d_ns', 'copy_begin_ns',
            'copy_end_ns', 'sampled_begin_ns', 't3b_ns']
    values = [row[k] for k in keys]
    assert values == sorted(values)
    assert row['publish_order'] == 'tf_first'


def test_copy_cost_is_removed_from_tf_critical_path_without_extra_copy():
    results = {}
    original = copy.deepcopy
    for baseline in (True, False):
        node, module, clock = make_node(baseline=baseline, diagnostics=True)
        value = source()
        copied = []
        def expensive_copy(obj, *args, **kwargs):
            if obj is value:
                copied.append(clock.ns)
                clock.advance(6)
            return original(obj, *args, **kwargs)
        node.callback(value)
        start = clock.ns
        with patch.object(module.copy, 'deepcopy', side_effect=expensive_copy):
            node.publish_latest()
        assert len(copied) == len(node.transforms) == len(node.samples) == 1
        results[baseline] = node.tf_entries[0]-start
    assert results[True] == 6000000 and results[False] == 0


def test_sampling_period_qos_and_validation_unchanged_except_correct_source_clock():
    before = ast.parse(BASELINE)
    after = ast.parse((ROOT/'scripts/leg_odom_tf_bridge.py').read_text())
    def callback(tree):
        return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'callback')
    class SourceClock(ast.NodeTransformer):
        def visit_Call(self, node):
            if isinstance(node.func, ast.Name) and node.func.id == 'sensor_time':
                node.func = ast.Attribute(value=ast.Name(id='time', ctx=ast.Load()),
                                          attr='monotonic', ctx=ast.Load())
            return self.generic_visit(node)
    assert ast.dump(callback(before)) == ast.dump(SourceClock().visit(callback(after)))
    node, _, _ = make_node()
    assert node.timers == [(.05, 'publish_latest'), (1., 'check_freshness')]
    assert node.subscription_args[-1].depth == 1
    assert node.subscription_args[-1].reliability == 'BEST_EFFORT'
    assert node.publisher_args[-1] == 2


@pytest.mark.parametrize('delay_ms', [0, 10, 30, 100])
def test_auxiliary_delay_matrix(tmp_path, delay_ms):
    rows = {}
    for label, baseline in [('before', True), ('after', False)]:
        node, _, clock = make_node(baseline=baseline, delay_ms=delay_ms, diagnostics=True)
        beginning = clock.ns
        entry_times = []
        for tick in range(5):
            # Model serial timer callbacks: a 100ms auxiliary call prevents the
            # next 50ms tick from running until it returns. This is not RMW.
            clock.ns = max(clock.ns, beginning+tick*50000000)
            node.callback(F.odometry(clock.monotonic()-.01))
            entry_times.append(clock.ns)
            node.publish_latest()
            node.publish_latest()
        rows[label] = dict(timer_to_tf_ms=(node.tf_entries[0]-entry_times[0])/1e6,
                           tf_intervals_ms=[(b-a)/1e6 for a,b in zip(node.tf_entries,node.tf_entries[1:])],
                           tf_count=len(node.transforms), auxiliary_count=len(node.samples))
    assert rows['before']['timer_to_tf_ms'] == delay_ms
    assert rows['after']['timer_to_tf_ms'] == 0
    assert rows['before']['tf_count'] == rows['after']['tf_count'] == 5
    assert rows['before']['tf_intervals_ms'] == rows['after']['tf_intervals_ms'] == [max(50,delay_ms)]*4
    (tmp_path/'leg_delay.json').write_text(json.dumps(dict(scope='deterministic Mock, not Jetson timing',
                                                       delay_ms=delay_ms, cases=rows), indent=2))
