"""Compare instrumentation with committed behavior using fake ROS APIs only."""
import ast
from collections import Counter
import copy
import itertools
from pathlib import Path
import subprocess
import sys
import types
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from t22_fixtures import ROOT, recorder as R, header

BASELINE = 'bdc18a0c4495a92fa6b79731e3968676e2867b2c'


class MemoryProbe:
    def __init__(self): self.rows, self.counts = [], Counter()
    def active(self): return True
    def emit(self, kind, **fields): self.rows.append(dict(kind=kind, **fields))
    def count(self, kind): self.counts[kind] += 1
    def tick(self, now): self.emit('leg_tick', t3a_ns=now)
    def received(self, value): self.last = value


def transform():
    return NS(header=header(100000000000), child_frame_id='leg_base_link',
              transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))


class FakeNode:
    def __init__(self, name):
        self.name, self.calls, self.samples, self.transforms, self.timers = name, [], [], [], []
        self.sample_error = self.tf_error = None
    def create_publisher(self, *args):
        self.publisher_args = args
        def publish(message):
            self.calls.append('sampled_publish')
            if self.sample_error: raise self.sample_error
            self.samples.append(copy.deepcopy(message))
        return NS(publish=publish)
    def create_subscription(self, *args): self.subscription_args = args
    def create_timer(self, period, callback): self.timers.append((period, callback.__name__))
    def get_logger(self): return NS(info=lambda *v: None, warning=lambda *v: None, error=lambda *v: None)
    def get_clock(self): return NS(now=lambda: NS(to_msg=lambda: NS(sec=1793426540, nanosec=123)))


class FakeBroadcaster:
    def __init__(self, node): self.node = node
    def sendTransform(self, value):
        self.node.calls.append('sendTransform')
        if self.node.tf_error: raise self.node.tf_error
        self.node.transforms.append(copy.deepcopy(value))


def load_leg(source):
    def module(name, **attributes):
        value = types.ModuleType(name)
        value.__dict__.update(attributes)
        return value
    modules = {
        'rclpy': module('rclpy'), 'rclpy.node': module('rclpy.node', Node=FakeNode),
        'rclpy.qos': module('rclpy.qos', QoSProfile=lambda depth: NS(depth=depth),
                           ReliabilityPolicy=NS(BEST_EFFORT='BEST_EFFORT'),
                           DurabilityPolicy=NS(VOLATILE='VOLATILE')),
        'geometry_msgs': module('geometry_msgs'),
        'geometry_msgs.msg': module('geometry_msgs.msg', TransformStamped=transform),
        'nav_msgs': module('nav_msgs'), 'nav_msgs.msg': module('nav_msgs.msg', Odometry=NS),
        'tf2_ros': module('tf2_ros', TransformBroadcaster=FakeBroadcaster)}
    result = module('fake_leg_node')
    with patch.dict(sys.modules, modules): exec(compile(source, '<mock leg source>', 'exec'), result.__dict__)
    return result


def odometry(stamp=99.99):
    nanoseconds = round(stamp*1000000000)
    return NS(header=header(nanoseconds), pose=NS(pose=NS(
        position=NS(x=1., y=2., z=.2), orientation=NS(x=0., y=0., z=0., w=1.))))


class LegInstrumentationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_source = subprocess.check_output(
            ['git','show',BASELINE+':scripts/leg_odom_tf_bridge.py'], cwd=ROOT, text=True)
        cls.current_source = (ROOT/'scripts/leg_odom_tf_bridge.py').read_text()
        cls.original = load_leg(cls.original_source)
        cls.current = load_leg(cls.current_source)

    def setUp(self):
        self.clock = patch.object(self.current.time, 'monotonic', return_value=100.)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        sequence = itertools.count(100000000000, 1000)
        clock_ns = patch.object(self.current.time, 'monotonic_ns', side_effect=lambda: next(sequence))
        clock_ns.start()
        self.addCleanup(clock_ns.stop)
        factory = patch.object(R, 'create_probe', return_value=None)
        factory.start()
        self.addCleanup(factory.stop)

    def nodes(self, diagnostic=True):
        nodes = self.original.LegOdomTfBridge(), self.current.LegOdomTfBridge()
        if diagnostic: nodes[1]._t22 = MemoryProbe()
        return nodes

    def assert_equivalent(self, nodes):
        first, second = nodes
        for attr in ('calls', 'samples', 'transforms', 'last_source_stamp', 'last_published_stamp',
                     'last_message', 'latest_message', 'timers'):
            self.assertEqual(getattr(first, attr), getattr(second, attr), attr)
        self.assertEqual(first.subscription_args[-1], second.subscription_args[-1])
        self.assertEqual(first.publisher_args, second.publisher_args)

    def test_valid_input_headers_call_order_and_duplicate_publish_unchanged(self):
        nodes = self.nodes()
        for node in nodes:
            source = odometry()
            node.callback(source)
            self.assertIsNone(node.publish_latest())
            self.assertIsNone(node.publish_latest())
            self.assertEqual(source.header.stamp.sec, 99)
            self.assertEqual(node.transforms[0].header.stamp, source.header.stamp)
            self.assertEqual(node.samples[0].header.stamp.sec, 1793426540)
        self.assert_equivalent(nodes)
        self.assertEqual(nodes[1].calls, ['sampled_publish','sendTransform'])
        row = next(r for r in nodes[1]._t22.rows if r['kind']=='leg_tx')
        times = [row[k] for k in ('t2_enter_ns','t2_valid_ns','cached_ns','t3a_ns',
                 'copy_begin_ns','copy_end_ns','sampled_begin_ns','t3b_ns','t3c_ns','t3d_ns')]
        self.assertEqual(times, sorted(times))

    def test_disabled_diagnostics_preserve_business_behavior(self):
        nodes = self.nodes(diagnostic=False)
        for node in nodes:
            node.callback(odometry())
            node.publish_latest()
        self.assertIsNone(nodes[1]._t22)
        self.assert_equivalent(nodes)

    def test_validation_boundaries_and_rejections_unchanged(self):
        messages = [odometry(stamp) for stamp in (99.5, 99.499, 100.05, 100.051, 99.99)]
        invalid = odometry(); invalid.header.stamp.nanosec = 1000000000; messages.append(invalid)
        invalid = odometry(); invalid.pose.pose.position.x = float('inf'); messages.append(invalid)
        invalid = odometry(); invalid.pose.pose.orientation.w = 2.; messages.append(invalid)
        for message in messages:
            with self.subTest(message=repr(message)):
                nodes = self.nodes()
                for node in nodes:
                    node.callback(copy.deepcopy(message))
                    node.publish_latest()
                self.assert_equivalent(nodes)

    def test_input_overwrite_duplicate_and_out_of_order_decisions_unchanged(self):
        nodes = self.nodes()
        for node in nodes:
            for stamp in (99.97, 99.98, 99.98, 99.96, 99.99): node.callback(odometry(stamp))
            node.publish_latest()
        self.assert_equivalent(nodes)
        counts = nodes[1]._t22.counts
        self.assertEqual(counts['input.accepted'], 3)
        self.assertEqual(counts['input.overwritten_before_tf'], 2)
        self.assertEqual(counts['input.rejected.duplicate'], 1)
        self.assertEqual(counts['input.rejected.out_of_order'], 1)

    def test_send_transform_exception_is_same_object_and_original_state_unchanged(self):
        nodes = self.nodes()
        failure = RuntimeError('original sendTransform exception')
        for node in nodes:
            node.callback(odometry())
            node.tf_error = failure
            with self.assertRaises(RuntimeError) as caught: node.publish_latest()
            self.assertIs(caught.exception, failure)
            self.assertIsNone(node.last_published_stamp)
        self.assert_equivalent(nodes)
        self.assertEqual(nodes[1]._t22.rows[-1]['error_type'], 'RuntimeError')

    def test_sampled_publish_exception_prevents_tf_in_both_versions(self):
        nodes = self.nodes()
        failure = ValueError('original auxiliary publisher exception')
        for node in nodes:
            node.callback(odometry())
            node.sample_error = failure
            with self.assertRaises(ValueError) as caught: node.publish_latest()
            self.assertIs(caught.exception, failure)
        self.assert_equivalent(nodes)
        self.assertFalse(nodes[1].transforms)

    def test_timer_periods_qos_and_spin_configuration_are_unchanged(self):
        nodes = self.nodes()
        self.assertEqual(nodes[1].timers, [(.05,'publish_latest'),(1.,'check_freshness')])
        self.assert_equivalent(nodes)
        trees = [ast.parse(source) for source in (self.original_source,self.current_source)]
        calls = lambda tree: [ast.dump(n) for n in ast.walk(tree) if isinstance(n,ast.Call)
            and isinstance(n.func,ast.Attribute) and n.func.attr in ('spin','create_timer','create_subscription')]
        self.assertEqual(calls(trees[0]), calls(trees[1]))


class ReceiverInstrumentationTests(unittest.TestCase):
    def setUp(self):
        self.probe = MemoryProbe()

        class Buffer:
            def __init__(self, *args, **kwargs):
                self.args, self.kwargs, self.calls = args, kwargs, []
                self.result, self.error = object(), None
            def set_transform(self, transform, authority):
                self.calls.append((transform,authority))
                if self.error: raise self.error
                return self.result

        class Listener:
            def __init__(self, buffer, node, *args, **kwargs):
                self.buffer, self.node, self.args, self.kwargs = buffer, node, args, kwargs
            def callback(self, data):
                for value in data.transforms: self.buffer.set_transform(value, 'original authority')
                return 'original callback return'

        self.base_buffer, self.base_listener = Buffer, Listener
        self.Buffer, self.Listener = R.receiver_types(Buffer, Listener, self.probe)

    def test_buffer_normal_return_and_call_count_preserved(self):
        buffer = self.Buffer('cache argument', marker='original keyword')
        value = transform()
        self.assertIs(buffer.set_transform(value,'original authority'), buffer.result)
        self.assertEqual(buffer.calls, [(value,'original authority')])
        self.assertEqual(buffer.args, ('cache argument',))
        self.assertEqual(buffer.kwargs, dict(marker='original keyword'))
        row = self.probe.rows[0]
        self.assertEqual(row['source_stamp_ns'], R.stamp_ns(value.header))
        self.assertFalse(row['queryability_checked'])

    def test_buffer_exception_identity_and_propagation_preserved(self):
        buffer = self.Buffer()
        failure = RuntimeError('native insertion error')
        buffer.error = failure
        with self.assertRaises(RuntimeError) as caught: buffer.set_transform(transform(),'authority')
        self.assertIs(caught.exception, failure)
        self.assertEqual(len(buffer.calls), 1)
        self.assertEqual(self.probe.rows[0]['outcome'], 'RuntimeError')

    def test_listener_records_real_callback_context_and_preserves_return(self):
        buffer = self.Buffer()
        listener = self.Listener(buffer, 'original node', spin_thread=False)
        self.assertEqual(listener.callback(NS(transforms=[transform()])), 'original callback return')
        row = self.probe.rows[0]
        self.assertLessEqual(row['t4a_ns'], row['t4b_ns'])
        self.assertLessEqual(row['t4b_ns'], row['t4c_ns'])
        self.assertEqual(row['batch_tf_count'], 1)
        self.assertEqual(listener.kwargs, dict(spin_thread=False))
        self.assertEqual(len(buffer.calls), 1)

    def test_listener_exception_still_propagates_without_duplicate_api_call(self):
        buffer = self.Buffer()
        listener = self.Listener(buffer, 'node')
        failure = LookupError('same error')
        buffer.error = failure
        with self.assertRaises(LookupError) as caught: listener.callback(NS(transforms=[transform()]))
        self.assertIs(caught.exception, failure)
        self.assertEqual(len(buffer.calls), 1)
        self.assertEqual(self.probe.rows[-1]['kind'], 'listener_callback')

    def test_unrelated_tf_not_recorded_and_base_classes_not_patched(self):
        buffer = self.Buffer()
        listener = self.Listener(buffer, 'node')
        value = transform(); value.child_frame_id = 'camera_link'
        self.assertEqual(listener.callback(NS(transforms=[value])), 'original callback return')
        self.assertFalse(self.probe.rows)
        self.assertEqual(len(buffer.calls), 1)
        self.assertNotIn('Timed', self.base_buffer.__name__)
        self.assertNotIn('Timed', self.base_listener.__name__)

    def test_inactive_receiver_has_no_records_and_calls_original_once(self):
        self.probe.active = lambda: False
        buffer = self.Buffer()
        listener = self.Listener(buffer, 'node')
        listener.callback(NS(transforms=[transform()]))
        self.assertFalse(self.probe.rows)
        self.assertEqual(len(buffer.calls), 1)


class ProjectionDiagnosticTests(unittest.TestCase):
    def setUp(self):
        # Existing pure-Python fixtures; they do not construct a ROS node.
        import test_voa_projection_freshness as fixtures
        self.fixtures = fixtures
        self.case = fixtures.ProjectionFreshnessTests()
        self.case.setUp()
        self.addCleanup(self.case.doCleanups)
        self.events = []
        self.case.bridge._t22 = NS(miss=lambda *args: self.events.append(args))

    def test_anchor_and_grid_failures_are_distinguished_without_extra_queries(self):
        self.case.prepare_anchors(missing=(100.1,))
        self.case.bridge.publish()
        self.assertEqual([event[0] for event in self.events], ['map_grid','leg_anchor'])
        self.assertEqual(self.case.queries,
            [('map',100.2),('robot_odom',100.1),('robot_odom',100.),('robot_odom',100.2)])
        self.assertEqual(len(self.case.clouds()), 1)
        self.case.queries.clear()
        self.events.clear()
        self.case.prepare_anchors(missing=(100.2,))
        with self.assertRaises(ValueError):
            self.case.bridge.projection_transform(self.fixtures.grid(100.2))
        self.assertEqual([event[0] for event in self.events], ['map_grid','leg_grid'])
        self.assertEqual(self.case.queries.count(('robot_odom', 100.2)), 1)
        for stage, grid_header, requested_header, error in self.events:
            self.assertIsInstance(error, self.fixtures.TfUnavailable)
            if stage == 'leg_grid': self.assertEqual(grid_header.stamp, requested_header.stamp)

    def test_exact_transform_priority_has_no_added_tf_lookups_or_fallback(self):
        self.case.bridge.receive(self.fixtures.grid(100.1))
        self.case.bridge.publish()
        self.assertEqual(self.case.queries, [('map',100.1)])
        self.assertFalse(self.events)
        self.assertEqual(len(self.case.clouds()), 1)


if __name__ == '__main__': unittest.main()
