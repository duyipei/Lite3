import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    'rviz_ssh_bridge', Path(__file__).resolve().parents[1]/'scripts/rviz_ssh_bridge.py')
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class FragmentedStream(io.BytesIO):
    def read(self, size=-1):
        return super().read(min(size, 3))


class RvizTransportTests(unittest.TestCase):
    def test_fragmented_binary_messages_round_trip(self):
        stream = FragmentedStream()
        bridge.write_packet(stream, 0, b'\x00\xff\nmap')
        bridge.write_packet(stream, bridge.GOAL_ID, b'goal\x00')
        stream.seek(0)
        self.assertEqual(bridge.read_packet(stream), (0, b'\x00\xff\nmap'))
        self.assertEqual(bridge.read_packet(stream), (bridge.GOAL_ID, b'goal\x00'))
        with self.assertRaises(EOFError):
            bridge.read_packet(stream)

    def test_invalid_and_truncated_frames_fail_closed(self):
        for header in (b'garbage!!!!',
                       bridge.HEADER.pack(bridge.MAGIC, 0, bridge.MAX_PAYLOAD+1)):
            with self.assertRaises(ValueError):
                bridge.read_packet(io.BytesIO(header))
        with self.assertRaises(EOFError):
            bridge.read_packet(io.BytesIO(bridge.HEADER.pack(bridge.MAGIC, 0, 5)+b'xx'))

    def test_only_isolated_goals_can_go_to_robot(self):
        bridge.validate_direction('remote', bridge.GOAL_ID)
        for ident in range(len(bridge.TOPICS)):
            with self.assertRaises(ValueError):
                bridge.validate_direction('remote', ident)
            bridge.validate_direction('local', ident)
        for ident in (bridge.GOAL_ID, 65535):
            with self.assertRaises(ValueError):
                bridge.validate_direction('local', ident)
        self.assertEqual(bridge.GOAL_TOPIC, '/patrol/goal_request')
        self.assertFalse(any('/cmd_vel' in topic[0] for topic in bridge.TOPICS))

    def test_display_samples_coalesce_and_events_keep_order(self):
        queue = bridge.OutboundBuffer()
        queue.put(3, b'old-cloud')
        queue.put(4, b'full-costmap', ordered=True)
        queue.put(3, b'latest-cloud')
        queue.put(5, b'costmap-patch-1', ordered=True)
        queue.put(5, b'costmap-patch-2', ordered=True)
        queue.put(bridge.GOAL_ID, b'goal-1', ordered=True)
        queue.put(bridge.GOAL_ID, b'goal-2', ordered=True)
        self.assertEqual([queue.get() for _ in range(6)], [
            (3, b'latest-cloud'), (4, b'full-costmap'),
            (5, b'costmap-patch-1'), (5, b'costmap-patch-2'),
            (bridge.GOAL_ID, b'goal-1'), (bridge.GOAL_ID, b'goal-2')])
        self.assertEqual(queue.events, 0)

    def test_manual_initialpose_is_explicitly_opt_in_and_one_direction_only(self):
        with self.assertRaises(ValueError):
            bridge.validate_direction('remote', bridge.INITIALPOSE_ID)
        bridge.validate_direction('remote', bridge.INITIALPOSE_ID, True)
        with self.assertRaises(ValueError):
            bridge.validate_direction('local', bridge.INITIALPOSE_ID, True)
        self.assertEqual(bridge.INITIALPOSE_TOPIC, '/patrol/manual_initialpose_request')
        self.assertNotEqual(bridge.INITIALPOSE_TOPIC, '/initialpose')

    def test_manual_pose_is_an_ordered_operator_event(self):
        queue = bridge.OutboundBuffer()
        for ident, payload in ((bridge.INITIALPOSE_ID, b'pose1'),
                               (bridge.INITIALPOSE_ID, b'pose2'),
                               (bridge.GOAL_ID, b'goal')):
            self.assertTrue(bridge.operator_event(ident))
            queue.put(ident, payload, ordered=True)
        self.assertEqual([queue.get() for _ in range(3)], [
            (bridge.INITIALPOSE_ID, b'pose1'), (bridge.INITIALPOSE_ID, b'pose2'),
            (bridge.GOAL_ID, b'goal')])

    def endpoint(self, side, enabled):
        """Build real endpoint registrations with inert ROS, threads and SSH."""
        ns = types.SimpleNamespace
        published, subscribed = [], []

        class Node:
            def __init__(self, name):
                pass

            def create_publisher(self, kind, topic, qos):
                published.append((topic, kind))
                return ns(publish=lambda message: None)

            def create_subscription(self, kind, topic, callback, qos):
                subscribed.append((topic, kind))

            def create_timer(self, period, callback):
                pass

            def get_logger(self):
                return ns(info=lambda value: None, error=lambda value: None)

            def destroy_node(self):
                pass

        modules = {
            'rclpy': ns(init=lambda **kw: None, ok=lambda: False),
            'rclpy.node': ns(Node=Node),
            'rclpy.qos': ns(QoSProfile=lambda **kw: ns(**kw),
                DurabilityPolicy=ns(TRANSIENT_LOCAL=1, VOLATILE=2),
                ReliabilityPolicy=ns(BEST_EFFORT=1, RELIABLE=2)),
            'rclpy.serialization': ns(serialize_message=lambda value: value,
                                     deserialize_message=lambda value, kind: value),
            'rosidl_runtime_py.utilities': ns(get_message=lambda value: value),
        }
        child = ns(stdin=io.BytesIO(), stdout=io.BytesIO(), wait=lambda **kw: None)
        args = ns(side=side, allow_manual_initialpose=enabled, parent_pid=0,
                  key='unused-key', robot='unused-host', remote_script='/unused/bridge.py')
        with patch.dict(sys.modules, modules), \
                patch.object(bridge.threading, 'Thread') as threads, \
                patch.object(bridge.subprocess, 'Popen', return_value=child) as popen:
            self.assertEqual(bridge.run(args), 0)
            self.assertEqual(threads.call_count, 2)
            if side == 'remote':
                popen.assert_not_called()
            else:
                self.assertEqual('--allow-manual-initialpose' in popen.call_args[0][0][-1], enabled)
        self.assertNotIn('/initialpose', [topic for topic, _kind in published+subscribed])
        return published, subscribed

    def test_remote_endpoint_publishes_only_isolated_manual_requests(self):
        published, subscribed = self.endpoint('remote', True)
        self.assertIn((bridge.INITIALPOSE_TOPIC, 'geometry_msgs/msg/PoseWithCovarianceStamped'), published)
        self.assertNotIn(bridge.INITIALPOSE_TOPIC, [topic for topic, _kind in subscribed])

    def test_local_endpoint_subscribes_manual_requests_and_enables_peer(self):
        published, subscribed = self.endpoint('local', True)
        self.assertIn((bridge.INITIALPOSE_TOPIC, 'geometry_msgs/msg/PoseWithCovarianceStamped'), subscribed)
        self.assertNotIn(bridge.INITIALPOSE_TOPIC, [topic for topic, _kind in published])

    def test_default_endpoints_have_no_manual_initialpose_transport(self):
        for side in ('local', 'remote'):
            with self.subTest(side=side):
                published, subscribed = self.endpoint(side, False)
                self.assertNotIn(bridge.INITIALPOSE_TOPIC, [topic for topic, _kind in published+subscribed])

    def test_goal_backlog_fails_instead_of_silently_dropping(self):
        queue = bridge.OutboundBuffer()
        for _ in range(128):
            queue.put(bridge.GOAL_ID, b'goal', ordered=True)
        with self.assertRaises(OverflowError):
            queue.put(bridge.GOAL_ID, b'one-too-many', ordered=True)


if __name__ == '__main__':
    unittest.main()
