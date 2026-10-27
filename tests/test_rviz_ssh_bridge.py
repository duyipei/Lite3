import importlib.util
import io
from pathlib import Path
import unittest


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

    def test_goal_backlog_fails_instead_of_silently_dropping(self):
        queue = bridge.OutboundBuffer()
        for _ in range(128):
            queue.put(bridge.GOAL_ID, b'goal', ordered=True)
        with self.assertRaises(OverflowError):
            queue.put(bridge.GOAL_ID, b'one-too-many', ordered=True)


if __name__ == '__main__':
    unittest.main()
