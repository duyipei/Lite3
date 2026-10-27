"""Exercise the launcher's real ROS monitor with a deterministic message clock."""
import contextlib
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'scripts/navigation_start.sh').read_text()
MONITOR = SOURCE.split('wait_visual_ready() {', 1)[1].split("<<'PY' || result=$?\n", 1)[1].split('\nPY\n', 1)[0]


class ReadyMonitorTests(unittest.TestCase):
    def run_monitor(self, scenario):
        clock = [0.0]
        subscriptions = {}
        ns = types.SimpleNamespace

        class Node:
            def __init__(self, name):
                pass

            def create_subscription(self, kind, topic, callback, qos):
                subscriptions[topic] = callback

            def destroy_node(self):
                pass

        def spin_once(node, timeout_sec):
            clock[0] += 0.1
            for topic, callback in subscriptions.items():
                collecting = scenario == 'slow_initialization' and clock[0] < 16
                if collecting and topic == '/odom':
                    continue
                if scenario == 'missing_odom' and topic == '/odom':
                    continue
                if scenario == 'stopped_guard' and clock[0] > 1 and topic != '/odom':
                    continue
                if scenario == 'stopped_odom' and clock[0] > 1 and topic == '/odom':
                    continue
                failed = scenario == 'revoked' and clock[0] > 1
                if topic == '/odom':
                    callback(ns(pose=ns(pose=ns(position=ns(x=0.0, y=0.0),
                                               orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0)))))
                elif topic == '/patrol/localization_ready':
                    callback(ns(data=not failed and not collecting))
                elif topic == '/patrol/visual_init_state':
                    callback(ns(data='FAILED:ndt_status_stale' if failed else
                                'COLLECTING:2/10' if collecting else 'READY:confirmed'))
                # No visual messages: tracking must not depend on fresh images.

        modules = {
            'rclpy': ns(init=lambda: None, shutdown=lambda: None, ok=lambda: True, spin_once=spin_once),
            'rclpy.node': ns(Node=Node),
            'rclpy.qos': ns(QoSProfile=lambda **kw: ns(**kw),
                            DurabilityPolicy=ns(TRANSIENT_LOCAL=1), ReliabilityPolicy=ns(RELIABLE=1)),
            'nav_msgs.msg': ns(Odometry=object),
            'std_msgs.msg': ns(Bool=object, String=object),
            'time': ns(monotonic=lambda: clock[0]),
        }
        output = io.StringIO()
        with patch.dict(sys.modules, modules), patch.object(sys, 'argv', ['monitor', '20', '3', 'tracking']), \
                contextlib.redirect_stdout(output), self.assertRaises(SystemExit) as result:
            exec(compile(MONITOR, '<launcher-monitor>', 'exec'), {})
        return result.exception.code, output.getvalue()

    def test_tracking_succeeds_without_visual_messages(self):
        code, output = self.run_monitor('healthy')
        self.assertEqual(code, 0)
        self.assertIn('NDT + 里程计持续就绪 3 秒', output)

    def test_initialization_may_take_longer_than_subscription_retry_delay(self):
        code, output = self.run_monitor('slow_initialization')
        self.assertEqual(code, 0)
        self.assertNotIn('[WARN]', output)

    def test_missing_subscription_requests_reconnect_without_passing(self):
        code, output = self.run_monitor('missing_odom')
        self.assertEqual(code, 2)
        self.assertIn('odom', output)
        self.assertNotIn('[PASS]', output)

    def test_stale_ready_or_odometry_cannot_pass(self):
        for scenario in ('stopped_guard', 'stopped_odom', 'revoked'):
            with self.subTest(scenario=scenario):
                code, output = self.run_monitor(scenario)
                self.assertEqual(code, 1)
                self.assertNotIn('[PASS]', output)


if __name__ == '__main__':
    unittest.main()
