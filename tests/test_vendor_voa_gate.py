"""Exercise the actual Python velocity gate without ROS or network."""
import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav'
NS = types.SimpleNamespace


class Twist:
    def __init__(self):
        self.linear = NS(x=0., y=0., z=0.)
        self.angular = NS(x=0., y=0., z=0.)


class Stamped:
    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.twist = Twist()


class Node:
    def __init__(self, name):
        self.params = {}
        self.outputs = {}
        self.clock_time = 10.
        self.counts = {'/cmd_vel': 0, '/cmd_vel_corrected': 1, '/patrol/voa/corrected': 1, '/patrol/voa/input': 1}
    def declare_parameter(self, key, value): self.params[key] = True if key == 'use_vendor_voa' else value
    def get_parameter(self, key): return NS(value=self.params[key])
    def create_publisher(self, kind, topic, qos):
        self.outputs[topic] = []
        return NS(publish=lambda msg: self.outputs[topic].append(copy.deepcopy(msg)))
    def create_subscription(self, *args): pass
    def create_service(self, *args): pass
    def create_timer(self, *args): pass
    def count_publishers(self, topic): return self.counts.get(topic, 0)
    def get_logger(self): return NS(info=lambda s: None, warn=lambda s: None, error=lambda s: None)
    def get_clock(self): return NS(now=lambda: NS(to_msg=lambda: NS(sec=10, nanosec=0)))


def mod(name, **kwargs):
    result = types.ModuleType(name)
    result.__dict__.update(kwargs)
    return result


class GateTests(unittest.TestCase):
    def setUp(self):
        fake = {
            'rclpy': mod('rclpy'), 'rclpy.node': mod('rclpy.node', Node=Node),
            'rclpy.qos': mod('rclpy.qos', QoSProfile=lambda **kw: NS(),
                DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1, BEST_EFFORT=2)),
            'geometry_msgs.msg': mod('geometry_msgs.msg', Twist=Twist, TwistStamped=Stamped),
            'nav_msgs.msg': mod('nav_msgs.msg', Odometry=NS),
            'std_msgs.msg': mod('std_msgs.msg', Bool=NS, Empty=NS, Header=NS, String=NS),
            'std_srvs.srv': mod('std_srvs.srv', SetBool=NS, Trigger=NS),
            'voa_gate_test': mod('voa_gate_test', __path__=[str(PACKAGE)]),
        }
        self.patch = patch.dict(sys.modules, fake)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        spec = importlib.util.spec_from_file_location('voa_gate_test.velocity_gate', PACKAGE/'velocity_gate.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.gate = module.VelocityGate()
        self.gate.now_monotonic = lambda: self.gate.clock_time
        self.gate.localization_ready = True
        self.gate.last_ready_time = self.gate.last_odom_time = self.gate.last_lidar_time = 10.

    def output(self):
        v = self.gate.outputs['/cmd_vel_corrected'][-1]
        return v.linear.x, v.linear.y, v.angular.z

    def respond(self, vx=0., w=0.):
        reply = copy.deepcopy(self.gate.outputs['/patrol/voa/input'][-1])
        reply.twist.linear.x, reply.twist.angular.z = vx, w
        self.gate.voa_callback(reply)

    def arm(self):
        self.gate.goal_pending = True
        return self.gate.arm_callback(NS(data=True), NS())

    def test_no_voa_cannot_arm(self):
        self.assertFalse(self.arm().success)
        self.gate.timer_callback()
        self.assertEqual(self.output(), (0, 0, 0))

    def enable_costmap(self):
        self.gate.require_voa_costmap = True
        self.gate.counts.update({'/patrol/voa/costmap_heartbeat': 1,
                                 '/patrol/voa/costmap_obstacles': 1})

    def test_voa_braking_alone_cannot_arm_when_planner_bridge_required(self):
        self.enable_costmap()
        self.gate.timer_callback(); self.respond()
        self.assertFalse(self.arm().success)
        self.assertEqual(self.gate.sensor_health()[1], 'voa_costmap_not_ready_or_stale')
        self.gate.voa_costmap_callback(NS(frame_id='map', stamp=NS(sec=10, nanosec=0)))
        self.assertTrue(self.arm().success)

    def test_bridge_loss_stops_and_latches_even_when_voa_is_healthy(self):
        self.enable_costmap()
        self.gate.timer_callback(); self.respond()
        self.gate.voa_costmap_callback(NS(frame_id='map', stamp=NS(sec=10, nanosec=0)))
        self.assertTrue(self.arm().success)
        self.gate.goal_active_callback(NS(data=True))
        self.gate.clock_time = 10.6
        self.gate.timer_callback(); self.respond()
        self.assertFalse(self.gate.armed)
        self.assertEqual(self.gate.fault, 'voa_costmap_not_ready_or_stale')
        self.assertEqual(self.output(), (0, 0, 0))
        self.gate.voa_costmap_callback(NS(frame_id='map', stamp=NS(sec=10, nanosec=600000000)))
        self.assertFalse(self.arm().success)

    def test_bridge_replay_wrong_frame_future_and_conflict_rejected(self):
        self.enable_costmap()
        for frame, sec, nano in [('map', 9, 0), ('map', 11, 0), ('base_link', 10, 0),
                                  ('map', 10, 1000000000)]:
            self.gate.voa_costmap_callback(NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nano)))
            self.assertEqual(self.gate.last_voa_costmap_time, 0)
        header = NS(frame_id='map', stamp=NS(sec=10, nanosec=0))
        self.gate.voa_costmap_callback(header)
        self.gate.clock_time = 10.6
        self.gate.voa_costmap_callback(header)
        self.assertFalse(self.gate.sensor_health()[0])
        self.gate.clock_time = 10.1
        self.gate.counts['/patrol/voa/costmap_obstacles'] = 2
        self.assertEqual(self.gate.sensor_health()[1], 'voa_costmap_publisher_conflict')

    def test_disarmed_sends_zero_probe_never_motion(self):
        cmd = Twist(); cmd.linear.x = .1
        self.gate.raw_callback(cmd)
        self.gate.timer_callback()
        self.respond(.1)
        self.gate.timer_callback()
        self.assertEqual(self.output(), (0, 0, 0))
        self.assertFalse(self.arm().success)

    def test_correction_used_only_after_arm_and_active_goal(self):
        self.gate.timer_callback(); self.respond()
        self.assertTrue(self.arm().success)
        self.gate.goal_active_callback(NS(data=True))
        cmd = Twist(); cmd.linear.x = .1
        self.gate.raw_callback(cmd)
        self.gate.timer_callback()
        self.assertTrue(self.gate.armed)
        self.assertEqual(self.output(), (0, 0, 0))
        self.respond(.06, 0)
        self.gate.timer_callback()
        self.assertEqual(self.output(), (.06, 0, 0))
        self.gate.arm_callback(NS(data=False), NS())
        self.gate.timer_callback()
        self.assertEqual(self.output(), (0, 0, 0))

    def test_voa_cannot_override_nav2_steering(self):
        self.gate.timer_callback(); self.respond()
        self.assertTrue(self.arm().success)
        self.gate.goal_active_callback(NS(data=True))
        cmd = Twist(); cmd.linear.x = .1
        self.gate.raw_callback(cmd)
        self.gate.timer_callback(); self.respond(.06, .2)
        self.gate.timer_callback()
        self.assertEqual(self.output(), (0, 0, 0))
        self.assertIn('VOA_BRAKING', self.gate.outputs['/patrol/gate_state'][-1].data)
        # A fresh, same-path safe response resumes without a new ARM.
        self.respond(.1, 0); self.gate.timer_callback()
        self.assertEqual(self.output(), (.1, 0, 0))

    def test_input_publisher_conflict_blocks_arm(self):
        self.gate.timer_callback(); self.respond()
        self.gate.counts['/patrol/voa/input'] = 2
        self.assertFalse(self.arm().success)

    def test_voa_disconnect_disarms(self):
        self.gate.timer_callback(); self.respond()
        self.assertTrue(self.arm().success)
        self.gate.clock_time += .3
        self.gate.timer_callback()
        self.assertFalse(self.gate.armed)
        self.assertIn('voa', self.gate.fault)
        self.assertEqual(self.output(), (0, 0, 0))

    def test_zero_probe_in_flight_survives_arm_and_goal_transition(self):
        self.gate.timer_callback(); self.respond()
        self.gate.clock_time = 10.12
        self.gate.timer_callback()
        # ARM and accepted-goal callbacks both reset motion corrections. The
        # already-issued zero probe must still refresh health at its own age.
        self.assertTrue(self.arm().success)
        self.gate.clock_time = 10.18
        self.gate.goal_active_callback(NS(data=True))
        self.gate.clock_time = 10.22
        self.respond()
        self.gate.clock_time = 10.28
        self.gate.timer_callback()
        self.assertTrue(self.gate.armed)
        self.assertEqual(self.output(), (0, 0, 0))
        self.gate.clock_time = 10.38
        self.gate.timer_callback()
        self.assertFalse(self.gate.armed)  # still expires 250 ms from request

    def test_public_command_conflict_blocks_arm(self):
        self.gate.timer_callback(); self.respond()
        self.gate.counts['/cmd_vel'] = 1
        self.assertFalse(self.arm().success)

    def prepare_command_wait(self):
        from importlib.util import spec_from_file_location, module_from_spec
        spec = spec_from_file_location('recovery_budget', ROOT/'scripts/navigation_recovery_budget.py')
        module = module_from_spec(spec); spec.loader.exec_module(module)
        n = self.gate
        n.params['nav_command_wait_timeout_s'] = module.command_wait_budget(ROOT/'navigation/navigate_dynamic_obstacles.xml')
        n.use_vendor_voa = False  # Exercise the independent command watchdog.
        self.assertTrue(self.arm().success)
        n.goal_active_callback(NS(data=True))
        return n

    def healthy_tick(self, now):
        n = self.gate
        n.clock_time = now
        n.last_ready_time = n.last_odom_time = n.last_lidar_time = now
        n.timer_callback()

    def test_all_eight_waits_plus_planning_remain_armed_at_zero_then_resume(self):
        n = self.prepare_command_wait()
        # Eight 3 s waits with 2 s planning per attempt, plus final planning.
        for elapsed in range(43):
            self.healthy_tick(10+elapsed)
            self.assertTrue(n.armed)
            self.assertEqual(self.output(), (0, 0, 0))
            self.assertIn('WAITING_NAV_COMMAND', n.outputs['/patrol/gate_state'][-1].data)
        cmd = Twist(); cmd.linear.x = .1
        n.raw_callback(cmd); n.timer_callback()
        self.assertEqual(self.output(), (.1, 0, 0))

    def test_old_motion_stops_after_half_second_and_stall_is_bounded(self):
        n = self.prepare_command_wait()
        cmd = Twist(); cmd.linear.x = .1
        n.raw_callback(cmd); n.timer_callback()
        self.assertEqual(self.output(), (.1, 0, 0))
        self.healthy_tick(10.51)
        self.assertTrue(n.armed)
        self.assertEqual(self.output(), (0, 0, 0))
        # Repeated active heartbeats must not extend a hung goal's budget.
        for elapsed in (20, 40, 60):
            n.goal_active_callback(NS(data=True))
            self.healthy_tick(10+elapsed)
            self.assertTrue(n.armed)
            self.assertEqual(self.output(), (0, 0, 0))
        self.healthy_tick(70.01)
        self.assertFalse(n.armed)
        self.assertEqual(n.fault, 'nav_command_timeout')
        n.raw_callback(cmd); n.timer_callback()
        self.assertFalse(n.armed)
        self.assertEqual(self.output(), (0, 0, 0))

    def test_long_recovery_does_not_extend_sensor_freshness(self):
        n = self.prepare_command_wait()
        self.healthy_tick(30)
        n.clock_time = 31.6
        n.timer_callback()
        self.assertFalse(n.armed)
        self.assertEqual(n.fault, 'localization_guard_stale')
        self.assertEqual(self.output(), (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
