#!/usr/bin/env python3
"""Offline tests: deterministic ROS/service/action doubles, no DDS or hardware."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
NS = types.SimpleNamespace


class Message:
    def __init__(self):
        self.data = None
        self.header = NS(frame_id='', stamp=None)
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class Future:
    def __init__(self, value): self.value = value
    def done(self): return True
    def result(self): return self.value
    def add_done_callback(self, callback): callback(self)


class PendingFuture(Future):
    def __init__(self): self.callbacks = []; self.complete = False
    def done(self): return self.complete
    def add_done_callback(self, cb): self.callbacks.append(cb)
    def finish(self, value):
        self.value = value; self.complete = True
        for cb in self.callbacks: cb(self)


class Bus:
    def __init__(self):
        self.now = 100.; self.subs = {}; self.services = {}; self.timers = []
        self.queue = []; self.goals = []; self.statuses = []; self.handles = []
        self.armed = False; self.ready = True; self.arm_calls = []
        self.services['/patrol/set_armed'] = self.arm
    def arm(self, req, response):
        self.arm_calls.append(req.data)
        self.armed = req.data
        return NS(success=True, message='ARMED' if req.data else 'DISARMED')
    def emit(self, topic, msg):
        for cb in self.subs.get(topic, []): self.queue.append(lambda cb=cb: cb(msg))
    def spin(self):
        self.now += .05
        self.emit('/patrol/localization_ready', NS(data=self.ready))
        self.emit('/patrol/armed', NS(data=self.armed))
        for timer in self.timers:
            if self.now >= timer[0]:
                timer[0] = self.now+timer[1]; self.queue.append(timer[2])
        for callback in list(self.queue):
            self.queue.remove(callback); callback()


BUS = None


class Node:
    def __init__(self, name): self.params = {}; self.name = name
    def declare_parameter(self, key, value): self.params[key] = 'outdoor_20260921_003_area_review' if key == 'active_map_id' else value
    def get_parameter(self, key): return NS(value=self.params[key])
    def create_publisher(self, _kind, topic, _qos):
        return NS(publish=lambda msg: BUS.emit(topic, msg), get_subscription_count=lambda: len(BUS.subs.get(topic, [])))
    def create_subscription(self, _kind, topic, callback, _qos): BUS.subs.setdefault(topic, []).append(callback)
    def create_service(self, _kind, name, callback): BUS.services[name] = callback
    def create_client(self, _kind, name):
        return NS(srv_name=name, service_is_ready=lambda: name in BUS.services,
                  wait_for_service=lambda timeout_sec: name in BUS.services,
                  call_async=lambda req: Future(BUS.services[name](req, NS())))
    def create_timer(self, seconds, callback): BUS.timers.append([BUS.now+seconds, seconds, callback])
    def get_logger(self): return NS(info=lambda s: None, warn=lambda s: None, error=lambda s: None)
    def get_clock(self): return NS(now=lambda: NS(to_msg=lambda: NS(sec=int(BUS.now), nanosec=0)))
    def destroy_node(self): pass


class ActionClient:
    def __init__(self, node, action, name): self.name = name
    def server_is_ready(self): return True
    def send_goal_async(self, goal):
        if self.name == '/compute_path_to_pose':
            return Future(NS(accepted=True, get_result_async=lambda: Future(NS(result=NS(path=NS(poses=[1, 2]))))))
        BUS.goals.append(goal)
        result = PendingFuture()
        status = BUS.statuses.pop(0) if BUS.statuses else 4
        handle = NS(accepted=True, canceled=False, get_result_async=lambda: result)
        def cancel(): handle.canceled = True; return Future(NS())
        handle.cancel_goal_async = cancel
        BUS.handles.append(handle)
        def finish():
            if not result.done(): result.finish(NS(status=status))
        BUS.timers.append([BUS.now+.4, 10000, finish])
        return Future(handle)


def module(name, **attrs):
    obj = types.ModuleType(name); obj.__dict__.update(attrs); sys.modules[name] = obj


module('rclpy', init=lambda **kw: None, shutdown=lambda: None, spin_once=lambda *a, **kw: BUS.spin())
module('rclpy.node', Node=Node)
module('rclpy.action', ActionClient=ActionClient)
module('rclpy.qos', QoSProfile=lambda **kw: NS(), DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1))
for pkg, attrs in [('geometry_msgs', dict(PoseStamped=Message, Twist=Message)), ('nav_msgs', dict(Path=Message, Odometry=Message)),
                   ('std_msgs', dict(Bool=Message, String=Message))]:
    module(pkg); module(pkg+'.msg', **attrs)
module('std_srvs'); module('std_srvs.srv', Trigger=NS(Request=Message), SetBool=NS(Request=Message))
module('nav2_msgs'); module('nav2_msgs.action', ComputePathToPose=NS(Goal=Message), NavigateToPose=NS(Goal=Message))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    obj = importlib.util.module_from_spec(spec); spec.loader.exec_module(obj); return obj


bridge = load('bridge', 'scripts/goal_pose_bridge.py')
runner = load('runner', 'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/patrol_runner.py')
clock = NS(monotonic=lambda: BUS.now)
bridge.time = runner.time = clock


class Tests(unittest.TestCase):
    def setUp(self):
        global BUS
        BUS = Bus(); self.bridge = bridge.GoalPoseBridge()
        for _ in range(4): BUS.spin()
        self.mission = dict(map_id='outdoor_20260921_003_area_review', frame_id='map',
                            waypoints=[dict(id='a', x=1., y=0., yaw=0.), dict(id='b', x=2., y=0., yaw=0.)])
    def acquire(self): return self.bridge.acquire_mission(None, NS())
    def test_two_goals_success_disarms(self):
        runner.execute(self.mission, 10)
        self.assertEqual(len(BUS.goals), 2)
        self.assertEqual(BUS.arm_calls.count(True), 1)
        self.assertFalse(BUS.armed); self.assertIsNone(self.bridge.mission_token)
    def test_failure_stops_before_second(self):
        BUS.statuses = [6]
        with self.assertRaises(RuntimeError): runner.execute(self.mission, 10)
        self.assertEqual(len(BUS.goals), 1); self.assertFalse(BUS.armed)
    def test_timeout_stops(self):
        with self.assertRaises(RuntimeError): runner.execute(self.mission, .01)
        self.assertFalse(BUS.armed); self.assertTrue(BUS.handles[0].canceled)
    def test_heartbeat_expiry(self):
        self.assertTrue(self.acquire().success)
        for _ in range(60): BUS.spin()
        self.assertIsNone(self.bridge.mission_token); self.assertIn(False, BUS.arm_calls)
    def test_manual_goal_ignored_during_mission(self):
        self.acquire(); p=Message(); p.header.frame_id='map'
        self.bridge.goal_callback(p)
        self.assertEqual(self.bridge.request_serial, 0)
    def test_cannot_acquire_armed_or_busy(self):
        self.bridge.armed=True; self.assertFalse(self.acquire().success)
        self.bridge.armed=False; self.bridge.goal_active=True; self.assertFalse(self.acquire().success)
    def test_wrong_token_and_old_result(self):
        self.acquire()
        self.bridge.mission_goal(NS(data=json.dumps(dict(token='wrong', index=1))))
        self.assertEqual(self.bridge.mission_index, 0)
        self.assertEqual(runner.classify_event(dict(token='old',index=1,state='RESULT:GOAL_1:SUCCEEDED'), 'new',1),'ignore')
        self.assertEqual(runner.classify_event(dict(token='new',index=1,state='RESULT:GOAL_1:SUCCEEDED'), 'new',2),'ignore')
    def test_cancel_late_acceptance(self):
        self.acquire(); serial=self.bridge.request_serial
        self.bridge.goal_active=True
        self.bridge.cancel_callback(None,NS())
        called=[]; handle=NS(accepted=True,cancel_goal_async=lambda:called.append(True))
        self.bridge.goal_response_callback(serial,Future(handle))
        self.assertEqual(called,[True]); self.assertFalse(self.bridge.goal_active)
    def test_ready_loss_cancels(self):
        self.acquire(); self.bridge.ready_callback(NS(data=False))
        self.assertIsNone(self.bridge.mission_token); self.assertIn(False,BUS.arm_calls)
    def test_out_of_order_fails_closed(self):
        self.acquire(); token=self.bridge.mission_token
        self.bridge.mission_goal(NS(data=json.dumps(dict(token=token,index=2,x=0,y=0,yaw=0))))
        self.assertIsNone(self.bridge.mission_token)
    def test_local_mission_and_fingerprints(self):
        p=ROOT/'navigation/outdoor_20260921_003_area_review/mission_recovered_20261008.yaml'
        m=runner.load_mission(p);runner.verify_map(m,p.parent/'patrol_map.yaml')
        self.assertEqual(len(m['waypoints']),11);self.assertFalse(m['enabled'])
        m['map_image_sha256']='bad'
        with self.assertRaises(ValueError): runner.verify_map(m,p.parent/'patrol_map.yaml')
    def test_bad_coordinates(self):
        self.mission['waypoints'][0]['x']=float('nan')
        with patch.object(runner.yaml,'safe_load',return_value=self.mission), patch.object(Path,'read_text',return_value=''):
            with self.assertRaises(ValueError): runner.load_mission(Path('dummy'))
    def test_manual_disarm_never_rearms(self):
        def disarm():
            if BUS.armed: BUS.armed=False
        BUS.timers.append([BUS.now+1.3,10000,disarm])
        with self.assertRaises(RuntimeError): runner.execute(self.mission,10)
        self.assertEqual(BUS.arm_calls.count(True),1); self.assertFalse(BUS.armed)
    def test_duplicate_ids_rejected(self):
        self.mission['waypoints'][1]['id']='a'
        with patch.object(runner.yaml,'safe_load',return_value=self.mission), patch.object(Path,'read_text',return_value=''):
            with self.assertRaises(ValueError): runner.load_mission(Path('dummy'))
    def test_active_map_mismatch_stops_without_arm(self):
        self.mission['map_id']='other_map'
        with self.assertRaises(RuntimeError): runner.execute(self.mission,10)
        self.assertNotIn(True,BUS.arm_calls); self.assertIsNone(self.bridge.mission_token)
    def test_unavailable_disarm_service_retried(self):
        self.acquire(); service=BUS.services.pop('/patrol/set_armed')
        self.bridge.stop_mission('test')
        self.assertTrue(self.bridge.disarm_required)
        BUS.services['/patrol/set_armed']=service
        for _ in range(30): BUS.spin()
        self.assertFalse(self.bridge.disarm_required); self.assertIn(False,BUS.arm_calls)
    def test_identical_consecutive_waypoints_not_dropped(self):
        self.mission['waypoints'][1].update(x=1.,y=0.,yaw=0.)
        runner.execute(self.mission,10)
        self.assertEqual(len(BUS.goals),2)
    def test_turn_watchdog_cancels_and_disarms_no_rearm(self):
        import math
        original = ActionClient.send_goal_async
        def pending_goal(client, goal):
            if client.name == '/compute_path_to_pose':
                return original(client, goal)
            BUS.goals.append(goal)
            result = PendingFuture()
            handle = NS(accepted=True, canceled=False, get_result_async=lambda: result)
            def cancel():
                handle.canceled = True
                return Future(NS())
            handle.cancel_goal_async = cancel
            BUS.handles.append(handle)
            return Future(handle)
        def motion():
            BUS.emit('/odom', NS(pose=NS(pose=NS(
                position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=math.sin(-math.pi/4), w=math.cos(-math.pi/4)))),
                twist=NS(twist=NS(angular=NS(z=0.)))))
            BUS.emit('/cmd_vel_corrected', NS(linear=NS(x=0.), angular=NS(z=-.0266667)))
        BUS.timers.append([BUS.now, .1, motion])
        with patch.object(ActionClient, 'send_goal_async', pending_goal), patch('builtins.print'):
            with self.assertRaisesRegex(RuntimeError, 'turn_no_progress'):
                runner.execute(self.mission, 40)
        self.assertEqual(len(BUS.goals), 1)
        self.assertTrue(BUS.handles[0].canceled)
        self.assertEqual(BUS.arm_calls.count(True), 1)
        self.assertFalse(BUS.armed)
    def test_turn_config_and_launcher_preflight(self):
        import yaml
        path=ROOT/'navigation/outdoor_20260921_003_area_review/nav2_safe.yaml'
        config=yaml.safe_load(path.read_text())
        controller=config['controller_server']['ros__parameters'];dwb=controller['FollowPath']
        self.assertEqual(controller['goal_checker']['yaw_goal_tolerance'],.20)
        self.assertEqual(dwb['min_vel_x'],0.)
        self.assertIn('RotateToGoal',dwb['critics'])
        self.assertIn('ObstacleFootprint',dwb['critics'])
        self.assertLessEqual(dwb['max_vel_theta'],.4)
        self.assertLessEqual(dwb['max_vel_x'],.15)
        source=(ROOT/'scripts/indoor_nav_start.sh').read_text()
        checks=source.split('config = yaml.safe_load(open(sys.argv[1], "r"))',1)[1].split('\nPY',1)[0]
        exec(checks,dict(config=config))


if __name__ == '__main__': unittest.main()
