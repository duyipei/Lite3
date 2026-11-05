"""T2.4 offline Foxy execution tests. No rclpy.init(), DDS, or robot access.

The real, frozen Foxy group classes, Task, Executor handler and pool-dispatch
methods are executed against mock Node/RMW/timer readiness. This tests group
admission on actual worker threads, not merely mutual exclusion of a Python
lock. It is not an integration test of a running ROS graph or Jetson timing.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import hashlib
import inspect
import json
import multiprocessing
from pathlib import Path
import sys
import threading
import types
from types import SimpleNamespace as NS
import typing
import unittest
from unittest.mock import patch

import test_voa_costmap_bridge as geometry
from t22_fixtures import recorder

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = json.loads((Path(__file__).parent/'fixtures/t24_foxy_reference.json').read_text())


def source_module(name, source):
    module = types.ModuleType(name)
    exec(compile(source, '<offline '+name+'>', 'exec'), module.__dict__)
    return module


GROUPS = source_module('t24_foxy_groups', REFERENCE['callback_groups']['source'])
TASKS = source_module('t24_foxy_tasks', REFERENCE['task']['source'])


class TrackedPool(ThreadPoolExecutor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.submitted = []

    def submit(self, *args, **kwargs):
        future = super().submit(*args, **kwargs)
        self.submitted.append(future)
        return future


class OfflineProtocol:
    """Only RMW initialization, ready-entity selection and node lifetime are fake."""
    def __init__(self, *, context=None):
        self._tasks, self.ready = [], deque()
        self._tasks_lock = threading.Lock()
        self._is_shutdown = False
        self._guard = NS(trigger=lambda: None)
        self._work_tracker = EXEC['_WorkTracker']()
        self.node = None
        self.hook = None

    def add_node(self, node):
        self.node = node

    def spin(self):
        self.hook(self)

    def shutdown(self):
        self._executor.shutdown(wait=True)

    def wait_for_ready_callbacks(self, *args):
        if not self.ready:
            raise EXEC['TimeoutException']()
        return self.ready.popleft()

    def dispatch(self, entity, message=None, already_selected=False):
        # This slot mocks KEEP_LAST(1). Admission itself is the unmodified
        # Foxy Executor.can_execute / CallbackGroup.beginning_execution path.
        entity.latest = message
        if not already_selected and not self.can_execute(entity):
            return None

        def take(current):
            current.takes += 1
            return current.latest

        invoke = self._execute_timer if entity.kind == 'timer' else self._execute_subscription
        handler = self._make_handler(entity, self.node, take, invoke)
        self.ready.append((handler, entity, self.node))
        self.spin_once(timeout_sec=0)  # Actual Foxy MultiThreadedExecutor method.
        return NS(task=handler, future=self._executor.submitted[-1])

    @staticmethod
    def finish(job):
        if job is None:
            raise AssertionError('entity was not admitted')
        job.future.result(timeout=3)
        return job.task.result()


EXEC = dict(__name__='t24_foxy_executor', OfflineProtocol=OfflineProtocol,
            ThreadPoolExecutor=TrackedPool, Condition=threading.Condition,
            inspect=inspect, multiprocessing=multiprocessing, Task=TASKS.Task,
            Future=TASKS.Future, Context=object, WaitableEntityType=object,
            Callable=typing.Callable, Coroutine=typing.Coroutine, Any=typing.Any,
            Union=typing.Union)
exec(compile(REFERENCE['executors']['source'], '<Foxy executor extract>', 'exec'), EXEC)


class Entity:
    def __init__(self, node, kind, name, callback, group, qos=None, period=None):
        self.kind, self.name, self.callback_group = kind, name, group
        self.qos, self.period = qos, period
        self.original_callback = callback
        self._executor_event = False
        self.latest, self.takes, self.entries = None, 0, 0
        group.add_entity(self)

        def traced(*args):
            self.entries += 1
            node.trace.append(('enter', name, threading.get_ident()))
            try:
                return callback(*args)
            finally:
                node.trace.append(('exit', name, threading.get_ident()))
        self.callback = traced


class Publisher:
    def __init__(self): self.messages, self.error = [], None
    def publish(self, message):
        if self.error: raise self.error
        self.messages.append(copy.deepcopy(message))


class MockNode:
    def __init__(self):
        self.subscriptions, self.timers, self.publishers, self.trace = [], [], {}, []
        self.default_group = GROUPS.MutuallyExclusiveCallbackGroup()
        self.destroyed = False

    def create_publisher(self, kind, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, kind, topic, callback, qos, callback_group=None):
        entity = Entity(self, 'subscription', topic, callback,
                        callback_group or self.default_group, qos=qos)
        self.subscriptions.append(entity)
        return entity

    def create_timer(self, period, callback, callback_group=None):
        entity = Entity(self, 'timer', 'projection_timer', callback,
                        callback_group or self.default_group, period=period)
        self.timers.append(entity)
        return entity

    def destroy_subscription(self, subscription):
        if subscription in self.subscriptions: self.subscriptions.remove(subscription)

    def get_logger(self): return NS(info=lambda *v: None, warning=lambda *v: None)
    def destroy_node(self): self.destroyed = True


class MockBuffer:
    def __init__(self): self.queries, self.transforms, self.lookup_hook = [], [], None
    def lookup_transform(self, target, source, stamp):
        self.queries.append((target, source, stamp))
        if self.lookup_hook: self.lookup_hook()
        return NS(transform=geometry.transform())
    def set_transform(self, transform, authority): self.transforms.append(transform)
    def set_transform_static(self, transform, authority): self.transforms.append(transform)


class SpyRLock:
    def __init__(self):
        self.lock = threading.RLock()
        self.guard = threading.Lock()
        self.attempts = 0
        self.second_attempt = threading.Event()
    def __enter__(self):
        with self.guard:
            self.attempts += 1
            if self.attempts == 2: self.second_attempt.set()
        self.lock.acquire()
        return self
    def __exit__(self, *args): self.lock.release()


def run_main(hook, baseline=False, diagnostics=False):
    """Execute the real production main with ROS imports replaced, not its logic."""
    source = (ROOT/'scripts/voa_costmap_bridge.py').read_text()
    if baseline:
        source = source.replace('from rclpy.callback_groups import MutuallyExclusiveCallbackGroup',
                                'from rclpy.callback_groups import ReentrantCallbackGroup')
        source = source.replace('group = MutuallyExclusiveCallbackGroup()', 'group = ReentrantCallbackGroup()')
    bridge_module = source_module('t24_bridge', source)
    clock = NS(now=100.2)
    bridge_module.time = NS(monotonic=lambda: clock.now,
                            monotonic_ns=lambda: round(clock.now*1e9))
    node = MockNode()
    instances = []

    def module(name, **values):
        value = types.ModuleType(name); value.__dict__.update(values); return value

    class QoS:
        def __init__(self, depth, **values):
            self.depth, self.reliability = depth, 'reliable'
            self.__dict__.update(values)

    class UnexpectedSpinThread:
        def __init__(self, *args, **kwargs): raise AssertionError('no dedicated spin thread')

    qos = module('rclpy.qos', QoSProfile=QoS,
                 ReliabilityPolicy=NS(BEST_EFFORT='best_effort'),
                 DurabilityPolicy=NS(VOLATILE='volatile', TRANSIENT_LOCAL='transient_local'),
                 HistoryPolicy=NS(KEEP_LAST='keep_last'))
    modules = {'rclpy':module('rclpy', init=lambda: None, create_node=lambda *a: node, shutdown=lambda: None),
        'rclpy.callback_groups':GROUPS, 'rclpy.qos':qos,
        'rclpy.time':module('rclpy.time', Time=NS(from_msg=bridge_module.stamp_seconds)),
        'sensor_msgs':module('sensor_msgs'), 'sensor_msgs.msg':module('sensor_msgs.msg', PointCloud2=geometry.Cloud, PointField=geometry.Field),
        'nav_msgs':module('nav_msgs'), 'nav_msgs.msg':module('nav_msgs.msg', Odometry=NS),
        'std_msgs':module('std_msgs'), 'std_msgs.msg':module('std_msgs.msg', Header=NS),
        'grid_map_msgs':module('grid_map_msgs'), 'grid_map_msgs.msg':module('grid_map_msgs.msg', GridMap=NS),
        'tf2_msgs':module('tf2_msgs'), 'tf2_msgs.msg':module('tf2_msgs.msg', TFMessage=NS),
        'tf2_ros':module('tf2_ros', Buffer=MockBuffer, TransformException=LookupError)}

    def make_executor(*args, **kwargs):
        executor = EXEC['MultiThreadedExecutor'](*args, **kwargs)
        instances.append(executor)
        def configured(current):
            current.bridge = node.timers[0].original_callback.__self__
            current.clock = clock
            current.entities = {e.name:e for e in node.subscriptions}
            current.timer = node.timers[0]
            hook(current)
        executor.hook = configured
        return executor

    modules['rclpy.executors'] = module('rclpy.executors', MultiThreadedExecutor=make_executor,
                                       SingleThreadedExecutor=UnexpectedSpinThread)
    with patch.dict(sys.modules, modules):
        listener = source_module('t24_foxy_listener', REFERENCE['listener']['source']).TransformListener
        modules['tf2_ros'].TransformListener = listener
        probe = NS(active=lambda: False) if diagnostics else None
        with patch.object(recorder, 'create_probe', return_value=probe):
            with patch.object(sys, 'argv', ['voa_costmap_bridge.py','--config',str(ROOT/'navigation/vendor_voa.yaml')]):
                bridge_module.main()
    return instances[0]


def registration(stamp=100.1):
    return NS(header=NS(frame_id='map', stamp=geometry.grid(stamp=stamp).header.stamp),
              child_frame_id='ndt_registration',
              pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))


def block_grid(message, entered, release):
    original = message.header
    class Header:
        frame_id = original.frame_id
        @property
        def stamp(self):
            entered.set()
            if not release.wait(3): raise AssertionError('test release deadline')
            return original.stamp
    message.header = Header()
    return message


class ConfigurationTests(unittest.TestCase):
    def test_frozen_foxy_executor_and_groups_match_jetson_reference(self):
        for key in ('executors','callback_groups'):
            item=REFERENCE[key]
            recorded=item['jetson_t21_recorded_sha256']
            actual=item.get('full_source_sha256',hashlib.sha256(item['source'].encode()).hexdigest())
            self.assertEqual(actual, recorded)
        for key in ('executors','callback_groups','task','listener'):
            self.assertEqual(hashlib.sha256(REFERENCE[key]['source'].encode()).hexdigest(),REFERENCE[key]['source_sha256'])

    def check_configuration(self, executor):
        grid=executor.entities['/patrol/voa/grid_map']
        reg=executor.entities['/patrol/registration_odom']
        tf=executor.entities['tf'];static=executor.entities['tf_static']
        self.assertIs(grid.callback_group,reg.callback_group)
        self.assertIs(grid.callback_group,executor.timer.callback_group)
        self.assertIsInstance(grid.callback_group,GROUPS.MutuallyExclusiveCallbackGroup)
        self.assertIsInstance(tf.callback_group,GROUPS.ReentrantCallbackGroup)
        self.assertIs(tf.callback_group,static.callback_group)
        self.assertIsNot(tf.callback_group,grid.callback_group)
        self.assertEqual(executor._executor._max_workers,2)
        self.assertEqual(len(executor.node.timers),1)
        self.assertEqual(executor.timer.period,.025)
        self.assertEqual(grid.qos.depth,1)
        self.assertEqual(grid.qos.reliability,'best_effort')
        self.assertEqual(reg.qos,1)
        self.assertFalse(hasattr(tf.original_callback.__self__,'dedicated_listener_thread'))

    def test_main_registers_three_business_callbacks_and_independent_tf_group(self):
        run_main(self.check_configuration)

    def test_diagnostic_wrapper_preserves_listener_group_and_thread_configuration(self):
        run_main(self.check_configuration,diagnostics=True)


class DispatchTests(unittest.TestCase):
    def test_business_admission_does_not_occupy_second_worker_waiting_for_lock(self):
        def scenario(e):
            lock=SpyRLock();e.bridge.lock=lock
            entered,release=threading.Event(),threading.Event()
            try:
                first=e.dispatch(e.entities['/patrol/voa/grid_map'],block_grid(geometry.grid(stamp=100.1),entered,release))
                self.assertTrue(entered.wait(3))
                self.assertIsNone(e.dispatch(e.entities['/patrol/registration_odom'],registration()))
                self.assertIsNone(e.dispatch(e.timer))
                self.assertEqual(lock.attempts,1)
                self.assertEqual(e.entities['/patrol/registration_odom'].entries,0)
                self.assertEqual(e.timer.entries,0)
                tf=e.dispatch(e.entities['tf'],NS(transforms=[NS(child_frame_id='leg_base_link')]))
                e.finish(tf)
                self.assertEqual(len(e.bridge.buffer.transforms),1)
                grid_tid=next(t for event,name,t in e.node.trace if event=='enter' and name=='/patrol/voa/grid_map')
                tf_tid=next(t for event,name,t in e.node.trace if event=='enter' and name=='tf')
                self.assertNotEqual(grid_tid,tf_tid)
            finally: release.set()
            e.finish(first)
            e.finish(e.dispatch(e.entities['/patrol/registration_odom'],registration()))
            e.finish(e.dispatch(e.timer))
            self.assertEqual(len(e.bridge.anchors),1)
            self.assertEqual(e.timer.entries,1)
        run_main(scenario)

    def test_foxy_worker_beginning_execution_rejects_preselected_business_handler(self):
        def scenario(e):
            entered,release=threading.Event(),threading.Event()
            lock=SpyRLock();e.bridge.lock=lock
            try:
                first=e.dispatch(e.entities['/patrol/voa/grid_map'],block_grid(geometry.grid(stamp=100.1),entered,release))
                self.assertTrue(entered.wait(3))
                reg=e.entities['/patrol/registration_odom']
                rejected=e.dispatch(reg,registration(),already_selected=True)
                e.finish(rejected)
                self.assertEqual(reg.takes,0)
                self.assertEqual(reg.entries,0)
                self.assertFalse(reg._executor_event)
                self.assertEqual(lock.attempts,1)
                e.finish(e.dispatch(e.entities['tf'],NS(transforms=[])))
            finally: release.set()
            e.finish(first)
        run_main(scenario)

    def test_reentrant_baseline_can_block_both_workers_and_delay_tf(self):
        def scenario(e):
            lock=SpyRLock();e.bridge.lock=lock
            entered,release=threading.Event(),threading.Event()
            try:
                first=e.dispatch(e.entities['/patrol/voa/grid_map'],block_grid(geometry.grid(stamp=100.1),entered,release))
                self.assertTrue(entered.wait(3))
                second=e.dispatch(e.entities['/patrol/registration_odom'],registration())
                self.assertTrue(lock.second_attempt.wait(3))
                tf=e.dispatch(e.entities['tf'],NS(transforms=[NS(child_frame_id='leg_base_link')]))
                self.assertFalse(tf.task.executing())
                self.assertFalse(tf.task.done())
                self.assertEqual(e.entities['tf'].entries,0)
            finally: release.set()
            for job in (first,second,tf):e.finish(job)
            self.assertEqual(e.entities['tf'].entries,1)
        run_main(scenario,baseline=True)

    def test_projection_timer_cannot_reenter_while_tf_is_still_admissible(self):
        def scenario(e):
            e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'],geometry.grid(stamp=100.1)))
            entered,release=threading.Event(),threading.Event()
            def blocked_lookup():
                entered.set()
                if not release.wait(3):raise AssertionError('lookup test deadline')
            e.bridge.buffer.lookup_hook=blocked_lookup
            try:
                first=e.dispatch(e.timer)
                self.assertTrue(entered.wait(3))
                self.assertIsNone(e.dispatch(e.timer))
                self.assertEqual(e.timer.entries,1)
                e.finish(e.dispatch(e.entities['tf'],NS(transforms=[])))
            finally: release.set()
            e.finish(first)
            e.bridge.buffer.lookup_hook=None
            e.finish(e.dispatch(e.timer))
            self.assertEqual(e.timer.entries,2)
            self.assertEqual(len(e.node.timers),1)
            self.assertEqual(len(e.node.publishers[geometry.M.TOPIC].messages),1)
        run_main(scenario)

    def test_exception_releases_business_group_and_preserves_original_error(self):
        def scenario(e):
            bad=geometry.grid();bad.header.stamp.nanosec=1000000000
            with self.assertRaisesRegex(ValueError,'invalid sensor stamp'):
                e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'],bad))
            self.assertTrue(e.timer.callback_group.can_execute(e.timer))
            self.assertTrue(e.bridge.lock.acquire(blocking=False))
            e.bridge.lock.release()
        run_main(scenario)


class BusinessRegressionTests(unittest.TestCase):
    def test_serial_dispatch_updates_registration_and_publishes_only_newer_stamps(self):
        def scenario(e):
            e.finish(e.dispatch(e.entities['/patrol/registration_odom'],registration()))
            for stamp in (100.,100.1,100.05,100.15):
                e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'],geometry.grid(stamp=stamp)))
                e.finish(e.dispatch(e.timer))
            cloud=e.node.publishers[geometry.M.TOPIC].messages
            stamps=[geometry.M.stamp_seconds(m.header.stamp) for m in cloud]
            self.assertEqual(stamps,[100.,100.1,100.15])
            self.assertEqual(len(e.bridge.anchors),1)
            self.assertEqual(len(e.node.publishers[geometry.M.HEARTBEAT].messages),3)
            self.assertTrue(all(q[:2]==('map','base_link') for q in e.bridge.buffer.queries))
        run_main(scenario)

    def test_mock_depth_one_overwrites_pending_input_without_unbounded_backlog(self):
        def scenario(e):
            grid=e.entities['/patrol/voa/grid_map'];entered,release=threading.Event(),threading.Event()
            try:
                first=e.dispatch(grid,block_grid(geometry.grid(stamp=100.),entered,release))
                self.assertTrue(entered.wait(3))
                for i in range(1,31):self.assertIsNone(e.dispatch(grid,geometry.grid(stamp=100.+i*.005)))
                self.assertEqual(len(e._tasks),1)
            finally:release.set()
            e.finish(first)
            e.finish(e.dispatch(grid,grid.latest))
            self.assertEqual(grid.entries,2)
            self.assertEqual(len(e.bridge.pending),2)
            self.assertAlmostEqual(e.bridge.last_received,100.15)
            e.finish(e.dispatch(e.timer))
            self.assertAlmostEqual(e.bridge.last_published,100.15)
        run_main(scenario)

    def test_grid_cache_remains_sixteen_and_newest_valid_projection_is_used(self):
        def scenario(e):
            for i in range(20):e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'],geometry.grid(stamp=100.+i*.005)))
            self.assertEqual(len(e.bridge.pending),16)
            e.finish(e.dispatch(e.timer))
            self.assertAlmostEqual(e.bridge.last_published,100.095)
            self.assertEqual(e.bridge.buffer.queries,[('map','base_link',100.095)])
        run_main(scenario)

    def test_registration_cache_remains_twelve_under_serial_dispatch(self):
        def scenario(e):
            entity=e.entities['/patrol/registration_odom']
            for i in range(20):e.finish(e.dispatch(entity,registration(100.+i*.005)))
            self.assertEqual(len(e.bridge.anchors),12)
            stamps=[geometry.M.stamp_seconds(m.header.stamp) for m in e.bridge.anchors]
            self.assertEqual(stamps,sorted(stamps))
            self.assertAlmostEqual(stamps[-1],100.095)
        run_main(scenario)

    def test_expired_source_still_cannot_publish_or_refresh_heartbeat(self):
        def scenario(e):
            e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'],geometry.grid(stamp=100.)))
            e.clock.now=100.36
            e.finish(e.dispatch(e.timer))
            self.assertFalse(e.node.publishers[geometry.M.TOPIC].messages)
            self.assertFalse(e.node.publishers[geometry.M.HEARTBEAT].messages)
        run_main(scenario)

    def test_malformed_grid_keeps_original_fail_closed_behavior(self):
        def scenario(e):
            message=geometry.grid();message.info.resolution=0
            e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'],message))
            e.finish(e.dispatch(e.timer))
            self.assertEqual(e.bridge.fail_grid,1)
            self.assertFalse(e.node.publishers[geometry.M.TOPIC].messages)
            self.assertTrue(e.timer.callback_group.can_execute(e.timer))
        run_main(scenario)

    def test_heartbeat_publish_failure_does_not_commit_publish_marker(self):
        def scenario(e):
            e.finish(e.dispatch(e.entities['/patrol/voa/grid_map'],geometry.grid(stamp=100.1)))
            failure=RuntimeError('original heartbeat publish failure')
            e.node.publishers[geometry.M.HEARTBEAT].error=failure
            with self.assertRaises(RuntimeError) as caught:e.finish(e.dispatch(e.timer))
            self.assertIs(caught.exception,failure)
            self.assertEqual(e.bridge.last_published,0.)
            self.assertEqual(e.bridge.diag_counts['published'],0)
            self.assertTrue(e.timer.callback_group.can_execute(e.timer))
        run_main(scenario)


if __name__=='__main__':unittest.main()
