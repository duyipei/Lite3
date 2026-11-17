"""Dedicated listener lifecycle and Foxy dispatch, without ROS/RMW or robot IO.

Actual saved Foxy Listener/Task/group/single/pool dispatch code is used.
Readiness, nodes, BufferCore and publishers are doubles. Timings describe this
host's synthetic workload, not DDS transport or Jetson performance.
"""
import ast
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
import types
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest

import test_voa_callback_groups as F
from test_t22_instrumentation import MemoryProbe
from t22_fixtures import recorder

M = F.geometry.M


class QoS:
    def __init__(self, depth, **kwargs):
        self.depth, self.reliability = depth, 'reliable'
        self.__dict__.update(kwargs)


LISTENER = dict(__name__='offline_foxy_listener', QoSProfile=QoS, TFMessage=NS,
    DurabilityPolicy=NS(VOLATILE='volatile', TRANSIENT_LOCAL='transient_local'),
    HistoryPolicy=NS(KEEP_LAST='keep_last'), ReentrantCallbackGroup=F.GROUPS.ReentrantCallbackGroup,
    SingleThreadedExecutor=F.OfflineSingleExecutor, Thread=threading.Thread)
exec('class TransformListener:' + F.REFERENCE['listener']['source'].split('class TransformListener:', 1)[1], LISTENER)


def tf(stamp=100., parent='robot_odom', child='leg_base_link'):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=int(stamp), nanosec=round(stamp % 1*1e9))),
              child_frame_id=child, transform=F.geometry.transform())


class Buffer:
    """Model BufferCore's locking; it is not an implementation of TF2 geometry."""
    def __init__(self):
        self.lock = threading.RLock()
        self.data, self.queries, self.static = {}, [], []
        self.error, self.hook = None, None

    def set_transform(self, value, authority):
        if self.hook: self.hook()
        if self.error: raise self.error
        with self.lock:
            self.data[(value.header.frame_id, value.child_frame_id,
                       value.header.stamp.sec, value.header.stamp.nanosec)] = value

    def set_transform_static(self, value, authority):
        with self.lock: self.static.append(value)

    def lookup_transform(self, target, source, stamp):
        with self.lock:
            self.queries.append((target, source, stamp))
            key = target, source, stamp.sec, stamp.nanosec
            if key not in self.data: raise LookupError('exact-time TF not inserted')
            return self.data[key]


class Node(F.MockNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.unregistered, self.destroy_count = [], 0
        self.fail_static = False

    def create_subscription(self, kind, topic, *args, **kwargs):
        if self.fail_static and topic == 'tf_static':
            raise RuntimeError('static subscription setup failed')
        return super().create_subscription(kind, topic, *args, **kwargs)

    def destroy_subscription(self, value):
        assert not self.destroyed, 'subscription touched after node destruction'
        assert value in self.subscriptions, 'subscription destroyed twice'
        self.unregistered.append(value)
        super().destroy_subscription(value)

    def destroy_node(self):
        assert self.executor is None or self.executor._is_shutdown
        self.destroy_count += 1
        super().destroy_node()


class Environment:
    def __init__(self):
        self.context = NS(running=True)
        self.context.ok = lambda: self.context.running
        self.nodes, self.singles = [], []
        self.node_options = {}
        self.fail_static = self.fail_executor = False
        self.fail_spin = False
        self.business = self.create_node('business')
        self.pool = F.EXEC['MultiThreadedExecutor'](num_threads=2, context=self.context)
        self.pool.node = self.business
        self.pool.add_node(self.business)

    def create_node(self, name, **kwargs):
        assert not kwargs or kwargs['context'] is self.context
        self.node_options[name] = kwargs
        node = Node(name, self.context)
        node.fail_static = self.fail_static if name != 'business' else False
        self.nodes.append(node)
        return node

    def single(self, **kwargs):
        if self.fail_executor: raise RuntimeError('executor setup failed')
        current = F.OfflineSingleExecutor(**kwargs)
        if self.fail_spin:
            current.spin_once = lambda **kwargs: (_ for _ in ()).throw(RuntimeError('spin failed'))
        self.singles.append(current)
        return current

    @contextmanager
    def imports(self):
        core = types.ModuleType('rclpy')
        core.create_node, core.init = self.create_node, lambda: None
        core.ok = self.context.ok
        core.shutdown = lambda: setattr(self.context, 'running', False)
        executors = types.ModuleType('rclpy.executors')
        executors.SingleThreadedExecutor = self.single
        executors.MultiThreadedExecutor = F.EXEC['MultiThreadedExecutor']
        with patch.dict(sys.modules, {'rclpy': core, 'rclpy.executors': executors}):
            yield

    def close(self):
        M.stop_business_executor(self.pool)
        self.context.running = False


@pytest.fixture
def env():
    current = Environment()
    yield current
    current.close()


@contextmanager
def receiver(env, *, diagnostic=False):
    probe = MemoryProbe()
    buffer_type, listener_type = Buffer, LISTENER['TransformListener']
    if diagnostic:
        buffer_type, listener_type = recorder.receiver_types(buffer_type, listener_type, probe)
    buffer = buffer_type()
    with env.imports():
        owner = M.DedicatedTFListener(buffer, env.business, listener_type)
        try:
            yield owner, buffer, probe
        finally:
            owner.close()


def dispatch(owner, value, static=False):
    topic = 'tf_static' if static else 'tf'
    entity = next(s for s in owner.node.subscriptions if s.name == topic)
    job = owner.executor.dispatch(entity, NS(transforms=[value]))
    F.OfflineProtocol.finish(job)


def test_frozen_foxy_single_and_lifecycle_reference():
    ref = F.TF_REFERENCE
    assert ref['full_source_sha256'] == F.REFERENCE['executors']['full_source_sha256']
    assert hashlib.sha256(ref['single_threaded_source'].encode()).hexdigest() == ref['single_threaded_sha256']
    assert 'self._executor.shutdown' not in ref['lifecycle_source']['shutdown']


def test_distinct_node_executor_shared_context_buffer_and_qos(env):
    with receiver(env) as (owner, buffer, _):
        assert owner.node is not env.business
        assert owner.executor is not env.pool
        assert owner.node.context is env.business.context is env.context
        assert owner.node.executor is owner.executor
        assert env.business.executor is env.pool
        assert owner.listener.buffer is buffer
        assert owner.executor.nodes_seen == [owner.node]
        assert env.node_options['patrol_voa_tf_listener']['start_parameter_services'] is False
        assert env.node_options['patrol_voa_tf_listener']['enable_rosout'] is False
        assert not env.business.subscriptions
        assert len(owner.node.subscriptions) == 2
        dynamic, static = owner.node.subscriptions
        assert dynamic.callback_group is static.callback_group
        assert isinstance(dynamic.callback_group, F.GROUPS.ReentrantCallbackGroup)
        assert dynamic.qos.depth == static.qos.depth == 100
        assert dynamic.qos.reliability == static.qos.reliability == 'reliable'
        assert dynamic.qos.durability == 'volatile'
        assert static.qos.durability == 'transient_local'
        assert owner.thread.is_alive()
        assert not hasattr(owner.listener, 'dedicated_listener_thread')


def test_dynamic_static_and_original_grid_query_stamp(env):
    with receiver(env) as (owner, buffer, _):
        dynamic = tf()
        dispatch(owner, dynamic)
        static = tf(0, 'base_link', 'camera_link')
        dispatch(owner, static, static=True)
        assert buffer.lookup_transform('robot_odom', 'leg_base_link', dynamic.header.stamp) is dynamic
        assert buffer.static == [static]
        grid = F.geometry.grid(stamp=100.125)
        dispatch(owner, tf(100.125, 'map', 'base_link'))
        bridge = M.Bridge(env.business, buffer, F.geometry.Cloud, F.geometry.Field, NS,
                         NS(from_msg=lambda value: value), .1, tf_exceptions=(LookupError,))
        bridge.projection_transform(grid)
        assert buffer.queries[-1] == ('map', 'base_link', grid.header.stamp)
        assert buffer.queries[-1][2] is grid.header.stamp


def test_diagnostics_keep_listener_and_insertion_boundaries(env):
    with receiver(env, diagnostic=True) as (owner, buffer, probe):
        value = tf()
        dispatch(owner, value)
        row = next(r for r in probe.rows if r['kind'] == 'leg_rx')
        assert row['t4a_ns'] <= row['t4b_ns'] <= row['t4c_ns']
        assert row['source_stamp_ns'] == 100000000000
        assert row['batch_tf_count'] == 1 and row['outcome'] == 'returned_no_exception'
        assert any(r['kind'] == 'listener_callback' for r in probe.rows)
        assert owner.listener.buffer is buffer


def test_close_waits_for_insertion_then_releases_resources_once(env):
    with receiver(env) as (owner, buffer, _):
        entered, release, closed = threading.Event(), threading.Event(), threading.Event()
        def blocked():
            entered.set()
            assert release.wait(3)
        buffer.hook = blocked
        job = owner.executor.dispatch(owner.node.subscriptions[0], NS(transforms=[tf()]))
        assert entered.wait(3)
        closer = threading.Thread(target=lambda: (owner.close(), closed.set()))
        closer.start()
        try:
            assert not closed.wait(.02)
            assert not owner.node.destroyed
        finally:
            release.set()
        closer.join(3)
        assert closed.is_set() and not owner.thread.is_alive()
        F.OfflineProtocol.finish(job)
        owner.close()
        owner.listener.__del__()  # Foxy's destructor must not re-touch the dead node.
        assert owner.node.destroy_count == 1
        assert len(owner.node.unregistered) == 2


def test_listener_exception_is_reported_instead_of_silent_background_exit(env):
    with receiver(env, diagnostic=True) as (owner, buffer, probe):
        failure = ValueError('insertion failure')
        buffer.error = failure
        with pytest.raises(ValueError): dispatch(owner, tf())
        owner.thread.join(3)
        with pytest.raises(RuntimeError, match='TF Listener stopped') as caught:
            owner.check()
        assert caught.value.__cause__ is failure
        assert any(r.get('outcome') == 'ValueError' for r in probe.rows)


@pytest.mark.parametrize('mode', ['subscription', 'executor', 'thread_start'])
def test_constructor_failure_releases_partial_resources(env, mode):
    env.fail_static = mode == 'subscription'
    env.fail_executor = mode == 'executor'
    with env.imports():
        if mode == 'thread_start':
            with patch.object(threading.Thread, 'start', side_effect=RuntimeError('start failed')):
                with pytest.raises(RuntimeError):
                    M.DedicatedTFListener(Buffer(), env.business, LISTENER['TransformListener'])
        else:
            with pytest.raises(RuntimeError):
                M.DedicatedTFListener(Buffer(), env.business, LISTENER['TransformListener'])
    assert len(env.nodes) == 2
    assert env.nodes[-1].destroyed and not env.nodes[-1].subscriptions


def test_context_shutdown_and_repeated_close(env):
    with receiver(env) as (owner, _, _):
        env.context.running = False
        owner.thread.join(3)
        assert not owner.thread.is_alive()
        owner.check()
        owner.close()
        owner.close()
        assert owner.node.destroy_count == 1


@pytest.mark.parametrize('failure', ['listener', 'business_setup'])
def test_real_main_exits_and_closes_resources_on_failure(env, failure):
    def module(name, **values):
        result = types.ModuleType(name)
        result.__dict__.update(values)
        return result
    modules = {
        'rclpy.callback_groups': F.GROUPS,
        'rclpy.qos': module('rclpy.qos', QoSProfile=QoS, ReliabilityPolicy=NS(BEST_EFFORT='best_effort')),
        'rclpy.time': module('rclpy.time', Time=NS),
        'sensor_msgs': module('sensor_msgs'),
        'sensor_msgs.msg': module('sensor_msgs.msg', PointCloud2=F.geometry.Cloud, PointField=F.geometry.Field),
        'nav_msgs': module('nav_msgs'), 'nav_msgs.msg': module('nav_msgs.msg', Odometry=NS),
        'std_msgs': module('std_msgs'), 'std_msgs.msg': module('std_msgs.msg', Header=NS),
        'grid_map_msgs': module('grid_map_msgs'), 'grid_map_msgs.msg': module('grid_map_msgs.msg', GridMap=NS),
        'tf2_ros': module('tf2_ros', Buffer=Buffer, TransformListener=LISTENER['TransformListener'],
                         TransformException=LookupError),
    }
    env.fail_spin = failure == 'listener'
    with env.imports(), patch.dict(sys.modules, modules), \
            patch.object(recorder, 'create_probe', return_value=None), \
            patch.object(sys, 'argv', ['voa_costmap_bridge.py', '--config', str(F.ROOT/'navigation/vendor_voa.yaml')]):
        if failure == 'business_setup':
            with patch.object(M, 'Bridge', side_effect=ValueError('business setup failed')):
                with pytest.raises(ValueError, match='business setup failed'): M.main()
        else:
            with pytest.raises(RuntimeError, match='TF Listener stopped'): M.main()
    assert not env.context.running
    assert all(node.destroyed for node in env.nodes[1:])
    assert not env.nodes[-1].subscriptions
    assert len(env.nodes[-1].unregistered) == 2
    assert not any(t.name == 'voa_tf_listener' and t.is_alive() for t in threading.enumerate())


def test_idle_listener_wait_is_bounded_without_busy_spinning(env):
    with receiver(env) as (owner, _, _):
        entries = []
        original = owner.executor.spin_once
        def count(**kwargs):
            entries.append(time.monotonic_ns())
            return original(**kwargs)
        owner.executor.spin_once = count
        time.sleep(.25)
        assert len(entries) <= 4  # 100ms idle wait, not a tight Python loop.


def test_business_pool_drained_before_guard_destruction(env):
    entered, release = threading.Event(), threading.Event()
    events = []
    original_destroy = env.pool._guard.destroy
    env.pool._guard.destroy = lambda: (events.append('guard_destroyed'), original_destroy())
    def callback():
        entered.set()
        assert release.wait(3)
        assert not env.business.destroyed
        assert 'guard_destroyed' not in events
        events.append('callback_finished')
    jobs = [env.pool.dispatch(F.Entity(env.business, 'timer', str(i), callback,
                 F.GROUPS.ReentrantCallbackGroup())) for i in range(3)]
    assert entered.wait(3)
    stopped = threading.Event()
    closer = threading.Thread(target=lambda: (M.stop_business_executor(env.pool), stopped.set()))
    closer.start()
    try:
        assert not stopped.wait(.02)
        assert 'guard_destroyed' not in events
    finally: release.set()
    closer.join(3)
    assert stopped.is_set()
    for job in jobs: env.pool.finish(job)
    assert events == ['callback_finished']*3+['guard_destroyed']


def test_listener_changes_preserve_business_methods_except_explicit_clock_repair():
    root = F.ROOT
    baseline = F.subprocess.check_output(['git', 'show',
        'b9f81142f6dee3a43a9c62582b3890dc2db92ac5:scripts/voa_costmap_bridge.py'],
        cwd=root, text=True)
    before = next(n for n in ast.parse(baseline).body if isinstance(n, ast.ClassDef) and n.name == 'Bridge')
    after = next(n for n in ast.parse((root/'scripts/voa_costmap_bridge.py').read_text()).body
                 if isinstance(n, ast.ClassDef) and n.name == 'Bridge')
    old_methods = {n.name: n for n in before.body if isinstance(n, ast.FunctionDef)}
    new_methods = {n.name: n for n in after.body if isinstance(n, ast.FunctionDef)}
    assert set(old_methods) == set(new_methods)
    # The clock-domain repair deliberately changes source-age reads and the
    # conversion of source budgets into elapsed retry deadlines. Its behavior
    # is covered by test_sensor_time_domains and the full T3 retry suite.
    clock_repair = {'receive', 'registration', '_retry_limit', '_prune_candidates',
                    '_defer_candidate', '_projection_diagnostics', 'publish'}
    for name, method in old_methods.items():
        if name not in clock_repair:
            assert ast.dump(method) == ast.dump(new_methods[name]), name


@pytest.mark.parametrize('load_ms', [20, 100, 250, 500])
@pytest.mark.parametrize('isolated', [False, True])
@pytest.mark.parametrize('occupied_workers', [1, 2])
def test_synthetic_pool_occupancy_timing(tmp_path, env, load_ms, isolated, occupied_workers):
    """Artificial handlers occupy one or both business workers.

    T2.4 admission still permits only one business callback at a time. This is
    a deliberate pool-saturation stress case, not proof of historical occupancy.
    """
    probe = MemoryProbe()
    BufferType, ListenerType = recorder.receiver_types(Buffer, LISTENER['TransformListener'], probe)
    buffer = BufferType()
    owner = None
    if isolated:
        with env.imports(): owner = M.DedicatedTFListener(buffer, env.business, ListenerType)
        listener, executor, node = owner.listener, owner.executor, owner.node
    else:
        class BaselineListener(ListenerType):
            # Cleanup-only adapter: retain the frozen callback/dispatch path,
            # but avoid the baseline destructor re-unregistering test entities.
            def unregister(self):
                if getattr(self, '_test_closed', False): return
                self._test_closed = True
                super().unregister()
        listener, executor, node = BaselineListener(buffer, env.business), env.pool, env.business
    topic = next(s for s in node.subscriptions if s.name == 'tf')
    samples, queries_failed = [], 0
    cpu_begin = time.process_time_ns()
    try:
        for i in range(3):
            entered = [threading.Event() for _ in range(occupied_workers)]
            release = threading.Event()
            def block(index):
                entered[index].set()
                assert release.wait(3)
            jobs = [env.pool.dispatch(F.Entity(env.business, 'timer', str(k),
                         lambda k=k: block(k), F.GROUPS.ReentrantCallbackGroup())) for k in range(occupied_workers)]
            assert all(e.wait(3) for e in entered)
            timer = threading.Timer(load_ms/1000, release.set)
            timer.start()
            value = tf(100.+i*.05)
            job = executor.dispatch(topic, NS(transforms=[value]))
            mock_return = time.monotonic_ns()
            try:
                deadline = time.monotonic()+3
                while not job.task.done() and time.monotonic() < deadline:
                    try: buffer.lookup_transform('robot_odom', 'leg_base_link', value.header.stamp)
                    except LookupError: queries_failed += 1
                    time.sleep(.005)
                F.OfflineProtocol.finish(job)
            finally:
                timer.join()
                release.set()
                for work in jobs: env.pool.finish(work)
            row = [r for r in probe.rows if r['kind'] == 'leg_rx'][-1]
            samples.append(dict(mock_publish_return_to_listener_ns=row['t4a_ns']-mock_return,
                listener_to_insert_return_ns=row['t4c_ns']-row['t4a_ns'],
                buffer_call_ns=row['t4c_ns']-row['t4b_ns'], inserted_ns=row['t4c_ns']))
        result = dict(scope='offline Foxy dispatch, fake readiness and Buffer; no DDS/Jetson timing claim',
            load_ms=load_ms, isolated=isolated, occupied_workers=occupied_workers,
            samples=samples, query_failures=queries_failed,
            update_intervals_ns=[b['inserted_ns']-a['inserted_ns'] for a,b in zip(samples,samples[1:])],
            additional_listener_threads=int(isolated), process_cpu_ms=(time.process_time_ns()-cpu_begin)/1e6)
        (tmp_path/'listener_load.json').write_text(json.dumps(result, indent=2))
        assert len(samples) == 3 and len(buffer.data) == 3
        if isolated:
            assert executor is not env.pool and node is not env.business
    finally:
        if owner is not None: owner.close()
        else: listener.unregister()
