#!/usr/bin/env python3
"""Capture this map passively, then compare the sealed offline CPU/P2D/D2D binary."""
import argparse
from collections import OrderedDict
import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

import numpy as np

if __package__:
    from . import analyze, dataset, native, run
else:
    import analyze
    import dataset
    import native
    import run

HERE = Path(__file__).resolve().parent
MAP_ID = 'patrol_map_20261031_01_area_v2_review'
PROJECT = Path('/home/ysc/lite3_patrol_project')
NATIVE_ROOT = PROJECT/'results/gpu_ndt_20261116_ksq6a947/workspace'
PARAMETERS = ['globalmap_pcd', 'reg_method', 'ndt_neighbor_search_method',
              'ndt_resolution', 'downsample_resolution',
              'globalmap_downsample_resolution', 'fine_registration_enabled',
              'convert_utm_to_local']


def unique_id():
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6]


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def validate_parameters(values, map_file):
    if set(PARAMETERS)-set(values):
        raise ValueError('cannot read all current localization parameters')
    if Path(values['globalmap_pcd']).resolve() != Path(map_file).resolve():
        raise ValueError('running localization is using another map')
    if values['reg_method'] != 'NDT_OMP' or values['ndt_neighbor_search_method'] != 'DIRECT1':
        raise ValueError('capture requires the production CPU NDT_OMP + DIRECT1')
    if values['fine_registration_enabled'] is not False or values['convert_utm_to_local'] is not False:
        raise ValueError('fine ICP / UTM conversion is outside this comparison')
    for key, expected in [('downsample_resolution', .3),
                          ('globalmap_downsample_resolution', .3), ('ndt_resolution', 1.)]:
        if type(values[key]) not in (int, float) or not math.isclose(values[key], expected, abs_tol=1e-9):
            raise ValueError('this map test expects %s=%s; actual=%r' % (key, expected, values[key]))


def cloud_xyz(msg):
    """Honor PointCloud2 endianness, padding and organized row strides."""
    if not (1 <= msg.width*msg.height <= 1000000 and msg.point_step > 0
            and msg.row_step >= msg.width*msg.point_step
            and len(msg.data) >= msg.row_step*msg.height):
        raise ValueError('invalid/beyond-bound PointCloud2 layout')
    fields = {f.name: f for f in msg.fields}
    types = {7: 'f4', 8: 'f8'}
    for key in ('x', 'y', 'z'):
        field = fields.get(key)
        if (field is None or field.datatype not in types or field.count != 1
                or field.offset < 0 or field.offset + (4 if field.datatype == 7 else 8) > msg.point_step):
            raise ValueError('unsupported XYZ field')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [endian+types[fields[k].datatype] for k in ('x', 'y', 'z')],
                      'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([array[k].ravel() for k in ('x', 'y', 'z')])
    if not np.isfinite(xyz).all():
        raise ValueError('nonfinite captured cloud')
    return xyz


def stamp_ns(message):
    s = message.header.stamp
    if s.sec < 0 or not 0 <= s.nanosec < 1000000000:
        raise ValueError('invalid source stamp')
    return int(s.sec)*1000000000 + int(s.nanosec)


class PairCapture:
    """Two bounded exact-stamp caches. Never pair planar /odom with metric points."""
    def __init__(self, output, interval=.5, limit=20, clock=time.monotonic):
        self.output, self.interval, self.limit, self.clock = Path(output), interval, limit, clock
        self.cache = {'aligned': OrderedDict(), 'pose': OrderedDict()}
        self.rows, self.last_stamp, self.last_saved = [], -1, float('-inf')
        self.ready, self.ready_seen = False, float('-inf')
        self.max_cache, self.evictions = 0, 0

    def set_ready(self, ready):
        self.ready, self.ready_seen = bool(ready), self.clock()

    def add(self, kind, message):
        if message.header.frame_id != 'map':
            return
        if kind == 'pose' and message.child_frame_id != 'ndt_registration':
            return
        stamp = stamp_ns(message)
        if stamp <= self.last_stamp or len(self.rows) >= self.limit:
            return
        cache = self.cache[kind]
        cache[stamp] = message
        while len(cache) > 8:
            cache.popitem(last=False)
            self.evictions += 1
        self.max_cache = max(self.max_cache, sum(len(c) for c in self.cache.values()))
        common = set(self.cache['aligned']) & set(self.cache['pose'])
        now = self.clock()
        if not common or not self.ready or now-self.ready_seen > 1.5 or now-self.last_saved < self.interval:
            return
        stamp = max(common)
        p = self.cache['pose'][stamp].pose.pose
        pose = [p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        dataset.pose_matrix(pose)  # Full XYZ and quaternion; do not substitute /odom Z=0.
        xyz = cloud_xyz(self.cache['aligned'][stamp])
        name = 'frame_%02d.npz' % len(self.rows)
        with (self.output/name).open('xb') as stream:
            np.savez_compressed(stream, aligned=xyz, published=pose)
        self.rows.append(dict(stamp=stamp/1e9, stamp_ns=stamp, file=name,
                              sha256=dataset.sha(self.output/name), points=len(xyz),
                              received_monotonic_ns=time.monotonic_ns()))
        self.last_stamp, self.last_saved = stamp, now
        for cache in self.cache.values():
            for old in list(cache):
                if old <= stamp:
                    del cache[old]
        print('[CAPTURE] %d/%d stamp_ns=%d points=%d' %
              (len(self.rows), self.limit, stamp, len(xyz)), flush=True)


def merge_parameter_reply(collected, values):
    """Foxy composition can remap TWO parameter servers to the same node name.

    PARAMETER_NOT_SET is a missing declaration on that component, not false,
    zero or a reason to invent defaults. Conflicting declarations fail closed.
    """
    if len(values) != len(PARAMETERS):
        raise ValueError('parameter response length does not match request')
    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
    expected = {'globalmap_pcd': (4,), 'reg_method': (4,),
                'ndt_neighbor_search_method': (4,), 'ndt_resolution': (2, 3),
                'downsample_resolution': (2, 3), 'globalmap_downsample_resolution': (2, 3),
                'fine_registration_enabled': (1,), 'convert_utm_to_local': (1,)}
    for key, value in zip(PARAMETERS, values):
        if value.type == 0:
            continue
        if value.type not in expected[key]:
            raise ValueError('unexpected parameter type: %s type=%s' % (key, value.type))
        decoded = getattr(value, fields[value.type])
        if key in collected and collected[key] != decoded:
            raise ValueError('conflicting composed-node parameter: '+key)
        collected[key] = decoded
    return collected


def read_parameters(node, client):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    if not client.wait_for_service(timeout_sec=5):
        raise ValueError('/patrol_localization/get_parameters unavailable; start CPU localization first')
    collected = {}
    deadline = time.monotonic()+15
    for _ in range(16):
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            break
        request = GetParameters.Request()
        request.names = PARAMETERS
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=min(3, remaining))
        if not future.done():
            future.cancel()
            continue
        if future.result() is None:
            continue
        merge_parameter_reply(collected, future.result().values)
        if set(collected) == set(PARAMETERS):
            return collected
        rclpy.spin_once(node, timeout_sec=.05)
    missing = sorted(set(PARAMETERS)-set(collected))
    raise ValueError('incomplete composed localization parameters after bounded queries; missing: '+', '.join(missing))


def capture(project, output, seconds=30, frames=20, interval=.5, confirmed=False):
    if not confirmed:
        raise ValueError('confirm stationary / DISARMED; this tool does not change motion authorization')
    if not (5 <= seconds <= 180 and 2 <= frames <= 120 and .2 <= interval <= 5):
        raise ValueError('capture limits: 5..180s, 2..120 frames, interval .2..5s')
    project, output = Path(project).resolve(), Path(output).resolve()
    map_file = project/'navigation'/MAP_ID/'localization_map.pcd'
    if output.exists():
        raise ValueError('capture output already exists; choose a fresh path')
    if not map_file.is_file() or map_file.parent == output or output in map_file.parents:
        raise ValueError('missing map or capture would overlap map inputs')
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    rclpy.init(args=[])
    node = rclpy.create_node('gpu_ndt_passive_indoor_capture')
    state, metadata, error = None, None, None
    try:
        client = node.create_client(GetParameters, '/patrol_localization/get_parameters')
        parameters = read_parameters(node, client)
        validate_parameters(parameters, map_file)
        output.mkdir(parents=True)
        state = PairCapture(output, interval, frames)
        import shutil
        shutil.copyfile(map_file, output/'map.pcd')
        metadata = dict(schema_version=1, scope='paired_metric_indoor_capture', map_id=MAP_ID,
                        parameters=parameters, map_sha256=dataset.sha(map_file),
                        source='inverse full registration pose times unfiltered aligned_points',
                        limitations='accepted CPU poses are seeds, not truth; not a full raw/IMU/UKF replay',
                        source_frame='ndt_registration', target_frame='map',
                        operator_confirmed_disarmed=True, completed=False)
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            node.create_subscription(PointCloud2, '/aligned_points',
                                     lambda m: state.add('aligned', m), qos_profile_sensor_data),
            node.create_subscription(Odometry, '/patrol/registration_odom',
                                     lambda m: state.add('pose', m), qos_profile_sensor_data),
            node.create_subscription(Bool, '/patrol/localization_ready',
                                     lambda m: state.set_ready(m.data), ready_qos)]
        begin = time.monotonic()
        print('[CAPTURE] map verified; wait for fresh READY and exact-stamp points/metric pose', flush=True)
        while rclpy.ok() and time.monotonic()-begin < seconds and len(state.rows) < frames:
            rclpy.spin_once(node, timeout_sec=.05)
        final = read_parameters(node, client)
        if final != parameters or dataset.sha(map_file) != metadata['map_sha256']:
            raise ValueError('localization parameters/map changed during capture')
        if len(state.rows) < 2:
            raise ValueError('fewer than two valid pairs: check READY, aligned_points and registration_odom')
        metadata['completed'] = True
        return output
    except BaseException as exc:
        error = str(exc) or type(exc).__name__
        raise
    finally:
        if metadata is not None:
            metadata.update(frames=len(state.rows), cache_high_water=state.max_cache,
                            unmatched_evictions=state.evictions, error=error)
            dump(output/'report.json', state.rows)
            dump(output/'capture.json', metadata)
        node.destroy_node()
        rclpy.shutdown()


def verify_capture(directory):
    directory = Path(directory).resolve()
    meta = json.loads((directory/'capture.json').read_text())
    if (meta.get('schema_version') != 1 or meta.get('scope') != 'paired_metric_indoor_capture' or meta.get('map_id') != MAP_ID
            or meta.get('completed') is not True):
        raise ValueError('incomplete capture or wrong map')
    # The live parameter path can stay on Jetson when the archive is copied.
    validate_parameters(meta['parameters'], meta['parameters']['globalmap_pcd'])
    if Path(meta['parameters']['globalmap_pcd']).parent.name != MAP_ID:
        raise ValueError('capture parameter map identity mismatch')
    if dataset.sha(directory/'map.pcd') != meta['map_sha256']:
        raise ValueError('captured map hash changed')
    rows = json.loads((directory/'report.json').read_text())
    if not 2 <= len(rows) <= 120 or len(rows) != meta['frames']:
        raise ValueError('invalid capture length')
    previous = -1
    for i, row in enumerate(rows):
        stamp = row['stamp_ns']
        if (type(stamp) is not int or stamp <= previous or not math.isfinite(row['stamp'])
                or not math.isclose(row['stamp'], stamp/1e9, rel_tol=0, abs_tol=1e-9)):
            raise ValueError('source stamps must increase strictly')
        previous = stamp
        if row['file'] != 'frame_%02d.npz' % i or dataset.sha(directory/row['file']) != row['sha256']:
            raise ValueError('capture frame hash/index changed')
    return meta, rows


def prepare_dataset(capture_dir, output, filter_binary):
    meta, rows = verify_capture(capture_dir)
    output = Path(output)
    result = dataset.prepare(capture_dir, Path(capture_dir)/'map.pcd', output,
                             meta['parameters']['globalmap_downsample_resolution'],
                             meta['parameters']['ndt_resolution'])
    # Current aligned_points contains ALL obstacle returns. Apply the same PCL
    # VoxelGrid as NDT after inverse projection; do not benchmark a dense cloud
    # as though it had already undergone the current .3m registration filter.
    for i, frame in enumerate(result['frames']):
        native.idle_required(True)
        source = output/('scan_%04d.pcd' % i)
        filtered = output/('filtered_%04d.pcd' % i)
        done = subprocess.run([str(filter_binary), str(source), str(filtered),
                               str(meta['parameters']['downsample_resolution'])],
                              capture_output=True, text=True, timeout=30, check=True)
        count = int(done.stdout.strip())
        if count <= 0 or count > 1000000:
            raise ValueError('invalid source filter result')
        filtered.replace(source)  # Only this fresh, private dataset is modified.
        frame.update(source_points=count, source_pcd_sha256=dataset.sha(source),
                     source_stamp_ns=rows[i]['stamp_ns'])
    lines = (output/'cases.tsv').read_text().splitlines()
    for i, line in enumerate(lines):
        fields = line.split('\t')
        fields[1] = str(rows[i//2]['stamp_ns'])
        lines[i] = '\t'.join(fields)
    (output/'cases.tsv').write_text('\n'.join(lines)+'\n')
    result.update(cases_sha256=dataset.sha(output/'cases.tsv'), map_id=MAP_ID,
                  source_leaf=meta['parameters']['downsample_resolution'],
                  preprocessing='inverse recorded full pose, then PCL VoxelGrid; float roundtrip differs from raw scan replay',
                  input='same recorded metric geometry and seeds for all backends',
                  capture_metadata=meta, source_filter_sha256=dataset.sha(filter_binary))
    dump(output/'dataset.json', result)
    return result


def summarize(directories):
    reports = [analyze.analyze(p) for p in directories]
    identities = {v['dataset_id'] for v in reports}
    if len(identities) != 1:
        raise ValueError('rounds used different inputs')
    backends = {}
    for backend in native.prepare.BACKENDS:
        samples = []
        for directory in directories:
            samples.extend(analyze.read_result(Path(directory)/(backend+'.jsonl'))[1])
        backends[backend] = dict(samples=len(samples),
            converged=sum(bool(s['converged']) for s in samples),
            spatial_quality_pass=sum(bool(s['spatial_quality_pass']) for s in samples),
            metrics={key: analyze.statistics([s[key] for s in samples if s.get(key) is not None])
                     for key in ('align_sync_ms', 'registration_pipeline_ms', 'process_cpu_ms', 'rss_bytes')},
            cuda_available_free_bytes=analyze.statistics([s['device_free_bytes'] for s in samples
                                                         if s.get('device_free_bytes') is not None]),
            cuda_total_bytes=analyze.statistics([s['device_total_bytes'] for s in samples
                                                if s.get('device_total_bytes') is not None]))
    for values in backends.values():
        align_mean = values['metrics']['align_sync_ms']['mean']
        pipeline_mean = values['metrics']['registration_pipeline_ms']['mean']
        values['offline_align_equivalent_hz'] = 1000/align_mean if align_mean and align_mean > 0 else None
        values['offline_pipeline_equivalent_hz'] = 1000/pipeline_mean if pipeline_mean and pipeline_mean > 0 else None
    return dict(scope='registration_only', map_id=MAP_ID, dataset_id=next(iter(identities)),
                status='complete' if all(v['status'] == 'complete' for v in reports) else 'partial',
                backends=backends, rounds=reports, memory_scope='CUDA free/total describes shared allocation availability, not NDT exclusive VRAM',
                limitations=['offline equivalent Hz = 1000/mean ms; not live ROS localization output frequency',
                             'sampled accepted CPU outputs, no absolute truth',
                             'no UKF, TF/VOA continuity or full localization latency measured'])


def compare(root, capture_dir, output, warmup=2, repeats=3, confirmed=False):
    native.idle_required(confirmed)
    root, output = Path(root).resolve(), Path(output).resolve()
    if root not in output.parents or output.exists():
        raise ValueError('comparison must use a NEW directory inside the private native workspace')
    if not 1 <= warmup <= 10 or not 1 <= repeats <= 20:
        raise ValueError('warmup 1..10 and repeats 1..20 required')
    seal = native.check(root)
    verify_capture(capture_dir)
    output.mkdir(parents=True)
    env = dict(os.environ)
    env.update(seal['environment'])
    try:
        tool_build = output/'source_filter_build'
        native.command(['cmake', '-S', str(HERE/'source_filter'), '-B', str(tool_build),
                        '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'],
                       output/'source_filter_configure.log', env, 180)
        native.command(['cmake', '--build', str(tool_build), '--parallel', '1'],
                       output/'source_filter_build.log', env, 300)
        prepare_dataset(capture_dir, output/'dataset', tool_build/'lite3_ndt_source_filter')
        def guard(owned_pid):
            busy = [v for v in native.navigation_processes() if v['pid'] != owned_pid]
            if busy:
                raise ValueError('navigation/localization started; stopping only this comparison: '+str(busy))
        for name, order in [('forward', native.prepare.BACKENDS),
                             ('reverse', tuple(reversed(native.prepare.BACKENDS)))]:
            exits = run.run(seal['binary'], output/'dataset', output/name, order,
                            warmup, repeats, interference_check=guard, env_override=seal['environment'])
            if any(v != 0 for v in exits.values()):
                raise ValueError('native backend failed; see '+str(output/name))
        result = summarize([output/'forward', output/'reverse'])
        dump(output/'comparison.json', result)
        lines = ['CPU/GPU 同输入对照：'+MAP_ID,
                 '后端             样本  几何通过  align P50/P95/P99/最大(ms)  流程平均(ms)  配准等效Hz']
        for name, data in result['backends'].items():
            m = data['metrics']['align_sync_ms']
            rate = data['offline_align_equivalent_hz']
            lines.append('%-16s %4d %4d/%-4d %.3f / %.3f / %.3f / %.3f   %.3f   %s' %
                         (name, data['samples'], data['spatial_quality_pass'], data['samples'],
                          m['p50'], m['p95'], m['p99'], m['max'],
                          data['metrics']['registration_pipeline_ms']['mean'],
                          '%.2f' % rate if rate is not None else '未测量'))
        lines.append('等效Hz=1000/平均配准ms，不是实时定位输出频率。流程不含UKF/TF/VOA。')
        lines.append('几何失败和位姿差异需同时看 comparison.json；失败样本没有从耗时统计中删除。')
        (output/'comparison.txt').write_text('\n'.join(lines)+'\n')
        print('\n'.join(lines), flush=True)
        return result
    finally:
        native.verify_hashes(seal['official'])


def memory_snapshot():
    info = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        fields = line.split()
        if fields[0] in ('MemTotal:', 'MemAvailable:', 'MemFree:'):
            info[fields[0][:-1]+'_MiB'] = int(fields[1])/1024
    return dict(system_shared_ram=info, fixed_gpu_quota_configured=False,
                ndt_exclusive_gpu_bytes=None,
                explanation='Xavier NX CPU/GPU share RAM. No fixed NDT GPU reservation. CUDA free/total is not per-process usage.',
                active_navigation_or_benchmark_processes=native.navigation_processes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    c = sub.add_parser('capture', help='read-only ROS subscriptions while CPU localization is READY / DISARMED')
    c.add_argument('--project', type=Path, default=PROJECT)
    c.add_argument('--output', type=Path, required=True)
    c.add_argument('--seconds', type=float, default=30)
    c.add_argument('--frames', type=int, default=20)
    c.add_argument('--interval', type=float, default=.5)
    c.add_argument('--operator-confirmed-disarmed', action='store_true')
    b = sub.add_parser('compare', help='AFTER exiting navigation/localization: sequential offline benchmark only')
    b.add_argument('--root', type=Path, default=NATIVE_ROOT)
    b.add_argument('--capture', type=Path, required=True)
    b.add_argument('--output', type=Path)
    b.add_argument('--warmup', type=int, default=2)
    b.add_argument('--repeats', type=int, default=3)
    b.add_argument('--operator-confirmed-disarmed', action='store_true')
    sub.add_parser('memory', help='read-only shared RAM; does not initialize CUDA')
    a = parser.parse_args()
    try:
        if a.action == 'capture':
            result = capture(a.project, a.output, a.seconds, a.frames, a.interval, a.operator_confirmed_disarmed)
            print('采样完成：'+str(result)+'\n请先正常退出原导航/定位进程，再运行 compare。')
        elif a.action == 'compare':
            output = a.output or a.root/'indoor_v2_tests'/unique_id()
            result = compare(a.root, a.capture, output, a.warmup, a.repeats, a.operator_confirmed_disarmed)
            print('结果：'+str(output/'comparison.txt'))
            return 0 if result['status'] == 'complete' else 1
        else:
            print(json.dumps(memory_snapshot(), ensure_ascii=False, indent=2))
        return 0
    except (ValueError, OSError, KeyError, TypeError, subprocess.SubprocessError) as error:
        parser.exit(2, str(error)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
