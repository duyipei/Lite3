"""Indoor comparator: offline pairing/provenance, guarded execution and fair inputs."""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.gpu_ndt import indoor_compare as indoor, dataset, run
from test_gpu_ndt_experiment import result_fixture


def parameters(map_file):
    return dict(globalmap_pcd=str(map_file), reg_method='NDT_OMP',
                ndt_neighbor_search_method='DIRECT1', ndt_resolution=1.,
                downsample_resolution=.3, globalmap_downsample_resolution=.3,
                fine_registration_enabled=False, convert_utm_to_local=False)


def parameter_reply(values):
    reply = []
    for key in indoor.PARAMETERS:
        v = NS(type=0, bool_value=False, integer_value=0, double_value=0., string_value='')
        if key in values:
            value = values[key]
            if type(value) is bool:v.type = 1;v.bool_value = value
            elif type(value) is int:v.type = 2;v.integer_value = value
            elif type(value) is float:v.type = 3;v.double_value = value
            else:v.type = 4;v.string_value = value
        reply.append(v)
    return reply


def composed_replies(tmp_path):
    params = parameters(tmp_path/indoor.MAP_ID/'localization_map.pcd')
    map_keys = ['globalmap_pcd', 'globalmap_downsample_resolution',
                'downsample_resolution', 'convert_utm_to_local']
    loc_keys = ['reg_method', 'ndt_neighbor_search_method', 'ndt_resolution',
                'downsample_resolution', 'fine_registration_enabled']
    return params, parameter_reply({k:params[k] for k in map_keys}), parameter_reply({k:params[k] for k in loc_keys})


@pytest.mark.parametrize('reverse', [False, True])
def test_real_foxy_composition_partial_replies_merge_without_default_values(tmp_path, reverse):
    params, map_reply, loc_reply = composed_replies(tmp_path)
    merged = {}
    for values in ([loc_reply, map_reply] if reverse else [map_reply, loc_reply]):
        indoor.merge_parameter_reply(merged, values)
    assert merged == params
    assert merged['fine_registration_enabled'] is False


def test_not_set_is_not_decoded_as_false_or_zero():
    assert indoor.merge_parameter_reply({}, parameter_reply({})) == {}


def test_composed_conflicting_shared_parameter_is_rejected(tmp_path):
    _, m, _ = composed_replies(tmp_path)
    collected = indoor.merge_parameter_reply({}, m)
    with pytest.raises(ValueError, match='conflicting.*downsample_resolution'):
        indoor.merge_parameter_reply(collected, parameter_reply({'downsample_resolution': .5}))


@pytest.mark.parametrize('case', ['length', 'wrong_type'])
def test_malformed_live_parameter_reply_names_error(case):
    values = parameter_reply({})
    if case == 'length':values.pop()
    else:values[indoor.PARAMETERS.index('ndt_resolution')].type = 4
    with pytest.raises(ValueError, match='length|ndt_resolution'):
        indoor.merge_parameter_reply({}, values)


def fake_parameter_ros(monkeypatch):
    ros = NS(spin_until_future_complete=lambda *a, **k: None, spin_once=lambda *a, **k: None)
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setitem(sys.modules, 'rcl_interfaces', NS())
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=NS(Request=NS)))


def test_parameter_reader_retries_actual_split_component_responses(tmp_path, monkeypatch):
    fake_parameter_ros(monkeypatch)
    params, m, l = composed_replies(tmp_path)
    answers = iter([m, m, l])
    requests = []
    def call(request):
        requests.append(request.names)
        response = NS(values=next(answers))
        return NS(done=lambda: True, result=lambda: response)
    client = NS(wait_for_service=lambda **k: True, call_async=call)
    assert indoor.read_parameters(NS(), client) == params
    assert len(requests) == 3


def test_parameter_reader_never_invents_missing_component_after_bounded_retry(tmp_path, monkeypatch):
    fake_parameter_ros(monkeypatch)
    _, m, _ = composed_replies(tmp_path)
    calls = []
    def call(request):
        calls.append(request)
        return NS(done=lambda: True, result=lambda: NS(values=m))
    client = NS(wait_for_service=lambda **k: True, call_async=call)
    with pytest.raises(ValueError, match='missing:.*reg_method'):
        indoor.read_parameters(NS(), client)
    assert len(calls) == 16


def cloud(stamp=12345678901, frame='map', endian='<', organized=False):
    height, width, step = (2, 2, 16) if organized else (1, 4, 16)
    row_step = width*step + 8
    data = bytearray(row_step*height)
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [endian+'f4']*3,
                      'offsets': [0, 4, 8], 'itemsize': step})
    a = np.ndarray((height, width), dtype=dtype, buffer=data, strides=(row_step, step))
    for i, k in enumerate(('x', 'y', 'z')):
        a[k] = np.arange(4).reshape(height, width)+i
    return NS(header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9), frame_id=frame),
              fields=[NS(name=k, datatype=7, count=1, offset=offset)
                      for k, offset in [('x', 0), ('y', 4), ('z', 8)]],
              height=height, width=width, point_step=step, row_step=row_step,
              is_bigendian=endian == '>', data=data)


def pose(stamp=12345678901, z=.63, child='ndt_registration'):
    return NS(header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9), frame_id='map'),
              child_frame_id=child, pose=NS(pose=NS(position=NS(x=1., y=2., z=z),
                                                  orientation=NS(x=0., y=0., z=0., w=1.))))


@pytest.mark.parametrize('key,value', [('reg_method', 'NDT_CUDA_P2D'),
    ('ndt_neighbor_search_method', 'DIRECT7'), ('downsample_resolution', .5),
    ('globalmap_downsample_resolution', .5), ('ndt_resolution', 2.),
    ('fine_registration_enabled', True), ('convert_utm_to_local', True)])
def test_capture_rejects_mismatched_production_settings(tmp_path, key, value):
    p = parameters(tmp_path/'map.pcd');p[key] = value
    with pytest.raises(ValueError): indoor.validate_parameters(p, tmp_path/'map.pcd')


def test_capture_checks_actual_running_map_not_only_requested_label(tmp_path):
    p = parameters(tmp_path/'wrong.pcd')
    with pytest.raises(ValueError, match='another map'):
        indoor.validate_parameters(p, tmp_path/'correct.pcd')


@pytest.mark.parametrize('endian,organized', [('<', False), ('>', False), ('<', True), ('>', True)])
def test_cloud_honors_real_pointcloud_stride_and_endian(endian, organized):
    xyz = indoor.cloud_xyz(cloud(endian=endian, organized=organized))
    assert np.array_equal(xyz, np.array([[0, 1, 2], [1, 2, 3], [2, 3, 4], [3, 4, 5]]))


@pytest.mark.parametrize('case', ['short', 'field', 'nan', 'too_large'])
def test_bad_cloud_rejected(case):
    message = cloud()
    if case == 'short':message.data = b''
    elif case == 'field':message.fields[0].datatype = 2
    elif case == 'too_large':message.width = 1000001
    else:
        message.data[:4] = np.float32(float('nan')).tobytes()
    with pytest.raises(ValueError): indoor.cloud_xyz(message)


@pytest.mark.parametrize('order', [('pose', 'aligned'), ('aligned', 'pose')])
def test_pairs_exact_stamp_and_full_metric_z_once(tmp_path, order):
    recorder = indoor.PairCapture(tmp_path, clock=lambda: 10.)
    recorder.set_ready(True)
    messages = {'pose': pose(), 'aligned': cloud()}
    for kind in order:recorder.add(kind, messages[kind])
    for kind in order:recorder.add(kind, messages[kind])
    assert len(recorder.rows) == 1
    with np.load(tmp_path/'frame_00.npz') as d:
        assert d['published'][2] == .63
    assert recorder.rows[0]['stamp_ns'] == 12345678901


def test_different_stamp_or_planar_odom_cannot_pair(tmp_path):
    recorder = indoor.PairCapture(tmp_path, clock=lambda: 10.);recorder.set_ready(True)
    recorder.add('aligned', cloud(101))
    recorder.add('pose', pose(102))
    recorder.add('pose', pose(101, child='base_link'))
    assert recorder.rows == []
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('ready', [False, True])
def test_no_capture_without_fresh_ready(tmp_path, ready):
    now = [0.]
    recorder = indoor.PairCapture(tmp_path, clock=lambda: now[0]);recorder.set_ready(ready)
    if ready:now[0] = 2.
    recorder.add('aligned', cloud());recorder.add('pose', pose())
    assert recorder.rows == []


def test_caches_are_bounded_while_no_matching_pose_arrives(tmp_path):
    recorder = indoor.PairCapture(tmp_path)
    for i in range(100):recorder.add('aligned', cloud(i+1))
    assert len(recorder.cache['aligned']) == 8
    assert recorder.max_cache == 8 and recorder.evictions == 92


def test_out_of_order_or_duplicate_source_never_creates_new_observation(tmp_path):
    now = [0.]
    recorder = indoor.PairCapture(tmp_path, clock=lambda: now[0]);recorder.set_ready(True)
    for stamp in [100, 100, 90, 200]:
        recorder.add('pose', pose(stamp));recorder.add('aligned', cloud(stamp));now[0] += 1
        recorder.set_ready(True)
    assert [r['stamp_ns'] for r in recorder.rows] == [100, 200]


@pytest.fixture
def capture(tmp_path):
    root = tmp_path/'capture';root.mkdir()
    recorder = indoor.PairCapture(root, interval=.2, clock=lambda: 1.)
    # Separate recorder time values keep both legitimate frames.
    now = [0.];recorder.clock = lambda: now[0]
    for stamp in [12345678901, 12445678901]:
        recorder.set_ready(True)
        recorder.add('pose', pose(stamp));recorder.add('aligned', cloud(stamp));now[0] += 1
    dataset.write_pcd(root/'map.pcd', np.ones((4, 3)))
    indoor.dump(root/'report.json', recorder.rows)
    indoor.dump(root/'capture.json', dict(schema_version=1, scope='paired_metric_indoor_capture',
        map_id=indoor.MAP_ID, completed=True, frames=2,
        parameters=parameters(tmp_path/indoor.MAP_ID/'localization_map.pcd'),
        map_sha256=dataset.sha(root/'map.pcd')))
    return root


@pytest.mark.parametrize('case', ['map', 'frame', 'stamp', 'partial', 'other_map', 'wrong_count'])
def test_capture_tampering_or_partial_inputs_refused(capture, case):
    metadata = json.loads((capture/'capture.json').read_text())
    rows = json.loads((capture/'report.json').read_text())
    if case == 'map':(capture/'map.pcd').write_bytes(b'changed')
    elif case == 'frame':(capture/'frame_00.npz').write_bytes(b'changed')
    elif case == 'stamp':rows[1]['stamp_ns'] = rows[0]['stamp_ns']
    elif case == 'partial':metadata['completed'] = False
    elif case == 'other_map':metadata['map_id'] = 'old_indoor_map'
    elif case == 'wrong_count':metadata['frames'] = 3
    indoor.dump(capture/'capture.json', metadata);indoor.dump(capture/'report.json', rows)
    with pytest.raises(ValueError): indoor.verify_capture(capture)


def test_preparation_applies_pcl_once_per_source_not_per_backend(capture, tmp_path, monkeypatch):
    monkeypatch.setattr(indoor.native, 'idle_required', lambda _: None)
    calls = []
    def filter_run(args, **kwargs):
        calls.append(args)
        assert args[-1] == '0.3'
        # A PCL double for this unit test, not a real CUDA/PCL claim.
        dataset.write_pcd(args[2], [[1., 2., 3.], [4., 5., 6.]])
        return NS(stdout='2\n')
    monkeypatch.setattr(indoor.subprocess, 'run', filter_run)
    helper = tmp_path/'source_filter';helper.write_bytes(b'unit test helper')
    out = tmp_path/'dataset'
    meta = indoor.prepare_dataset(capture, out, helper)
    assert len(calls) == 2 and meta['source_leaf'] == .3 and meta['map_leaf'] == .3
    assert meta['cases'] == 4
    assert meta['frames'][0]['source_stamp_ns'] == 12345678901
    assert all(v['source_points'] == 2 for v in meta['frames'])
    assert [row.split('\t')[1] for row in (out/'cases.tsv').read_text().splitlines()] == [
        '12345678901', '12345678901', '12445678901', '12445678901']
    assert run.verify_dataset(out) == meta


def test_summarizes_both_orders(tmp_path):
    directory = tmp_path
    result_fixture(directory)
    result = indoor.summarize([directory, directory])
    assert result['backends']['NDT_OMP']['samples'] == 2
    assert result['backends']['NDT_OMP']['spatial_quality_pass'] == 2
    assert result['status'] == 'complete'
    assert result['backends']['NDT_OMP']['offline_align_equivalent_hz'] == 1000.


def test_summary_retains_nonconverged_measurements(tmp_path):
    result_fixture(tmp_path)
    file = tmp_path/'NDT_CUDA_P2D.jsonl'
    rows = [json.loads(line) for line in file.read_text().splitlines()]
    rows[1].update(converged=False, spatial_quality_pass=False)
    file.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    result = indoor.summarize([tmp_path, tmp_path])
    assert result['backends']['NDT_CUDA_P2D']['samples'] == 2
    assert result['backends']['NDT_CUDA_P2D']['spatial_quality_pass'] == 0
    assert result['backends']['NDT_CUDA_P2D']['metrics']['align_sync_ms']['n'] == 2


def test_capture_requires_confirmation_without_importing_ros(tmp_path):
    with pytest.raises(ValueError, match='confirm'):
        indoor.capture(tmp_path, tmp_path/'capture')
    assert not (tmp_path/'capture').exists()


def test_comparison_requires_confirmation_and_an_idle_device(tmp_path):
    with pytest.raises(ValueError, match='confirmation'):
        indoor.compare(tmp_path/'root', tmp_path/'capture', tmp_path/'out')
    assert list(tmp_path.iterdir()) == []


def test_comparison_refuses_output_outside_private_workspace(tmp_path, monkeypatch):
    monkeypatch.setattr(indoor.native, 'idle_required', lambda _: None)
    with pytest.raises(ValueError, match='NEW directory inside'):
        indoor.compare(tmp_path/'workspace', tmp_path/'capture', tmp_path/'outside', confirmed=True)


def test_full_comparison_uses_one_sealed_binary_identical_inputs_and_reverse_order(capture, tmp_path, monkeypatch):
    root = tmp_path/'workspace';root.mkdir()
    binary = root/'lite3_ndt_benchmark';binary.write_bytes(b'sealed')
    seal = dict(binary=str(binary), environment={}, official={'protected': 'hash'})
    monkeypatch.setattr(indoor.native, 'idle_required', lambda _: None)
    monkeypatch.setattr(indoor.native, 'check', lambda _: seal)
    monkeypatch.setattr(indoor.native, 'command', lambda *a: None)
    monkeypatch.setattr(indoor, 'prepare_dataset', lambda *a: None)
    calls, checks = [], []
    def run_fake(binary_arg, ds, output, backends, warmup, repeats, **kwargs):
        calls.append((binary_arg, ds, tuple(backends), warmup, repeats))
        return {key: 0 for key in backends}
    monkeypatch.setattr(indoor.run, 'run', run_fake)
    monkeypatch.setattr(indoor, 'summarize', lambda _: dict(status='complete', backends={}))
    monkeypatch.setattr(indoor.native, 'verify_hashes', lambda files: checks.append(files))
    out = root/'trial'
    indoor.compare(root, capture, out, confirmed=True)
    assert calls[0][:2] == calls[1][:2]
    assert calls[0][2] == tuple(reversed(calls[1][2]))
    assert checks == [seal['official']]


def test_comparison_failure_still_verifies_original_cpu(capture, tmp_path, monkeypatch):
    root = tmp_path/'workspace';root.mkdir()
    monkeypatch.setattr(indoor.native, 'idle_required', lambda _: None)
    monkeypatch.setattr(indoor.native, 'check', lambda _: dict(environment={}, official={'cpu': 'hash'}))
    def fail(*args):raise ValueError('injected compile error')
    monkeypatch.setattr(indoor.native, 'command', fail)
    verified = []
    monkeypatch.setattr(indoor.native, 'verify_hashes', lambda f: verified.append(f))
    with pytest.raises(ValueError, match='injected'):
        indoor.compare(root, capture, root/'trial', confirmed=True)
    assert verified == [{'cpu': 'hash'}]


def test_tools_have_no_motion_publishers_or_control_actions():
    text = (ROOT/'experiments/gpu_ndt/indoor_compare.py').read_text()
    assert 'create_publisher(' not in text
    assert 'ActionClient(' not in text
    assert 'SetBool' not in text
    assert 'cudaMalloc' not in text
    assert 'operator-confirmed-disarmed' in text


def test_memory_does_not_mislabel_shared_ram_as_dedicated_ndt_vram(monkeypatch):
    monkeypatch.setattr(indoor.native, 'navigation_processes', lambda: [])
    m = indoor.memory_snapshot()
    assert m['ndt_exclusive_gpu_bytes'] is None
    assert m['fixed_gpu_quota_configured'] is False
    assert m['system_shared_ram']['MemTotal_MiB'] > 0
