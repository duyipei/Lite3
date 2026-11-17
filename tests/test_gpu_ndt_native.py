"""Private native-tool safety tests; no SSH, ROS node or GPU execution."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.gpu_ndt import native,run,analyze,dataset,trace
from test_gpu_ndt_experiment import capture,result_fixture


def test_build_requires_operator_confirmation_before_any_writes(tmp_path):
    with pytest.raises(ValueError,match='confirmation'):
        native.build(tmp_path/'new',tmp_path/'fast',tmp_path/'ndt',tmp_path/'official.json')
    assert list(tmp_path.iterdir())==[]


@pytest.mark.parametrize('name', ['velocity_gate','visual_pose_initializer','patrol_runner',
                                'start_localization_only.sh','voa_costmap_bridge.py'])
def test_real_ros_console_entrypoints_are_busy_even_without_py_suffix(tmp_path,name):
    p=tmp_path/'123456';p.mkdir()
    (p/'cmdline').write_bytes(b'/usr/bin/python3\0/project/install/lib/pkg/'+name.encode()+b'\0--ros-args\0')
    assert native.navigation_processes(tmp_path)==[dict(pid=123456,programs=[name])]


def test_shell_command_string_does_not_count_as_a_live_navigation_node(tmp_path):
    p=tmp_path/'123456';p.mkdir()
    (p/'cmdline').write_bytes(b'bash\0-c\0echo controller_server and velocity_gate\0')
    assert native.navigation_processes(tmp_path)==[]


@pytest.mark.parametrize('export', ['ndt_omp::ndt_omp','ndt_omp','legacy_libraries','missing'])
def test_cmake_supports_actual_foxy_target_export_not_only_legacy_variables(tmp_path,export):
    prefix=tmp_path/'prefix';prefix.mkdir()
    pcl=prefix/'PCLConfig.cmake';pcl.write_text('set(PCL_FOUND TRUE)\n')
    ndt=prefix/'ndt_ompConfig.cmake'
    if export=='legacy_libraries':ndt.write_text('set(ndt_omp_LIBRARIES legacy_cpu_library)\n')
    elif export=='missing':ndt.write_text('set(ndt_omp_FOUND TRUE)\n')
    else:
        library=prefix/'libndt_omp.so';library.write_bytes(b'configuration fixture only')
        ndt.write_text('add_library('+export+' SHARED IMPORTED)\nset_target_properties('+export+' PROPERTIES IMPORTED_LOCATION "'+str(library)+'")\n')
    source=tmp_path/'source';source.mkdir()
    for name in ['benchmark.cpp','targets.cmake']:
        (source/name).write_bytes((ROOT/'experiments/gpu_ndt'/name).read_bytes())
    (source/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.15)\nproject(test_exports LANGUAGES C CXX)\ninclude(targets.cmake)\nget_target_property(links lite3_ndt_benchmark LINK_LIBRARIES)\nfile(WRITE "${CMAKE_BINARY_DIR}/linked.txt" "${links}")\n')
    p=subprocess.run(['cmake','-S',str(source),'-B',str(tmp_path/'build'),
                      '-DCMAKE_PREFIX_PATH='+str(prefix),'-DLITE3_GPU_NDT_ENABLED=OFF'],capture_output=True,text=True)
    if export=='missing':
        assert p.returncode!=0 and 'no exported link target/libraries' in p.stderr
    else:
        assert p.returncode==0,p.stdout+p.stderr
        assert (export if export!='legacy_libraries' else 'legacy_cpu_library') in (tmp_path/'build/linked.txt').read_text()


def test_build_pins_external_cpu_library_even_when_node_manifest_omits_it(tmp_path,monkeypatch):
    inputs=tmp_path/'inputs';inputs.mkdir()
    stable=inputs/'node.bin';stable.write_bytes(b'node')
    manifest=inputs/'node_manifest.json';manifest.write_text(json.dumps(dict(files={str(stable):native.sha(stable)})))
    ndt=inputs/'ndt';(ndt/'lib').mkdir(parents=True)
    cpu=ndt/'lib/libndt_omp.so';cpu.write_bytes(b'original CPU')
    original=native.sha(cpu)
    fast=inputs/'fast';(fast/'src/fast_gicp/cuda').mkdir(parents=True)
    (fast/'src/fast_gicp/cuda/ndt_cuda.cu').write_text('source fixture')
    (fast/'CMakeLists.txt').write_text('set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")')
    monkeypatch.setattr(native,'idle_required',lambda confirmed:None)
    def interrupted(*a,**k):
        cpu.write_bytes(b'changed during build')
        raise RuntimeError('injected interruption')
    monkeypatch.setattr(native,'command',interrupted)
    root=tmp_path/'experiment'
    with pytest.raises(ValueError,match='protected/runtime file changed'):
        native.build(root,fast,ndt,manifest,True)
    assert json.loads((root/'official_before.json').read_text())[str(cpu)]==original


def test_kernel_proof_requires_executed_ndt_events_not_library_presence():
    report='Time (%),Total Time (ns),Instances,Name\n90,1000,4,"void fast_gicp::cuda::compute_ndt_derivatives()"\n10,100,2,unrelated_kernel\n'
    assert trace.ndt_kernels(report)[0]['Instances']=='4'
    assert trace.ndt_kernels('Time (%),Instances,Name\n100,0,ndt_kernel\n')==[]
    assert trace.ndt_kernels('libfast_gicp.so contains NDTCuda\n')==[]


def test_trace_requires_fresh_operator_confirmation_before_files(tmp_path):
    with pytest.raises(ValueError,match='confirmation'):
        trace.trace(tmp_path/'root',tmp_path/'dataset',tmp_path/'output','NDT_CUDA_P2D')
    assert list(tmp_path.iterdir())==[]


def test_trace_must_stay_inside_private_root(tmp_path,monkeypatch):
    monkeypatch.setattr(native,'idle_required',lambda c:None)
    monkeypatch.setattr(native,'check',lambda r:{})
    with pytest.raises(ValueError,match='private root/traces'):
        trace.trace(tmp_path/'root',tmp_path/'dataset',tmp_path/'elsewhere','NDT_CUDA_P2D',True)


def test_trace_interference_terminates_owned_group_only(tmp_path,monkeypatch):
    foreign=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True)
    real_spawn=trace.subprocess.Popen;owned=[]
    def spawn(*a,**k):
        child=real_spawn(*a,**k);owned.append(child.pid);return child
    monkeypatch.setattr(trace.subprocess,'Popen',spawn)
    monkeypatch.setattr(native,'navigation_processes',lambda:[dict(pid=foreign.pid,programs=['controller_server'])])
    try:
        with pytest.raises(ValueError,match='stopping only owned trace'):
            trace.capture([sys.executable,'-c','import time;time.sleep(30)'],tmp_path/'trace.log',dict(os.environ))
        assert foreign.poll() is None
        with pytest.raises(ProcessLookupError):os.kill(owned[0],0)
    finally:
        foreign.terminate();foreign.wait(timeout=5)


def test_nsight_target_can_have_a_different_owned_process_group(tmp_path):
    code='import subprocess,sys,time; p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(20)"],start_new_session=True); print(p.pid,flush=True); time.sleep(20)'
    owner=subprocess.Popen([sys.executable,'-u','-c',code],stdout=subprocess.PIPE,text=True,start_new_session=True)
    grandchild=int(owner.stdout.readline())
    try:
        assert os.getpgid(grandchild)!=owner.pid
        assert trace.owned_process_group(grandchild,owner.pid)
    finally:
        import signal
        for group in [grandchild,owner.pid]:
            try:os.killpg(group,signal.SIGTERM)
            except ProcessLookupError:pass
        owner.wait(timeout=5);owner.stdout.close()


@pytest.mark.parametrize('program',['navigation_start.sh','controller_server','hdl_localization_composition','velocity_gate.py','lite3_ndt_benchmark'])
def test_navigation_or_other_benchmark_prevents_build(tmp_path,monkeypatch,program):
    monkeypatch.setattr(native,'navigation_processes',lambda:[dict(pid=123,programs=[program])])
    with pytest.raises(ValueError,match='active'):
        native.build(tmp_path/'new',tmp_path/'fast',tmp_path/'ndt',tmp_path/'official.json',True)
    assert list(tmp_path.iterdir())==[]


@pytest.mark.parametrize('case',['same','within','around','exists'])
def test_build_cannot_overwrite_or_nest_in_protected_trees(tmp_path,case):
    p=tmp_path/'original';p.mkdir()
    root={'same':p,'within':p/'new','around':tmp_path,'exists':tmp_path/'exists'}[case]
    if case=='exists':root.mkdir()
    with pytest.raises(ValueError):native.independent(root,[p])


def test_environment_replaces_old_overlay_paths(tmp_path,monkeypatch):
    monkeypatch.setenv('LD_LIBRARY_PATH','/old/gpu/lib')
    monkeypatch.setenv('AMENT_PREFIX_PATH','/old/audited')
    env=native.environment(tmp_path/'experiment',tmp_path/'official_ndt')
    assert '/old/' not in str(env)
    assert env['LD_LIBRARY_PATH'].split(':')[0]==str(tmp_path/'experiment/deps/install/lib')
    assert env['LD_PRELOAD']=='' and env['OMP_NUM_THREADS']=='1'
    assert env['CMAKE_PREFIX_PATH'].split(':')[-1]=='/opt/ros/foxy'
    assert os.environ['LD_LIBRARY_PATH']=='/old/gpu/lib'


def test_official_manifest_is_verified_and_never_updated(tmp_path):
    f=tmp_path/'official.bin';f.write_bytes(b'official')
    p=tmp_path/'manifest.json';p.write_text(json.dumps(dict(files={str(f):native.sha(f)})))
    before=p.read_bytes();inventory=native.protected_files(p)
    native.verify_hashes(inventory)
    f.write_bytes(b'changed')
    with pytest.raises(ValueError):native.verify_hashes(inventory)
    with pytest.raises(ValueError):native.protected_files(p)
    assert p.read_bytes()==before


@pytest.mark.parametrize('output',[
    'libfast_gicp.so => not found\n',
    'librclcpp.so => /opt/ros/foxy/lib/librclcpp.so (0xff)\n',
])
def test_unresolved_or_ros_dependency_refused(monkeypatch,output):
    monkeypatch.setattr(native.subprocess,'run',lambda *a,**k:subprocess.CompletedProcess(a[0],0,output,''))
    with pytest.raises(ValueError):native.dependencies('/private/benchmark',{})


def test_build_guard_terminates_only_its_owned_child(tmp_path,monkeypatch):
    calls=[]
    def idle(confirmed):
        calls.append(confirmed)
        if len(calls)>1:raise ValueError('navigation started')
    monkeypatch.setattr(native,'idle_required',idle)
    with pytest.raises(ValueError,match='navigation started'):
        native.command([sys.executable,'-c','import time;time.sleep(30)'],tmp_path/'build.log',dict(os.environ),5)
    assert calls==[True,True]


def test_sampler_cleanup_uses_only_the_owned_process(tmp_path,monkeypatch):
    actions=[]
    class Child:
        pid=123
        returncode=None
        def poll(self):return self.returncode
        def terminate(self):actions.append('terminate_owned');self.returncode=-15
        def wait(self,timeout=None):actions.append('wait_owned');return self.returncode
    def spawn(args,**kwargs):
        assert args==['/fake/tegrastats','--interval','1000']
        actions.append('start_owned');return Child()
    monkeypatch.setattr(run.shutil,'which',lambda _: '/fake/tegrastats')
    monkeypatch.setattr(run.subprocess,'Popen',spawn)
    with pytest.raises(RuntimeError):
        with run.tegra_capture(tmp_path/'tegra.log') as status:raise RuntimeError('benchmark error')
    assert actions==['start_owned','terminate_owned','wait_owned']
    assert status['exit_code']==-15


def test_native_tool_never_calls_ros_launch_or_changes_power_modes():
    source=(ROOT/'experiments/gpu_ndt/native.py').read_text()
    assert "['cmake'" in source
    for forbidden in ["'sudo'","'ros2'","'jetson_clocks'","'nvpmodel'","'pkill'","--stop"]:
        assert forbidden not in source


def test_runtime_interference_stops_only_the_owned_benchmark(capture,tmp_path,monkeypatch):
    cap,mapfile,xyz=capture
    data=tmp_path/'dataset';dataset.prepare(cap,mapfile,data,.1)
    binary=tmp_path/'lite3_ndt_benchmark'
    binary.write_text('#!/usr/bin/env python3\nimport time\ntime.sleep(30)\n');binary.chmod(0o700)
    monkeypatch.setattr(run.shutil,'which',lambda _:None)
    owned=[]
    def guard(pid):
        if pid is not None:
            owned.append(pid);raise ValueError('navigation started')
    with pytest.raises(ValueError,match='navigation started'):
        run.run(binary,data,tmp_path/'run',['NDT_OMP'],1,1,5,interference_check=guard)
    assert len(owned)==1
    with pytest.raises(ProcessLookupError):os.kill(owned[0],0)


def test_tegrastats_reports_emc_power_and_missing_fields(tmp_path):
    p=tmp_path/'tegra.log'
    p.write_text('RAM 1000/6842MB EMC_FREQ 20%@1600 GR3D_FREQ 60%@1109 VDD_IN 12000mW/11000mW\n'
                 'RAM 1100/6842MB EMC_FREQ 40%@1600 GR3D_FREQ 80%@1109 VDD_IN 14000mW/12000mW\n')
    report=analyze.tegra_metrics(p)
    assert report['metrics']['emc_busy_percent']['mean']==30
    assert report['metrics']['vdd_in_mw']['mean']==13000
    p.write_text('EMC unavailable\n')
    assert analyze.tegra_metrics(p)['metrics']['emc_busy_percent']['n']==0


def test_failed_pose_differences_are_not_hidden_by_quality_filter(tmp_path):
    result_fixture(tmp_path)
    p=tmp_path/'NDT_CUDA_P2D.jsonl';rows=[json.loads(l) for l in p.read_text().splitlines()]
    rows[1]['spatial_quality_pass']=False;rows[1]['pose'][3]=1.
    p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    report=analyze.analyze(tmp_path)
    assert report['all_spatial_checks_pass'] is False
    assert report['backends']['NDT_CUDA_P2D']['failed_samples']==1
    pair=report['paired_differences']['NDT_CUDA_P2D']
    assert pair['position_difference_m']['n']==0
    assert pair['all_finite_position_difference_m']['mean']==1
