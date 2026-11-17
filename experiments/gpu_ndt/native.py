#!/usr/bin/env python3
"""Build/check only a private standalone benchmark. Never starts ROS or uses sudo."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

if __package__:
    from . import prepare, run
    from .dataset import sha
else:
    import prepare
    import run
    from dataset import sha

HERE=Path(__file__).resolve().parent
BUSY={'hdl_localization_composition','controller_server','planner_server','bt_navigator',
      'velocity_gate','velocity_gate.py','patrol_runner','patrol_runner.py',
      'navigation_start.sh','start_localization_only.sh','start_nav_isolated.sh',
      'start_gate_disarmed.sh','visual_pose_initializer','rgbd_visual_localizer',
      'voa_composition','voa_costmap_bridge.py','leg_odom_tf_bridge.py',
      'livox_ros_driver2_node','lite3_ndt_benchmark'}


def navigation_processes(proc=Path('/proc')):
    found=[]
    for p in proc.iterdir():
        if not p.name.isdigit() or int(p.name)==os.getpid():continue
        try:args=(p/'cmdline').read_bytes().split(b'\0')
        except OSError:continue
        if not args or not args[0]:continue
        first=Path(args[0].decode(errors='replace')).name
        names={first}
        if first.startswith(('python','bash','sh','env')):
            # ROS Python console entry points often have no .py suffix.
            if len(args)>1 and args[1] and not args[1].startswith(b'-'):
                names.add(Path(args[1].decode(errors='replace')).name)
            names.update(Path(a.decode(errors='replace')).name for a in args[1:]
                         if a and not a.startswith(b'-') and a.endswith((b'.py',b'.sh')))
        if names & BUSY:found.append(dict(pid=int(p.name),programs=sorted(names & BUSY)))
    return found


def idle_required(confirmed):
    if not confirmed:
        raise ValueError('operator DISARMED/stationary confirmation is required; this tool cannot read or change motion mode')
    processes=navigation_processes()
    if processes:raise ValueError('navigation/localization/another benchmark is active: '+json.dumps(processes))


def protected_files(manifest):
    path=Path(manifest).resolve()
    data=json.loads(path.read_text())
    result={str(path):sha(path)}
    for name,digest in data['files'].items():
        if sha(name)!=digest:raise ValueError('official manifest mismatch: '+name)
        result[str(Path(name).resolve())]=digest
    return result


def verify_hashes(files):
    for name,digest in files.items():
        if sha(name)!=digest:raise ValueError('protected/runtime file changed: '+name)


def independent(root,inputs):
    root=Path(root).resolve()
    if root.exists():raise ValueError('private root already exists')
    for p in inputs:
        p=Path(p).resolve()
        if p==root or p in root.parents or root in p.parents:
            raise ValueError('private root overlaps a protected/input tree')
    return root


def environment(root,ndt_prefix):
    root,ndt_prefix=Path(root).resolve(),Path(ndt_prefix).resolve()
    prefix=[root/'deps/install',ndt_prefix,Path('/opt/ros/foxy')]
    libs=[root/'deps/install/lib',ndt_prefix/'lib',Path('/usr/local/cuda-11.4/lib64'),Path('/opt/ros/foxy/lib')]
    return dict(PATH='/usr/local/cuda-11.4/bin:/usr/local/bin:/usr/bin:/bin',
                LD_LIBRARY_PATH=':'.join(map(str,libs)),LD_PRELOAD='',
                DESTDIR='',MAKEFLAGS='',CMAKE_BUILD_PARALLEL_LEVEL='1',
                CUDA_CACHE_PATH=str(root/'cuda_cache'),XDG_CACHE_HOME=str(root/'cache'),
                CMAKE_PREFIX_PATH=':'.join(map(str,prefix)),
                AMENT_PREFIX_PATH=':'.join(map(str,prefix)),COLCON_PREFIX_PATH='',
                PYTHONPATH='/opt/ros/foxy/lib/python3.8/site-packages',
                ROS_VERSION='2',ROS_DISTRO='foxy',ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='151',
                OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE',GOMP_SPINCOUNT='0')


def command(args,log,env,timeout=1800):
    """Stop only this owned process group if navigation starts or a limit expires."""
    with Path(log).open('xb') as stream:
        child=subprocess.Popen(args,stdout=stream,stderr=stream,env=env,start_new_session=True)
        started=time.monotonic()
        try:
            while child.poll() is None:
                idle_required(True)
                if time.monotonic()-started>timeout:raise TimeoutError('command time limit: '+str(args[0]))
                time.sleep(1)
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()
        if child.returncode:raise ValueError('native build failed; inspect '+str(log))


def dependencies(binary,env):
    result=subprocess.run(['ldd',str(binary)],env=env,capture_output=True,text=True,check=True)
    if 'not found' in result.stdout:raise ValueError('unresolved native libraries')
    libs={}
    for line in result.stdout.splitlines():
        parts=line.split()
        if len(parts)>2 and parts[1]=='=>' and parts[2].startswith('/'):
            libs[parts[0]]=str(Path(parts[2]).resolve())
    if any(n.startswith(('librcl','librmw','libtf2_ros')) for n in libs):
        raise ValueError('standalone benchmark unexpectedly depends on ROS runtime')
    return libs


def build(root,fast_source,ndt_prefix,official_manifest,confirmed=False,benchmark_source=None):
    idle_required(confirmed)
    fast_source,ndt_prefix=Path(fast_source).resolve(),Path(ndt_prefix).resolve()
    official_manifest=Path(official_manifest).resolve()
    official=protected_files(official_manifest)
    benchmark_source=Path(benchmark_source).resolve() if benchmark_source is not None else HERE
    for name in ['CMakeLists.txt','targets.cmake','benchmark.cpp','include/lite3_gpu_ndt/backend.hpp']:
        if not (benchmark_source/name).is_file():raise ValueError('incomplete standalone benchmark source: '+name)
    root=independent(root,[fast_source,ndt_prefix,official_manifest.parent,HERE,benchmark_source])
    cpu=ndt_prefix/'lib/libndt_omp.so'
    if not cpu.is_file():raise ValueError('official ndt_omp library missing')
    # The audited node manifest may omit its external ndt_omp dependency.
    # Pin that exact installed CPU library before any experimental build.
    official[str(cpu.resolve())]=sha(cpu)
    if not (fast_source/'src/fast_gicp/cuda/ndt_cuda.cu').is_file():raise ValueError('complete fast_gicp source required')
    cmake=(fast_source/'CMakeLists.txt').read_text()
    known_sm72=prepare.patch_cuda_arch('set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")')
    # A new private variant may be based on our already-built sm_72 source.
    # Accept only that exact known flag; arbitrary flag layouts still fail.
    patched=cmake if cmake.count(known_sm72)==1 else prepare.patch_cuda_arch(cmake)
    root.mkdir(parents=True)
    (root/'cuda_cache').mkdir();(root/'cache').mkdir()
    (root/'official_before.json').write_text(json.dumps(official,indent=2))
    source=root/'deps/src/fast_gicp'
    shutil.copytree(fast_source,source,ignore=shutil.ignore_patterns('.git','build','install','log'))
    (source/'CMakeLists.txt').write_text(patched)
    bench=root/'source';bench.mkdir()
    for name in ['CMakeLists.txt','targets.cmake','benchmark.cpp']:shutil.copy2(benchmark_source/name,bench/name)
    shutil.copytree(benchmark_source/'include',bench/'include')
    env=dict(os.environ);env.update(environment(root,ndt_prefix))
    commands=[
        ['cmake','-S',str(source),'-B',str(root/'deps/build'),'-DBUILD_VGICP_CUDA=ON',
         '-DBUILD_apps=OFF','-DBUILD_test=OFF','-DBUILD_TESTING=OFF','-DBUILD_PYTHON_BINDINGS=OFF',
         '-DCMAKE_INSTALL_PREFIX='+str(root/'deps/install'),'-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4'],
        ['cmake','--build',str(root/'deps/build'),'--','-j1'],
        ['cmake','--install',str(root/'deps/build')],
        ['cmake','-S',str(bench),'-B',str(root/'build'),'-DLITE3_GPU_NDT_ENABLED=ON',
         '-Dndt_omp_DIR='+str(ndt_prefix/'share/ndt_omp/cmake'),
         '-Dfast_gicp_DIR='+str(root/'deps/install/share/fast_gicp/cmake'),
         '-DLITE3_FAST_GICP_LIBRARY='+str(root/'deps/install/lib/libfast_gicp.so'),
         '-DLITE3_FAST_CUDA_LIBRARY='+str(root/'deps/install/lib/libfast_vgicp_cuda.so'),
         '-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4','-DCMAKE_BUILD_TYPE=Release',
         '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG','-DCMAKE_INSTALL_PREFIX='+str(root/'install')],
        ['cmake','--build',str(root/'build'),'--','-j1'],
    ]
    (root/'build_commands.json').write_text(json.dumps(commands,indent=2))
    try:
        for i,args in enumerate(commands):command(args,root/('build_%02d.log'%i),env)
        return finalize(root,ndt_prefix,official)
    finally:verify_hashes(official)


def finalize(root,ndt_prefix,official):
    """Seal a completed private build, including a repaired link-only retry."""
    idle_required(True)
    root,ndt_prefix=Path(root).resolve(),Path(ndt_prefix).resolve()
    verify_hashes(official)
    if (root/'native_runtime.json').exists():raise ValueError('native runtime already sealed')
    if any(root==Path(p) or root in Path(p).parents for p in official):
        raise ValueError('private root contains protected files')
    env=dict(os.environ);env.update(environment(root,ndt_prefix))
    binary=root/'build/lite3_ndt_benchmark';libs=dependencies(binary,env)
    expected={'libndt_omp.so':str((ndt_prefix/'lib/libndt_omp.so').resolve()),
              'libfast_gicp.so':str((root/'deps/install/lib/libfast_gicp.so').resolve()),
              'libfast_vgicp_cuda.so':str((root/'deps/install/lib/libfast_vgicp_cuda.so').resolve())}
    if expected['libndt_omp.so'] not in official:
        raise ValueError('CPU baseline library is not in the protected production inventory')
    if any(libs.get(n)!=p for n,p in expected.items()):raise ValueError('native CPU/GPU dependency isolation failed')
    arch=subprocess.run(['/usr/local/cuda-11.4/bin/cuobjdump','--list-elf',expected['libfast_vgicp_cuda.so']],
                        capture_output=True,text=True,check=True).stdout
    if 'sm_72' not in arch:raise ValueError('native sm_72 cubins not present')
    (root/'cuda_arch.txt').write_text(arch)
    files={str(binary):sha(binary),**{p:sha(p) for p in libs.values()}}
    for p in (root/'source').rglob('*'):
        if p.is_file():files[str(p)]=sha(p)
    # Pin headers and implementation together for experimental ABI changes.
    # A partial library/header replacement must invalidate the private runtime.
    for p in (root/'deps/src/fast_gicp').rglob('*'):
        if p.is_file():files[str(p)]=sha(p)
    for directory in [root/'deps/install/include',root/'deps/install/share/fast_gicp']:
        for p in directory.rglob('*'):
            if p.is_file():files[str(p)]=sha(p)
    result=dict(scope='standalone_registration_only',ros_publications=False,root=str(root),
                binary=str(binary),ndt_prefix=str(ndt_prefix),files=files,official=official,
                expected_dependencies=expected,environment=environment(root,ndt_prefix),
                kernel_execution_verified=False)
    with (root/'native_runtime.json').open('x') as f:json.dump(result,f,indent=2)
    return result


def check(root):
    root=Path(root).resolve();data=json.loads((root/'native_runtime.json').read_text())
    if data.get('scope')!='standalone_registration_only' or data.get('ros_publications') is not False:
        raise ValueError('invalid standalone manifest')
    if Path(data['root'])!=root:raise ValueError('native runtime moved; rebuild in a fresh root')
    verify_hashes(data['files']);verify_hashes(data['official'])
    env=dict(os.environ);env.update(data['environment'])
    libs=dependencies(data['binary'],env)
    if any(libs.get(n)!=p for n,p in data['expected_dependencies'].items()):raise ValueError('runtime library resolution changed')
    return data


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    b=sub.add_parser('build',help='future idle Jetson only; private source/libraries, one build job')
    for name in ['root','fast-source','ndt-prefix','official-manifest']:b.add_argument('--'+name,type=Path,required=True)
    b.add_argument('--operator-confirmed-disarmed',action='store_true')
    c=sub.add_parser('check',help='read-only source/binary/library/protected-file checks');c.add_argument('--root',type=Path,required=True)
    r=sub.add_parser('run',help='standalone benchmark only; refuses navigation/another benchmark')
    for name in ['root','dataset','output']:r.add_argument('--'+name,type=Path,required=True)
    r.add_argument('--operator-confirmed-disarmed',action='store_true')
    r.add_argument('--backend',choices=prepare.BACKENDS,action='append')
    r.add_argument('--warmup',type=int,default=3);r.add_argument('--repeats',type=int,default=5)
    a=p.parse_args()
    try:
        if a.action=='build':result=build(a.root,a.fast_source,a.ndt_prefix,a.official_manifest,a.operator_confirmed_disarmed)
        elif a.action=='check':result=check(a.root)
        else:
            idle_required(a.operator_confirmed_disarmed);data=check(a.root)
            output=a.output.resolve()
            if a.root.resolve() not in output.parents:raise ValueError('run output must be inside the private experiment root')
            def guard(owned_pid):
                # The owned child is the sole benchmark permitted while run.py is sampling.
                busy=[q for q in navigation_processes() if q['pid']!=owned_pid]
                if busy:raise ValueError('navigation/localization started; stopping only this experiment')
            try:
                result=run.run(data['binary'],a.dataset,a.output,a.backend or prepare.BACKENDS,
                               a.warmup,a.repeats,interference_check=guard,env_override=data['environment'])
            finally:verify_hashes(data['official'])
        print(json.dumps({k:v for k,v in result.items() if k not in ['files','official','environment']},indent=2))
        return 0 if a.action!='run' or all(v==0 for v in result.values()) else 1
    except (ValueError,OSError,KeyError,subprocess.SubprocessError) as e:p.exit(2,str(e)+'\n')


if __name__=='__main__':raise SystemExit(main())
