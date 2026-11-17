#!/usr/bin/env python3
"""Build a private C library and ROS node; never execute registration or ROS."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from .. import native,prepare
from ..p2d_optimization import patch

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[2]


def write(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as stream:json.dump(data,stream,indent=2);stream.write('\n')


def identity_source(build_id):
    return '\nextern "C" const char* lite3_gpu_p2d_c_build_id() { return "'+build_id+'"; }\n'


def environment(root,cpu_prefix):
    env=dict(os.environ);env.update(native.environment(root,cpu_prefix))
    extra='/home/ysc/lite_cog_ros2/nav/install'
    packages=sorted(str(p) for p in Path(extra).iterdir() if (p/'share').is_dir() and p.name!='hdl_localization')
    env['CMAKE_PREFIX_PATH']=':'.join([str(root/'deps/install'),str(cpu_prefix),*packages,'/opt/ros/foxy'])
    env['AMENT_PREFIX_PATH']=env['CMAKE_PREFIX_PATH']
    env['LD_LIBRARY_PATH']=':'.join([str(root/'ws/install/hdl_localization/lib'),str(root/'deps/install/lib'),str(cpu_prefix/'lib'),
        *[p+'/lib' for p in packages],'/usr/local/cuda-11.4/lib64','/opt/ros/foxy/lib',
        '/opt/ros/foxy/lib/aarch64-linux-gnu'])
    return env


def deps(root,fast_source,cpu_manifest,cpu_prefix,build_id,confirmed):
    native.idle_required(confirmed)
    root=Path(root).resolve();fast_source=Path(fast_source).resolve();cpu_prefix=Path(cpu_prefix).resolve()
    official=native.protected_files(cpu_manifest)
    cpu_lib=cpu_prefix/'lib/libndt_omp.so';official[str(cpu_lib)]=native.sha(cpu_lib)
    native.independent(root,[fast_source,Path(cpu_manifest).resolve().parent,cpu_prefix,REPO])
    if not build_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in build_id):
        raise ValueError('invalid build id')
    root.mkdir(parents=True)
    write(root/'official_before.json',official)
    source=root/'deps/src/fast_gicp'
    variant=patch.prepare(fast_source,source,'C')
    # The algorithm is exactly variant C. Append only an exported identity
    # symbol, for static linking checks and future startup dladdr logging.
    core=source/patch.CORE
    core.write_text(core.read_text()+identity_source(build_id))
    cmake=source/'CMakeLists.txt';text=cmake.read_text()
    known=prepare.patch_cuda_arch('set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")')
    cmake.write_text(text if text.count(known)==1 else prepare.patch_cuda_arch(text))
    env=environment(root,cpu_prefix)
    commands=[['cmake','-S',str(source),'-B',str(root/'deps/build'),'-DBUILD_VGICP_CUDA=ON',
        '-DBUILD_apps=OFF','-DBUILD_test=OFF','-DBUILD_TESTING=OFF','-DBUILD_PYTHON_BINDINGS=OFF',
        '-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4','-DCMAKE_INSTALL_PREFIX='+str(root/'deps/install')],
        ['cmake','--build',str(root/'deps/build'),'--','-j1'],
        ['cmake','--install',str(root/'deps/build')]]
    write(root/'deps_commands.json',commands)
    try:
        for i,command in enumerate(commands):native.command(command,root/('deps_%02d.log'%i),env,timeout=3600)
        lib=root/'deps/install/lib/libfast_vgicp_cuda.so'
        arch=subprocess.run(['/usr/local/cuda-11.4/bin/cuobjdump','--list-elf',str(lib)],capture_output=True,text=True,check=True).stdout
        if 'sm_72' not in arch:raise ValueError('sm_72 cubins missing')
        symbols=subprocess.run(['nm','-D','-C',str(lib)],capture_output=True,text=True,check=True).stdout
        for required in ['p2d_ndt_compute_error_only','lite3_gpu_p2d_c_build_id','NDTCudaCore::compute_error']:
            if required not in symbols:raise ValueError('required C library symbol missing: '+required)
        (root/'cuda_arch.txt').write_text(arch);(root/'cuda_symbols.txt').write_text(symbols)
        write(root/'deps_complete.json',dict(build_id=build_id,variant='C',cpu_prefix=str(cpu_prefix),
            cpu_manifest=str(Path(cpu_manifest).resolve()),official=official,
            modified_algorithm_files=variant['modified_files'],native_compiled=True,runtime_tested=False,
            files={str(p):native.sha(p) for p in [lib,root/'deps/install/lib/libfast_gicp.so']}))
    finally:native.verify_hashes(official)


def node_header(text,build_id):
    original=text
    start,end=prepare.method_bounds(text,'    pcl::Registration<PointT, PointT>::Ptr CreateRegistration()')
    factory=text[start:end]
    a,b=prepare.method_bounds(factory,'        if(reg_method == "NDT_OMP")')
    cpu=factory[a:b]
    replacement='''    pcl::Registration<PointT, PointT>::Ptr CreateRegistration(){
        if(reg_method == "NDT_CUDA_P2D") {
            const char* actual=lite3_gpu_p2d_c_build_id();
            if(std::string(actual)!="'''+build_id+'''") throw std::runtime_error("GPU C build identity mismatch");
            Dl_info library{};
            if(!dladdr(reinterpret_cast<const void*>(&lite3_gpu_p2d_c_build_id), &library) || !library.dli_fname)
                throw std::runtime_error("cannot identify loaded GPU C library");
            auto result=lite3_gpu_ndt::make_registration<PointT>(reg_method,ndt_resolution,ndt_neighbor_search_method);
            RCLCPP_INFO(get_logger(),"NDT_BACKEND=GPU_P2D_C build=%s library=%s optimizer=CPU_LM",actual,library.dli_fname);
            return result;
        }
'''+cpu+'''
        throw std::runtime_error("navigation trial supports only NDT_OMP or NDT_CUDA_P2D");
    }'''
    text=text[:start]+replacement+text[end:]
    text=patch.replace_once(text,'#include <fast_gicp/ndt/ndt_cuda.hpp>',
        '#include <lite3_gpu_ndt/backend.hpp>\n#include <dlfcn.h>\nextern "C" const char* lite3_gpu_p2d_c_build_id();')
    text=patch.replace_once(text,'        InitParams();\n',
        '        InitParams();\n        lite3_gpu_ndt::preflight(reg_method);\n')
    # Safety, fusion and publication routines must be copied verbatim.
    for signature in ['std::vector<float> TargetDistances(', 'double BatchFitnessScore(',
        'bool TryPromoteStableNdtCandidate(', 'bool FuseNdtCorrectionWithLegOdom(',
        'void LidarCallback(', 'void PublishOdometry(', 'void PublishScanMatchingStatus(']:
        a,b=prepare.method_bounds(original,signature);c,d=prepare.method_bounds(text,signature)
        if original[a:b]!=text[c:d]:raise ValueError('protected quality/fusion routine changed: '+signature)
    return text


def node(root,base_package):
    native.idle_required(True)
    root=Path(root).resolve();state=json.loads((root/'deps_complete.json').read_text())
    native.verify_hashes(state['official']);native.verify_hashes(state['files'])
    package=root/'ws/src/hdl_localization'
    if package.exists():raise ValueError('private node source already exists; keep failed build intact')
    shutil.copytree(Path(base_package).resolve(),package,ignore=shutil.ignore_patterns('.git','build','install','log'))
    for p in (REPO/'vendor_patch/hdl_localization').rglob('*'):
        if not p.is_file():continue
        rel=p.relative_to(REPO/'vendor_patch/hdl_localization')
        if rel.name in ['hdl_localization_composition.cpp','globalmap_server_app.hpp']:rel=Path('apps')/rel
        elif rel.name=='lite_localization.launch.py':rel=Path('launch')/rel
        dest=package/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
    header=(REPO/'vendor_patch/hdl_localization_app.hpp').read_text()
    (package/'apps/hdl_localization_app.hpp').write_text(node_header(header,state['build_id']))
    launch=(REPO/'patrol_nav_ws/src/lite3_patrol_nav/launch/patrol_navigation.launch.py').read_text()
    launch=patch.replace_once(launch,'"reg_method": "NDT_OMP"','"reg_method": "NDT_CUDA_P2D"')
    launch=patch.replace_once(launch,'DeclareLaunchArgument("start_navigation", default_value="True")',
                              'DeclareLaunchArgument("start_navigation", default_value="False")')
    (package/'launch/gpu_p2d_navigation.launch.py').write_text(launch)
    target=package/'gpu_navigation/include';shutil.copytree(REPO/'experiments/gpu_ndt/include',target)
    cmake=(package/'CMakeLists.txt').read_text()
    addition='''
find_package(CUDA 11.4 REQUIRED)
target_compile_definitions(hdl_localization_composition PRIVATE LITE3_GPU_NDT_ENABLED)
target_include_directories(hdl_localization_composition BEFORE PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/gpu_navigation/include" "'''+str(root/'deps/install/include')+'''" ${CUDA_INCLUDE_DIRS})
target_link_libraries(hdl_localization_composition
  "'''+str(root/'deps/install/lib/libfast_gicp.so')+'''"
  "'''+str(root/'deps/install/lib/libfast_vgicp_cuda.so')+'''" ${CUDA_CUDART_LIBRARY} ${CMAKE_DL_LIBS})
set_target_properties(hdl_localization_composition PROPERTIES
  INSTALL_RPATH "'''+str(root/'deps/install/lib')+'''" BUILD_WITH_INSTALL_RPATH TRUE)
'''
    (package/'CMakeLists.txt').write_text(patch.replace_once(cmake,'ament_auto_package()',addition+'\nament_auto_package()'))
    env=environment(root,Path(state['cpu_prefix']))
    command=['colcon','build','--base-paths',str(root/'ws/src'),'--build-base',str(root/'ws/build'),
        '--install-base',str(root/'ws/install'),'--log-base',str(root/'ws/log'),
        '--executor','sequential','--packages-select','hdl_localization','--cmake-args',
        '-DBUILD_TESTING=OFF','-DCMAKE_BUILD_TYPE=Release','-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG',
        '-Dfast_gicp_DIR='+str(root/'deps/install/share/fast_gicp/cmake'),
        '-Dndt_omp_DIR='+str(Path(state['cpu_prefix'])/'share/ndt_omp/cmake')]
    # colcon's --log-base is a global argument, before the build verb.
    command=['colcon','--log-base',str(root/'ws/log')]+command[1:]
    pos=command.index('--log-base',3);del command[pos:pos+2]
    write(root/'node_commands.json',[command])
    env['MAKEFLAGS']='-j1'
    try:native.command(command,root/'node_build.log',env,timeout=3600)
    finally:native.verify_hashes(state['official'])


def seal(root):
    root=Path(root).resolve();state=json.loads((root/'deps_complete.json').read_text())
    native.verify_hashes(state['official']);native.verify_hashes(state['files'])
    env=environment(root,Path(state['cpu_prefix']))
    binary=root/'ws/install/hdl_localization/lib/hdl_localization/hdl_localization_composition'
    output=subprocess.run(['ldd',str(binary)],env=env,capture_output=True,text=True,check=True).stdout
    if 'not found' in output:raise ValueError('unresolved node dependency')
    expected={'libfast_gicp.so':str(root/'deps/install/lib/libfast_gicp.so'),
              'libfast_vgicp_cuda.so':str(root/'deps/install/lib/libfast_vgicp_cuda.so'),
              'libndt_omp.so':str(Path(state['cpu_prefix'])/'lib/libndt_omp.so')}
    libs={}
    for line in output.splitlines():
        fields=line.split()
        if len(fields)>2 and fields[1]=='=>' and fields[2].startswith('/'):
            libs[fields[0]]=str(Path(fields[2]).resolve())
    for name,path in expected.items():
        if libs.get(name)!=str(Path(path).resolve()):raise ValueError('wrong node dependency: '+name)
    (root/'node_ldd.txt').write_text(output)
    files={str(p):native.sha(p) for p in [binary,*map(Path,libs.values())]}
    for directory in [root/'ws/src',root/'ws/install',root/'deps/src',root/'deps/install']:
        for p in directory.rglob('*'):
            if p.is_file():files[str(p.resolve())]=native.sha(p)
    write(root/'runtime_manifest.json',dict(version=1,build_id=state['build_id'],backend='GPU_P2D_C',
        native_compiled=True,runtime_tested=False,numerical_accepted=False,performance_accepted=False,
        binary=str(binary),files=files,expected_dependencies=expected,
        environment={k:env[k] for k in ['LD_LIBRARY_PATH','CMAKE_PREFIX_PATH','AMENT_PREFIX_PATH','OMP_NUM_THREADS','CUDA_CACHE_PATH']},
        cpu_root=str(Path(state['cpu_manifest']).parent),cpu_manifest=state['cpu_manifest'],official=state['official']))


def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='action',required=True)
    d=s.add_parser('deps')
    for name in ['root','fast-source','cpu-manifest','cpu-prefix','build-id']:d.add_argument('--'+name,required=True)
    d.add_argument('--operator-confirmed-no-navigation',action='store_true')
    n=s.add_parser('node');n.add_argument('--root',required=True);n.add_argument('--base-package',required=True)
    q=s.add_parser('seal');q.add_argument('--root',required=True)
    a=p.parse_args()
    if a.action=='deps':deps(a.root,a.fast_source,a.cpu_manifest,a.cpu_prefix,a.build_id,a.operator_confirmed_no_navigation)
    elif a.action=='node':node(a.root,a.base_package)
    else:seal(a.root)
    print(json.dumps(dict(action=a.action,root=a.root,completed=True,runtime_tested=False)))


if __name__=='__main__':main()
