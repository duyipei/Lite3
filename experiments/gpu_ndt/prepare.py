#!/usr/bin/env python3
"""Generate an independent experimental workspace; never build or deploy it."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BACKENDS = ('NDT_OMP', 'NDT_CUDA_P2D', 'NDT_CUDA_D2D')


def method_bounds(text, signature):
    start = text.index(signature)
    opening = text.index('{', start)
    depth = 1
    for end in range(opening+1, len(text)):
        depth += (text[end] == '{') - (text[end] == '}')
        if not depth:
            return start, end+1
    raise ValueError('unclosed method: '+signature)


def experiment_header(text):
    start, end = method_bounds(text, '    pcl::Registration<PointT, PointT>::Ptr CreateRegistration()')
    text = text[:start] + '''    pcl::Registration<PointT, PointT>::Ptr CreateRegistration(){
        return lite3_gpu_ndt::make_registration<PointT>(
            reg_method, ndt_resolution, ndt_neighbor_search_method);
    }''' + text[end:]
    required = '#include <fast_gicp/ndt/ndt_cuda.hpp>'
    if text.count(required) != 1:
        raise ValueError('unexpected CUDA include layout')
    text = text.replace(required, '#include <lite3_gpu_ndt/backend.hpp>')
    text = text.replace('        InitParams();\n', '''        InitParams();
        lite3_gpu_ndt::preflight(reg_method);  // Before waiting for sensor TF.
''', 1)
    old = 'delta_estimater.reset(new DeltaEstimater(CreateRegistration()));'
    if text.count(old) != 1:
        raise ValueError('unexpected relocalization matcher layout')
    # Keep the existing CPU fallback, rather than allocate a second CUDA matcher
    # whose source/target changes on every relocalization delta frame.
    text = text.replace(old, '''delta_estimater.reset(new DeltaEstimater(
            lite3_gpu_ndt::make_registration<PointT>(
                "NDT_OMP", ndt_resolution, ndt_neighbor_search_method)));''')
    return text


def patch_cuda_arch(text):
    old = 'set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")'
    if text.count(old) != 1:
        raise ValueError('unsupported fast_gicp CMake CUDA flag layout')
    return text.replace(old, '''set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};--expt-relaxed-constexpr;-gencode=arch=compute_72,code=sm_72;-gencode=arch=compute_72,code=compute_72")''')


def inventory(directory):
    return {p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(directory.rglob('*')) if p.is_file()}


def experiment_launch(text):
    """Reuse the exact production localization parameters, omit all Nav2 nodes."""
    if text.count('    nav2_launch = IncludeLaunchDescription(') != 1:
        raise ValueError('unexpected production launch layout')
    text = text.split('    nav2_launch = IncludeLaunchDescription(', 1)[0]
    text = text.replace('def generate_launch_description():\n', '''def generate_launch_description():
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not 100 <= domain <= 200 or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("experiment requires isolated ROS_DOMAIN_ID 100..200 and ROS_LOCALHOST_ONLY=1")
''', 1)
    if text.count('"reg_method": "NDT_OMP"') != 1:
        raise ValueError('unexpected production backend setting')
    text = text.replace('"reg_method": "NDT_OMP"',
                        '"reg_method": LaunchConfiguration("reg_method")')
    return text + '''    return LaunchDescription([
        DeclareLaunchArgument("globalmap_pcd"),
        DeclareLaunchArgument("reg_method", default_value="NDT_OMP",
            choices=["NDT_OMP", "NDT_CUDA_P2D", "NDT_CUDA_D2D"]),
        DeclareLaunchArgument("lidar_topic", default_value="/gpu_ndt/input/points"),
        DeclareLaunchArgument("imu_topic", default_value="/gpu_ndt/input/imu"),
        DeclareLaunchArgument("start_localization", default_value="True"),
        DeclareLaunchArgument("specify_init_pose", default_value="False"),
        DeclareLaunchArgument("use_imu", default_value="False"),
        DeclareLaunchArgument("convert_utm_to_local", default_value="False"),
        DeclareLaunchArgument("ndt_resolution", default_value="1.0"),
        DeclareLaunchArgument("fine_registration_enabled", default_value="false"),
        DeclareLaunchArgument("downsample_resolution", default_value="0.3"),
        DeclareLaunchArgument("globalmap_downsample_resolution", default_value=downsample_resolution),
        localization,
    ])
'''


def prepare(base, output, fast_gicp=None):
    base, output = Path(base).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('output already exists; choose a new experimental workspace')
    # In particular, refuse existing audited/current/vendor roots and nesting in
    # an input package. There is no overwrite or install-prefix option.
    if base in output.parents or output in base.parents or output == base:
        raise ValueError('experimental output must be independent of the base package')
    for required in ('CMakeLists.txt', 'package.xml', 'msg/ScanMatchingStatus.msg',
                     'include/hdl_localization/delta_estimater.hpp',
                     'include/kkl/alg/unscented_kalman_filter.hpp'):
        if not (base/required).is_file():
            raise ValueError('incomplete base source package: '+required)
    if any(p.is_symlink() for p in base.rglob('*')):
        raise ValueError('base source contains symlinks; provide a regular source snapshot')
    header = experiment_header((REPO/'vendor_patch/hdl_localization_app.hpp').read_text())
    launch = experiment_launch((REPO/'patrol_nav_ws/src/lite3_patrol_nav/launch/patrol_navigation.launch.py').read_text())
    cmake = (base/'CMakeLists.txt').read_text()
    if cmake.count('ament_auto_package()') != 1:
        raise ValueError('unexpected hdl_localization CMake layout')
    fast_text = None
    if fast_gicp:
        fast_gicp = Path(fast_gicp).resolve()
        if fast_gicp in output.parents or output in fast_gicp.parents or fast_gicp == output:
            raise ValueError('experimental output must be independent of fast_gicp')
        for required in ('CMakeLists.txt','package.xml','include/fast_gicp/ndt/ndt_cuda.hpp',
                         'include/fast_gicp/cuda/ndt_cuda.cuh','src/fast_gicp/cuda/ndt_cuda.cu'):
            if not (fast_gicp/required).is_file():
                raise ValueError('incomplete fast_gicp source: '+required)
        if any(p.is_symlink() for p in fast_gicp.rglob('*')):
            raise ValueError('fast_gicp source contains symlinks; provide a regular source snapshot')
        fast_text = patch_cuda_arch((fast_gicp/'CMakeLists.txt').read_text())
    output.mkdir(parents=True)
    package = output/'src/hdl_localization'
    shutil.copytree(base, package, ignore=shutil.ignore_patterns('.git', 'build', 'install', 'log'))
    # Overlay the CURRENT worktree, including all P0/P1, fusion and UKF repairs.
    for p in (REPO/'vendor_patch/hdl_localization').rglob('*'):
        if not p.is_file():
            continue
        rel = p.relative_to(REPO/'vendor_patch/hdl_localization')
        if rel.name in ('hdl_localization_composition.cpp', 'globalmap_server_app.hpp'):
            rel = Path('apps')/rel
        elif rel.name == 'lite_localization.launch.py':
            rel = Path('launch')/rel
        dest = package/rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dest)
    (package/'apps/hdl_localization_app.hpp').write_text(header)
    exp = package/'gpu_experiment'
    shutil.copytree(HERE/'include', exp/'include')
    for name in ('benchmark.cpp', 'targets.cmake'):
        shutil.copy2(HERE/name, exp/name)
    (package/'launch/gpu_experiment.launch.py').write_text(launch)
    (package/'CMakeLists.txt').write_text(cmake.replace(
        'ament_auto_package()', 'include(gpu_experiment/targets.cmake)\n\nament_auto_package()'))
    if fast_gicp:
        target = output/'src/fast_gicp'
        shutil.copytree(Path(fast_gicp).resolve(), target,
                        ignore=shutil.ignore_patterns('.git', 'build', 'install', 'log'))
        (target/'CMakeLists.txt').write_text(fast_text)
    manifest = dict(schema_version=1, experiment=True, production_activated=False,
                    default_backend='NDT_OMP', cuda_enabled_by_default=False,
                    base_source=str(base), current_worktree_header_sha256=hashlib.sha256(
                        (REPO/'vendor_patch/hdl_localization_app.hpp').read_bytes()).hexdigest(),
                    source_files=inventory(output/'src'), includes_sm72_build=bool(fast_gicp))
    (output/'experiment_sources.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-package', type=Path, required=True,
                        help='complete audited hdl_localization source snapshot; read only')
    parser.add_argument('--output', type=Path, required=True, help='new workspace; must not exist')
    parser.add_argument('--fast-gicp-source', type=Path,
                        help='optional complete local fast_gicp tree for isolated sm_72 rebuild')
    args = parser.parse_args()
    try:
        result = prepare(args.base_package, args.output, args.fast_gicp_source)
    except (ValueError, OSError) as error:
        parser.exit(2, str(error)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'source_files'}, indent=2))


if __name__ == '__main__':
    main()
