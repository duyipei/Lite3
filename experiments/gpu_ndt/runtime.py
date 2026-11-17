#!/usr/bin/env python3
"""Seal or launch only a separately built localization experiment."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess

if __package__:
    from .dataset import sha
    from .prepare import BACKENDS
else:
    from dataset import sha
    from prepare import BACKENDS


def verify_sources(workspace):
    workspace = Path(workspace).resolve()
    inventory = json.loads((workspace/'experiment_sources.json').read_text())
    if not inventory.get('experiment') or inventory.get('production_activated') is not False:
        raise ValueError('not an independent experiment')
    for name, digest in inventory['source_files'].items():
        path = (workspace/'src'/name).resolve()
        if workspace/'src' not in path.parents or sha(path) != digest:
            raise ValueError('experimental source changed: '+name)
    return inventory


def seal(workspace):
    workspace = Path(workspace).resolve()
    source = verify_sources(workspace)
    destination = workspace/'experiment_runtime.json'
    if destination.exists():
        raise ValueError('runtime inventory already exists; keep the previous build intact')
    binary = workspace/'install/hdl_localization/lib/hdl_localization/hdl_localization_composition'
    benchmark = binary.with_name('lite3_ndt_benchmark')
    cache = workspace/'build/hdl_localization/CMakeCache.txt'
    if not binary.is_file() or not benchmark.is_file() or not cache.is_file():
        raise ValueError('experimental native build is incomplete')
    cuda = 'LITE3_GPU_NDT_ENABLED:BOOL=ON' in cache.read_text()
    files = {str(binary):sha(binary), str(benchmark):sha(benchmark)}
    for name in source['source_files']:
        p = workspace/'src'/name
        files[str(p)] = sha(p)
    dependencies = subprocess.run(['ldd',str(binary)],capture_output=True,text=True,check=False)
    if dependencies.returncode or 'not found' in dependencies.stdout:
        raise ValueError('experimental runtime has unresolved libraries')
    for line in dependencies.stdout.splitlines():
        match = re.search(r'=> (/\S+)',line)
        if match:
            p = Path(match[1])
            files[str(p)] = sha(p)
    result = dict(version=1, experiment=True, cuda_enabled=cuda,
                  static_accepted=False, motion_accepted=False,
                  build_id='lite3_gpu_ndt_experiment', files=files)
    with destination.open('x') as f:
        json.dump(result,f,indent=2)
    return result


def launch_command(workspace, backend, domain, map_pcd, scan_leaf, map_leaf):
    workspace = Path(workspace).resolve()
    verify_sources(workspace)
    manifest = json.loads((workspace/'experiment_runtime.json').read_text())
    if manifest.get('experiment') is not True or backend not in BACKENDS:
        raise ValueError('invalid experimental runtime/backend')
    if backend != 'NDT_OMP' and not manifest.get('cuda_enabled'):
        raise ValueError('GPU backend requested but this workspace was built without CUDA')
    if not 100 <= domain <= 200 or str(domain) == os.environ.get('ROS_DOMAIN_ID','0'):
        raise ValueError('choose an independent ROS domain in 100..200')
    for name,digest in manifest['files'].items():
        if sha(name) != digest:
            raise ValueError('experimental runtime changed: '+name)
    if not Path(map_pcd).is_file():
        raise ValueError('map PCD missing')
    for v in (scan_leaf,map_leaf):
        if not __import__('math').isfinite(v) or v<=0:
            raise ValueError('leaf sizes must be finite and positive')
    env = dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',
               OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE',GOMP_SPINCOUNT='0')
    # Positional shell arguments are quoted by bash, never interpolated as code.
    script = '''set -e
set +u
source "$1/install/setup.bash"
actual=$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("hdl_localization"))')
test "$(readlink -f "$actual")" = "$(readlink -f "$1/install/hdl_localization")"
exec ros2 launch "$1/src/hdl_localization/launch/gpu_experiment.launch.py" "${@:2}"
'''
    args = ['bash','-c',script,'gpu_ndt_experiment',str(workspace),
            'reg_method:='+backend,'globalmap_pcd:='+str(Path(map_pcd).resolve()),
            'downsample_resolution:='+str(scan_leaf),
            'globalmap_downsample_resolution:='+str(map_leaf)]
    return args,env


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='action',required=True)
    s=sub.add_parser('seal',help='hash an already built isolated workspace; does not start nodes')
    s.add_argument('--workspace',type=Path,required=True)
    s=sub.add_parser('launch',help='future experiment only; private domain, no Nav2 or gate')
    s.add_argument('--workspace',type=Path,required=True)
    s.add_argument('--backend',choices=BACKENDS,default='NDT_OMP')
    s.add_argument('--domain',type=int,required=True)
    s.add_argument('--map',type=Path,required=True)
    s.add_argument('--scan-leaf',type=float,required=True)
    s.add_argument('--map-leaf',type=float,required=True)
    a=p.parse_args()
    try:
        if a.action=='seal':
            result=seal(a.workspace)
            print(json.dumps({k:v for k,v in result.items() if k!='files'},indent=2))
            return 0
        args,env=launch_command(a.workspace,a.backend,a.domain,a.map,a.scan_leaf,a.map_leaf)
    except (ValueError,OSError,KeyError) as error:
        p.exit(2,str(error)+'\n')
    return subprocess.call(args,env=env)


if __name__=='__main__':
    raise SystemExit(main())
