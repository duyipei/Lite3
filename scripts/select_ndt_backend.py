#!/usr/bin/env python3
"""Read-only selection/hash/link check. Never initializes ROS or CUDA."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

from check_localization_runtime import verify


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def select(project,backend='default'):
    project=Path(project).resolve()
    if backend not in ['default','cpu_omp','gpu_p2d']:raise ValueError('invalid NDT backend')
    if backend=='default':
        default=project/'navigation/ndt_backend.json'
        if not default.is_file():raise ValueError('default NDT release is not configured; select cpu_omp explicitly')
        backend=json.loads(default.read_text())['default_backend']
    if backend not in ['cpu_omp','gpu_p2d']:raise ValueError('invalid configured NDT backend')
    config=project/'navigation'/('cpu_baseline' if backend=='cpu_omp' else 'gpu_p2d_trial')/'runtime.json'
    if not config.is_file():raise ValueError('backend release configuration missing: '+str(config))
    release=json.loads(config.read_text())
    if release.get('backend')!=backend:raise ValueError('backend release identity mismatch')
    manifest_path=Path(release['manifest']).resolve()
    if digest(manifest_path)!=release['manifest_sha256']:raise ValueError('backend manifest changed')
    manifest=verify(manifest_path)
    if backend=='gpu_p2d' and (manifest.get('backend')!='GPU_P2D_C' or manifest.get('native_compiled') is not True):
        raise ValueError('GPU C native build is incomplete')
    prefix=Path(release['root']).resolve()/'ws/install/hdl_localization'
    binary=prefix/'lib/hdl_localization/hdl_localization_composition'
    if str(binary) not in manifest['files']:raise ValueError('selected node is absent from its manifest')
    # Mandatory original/private directories precede all inherited paths.
    # CPU selection never reads the GPU release and cannot inherit its libs.
    directories=list(release['library_dirs'])
    for name in os.environ.get('LD_LIBRARY_PATH','').split(':'):
        if not name:continue
        if backend=='cpu_omp' and ('/gpu_p2d_navigation_' in name or '/gpu_ndt_' in name):continue
        if name not in directories:directories.append(name)
    env=dict(os.environ,LD_LIBRARY_PATH=':'.join(directories),LD_PRELOAD='')
    result=subprocess.run(['ldd',str(binary)],env=env,capture_output=True,text=True,check=True)
    if 'not found' in result.stdout:raise ValueError('unresolved localization dependency')
    libraries={}
    for line in result.stdout.splitlines():
        fields=line.split()
        if len(fields)>2 and fields[1]=='=>' and fields[2].startswith('/'):
            libraries[fields[0]]=str(Path(fields[2]).resolve())
    for name,expected in release['expected_dependencies'].items():
        if libraries.get(name)!=str(Path(expected).resolve()):raise ValueError('wrong '+backend+' dependency: '+name)
    for path,expected in release['pinned_libraries'].items():
        if digest(path)!=expected:raise ValueError('backend dependency changed: '+path)
    if digest(release['launch_file'])!=release['launch_sha256']:raise ValueError('backend launch file changed')
    return dict(backend=backend,label='GPU_P2D_C' if backend=='gpu_p2d' else 'CPU_NDT_OMP',
        root=release['root'],prefix=str(prefix),binary=str(binary),
        reg_method='NDT_CUDA_P2D' if backend=='gpu_p2d' else 'NDT_OMP',
        ld_library_path=env['LD_LIBRARY_PATH'],build_id=manifest.get('build_id','unknown'),launch_file=release['launch_file'],
        libraries={name:libraries[name] for name in release['expected_dependencies']})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--backend',choices=['default','cpu_omp','gpu_p2d'],default='default')
    parser.add_argument('--format',choices=['json','lines'],default='json')
    args=parser.parse_args()
    try:
        result=select(args.project,args.backend)
        if args.format=='lines':
            for name in ['label','root','prefix','reg_method','ld_library_path','binary','build_id','launch_file']:
                if '\n' in result[name] or '\r' in result[name]:raise ValueError('invalid release path')
            print('\n'.join(result[name] for name in ['label','root','prefix','reg_method','ld_library_path','binary','build_id','launch_file']))
        else:print(json.dumps(result,indent=2))
    except (ValueError,OSError,KeyError,subprocess.SubprocessError) as error:parser.exit(2,str(error)+'\n')


if __name__=='__main__':main()
