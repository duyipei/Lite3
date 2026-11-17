#!/usr/bin/env python3
"""Activate an already linked trial for the NEXT manual start; never launch."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys

from .. import native

REPO=Path(__file__).resolve().parents[3]


def load(path):return json.loads(Path(path).read_text())


def atomic_copy(source,dest):
    dest=Path(dest);dest.parent.mkdir(parents=True,exist_ok=True)
    temporary=dest.with_name(dest.name+'.gpu_stage')
    if temporary.exists():raise ValueError('staging file already exists: '+str(temporary))
    shutil.copy2(source,temporary);os.replace(temporary,dest)


def atomic_json(data,path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.gpu_stage')
    with temporary.open('x') as f:json.dump(data,f,indent=2);f.write('\n')
    os.replace(temporary,path)


def activate(project,root,tooling,cpu_release):
    native.idle_required(True)
    project=Path(project).resolve();root=Path(root).resolve();tooling=Path(tooling).resolve()
    state=load(root/'runtime_manifest.json');cpu=load(cpu_release)
    if state.get('backend')!='GPU_P2D_C' or state.get('native_compiled') is not True:
        raise ValueError('GPU node build/link is incomplete')
    native.verify_hashes(state['files']);native.verify_hashes(state['official'])
    native.verify_hashes(cpu['pinned_libraries'])
    if native.sha(cpu['manifest'])!=cpu['manifest_sha256']:raise ValueError('original CPU manifest changed')
    launch=root/'ws/install/hdl_localization/share/hdl_localization/launch/gpu_p2d_navigation.launch.py'
    gpu=dict(backend='gpu_p2d',root=str(root),manifest=str(root/'runtime_manifest.json'),
        manifest_sha256=native.sha(root/'runtime_manifest.json'),
        expected_dependencies=state['expected_dependencies'],
        pinned_libraries={p:native.sha(p) for p in state['expected_dependencies'].values()},
        library_dirs=state['environment']['LD_LIBRARY_PATH'].split(':'),
        launch_file=str(launch),launch_sha256=native.sha(launch))
    relative=['scripts/select_ndt_backend.py','scripts/navigation_start.sh',
        'scripts/navigation_cpu_omp.sh','scripts/navigation_gpu_p2d.sh',
        'navigation/ndt_runtime_env.sh','navigation/start_localization_only.sh']
    configs=['navigation/cpu_baseline/runtime.json','navigation/gpu_p2d_trial/runtime.json','navigation/ndt_backend.json']
    backup=root.parent/'before_deploy'
    backup.mkdir()
    before={}
    for name in relative+configs:
        path=project/name
        before[name]=native.sha(path) if path.exists() else None
        if path.is_file():
            dest=backup/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,dest)
    (backup/'inventory.json').write_text(json.dumps(before,indent=2)+'\n')
    for name in relative:
        if not (tooling/name).is_file():raise ValueError('missing deployment input: '+name)
    # Only release metadata/startup files are deployed. No system package,
    # original CPU prefix, common Nav2 launch or robot service is replaced.
    changed=[]
    try:
        for name in relative:
            atomic_copy(tooling/name,project/name);changed.append(name)
        atomic_json(cpu,project/configs[0]);changed.append(configs[0])
        atomic_json(gpu,project/configs[1]);changed.append(configs[1])
        spec=importlib.util.spec_from_file_location('selected_ndt_backend',project/'scripts/select_ndt_backend.py')
        module=importlib.util.module_from_spec(spec)
        sys.path.insert(0,str(project/'scripts'))
        try:spec.loader.exec_module(module)
        finally:sys.path.pop(0)
        # Hash+ldd inspection only. Does not invoke ROS, CUDA or the node binary.
        selected={mode:module.select(project,mode) for mode in ['cpu_omp','gpu_p2d']}
        native.idle_required(True)
        atomic_json(dict(schema_version=1,default_backend='gpu_p2d',build_id=state['build_id'],
                         runtime_tested=False),project/configs[2]);changed.append(configs[2])
        native.verify_hashes(state['official']);native.verify_hashes(cpu['pinned_libraries'])
    except BaseException:
        # Undo only files written by this activation, retaining all backups
        # and failed build logs. Never leave a partially activated GPU default.
        for name in reversed(changed):
            if before[name] is None:(project/name).unlink()
            else:atomic_copy(backup/name,project/name)
        raise
    result=dict(default_backend='gpu_p2d',build_id=state['build_id'],runtime_tested=False,
        benchmark_executed=False,navigation_started=False,source=relative,
        deployed={name:native.sha(project/name) for name in relative+configs},
        selected=selected,backup=str(backup))
    with (root.parent/'deployment.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['project','root','tooling','cpu-release']:p.add_argument('--'+name,required=True,type=Path)
    a=p.parse_args();result=activate(a.project,a.root,a.tooling,a.cpu_release)
    print(json.dumps({k:result[k] for k in ['default_backend','build_id','runtime_tested','navigation_started','backup']},indent=2))


if __name__=='__main__':main()
