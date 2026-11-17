#!/usr/bin/env python3
"""Short Nsight CUDA proof for the sealed standalone benchmark, never ROS."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

if __package__:
    from . import native, run
else:
    import native
    import run


def owned_process_group(pid, owner):
    try:
        if os.getpgid(pid) == owner:
            return True
        # Nsight launches the target in a different process group. Check the
        # actual parent chain instead of treating our target as interference.
        current = pid
        for _ in range(64):
            if current == owner:
                return True
            if current <= 1:
                break
            stat = Path('/proc/{}/stat'.format(current)).read_text()
            current = int(stat.rsplit(')',1)[1].split()[1])
        return False
    except (ProcessLookupError, FileNotFoundError):
        return True  # It has already exited; there is no interference to stop.


def capture(args, log, env, timeout=120):
    with Path(log).open('xb') as stream:
        child = subprocess.Popen(args, stdout=stream, stderr=stream, env=env,
                                 start_new_session=True)
        started = time.monotonic()
        owned_groups = {child.pid}
        try:
            while child.poll() is None:
                busy = []
                for p in native.navigation_processes():
                    if owned_process_group(p['pid'], child.pid):
                        try:
                            owned_groups.add(os.getpgid(p['pid']))
                        except ProcessLookupError:
                            pass
                    else:
                        busy.append(p)
                if busy:
                    raise ValueError('business/another experiment started; stopping only owned trace')
                if time.monotonic()-started > timeout:
                    raise TimeoutError('trace time limit exceeded')
                time.sleep(.5)
        finally:
            if child.poll() is None:
                for group in owned_groups:
                    try:
                        os.killpg(group, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    for group in owned_groups:
                        try:
                            os.killpg(group, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    child.wait()
        if child.returncode:
            raise ValueError('Nsight failed; inspect '+str(log))


def ndt_kernels(text):
    rows = list(csv.reader(text.splitlines()))
    result = []
    header = None
    for row in rows:
        if 'Name' in row and 'Instances' in row:
            header = row
            continue
        if header and len(row) == len(header):
            r = dict(zip(header, row))
            if 'ndt' in r['Name'].lower() and int(r['Instances']) > 0:
                result.append(r)
    return result


def trace(root, dataset, output, backend, confirmed=False):
    native.idle_required(confirmed)
    data = native.check(root)
    root, output = Path(root).resolve(), Path(output).resolve()
    if root/'traces' not in output.parents or output.exists():
        raise ValueError('choose a new output directory under the private root/traces')
    if backend not in ('NDT_CUDA_P2D','NDT_CUDA_D2D'):
        raise ValueError('select an explicit CUDA backend')
    meta = run.verify_dataset(dataset)
    if not 1 <= meta['cases'] <= 32:
        raise ValueError('Kernel trace is for a bounded smoke dataset (1..32 cases)')
    nsys = shutil.which('nsys')
    if not nsys:
        raise ValueError('Nsight Systems is not installed; no Kernel proof collected')
    output.mkdir(parents=True)
    dataset = Path(dataset).resolve()
    identity = hashlib.sha256(json.dumps(meta, sort_keys=True).encode()).hexdigest()
    report = output/'cuda_trace'
    args = [nsys,'profile','--trace=cuda','--sample=none','--cpuctxsw=none',
            '--force-overwrite=false','--output='+str(report), data['binary'], backend,
            str(dataset/'map.pcd'),str(dataset/'cases.tsv'),str(output/'samples.jsonl'),
            str(meta['resolution']),str(meta['map_leaf']),'1','1',identity]
    env = dict(os.environ); env.update(data['environment'])
    (output/'request.json').write_text(json.dumps(dict(
        command=args, backend=backend, dataset_id=identity, performance_run=False,
        ros_publications=False), indent=2))
    try:
        capture(args, output/'profile.log', env)
        rep = report.with_suffix('.nsys-rep')
        stats = subprocess.run([nsys,'stats','--report','cuda_gpu_kern_sum',
                                '--format','csv',str(rep)],env=env,capture_output=True,
                               text=True,timeout=60,check=True)
        (output/'kernel_summary.csv').write_text(stats.stdout)
        (output/'kernel_stats_stderr.log').write_text(stats.stderr)
        kernels = ndt_kernels(stats.stdout)
        if not kernels:
            raise ValueError('trace has no executed NDT Kernel events')
        api = subprocess.run([nsys,'stats','--report','cuda_api_sum','--format','csv',str(rep)],
                             env=env,capture_output=True,text=True,timeout=60,check=True)
        (output/'cuda_api_summary.csv').write_text(api.stdout)
        evidence = dict(backend=backend, cuda_kernel_executed=True, ndt_kernels=kernels,
                        nsys_report=str(rep), report_sha256=native.sha(rep),
                        performance_run=False, ros_publications=False)
        (output/'evidence.json').write_text(json.dumps(evidence,indent=2))
        return evidence
    finally:
        native.verify_hashes(data['official'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('root','dataset','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--backend',choices=['NDT_CUDA_P2D','NDT_CUDA_D2D'],required=True)
    p.add_argument('--operator-confirmed-disarmed',action='store_true')
    a = p.parse_args()
    try:
        print(json.dumps(trace(a.root,a.dataset,a.output,a.backend,a.operator_confirmed_disarmed),indent=2))
        return 0
    except (ValueError,OSError,TimeoutError,subprocess.SubprocessError) as e:
        p.exit(2,str(e)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
