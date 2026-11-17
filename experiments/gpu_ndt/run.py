#!/usr/bin/env python3
"""Run sequential, offline registration benchmarks; contains no ROS operations."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import shutil
from collections import deque
from contextlib import contextmanager

if __package__:
    from .dataset import sha
    from .prepare import BACKENDS
else:
    from dataset import sha
    from prepare import BACKENDS


def verify_dataset(directory):
    directory = Path(directory).resolve()
    meta = json.loads((directory/'dataset.json').read_text())
    if meta.get('scope') != 'registration_only' or meta.get('schema_version') != 1:
        raise ValueError('unsupported dataset')
    for file, key in [('map.pcd','map_sha256'), ('cases.tsv','cases_sha256')]:
        if sha(directory/file) != meta[key]:
            raise ValueError('dataset changed: '+file)
    for frame in meta['frames']:
        p = directory/('scan_%04d.pcd' % frame['index'])
        if sha(p) != frame['source_pcd_sha256']:
            raise ValueError('scan changed: '+str(p))
    return meta


def resource_snapshot():
    def read(path):
        try:
            return Path(path).read_text().strip()
        except OSError:
            return None
    cpu = read('/proc/stat')
    return dict(monotonic_ns=time.monotonic_ns(), system_cpu_stat=cpu.splitlines()[0] if cpu else None,
                loadavg=read('/proc/loadavg'), meminfo=read('/proc/meminfo'),
                gpu_load_per_mille=read('/sys/devices/17000000.gv11b/load'),
                gpu_frequency_hz=read('/sys/devices/17000000.gv11b/devfreq/17000000.gv11b/cur_freq'),
                cpu_temperature_millideg=read('/sys/class/thermal/thermal_zone0/temp'),
                gpu_temperature_millideg=read('/sys/class/thermal/thermal_zone1/temp'))


@contextmanager
def tegra_capture(output):
    """One owned, read-only sampler; never use the global --stop command."""
    binary=shutil.which('tegrastats')
    status=dict(available=bool(binary),interval_ms=1000,scope='background system load, not process-only',
                start_monotonic_ns=time.monotonic_ns())
    if binary is None:
        yield status
        return
    with Path(output).open('xb') as log:
        child=subprocess.Popen([binary,'--interval','1000'],stdout=log,stderr=log)
        status['pid']=child.pid
        try:
            yield status
        finally:
            status['running_at_stop']=child.poll() is None
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill();child.wait()
            status['exit_code']=child.returncode
            status['end_monotonic_ns']=time.monotonic_ns()


def run(binary, dataset, output, backends=BACKENDS, warmup=3, repeats=5, timeout=600,
        interference_check=None, env_override=None):
    binary, dataset, output = Path(binary).resolve(), Path(dataset).resolve(), Path(output).resolve()
    if binary.name != 'lite3_ndt_benchmark' or not binary.is_file():
        raise ValueError('select the standalone lite3_ndt_benchmark executable')
    if output.exists():
        raise ValueError('output already exists')
    if not backends or any(b not in BACKENDS for b in backends):
        raise ValueError('unknown backend')
    if len(set(backends)) != len(backends):
        raise ValueError('duplicate backend')
    if not 1 <= warmup <= 20 or not 1 <= repeats <= 100 or not 1 <= timeout <= 3600:
        raise ValueError('invalid benchmark bounds')
    meta = verify_dataset(dataset)
    output.mkdir(parents=True)
    identity = hashlib.sha256(json.dumps(meta, sort_keys=True).encode()).hexdigest()
    request = dict(schema_version=1, scope='registration_only', dataset_id=identity,
                   dataset=str(dataset), binary=str(binary), binary_sha256=sha(binary),
                   backends=list(backends), warmup=warmup, repeats=repeats,
                   dataset_metadata=meta, end_to_end_measured=False)
    (output/'request.json').write_text(json.dumps(request, indent=2)+'\n')
    exits = {}
    env = dict(os.environ)
    if env_override:env.update(env_override)
    env.update(OMP_NUM_THREADS='1', OMP_WAIT_POLICY='PASSIVE', GOMP_SPINCOUNT='0')
    for backend in backends:
        if interference_check:interference_check(None)
        if verify_dataset(dataset) != meta:
            raise ValueError('dataset changed between backends')
        args = [str(binary), backend, str(dataset/'map.pcd'), str(dataset/'cases.tsv'),
                str(output/(backend+'.jsonl')), str(meta['resolution']), str(meta['map_leaf']),
                str(warmup), str(repeats), identity]
        before = resource_snapshot()
        begin = time.monotonic()
        samples, dropped = deque(maxlen=4096), 0
        with tegra_capture(output/(backend+'.tegrastats.log')) as tegra, (output/(backend+'.stderr.log')).open('xb') as stderr:
            child = subprocess.Popen(args, env=env, stdout=stderr, stderr=stderr)
            try:
                while child.poll() is None:
                    if interference_check:interference_check(child.pid)
                    if len(samples) == samples.maxlen:
                        dropped += 1
                    samples.append(resource_snapshot())
                    if time.monotonic()-begin > timeout:
                        child.kill()
                        child.wait()
                        exits[backend] = 'timeout'
                        break
                    time.sleep(.5)
                else:
                    exits[backend] = child.returncode
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait()
        record = dict(before=before, after=resource_snapshot(), elapsed_s=time.monotonic()-begin,
                      backend=backend, returncode=exits[backend], resource_samples=list(samples),
                      dropped_resource_samples=dropped, resource_sampling_period_s=.5,
                      tegrastats=tegra)
        (output/(backend+'.resources.json')).write_text(json.dumps(record, indent=2)+'\n')
    if verify_dataset(dataset) != meta:
        raise ValueError('dataset changed during final backend')
    (output/'exit_codes.json').write_text(json.dumps(exits, indent=2)+'\n')
    return exits


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--backend', action='append', choices=BACKENDS)
    p.add_argument('--warmup', type=int, default=3)
    p.add_argument('--repeats', type=int, default=5)
    p.add_argument('--timeout', type=int, default=600)
    a = p.parse_args()
    try:
        exits = run(a.binary, a.dataset, a.output, a.backend or BACKENDS,
                    a.warmup, a.repeats, a.timeout)
    except (ValueError, OSError) as e:
        p.exit(2, str(e)+'\n')
    print(json.dumps(exits, indent=2))
    return int(any(value != 0 for value in exits.values()))


if __name__ == '__main__':
    raise SystemExit(main())
