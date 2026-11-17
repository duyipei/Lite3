#!/usr/bin/env python3
"""Compare registration results; never claim TF/UKF/VOA acceptance from them."""
import argparse
import json
import math
import re
from pathlib import Path
import numpy as np

if __package__:
    from .prepare import BACKENDS
else:
    from prepare import BACKENDS

METRICS = ('prepare_ms', 'source_set_sync_ms', 'voxel_build_sync_ms',
           'optimizer_sync_ms', 'align_sync_ms', 'quality_ms',
           'registration_pipeline_ms', 'process_cpu_ms', 'rss_bytes',
           'scan_load_ms','explicit_sync_wait_ms','explicit_sync_calls')
REQUIRED_METRICS = set(METRICS)-{'voxel_build_sync_ms', 'optimizer_sync_ms',
                               'scan_load_ms','explicit_sync_wait_ms','explicit_sync_calls'}


def valid_pose(value):
    try:
        pose = np.asarray(value, dtype=float).reshape(4,4)
    except (TypeError, ValueError):
        return False
    rotation = pose[:3,:3]
    return (np.isfinite(pose).all() and np.allclose(pose[3], [0,0,0,1], atol=1e-4)
            and np.allclose(rotation.T@rotation, np.eye(3), atol=1e-3)
            and abs(np.linalg.det(rotation)-1) < 1e-3)


def tegra_metrics(path):
    fields={'ram_used_mb':r'RAM (\d+)/\d+MB',
            'emc_busy_percent':r'EMC_FREQ (\d+)%',
            'emc_frequency_mhz':r'EMC_FREQ \d+%@(\d+)',
            'gpu_busy_percent':r'GR3D_FREQ (\d+)%',
            'vdd_in_mw':r'\bVDD_IN (\d+)mW'}
    values={key:[] for key in fields};lines=0
    with Path(path).open() as f:
        for row in f:
            lines+=1
            if lines>4096:continue
            for key,pattern in fields.items():
                match=re.search(pattern,row)
                if match:values[key].append(float(match[1]))
    return dict(metrics={key:statistics(v) for key,v in values.items()},
                lines=lines,truncated=lines>4096,scope='coarse whole-system snapshots; not process-only')


def statistics(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return dict(n=0, mean=None, p50=None, p95=None, p99=None, max=None)
    return dict(n=len(a), mean=float(a.mean()), p50=float(np.percentile(a,50)),
                p95=float(np.percentile(a,95)), p99=float(np.percentile(a,99)), max=float(a.max()))


def read_result(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    metadata = [r for r in rows if r.get('kind') == 'metadata']
    if len(metadata) != 1:
        raise ValueError('exactly one metadata row required: '+str(path))
    meta = metadata[0]
    if meta.get('scope') != 'registration_only' or meta.get('backend') not in BACKENDS:
        raise ValueError('unsupported result scope/backend')
    if any(type(meta.get(k)) is not int or meta[k] <= 0 for k in ('expected_cases','repeats','warmup')):
        raise ValueError('invalid benchmark sample bounds')
    samples = [r for r in rows if r.get('kind') == 'sample' and r.get('phase') == 'steady']
    keys = [(r['case_id'],r['trial'],r['source_stamp_ns']) for r in samples]
    errors = [r for r in rows if r.get('kind') == 'error']
    finished = bool(rows and rows[-1].get('kind') == 'summary' and rows[-1].get('finished') is True)
    complete = (finished and not errors and len(keys) == len(set(keys))
                and len(samples) == meta['expected_cases']*meta['repeats'])
    bad_values = []
    cases = {}
    for r in samples:
        cases.setdefault(r['case_id'], []).append((r['trial'], r['source_stamp_ns']))
        if type(r['source_stamp_ns']) is not int or r['source_stamp_ns'] < 0:
            bad_values.append((r['case_id'],'invalid source timestamp'))
        for k in METRICS:
            v = r.get(k)
            if ((k in REQUIRED_METRICS and v is None) or
                    (v is not None and (type(v) not in (int,float) or not math.isfinite(v) or v<0))):
                bad_values.append((r['case_id'],k))
        if r.get('spatial_quality_pass') and not valid_pose(r.get('pose')):
            bad_values.append((r['case_id'],'invalid accepted pose'))
        for k in ('quality_mse','overlap'):
            v=r.get(k)
            if v is not None and (type(v) not in (int,float) or not math.isfinite(v) or v<0 or (k=='overlap' and v>1)):
                bad_values.append((r['case_id'],k))
    if len(cases) != meta['expected_cases']:
        complete = False
    for name, entries in cases.items():
        if ({trial for trial,stamp in entries} != set(range(meta['repeats'])) or
                len({stamp for trial,stamp in entries}) != 1):
            bad_values.append((name,'invalid trial sequence/source timestamp'))
    if bad_values:
        complete = False
    failures=[];failed_count=0
    for r in samples:
        reasons=[]
        if not r.get('converged'):reasons.append('not_converged')
        if not r.get('spatial_quality_pass'):reasons.append('spatial_quality_rejected')
        if not valid_pose(r.get('pose')):reasons.append('invalid_pose')
        if reasons:
            failed_count+=1
            if len(failures)<256:
                failures.append(dict(case_id=r['case_id'],trial=r['trial'],source_stamp_ns=r['source_stamp_ns'],
                                     reasons=reasons,quality_mse=r.get('quality_mse') if type(r.get('quality_mse')) in (int,float) and math.isfinite(r['quality_mse']) else None,
                                     overlap=r.get('overlap') if type(r.get('overlap')) in (int,float) and math.isfinite(r['overlap']) else None))
    report = dict(complete=complete, finished=finished, errors=errors, invalid_values=bad_values,
                  samples=len(samples), converged=sum(bool(r.get('converged')) for r in samples),
                  spatial_quality_pass=sum(bool(r.get('spatial_quality_pass')) for r in samples),
                  failed_samples=failed_count,failure_examples=failures,
                  failure_examples_omitted=max(0,failed_count-len(failures)),
                  metrics={k:statistics([r[k] for r in samples if r.get(k) is not None
                                        and isinstance(r[k],(int,float)) and math.isfinite(r[k])])
                           for k in METRICS}, metadata=meta)
    return meta, samples, report


def analyze(directory):
    directory = Path(directory)
    request = json.loads((directory/'request.json').read_text())
    if request.get('scope') != 'registration_only' or request.get('end_to_end_measured') is not False:
        raise ValueError('unsupported request')
    reports, samples, warnings = {}, {}, []
    settings = None
    exit_file = directory/'exit_codes.json'
    exits = json.loads(exit_file.read_text()) if exit_file.exists() else {}
    for backend in request['backends']:
        file = directory/(backend+'.jsonl')
        if not file.exists():
            warnings.append('missing backend: '+backend)
            continue
        try:
            meta, rows, report = read_result(file)
        except (ValueError, KeyError, TypeError) as error:
            warnings.append(backend+': '+str(error))
            continue
        if meta['backend'] != backend:
            raise ValueError('backend identity mismatch')
        if meta.get('dataset_id') != request.get('dataset_id'):
            raise ValueError('CPU/GPU dataset identity mismatch')
        current = tuple(meta[k] for k in ('resolution','map_leaf','search','expected_cases','warmup','repeats'))
        if settings is not None and current != settings:
            raise ValueError('CPU/GPU used different benchmark settings')
        settings = current
        if exits.get(backend) != 0:
            report['complete'] = False
            warnings.append('backend did not exit successfully: '+backend)
        resource_file = directory/(backend+'.resources.json')
        if resource_file.exists():
            resource = json.loads(resource_file.read_text())
            gpu = [int(r['gpu_load_per_mille'])/10 for r in resource.get('resource_samples',[])
                   if r.get('gpu_load_per_mille') is not None]
            report['gpu_utilization_percent_coarse_samples'] = statistics(gpu)
            try:
                a = [int(v) for v in resource['before']['system_cpu_stat'].split()[1:9]]
                b = [int(v) for v in resource['after']['system_cpu_stat'].split()[1:9]]
                delta = [y-x for x,y in zip(a,b)]
                report['system_cpu_busy_percent'] = 100*(1-(delta[3]+delta[4])/sum(delta)) if sum(delta)>0 else None
            except (KeyError,TypeError,ValueError,IndexError):
                report['system_cpu_busy_percent'] = None
            if resource.get('dropped_resource_samples'):
                report['complete'] = False
                warnings.append('resource sampling incomplete: '+backend)
        tegra_file=directory/(backend+'.tegrastats.log')
        if tegra_file.exists():report['tegrastats']=tegra_metrics(tegra_file)
        reports[backend], samples[backend] = report, rows
    pairs = {}
    cpu = {(r['case_id'],r['trial'],r['source_stamp_ns']):r for r in samples.get('NDT_OMP',[])}
    for backend, rows in samples.items():
        if backend == 'NDT_OMP':
            continue
        position, yaw, missing = [], [], 0
        finite_position,finite_yaw=[],[]
        for r in rows:
            c = cpu.get((r['case_id'],r['trial'],r['source_stamp_ns']))
            if c is None:
                missing += 1
                continue
            if not valid_pose(c.get('pose')) or not valid_pose(r.get('pose')):
                continue
            a, b = np.asarray(c['pose'],dtype=float).reshape(4,4), np.asarray(r['pose'],dtype=float).reshape(4,4)
            delta_position=float(np.linalg.norm(a[:3,3]-b[:3,3]))
            d = math.atan2(a[1,0],a[0,0])-math.atan2(b[1,0],b[0,0])
            delta_yaw=abs(math.atan2(math.sin(d),math.cos(d)))
            finite_position.append(delta_position);finite_yaw.append(delta_yaw)
            if c.get('spatial_quality_pass') and r.get('spatial_quality_pass'):
                position.append(delta_position);yaw.append(delta_yaw)
        pairs[backend] = dict(position_difference_m=statistics(position),
                              yaw_difference_rad=statistics(yaw), unmatched=missing,
                              all_finite_position_difference_m=statistics(finite_position),
                              all_finite_yaw_difference_rad=statistics(finite_yaw),
                              absolute_accuracy_proven=False)
        if missing or {(r['case_id'],r['trial'],r['source_stamp_ns']) for r in rows} != set(cpu):
            warnings.append('unmatched CPU/GPU samples: '+backend)
    complete = (not warnings and len(reports)==3 and all(r['complete'] for r in reports.values()))
    return dict(scope='registration_only', status='complete' if complete else 'partial',
                dataset_id=request.get('dataset_id'), warnings=warnings, backends=reports,
                all_spatial_checks_pass=bool(reports) and all(r['samples']>0 and r['spatial_quality_pass']==r['samples'] for r in reports.values()),
                paired_differences=pairs, full_localization_latency='未验证',
                ukf_and_fusion='未验证', tf_and_anchor_continuity='未验证',
                voa_improvement='未验证', gpu_kernel_execution='需 Nsight/CUDA trace 验证；同步耗时不能独立证明 Kernel 已执行')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    try:
        result=analyze(a.directory)
    except (ValueError,KeyError,OSError) as e:
        p.exit(2,str(e)+'\n')
    text=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if a.output:
        with a.output.open('x') as f:f.write(text)
    print(text,end='')
    return 0 if result['status']=='complete' else 1


if __name__=='__main__':
    raise SystemExit(main())
