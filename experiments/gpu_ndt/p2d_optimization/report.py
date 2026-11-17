#!/usr/bin/env python3
"""Summarize NORMAL A/B/C timings; keep Nsight aggregate counts separate."""
import argparse
import csv
import json
import math
from pathlib import Path

from .. import analyze


def api_totals(path):
    """Nsight cuda_api_sum totals, including startup/warmup, NOT per scan."""
    result={};header=None
    for row in csv.reader(Path(path).read_text().splitlines()):
        if 'Name' in row and any('Total Time' in cell for cell in row):header=row;continue
        if header is None or len(row)!=len(header):continue
        item=dict(zip(header,row));name=item['Name']
        if not any(s in name for s in ['cudaMalloc','cudaFree','Synchronize']):continue
        times=[v for k,v in item.items() if k.startswith('Total Time')]
        counts=[v for k,v in item.items() if k in ['Num Calls','Instances']]
        if len(times)!=1 or len(counts)!=1:raise ValueError('unsupported Nsight API columns')
        result[name]=dict(calls=int(counts[0].replace(',','')),total_time_ms=float(times[0].replace(',',''))/1e6)
    return dict(scope='whole trace including initialization and warmup; never divided into per-frame cost',apis=result)


def summarize(a_run,b_run,c_run):
    results={};settings=None;samples={};frequency_means=[]
    for label,directory in [('A',a_run),('B',b_run),('C',c_run)]:
        directory=Path(directory)
        exits=json.loads((directory/'exit_codes.json').read_text())
        request=json.loads((directory/'request.json').read_text())
        if request.get('scope')!='registration_only':raise ValueError('invalid benchmark scope')
        for backend in (['NDT_OMP'] if label=='A' and 'NDT_OMP' in exits else [])+['NDT_CUDA_P2D']:
            key='CPU' if backend=='NDT_OMP' else label
            meta,rows,report=analyze.read_result(directory/(backend+'.jsonl'))
            if not report['complete'] or exits.get(backend)!=0:raise ValueError('incomplete normal benchmark: '+key)
            if meta.get('p2d_verification_run') is not False:
                raise ValueError('verification/debug timings cannot be used as performance results')
            if backend=='NDT_CUDA_P2D' and meta.get('p2d_variant')!=label:raise ValueError('variant identity mismatch')
            fields=('dataset_id','resolution','map_leaf','search','expected_cases','warmup','repeats')
            current=tuple(meta[k] for k in fields)
            if settings is not None and current!=settings:raise ValueError('different A/B/C input/parameters')
            settings=current
            for field in ['lm_linearizations','lm_error_evaluations','candidate_error_ms','align_process_cpu_ms']:
                values=[r.get(field) for r in rows]
                if any(not isinstance(v,(float,int)) or not math.isfinite(v) or v<0 for v in values):
                    raise ValueError('missing/invalid observation counter: '+field)
                report['metrics'][field]=analyze.statistics(values)
            resource=directory/(backend+'.resources.json')
            stats=json.loads(resource.read_text()) if resource.is_file() else {}
            frequencies=[float(r['gpu_frequency_hz'])/1e6 for r in stats.get('resource_samples',[]) if r.get('gpu_frequency_hz') is not None]
            report['gpu_frequency_mhz_coarse']=analyze.statistics(frequencies)
            endpoints=[r[k]/1e6 for r in rows for k in ['gpu_frequency_before_hz','gpu_frequency_after_hz']
                       if isinstance(r.get(k),(float,int)) and r[k]>0]
            report['gpu_frequency_mhz_align_endpoints']=analyze.statistics(endpoints)
            if key!='CPU' and frequencies:frequency_means.append(sum(frequencies)/len(frequencies))
            tegra=directory/(backend+'.tegrastats.log')
            if tegra.is_file():report['tegrastats']=analyze.tegra_metrics(tegra)
            results[key]=report;samples[key]={(r['case_id'],r['trial'],r['source_stamp_ns']):r for r in rows}
    baseline=samples['A'];differences={}
    for label,rows in samples.items():
        if set(rows)!=set(baseline):raise ValueError('unpaired A/B/C cases')
        positions=[];counts_same=True;quality_same=True
        for key,left in baseline.items():
            right=rows[key]
            if analyze.valid_pose(left['pose']) and analyze.valid_pose(right['pose']):
                positions.append(math.sqrt(sum((left['pose'][i]-right['pose'][i])**2 for i in [3,7,11])))
            counts_same &= all(left[k]==right[k] for k in ['lm_linearizations','lm_error_evaluations'])
            quality_same &= left['spatial_quality_pass']==right['spatial_quality_pass'] and left['converged']==right['converged']
        differences[label]=dict(position_difference_from_A_m=analyze.statistics(positions),
                                 lm_counts_equal_to_A=counts_same if label!='CPU' else None,
                                 quality_and_convergence_equal_to_A=quality_same)
    matched=(len(frequency_means)==3 and min(frequency_means)>0 and max(frequency_means)/min(frequency_means)<=1.05)
    return dict(scope='registration_only',performance_instrumentation='counters/timers only; no dual evaluation or LM debug',
                dataset_id=settings[0],backends=results,paired_with_original_p2d=differences,
                coarse_mean_gpu_frequencies_within_5_percent=matched,
                warnings=['Coarse clocks do not prove identical clocks during each align; inspect raw frequency/temperature/load traces.',
                          'Repeated alignments are not independent scans; use unique source_stamp_ns for scan counts.',
                          'CPU NDT_OMP and P2D use different optimizers; CPU/GPU LM counts are not equivalent.',
                          'Process CPU includes standalone geometry checks; explicit_sync_wait_ms is only a subset of CUDA waiting.'],
                full_localization_and_voa='未验证')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['a-run','b-run','c-run','output']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    try:
        result=summarize(args.a_run,args.b_run,args.c_run)
        with args.output.open('x') as f:json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    except (ValueError,OSError,KeyError) as error:parser.exit(2,str(error)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    main()
