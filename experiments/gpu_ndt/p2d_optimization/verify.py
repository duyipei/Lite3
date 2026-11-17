"""Conservative native A/B/C gate. Not a substitute for Jetson integration."""
import json
import math
from pathlib import Path
import re

from ..dataset import sha

BACKEND='NDT_CUDA_P2D'


def lm_trace(path):
    current=None;traces={}
    for line in Path(path).read_text().splitlines():
        if line.startswith('LITE3_CASE '):
            fields=line.split()
            if len(fields)!=3:raise ValueError('invalid case marker')
            current=(fields[1],int(fields[2]))
            if current in traces:raise ValueError('repeated trace case')
            traces[current]=[]
        elif current is not None:
            fields=line.split()
            # Upstream's built-in boost::format LM diagnostic, unchanged.
            if len(fields) in (6,7) and re.fullmatch(r'\d+',fields[0]):
                values=[float(s) for s in fields[1:6]]
                if not all(math.isfinite(v) for v in values):raise ValueError('nonfinite LM diagnostic')
                # A positive diagnostic "x" is not used as the accept flag:
                # the actual LM rejection condition is rho < 0, including 0.
                traces[current].append((int(fields[0]),values[2]<0))
    return traces


def read_run(directory):
    directory=Path(directory).resolve()
    request=json.loads((directory/'request.json').read_text())
    if request.get('scope')!='registration_only' or request.get('backends')!=[BACKEND]:
        raise ValueError('need a P2D-only registration verification run')
    if json.loads((directory/'exit_codes.json').read_text())!={BACKEND:0}:
        raise ValueError('native verification run failed')
    result=directory/(BACKEND+'.jsonl')
    rows=[json.loads(line) for line in result.read_text().splitlines()]
    if len([r for r in rows if r.get('kind')=='metadata'])!=1:raise ValueError('invalid metadata count')
    meta=next(r for r in rows if r.get('kind')=='metadata')
    if meta.get('p2d_verification_run') is not True:raise ValueError('timing/Mock run cannot pass native verification gate')
    if not any(r.get('kind')=='summary' and r.get('finished') is True for r in rows):
        raise ValueError('incomplete verification run')
    if not any(r.get('kind')=='device' and r.get('compute_major')==7 and r.get('compute_minor')==2 for r in rows):
        raise ValueError('no native sm_72 device evidence')
    if any(r.get('kind')=='error' for r in rows):raise ValueError('native verification error')
    samples={}
    for r in rows:
        if r.get('kind')!='sample':continue
        key=(r['case_id'],r['trial'])
        if key in samples:raise ValueError('repeated sample')
        if not r['pose'] or len(r['pose'])!=16 or not all(math.isfinite(v) for v in r['pose']):
            raise ValueError('invalid native pose')
        for field in ['lm_linearizations','lm_error_evaluations']:
            if not isinstance(r.get(field),int) or r[field]<=0:raise ValueError('missing LM counter')
        for field in ['scalar_reference_max_abs','scalar_reference_max_relative','quality_mse','overlap']:
            if not isinstance(r.get(field),(float,int)) or not math.isfinite(r[field]):
                raise ValueError('missing/nonfinite native check: '+field)
        # Threshold here is only for scalar consistency, never localization.
        if r['scalar_reference_max_relative']>2.2e-5:raise ValueError('scalar/reference discrepancy')
        samples[key]=r
    expected=meta['expected_cases']*(meta['warmup']+meta['repeats'])
    if len(samples)!=expected or not samples:raise ValueError('missing verification samples')
    tracepath=directory/(BACKEND+'.stderr.log')
    traces=lm_trace(tracepath)
    if set(traces)!=set(samples):raise ValueError('missing case LM trace')
    for key,r in samples.items():
        if len(traces[key])!=r['lm_error_evaluations']:
            raise ValueError('LM trace/evaluation count mismatch')
        if sum(i==0 for i,_ in traces[key])!=r['lm_linearizations']:
            raise ValueError('LM trace/linearization count mismatch')
    return request,meta,samples,traces,{str(p):sha(p) for p in
            [directory/'request.json',directory/'exit_codes.json',result,tracepath]}


def compare(a_run,other_run):
    ar,am,a,at,af=read_run(a_run)
    br,bm,b,bt,bf=read_run(other_run)
    if am['p2d_variant']!='A' or bm['p2d_variant'] not in ['B','C']:
        raise ValueError('compare original A with B or C')
    for name in ['dataset_id','resolution','map_leaf','search','warmup','repeats','max_iterations','transformation_epsilon']:
        if am.get(name)!=bm.get(name):raise ValueError('incomparable verification input/parameters: '+name)
    if ar['dataset_id']!=br['dataset_id'] or set(a)!=set(b):raise ValueError('dataset/sample mismatch')
    max_position=0.;max_rotation=0.
    for key,left in a.items():
        right=b[key]
        if at[key]!=bt[key]:raise ValueError('LM rho-sign/inner-iteration sequence changed: '+str(key))
        for field in ['lm_linearizations','lm_error_evaluations','converged','spatial_quality_pass']:
            if left[field]!=right[field]:raise ValueError(field+' changed: '+str(key))
        p,q=left['pose'],right['pose']
        position=math.sqrt(sum((p[i]-q[i])**2 for i in [3,7,11]))
        # trace(Ra^T Rb) is the rotation elementwise inner product.
        # Matrix4f orthogonality error can exceed acos(1-1e-8) even for
        # identical matrices; compare rotations by Frobenius norm instead.
        rotation=math.sqrt(sum((p[r*4+c]-q[r*4+c])**2 for r in range(3) for c in range(3)))
        max_position=max(max_position,position);max_rotation=max(max_rotation,rotation)
        if position>1e-4 or rotation>1e-4:raise ValueError('final pose changed beyond verification tolerance: '+str(key))
        if abs(left['quality_mse']-right['quality_mse'])>1e-5 or abs(left['overlap']-right['overlap'])>1e-5:
            raise ValueError('geometric quality changed: '+str(key))
    return dict(scope='native_P2D_scalar_LM_pose_verification',passed=True,variant=bm['p2d_variant'],
                dataset_id=ar['dataset_id'],samples=len(a),max_position_difference_m=max_position,
                max_rotation_matrix_difference=max_rotation,files={**af,**bf},
                kernel_trace_separately_required=True,performance_measured=False)


def require_b_receipt(path):
    if path is None:raise ValueError('native C requires a passed A/B native verification receipt first')
    receipt=json.loads(Path(path).read_text())
    if (receipt.get('scope')!='native_P2D_scalar_LM_pose_verification' or receipt.get('passed') is not True
            or receipt.get('variant')!='B' or receipt.get('cuda_error_kernel_executed') is not True):
        raise ValueError('invalid A/B native verification receipt')
    if not receipt.get('files'):raise ValueError('verification receipt has no evidence')
    for file,digest in receipt['files'].items():
        if sha(file)!=digest:raise ValueError('native verification evidence changed')
    return receipt
