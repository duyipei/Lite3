#!/usr/bin/env python3
"""Private P2D A/B/C build/run/verification. No ROS publishers or clock writes."""
import argparse
import hashlib
import json
from pathlib import Path

from .. import native, run
from . import benchmark_source, patch, verify


def checked_variant(source):
    source=Path(source).resolve()
    manifest=json.loads((source/'lite3_p2d_variant.json').read_text())
    if manifest.get('scope')!='independent_GPU_P2D_only' or manifest.get('variant') not in ['A','B','C']:
        raise ValueError('not a prepared P2D variant source')
    for name,digest in manifest['generated_hashes'].items():
        if patch.sha((source/name).read_bytes())!=digest:raise ValueError('prepared variant source changed: '+name)
    if patch.inventory(source)!=manifest.get('generated_inventory'):
        raise ValueError('complete prepared source inventory changed')
    # Never try to build the deliberately incomplete offline/reference bundle.
    if not (source/'CMakeLists.txt').is_file() or not (source/'src/fast_gicp/ndt/ndt_cuda.cpp').is_file():
        raise ValueError('complete installed fast_gicp source is required for native build')
    return manifest


def build(root,source,ndt_prefix,official_manifest,confirmed=False,b_receipt=None):
    native.idle_required(confirmed)
    source=Path(source).resolve();root=Path(root).resolve()
    manifest=checked_variant(source)
    if manifest['variant']=='C':
        receipt=verify.require_b_receipt(b_receipt)
        if receipt.get('baseline_inventory')!=manifest['baseline_inventory']:
            raise ValueError('C must use the same complete baseline source as verified A/B')
    native.independent(root,[source,ndt_prefix,Path(official_manifest).resolve().parent,native.HERE])
    bench=root.with_name(root.name+'_benchmark_source')
    native.independent(bench,[source,ndt_prefix,Path(official_manifest).resolve().parent,native.HERE])
    benchmark_source.prepare_source(bench,manifest['variant'])
    # Rebuild BOTH libraries, headers, and the benchmark for every variant.
    # Never replace a single ABI-incompatible .so in an existing prefix.
    return native.build(root,source,ndt_prefix,official_manifest,True,benchmark_source=bench)


def run_variant(root,dataset,output,mode,confirmed=False,receipt=None,warmup=2,repeats=3,cpu=False):
    native.idle_required(confirmed)
    if mode not in ['verify','measure']:raise ValueError('unknown experiment mode')
    root=Path(root).resolve();output=Path(output).resolve()
    data=native.check(root)
    manifest=json.loads((root/'deps/src/fast_gicp/lite3_p2d_variant.json').read_text())
    variant=manifest['variant']
    if root not in output.parents:raise ValueError('result must remain inside its private runtime root')
    meta=run.verify_dataset(dataset)
    identity=hashlib.sha256(json.dumps(meta,sort_keys=True).encode()).hexdigest()
    if mode=='measure' and variant!='A':
        if receipt is None:raise ValueError('numerical A/'+variant+' verification must pass before timing')
        r=json.loads(Path(receipt).read_text())
        if (r.get('variant')!=variant or r.get('passed') is not True or r.get('dataset_id')!=identity
                or r.get('cuda_error_kernel_executed') is not True):
            raise ValueError('timing receipt variant/dataset mismatch')
        if r.get('scope')!='native_P2D_scalar_LM_pose_verification' or not r.get('files'):
            raise ValueError('invalid numerical verification receipt')
        native.verify_hashes(r['files'])
        if str(root/'native_runtime.json') not in r['files']:
            raise ValueError('verification receipt belongs to another native runtime')
    def guard(owned_pid):
        busy=[p for p in native.navigation_processes() if p['pid']!=owned_pid]
        if busy:raise ValueError('navigation/another benchmark active; stopping only this experiment')
    env=dict(data['environment']);env['LITE3_P2D_VERIFY']='1' if mode=='verify' else '0'
    backends=(["NDT_OMP"] if cpu and mode=='measure' else [])+[verify.BACKEND]
    try:
        exits=run.run(data['binary'],dataset,output,backends,warmup,repeats,interference_check=guard,env_override=env)
    finally:
        native.verify_hashes(data['files']);native.verify_hashes(data['official'])
    if any(v!=0 for v in exits.values()):raise ValueError('native run failed; inspect result logs')
    (output/'native_evidence.json').write_text(json.dumps(dict(
        mode=mode,variant=variant,runtime_manifest=str(root/'native_runtime.json'),
        runtime_manifest_sha256=native.sha(root/'native_runtime.json'),
        binary_sha256=native.sha(data['binary']),ros_publications=False),indent=2)+'\n')
    return dict(output=str(output),variant=variant,mode=mode,exit_codes=exits)


def error_kernel_evidence(path,runtime):
    if path is None:raise ValueError('new error-only Kernel requires its own Nsight execution evidence')
    path=Path(path).resolve();root=Path(runtime['root'])
    if root/'traces' not in path.parents:raise ValueError('Kernel evidence must be inside the same private runtime/traces')
    evidence=json.loads(path.read_text())
    report_path=Path(evidence['nsys_report']).resolve()
    if path.parent!=report_path.parent or native.sha(report_path)!=evidence.get('report_sha256'):
        raise ValueError('Nsight report changed or belongs to another trace')
    request_path=path.parent/'request.json';request=json.loads(request_path.read_text())
    if runtime['binary'] not in request.get('command',[]):raise ValueError('Nsight used another native binary')
    if evidence.get('backend')!=verify.BACKEND or evidence.get('cuda_kernel_executed') is not True:
        raise ValueError('no executed P2D Kernel evidence')
    # Evidence must contain the NEW scalar kernel, not merely the unchanged
    # full-derivative kernel or an sm_72 symbol in a library.
    kernel_csv=path.parent/'kernel_summary.csv'
    from .. import trace
    kernels=trace.ndt_kernels(kernel_csv.read_text())
    if not any('p2d_ndt_error_only_kernel' in k['Name'] for k in kernels):
        raise ValueError('trace does not contain an executed error-only Kernel')
    return {str(p):native.sha(p) for p in [path,report_path,request_path,kernel_csv]}


def validate(a_run,other_run,output,kernel_evidence=None):
    output=Path(output)
    if output.exists():raise ValueError('verification output already exists')
    # The native wrapper seals source/library/binary evidence; plain host test
    # outputs cannot be confused with an actual native verification run.
    extra={};inventories=[]
    for directory in [Path(a_run),Path(other_run)]:
        p=directory/'native_evidence.json'
        e=json.loads(p.read_text())
        if e.get('mode')!='verify' or e.get('ros_publications') is not False:
            raise ValueError('not a native numerical verification run')
        mp=Path(e['runtime_manifest'])
        if native.sha(mp)!=e['runtime_manifest_sha256']:raise ValueError('runtime manifest changed')
        runtime=native.check(mp.parent)
        variant=json.loads((mp.parent/'deps/src/fast_gicp/lite3_p2d_variant.json').read_text())
        inventories.append(variant['baseline_inventory'])
        if native.sha(runtime['binary'])!=e['binary_sha256']:raise ValueError('native binary changed')
        extra[str(p.resolve())]=native.sha(p)
        extra[str(mp.resolve())]=native.sha(mp)
        extra.update(runtime['files'])
    result=verify.compare(a_run,other_run)
    if inventories[0]!=inventories[1]:raise ValueError('native A/B/C complete baseline source differs')
    result['baseline_inventory']=inventories[0]
    extra.update(error_kernel_evidence(kernel_evidence,runtime))
    result['cuda_error_kernel_executed']=True
    result['kernel_trace_separately_required']=False
    result['files'].update(extra)
    with output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    b=sub.add_parser('build')
    for name in ['root','source','ndt-prefix','official-manifest']:b.add_argument('--'+name,type=Path,required=True)
    b.add_argument('--b-receipt',type=Path)
    b.add_argument('--operator-confirmed-disarmed',action='store_true')
    r=sub.add_parser('run')
    for name in ['root','dataset','output']:r.add_argument('--'+name,type=Path,required=True)
    r.add_argument('--mode',choices=['verify','measure'],required=True)
    r.add_argument('--receipt',type=Path)
    r.add_argument('--warmup',type=int,default=2);r.add_argument('--repeats',type=int,default=3)
    r.add_argument('--cpu-reference',action='store_true')
    r.add_argument('--operator-confirmed-disarmed',action='store_true')
    v=sub.add_parser('validate')
    for name in ['a-run','other-run','output']:v.add_argument('--'+name,type=Path,required=True)
    v.add_argument('--kernel-evidence',type=Path,required=True)
    args=parser.parse_args()
    try:
        if args.action=='build':
            result=build(args.root,args.source,args.ndt_prefix,args.official_manifest,args.operator_confirmed_disarmed,args.b_receipt)
        elif args.action=='run':
            result=run_variant(args.root,args.dataset,args.output,args.mode,args.operator_confirmed_disarmed,
                               args.receipt,args.warmup,args.repeats,args.cpu_reference)
        else:result=validate(args.a_run,args.other_run,args.output,args.kernel_evidence)
    except (ValueError,OSError,KeyError) as error:
        parser.exit(2,str(error)+'\n')
    # The complete inventories/evidence are stored on disk, not printed as
    # hundreds of library/source hash lines in the operator terminal.
    if args.action=='build':
        result=dict(root=result['root'],binary=result['binary'],manifest=str(args.root/'native_runtime.json'))
    elif args.action=='validate':
        result={k:v for k,v in result.items() if k not in ['files','baseline_inventory']}
        result['receipt']=str(args.output)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
