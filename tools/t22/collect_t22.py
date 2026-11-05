#!/usr/bin/env python3
"""Copy an explicitly selected capture; never deploy, remove, or run ROS nodes."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys

from analyze_t22 import analyze

# Executed only after the operator invokes this collector with an explicit host.
# It reads diagnostic files and returns metadata/hashes; it creates no remote files.
INVENTORY = r'''
from pathlib import Path
import hashlib,json,re,sys
name=sys.argv[1]
if not re.fullmatch('[A-Za-z0-9_-]{1,48}',name):raise SystemExit('invalid capture_id')
items=[]
for path in sorted(Path('/tmp').glob('lite3_t22_'+name+'_*.jsonl')):
 if path.is_symlink() or not path.is_file():raise SystemExit('unsafe result file')
 if not re.fullmatch('lite3_t22_'+re.escape(name)+r'_(leg|voa)_[0-9]+\.jsonl',path.name):raise SystemExit('unexpected result filename')
 size=path.stat().st_size
 if not 0<size<=64*1024*1024:raise SystemExit('empty or unexpectedly large result')
 meta=None;finished=False;errors=[];drops=None
 with path.open() as handle:
  for line in handle:
   try:row=json.loads(line)
   except ValueError:errors.append('invalid/truncated JSON');continue
   if not isinstance(row,dict):errors.append('invalid record');continue
   if row.get('kind')=='metadata':
    if meta is not None:errors.append('duplicate metadata')
    meta=row
   elif row.get('kind')=='batch':finished=row.get('finished') is True;drops=row.get('dropped_rows')
   elif row.get('kind')=='recorder_error':errors.append(row.get('error','recorder_error'))
 if not meta or meta.get('capture_id')!=name or meta.get('role') not in ('leg','voa'):raise SystemExit('metadata mismatch')
 digest=hashlib.sha256()
 with path.open('rb') as handle:
  for chunk in iter(lambda:handle.read(1048576),b''):digest.update(chunk)
 items.append(dict(name=path.name,bytes=size,sha256=digest.hexdigest(),metadata=meta,finished=finished,dropped_rows=drops,recorder_error=errors))
print(json.dumps(items))
'''


def collect(host, capture_id, output, identity=None, allow_partial=False, runner=None):
    if (not host or host.startswith('-') or not re.fullmatch(r'[A-Za-z0-9_.@\[\]:-]+',host)):
        raise ValueError('provide an explicit SSH Host Alias or user@host')
    if ':' in host and '[' not in host:
        raise ValueError('use an SSH Host Alias for IPv6, or a bracketed host')
    if not re.fullmatch('[A-Za-z0-9_-]{1,48}',capture_id):
        raise ValueError('invalid capture_id')
    destination=Path(output).expanduser()/capture_id
    if destination.exists():raise ValueError('destination already exists; refusing to overwrite '+str(destination))
    runner=runner or subprocess.run
    options=['-o','BatchMode=yes','-o','ConnectTimeout=8']
    if identity:options+=['-i',str(Path(identity).expanduser())]
    command='python3 -c '+shlex.quote(INVENTORY)+' '+shlex.quote(capture_id)
    response=runner(['ssh']+options+[host,command],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=30)
    if response.returncode:raise ValueError('remote inventory failed: '+response.stderr.strip())
    items=json.loads(response.stdout)
    if not isinstance(items,list) or not items:raise ValueError('no capture files found')
    roles={role:[v for v in items if v.get('metadata',{}).get('role')==role] for role in ('leg','voa')}
    if any(len(v)>1 for v in roles.values()):raise ValueError('multiple processes for one role; refusing ambiguous capture')
    if not allow_partial and any(len(v)!=1 for v in roles.values()):raise ValueError('both leg and voa files are required')
    identities=set()
    for item in items:
        if not re.fullmatch('lite3_t22_'+re.escape(capture_id)+r'_(leg|voa)_[0-9]+\.jsonl',item.get('name','')):
            raise ValueError('unsafe remote filename')
        meta=item['metadata']
        if meta.get('capture_id')!=capture_id:raise ValueError('capture_id mismatch')
        if not re.fullmatch('lite3_t22_'+re.escape(capture_id)+'_'+re.escape(str(meta.get('role')))+r'_[0-9]+\.jsonl',item['name']):
            raise ValueError('filename role and metadata differ')
        if (type(item.get('bytes')) is not int or not 0<item['bytes']<=64*1024*1024
                or not re.fullmatch('[a-f0-9]{64}',str(item.get('sha256','')))):
            raise ValueError('invalid size or SHA-256 in remote inventory')
        identities.add(tuple(meta.get(k) for k in ('capture_id','boot_id','enable_id','start_ns','duration_s')))
        if not allow_partial and (not item.get('finished') or item.get('recorder_error')):
            raise ValueError('capture not finished or recorder_error; inspect status, or explicitly --allow-partial')
    if len(identities)!=1:raise ValueError('different capture windows or boots')
    destination.mkdir(parents=True)  # Atomic reservation: existing history is never overwritten.
    try:
        paths=[]
        for item in items:
            target=destination/item['name']
            response=runner(['scp','-q']+options+[host+':/tmp/'+item['name'],str(target)],
                            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=60)
            if response.returncode:raise ValueError('copy failed: '+response.stderr.strip())
            digest=hashlib.sha256(target.read_bytes()).hexdigest()
            if target.stat().st_size!=item['bytes'] or digest!=item['sha256']:
                raise ValueError('file changed during collection or checksum mismatch: '+item['name'])
            paths.append(target)
        report=analyze(paths)
        with (destination/'analysis.json').open('x') as handle:json.dump(report,handle,ensure_ascii=False,indent=2)
        manifest=dict(capture_id=capture_id,host=host,copy_verified=True,remote_files_preserved=True,
                      analysis_complete=report['complete'],files=items,destination=str(destination))
        with (destination/'collection_manifest.json').open('x') as handle:json.dump(manifest,handle,ensure_ascii=False,indent=2)
        return manifest
    except Exception as error:
        with (destination/'collection_error.json').open('x') as handle:
            json.dump(dict(error=str(error),partial_files_preserved=True),handle,ensure_ascii=False,indent=2)
        raise


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True,help='用户明确指定的 SSH Host Alias 或 user@host；无默认地址')
    parser.add_argument('--capture-id',required=True)
    parser.add_argument('--output',required=True,type=Path,help='开发端父目录；在其下创建新的 capture_id 子目录')
    parser.add_argument('--identity',help='可选的现有 SSH 私钥路径；不创建或修改密钥')
    parser.add_argument('--allow-partial',action='store_true',help='明确收集未结束或缺少一端的原始文件，并标记部分结果')
    args=parser.parse_args(argv)
    try:
        result=collect(args.host,args.capture_id,args.output,args.identity,args.allow_partial)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result['analysis_complete'] else 1
    except (OSError,ValueError,subprocess.TimeoutExpired) as error:
        print(json.dumps(dict(error=str(error)),ensure_ascii=False),file=sys.stderr)
        return 2


if __name__=='__main__':sys.exit(main())
