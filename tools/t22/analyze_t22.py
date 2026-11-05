#!/usr/bin/env python3
"""Offline T2.2 analysis. No ROS imports or network operations."""
import argparse
from collections import Counter
from decimal import Decimal
import json
import math
from pathlib import Path
import re
import statistics
import sys


def stats_ns(values):
    ordered = sorted(values)
    if not ordered:
        return dict(n=0, mean_ms=None, p50_ms=None, p95_ms=None, max_ms=None,
                    over_20ms=0, over_50ms=0, over_100ms=0, negative_count=0)
    return dict(n=len(ordered), mean_ms=statistics.mean(ordered)/1e6,
                p50_ms=statistics.median(ordered)/1e6,
                p95_ms=ordered[math.ceil(.95*len(ordered))-1]/1e6, max_ms=ordered[-1]/1e6,
                over_20ms=sum(v > 20000000 for v in ordered),
                over_50ms=sum(v > 50000000 for v in ordered),
                over_100ms=sum(v > 100000000 for v in ordered),
                negative_count=sum(v < 0 for v in ordered))


def integer(value):
    return type(value) is int


def read_capture(filename):
    metadata, rows, batches, errors, issues = [], [], [], [], []
    with Path(filename).open() as handle:
        for number, line in enumerate(handle, 1):
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('not an object')
            except ValueError:
                issues.append('invalid/truncated JSON at line '+str(number))
                continue
            kind = row.get('kind')
            if kind == 'metadata':
                metadata.append(row)
                if number != 1: issues.append('metadata is not first')
            elif kind == 'batch':
                batches.append(row)
                if isinstance(row.get('rows'), list):
                    rows.extend(r for r in row['rows'] if isinstance(r, dict))
                    if any(not isinstance(r, dict) for r in row['rows']): issues.append('invalid row')
                else: issues.append('batch rows missing')
                if row.get('batch_index') != len(batches): issues.append('missing/reordered batch')
                if row.get('written_rows') != len(rows): issues.append('written_rows mismatch')
                if not integer(row.get('dropped_rows')) or row['dropped_rows'] < 0:
                    issues.append('invalid dropped_rows')
            elif kind == 'recorder_error': errors.append(row.get('error', 'recorder_error'))
            else: issues.append('unknown top-level record')
    meta = metadata[0] if len(metadata) == 1 else None
    if meta is None: issues.append('exactly one metadata record required')
    valid = bool(meta and integer(meta.get('schema_version')) and meta['schema_version'] == 1
        and meta.get('role') in ('leg', 'voa')
        and all(isinstance(meta.get(k), str) and meta[k] for k in ('capture_id','boot_id','enable_id'))
        and all(integer(meta.get(k)) for k in
                ('start_ns','duration_s','end_ns','accepted_ns','issued_ns','probe_created_ns'))
        and meta['start_ns'] > 0 and 30 <= meta['duration_s'] <= 60
        and 0 < meta['probe_created_ns'] < meta['issued_ns'] <= meta['accepted_ns']
        and meta['end_ns'] == meta['start_ns']+meta['duration_s']*1000000000
        and meta.get('clock') == 'CLOCK_MONOTONIC ns')
    if not valid: issues.append('invalid/unsupported metadata; rows excluded')
    last = batches[-1] if batches else {}
    finished = last.get('finished') is True
    if not finished: issues.append('no finished=true final batch')
    window_complete = bool(valid and finished and meta['accepted_ns'] < meta['start_ns']
                           and integer(last.get('at_ns')) and last['at_ns'] >= meta['end_ns'])
    if not window_complete:
        issues.append('sampling window not fully covered')
    drops = last.get('dropped_rows')
    if not integer(drops) or drops < 0: drops = None
    if drops: issues.append('dropped_rows > 0')
    if errors: issues.append('recorder_error')
    counts = last.get('counts', {})
    if not isinstance(counts, dict) or any(not isinstance(k,str) or not integer(v) or v<0 for k,v in counts.items()):
        issues.append('invalid counters'); counts = {}
    return dict(metadata=meta if valid else None, rows=rows if valid else [], counts=counts,
                quality=dict(file=str(filename), role=meta.get('role') if meta else None,
                    finished=finished, sampling_window_complete=window_complete,
                    dropped_rows=drops, recorder_error=errors, issues=issues),
                cpu=dict(bookkeeping_thread_cpu_ns=last.get('bookkeeping_thread_cpu_ns'),
                         writer_thread_cpu_ns=last.get('writer_thread_cpu_ns'),
                         process_cpu_ns=last.get('process_cpu_ns'),
                         process_cpu_base_ns=meta.get('process_cpu_base_ns') if meta else None,
                         observed_elapsed_ns=(last['at_ns']-meta['accepted_ns']
                             if valid and integer(last.get('at_ns')) else None)))


def analyze(files):
    if not files: raise ValueError('provide one leg and one voa JSONL file')
    captures = [read_capture(path) for path in files]
    metadata = [c['metadata'] for c in captures if c['metadata']]
    identity = {(m['capture_id'],m['boot_id'],m['enable_id'],m['start_ns'],m['duration_s']) for m in metadata}
    if len(identity) > 1:
        raise ValueError('capture_id, boot_id, enable_id, start_ns or duration_s differ; refusing to merge')
    roles = Counter(m['role'] for m in metadata)
    if any(v > 1 for v in roles.values()):
        raise ValueError('multiple files for one role; select exactly one process per role')
    warnings = []
    for role in ('leg','voa'):
        if roles[role] != 1: warnings.append('missing role: '+role)
    records, counts = [], Counter()
    for c in captures:
        records.extend(c['rows']); counts.update(c['counts'])
        warnings.extend(str(c['quality']['file'])+': '+s for s in c['quality']['issues'])
        allowed = ({'leg_input','leg_tick','leg_tx','leg_tx_error'} if
                   (c['metadata'] or {}).get('role') == 'leg' else
                   {'leg_rx','listener_callback','lookup_failure_example'})
        if any(r.get('kind') not in allowed for r in c['rows']):
            warnings.append('unexpected event kind for file role: '+str(c['quality']['file']))
    publish_errors = [r for r in records if r.get('kind') == 'leg_tx_error']
    if publish_errors: warnings.append('leg publication exceptions recorded')
    tx_groups, rx_groups = {}, {}
    for r in records:
        if r.get('kind') not in ('leg_tx','leg_rx'): continue
        if not integer(r.get('source_stamp_ns')) or r['source_stamp_ns'] <= 0:
            warnings.append('invalid source_stamp'); continue
        if r['kind']=='leg_rx' and r.get('outcome') != 'returned_no_exception':
            warnings.append('Buffer insertion exception: '+str(r.get('outcome'))); continue
        if r['kind']=='leg_rx' and (r.get('parent')!='robot_odom' or r.get('child')!='leg_base_link'):
            warnings.append('unexpected tracked TF frames'); continue
        groups = tx_groups if r['kind']=='leg_tx' else rx_groups
        groups.setdefault(r['source_stamp_ns'],[]).append(r)
    duplicates = dict(tx=sum(len(v)-1 for v in tx_groups.values()),
                      rx=sum(len(v)-1 for v in rx_groups.values()))
    if any(duplicates.values()): warnings.append('duplicate source_stamp; ambiguous stamps excluded')
    tx = {k:v[0] for k,v in tx_groups.items() if len(v)==1}
    rx = {k:v[0] for k,v in rx_groups.items() if len(v)==1}
    if not tx: warnings.append('no unique leg_tx records')
    if not rx: warnings.append('no unique leg_rx records')
    inputs = [r for r in records if r.get('kind') == 'leg_input']
    if not inputs: warnings.append('no leg input callback records')
    unmatched_tx, unmatched_rx = len(tx.keys()-rx.keys()), len(rx.keys()-tx.keys())
    if unmatched_tx or unmatched_rx: warnings.append('unmatched source_stamp (including possible window-edge samples)')
    metrics = {}
    def differences(rows, end, begin, label, negative_allowed=False):
        values = []
        for r in rows:
            if not integer(r.get(end)) or not integer(r.get(begin)):
                warnings.append('missing/non-integer timestamp for '+label); continue
            value = r[end]-r[begin]; values.append(value)
            if value < 0 and not negative_allowed: warnings.append('invalid negative local interval: '+label)
        return stats_ns(values)
    metrics['T2_enter_to_valid']=differences(inputs,'t2_valid_ns','t2_enter_ns','T2_enter_to_valid')
    for name,end,begin in [('T2_valid_to_T3a','t3a_ns','t2_valid_ns'),('cache_to_T3a','t3a_ns','cached_ns'),
            ('T3a_to_T3b','t3b_ns','t3a_ns'),('deepcopy','copy_end_ns','copy_begin_ns'),
            ('sampled_publish_call','t3b_ns','sampled_begin_ns'),('TF_build','t3c_ns','t3b_ns'),
            ('T3c_to_T3d','t3d_ns','t3c_ns')]:
        metrics[name]=differences(tx.values(),end,begin,name)
    metrics['T1_to_T2']=dict(stats_ns([]),reason='not measured: actual C++ publish time T1 is absent')
    metrics['T4b_to_T4c']=differences(rx.values(),'t4c_ns','t4b_ns','T4b_to_T4c')
    metrics['T4a_to_T4b']=differences(rx.values(),'t4b_ns','t4a_ns','T4a_to_T4b')
    matched = tx.keys() & rx.keys()
    delivery, available = [], []
    for key in matched:
        t,r=tx[key],rx[key]
        if all(integer(x) for x in (t.get('t3c_ns'),t.get('t3d_ns'),r.get('t4a_ns'),r.get('t4c_ns'))):
            delivery.append(r['t4a_ns']-t['t3d_ns'])
            available.append(r['t4c_ns']-t['t3c_ns'])
            # Delivery can precede publish RETURN, but cannot precede its START.
            if r['t4a_ns'] < t['t3c_ns'] or r['t4c_ns'] < t['t3c_ns']:
                warnings.append('unreasonable cross-process negative: receive precedes sendTransform entry')
        else: warnings.append('missing timestamp in matched pair')
    metrics['T3d_to_T4a']=stats_ns(delivery)
    metrics['T3c_to_T4c_call_return']=stats_ns(available)
    ticks=[r['interval_ns'] for r in records if r.get('kind')=='leg_tick' and integer(r.get('interval_ns'))]
    metrics['timer_interval']=stats_ns(ticks)
    metrics['timer_interval_minus_50ms']=stats_ns([v-50000000 for v in ticks])
    metrics['listener_callback']=differences([r for r in records if r.get('kind')=='listener_callback'],
                                            'returned_ns','t4a_ns','listener_callback')
    if any(v<0 for v in ticks): warnings.append('negative timer interval')
    if not ticks: warnings.append('no timer intervals')
    examples=[]
    for r in records:
        if r.get('kind')!='lookup_failure_example': continue
        text=r.get('error','')
        if not isinstance(text,str):
            warnings.append('invalid lookup exception text');text=str(text)
        match=re.search(r'latest data is at time ([0-9]+\.[0-9]+)',text)
        latest=int(Decimal(match.group(1))*1000000000) if match else None
        grid_age=r.get('grid_age_ns')
        if not integer(grid_age): warnings.append('missing/non-integer lookup grid_age_ns')
        examples.append(dict(stage=r.get('stage'),error_type=r.get('error_type'),direction=r.get('direction'),
            grid_stamp_ns=r.get('grid_stamp_ns'),grid_age_ms=grid_age/1e6 if integer(grid_age) else None,
            requested_stamp_ns=r.get('requested_stamp_ns'),buffer_latest_from_exception_ns=latest,
            last_set_returned_stamp_ns=r.get('last_set_returned_stamp_ns'),
            shortfall_ms=(r['requested_stamp_ns']-latest)/1e6 if latest is not None and integer(r.get('requested_stamp_ns')) else None,
            error=text))
    warnings=list(dict.fromkeys(warnings))
    complete=not warnings
    future_by_stage = {stage:sum(v for k,v in counts.items()
                       if k.startswith('lookup.'+stage+'.') and k.endswith('.future'))
                       for stage in ('map_grid','leg_anchor','leg_grid')}
    return dict(complete=complete,status='complete' if complete else 'partial',warnings=warnings,
        sampling_window_complete=bool(roles['leg']==1 and roles['voa']==1 and
            all(c['quality']['sampling_window_complete'] for c in captures)),
        identity=metadata[0] if metadata else None,metrics=metrics,
        quality=[c['quality'] for c in captures],recorder_cpu=[c['cpu'] for c in captures],
        duplicate_source_stamps=duplicates,matched_source_stamps=len(matched),
        unmatched_tx=unmatched_tx,unmatched_rx=unmatched_rx,
        dropped_rows=(sum(c['quality']['dropped_rows'] for c in captures)
                      if all(c['quality']['dropped_rows'] is not None for c in captures) else None),
        recorder_error=[e for c in captures for e in c['quality']['recorder_error']],counts=dict(counts),
        future_lookup_calls=sum(v for k,v in counts.items() if k.startswith('lookup.') and k.endswith('.future')),
        future_lookup_calls_by_stage=future_by_stage,
        sampled_failure_examples=examples,publish_errors=publish_errors,limitations=[
            'T1 is unmeasured; Header.stamp is not a substitute for publish time.',
            'T3d is an API return; T4a-T3d includes DDS and subscription scheduling and can legitimately be negative.',
            'set_transform returning without exception is not proof of native acceptance/queryability.',
            'Failure counters include retries; detailed failure examples are rate limited.',
            'Recorder CPU is a lower bound; process_cpu_ns includes business work.',
            'Partial captures are not declared passed; duplicate stamps are excluded, not silently selected.'])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files',nargs='+',help='同一窗口的 leg 和 voa 原始 JSONL')
    parser.add_argument('--json',action='store_true',help='标准输出使用完整 JSON')
    parser.add_argument('--json-output',type=Path,help='另外保存 JSON 报告；已有文件拒绝覆盖')
    args=parser.parse_args(argv)
    try:
        result=analyze(args.files)
        if args.json_output:
            with args.json_output.open('x') as handle:json.dump(result,handle,ensure_ascii=False,indent=2)
        if args.json: print(json.dumps(result,ensure_ascii=False,indent=2))
        else:
            print('完整性: '+result['status'])
            print('窗口完整: {}；匹配 {}；unmatched_tx={} unmatched_rx={}；重复={}'.format(
                result['sampling_window_complete'],result['matched_source_stamps'],result['unmatched_tx'],
                result['unmatched_rx'],result['duplicate_source_stamps']))
            print('dropped_rows={}；recorder_error={}；未来外推调用数={}，按查询段={}'.format(
                result['dropped_rows'],result['recorder_error'],result['future_lookup_calls'],
                result['future_lookup_calls_by_stage']))
            print('指标 | N | mean/P50/P95/max (ms) | >20/>50/>100 ms | negative')
            for key,m in result['metrics'].items():
                values=['—' if m[k] is None else '{:.3f}'.format(m[k]) for k in ('mean_ms','p50_ms','p95_ms','max_ms')]
                print('{} | {} | {} | {}/{}/{} | {}'.format(key,m['n'],' / '.join(values),m['over_20ms'],m['over_50ms'],m['over_100ms'],m['negative_count']))
            for warning in result['warnings']:print('WARNING: '+warning)
            print('T1 未测；T3d 不是送达确认；Buffer 返回不等于已验证可查询。')
        return 0 if result['complete'] else 1
    except (OSError,ValueError,TypeError) as error:
        print(json.dumps(dict(status='rejected',error=str(error)),ensure_ascii=False),file=sys.stderr)
        return 2


if __name__=='__main__':sys.exit(main())
