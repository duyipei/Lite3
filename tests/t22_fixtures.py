"""Small, ROS-free fixtures shared by the T2.2 offline tests."""
from collections import Counter
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'tools/t22'))
import lite3_t22_probe as recorder

BOOT = 'test-jetson-boot'
TOKEN = 'a'*32
NS_SECOND = 1000000000


class Clock:
    def __init__(self, now=100*NS_SECOND): self.now = now
    def __call__(self): return self.now
    def advance(self, seconds): self.now += int(seconds*NS_SECOND)


def marker(directory, now=99*NS_SECOND):
    value = dict(schema_version=1, boot_id=BOOT, enable_id=TOKEN, enabled_ns=now)
    recorder.atomic_json(Path(directory)/recorder.ENABLE, value)
    return value


def request(directory, clock, name='test_capture', duration=45):
    value = dict(schema_version=1, boot_id=BOOT, enable_id=TOKEN, capture_id=name,
                 issued_ns=clock(), start_ns=clock()+10*NS_SECOND, duration_s=duration)
    recorder.atomic_json(Path(directory)/recorder.REQUEST, value)
    return value


def header(source_ns):
    return NS(stamp=NS(sec=source_ns//NS_SECOND, nanosec=source_ns % NS_SECOND),
              frame_id='robot_odom')


def sample_rows(start=110*NS_SECOND):
    t = start+NS_SECOND
    source = t-2000000
    tx = dict(kind='leg_tx', source_stamp_ns=source, t2_enter_ns=t,
              t2_valid_ns=t+1000, cached_ns=t+2000, t3a_ns=t+5000000,
              copy_begin_ns=t+5001000, copy_end_ns=t+5011000,
              sampled_begin_ns=t+5012000, t3b_ns=t+5022000,
              t3c_ns=t+5040000, t3d_ns=t+5060000)
    rx = dict(kind='leg_rx', source_stamp_ns=source, parent='robot_odom', child='leg_base_link',
              t4a_ns=t+5080000, t4b_ns=t+5090000, t4c_ns=t+5100000,
              batch_tf_count=1, outcome='returned_no_exception', queryability_checked=False)
    inp = dict(kind='leg_input', source_stamp_ns=source, t2_enter_ns=t,
               t2_valid_ns=t+1000, cached_ns=t+2000)
    leg = [inp, dict(kind='leg_tick', t3a_ns=tx['t3a_ns'], interval_ns=None), tx,
           dict(kind='leg_tick', t3a_ns=tx['t3a_ns']+50000000, interval_ns=50000000)]
    voa = [rx, dict(kind='listener_callback', t4a_ns=rx['t4a_ns'],
                   returned_ns=rx['t4c_ns']+1000, batch_tf_count=1)]
    return leg, voa


def capture_file(directory, role, rows=None, **overrides):
    meta = dict(kind='metadata', schema_version=1, capture_id='test_capture', boot_id=BOOT,
                enable_id=TOKEN, start_ns=110*NS_SECOND, duration_s=45,
                end_ns=155*NS_SECOND, accepted_ns=101*NS_SECOND,
                probe_created_ns=100*NS_SECOND, issued_ns=101*NS_SECOND,
                role=role, pid=1 if role=='leg' else 2,
                clock='CLOCK_MONOTONIC ns')
    meta.update(overrides.pop('metadata', {}))
    if rows is None: rows = sample_rows(meta['start_ns'])[0 if role=='leg' else 1]
    batch = dict(kind='batch', batch_index=1, at_ns=meta['end_ns'], finished=True,
                 rows=rows, written_rows=len(rows), counts=dict(Counter(r['kind'] for r in rows)),
                 dropped_rows=0, bookkeeping_thread_cpu_ns=1000, writer_thread_cpu_ns=2000,
                 process_cpu_ns=20000000)
    batch.update(overrides.pop('batch', {}))
    name = overrides.pop('filename', 'lite3_t22_{}_{}_{}.jsonl'.format(meta['capture_id'], role, meta['pid']))
    path = Path(directory)/name
    path.write_text(json.dumps(meta)+'\n'+json.dumps(batch)+'\n')
    return path
