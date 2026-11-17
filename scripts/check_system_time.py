#!/usr/bin/env python3
"""Fail closed before project localization; never changes either clock."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

MIN_EPOCH = 1704067200.0  # 2024-01-01 UTC; a range check is not sync evidence.
MAX_EPOCH = 4102444800.0  # 2100-01-01 UTC.
MAX_CLOCK_STEP_S = 0.5


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def validate(snapshot):
    if not math.isfinite(snapshot['wall']) or not MIN_EPOCH <= snapshot['wall'] < MAX_EPOCH:
        raise ValueError('system date is outside the supported range')
    if snapshot['synchronized'] != 'yes':
        raise ValueError('system time is NOT NTP synchronized; check timedatectl timesync-status')
    if not snapshot.get('timesyncd_synchronized_marker', False):
        raise ValueError('no successful timesyncd synchronization evidence from this boot')
    if snapshot['timezone'] != 'Asia/Shanghai':
        raise ValueError('expected Asia/Shanghai timezone')
    if not snapshot['boot_id']:
        raise ValueError('cannot identify this boot')
    if (not math.isfinite(snapshot['monotonic']) or snapshot['monotonic'] < 0
            or not math.isfinite(snapshot['sample_clock_step_s'])
            or abs(snapshot['sample_clock_step_s']) > MAX_CLOCK_STEP_S):
        raise ValueError('system clock changed during preflight; wait for synchronization to settle')
    return snapshot


def snapshot():
    # A single DBus request; unavailable/failed service evidence is not success.
    result = subprocess.run(
        ['timedatectl', 'show', '-p', 'NTPSynchronized', '-p', 'Timezone'],
        capture_output=True, text=True, check=True, timeout=5)
    values = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    m0, w0 = time.monotonic(), time.time()
    time.sleep(0.5)
    m1, w1 = time.monotonic(), time.time()
    return validate(dict(wall=w1, monotonic=m1, boot_id=boot_id(),
                         synchronized=values.get('NTPSynchronized', 'unknown'),
                         timesyncd_synchronized_marker=Path('/run/systemd/timesync/synchronized').is_file(),
                         timezone=values.get('Timezone', 'unknown'),
                         sample_clock_step_s=(w1-w0)-(m1-m0)))


def check_offset(baseline, wall, monotonic, current_boot):
    if baseline['boot_id'] != current_boot:
        raise ValueError('boot changed; cached scans/TF must not be reused')
    if not all(math.isfinite(x) for x in (wall, monotonic)):
        raise ValueError('invalid clock sample')
    difference = (wall-baseline['wall'])-(monotonic-baseline['monotonic'])
    if abs(difference) > MAX_CLOCK_STEP_S:
        raise ValueError('system clock moved by {:.6f}s; stop localization and reinitialize'.format(difference))


def process_alive(pid):
    try:
        # kill(pid, 0) alone considers an unreaped child zombie alive forever.
        stat = Path('/proc/{}/stat'.format(pid)).read_text()
        return stat.rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def watch(pid, baseline):
    validate(baseline)
    while process_alive(pid):
        check_offset(baseline, time.time(), time.monotonic(), boot_id())
        time.sleep(0.25)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('check', help='read-only clock and NTP preflight; JSON baseline on success')
    p = sub.add_parser('watch', help='watch the owned localization launch; never sends a signal')
    p.add_argument('--pid', type=int, required=True)
    p.add_argument('--baseline', required=True)
    args = parser.parse_args()
    try:
        if args.action == 'check':
            print(json.dumps(snapshot(), allow_nan=False))
            return 0
        if args.pid <= 1 or args.pid == os.getpid():
            raise ValueError('expected an owned child PID')
        return watch(args.pid, json.loads(args.baseline))
    except (ValueError, KeyError, TypeError, OSError, subprocess.SubprocessError) as error:
        print('[TIME FAIL] ' + str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
