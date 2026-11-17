#!/usr/bin/env python3
"""Repair the existing timesyncd configuration, with backups and no fake sync."""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

SERVERS = ('ntp.tuna.tsinghua.edu.cn', 'ntp.ubuntu.com', 'ntp.aliyun.com', 'time.cloudflare.com')
CONFIG = '/etc/systemd/timesyncd.conf.d/90-lite3-time.conf'
RTC_SCRIPT = '/usr/local/libexec/lite3-save-rtc'
RTC_UNIT = '/etc/systemd/system/lite3-save-rtc.service'
RTC_TIMER = '/etc/systemd/system/lite3-save-rtc.timer'
BLOCKERS = {'hdl_localization_composition', 'velocity_gate', 'velocity_gate.py',
            'controller_server', 'planner_server', 'bt_navigator', 'patrol_runner',
            'patrol_runner.py', 'voa_composition', 'voa_costmap_bridge.py',
            'leg_odom_tf_bridge.py', 'livox_ros_driver2_node', 'visual_pose_initializer',
            'rgbd_visual_localizer', 'fastlivo_mapping', 'fastlio_mapping',
            'lite3_ndt_benchmark', 'navigation_start.sh', 'start_localization_only.sh'}


def command(args):
    return subprocess.run(args, capture_output=True, text=True, timeout=12, check=False)


def active_processes(proc=Path('/proc')):
    result = []
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            argv = (entry/'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        # Inspect executable and actual script arguments, never shell command
        # strings containing this audit's list of names.
        names = {Path(a.decode(errors='replace')).name for a in argv[:2] if a}
        if names & BLOCKERS:
            result.append(dict(pid=int(entry.name), names=sorted(names & BLOCKERS)))
    return result


def configuration(servers):
    if not servers or any(not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.:-]{0,252}', s) for s in servers):
        raise ValueError('invalid NTP server name/address')
    # systemd 245 APPENDS repeated NTP= assignments across drop-ins. Reset
    # the old 20-campus list first so reachable IPv6 sources really lead.
    return ('[Time]\nNTP=\nNTP='+' '.join(servers)+'\n'
            'FallbackNTP=ntp.ubuntu.com time.cloudflare.com\n'
            'PollIntervalMinSec=32\nPollIntervalMaxSec=256\n')


def files(servers):
    return {
        CONFIG: (configuration(servers), 0o644),
        RTC_SCRIPT: ('#!/bin/sh\nset -eu\n'
                     'test "$(timedatectl show -p NTPSynchronized --value)" = yes || exit 0\n'
                     'exec /sbin/hwclock --systohc --utc --rtc /dev/rtc0\n', 0o755),
        RTC_UNIT: ('[Unit]\nDescription=Save trusted system time to Lite3 RTC\n'
                   'After=systemd-timesyncd.service\n'
                   '[Service]\nType=oneshot\nExecStart='+RTC_SCRIPT+'\n', 0o644),
        RTC_TIMER: ('[Unit]\nDescription=Retry trusted Lite3 RTC save after network synchronization\n'
                    '[Timer]\nOnBootSec=3min\nOnUnitActiveSec=15min\n'
                    'Unit=lite3-save-rtc.service\n[Install]\nWantedBy=timers.target\n', 0o644),
    }


def save_and_install(root, backup, planned):
    """Pure file transaction; root may be a temporary test filesystem."""
    manifest = {}
    for name, (text, mode) in planned.items():
        path = root/name.lstrip('/')
        if path.is_symlink():
            raise ValueError('refusing to replace symlink: '+name)
        saved = backup/name.lstrip('/')
        manifest[name] = dict(existed=path.exists())
        if path.exists():
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, saved)
    (backup/'files_manifest.json').write_text(json.dumps(manifest, indent=2))
    try:
        for name, (text, mode) in planned.items():
            path = root/name.lstrip('/')
            path.parent.mkdir(parents=True, exist_ok=True)
            temp = path.with_name(path.name+'.lite3-new')
            if temp.exists():
                raise ValueError('temporary configuration already exists: '+str(temp))
            try:
                with temp.open('x') as f:
                    f.write(text)
                temp.chmod(mode)
                temp.replace(path)
            finally:
                if temp.exists():
                    temp.unlink()
    except BaseException:
        restore_files(root, backup, manifest)
        raise
    return manifest


def restore_files(root, backup, manifest):
    for name, state in manifest.items():
        path = root/name.lstrip('/')
        if state['existed']:
            shutil.copy2(backup/name.lstrip('/'), path)
        elif path.exists():
            path.unlink()


def status():
    cmds = [ ['timedatectl', 'status'], ['timedatectl', 'timesync-status'],
             ['systemctl', 'is-enabled', 'systemd-timesyncd'],
             ['systemctl', 'is-active', 'systemd-timesyncd', 'chrony', 'ntp'],
             ['journalctl', '-b', '-u', 'systemd-timesyncd', '-n', '20', '--no-pager'] ]
    return [dict(command=a, exit=(r:=command(a)).returncode, stdout=r.stdout, stderr=r.stderr) for a in cmds]


def root_and_idle(confirmed):
    if os.geteuid() != 0:
        raise ValueError('run in the Jetson terminal with sudo; never send a password to chat')
    if not confirmed:
        raise ValueError('--operator-confirmed-safe is required (stationary, DISARMED, no RC operation)')
    if not Path('/etc/nv_tegra_release').is_file():
        raise ValueError('this installer is for the Jetson, not the development VM')
    busy = active_processes()
    if busy:
        raise ValueError('localization/navigation/perception experiment is still running: '+json.dumps(busy))


def apply(servers, wait_seconds, confirmed):
    root_and_idle(confirmed)
    for unit in ('chrony', 'ntp'):
        if command(['systemctl', 'is-active', unit]).returncode == 0:
            raise ValueError('another time client is active: '+unit+'; no changes made')
    planned = files(servers)
    # Preflight every path before the first write.
    for name in planned:
        path = Path(name)
        if path.is_symlink() or path.with_name(path.name+'.lite3-new').exists():
            raise ValueError('unsafe existing configuration path: '+name)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')
    backup = Path('/var/backups/lite3-time-{}-{}'.format(stamp, os.getpid()))
    backup.mkdir(mode=0o700)
    before = status()
    (backup/'before.json').write_text(json.dumps(before, indent=2))
    # Read firewall policy for diagnosis; never insert/delete firewall rules.
    fw = command(['/sbin/iptables-save'])
    (backup/'iptables-before.txt').write_text(fw.stdout+fw.stderr)
    enabled = command(['systemctl', 'is-enabled', 'systemd-timesyncd']).stdout.strip()
    active = command(['systemctl', 'is-active', 'systemd-timesyncd']).returncode == 0
    rtc_enabled = command(['systemctl', 'is-enabled', 'lite3-save-rtc.timer']).stdout.strip()
    rtc_active = command(['systemctl', 'is-active', 'lite3-save-rtc.timer']).returncode == 0
    manifest = save_and_install(Path('/'), backup, planned)
    record = dict(files=manifest, timesyncd_enabled=enabled, timesyncd_active=active,
                  rtc_timer_enabled=rtc_enabled, rtc_timer_active=rtc_active)
    (backup/'manifest.json').write_text(json.dumps(record, indent=2))
    print(json.dumps({'backup':str(backup),'servers':servers,'note':'no clock set and no fabricated sync flag'}), flush=True)
    marker = Path('/run/systemd/timesync/synchronized')
    previous_marker_ns = marker.stat().st_mtime_ns if marker.exists() else None
    for args in (['systemctl','daemon-reload'], ['systemctl','enable','--now','systemd-timesyncd'],
                 ['systemctl','restart','systemd-timesyncd'],
                 ['systemctl','enable','--now','lite3-save-rtc.timer']):
        root_and_idle(confirmed)
        r = command(args)
        if r.returncode:
            raise ValueError('command failed: '+repr(args)+' '+r.stderr+'; backup='+str(backup))
    until = time.monotonic()+wait_seconds
    synchronized = False
    while time.monotonic() < until:
        root_and_idle(confirmed)
        fresh_marker = marker.exists() and marker.stat().st_mtime_ns != previous_marker_ns
        if fresh_marker and command(['timedatectl','show','-p','NTPSynchronized','--value']).stdout.strip() == 'yes':
            synchronized = True
            command(['systemctl','start','lite3-save-rtc.service'])
            break
        time.sleep(2)
    after = dict(synchronized=synchronized, status=status(), backup=str(backup), reboot_verified=False)
    (backup/'after.json').write_text(json.dumps(after, indent=2))
    print(json.dumps(after, ensure_ascii=False, indent=2))
    if not synchronized:
        raise ValueError('NTP still unavailable; configuration installed but synchronization NOT repaired. Obtain a reachable trusted NTP source. Backup='+str(backup))


def rollback(backup, confirmed):
    root_and_idle(confirmed)
    backup = Path(backup).resolve()
    if backup.parent != Path('/var/backups') or not backup.name.startswith('lite3-time-'):
        raise ValueError('expected the exact backup path reported by apply')
    record = json.loads((backup/'manifest.json').read_text())
    command(['systemctl','disable','--now','lite3-save-rtc.timer'])
    restore_files(Path('/'), backup, record['files'])
    command(['systemctl','daemon-reload'])
    if record['timesyncd_enabled'] == 'enabled':
        command(['systemctl','enable','systemd-timesyncd'])
    else:
        command(['systemctl','disable','systemd-timesyncd'])
    command(['systemctl','restart' if record['timesyncd_active'] else 'stop','systemd-timesyncd'])
    # Also support rolling back a later update over an already installed timer.
    if record.get('rtc_timer_enabled') == 'enabled':
        command(['systemctl','enable','lite3-save-rtc.timer'])
    if record.get('rtc_timer_active'):
        command(['systemctl','start','lite3-save-rtc.timer'])
    print('Configuration restored; deliberately did not restore an incorrect old date/RTC.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    sub.add_parser('status', help='read-only existing time service evidence')
    a = sub.add_parser('apply', help='Jetson only: back up and repair existing timesyncd')
    a.add_argument('--server', action='append')
    a.add_argument('--wait-seconds', type=int, default=90)
    a.add_argument('--operator-confirmed-safe', action='store_true')
    r = sub.add_parser('rollback', help='restore only files saved by apply, never reset clocks backwards')
    r.add_argument('--backup', type=Path, required=True)
    r.add_argument('--operator-confirmed-safe', action='store_true')
    args = p.parse_args()
    try:
        if args.action == 'status':
            print(json.dumps(status(), ensure_ascii=False, indent=2))
        elif args.action == 'apply':
            if not 0 <= args.wait_seconds <= 300:
                raise ValueError('--wait-seconds must be 0..300')
            apply(args.server or list(SERVERS), args.wait_seconds, args.operator_confirmed_safe)
        else:
            rollback(args.backup, args.operator_confirmed_safe)
        return 0
    except (ValueError, OSError, subprocess.SubprocessError) as e:
        print('[TIME REPAIR FAIL] '+str(e), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
