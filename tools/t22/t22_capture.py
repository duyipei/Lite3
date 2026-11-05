#!/usr/bin/env python3
"""Run on Jetson: control diagnostic files only. No ROS, SSH, or node control."""
import argparse
import datetime
import json
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from lite3_t22_probe import (ENABLE, REQUEST, ROLES, SCHEMA, atomic_json, boot_id,
                             read_json, validate_marker, validate_request)


def result_summary(path):
    result = dict(name=path.name, bytes=path.stat().st_size, metadata=None,
                  finished=False, dropped_rows=None, recorder_error=[])
    if path.is_symlink():
        raise ValueError('refusing a symlink result')
    with path.open() as handle:
        for number, line in enumerate(handle, 1):
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('not a JSON object')
            except ValueError:
                result['recorder_error'].append('invalid/truncated JSON at line '+str(number))
                continue
            if row.get('kind') == 'metadata':
                if result['metadata'] is not None:
                    result['recorder_error'].append('duplicate metadata')
                result['metadata'] = row
            elif row.get('kind') == 'batch':
                result['finished'] = row.get('finished') is True
                result['dropped_rows'] = row.get('dropped_rows')
            elif row.get('kind') == 'recorder_error':
                result['recorder_error'].append(row.get('error', 'recorder_error'))
    return result


class CaptureControl:
    def __init__(self, root='/tmp', current_boot=None, clock_ns=None):
        self.root = Path(root).resolve()
        self.root.relative_to(Path('/tmp'))
        self.boot = current_boot or boot_id()
        self.clock_ns = clock_ns or time.monotonic_ns

    def marker(self):
        return validate_marker(read_json(self.root/ENABLE), self.boot)

    def _existing_request(self):
        try:
            value = read_json(self.root/REQUEST)
        except FileNotFoundError:
            return None
        return validate_request(value, self.marker(), self.boot)

    def _not_in_progress(self, require_finished=False):
        try:
            request = self._existing_request()
        except (OSError, ValueError) as error:
            raise ValueError('cannot validate existing request; inspect status, then cleanup --force if appropriate: '+str(error))
        if request and self.clock_ns() < request['end_ns']:
            raise ValueError('capture is scheduled or running; cleanup refuses unless --force is explicit')
        if request and require_finished and not self.status(request['capture_id'])['both_finished']:
            raise ValueError('window ended but both final files are not finished; wait for writer flush, or explicitly cleanup --force')

    def enable(self):
        self._not_in_progress()
        try:
            value = self.marker()
            reused = True
        except (OSError, ValueError):
            value = dict(schema_version=SCHEMA, enable_id=uuid.uuid4().hex,
                         boot_id=self.boot, enabled_ns=self.clock_ns())
            atomic_json(self.root/ENABLE, value)
            reused = False
        # Clear a completed request, not JSONL or node-status files.
        try:
            (self.root/REQUEST).unlink()
        except FileNotFoundError:
            pass
        return dict(enabled=True, reused_marker=reused, marker=value,
                    note='仅启用诊断；必须先创建标记，再由操作者启动加载诊断代码的节点。不会 ARM 或启动节点。')

    def readiness(self, marker):
        result = {role: [] for role in ROLES}
        for path in self.root.glob('lite3_t22_ready_*.json'):
            try:
                row = read_json(path)
            except (OSError, ValueError):
                continue
            if (row.get('role') in result and row.get('boot_id') == self.boot
                    and row.get('enable_id') == marker['enable_id']):
                row['fresh'] = (type(row.get('heartbeat_ns')) is int and
                                0 <= self.clock_ns()-row['heartbeat_ns'] <= 5000000000)
                result[row['role']].append(row)
        return result

    def start(self, duration=45, lead_seconds=10, capture_id=None):
        if type(duration) is not int or not 30 <= duration <= 60:
            raise ValueError('duration must be an integer in [30, 60] seconds')
        if type(lead_seconds) is not int or not 3 <= lead_seconds <= 60:
            raise ValueError('lead time must be in [3, 60] seconds')
        marker = self.marker()
        self._not_in_progress()
        ready = self.readiness(marker)
        for role, rows in ready.items():
            fresh = [r for r in rows if r['fresh']]
            if (len(fresh) != 1 or fresh[0].get('state') not in ('waiting', 'finished')
                    or fresh[0].get('recorder_error')):
                raise ValueError('role '+role+' is not uniquely ready; enable before starting both instrumented nodes, then inspect status')
        issued = self.clock_ns()  # MUST be evaluated on the Jetson running this tool.
        name = capture_id or datetime.datetime.now().strftime('%Y%m%dT%H%M%S_')+uuid.uuid4().hex[:10]
        request = dict(schema_version=SCHEMA, capture_id=name, boot_id=self.boot,
                       enable_id=marker['enable_id'], issued_ns=issued,
                       start_ns=issued+lead_seconds*1000000000, duration_s=duration)
        window = validate_request(request, marker, self.boot)
        if any(self.root.glob('lite3_t22_'+name+'_*.jsonl')):
            raise ValueError('capture_id already has results; never overwrite or reuse it')
        for rows in ready.values():
            for row in rows:
                if row['fresh'] and issued <= row.get('probe_created_ns', issued):
                    raise ValueError('request must be newer than both probes')
        atomic_json(self.root/REQUEST, request)
        return dict(request=window, starts_in_seconds=lead_seconds,
                    ends_in_seconds=lead_seconds+duration, clock_host='this machine (run on Jetson)',
                    note='不调用 ROS；不检查或改变 ARM。操作者须先确认 DISARMED。')

    def status(self, capture_id=None):
        result = dict(boot_id=self.boot, now_ns=self.clock_ns(), enabled=False,
                      marker=None, request=None, phase='idle', errors=[], ready={}, results=[])
        try:
            marker = self.marker()
            result.update(enabled=True, marker=marker, ready=self.readiness(marker))
        except (OSError, ValueError) as error:
            result['errors'].append('enable: '+str(error))
        try:
            request = self._existing_request()
            if request:
                result['request'] = request
                result['phase'] = ('scheduled' if self.clock_ns() < request['start_ns'] else
                                   'recording' if self.clock_ns() < request['end_ns'] else 'window_ended')
        except (OSError, ValueError) as error:
            result['errors'].append('request: '+str(error))
        chosen = capture_id or (result['request'] or {}).get('capture_id')
        if chosen:
            import re
            if not re.fullmatch('[A-Za-z0-9_-]{1,48}', chosen):
                raise ValueError('invalid capture_id')
            for path in sorted(self.root.glob('lite3_t22_'+chosen+'_*.jsonl')):
                try:
                    result['results'].append(result_summary(path))
                except (OSError, ValueError) as error:
                    result['errors'].append(path.name+': '+str(error))
        by_role = {role: [r for r in result['results']
                         if (r.get('metadata') or {}).get('role') == role] for role in ROLES}
        identities = {tuple((r.get('metadata') or {}).get(k) for k in
                      ('capture_id', 'boot_id', 'enable_id', 'start_ns', 'duration_s'))
                      for r in result['results']}
        result['both_finished'] = all(len(rows) == 1 and rows[0]['finished']
            and not rows[0]['recorder_error'] for rows in by_role.values()) and len(identities) == 1
        if len(identities) > 1:
            result['errors'].append('result identity/window mismatch; not a complete pair')
        if identities and (any(v[0] != chosen for v in identities) or
                (result['request'] and chosen == result['request']['capture_id'] and
                 identities != {tuple(result['request'][k] for k in
                               ('capture_id', 'boot_id', 'enable_id', 'start_ns', 'duration_s'))})):
            result['errors'].append('result metadata differs from selected capture request')
            result['both_finished'] = False
        result['recorder_errors'] = [r.get('recorder_error') for rows in result['ready'].values()
                                     for r in rows if r.get('recorder_error')]
        return result

    def cleanup(self, force=False):
        if not force:
            self._not_in_progress(require_finished=True)
        removed = []
        for name in (ENABLE, REQUEST):
            try:
                (self.root/name).unlink()
                removed.append(name)
            except FileNotFoundError:
                pass
        return dict(removed=removed, forced=force, results_preserved=True,
                    note='只删除诊断控制文件；不删除 JSONL、不关闭业务节点。撤销将在 writer 下次轮询（通常 1 秒）生效。')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, epilog='须在 Jetson 终端执行；本工具不验证 DISARMED，操作者负责确认。')
    parser.add_argument('--root', default='/tmp', help='诊断目录；生产使用 /tmp，其他 /tmp 子目录仅用于隔离测试')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('enable-diagnostics', help='先启用诊断标记，然后人工启动节点；不 ARM、不启动节点')
    start = sub.add_parser('start', help='两端 ready 后创建统一未来窗口；不调用 ROS')
    start.add_argument('--duration', type=int, default=45, help='30～60 秒，默认 45')
    start.add_argument('--lead-seconds', type=int, default=10, help='预留 3～60 秒，默认 10')
    start.add_argument('--capture-id', help='通常自动生成；手工指定也必须唯一')
    status = sub.add_parser('status', help='读取控制文件、节点诊断状态及结果；不调用 ROS')
    status.add_argument('--capture-id', help='查看指定历史采样，覆盖当前请求的 ID')
    cleanup = sub.add_parser('cleanup', help='只删控制文件，保留全部 JSONL；未结束时默认拒绝')
    cleanup.add_argument('--force', action='store_true', help='明确中断诊断记录；不停止业务节点')
    help_command = sub.add_parser('help', help='显示总帮助或指定子命令帮助，不读取控制状态')
    help_command.add_argument('topic', nargs='?', choices=('enable-diagnostics','start','status','cleanup'))
    args = parser.parse_args(argv)
    if args.command == 'help':
        (sub.choices[args.topic] if args.topic else parser).print_help()
        return 0
    try:
        control = CaptureControl(args.root)
        if args.command == 'enable-diagnostics': result = control.enable()
        elif args.command == 'start': result = control.start(args.duration, args.lead_seconds, args.capture_id)
        elif args.command == 'status': result = control.status(args.capture_id)
        else: result = control.cleanup(args.force)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError) as error:
        print(json.dumps(dict(error=str(error)), ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
