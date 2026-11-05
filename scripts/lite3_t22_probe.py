"""Opt-in, bounded T2.2 diagnostics. Standard library only; no ROS control.

Adapted from the reviewed temporary recorder. A fresh enable generation and
a request issued AFTER this probe started are required. Only the writer
thread reads control files or writes JSONL. Callback recording never waits
for the recorder lock and never performs file I/O.
"""
from collections import Counter, deque
import itertools
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import time

SCHEMA = 1
ENABLE = 'lite3_t22_enable'
REQUEST = 'lite3_t22_start.json'
ROLES = ('leg', 'voa')
CAPACITY = 4096
MISS_EXAMPLES_PER_SECOND = 20


def stamp_ns(header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def read_json(path, limit=65536):
    if Path(path).is_symlink():
        raise ValueError('diagnostic files must not be symlinks')
    with Path(path).open() as handle:
        text = handle.read(limit+1)
    if len(text) > limit:
        raise ValueError('diagnostic control file is too large')
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError('expected a JSON object')
    return value


def atomic_json(path, value):
    """Atomic replacement of a diagnostic file, never a business file."""
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix='.'+path.name+'_', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as handle:
            json.dump(value, handle)
            handle.write('\n')
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def validate_marker(value, current_boot):
    if (type(value.get('schema_version')) is not int or value['schema_version'] != SCHEMA
            or value.get('boot_id') != current_boot
            or not isinstance(value.get('enable_id'), str)
            or not re.fullmatch('[a-f0-9]{32}', value['enable_id'])
            or type(value.get('enabled_ns')) is not int or value['enabled_ns'] <= 0):
        raise ValueError('invalid or previous-boot enable marker')
    return value


def validate_request(value, marker, current_boot, probe_created_ns=None):
    validate_marker(marker, current_boot)
    if (type(value.get('schema_version')) is not int or value['schema_version'] != SCHEMA
            or value.get('boot_id') != current_boot
            or value.get('enable_id') != marker['enable_id']
            or not isinstance(value.get('capture_id'), str)
            or not re.fullmatch('[A-Za-z0-9_-]{1,48}', value['capture_id'])
            or type(value.get('duration_s')) is not int or not 30 <= value['duration_s'] <= 60
            or type(value.get('issued_ns')) is not int or value['issued_ns'] <= 0
            or type(value.get('start_ns')) is not int
            or not 0 < value['start_ns']-value['issued_ns'] <= 60000000000):
        raise ValueError('invalid, stale, or different-generation capture request')
    if probe_created_ns is not None and value['issued_ns'] <= probe_created_ns:
        raise ValueError('request predates this probe; run start after both nodes are ready')
    return dict(value, end_ns=value['start_ns']+value['duration_s']*1000000000)


def create_probe(role, directory='/tmp'):
    """Absent/invalid marker means no probe and no background thread."""
    try:
        root = Path(directory).resolve()
        root.relative_to(Path('/tmp'))
        marker = validate_marker(read_json(root/ENABLE), boot_id())
        return Probe(role, root, marker=marker)
    except Exception:
        # Diagnostics must not prevent a production node from starting.
        return None


class Probe:
    def __init__(self, role, directory='/tmp', marker=None, current_boot=None,
                 clock_ns=None, start_writer=True):
        if role not in ROLES:
            raise ValueError('unknown diagnostic role')
        self.role, self.root = role, Path(directory).resolve()
        self.root.relative_to(Path('/tmp'))
        self.clock_ns = clock_ns or time.monotonic_ns
        self.boot = current_boot or boot_id()
        self.created_ns = self.clock_ns()
        self.marker = validate_marker(marker or read_json(self.root/ENABLE), self.boot)
        self._lock = threading.Lock()
        self._rows = deque(maxlen=CAPACITY)
        self._counts = Counter()
        # next(count) and its pickle snapshot are atomic in the supported CPython.
        # This counts lock-contention drops without ever waiting in a callback.
        self._drops = itertools.count()
        self._bookkeeping_cpu_ns = 0
        self._writer_cpu_ns = 0
        self._writer_cpu_base_ns = 0
        self._window = None
        self._active = False
        self._failed = False
        self._pending_error = None
        self._last_issued_ns = 0
        self._last_tick_ns = None
        self._last_leg_stamp_ns = None
        self._miss_budgets = {}
        self._handle = None
        self._written_rows = 0
        self._batch_index = 0
        self.state = 'waiting'
        self.error = None
        self.request_error = None
        self._stop = threading.Event()
        self._thread = None
        self.ready_path = self.root/('lite3_t22_ready_{}_{}.json'.format(role, os.getpid()))
        if start_writer:
            self._thread = threading.Thread(target=self._writer, name='t22-writer', daemon=True)
            self._thread.start()

    def active(self):
        try:
            window = self._window
            return bool(self._active and window and
                        window['start_ns'] <= self.clock_ns() < window['end_ns'])
        except Exception as error:
            self._fault(error)
            return False

    def _fault(self, error):
        self._active = False
        self._failed = True
        self._pending_error = '{}: {}'.format(type(error).__name__, str(error)[:300])

    def _drop_total(self):
        return self._drops.__reduce__()[1][0]

    def _record(self, kind, fields=None):
        if not self.active():
            return
        cpu = time.thread_time_ns()
        if not self._lock.acquire(False):
            next(self._drops)
            return
        try:
            if not self.active():
                return
            self._counts[kind] += 1
            if fields is not None:
                if len(self._rows) == CAPACITY:
                    next(self._drops)
                else:
                    self._rows.append(dict(kind=kind, tid=threading.get_native_id(), **fields))
            self._bookkeeping_cpu_ns += time.thread_time_ns()-cpu
        except Exception as error:
            self._fault(error)
        finally:
            self._lock.release()

    def count(self, kind):
        try:
            self._record(kind)
        except Exception as error:
            self._fault(error)

    def emit(self, kind, **fields):
        try:
            self._record(kind, fields)
        except Exception as error:
            self._fault(error)

    def tick(self, now_ns):
        try:
            previous, self._last_tick_ns = self._last_tick_ns, now_ns
            self.emit('leg_tick', t3a_ns=now_ns,
                      interval_ns=None if previous is None else now_ns-previous)
        except Exception as error:
            self._fault(error)

    def received(self, header):
        try:
            # An observed call return, NOT a claim about BufferCore acceptance.
            self._last_leg_stamp_ns = stamp_ns(header)
        except Exception as error:
            self._fault(error)

    def miss(self, stage, grid_header, requested_header, error):
        if not self.active():
            return
        try:
            now = self.clock_ns()
            text = str(error)
            direction = ('future' if 'future' in text.lower() else
                         'past' if 'past' in text.lower() else 'other')
            self.count('lookup.'+stage+'.'+type(error).__name__+'.'+direction)
            # Calls occur under the existing Bridge lock; no new business lock.
            second = now//1000000000
            previous_second, used = self._miss_budgets.get(stage, (None, 0))
            if second != previous_second:
                used = 0
            if used >= MISS_EXAMPLES_PER_SECOND:
                return
            self._miss_budgets[stage] = (second, used+1)
            self.emit('lookup_failure_example', stage=stage, at_ns=now,
                      grid_stamp_ns=stamp_ns(grid_header), grid_age_ns=now-stamp_ns(grid_header),
                      requested_stamp_ns=stamp_ns(requested_header),
                      last_set_returned_stamp_ns=self._last_leg_stamp_ns,
                      error_type=type(error).__name__, direction=direction, error=text[:512])
        except Exception as diagnostic_error:
            self._fault(diagnostic_error)

    def _status(self):
        atomic_json(self.ready_path, dict(schema_version=SCHEMA, role=self.role,
            pid=os.getpid(), boot_id=self.boot, enable_id=self.marker['enable_id'],
            probe_created_ns=self.created_ns, heartbeat_ns=self.clock_ns(), state=self.state,
            capture_id=self._window['capture_id'] if self._window else None,
            recorder_error=self.error, request_error=self.request_error))

    def _begin(self, request):
        now = self.clock_ns()
        window = validate_request(request, self.marker, self.boot, self.created_ns)
        if window['issued_ns'] <= self._last_issued_ns:
            return
        self._last_issued_ns = window['issued_ns']
        if now >= window['start_ns']:
            raise ValueError('missed start; create a new future capture request')
        path = self.root/('lite3_t22_{}_{}_{}.jsonl'.format(window['capture_id'], self.role, os.getpid()))
        handle = path.open('x')  # Never overwrite a previous capture.
        self._window = window
        self._handle = handle
        self._writer_cpu_base_ns = self._writer_cpu_ns
        with self._lock:
            self._rows.clear()
            self._counts.clear()
            self._drops = itertools.count()
            self._bookkeeping_cpu_ns = 0
            self._last_tick_ns = self._last_leg_stamp_ns = None
            self._miss_budgets.clear()
            self._written_rows = self._batch_index = 0
        handle.write(json.dumps(dict(kind='metadata', schema_version=SCHEMA, role=self.role,
            pid=os.getpid(), boot_id=self.boot, enable_id=self.marker['enable_id'],
            capture_id=window['capture_id'], start_ns=window['start_ns'],
            duration_s=window['duration_s'], end_ns=window['end_ns'], issued_ns=window['issued_ns'],
            probe_created_ns=self.created_ns, accepted_ns=now, clock='CLOCK_MONOTONIC ns',
            process_cpu_base_ns=time.process_time_ns(),
            capacity=CAPACITY, miss_examples_per_second=MISS_EXAMPLES_PER_SECOND,
            recorder_cpu_scope='bookkeeping lower bound + writer thread; process CPU includes business'))+'\n')
        handle.flush()
        self._active = True
        self.state = 'scheduled'
        self.request_error = None

    def _flush(self, finished=False, reason=None):
        if self._handle is None:
            return
        with self._lock:
            rows = list(self._rows)
            self._rows.clear()
            counts = dict(self._counts)
            drops, cpu = self._drop_total(), self._bookkeeping_cpu_ns
        # All JSON encoding and file I/O occurs outside the recorder lock.
        self._written_rows += len(rows)
        self._batch_index += 1
        self._handle.write(json.dumps(dict(kind='batch', batch_index=self._batch_index,
            at_ns=self.clock_ns(), finished=finished, stop_reason=reason, rows=rows,
            counts=counts, dropped_rows=drops, written_rows=self._written_rows,
            bookkeeping_thread_cpu_ns=cpu,
            writer_thread_cpu_ns=self._writer_cpu_ns-self._writer_cpu_base_ns,
            process_cpu_ns=time.process_time_ns()))+'\n')
        self._handle.flush()
        if finished or reason:
            self._handle.close()
            self._handle = None

    def poll_once(self):
        """Writer-thread operation; also supports deterministic offline tests."""
        try:
            if self._failed:
                raise RuntimeError(self._pending_error or 'recorder failed')
            try:
                marker = validate_marker(read_json(self.root/ENABLE), self.boot)
                enabled = marker['enable_id'] == self.marker['enable_id']
            except (OSError, ValueError):
                enabled = False
            if not enabled or self._stop.is_set():
                self._active = False
                self._flush(reason='disabled_or_closed')
                self.state = 'disabled'
                self._status()
                return False
            if self._handle is not None:
                finished = self.clock_ns() >= self._window['end_ns']
                if finished:
                    self._active = False
                self.state = ('finished' if finished else 'recording' if self.active() else 'scheduled')
                self._flush(finished=finished)
            else:
                try:
                    request = read_json(self.root/REQUEST)
                    if request.get('issued_ns', 0) != self._last_issued_ns:
                        self._begin(request)
                except FileNotFoundError:
                    pass
                except ValueError as error:
                    self.request_error = str(error)
            self._status()
            return True
        except Exception as error:
            self._active = False
            self._failed = True
            self.error = '{}: {}'.format(type(error).__name__, str(error)[:300])
            self.state = 'error'
            try:
                if self._handle is not None:
                    self._handle.write(json.dumps(dict(kind='recorder_error', error=self.error))+'\n')
                    self._flush(reason='recorder_error')
            except Exception:
                pass
            try:
                self._status()
            except Exception:
                pass
            return False

    def _writer(self):
        try:
            while True:
                cpu = time.thread_time_ns()
                keep_running = self.poll_once()
                self._writer_cpu_ns += time.thread_time_ns()-cpu
                if not keep_running:
                    return
                self._stop.wait(1.0)
        finally:
            self._active = False
            if self._handle is not None:
                try:
                    self._handle.close()
                except Exception:
                    pass

    def close(self):
        # No join or file operation in a business thread.
        self._active = False
        self._stop.set()


def receiver_types(Buffer, TransformListener, probe):
    """Wrap only this listener instance via local subclasses, never monkey patch TF2."""
    context = threading.local()

    class TimedBuffer(Buffer):
        def set_transform(self, transform, authority):
            if (not probe.active() or transform.header.frame_id != 'robot_odom'
                    or transform.child_frame_id != 'leg_base_link'):
                return super().set_transform(transform, authority)
            t4b = time.monotonic_ns()
            outcome = 'returned_no_exception'
            try:
                return super().set_transform(transform, authority)
            except Exception as error:
                outcome = type(error).__name__
                raise
            finally:
                t4c = time.monotonic_ns()
                if outcome == 'returned_no_exception':
                    probe.received(transform.header)
                probe.emit('leg_rx', source_stamp_ns=stamp_ns(transform.header),
                    parent=transform.header.frame_id, child=transform.child_frame_id,
                    t4a_ns=getattr(context, 'entered_ns', None), t4b_ns=t4b, t4c_ns=t4c,
                    batch_tf_count=getattr(context, 'batch_tf_count', None), outcome=outcome,
                    queryability_checked=False)

    class TimedListener(TransformListener):
        def callback(self, data):
            if not probe.active():
                return super().callback(data)
            entered = time.monotonic_ns()
            if not any(t.header.frame_id == 'robot_odom' and t.child_frame_id == 'leg_base_link'
                       for t in data.transforms):
                return super().callback(data)
            previous = (getattr(context, 'entered_ns', None), getattr(context, 'batch_tf_count', None))
            context.entered_ns, context.batch_tf_count = entered, len(data.transforms)
            try:
                return super().callback(data)
            finally:
                probe.emit('listener_callback', t4a_ns=entered,
                    returned_ns=time.monotonic_ns(), batch_tf_count=len(data.transforms))
                context.entered_ns, context.batch_tf_count = previous

    return TimedBuffer, TimedListener
