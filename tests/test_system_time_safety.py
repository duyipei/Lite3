"""Offline clock evidence, jump supervision and reversible file installation."""
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard = load('clock_guard', 'scripts/check_system_time.py')
repair = load('clock_repair', 'tools/repair_jetson_time.py')


def good():
    return dict(wall=1794816000., monotonic=100., boot_id='boot-A',
                synchronized='yes', timesyncd_synchronized_marker=True,
                timezone='Asia/Shanghai', sample_clock_step_s=0.)


def test_trusted_clock_accepted():
    assert guard.validate(good())['synchronized'] == 'yes'


@pytest.mark.parametrize('state', ['no', 'unknown', '', 'true'])
def test_active_time_daemon_is_not_sync_evidence(state):
    s = good(); s['synchronized'] = state
    with pytest.raises(ValueError, match='NOT NTP synchronized'):
        guard.validate(s)


@pytest.mark.parametrize('wall', [0., 946684800., float('nan'), float('inf'), 4102444800.])
def test_bad_dates_rejected(wall):
    s = good(); s['wall'] = wall
    with pytest.raises(ValueError): guard.validate(s)


@pytest.mark.parametrize('key,value', [('timezone', 'UTC'), ('boot_id', ''),
                                      ('sample_clock_step_s', 1.), ('sample_clock_step_s', -1.),
                                      ('sample_clock_step_s', float('nan')),
                                      ('timesyncd_synchronized_marker', False)])
def test_preflight_clock_evidence_failure(key, value):
    s = good(); s[key] = value
    with pytest.raises(ValueError): guard.validate(s)


def test_monotonic_elapsed_and_small_slew_preserved():
    s = good()
    guard.check_offset(s, s['wall']+3600.01, 3700., 'boot-A')


@pytest.mark.parametrize('step', [-1382400., -1., 1., 1382400.])
def test_forward_and_backward_steps_refuse_reusing_tf_cache(step):
    s = good()
    with pytest.raises(ValueError, match='system clock moved'):
        guard.check_offset(s, s['wall']+5+step, 105., 'boot-A')


def test_previous_boot_baseline_not_accepted():
    with pytest.raises(ValueError, match='boot changed'):
        guard.check_offset(good(), good()['wall'], 100., 'boot-B')


def test_dbus_failure_never_reports_success(monkeypatch):
    def fail(*a, **k): raise OSError('DBus unavailable')
    monkeypatch.setattr(guard.subprocess, 'run', fail)
    with pytest.raises(OSError): guard.snapshot()


def test_watcher_returns_when_owned_process_exits(monkeypatch):
    monkeypatch.setattr(guard, 'process_alive', lambda pid: False)
    assert guard.watch(123, good()) == 0


def test_watcher_detects_clock_step_without_sending_signals(monkeypatch):
    monkeypatch.setattr(guard, 'process_alive', lambda pid: True)
    monkeypatch.setattr(guard, 'boot_id', lambda: 'boot-A')
    monkeypatch.setattr(guard.time, 'time', lambda: good()['wall']+10)
    monkeypatch.setattr(guard.time, 'monotonic', lambda: 100.)
    with pytest.raises(ValueError, match='system clock moved'): guard.watch(123, good())


def test_time_configuration_preserves_vendor_and_supports_245(tmp_path):
    root = tmp_path/'root'; backup = tmp_path/'backup'; backup.mkdir()
    vendor = root/'etc/systemd/timesyncd.conf.d/nv-fallback-ntp.conf'
    vendor.parent.mkdir(parents=True); vendor.write_text('[Time]\nFallbackNTP=vendor.example\n')
    old = vendor.read_bytes()
    manifest = repair.save_and_install(root, backup, repair.files(repair.SERVERS))
    assert vendor.read_bytes() == old
    generated=(root/repair.CONFIG.lstrip('/')).read_text()
    assert 'PollIntervalMaxSec=256' in generated
    # Old drop-ins must not keep unreachable IPv4 servers ahead of TUNA/Ubuntu.
    ntp_lines=[line for line in generated.splitlines() if line.startswith('NTP=')]
    assert ntp_lines[0]=='NTP=' and ntp_lines[1].startswith('NTP=ntp.tuna.tsinghua.edu.cn ntp.ubuntu.com ')
    assert 'SaveIntervalSec=' not in (root/repair.CONFIG.lstrip('/')).read_text()
    rtc = (root/repair.RTC_SCRIPT.lstrip('/')).read_text()
    assert 'NTPSynchronized' in rtc and '|| exit 0' in rtc
    assert '--systohc --utc' in rtc
    repair.restore_files(root, backup, manifest)
    assert vendor.read_bytes() == old
    assert not (root/repair.CONFIG.lstrip('/')).exists()


def test_existing_override_is_backed_up_and_restored(tmp_path):
    root = tmp_path/'root'; backup = tmp_path/'backup'; backup.mkdir()
    conf = root/repair.CONFIG.lstrip('/'); conf.parent.mkdir(parents=True)
    conf.write_text('[Time]\nNTP=campus.example\n'); before = conf.read_bytes()
    manifest = repair.save_and_install(root, backup, repair.files(repair.SERVERS))
    repair.restore_files(root, backup, manifest)
    assert conf.read_bytes() == before


@pytest.mark.parametrize('servers', [[], ['bad\nNTP=evil'], ['bad host'], ['--option']])
def test_server_injection_rejected(servers):
    with pytest.raises(ValueError): repair.configuration(servers)


def test_symlink_override_refused_before_writes(tmp_path):
    root = tmp_path/'root'; backup = tmp_path/'backup'; backup.mkdir()
    conf = root/repair.CONFIG.lstrip('/'); conf.parent.mkdir(parents=True)
    target = tmp_path/'original'; target.write_text('preserved')
    conf.symlink_to(target)
    with pytest.raises(ValueError): repair.save_and_install(root, backup, repair.files(repair.SERVERS))
    assert target.read_text() == 'preserved'


def test_busy_scan_ignores_shell_command_strings_and_detects_real_nodes(tmp_path):
    (tmp_path/'100').mkdir(); (tmp_path/'100/cmdline').write_bytes(b'bash\0-c\0echo hdl_localization_composition\0')
    (tmp_path/'101').mkdir(); (tmp_path/'101/cmdline').write_bytes(b'python3\0/project/scripts/voa_costmap_bridge.py\0')
    (tmp_path/'102').mkdir(); (tmp_path/'102/cmdline').write_bytes(b'/opt/nav2/controller_server\0--ros-args\0')
    assert {r['pid'] for r in repair.active_processes(tmp_path)} == {101, 102}


def test_install_requires_root_without_trying_passwords(monkeypatch):
    monkeypatch.setattr(repair.os, 'geteuid', lambda: 1000)
    with pytest.raises(ValueError, match='sudo'): repair.root_and_idle(True)


@pytest.mark.parametrize('name', ['start_localization_only.sh', 'start_visual_initialization.sh',
                                'start_manual_initialization.sh', 'start_nav_isolated.sh',
                                'start_gate_disarmed.sh'])
def test_project_startup_checks_clock_before_ros_environment_or_children(name):
    text = (ROOT/'navigation'/name).read_text()
    assert text.index('check_system_time.py" check') < text.index('source "$PROJECT/navigation/patrol_env.sh"')
    assert 'set -Eeuo pipefail' in text


def test_clock_jump_uses_existing_launcher_cleanup_not_robot_control_services():
    text = (ROOT/'navigation/start_localization_only.sh').read_text()
    assert text.index('localization_pid=$!') < text.index('check_system_time.py" watch') < text.index('wait "$localization_pid"', text.index('localization_pid=$!'))
    assert 'trap cleanup EXIT INT TERM HUP' in text
    source = (ROOT/'scripts/check_system_time.py').read_text()
    assert 'ros2' not in source and 'os.kill(' not in source and 'set-time' not in source


def test_partial_configuration_install_restores_original_files(tmp_path):
    root = tmp_path/'root'; backup = tmp_path/'backup'; backup.mkdir()
    a = root/'etc/a'; a.parent.mkdir(parents=True); a.write_text('before')
    broken = root/'etc/b.lite3-new'; broken.write_text('someone else owns this')
    with pytest.raises(ValueError):
        repair.save_and_install(root, backup, {'/etc/a':('after',0o644), '/etc/b':('new',0o644)})
    assert a.read_text() == 'before'
    assert not (root/'etc/b').exists()
    assert broken.read_text() == 'someone else owns this'


@pytest.mark.parametrize('name', ['start_gate_disarmed.sh','start_localization_only.sh',
                                'start_nav_isolated.sh','start_visual_initialization.sh',
                                'start_manual_initialization.sh'])
def test_untrusted_time_stops_real_launcher_before_any_ros_start(tmp_path,name):
    scripts=tmp_path/'scripts';scripts.mkdir()
    (scripts/'check_system_time.py').write_text('import sys\nprint("UNTRUSTED_TIME", file=sys.stderr)\nsys.exit(2)\n')
    # Deliberately provide no ROS environment or binaries. Reaching those
    # would fail for a different reason and violate the startup ordering.
    launch=tmp_path/name
    launch.write_text((ROOT/'navigation'/name).read_text().replace(
        'PROJECT=/home/ysc/lite3_patrol_project','PROJECT='+str(tmp_path)))
    p=subprocess.run(['bash',str(launch)],capture_output=True,text=True,timeout=3)
    assert p.returncode==2 and 'UNTRUSTED_TIME' in p.stderr
    assert 'patrol_env.sh' not in p.stderr
