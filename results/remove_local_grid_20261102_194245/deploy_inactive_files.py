"""Install the prepared removal only while related project processes are absent.

This script does not import ROS, start/stop nodes, call ARM or publish commands.
"""
from pathlib import Path
import hashlib
import json
import os
import shlex
import stat
import subprocess
import tarfile
import tempfile

ROOT = Path('/home/ysc/lite3_patrol_project')
OUT = ROOT / 'results/remove_local_grid_20261102_194245'
PAYLOAD = Path('/tmp/lite3_remove_local_grid_20261102_194245.tar.gz')
WATCH = {'velocity_gate', 'goal_pose_bridge.py', 'check_avoidance_readiness.py',
         'start_gate_disarmed.sh', 'navigation_start.sh'}


def related_processes():
    found = []
    for line in subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True).splitlines():
        try:
            pid, command = line.strip().split(None, 1)
            names = {Path(x).name for x in shlex.split(command)}
        except ValueError:
            continue
        if names & WATCH:
            found.append(dict(pid=int(pid), command=command))
    return found


def require_inactive():
    active = related_processes()
    if active:
        raise SystemExit('No runtime files changed: related processes active: ' + json.dumps(active))


require_inactive()
OUT.mkdir(parents=True, exist_ok=True)
if (OUT / 'remote_before.tar.gz').exists():
    raise SystemExit('Backup already exists; refusing to overwrite it')
with tarfile.open(PAYLOAD, 'r:gz') as archive:
    expected = json.loads(archive.extractfile('payload_sha256.json').read())
    data = {}
    for name, digest in expected.items():
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise SystemExit('Unsafe payload path')
        contents = archive.extractfile(name).read()
        if hashlib.sha256(contents).hexdigest() != digest:
            raise SystemExit('Payload hash mismatch: ' + name)
        if relative.suffix == '.py':
            compile(contents, name, 'exec')
        elif relative.suffix == '.sh':
            subprocess.run(['bash', '-n'], input=contents, check=True)
        elif relative.suffix == '.yaml':
            import yaml
            if 'local_stop' in yaml.safe_load(contents):
                raise SystemExit('Retired local-stop settings remain')
        data[name] = contents

installed = ROOT / 'patrol_nav_ws/install_merged/lib/python3.8/site-packages/lite3_patrol_nav'
if not installed.is_dir():
    raise SystemExit('Missing installed package directory; no runtime files changed')
targets = {ROOT / name: contents for name, contents in data.items()}
for name, contents in data.items():
    if name.startswith('patrol_nav_ws/src/'):
        targets[installed / Path(name).name] = contents

resolved = {}
for logical, contents in targets.items():
    actual = logical.resolve()
    actual.relative_to(ROOT)
    if actual in resolved and resolved[actual] != contents:
        raise SystemExit('Conflicting aliased deployment targets')
    resolved[actual] = contents

require_inactive()
with tarfile.open(OUT / 'remote_before.tar.gz', 'w:gz') as archive:
    for logical in targets:
        if logical.exists() or logical.is_symlink():
            archive.add(logical, arcname=str(logical.relative_to(ROOT)))

# Prepare replacements before the final process check; no live destination
# changes until all payload syntax and content hashes have been verified.
prepared = []
try:
    for actual, contents in resolved.items():
        mode = stat.S_IMODE(actual.stat().st_mode) if actual.exists() else 0o644
        fd, temporary = tempfile.mkstemp(prefix='.' + actual.name + '.', dir=str(actual.parent))
        with os.fdopen(fd, 'wb') as stream:
            stream.write(contents)
        os.chmod(temporary, mode)
        prepared.append((Path(temporary), actual))
    require_inactive()
    for temporary, actual in prepared:
        os.replace(temporary, actual)
finally:
    for temporary, _actual in prepared:
        if temporary.exists():
            temporary.unlink()

verified = {}
for logical, contents in targets.items():
    digest = hashlib.sha256(logical.read_bytes()).hexdigest()
    if digest != hashlib.sha256(contents).hexdigest():
        raise SystemExit('Installed hash mismatch: ' + str(logical))
    verified[str(logical.relative_to(ROOT))] = digest
report = dict(files=verified, related_processes_before=[], syntax_only=True,
              tests_run=False, services_restarted=False, nodes_started=False,
              arm_or_motion_commands_sent=False)
(OUT / 'deployment_manifest.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, ensure_ascii=False, indent=2))
