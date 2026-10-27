#!/usr/bin/env bash
# Narrow offline deployment; no ROS startup or motion commands.
set -Eeuo pipefail
PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ROBOT=${ROBOT:-ysc@${LITE3_ROBOT_HOST:-10.119.1.171}}
KEY=${SSH_KEY:-/home/duyipei/.ssh/id_ed25519_cyberdog}
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 -i "$KEY" "$ROBOT")
BUNDLE=$(mktemp /tmp/lite3-xfeat-deploy.XXXXXXXX.tar.gz)
trap 'rm -f -- "$BUNDLE"' EXIT
python3 - "$PROJECT" "$BUNDLE" <<'PY'
import json,sys,tarfile
from pathlib import Path
root=Path(sys.argv[1]); entries=[]
package='patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/'
entries += [package+n+'.py' for n in ['rgbd_visual_localizer','xfeat_matching','visual_matching','learned_matching']]
entries += ['navigation/start_visual_localizer.sh','tools/install_xfeat_runtime.sh','tools/check_xfeat_runtime.py']
for folder,manifest in [('accelerated_features','runtime_manifest.json'),('xfeat_wheels','manifest.json')]:
 prefix='third_party/'+folder+'/'
 entries += [prefix+manifest]+[prefix+n for n in json.loads((root/prefix/manifest).read_text())['sha256']]
entries += ['third_party/accelerated_features/LICENSE']
with tarfile.open(sys.argv[2],'w:gz') as t:
 for name in entries: t.add(root/name,arcname=name,recursive=False)
PY
"${SSH[@]}" 'test -d /home/ysc/lite3_patrol_project'
REMOTE_BUNDLE=$("${SSH[@]}" 'mktemp /tmp/lite3-xfeat-deploy.XXXXXXXX.tar.gz')
scp -o BatchMode=yes -o ConnectTimeout=10 -i "$KEY" "$BUNDLE" "$ROBOT:$REMOTE_BUNDLE"
"${SSH[@]}" "bash -s -- '$REMOTE_BUNDLE'" <<'REMOTE'
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
BUNDLE=$1
python3 - "$PROJECT" "$BUNDLE" <<'PY'
import datetime,hashlib,json,os,shutil,subprocess,sys,tarfile,tempfile
from pathlib import Path
root=Path(sys.argv[1]); archive=Path(sys.argv[2])
# Do not change code underneath a live localization/navigation task.
active=subprocess.check_output(['ps','-eo','comm,args'],text=True)
for line in active.splitlines():
 if line.split(maxsplit=1)[0] in ('bash','ssh','sshd','sh'): continue
 if any(token in line for token in ['rgbd_visual_localizer','controller_server','patrol_runner']):
  raise RuntimeError('active navigation/localization task: '+line)
backup=root/'logs'/('xfeat_deployment_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
backup.mkdir(parents=True,exist_ok=False)
with tarfile.open(archive) as tar:
 members=tar.getmembers()
 for m in members:
  if not m.isfile() or Path(m.name).is_absolute() or '..' in Path(m.name).parts:
   raise RuntimeError('unexpected bundle entry '+m.name)
 with tempfile.TemporaryDirectory(prefix='lite3-xfeat-stage-') as tmp:
  tar.extractall(tmp)
  stage=Path(tmp)
  # Install isolated dependencies before replacing the running entrypoint.
  subprocess.run(['bash',str(stage/'tools/install_xfeat_runtime.sh')],check=True)
  depdst=root/'third_party/xfeat_deps'
  if depdst.exists(): shutil.copytree(depdst,backup/'previous_xfeat_deps')
  shutil.copytree(stage/'third_party/xfeat_deps',depdst,dirs_exist_ok=True)
  for m in members:
   dst=root/m.name
   if dst.exists():
    old=backup/m.name; old.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(dst,old)
   dst.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(stage/m.name,dst)
  package=Path('patrol_nav_ws/install_merged/lib/python3.8/site-packages/lite3_patrol_nav')
  for name in ['rgbd_visual_localizer','xfeat_matching','visual_matching','learned_matching']:
   rel=package/(name+'.py'); dst=root/rel
   if dst.exists():
    old=backup/rel; old.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(dst,old)
   shutil.copy2(root/'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav'/(name+'.py'),dst)
  (backup/'deployment.json').write_text(json.dumps({'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),
       'files':[m.name for m in members],'note':'ORB remains default. No ROS tasks started.'},indent=2)+'\n')
print('BACKUP='+str(backup))
PY
python3 "$PROJECT/tools/check_xfeat_runtime.py" --report "$PROJECT/logs/xfeat_runtime_check.json"
REMOTE
