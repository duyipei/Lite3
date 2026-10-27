#!/usr/bin/env bash
# Independent read-only camera bridge; does not start navigation or movement.
set -Eeuo pipefail
PROJECT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# Image transport needs the ROS base, not a built navigation workspace.
set +u
source /opt/ros/foxy/setup.bash
set -u
export ROS_LOCALHOST_ONLY=0
# Prevent another copy of this launcher from starting a duplicate publisher.
exec 9>"/tmp/lite3_front_camera_${UID}.lock"
flock -n 9 || { echo '[FAIL] Camera2 bridge already running; reuse its terminal.' >&2; exit 1; }
# Also recognize the earlier launcher, which did not take the lock.
if pgrep -f '[p]ython3 .*[/]rtsp_camera.py' >/dev/null; then
  echo '[FAIL] Existing RTSP bridge found. Reuse it; do not launch a duplicate.' >&2
  exit 1
fi
exec python3 "$PROJECT/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/rtsp_camera.py" "$@"
