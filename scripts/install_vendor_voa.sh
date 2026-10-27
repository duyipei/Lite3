#!/usr/bin/env bash
# Build an isolated copy on the existing Jetson; no services or motion started.
set -Eeuo pipefail
PROJECT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ROBOT=${ROBOT:-${LITE3_ROBOT_USER:-ysc}@${LITE3_ROBOT_HOST:-10.119.1.171}}
KEY=${SSH_KEY:-/home/duyipei/.ssh/id_ed25519_cyberdog}
REMOTE=/home/ysc/lite3_patrol_project
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -i "$KEY" "$ROBOT")
"${SSH[@]}" 'if pgrep -f "[c]ontroller_server|[r]os2 bag record|[p]atrol_runner.py|[v]oa_composition" >/dev/null; then echo "Stop active tasks before building VOA" >&2; exit 1; fi'
"${SSH[@]}" "mkdir -p '$REMOTE/tools' '$REMOTE/navigation' '$REMOTE/scripts'"
scp -o BatchMode=yes -o ConnectTimeout=8 -i "$KEY" \
  "$PROJECT/scripts/voa_camera_tf.py" "$PROJECT/scripts/check_vendor_voa.py" "$ROBOT:$REMOTE/scripts/"
scp -o BatchMode=yes -o ConnectTimeout=8 -i "$KEY" \
  "$PROJECT/tools/build_vendor_voa_overlay.py" "$ROBOT:$REMOTE/tools/"
scp -o BatchMode=yes -o ConnectTimeout=8 -i "$KEY" \
  "$PROJECT/navigation/start_vendor_voa.sh" "$PROJECT/navigation/vendor_voa.yaml" "$ROBOT:$REMOTE/navigation/"
"${SSH[@]}" 'bash -s' <<'REMOTE'
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
python3 "$PROJECT/tools/build_vendor_voa_overlay.py" --workspace "$PROJECT/voa_overlay_ws"
set +u
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/grid_map_foxy/install/setup.bash
set -u
cd "$PROJECT/voa_overlay_ws"
MAKEFLAGS=-j1 colcon build --executor sequential --packages-select voa --cmake-args -DCMAKE_BUILD_TYPE=Release
echo '[PASS] Isolated VOA built. No service/motion started. Shadow validation required.'
REMOTE
