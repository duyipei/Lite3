#!/usr/bin/env bash
# Run on Jetson: isolated VOA and private test traffic; no motor publisher.
set -Eeuo pipefail
PROJECT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE=${1:-$PROJECT/voa_overlay_ws}
[[ -x "$WORKSPACE/install/voa/lib/voa/voa_composition" ]] || exit 1
if pgrep -f '[v]oa_composition|[v]elocity_gate|[c]ontroller_server' >/dev/null; then
  echo '[FAIL] Stop other VOA/navigation processes before private shadow testing.' >&2
  exit 1
fi
source "$PROJECT/navigation/patrol_env.sh"
set +u
source "$WORKSPACE/install/setup.bash"
set -u
tf_pid=
voa_pid=
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "$voa_pid" "$tf_pid"; do
    [[ -z "$pid" ]] || kill -TERM "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
python3 "$PROJECT/scripts/voa_camera_tf.py" &
tf_pid=$!
"$WORKSPACE/install/voa/lib/voa/voa_composition" --ros-args \
  --params-file "$PROJECT/navigation/vendor_voa.yaml" \
  -r /grid_map:=/patrol/voa/grid_map -r /marker:=/patrol/voa/marker \
  -r /camera/depth/color/transformed_points:=/patrol/voa/transformed_points &
voa_pid=$!
sleep 5
kill -0 "$tf_pid" "$voa_pid"
python3 "$PROJECT/scripts/probe_voa_motion_shadow.py"
