#!/usr/bin/env bash
# Project-private factory VOA. Never publishes /cmd_vel or /cmd_vel_corrected.
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
source "$PROJECT/navigation/patrol_env.sh"
set +u
source "$PROJECT/voa_overlay_ws/install/setup.bash"
set -u
if systemctl is-active --quiet voa_ros2.service; then
  echo '[FAIL] Factory VOA service is active; no automatic service takeover.' >&2
  exit 1
fi
tf_pid=
voa_pid=
bridge_pid=
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "$bridge_pid" "$voa_pid" "$tf_pid"; do
    [[ -z "$pid" ]] || kill -TERM "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
python3 "$PROJECT/scripts/voa_camera_tf.py" &
tf_pid=$!
"$PROJECT/voa_overlay_ws/install/voa/lib/voa/voa_composition" --ros-args \
  --params-file "$PROJECT/navigation/vendor_voa.yaml" \
  -r /grid_map:=/patrol/voa/grid_map \
  -r /marker:=/patrol/voa/marker \
  -r /camera/depth/color/transformed_points:=/patrol/voa/transformed_points &
voa_pid=$!
children=("$tf_pid" "$voa_pid")
if [[ "${LITE3_DYNAMIC_OBSTACLES:-0}" == 1 ]]; then
  # Small rigid transforms do not benefit from BLAS worker threads. Limit
  # only this process, so it cannot starve sensor/TF callbacks under load.
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 "$PROJECT/scripts/voa_costmap_bridge.py" \
    --config "$PROJECT/navigation/vendor_voa.yaml" &
  bridge_pid=$!
  children+=("$bridge_pid")
fi
# Exit and stop the other child if either the transform guard or VOA fails.
wait -n "${children[@]}"
