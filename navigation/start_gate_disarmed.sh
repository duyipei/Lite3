#!/usr/bin/env bash
# The gate always starts DISARMED. Arming is a separate explicit service call.
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
source "$PROJECT/navigation/patrol_env.sh"
USE_VOA=${LITE3_USE_VENDOR_VOA:-0}
[[ "$USE_VOA" == 0 || "$USE_VOA" == 1 ]] || { echo 'Invalid VOA mode' >&2; exit 1; }
COMMAND_WAIT=60.0
REQUIRE_VOA_COSTMAP=false
if [[ "${LITE3_DYNAMIC_OBSTACLES:-0}" == 1 ]]; then
  [[ "$USE_VOA" == 1 ]] || { echo 'Dynamic costmaps require VOA' >&2; exit 1; }
  REQUIRE_VOA_COSTMAP=true
  COMMAND_WAIT=$(python3 "$PROJECT/scripts/navigation_recovery_budget.py" \
    "$PROJECT/navigation/navigate_dynamic_obstacles.xml")
fi
echo "[INFO] Nav2 zero-speed command-wait budget: ${COMMAND_WAIT}s"
voa_pid=
gate_pid=
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "$gate_pid" "$voa_pid"; do
    if [[ -n "$pid" ]]; then
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
if [[ "$USE_VOA" == 1 ]]; then
  [[ -x "$PROJECT/voa_overlay_ws/install/voa/lib/voa/voa_composition" ]] || {
    echo '[FAIL] Build isolated VOA with scripts/install_vendor_voa.sh first.' >&2; exit 1;
  }
  setsid bash "$PROJECT/navigation/start_vendor_voa.sh" &
  voa_pid=$!
fi
setsid ros2 run lite3_patrol_nav velocity_gate --ros-args \
  -p use_vendor_voa:=$([[ "$USE_VOA" == 1 ]] && echo true || echo false) \
  -p require_voa_costmap:="$REQUIRE_VOA_COSTMAP" \
  -p output_topic:=/cmd_vel_corrected \
  -p require_localization_ready:=true \
  -p freshness_timeout_s:=1.5 \
  -p command_timeout_s:=0.5 \
  -p nav_command_wait_timeout_s:="$COMMAND_WAIT" \
  -p max_linear_x:=0.15 -p max_reverse_x:=0.08 \
  -p max_linear_y:=0.0 -p max_angular_z:=0.40 &
gate_pid=$!
wait "$gate_pid"
