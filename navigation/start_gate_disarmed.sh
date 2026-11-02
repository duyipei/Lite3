#!/usr/bin/env bash
# The gate always starts DISARMED. Arming is a separate explicit service call.
# LITE3_OBSTACLE_BACKEND selects the obstacle/speed-correction integration:
#   factory_only     vendor VOA perception -> Nav2; gate is the only cmd source
#   private_combined private VOA speed correction + LiDAR/camera costmaps
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
source "$PROJECT/navigation/patrol_env.sh"
BACKEND=${LITE3_OBSTACLE_BACKEND:-}
DYNAMIC=${LITE3_DYNAMIC_OBSTACLES:-0}
USE_VOA=${LITE3_USE_VENDOR_VOA:-0}
DYNAMIC_EXPLICIT=${LITE3_DYNAMIC_OBSTACLES:+1}
USE_VOA_EXPLICIT=${LITE3_USE_VENDOR_VOA:+1}
[[ "$DYNAMIC" == 0 || "$DYNAMIC" == 1 ]] || { echo 'Invalid LITE3_DYNAMIC_OBSTACLES' >&2; exit 1; }
[[ "$USE_VOA" == 0 || "$USE_VOA" == 1 ]] || { echo 'Invalid VOA mode' >&2; exit 1; }
COMMAND_WAIT=60.0
START_FACTORY=0
START_PRIVATE=0
REQUIRE_VOA_COSTMAP=false
REQUIRE_LIDAR_COSTMAP=false
COSTMAP_SYNC_ARGS=()
case "$BACKEND" in
  factory_only)
    # Factory Only is a dynamic-costmap mode by definition; only an explicit
    # request to disable dynamic obstacles is rejected.
    [[ "$DYNAMIC_EXPLICIT" != 1 || "$DYNAMIC" == 1 ]] || { \
      echo 'factory_only cannot run with LITE3_DYNAMIC_OBSTACLES=0' >&2; exit 1; }
    [[ "$USE_VOA_EXPLICIT" != 1 || "$USE_VOA" == 0 ]] || { \
      echo 'factory_only must not use private VOA speed correction' >&2; exit 1; }
    DYNAMIC=1
    USE_VOA=0
    REQUIRE_VOA_COSTMAP=true
    REQUIRE_LIDAR_COSTMAP=false
    START_FACTORY=1
    COSTMAP_SYNC_ARGS=(
      -p require_costmap_sync:=true
      -p safety_response_file:="$PROJECT/navigation/obstacle_response.yaml")
    ;;
  private_combined)
    [[ "$DYNAMIC_EXPLICIT" != 1 || "$DYNAMIC" == 1 ]] || { \
      echo 'private_combined cannot run with LITE3_DYNAMIC_OBSTACLES=0' >&2; exit 1; }
    [[ "$USE_VOA_EXPLICIT" != 1 || "$USE_VOA" == 1 ]] || { \
      echo 'private_combined requires LITE3_USE_VENDOR_VOA=1' >&2; exit 1; }
    DYNAMIC=1
    USE_VOA=1
    REQUIRE_VOA_COSTMAP=true
    REQUIRE_LIDAR_COSTMAP=true
    START_PRIVATE=1
    ;;
  '')
    # Legacy behavior: dynamic mode implies the private combined VOA path.
    if [[ "$DYNAMIC" == 1 ]]; then
      [[ "$USE_VOA" == 1 ]] || { echo 'Dynamic costmaps require VOA' >&2; exit 1; }
      REQUIRE_VOA_COSTMAP=true
      REQUIRE_LIDAR_COSTMAP=true
      START_PRIVATE=1
    fi
    ;;
  *)
    echo "Invalid LITE3_OBSTACLE_BACKEND: $BACKEND (factory_only|private_combined)" >&2
    exit 1
    ;;
esac
if [[ "$DYNAMIC" == 1 ]]; then
  COMMAND_WAIT=$(python3 "$PROJECT/scripts/navigation_recovery_budget.py" \
    "$PROJECT/navigation/navigate_dynamic_obstacles.xml")
fi
echo "[INFO] Nav2 zero-speed command-wait budget: ${COMMAND_WAIT}s"
echo "[INFO] obstacle backend: ${BACKEND:-legacy} (vendor_voa=$USE_VOA require_voa_costmap=$REQUIRE_VOA_COSTMAP require_lidar_costmap=$REQUIRE_LIDAR_COSTMAP)"
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
if [[ "$START_FACTORY" == 1 ]]; then
  # Fail before starting the gate if the factory environment or exclusivity
  # checks fail; no service is stopped or modified by --check.
  bash "$PROJECT/navigation/start_factory_voa.sh" --check
  setsid bash "$PROJECT/navigation/start_factory_voa.sh" &
  voa_pid=$!
elif [[ "$START_PRIVATE" == 1 ]]; then
  [[ -x "$PROJECT/voa_overlay_ws/install/voa/lib/voa/voa_composition" ]] || {
    echo '[FAIL] Build isolated VOA with scripts/install_vendor_voa.sh first.' >&2; exit 1;
  }
  setsid bash "$PROJECT/navigation/start_vendor_voa.sh" &
  voa_pid=$!
fi
setsid env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  ros2 run lite3_patrol_nav velocity_gate --ros-args \
  -p use_vendor_voa:=$([[ "$USE_VOA" == 1 ]] && echo true || echo false) \
  -p require_voa_costmap:="$REQUIRE_VOA_COSTMAP" \
  -p require_lidar_costmap:="$REQUIRE_LIDAR_COSTMAP" \
  -p output_topic:=/cmd_vel_corrected \
  -p require_localization_ready:=true \
  -p freshness_timeout_s:=1.5 \
  -p command_timeout_s:=0.5 \
  -p nav_command_wait_timeout_s:="$COMMAND_WAIT" \
  "${COSTMAP_SYNC_ARGS[@]}" \
  -p max_linear_x:=0.15 -p max_reverse_x:=0.08 \
  -p max_linear_y:=0.0 -p max_angular_z:=0.40 &
gate_pid=$!
# A perception supervisor that exits must never leave the gate looking
# healthy. The gate independently times out the stale obstacle heartbeat and
# latches DISARMED; keep it alive publishing zero and request a safe DISARM.
if [[ -n "$voa_pid" ]]; then
  while kill -0 "$gate_pid" 2>/dev/null; do
    if ! kill -0 "$voa_pid" 2>/dev/null; then
      status=0
      wait "$voa_pid" 2>/dev/null || status=$?
      voa_pid=
      echo "[FAIL] VOA perception supervisor exited (status $status); gate must latch on stale heartbeat" >&2
      ( set +u
        source "$PROJECT/navigation/patrol_env.sh" >/dev/null 2>&1
        timeout 5s ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: false}' \
          >/dev/null 2>&1 ) || true
      break
    fi
    sleep 0.5
  done
fi
wait "$gate_pid"
