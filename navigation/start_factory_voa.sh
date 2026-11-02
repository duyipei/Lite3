#!/usr/bin/env bash
# Factory VOA perception-only launcher (Factory Only mode).
#
# Starts the vendor voa_composition binary from the installed factory
# workspace and remaps every vendor command endpoint onto private, disabled
# topics. The vendor SafetyController therefore cannot publish /cmd_vel or
# /cmd_vel_corrected. The project Velocity Gate remains the only publisher of
# /cmd_vel_corrected.
#
# This deliberately runs the factory executable directly rather than through
# voa_ros2.service or the official UDP 0x21012109 start command. Running the
# binary directly is what makes ROS 2 remapping possible; the systemd/UDP
# control flow is bypassed. The service must be inactive first and this script
# never stops or reconfigures it.
#
# Direct execution also means the factory voa_launch.py is NOT started. Thus
# only scripts/voa_camera_tf.py supplies base_link -> camera_link; it reuses an
# existing matching transform and refuses a conflicting one, so no duplicate
# static camera mount is published.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
CHECK_ONLY=0
case "${1:-}" in
  --check) CHECK_ONLY=1 ;;
  '') ;;
  *) echo "usage: $0 [--check]" >&2; exit 2 ;;
esac

source "$PROJECT/navigation/patrol_env.sh"

fail() { echo "[FAIL] $*" >&2; exit 1; }

# The factory VOA depends on grid_map_msgs/grid_map_ros. Source the same
# dependency workspace used by the recorded factory build before looking for
# the vendor package. Missing optional paths are ignored here and reported by
# the package discovery below.
GRID_MAP_SETUP=${LITE3_GRID_MAP_SETUP:-/home/ysc/lite_cog_ros2/grid_map_foxy/install/setup.bash}
if [[ -r "$GRID_MAP_SETUP" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "$GRID_MAP_SETUP" >/dev/null 2>&1 || true
  set -u
fi

factory_setups() {
  if [[ -n "${LITE3_FACTORY_VOA_SETUP:-}" ]]; then
    printf '%s\n' "$LITE3_FACTORY_VOA_SETUP"
  fi
  if [[ -n "${LITE3_FACTORY_VOA_WS:-}" ]]; then
    printf '%s\n' "$LITE3_FACTORY_VOA_WS/install/setup.bash"
    printf '%s\n' "$LITE3_FACTORY_VOA_WS/install/local_setup.bash"
  fi
  # Isolated factory perception repair: timestamps and unknown mask only.
  # No vendor command-selection changes, and no original install replacement.
  if [[ -r "$PROJECT/factory_voa_safety_ws/install/setup.bash" ]]; then
    printf '%s\n' "$PROJECT/factory_voa_safety_ws/install/setup.bash"
  fi
  cat <<'EOF'
/home/ysc/lite_cog_ros2/voa/install/setup.bash
/home/ysc/lite_cog_ros2/voa/install/local_setup.bash
/home/ysc/lite_cog_ros2/voa_ws/install/setup.bash
/home/ysc/lite_cog_ros2/voa_ws/install/local_setup.bash
EOF
}

FACTORY_VOA_SETUP=
FACTORY_VOA_PREFIX=
FACTORY_VOA_EXECUTABLE=
discover_factory_voa() {
  local setup prefix executable last=
  while IFS= read -r setup; do
    [[ -n "$setup" && -r "$setup" ]] || continue
    [[ "$setup" != "$last" ]] || continue
    last=$setup
    case "$setup" in
      */voa_overlay_ws/*)
        echo "[INFO] ignoring private overlay setup: $setup" >&2
        continue
        ;;
    esac
    set +u
    # shellcheck source=/dev/null
    source "$setup" >/dev/null 2>&1 || { set -u; continue; }
    set -u
    prefix=$(ros2 pkg prefix voa 2>/dev/null) || continue
    executable=$prefix/lib/voa/voa_composition
    [[ -x "$executable" ]] || continue
    case "$executable" in
      */voa_overlay_ws/*)
        fail "ros2 pkg prefix voa resolved to the private overlay: $prefix"
        ;;
    esac
    FACTORY_VOA_SETUP=$setup
    FACTORY_VOA_PREFIX=$prefix
    FACTORY_VOA_EXECUTABLE=$executable
    return 0
  done < <(factory_setups)
  return 1
}

discover_factory_voa || fail \
  'factory voa_composition not found. Set LITE3_FACTORY_VOA_SETUP to the factory install/setup.bash.'

echo "[INFO] factory VOA package prefix: $FACTORY_VOA_PREFIX"
echo "[INFO] factory VOA setup: $FACTORY_VOA_SETUP"
echo "[INFO] factory VOA executable: $FACTORY_VOA_EXECUTABLE"
factory_launch=$FACTORY_VOA_PREFIX/share/voa/launch/voa_launch.py
if [[ -r "$factory_launch" ]]; then
  echo "[INFO] factory launch file: $factory_launch (NOT started by this script)"
  if grep -qE 'static_transform_publisher|StaticTransformBroadcaster' "$factory_launch"; then
    echo '[INFO] factory launch declares a static TF; voa_camera_tf.py reuses a matching existing TF or refuses a conflict'
  fi
fi

# Do not start a second producer while the official service is active. This is
# read-only: the unit file and service state are never modified.
if command -v systemctl >/dev/null 2>&1; then
  if systemctl is-active --quiet voa_ros2.service 2>/dev/null; then
    fail 'voa_ros2.service is active; refusing duplicate VOA (no automatic service takeover)'
  fi
  service_state=$(systemctl is-active voa_ros2.service 2>/dev/null || true)
  echo "[INFO] voa_ros2.service state: ${service_state:-unknown}"
  service_exec=$(systemctl cat voa_ros2.service 2>/dev/null | sed -n 's/^ExecStart=//p' | head -n 1 || true)
  if [[ -n "$service_exec" ]]; then
    echo "[INFO] voa_ros2.service ExecStart: $service_exec"
  fi
  service_env=$(systemctl show -p Environment --value voa_ros2.service 2>/dev/null || true)
  if [[ -n "$service_env" ]]; then
    echo "[INFO] voa_ros2.service Environment: $service_env"
  fi
fi
if pgrep -f '[v]oa_composition|[v]oa_launch.py' >/dev/null 2>&1; then
  fail 'another factory VOA process/launch is already running; refusing to start a duplicate'
fi
if pgrep -f '[v]oa_costmap_bridge.py|[v]oa_camera_tf.py|[s]tart_vendor_voa.sh' \
    >/dev/null 2>&1; then
  fail 'a private VOA bridge/TF is already running; stop it before factory-only mode'
fi

if [[ "$CHECK_ONLY" == 1 ]]; then
  echo '[PASS] factory VOA environment and exclusivity checks passed'
  exit 0
fi

tf_pid=
voa_pid=
bridge_pid=
cleanup() {
  trap - EXIT INT TERM HUP
  local pid
  for pid in "$bridge_pid" "$voa_pid" "$tf_pid"; do
    [[ -n "$pid" ]] || continue
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# Camera mount first, so factory point-cloud processing has a complete tree.
setsid python3 "$PROJECT/scripts/voa_camera_tf.py" &
tf_pid=$!

# Every vendor command endpoint is isolated. The vendor sees no project
# velocity and its corrected output is never consumed by the chassis bridge.
setsid "$FACTORY_VOA_EXECUTABLE" --ros-args \
  --params-file "$PROJECT/navigation/vendor_voa.yaml" \
  -r /grid_map:=/patrol/voa/grid_map \
  -r /marker:=/patrol/voa/marker \
  -r /cmd_vel:=/patrol/voa/disabled_cmd_vel \
  -r /handle_state:=/patrol/voa/disabled_handle_state \
  -r /cmd_vel_corrected:=/patrol/voa/unused_corrected \
  -r /camera/depth/color/transformed_points:=/patrol/voa/transformed_points \
  -r "/camera/depth/color/points:=${LITE3_FACTORY_VOA_DEPTH_TOPIC:-/camera/depth/color/points}" &
voa_pid=$!

# Keep BLAS worker threads out of the TF/sensor callbacks this bridge needs.
setsid env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 "$PROJECT/scripts/voa_costmap_bridge.py" \
  --config "$PROJECT/navigation/vendor_voa.yaml" --known-obstacles-only --publish-costmap-origin &
bridge_pid=$!

children=("$tf_pid" "$voa_pid" "$bridge_pid")
set +e
wait -n "${children[@]}"
status=$?
set -e
echo "[FAIL] factory VOA perception child exited (status $status); stopping all children" >&2
(( status != 0 )) || status=1
exit "$status"
