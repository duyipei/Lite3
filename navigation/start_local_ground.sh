#!/usr/bin/env bash
# Local classification plus guarded map projection; never publishes velocity.
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
source "$PROJECT/navigation/patrol_env.sh"
DRIVER_CONFIG=/home/ysc/lite_cog_ros2/driver/mid360_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
mkdir -p "$PROJECT/logs"
OUT=$(mktemp -d "$PROJECT/logs/local_ground.XXXXXXXX")
children=()
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "${children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 "$PROJECT/scripts/local_ground_detector.py" --active --seconds 0 \
  --driver-config "$DRIVER_CONFIG" --output "$OUT/detector.json" &
children+=($!)
python3 "$PROJECT/scripts/local_ground_costmap_bridge.py" --active &
children+=($!)
wait -n "${children[@]}"
