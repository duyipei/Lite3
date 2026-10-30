#!/usr/bin/env bash
# Normal ORB initialization and production map parameters; no navigation or motion.
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
OUT=$PROJECT/results/local_ground_20261028/live
source "$PROJECT/navigation/patrol_env.sh"
mkdir -p "$OUT"
cat /proc/sys/kernel/random/boot_id > "$OUT/boot_id.txt"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid python3 "$PROJECT/scripts/voa_camera_tf.py" > "$OUT/camera_tf.log" 2>&1 & pids+=($!)
setsid env LITE3_PATROL_MAP_ID=outdoor_20260921_003_area_review LITE3_NDT_FINE_REFINE=false bash "$PROJECT/results/ground_acceptance_20261028/start_fine.sh" > "$OUT/localization.log" 2>&1 & pids+=($!)
setsid env LITE3_PATROL_MAP_ID=outdoor_20260921_003 bash "$PROJECT/navigation/start_visual_localizer.sh" > "$OUT/visual.log" 2>&1 & pids+=($!)
setsid env LITE3_PATROL_MAP_ID=outdoor_20260921_003_area_review LITE3_RELOCALIZATION_TEST_ONLY=1 bash "$PROJECT/navigation/start_visual_initialization.sh" > "$OUT/initializer.log" 2>&1 & pids+=($!)
printf '%s\n' "${pids[@]}" > "$OUT/process_groups.txt"
setsid env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 "$PROJECT/scripts/local_ground_shadow.py" --driver-config /home/ysc/lite_cog_ros2/driver/mid360_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json --output "$OUT/shadow.json" --seconds 200 > "$OUT/shadow.log" 2>&1 & pids+=($!)
python3 "$OUT/check_initialization.py" "$OUT/status.json" > "$OUT/status.log" 2>&1

python3 "$PROJECT/results/ground_acceptance_20261028/capture_geometry.py" "$OUT/geometry" > "$OUT/geometry.log" 2>&1
