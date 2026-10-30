#!/usr/bin/env bash
# Normal ORB initialization and production map parameters; no navigation or motion.
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
OUT=$PROJECT/results/audit_repairs_20261028/static
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
setsid env LITE3_PATROL_MAP_ID=outdoor_20260921_003_area_review LITE3_LOCALIZATION_RUNTIME=audited LITE3_NDT_FINE_REFINE=false bash "$PROJECT/navigation/start_localization_only.sh" > "$OUT/localization.log" 2>&1 & pids+=($!)
setsid env LITE3_PATROL_MAP_ID=outdoor_20260921_003 bash "$PROJECT/navigation/start_visual_localizer.sh" > "$OUT/visual.log" 2>&1 & pids+=($!)
setsid env LITE3_PATROL_MAP_ID=outdoor_20260921_003_area_review LITE3_RELOCALIZATION_TEST_ONLY=1 bash "$PROJECT/navigation/start_visual_initialization.sh" > "$OUT/initializer.log" 2>&1 & pids+=($!)
printf '%s\n' "${pids[@]}" > "$OUT/process_groups.txt"
python3 "$OUT/check_initialization.py" "$OUT/status.json" > "$OUT/status.log" 2>&1

python3 - "$OUT/status.json" <<'CHECK'
import json,sys
if json.load(open(sys.argv[1]))['status']!='READY_HELD_15S':raise SystemExit('initialization failed; stopping private sensing session')
CHECK
touch "$OUT/initialization_passed"
# Remain stationary and keep real localization alive for the following
# perception/costmap checks. This supervisor never sends a motion command.
wait "${pids[1]}"
