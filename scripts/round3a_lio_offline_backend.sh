#!/usr/bin/env bash
# Isolated 1x Round 3A LIO-only playback on the Jetson. Never starts motion nodes.

set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
OVERLAY=$PROJECT/fast_livo2_ws/fast-livo2-deep-robotics/install/setup.bash
CONFIG=$PROJECT/config/lite3_mid360_lio_only.yaml
INPUT_DB=${1:-$PROJECT/results/round3a_lio_motion/lite3_campus_loop_001_lio.db3}
RUN_ID=${2:-run_001}
VOXEL_OVERRIDE=${3:-}
PLAYBACK_RATE=${4:-1.0}
SAVE_PCD=${5:-0}
OUT=$PROJECT/results/round3a_lio_motion/$RUN_ID
PCD_DIR=$PROJECT/fast_livo2_ws/fast-livo2-deep-robotics/src/FAST-LIVO2/Log/PCD

log() { printf '[Round3A-LIO] %s\n' "$*"; }
die() { printf '[Round3A-LIO][ERROR] %s\n' "$*" >&2; exit 1; }

[[ -f "$INPUT_DB" ]] || die "input database missing: $INPUT_DB"
[[ -f "$OVERLAY" ]] || die "workspace setup missing: $OVERLAY"
[[ -f "$CONFIG" ]] || die "configuration missing: $CONFIG"
[[ ! -e "$OUT" ]] || die "refusing to overwrite existing run: $OUT"
[[ "$PLAYBACK_RATE" =~ ^0[.][0-9]+$|^1([.]0+)?$ ]] || die "invalid playback rate: $PLAYBACK_RATE"
[[ "$SAVE_PCD" == 0 || "$SAVE_PCD" == 1 ]] || die "SAVE_PCD must be 0 or 1"
if [[ "$SAVE_PCD" == 1 ]] && find "$PCD_DIR" -maxdepth 1 -type f -name '*.pcd' -print -quit | grep -q .; then
  die "PCD output directory already contains a .pcd file; refusing to overwrite"
fi

# This is deliberately process-based rather than graph-based: no validation
# process from an earlier run may remain anywhere on the Jetson.
if pgrep -af 'fastlivo_mapping|ros2 bag play|mapping_avia' >/dev/null; then
  pgrep -af 'fastlivo_mapping|ros2 bag play|mapping_avia' >&2 || true
  die "an offline mapping/playback process is already running"
fi

mkdir -p "$OUT"
set +u
source /opt/ros/foxy/setup.bash
source "$OVERLAY"
set -u
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_COLORIZED_OUTPUT=0
unset CYCLONEDDS_URI
FASTLIVO_BIN=$(ros2 pkg prefix fast_livo)/lib/fast_livo/fastlivo_mapping
[[ -x "$FASTLIVO_BIN" ]] || die "FAST-LIVO2 executable missing: $FASTLIVO_BIN"

algorithm_pid=
counter_pid=
odom_pid=
monitor_pid=
tegrastats_pid=
player_pid=

stop_one() {
  local pid=${1:-}
  local attempts=${2:-20}
  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 "$attempts"); do
    kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }
    sleep 0.25
  done
  kill -TERM "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  stop_one "$player_pid"
  stop_one "$algorithm_pid"
  stop_one "$counter_pid"
  stop_one "$odom_pid"
  stop_one "$monitor_pid"
  stop_one "$tegrastats_pid"
}
trap cleanup EXIT INT TERM HUP

log "starting isolated output collectors on ROS_DOMAIN_ID=42"
python3 "$PROJECT/scripts/count_fast_livo_outputs.py" \
  "$OUT/output_counts.json" --duration 900 >"$OUT/output_counter.log" 2>&1 &
counter_pid=$!
python3 "$PROJECT/scripts/analyze_lio_odometry.py" \
  "$OUT/odometry_summary.json" --trajectory-csv "$OUT/trajectory.csv" \
  --duration 0 >"$OUT/odometry_analyzer.log" 2>&1 &
odom_pid=$!

log "starting LIO-only node; camera is disabled; RViz and PCD save are disabled"
algorithm_args=("$FASTLIVO_BIN" --ros-args --params-file "$CONFIG" --log-level warn)
if [[ -n "$VOXEL_OVERRIDE" ]]; then
  [[ "$VOXEL_OVERRIDE" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "invalid voxel override: $VOXEL_OVERRIDE"
  algorithm_args+=(-p "lio.voxel_size:=$VOXEL_OVERRIDE")
fi
if [[ "$SAVE_PCD" == 1 ]]; then
  algorithm_args+=(-p pcd_save.pcd_save_en:=true -p pcd_save.interval:=-1)
fi
printf 'input_db=%s\nconfig=%s\nvoxel_override=%s\nplayback_rate=%s\nsave_pcd=%s\n' \
  "$INPUT_DB" "$CONFIG" "${VOXEL_OVERRIDE:-NONE}" "$PLAYBACK_RATE" "$SAVE_PCD" \
  >"$OUT/run_manifest.txt"
"${algorithm_args[@]}" \
  >"$OUT/algorithm.log" 2>&1 &
algorithm_pid=$!
sleep 5
kill -0 "$algorithm_pid" 2>/dev/null || die "LIO process exited during startup; inspect $OUT/algorithm.log"

python3 "$PROJECT/scripts/monitor_pid.py" "$algorithm_pid" "$OUT/resources.csv" \
  --interval 1 --duration 900 >"$OUT/resource_monitor.log" 2>&1 &
monitor_pid=$!
tegrastats --interval 1000 >"$OUT/tegrastats.log" 2>&1 &
tegrastats_pid=$!

log "playing derived LiDAR+IMU bag at ${PLAYBACK_RATE}x"
started_epoch=$(date +%s)
(
  ros2 bag play "$INPUT_DB" --rate "$PLAYBACK_RATE" --read-ahead-queue-size 2000 \
    --topics /fast_livo2/livox/lidar /fast_livo2/livox/imu
) >"$OUT/bag_play.log" 2>&1 &
player_pid=$!
wait "$player_pid"
player_pid=
ended_epoch=$(date +%s)
printf 'playback_wall_s=%s\n' "$((ended_epoch-started_epoch))" >"$OUT/playback_wall.txt"

log "playback ended; observing 20s for unrecovered backlog"
sleep 20
stop_one "$algorithm_pid" 240; algorithm_pid=
stop_one "$counter_pid"; counter_pid=
stop_one "$odom_pid"; odom_pid=
stop_one "$monitor_pid"; monitor_pid=
stop_one "$tegrastats_pid"; tegrastats_pid=

python3 "$PROJECT/scripts/profile_fast_livo.py" \
  --log "$OUT/algorithm.log" --resources "$OUT/resources.csv" \
  --tegrastats "$OUT/tegrastats.log" --output "$OUT/profile.json" \
  >"$OUT/profile_stdout.log"

if [[ "$SAVE_PCD" == 1 && -f "$PCD_DIR/all_raw_points.pcd" ]]; then
  mv "$PCD_DIR/all_raw_points.pcd" "$OUT/map_raw.pcd"
fi

log "run complete: $OUT"
cat "$OUT/playback_wall.txt"
cat "$OUT/odometry_summary.json"
cat "$OUT/profile.json"

trap - EXIT INT TERM
cleanup
