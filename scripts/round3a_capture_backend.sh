#!/usr/bin/env bash
# Jetson-side backend for Lite3 Round 3A data collection.
# This script never publishes a motion command and never starts SLAM/Nav2/VOA.

set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
ROS_SETUP=/opt/ros/foxy/setup.bash
WS_SETUP="$PROJECT/fast_livo2_ws/fast-livo2-deep-robotics/install/setup.bash"
REALSENSE_SETUP=/home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash
DRIVER_LAUNCH="$PROJECT/scripts/mid360_isolated.launch.py"
TIME_PROBE="$PROJECT/scripts/time_probe.py"
LOG_DIR="$PROJECT/logs"
BAG_DIR="$PROJECT/bags"
STATE_DIR="$PROJECT/logs/round3a_state"
ACTIVE_FILE="$STATE_DIR/active_run"
LOCK_DIR=/tmp/lite3_round3a_capture.lock
# The recorder is stopped operationally before free space drops below 5 GB.
# Allow a short field run to start with 10 GB free while retaining that margin.
MIN_PROJECT_FREE_BYTES=10000000000
MIN_PREFLIGHT_SHM_BYTES=1000000000
SPOOL_SEGMENT_BYTES=536870912
CLEANUP_ON_EXIT=0
CLEANUP_RUN=''
MAPPING_CAMERA=${LITE3_MAPPING_CAMERA:-1}
[[ "$MAPPING_CAMERA" == 1 || "$MAPPING_CAMERA" == 2 ]] || { echo 'Invalid mapping camera' >&2; exit 2; }
EXTRA_CAMERA_TOPICS=()
if [[ "$MAPPING_CAMERA" == 2 ]]; then
  EXTRA_CAMERA_TOPICS=(/front_camera/image_raw/compressed /front_camera/camera_info)
fi

log() { printf '[Round3A] %s\n' "$*"; }
die() { printf '[Round3A][ERROR] %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
Jetson backend. Normally invoke it through the DEV-VM round3a_capture.sh.

Actions:
  preflight
  start RUN_ID
  status [RUN_ID]
  stop [RUN_ID]        # wait 25 s stationary, stop and verify
  stop-now [RUN_ID]    # no 25 s wait
  abort [RUN_ID]       # stop acquisition immediately; preserve partial bags
  verify RUN_ID
EOF
}

source_ros() {
  # ROS setup scripts are not compatible with nounset in Foxy.
  set +u
  source "$ROS_SETUP"
  if [[ -f "$WS_SETUP" ]]; then
    source "$WS_SETUP"
  fi
  if [[ -f "$REALSENSE_SETUP" ]]; then
    source "$REALSENSE_SETUP"
  fi
  set -u
}

validate_run() {
  local run=${1:-}
  [[ "$run" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$ ]] || \
    die "invalid RUN_ID: '$run' (use letters, numbers, '_' or '-')"
}

acquire_lock() {
  if ! mkdir "$LOCK_DIR" 2>/dev/null; then
    die "another Round3A command is running; wait and retry"
  fi
  trap cleanup_on_exit EXIT
}

cleanup_on_exit() {
  local rc=$?
  trap - EXIT
  if [[ "$CLEANUP_ON_EXIT" == 1 ]]; then
    CLEANUP_ON_EXIT=0
    log "start failed; stopping every acquisition process started by this tool"
    if [[ -n "$CLEANUP_RUN" ]]; then
      stop_capture_processes "$CLEANUP_RUN" || true
    fi
    stop_driver_and_probe || true
    log "partial files are preserved; if an active run remains, use stop-now before a new RUN_ID"
  fi
  rmdir "$LOCK_DIR" 2>/dev/null || true
  exit "$rc"
}

pid_cmdline() {
  local pid=$1
  tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true
}

pid_matches() {
  local pid=$1 pattern=$2
  kill -0 "$pid" 2>/dev/null || return 1
  pid_cmdline "$pid" | grep -Eq "$pattern"
}

group_has_processes() {
  local pgid=$1
  ps -eo pgid= | awk -v wanted="$pgid" '$1 == wanted {found=1} END {exit !found}'
}

start_group() {
  local pidfile=$1 logfile=$2 command=$3
  if [[ -f "$pidfile" ]]; then
    local old
    old=$(cat "$pidfile" 2>/dev/null || true)
    if [[ -n "$old" ]] && kill -0 "$old" 2>/dev/null; then
      die "process already active: PID $old ($pidfile)"
    fi
  fi
  setsid bash -lc "$command" </dev/null >"$logfile" 2>&1 &
  local pid=$!
  printf '%s\n' "$pid" > "$pidfile"
  log "started PID=$pid log=$logfile"
}

stop_group() {
  local pidfile=$1 pattern=$2 signal=${3:-INT} wait_seconds=${4:-15}
  [[ -f "$pidfile" ]] || return 0
  local pid pgid
  pid=$(cat "$pidfile" 2>/dev/null || true)
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  if ! pid_matches "$pid" "$pattern"; then
    rm -f "$pidfile"
    return 0
  fi
  pgid=$(ps -o pgid= -p "$pid" | tr -d ' ')
  [[ "$pgid" =~ ^[0-9]+$ ]] || die "cannot resolve process group for PID $pid"
  kill -"$signal" -- "-$pgid" 2>/dev/null || true
  for _ in $(seq 1 "$wait_seconds"); do
    if ! group_has_processes "$pgid"; then
      rm -f "$pidfile"
      return 0
    fi
    sleep 1
  done
  log "process group $pgid did not stop after $signal in ${wait_seconds}s; sending TERM"
  kill -TERM -- "-$pgid" 2>/dev/null || true
  sleep 3
  if group_has_processes "$pgid"; then
    log "WARNING: process group $pgid still exists; inspect manually"
    return 1
  fi
  rm -f "$pidfile"
}

publisher_count() {
  local topic=$1 output
  output=$(timeout 12s ros2 topic info "$topic" 2>/dev/null || true)
  awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"$output"
}

refresh_graph() {
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 5
}

check_forbidden_runtime() {
  local nodes processes
  nodes=$(timeout 15s ros2 node list 2>/dev/null || true)
  if grep -Eiq '(^|/)(bt_navigator|controller_server|planner_server|waypoint|nav2|voa|fastlivo)' <<<"$nodes"; then
    printf '%s\n' "$nodes" >&2
    die "forbidden navigation/VOA/LIO node detected"
  fi
  processes=$(ps -eo pid=,cmd=)
  if grep -Eiq '[n]av2|[b]t_navigator|[c]ontroller_server|[p]lanner_server|[w]aypoint|[v]oa|[f]astlivo_mapping' <<<"$processes"; then
    die "forbidden navigation/VOA/LIO process detected"
  fi
}

check_motion_publishers() {
  local topic count
  for topic in /cmd_vel /cmd_vel_corrected /simple_cmd /complex_cmd; do
    count=$(publisher_count "$topic")
    [[ "$count" == 0 ]] || die "$topic has $count publisher(s); acquisition refused"
  done
}

check_space() {
  local project_free shm_free
  project_free=$(df -B1 --output=avail "$PROJECT" | tail -1 | tr -d ' ')
  shm_free=$(df -B1 --output=avail /dev/shm | tail -1 | tr -d ' ')
  [[ "$project_free" -ge "$MIN_PROJECT_FREE_BYTES" ]] || \
    die "project storage has less than 10 GB free"
  [[ "$shm_free" -ge "$MIN_PREFLIGHT_SHM_BYTES" ]] || \
    die "/dev/shm has less than 1 GB free for the 18-second preflight only"
}

metric() {
  local directory=$1 topic=$2
  python3 - "$directory" "$topic" <<'PY'
import glob, os, sqlite3, sys
directory, topic = sys.argv[1:]
count = 0
first = None
last = None
for path in sorted(glob.glob(os.path.join(directory, '*.db3'))):
    con = sqlite3.connect('file:' + path + '?mode=ro', uri=True)
    row = con.execute(
        'SELECT COUNT(m.id), MIN(m.timestamp), MAX(m.timestamp) '
        'FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=?',
        (topic,)).fetchone()
    con.close()
    count += int(row[0] or 0)
    if row[1] is not None:
        first = row[1] if first is None else min(first, row[1])
        last = row[2] if last is None else max(last, row[2])
span = ((last - first) / 1e9) if first is not None and last is not None else 0.0
rate = ((count - 1) / span) if count > 1 and span > 0 else 0.0
print(f'{count} {span:.6f} {rate:.6f}')
PY
}

show_metric() {
  local directory=$1 topic=$2 count span rate
  read -r count span rate < <(metric "$directory" "$topic")
  printf '  %-38s count=%-8s span=%8.3fs rate=%8.3f Hz\n' "$topic" "$count" "$span" "$rate"
}

compare_coverage() {
  local first_dir=$1 second_dir=$2
  python3 - "$first_dir" "$second_dir" <<'PY'
import glob, os, sqlite3, sys
def coverage(directory):
    lo = hi = None
    for path in glob.glob(os.path.join(directory, '*.db3')):
        con = sqlite3.connect('file:' + path + '?mode=ro', uri=True)
        row = con.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
        con.close()
        if row[0] is not None:
            lo = row[0] if lo is None else min(lo, row[0])
            hi = row[1] if hi is None else max(hi, row[1])
    return lo, hi
a0, a1 = coverage(sys.argv[1])
b0, b1 = coverage(sys.argv[2])
if None in (a0, a1, b0, b1):
    print('  coverage unavailable')
    raise SystemExit(1)
overlap = max(0, min(a1, b1) - max(a0, b0)) / 1e9
shorter = min(a1 - a0, b1 - b0) / 1e9
ratio = overlap / shorter if shorter > 0 else 0
print(f'  bag overlap={overlap:.3f}s shorter_span={shorter:.3f}s ratio={ratio:.6f}')
raise SystemExit(0 if ratio >= 0.95 else 1)
PY
}

basic_rate_gate() {
  local lidar=$1 camera=$2 depth=$3 failed=0
  require_rate "$lidar" /fast_livo2/livox/lidar 9.5 10.6 || failed=1
  require_rate "$camera" /fast_livo2/livox/imu 190 210 || failed=1
  require_rate "$camera" /camera/color/image_raw 24 32 || failed=1
  require_rate "$camera" /camera/color/camera_info 28 32 || failed=1
  require_rate "$camera" /fast_livo2/time_probe 8.5 11.5 || failed=1
  require_rate "$depth" /camera/depth/image_rect_raw 10 32 || failed=1
  require_rate "$depth" /camera/depth/camera_info 10 32 || failed=1
  require_count "$depth" /camera/extrinsics/depth_to_color 1 || failed=1
  compare_coverage "$lidar" "$camera" || failed=1
  compare_coverage "$lidar" "$depth" || failed=1
  compare_coverage "$camera" "$depth" || failed=1
  return "$failed"
}

require_count() {
  local directory=$1 topic=$2 minimum=$3 count span rate
  read -r count span rate < <(metric "$directory" "$topic")
  printf '  %-38s count=%-8s span=%8.3fs rate=%8.3f Hz\n' "$topic" "$count" "$span" "$rate"
  [[ "$count" -ge "$minimum" ]]
}

require_rate() {
  local directory=$1 topic=$2 minimum=$3 maximum=$4 count span rate
  read -r count span rate < <(metric "$directory" "$topic")
  printf '  %-38s count=%-8s span=%8.3fs rate=%8.3f Hz\n' "$topic" "$count" "$span" "$rate"
  python3 - "$count" "$rate" "$minimum" "$maximum" <<'PY'
import sys
count, rate, low, high = map(float, sys.argv[1:])
raise SystemExit(0 if count > 1 and low <= rate <= high else 1)
PY
}

start_driver() {
  start_group "$STATE_DIR/driver.pid" "$LOG_DIR/round3a_livox_driver.log" \
    "source '$ROS_SETUP'; source '$WS_SETUP'; exec ros2 launch '$DRIVER_LAUNCH'"
}

start_probe() {
  start_group "$STATE_DIR/time_probe.pid" "$LOG_DIR/round3a_time_probe.log" \
    "source '$ROS_SETUP'; exec python3 '$TIME_PROBE'"
}

stop_driver_and_probe() {
  stop_group "$STATE_DIR/time_probe.pid" 'time_probe[.]py' INT || true
  stop_group "$STATE_DIR/driver.pid" 'ros2.*launch.*mid360_isolated[.]launch[.]py' INT || true
}

spool_copy_one() {
  local source=$1 destination_dir=$2 name target partial
  name=$(basename "$source")
  target="$destination_dir/$name"
  partial="$target.partial"
  mkdir -p "$destination_dir"
  if [[ -f "$target" ]]; then
    cmp -s "$source" "$target" || die "spool target differs from source: $target"
    rm -f "$source"
    return 0
  fi
  rm -f "$partial"
  cp -- "$source" "$partial"
  cmp -s "$source" "$partial" || die "spool copy verification failed: $source"
  mv -- "$partial" "$target"
  rm -f "$source"
}

action_spooler() {
  local run=${1:-} source_dir destination_dir
  validate_run "$run"
  source_dir="/dev/shm/${run}_sensors"
  destination_dir="$BAG_DIR/$run/sensors"
  mkdir -p "$destination_dir"
  while true; do
    local files=() index
    mapfile -t files < <(find "$source_dir" -maxdepth 1 -type f -name '*.db3' -print 2>/dev/null | sort -V)
    # rosbag2 is writing the newest file. Only transfer older, closed segments.
    if (( ${#files[@]} > 1 )); then
      for ((index=0; index<${#files[@]}-1; index++)); do
        spool_copy_one "${files[$index]}" "$destination_dir"
      done
    fi
    sleep 2
  done
}

finalize_spool() {
  local run=$1 source_dir destination_dir file
  source_dir="/dev/shm/${run}_sensors"
  destination_dir="$BAG_DIR/$run/sensors"
  [[ -d "$source_dir" ]] || return 0
  mkdir -p "$destination_dir"
  while IFS= read -r file; do
    spool_copy_one "$file" "$destination_dir"
  done < <(find "$source_dir" -maxdepth 1 -type f -name '*.db3' -print | sort -V)
  if [[ -f "$source_dir/metadata.yaml" ]]; then
    spool_copy_one "$source_dir/metadata.yaml" "$destination_dir"
  fi
  rmdir "$source_dir" 2>/dev/null || true
}

base_preflight() {
  source_ros
  refresh_graph
  check_motion_publishers
  check_forbidden_runtime
  check_space
  [[ -f "$DRIVER_LAUNCH" ]] || die "missing $DRIVER_LAUNCH"
  [[ -f "$TIME_PROBE" ]] || die "missing $TIME_PROBE"
  [[ -f "$WS_SETUP" ]] || die "missing $WS_SETUP"
  [[ -f "$REALSENSE_SETUP" ]] || die "missing $REALSENSE_SETUP"
  [[ $(publisher_count /camera/color/image_raw) -ge 1 ]] || die "RGB publisher not found"
  [[ $(publisher_count /camera/color/camera_info) -ge 1 ]] || die "CameraInfo publisher not found"
  [[ $(publisher_count /camera/depth/image_rect_raw) -ge 1 ]] || die "Depth publisher not found"
  [[ $(publisher_count /camera/depth/camera_info) -ge 1 ]] || die "Depth CameraInfo publisher not found"
  [[ $(publisher_count /camera/extrinsics/depth_to_color) -ge 1 ]] || \
    die "Depth-to-color extrinsics publisher not found"
  if [[ "$MAPPING_CAMERA" == 2 ]]; then
    [[ $(publisher_count /front_camera/image_raw/compressed) -ge 1 ]] || \
      die 'Camera2 bridge missing/outdated. Start updated navigation/start_front_camera.sh on Jetson first.'
    log 'Camera2 acquisition selected: compressed RGB plus original D435i/LiDAR. NOT a calibrated camera2 visual map.'
  fi
  log "base preflight PASS"
  date --iso-8601=ns
  df -h "$PROJECT"
}

run_short_preflight() {
  local run=$1 bag="/dev/shm/${run}_preflight"
  [[ ! -e "$bag" ]] || die "preflight directory already exists: $bag"
  refresh_graph
  for topic in /fast_livo2/livox/lidar /fast_livo2/livox/imu /fast_livo2/time_probe \
               /camera/color/image_raw /camera/color/camera_info \
               /camera/depth/image_rect_raw /camera/depth/camera_info \
               /camera/extrinsics/depth_to_color; do
    [[ $(publisher_count "$topic") -ge 1 ]] || die "publisher missing: $topic"
  done
  log "recording 18-second tmpfs preflight bag"
  set +e
  timeout -k 10s -s INT 18s ros2 bag record -o "$bag" \
    /fast_livo2/livox/lidar \
    /fast_livo2/livox/imu \
    /camera/color/image_raw \
    /camera/color/camera_info \
    /camera/depth/image_rect_raw \
    /camera/depth/camera_info \
    /camera/extrinsics/depth_to_color \
    /fast_livo2/time_probe \
    "${EXTRA_CAMERA_TOPICS[@]}" \
    >"$LOG_DIR/${run}_preflight_recorder.log" 2>&1
  local rc=$?
  set -e
  [[ "$rc" == 0 || "$rc" == 124 || "$rc" == 130 ]] || \
    die "preflight recorder failed with rc=$rc"
  [[ -f "$bag/metadata.yaml" ]] || die "preflight bag has no metadata.yaml"
  log "preflight topic metrics"
  local failed=0
  require_rate "$bag" /fast_livo2/livox/lidar 9.5 10.6 || failed=1
  require_rate "$bag" /fast_livo2/livox/imu 190 210 || failed=1
  # Allow modest RGB startup overhead in the short preflight. The finalized
  # bag still uses the stricter 27 Hz gate in basic_rate_gate().
  require_rate "$bag" /camera/color/image_raw 24 32 || failed=1
  require_rate "$bag" /camera/color/camera_info 28 32 || failed=1
  require_rate "$bag" /camera/depth/image_rect_raw 10 32 || failed=1
  require_rate "$bag" /camera/depth/camera_info 10 32 || failed=1
  require_count "$bag" /camera/extrinsics/depth_to_color 1 || failed=1
  require_rate "$bag" /fast_livo2/time_probe 8.5 11.5 || failed=1
  if [[ "$MAPPING_CAMERA" == 2 ]]; then
    require_rate "$bag" /front_camera/image_raw/compressed 2 12 || failed=1
  fi
  if [[ "$failed" != 0 ]]; then
    log "preflight bag retained for diagnosis: $bag"
    return 1
  fi
  rm -rf "$bag"
  log "sensor preflight PASS"
}

action_preflight() {
  acquire_lock
  base_preflight
}

action_start() {
  local run=${1:-}
  validate_run "$run"
  acquire_lock
  [[ ! -f "$ACTIVE_FILE" ]] || die "active run exists: $(cat "$ACTIVE_FILE"); use status/stop-now"
  [[ ! -e "$BAG_DIR/$run" ]] || die "run directory already exists: $BAG_DIR/$run"
  [[ ! -e "/dev/shm/${run}_preflight" ]] || die "preflight bag already exists: /dev/shm/${run}_preflight"
  [[ ! -e "/dev/shm/${run}_sensors" ]] || die "sensor spool already exists: /dev/shm/${run}_sensors"
  command -v setsid >/dev/null || die "setsid is required"
  base_preflight

  if ps -eo cmd= | grep -Eiq '[l]ivox_ros_driver2_node|[t]ime_probe[.]py|ros2 bag (record|play)'; then
    die "an acquisition/driver/bag process is already running; inspect it before start"
  fi
  CLEANUP_ON_EXIT=1
  CLEANUP_RUN="$run"

  start_driver
  start_probe
  sleep 9
  if ! run_short_preflight "$run"; then
    stop_driver_and_probe
    die "short bag gate FAIL; robot must remain stationary"
  fi

  log "restarting acquisition nodes after the temporary subscriber test"
  stop_driver_and_probe
  sleep 3
  start_driver
  start_probe
  sleep 9
  refresh_graph
  [[ $(publisher_count /fast_livo2/livox/lidar) -ge 1 ]] || die "LiDAR publisher missing after restart"
  [[ $(publisher_count /fast_livo2/livox/imu) -ge 1 ]] || die "IMU publisher missing after restart"
  [[ $(publisher_count /fast_livo2/time_probe) -ge 1 ]] || die "time probe publisher missing after restart"
  [[ $(publisher_count /camera/color/image_raw) -ge 1 ]] || die "RGB publisher missing after restart"
  [[ $(publisher_count /camera/depth/image_rect_raw) -ge 1 ]] || die "Depth publisher missing after restart"
  check_motion_publishers
  check_forbidden_runtime

  mkdir -p "$BAG_DIR/$run/sensors"
  printf 'camera_id: %s\ncamera2_visual_map_ready: false\ncamera2_stamp_source: reception_time\ncamera1_retained: true\n' "$MAPPING_CAMERA" > "$BAG_DIR/$run/camera_selection.yaml"
  printf '%s\n' "$run" > "$ACTIVE_FILE"

  start_group "$STATE_DIR/${run}_vmstat.pid" "$LOG_DIR/${run}_vmstat.log" \
    "exec vmstat 1"
  start_group "$STATE_DIR/${run}_tegrastats.pid" "$LOG_DIR/${run}_tegrastats.log" \
    "exec tegrastats --interval 1000"
  # Record to bounded 512 MiB tmpfs segments at full sensor rate. A separate
  # worker copies only closed segments sequentially to persistent storage.
  # This avoids Foxy SQLite random-write stalls without limiting route time.
  start_group "$STATE_DIR/${run}_sensor_recorder.pid" "$LOG_DIR/${run}_sensor_recorder.log" \
    "source '$ROS_SETUP'; source '$WS_SETUP'; source '$REALSENSE_SETUP'; exec ros2 bag record -o '/dev/shm/${run}_sensors' --max-bag-size '$SPOOL_SEGMENT_BYTES' /fast_livo2/livox/lidar /fast_livo2/livox/imu /camera/color/image_raw /camera/color/camera_info /camera/depth/image_rect_raw /camera/depth/camera_info /camera/extrinsics/depth_to_color /fast_livo2/time_probe ${EXTRA_CAMERA_TOPICS[*]}"
  start_group "$STATE_DIR/${run}_spooler.pid" "$LOG_DIR/${run}_spooler.log" \
    "exec '$PROJECT/scripts/round3a_capture_backend.sh' spooler '$run'"

  log "unified recorder started; holding the robot stationary for 25 seconds"
  sleep 25
  pid_matches "$(cat "$STATE_DIR/${run}_sensor_recorder.pid")" 'ros2.*bag.*record' || \
    die "unified sensor recorder stopped unexpectedly"
  pid_matches "$(cat "$STATE_DIR/${run}_spooler.pid")" 'round3a_capture_backend[.]sh spooler' || \
    die "persistent spool worker stopped unexpectedly"
  local sensor_bytes
  sensor_bytes=$(du -sb "/dev/shm/${run}_sensors" "$BAG_DIR/$run/sensors" 2>/dev/null | awk '{sum += $1} END {print sum}')
  [[ ${sensor_bytes:-0} -ge 100000000 ]] || die "unified sensor bag is not growing"
  check_motion_publishers
  log "GO: recording is healthy; on-site operator may now use the official controller"
  log "formal recording has no time limit; return to the start and run stop manually"
  log "monitor free space with status; stop before project storage has less than 5 GB free"
  log "Unified sensor bag bytes=$sensor_bytes"
  CLEANUP_ON_EXIT=0
}

active_run() {
  [[ -f "$ACTIVE_FILE" ]] || return 1
  cat "$ACTIVE_FILE"
}

resolve_run() {
  local requested=${1:-} current
  current=$(active_run 2>/dev/null || true)
  if [[ -n "$requested" ]]; then
    validate_run "$requested"
    if [[ -n "$current" && "$requested" != "$current" ]]; then
      die "requested run '$requested' differs from active run '$current'"
    fi
    printf '%s\n' "$requested"
  elif [[ -n "$current" ]]; then
    printf '%s\n' "$current"
  else
    die "no active run and no RUN_ID was provided"
  fi
}

action_status() {
  local run
  run=$(resolve_run "${1:-}")
  source_ros
  log "status for $run"
  date --iso-8601=ns
  for item in sensor_recorder spooler vmstat tegrastats; do
    local file="$STATE_DIR/${run}_${item}.pid" pid=''
    [[ -f "$file" ]] && pid=$(cat "$file" 2>/dev/null || true)
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      printf '  %-22s RUNNING pid=%s elapsed=%s\n' "$item" "$pid" "$(ps -o etime= -p "$pid" | tr -d ' ')"
    else
      printf '  %-22s STOPPED\n' "$item"
    fi
  done
  du -sh "/dev/shm/${run}_sensors" "$BAG_DIR/$run/sensors" 2>/dev/null || true
  df -h /dev/shm
  df -h "$PROJECT"
  check_motion_publishers
  check_forbidden_runtime
  log "safety gate still PASS"
}

stop_capture_processes() {
  local run=$1
  stop_group "$STATE_DIR/${run}_sensor_recorder.pid" 'ros2.*bag.*record' INT 30 || true
  stop_group "$STATE_DIR/${run}_spooler.pid" 'round3a_capture_backend[.]sh spooler' TERM 30 || true
  finalize_spool "$run"
  stop_group "$STATE_DIR/${run}_vmstat.pid" 'vmstat' TERM || true
  stop_group "$STATE_DIR/${run}_tegrastats.pid" 'tegrastats' TERM || true
}

action_stop() {
  local run delay=${2:-25} final_rc=0
  run=$(resolve_run "${1:-}")
  acquire_lock
  if [[ "$delay" -gt 0 ]]; then
    log "robot must already be physically stopped; collecting ${delay}s final stationary segment"
    sleep "$delay"
  fi
  log "stopping recorders"
  stop_capture_processes "$run"
  stop_driver_and_probe
  sync

  local sensors="$BAG_DIR/$run/sensors"
  if [[ ! -f "$sensors/metadata.yaml" ]]; then
    log "ERROR: unified sensor bag metadata missing: $sensors"
    final_rc=1
  fi

  if [[ -f "$sensors/metadata.yaml" ]]; then
    log "LiDAR metrics"
    show_metric "$sensors" /fast_livo2/livox/lidar || final_rc=1
    log "Camera/IMU metrics"
    show_metric "$sensors" /fast_livo2/livox/imu || final_rc=1
    show_metric "$sensors" /camera/color/image_raw || final_rc=1
    show_metric "$sensors" /camera/color/camera_info || final_rc=1
    show_metric "$sensors" /fast_livo2/time_probe || final_rc=1
    log "Depth metrics"
    show_metric "$sensors" /camera/depth/image_rect_raw || final_rc=1
    show_metric "$sensors" /camera/depth/camera_info || final_rc=1
    show_metric "$sensors" /camera/extrinsics/depth_to_color || final_rc=1
    log "basic rate and cross-bag coverage gate"
    basic_rate_gate "$sensors" "$sensors" "$sensors" || final_rc=1
    if [[ -f "$BAG_DIR/$run/camera_selection.yaml" ]] && grep -qx 'camera_id: 2' "$BAG_DIR/$run/camera_selection.yaml"; then
      require_rate "$sensors" /front_camera/image_raw/compressed 2 12 || final_rc=1
      log 'Camera2 capture only: not yet a calibrated visual localization database.'
    fi
  fi

  rm -f "$ACTIVE_FILE"
  source_ros
  refresh_graph
  check_motion_publishers || final_rc=1
  check_forbidden_runtime || final_rc=1
  if timeout 10s ros2 topic list 2>/dev/null | grep -q '^/fast_livo2/'; then
    log "ERROR: /fast_livo2 topics remain after cleanup"
    final_rc=1
  fi
  if [[ "$final_rc" == 0 ]]; then
    log "capture finalized; BASIC BAG STRUCTURE PASS"
    log "offline timestamp/gap/image/LIO analysis is still required"
  else
    log "capture stopped safely but finalization needs inspection"
  fi
  return "$final_rc"
}

action_abort() {
  local run
  run=$(resolve_run "${1:-}")
  acquire_lock
  log "immediate acquisition abort; no motion command is sent"
  stop_capture_processes "$run"
  stop_driver_and_probe
  rm -f "$ACTIVE_FILE"
  log "partial bags are preserved at $BAG_DIR/$run"
  log "robot motion must be stopped by the on-site operator with the official controller"
}

action_verify() {
  local run=${1:-}
  validate_run "$run"
  local sensors="$BAG_DIR/$run/sensors"
  local lidar="$BAG_DIR/$run/lidar" camera="$BAG_DIR/$run/camera_imu" depth="$BAG_DIR/$run/depth"
  if [[ -f "$sensors/metadata.yaml" ]]; then
    lidar=$sensors
    camera=$sensors
    depth=$sensors
  else
    [[ -f "$lidar/metadata.yaml" ]] || die "missing finalized LiDAR bag"
    [[ -f "$camera/metadata.yaml" ]] || die "missing finalized Camera/IMU bag"
    [[ -f "$depth/metadata.yaml" ]] || die "missing finalized Depth bag"
  fi
  log "LiDAR metrics"
  show_metric "$lidar" /fast_livo2/livox/lidar
  log "Camera/IMU metrics"
  show_metric "$camera" /fast_livo2/livox/imu
  show_metric "$camera" /camera/color/image_raw
  show_metric "$camera" /camera/color/camera_info
  show_metric "$camera" /fast_livo2/time_probe
  log "Depth metrics"
  show_metric "$depth" /camera/depth/image_rect_raw
  show_metric "$depth" /camera/depth/camera_info
  show_metric "$depth" /camera/extrinsics/depth_to_color
  log "basic rate and cross-bag coverage gate"
  basic_rate_gate "$lidar" "$camera" "$depth"
  if [[ -f "$BAG_DIR/$run/camera_selection.yaml" ]] && grep -qx 'camera_id: 2' "$BAG_DIR/$run/camera_selection.yaml"; then
    require_rate "$sensors" /front_camera/image_raw/compressed 2 12 || die 'Camera2 recording rate failed'
    log 'Camera2 images recorded; calibration, time alignment and camera2 3D feature map generation remain required.'
  fi
  du -sh "$BAG_DIR/$run"
  log "structure/count verification complete; offline quality analysis remains"
}

action=${1:-}
shift || true
if [[ "$action" == -h || "$action" == --help || "$action" == help || -z "$action" ]]; then
  usage
  exit 0
fi
mkdir -p "$LOG_DIR" "$BAG_DIR" "$STATE_DIR"
case "$action" in
  spooler) action_spooler "$@" ;;
  preflight) action_preflight "$@" ;;
  start) action_start "$@" ;;
  status) action_status "$@" ;;
  stop) action_stop "${1:-}" 25 ;;
  stop-now) action_stop "${1:-}" 0 ;;
  abort) action_abort "$@" ;;
  verify) action_verify "$@" ;;
  *) usage; die "unknown action: $action" ;;
esac
