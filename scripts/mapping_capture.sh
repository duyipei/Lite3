#!/usr/bin/env bash
# Shared DEV-VM acquisition/deployment entry for indoor and outdoor mapping.

set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BACKEND_LOCAL="$SCRIPT_DIR/round3a_capture_backend.sh"
MERGE_LOCAL="$SCRIPT_DIR/merge_round3a_lio_bag.py"
OFFLINE_LOCAL="$SCRIPT_DIR/round3a_lio_offline_backend.sh"
ODOM_LOCAL="$SCRIPT_DIR/analyze_lio_odometry.py"
DRIVER_LOCAL="$SCRIPT_DIR/mid360_isolated.launch.py"
TIME_PROBE_LOCAL="$SCRIPT_DIR/time_probe.py"
COUNTER_LOCAL="$SCRIPT_DIR/count_fast_livo_outputs.py"
MONITOR_LOCAL="$SCRIPT_DIR/monitor_pid.py"
PROFILE_LOCAL="$SCRIPT_DIR/profile_fast_livo.py"
GUIDE_LOCAL="$SCRIPT_DIR/../docs/2026-09-12_indoor_lidar_rgbd_mapping.md"
VIDEO_VIEWER_LOCAL="$SCRIPT_DIR/camera_preview.py"
SSH_KEY=${LITE3_SSH_KEY:-$HOME/.ssh/id_ed25519_cyberdog}
ROBOT_USER=${LITE3_ROBOT_USER:-ysc}
ROBOT_HOST=${LITE3_ROBOT_HOST:-10.119.1.171}
ROBOT="$ROBOT_USER@$ROBOT_HOST"
BACKEND_REMOTE=/home/ysc/lite3_patrol_project/scripts/round3a_capture_backend.sh
MERGE_REMOTE=/home/ysc/lite3_patrol_project/scripts/merge_round3a_lio_bag.py
OFFLINE_REMOTE=/home/ysc/lite3_patrol_project/scripts/round3a_lio_offline_backend.sh
ODOM_REMOTE=/home/ysc/lite3_patrol_project/scripts/analyze_lio_odometry.py
GUIDE_REMOTE=/home/ysc/lite3_patrol_project/docs/2026-09-12_indoor_lidar_rgbd_mapping.md
SYNC_FILES=(
  "$BACKEND_LOCAL"
  "$MERGE_LOCAL"
  "$OFFLINE_LOCAL"
  "$ODOM_LOCAL"
  "$DRIVER_LOCAL"
  "$TIME_PROBE_LOCAL"
  "$COUNTER_LOCAL"
  "$MONITOR_LOCAL"
  "$PROFILE_LOCAL"
)

log() { printf '[Round3A-VM] %s\n' "$*"; }
die() { printf '[Round3A-VM][ERROR] %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
Lite3 shared mapping acquisition tool (run on DEV-VM).

First use / after script update:
  ./mapping_capture.sh install

Normal field workflow:
  ./mapping_capture.sh preflight
  ./mapping_capture.sh start lite3_campus_loop_001
  # Wait for GO: LiDAR, IMU, RGB and depth then record to persistent storage
  # with no fixed time limit. The operator drives with the official controller.
  ./mapping_capture.sh status
  ./mapping_capture.sh view-video
  ./mapping_capture.sh camera2-start  # separate terminal; Ctrl+C stops bridge
  # Monitor free storage, return to the start, stop the robot, then:
  ./mapping_capture.sh stop
  ./mapping_capture.sh verify lite3_campus_loop_001

Emergency acquisition cleanup (does not stop physical robot motion):
  ./mapping_capture.sh abort

Environment overrides:
  LITE3_ROBOT_HOST, LITE3_ROBOT_USER, LITE3_SSH_KEY,
  LITE3_VIEW_RMW, LITE3_ROS_DOMAIN_ID
EOF
}

ssh_base=(ssh -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" "$ROBOT")

check_local() {
  local file
  [[ -f "$SSH_KEY" ]] || die "SSH key not found: $SSH_KEY"
  for file in "${SYNC_FILES[@]}"; do
    [[ -f "$file" ]] || die "required mapping file not found: $file"
  done
  [[ -f "$GUIDE_LOCAL" ]] || die "mapping guide not found: $GUIDE_LOCAL"
  "${ssh_base[@]}" 'true' || die "cannot reach/authenticate Jetson at $ROBOT"
}

install_backend() {
  check_local
  scp -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "${SYNC_FILES[@]}" \
    "$ROBOT:/home/ysc/lite3_patrol_project/scripts/"
  scp -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$GUIDE_LOCAL" "$ROBOT:$GUIDE_REMOTE"
  "${ssh_base[@]}" \
    "chmod 0755 '$BACKEND_REMOTE' '$OFFLINE_REMOTE' '$MERGE_REMOTE' '$ODOM_REMOTE' /home/ysc/lite3_patrol_project/scripts/count_fast_livo_outputs.py /home/ysc/lite3_patrol_project/scripts/mid360_isolated.launch.py /home/ysc/lite3_patrol_project/scripts/monitor_pid.py /home/ysc/lite3_patrol_project/scripts/time_probe.py /home/ysc/lite3_patrol_project/scripts/profile_fast_livo.py && bash -n '$BACKEND_REMOTE' '$OFFLINE_REMOTE' && python3 -m py_compile '$MERGE_REMOTE' '$ODOM_REMOTE' /home/ysc/lite3_patrol_project/scripts/count_fast_livo_outputs.py /home/ysc/lite3_patrol_project/scripts/mid360_isolated.launch.py /home/ysc/lite3_patrol_project/scripts/monitor_pid.py /home/ysc/lite3_patrol_project/scripts/time_probe.py /home/ysc/lite3_patrol_project/scripts/profile_fast_livo.py"
  log "capture and offline mapping runtime installed on $ROBOT"
}

ensure_backend() {
  local file local_sum remote_sum local_guide_sum remote_guide_sum needs_install=0
  check_local
  for file in "${SYNC_FILES[@]}"; do
    local_sum=$(sha256sum "$file" | awk '{print $1}')
    remote_sum=$("${ssh_base[@]}" \
      "sha256sum '/home/ysc/lite3_patrol_project/scripts/$(basename "$file")' 2>/dev/null | awk '{print \$1}'" \
      || true)
    [[ "$local_sum" == "$remote_sum" ]] || needs_install=1
  done
  local_guide_sum=$(sha256sum "$GUIDE_LOCAL" | awk '{print $1}')
  remote_guide_sum=$("${ssh_base[@]}" "sha256sum '$GUIDE_REMOTE' 2>/dev/null | awk '{print \$1}'" || true)
  if (( needs_install == 1 )) || [[ "$local_guide_sum" != "$remote_guide_sum" ]]; then
    log "installing/updating Jetson backend"
    install_backend
  fi
}

ensure_clock() {
  local synced vm_epoch robot_epoch delta abs_delta answer iso
  synced=$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)
  [[ "$synced" == yes ]] || die "DEV-VM NTP is not synchronized; fix VM time first"
  vm_epoch=$(date +%s)
  robot_epoch=$("${ssh_base[@]}" 'date +%s')
  delta=$((vm_epoch - robot_epoch))
  abs_delta=${delta#-}
  log "VM-Jetson clock delta: ${delta}s"
  if (( abs_delta > 5 )); then
    printf 'Jetson clock differs by more than 5 seconds. Synchronize now? [y/N] '
    read -r answer
    [[ "$answer" =~ ^[Yy]$ ]] || die "clock synchronization declined"
    iso=$(date --iso-8601=seconds)
    ssh -t -o ConnectTimeout=8 -i "$SSH_KEY" "$ROBOT" \
      "sudo date -s '$iso'; date --iso-8601=ns"
    vm_epoch=$(date +%s)
    robot_epoch=$("${ssh_base[@]}" 'date +%s')
    delta=$((vm_epoch - robot_epoch))
    abs_delta=${delta#-}
    (( abs_delta <= 5 )) || die "clock still differs by ${delta}s"
  fi
}

remote_action() {
  local action=$1
  shift || true
  local camera=${LITE3_MAPPING_CAMERA:-1}
  [[ "$camera" == 1 || "$camera" == 2 ]] || die 'LITE3_MAPPING_CAMERA must be 1 or 2'
  "${ssh_base[@]}" "LITE3_MAPPING_CAMERA=$camera" "$BACKEND_REMOTE" "$action" "$@"
}

camera2_start() {
  # Only deploy the independent camera bridge; no Nav2, motor service or clock writes.
  local project_local="$SCRIPT_DIR/.."
  local project_remote=/home/ysc/lite3_patrol_project
  local module=patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/rtsp_camera.py
  [[ -f "$SSH_KEY" ]] || die "SSH key not found: $SSH_KEY"
  [[ -f "$project_local/$module" ]] || die "camera bridge source missing"
  "${ssh_base[@]}" "mkdir -p '$project_remote/navigation' '$project_remote/$(dirname "$module")'"
  scp -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$project_local/navigation/start_front_camera.sh" "$ROBOT:$project_remote/navigation/"
  scp -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$project_local/$module" "$ROBOT:$project_remote/$module"
  log 'Camera2: LRCP RTSP from 192.168.1.120. Keep this terminal open during recording.'
  log 'Reception timestamps only; camera2 calibration and visual-map generation are NOT complete.'
  # A PTY delivers Ctrl+C to the remote foreground bridge. No background orphan.
  ssh -t -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" "$ROBOT" \
    "bash '$project_remote/navigation/start_front_camera.sh'"
}

view_video() {
  check_local
  [[ -f "$VIDEO_VIEWER_LOCAL" ]] || die "video viewer missing: $VIDEO_VIEWER_LOCAL"
  set +u
  source /opt/ros/galactic/setup.bash
  set -u
  export RMW_IMPLEMENTATION=${LITE3_VIEW_RMW:-rmw_fastrtps_cpp}
  export ROS_DOMAIN_ID=${LITE3_ROS_DOMAIN_ID:-0}
  export ROS_LOCALHOST_ONLY=0
  unset CYCLONEDDS_URI
  log "opening camera ${LITE3_MAPPING_CAMERA:-1} preview; press ESC/q to close"
  python3 "$VIDEO_VIEWER_LOCAL" --camera "${LITE3_MAPPING_CAMERA:-1}"
}

action=${1:-}
shift || true
case "$action" in
  camera2-start)
    camera2_start
    ;;
  install)
    install_backend
    ;;
  preflight)
    ensure_backend
    ensure_clock
    remote_action preflight
    ;;
  start)
    [[ $# -eq 1 ]] || die "start requires one RUN_ID"
    ensure_backend
    ensure_clock
    remote_action start "$1"
    ;;
  status)
    ensure_backend
    remote_action status "${1:-}"
    ;;
  view-video)
    view_video
    ;;
  stop)
    ensure_backend
    remote_action stop "${1:-}"
    ;;
  stop-now)
    ensure_backend
    remote_action stop-now "${1:-}"
    ;;
  abort)
    ensure_backend
    remote_action abort "${1:-}"
    ;;
  verify)
    [[ $# -eq 1 ]] || die "verify requires one RUN_ID"
    ensure_backend
    remote_action verify "$1"
    ;;
  -h|--help|help|'') usage ;;
  *) usage; die "unknown action: $action" ;;
esac
