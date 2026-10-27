#!/usr/bin/env bash
# Unified indoor/outdoor LiDAR + visual data acquisition entry point.
# Motion is controlled only with the official controller; this script never
# publishes a velocity or starts Nav2.

set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CAPTURE="$SCRIPT_DIR/mapping_capture.sh"
source "$SCRIPT_DIR/mapping_camera_selection.sh"

die() {
  printf '[mapping][ERROR] %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Lite3 unified LiDAR + visual mapping acquisition

Usage:
  ./mapping_start.sh install
  ./mapping_start.sh start [RUN_ID] [CAMERA: 1|2]
  ./mapping_start.sh camera2-start
  ./mapping_start.sh status
  ./mapping_start.sh stop
  ./mapping_start.sh video [CAMERA: 1|2]
  ./mapping_start.sh abort
  ./mapping_start.sh verify RUN_ID

start  Synchronize and record Mid360, IMU, RGB and depth. If RUN_ID is omitted,
       an mapping_YYYYMMDD_HHMMSS identifier is generated automatically.
       Without CAMERA or LITE3_MAPPING_CAMERA, prompt for 1/2 (Enter: 1).
       Camera 2 additionally records LRCP compressed RGB; calibration and a
       camera-2 visual-map builder are still required. Camera 1 is retained.
status Show recorder state and remaining disk space.
install Sync acquisition backend without starting capture.
verify Check an existing saved run.
camera2-start  Sync/start LRCP ROS2 bridge on Jetson in a separate terminal.
              Keep it open until recording stops. No calibration is implied.
stop   Keep the robot stationary, close the bag, then perform the essential
       topic/rate verification.
video  Open the RGB preview in another local terminal (q/Esc exits preview).
abort  Immediately stop acquisition after the operator has stopped the robot.
EOF
}

[[ -x "$CAPTURE" ]] || die "missing capture program: $CAPTURE"

action=${1:-}
case "$action" in
  install)
    "$CAPTURE" install
    ;;
  verify)
    [[ $# -eq 2 ]] || die 'verify requires one RUN_ID'
    "$CAPTURE" verify "$2"
    ;;
  camera2-start)
    "$CAPTURE" camera2-start
    ;;
  start)
    run_id=${2:-mapping_$(date +%Y%m%d_%H%M%S)}
    [[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$ ]] || \
      die "RUN_ID may contain only letters, numbers, '_' and '-'"
    select_mapping_camera "${3:-}"
    printf '[mapping] RUN_ID=%s\n' "$run_id"
    printf '[mapping] Keep the robot stationary until the terminal prints GO.\n'
    "$CAPTURE" start "$run_id"
    ;;
  status)
    "$CAPTURE" status
    ;;
  stop)
    printf '[mapping] Confirm the robot is back at the start and fully stationary.\n'
    "$CAPTURE" stop
    ;;
  video)
    select_mapping_camera "${2:-}"
    "$CAPTURE" view-video
    ;;
  abort)
    printf '[mapping] This does not stop physical motion; use the official controller first.\n'
    "$CAPTURE" abort
    ;;
  help|-h|--help|'')
    usage
    ;;
  *)
    usage
    die "unknown action: $action"
    ;;
esac
