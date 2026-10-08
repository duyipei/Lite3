#!/usr/bin/env bash
# Lite3 indoor LiDAR + RGB-D mapping acquisition entry point.
# Motion remains under the official controller; this script only records data.

set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CAPTURE="$SCRIPT_DIR/round3a_capture.sh"

die() {
  printf '[indoor-mapping][ERROR] %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Lite3 indoor LiDAR + visual mapping acquisition

Usage:
  ./indoor_lidar_visual_mapping.sh start [RUN_ID] [CAMERA: 1|2]
  ./indoor_lidar_visual_mapping.sh status
  ./indoor_lidar_visual_mapping.sh stop
  ./indoor_lidar_visual_mapping.sh video
  ./indoor_lidar_visual_mapping.sh abort

start  Record synchronized Mid360, IMU, RGB and depth. If RUN_ID is omitted,
       an indoor_YYYYMMDD_HHMMSS identifier is generated.
       Camera 2 additionally records LRCP compressed RGB; calibration and a
       camera-2 visual-map builder are still required. Camera 1 is retained.
status Show recorder state and remaining storage.
stop   Stop while stationary, finalize the bag and run essential verification.
video  Open the local RGB preview; q/Esc closes it.
abort  Stop acquisition immediately after the operator stops the robot.
EOF
}

[[ -x "$CAPTURE" ]] || die "missing capture program: $CAPTURE"

action=${1:-}
case "$action" in
  start)
    export LITE3_MAPPING_CAMERA=${3:-${LITE3_MAPPING_CAMERA:-1}}
    [[ "$LITE3_MAPPING_CAMERA" == 1 || "$LITE3_MAPPING_CAMERA" == 2 ]] || die 'CAMERA must be 1 or 2'
    run_id=${2:-indoor_$(date +%Y%m%d_%H%M%S)}
    [[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$ ]] || \
      die "RUN_ID may contain only letters, numbers, '_' and '-'"
    printf '[indoor-mapping] RUN_ID=%s\n' "$run_id"
    printf '[indoor-mapping] Keep the robot stationary until GO is printed.\n'
    "$CAPTURE" start "$run_id"
    ;;
  status)
    "$CAPTURE" status
    ;;
  stop)
    printf '[indoor-mapping] Stop the robot and keep it stationary.\n'
    "$CAPTURE" stop
    ;;
  video)
    export LITE3_MAPPING_CAMERA=${2:-${LITE3_MAPPING_CAMERA:-1}}
    "$CAPTURE" view-video
    ;;
  abort)
    printf '[indoor-mapping] Stop physical motion with the official controller first.\n'
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
