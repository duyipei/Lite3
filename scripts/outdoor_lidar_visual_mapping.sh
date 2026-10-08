#!/usr/bin/env bash
# Lite3 outdoor LiDAR + RGB-D mapping acquisition entry point.
# Motion is controlled only with the official controller; this script never
# publishes a velocity or starts Nav2.

set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CAPTURE="$SCRIPT_DIR/round3a_capture.sh"

die() {
  printf '[outdoor-mapping][ERROR] %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Lite3 outdoor LiDAR + visual mapping acquisition

Usage:
  ./outdoor_lidar_visual_mapping.sh start [RUN_ID] [CAMERA: 1|2]
  ./outdoor_lidar_visual_mapping.sh status
  ./outdoor_lidar_visual_mapping.sh stop
  ./outdoor_lidar_visual_mapping.sh video
  ./outdoor_lidar_visual_mapping.sh abort

start  Synchronize and record Mid360, IMU, RGB and depth. If RUN_ID is omitted,
       an outdoor_YYYYMMDD_HHMMSS identifier is generated automatically.
       Camera 2 additionally records LRCP compressed RGB; calibration and a
       camera-2 visual-map builder are still required. Camera 1 is retained.
status Show recorder state and remaining disk space.
stop   Keep the robot stationary, close the bag, then perform the essential
       topic/rate verification.
video  Open the RGB preview in another local terminal (q/Esc exits preview).
abort  Immediately stop acquisition after the operator has stopped the robot.
EOF
}

[[ -x "$CAPTURE" ]] || die "missing capture program: $CAPTURE"

action=${1:-}
case "$action" in
  start)
    export LITE3_MAPPING_CAMERA=${3:-${LITE3_MAPPING_CAMERA:-1}}
    [[ "$LITE3_MAPPING_CAMERA" == 1 || "$LITE3_MAPPING_CAMERA" == 2 ]] || die 'CAMERA must be 1 or 2'
    run_id=${2:-outdoor_$(date +%Y%m%d_%H%M%S)}
    [[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$ ]] || \
      die "RUN_ID may contain only letters, numbers, '_' and '-'"
    printf '[outdoor-mapping] RUN_ID=%s\n' "$run_id"
    printf '[outdoor-mapping] Keep the robot stationary until the terminal prints GO.\n'
    "$CAPTURE" start "$run_id"
    ;;
  status)
    "$CAPTURE" status
    ;;
  stop)
    printf '[outdoor-mapping] Confirm the robot is back at the start and fully stationary.\n'
    "$CAPTURE" stop
    ;;
  video)
    export LITE3_MAPPING_CAMERA=${2:-${LITE3_MAPPING_CAMERA:-1}}
    "$CAPTURE" view-video
    ;;
  abort)
    printf '[outdoor-mapping] This does not stop physical motion; use the official controller first.\n'
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
