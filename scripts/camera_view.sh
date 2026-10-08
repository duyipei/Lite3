#!/usr/bin/env bash
# Read-only viewer. Camera selection does not change navigation or drivers.
set -eo pipefail
PROJECT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
camera=${1:-}
if [[ "$camera" == --help || "$camera" == -h ]]; then
  echo 'Usage: bash scripts/camera_view.sh [1|2]'
  echo '1 = D435i /camera/color/image_raw; 2 = LRCP /front_camera/image_raw'
  echo 'No argument: interactive choice (Enter defaults to camera 1).'
  exit 0
fi
if [[ -z "$camera" ]]; then
  if [[ ! -t 0 ]]; then
    echo 'Non-interactive use requires camera number 1 or 2.' >&2
    exit 2
  fi
  echo '相机1：RealSense D435i；相机2：LRCP F1080P（新接入）'
  read -r -p '请选择摄像头 [1/2，回车默认1]：' camera
  camera=${camera:-1}
fi
case "$camera" in 1|2) ;; *) echo '摄像头编号只能为1或2' >&2; exit 2 ;; esac
if [[ -z "${DISPLAY:-}" ]]; then
  echo '请在虚拟机或 Jetson 图形桌面的终端运行；当前无 DISPLAY。' >&2
  exit 1
fi
if [[ -r /opt/ros/galactic/setup.bash ]]; then
  source /opt/ros/galactic/setup.bash
elif [[ -r /opt/ros/foxy/setup.bash ]]; then
  source /opt/ros/foxy/setup.bash
else
  echo 'ROS2 Foxy/Galactic environment not found.' >&2
  exit 1
fi
export ROS_DOMAIN_ID=${LITE3_ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=${LITE3_VIEW_RMW:-rmw_cyclonedds_cpp}
exec python3 "$PROJECT/scripts/round3a_live_video.py" --camera "$camera"
