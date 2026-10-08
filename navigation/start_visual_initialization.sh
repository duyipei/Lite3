#!/usr/bin/env bash
# Bridges a verified visual-map base pose into hdl_localization /initialpose.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
CONFIG=$PROJECT/navigation/$MAP_ID/visual_initialization.yaml
source "$PROJECT/navigation/patrol_env.sh"

if [[ ! "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ || ! -r "$CONFIG" ]]; then
  echo "missing or invalid visual initialization config: $CONFIG" >&2
  exit 1
fi

exec ros2 run lite3_patrol_nav visual_pose_initializer --ros-args \
  --params-file "$CONFIG"
