#!/usr/bin/env bash
# Bridges a verified visual-map base pose into hdl_localization /initialpose.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
CONFIG=$PROJECT/navigation/$MAP_ID/visual_initialization.yaml
TEST_ONLY=${LITE3_RELOCALIZATION_TEST_ONLY:-0}
source "$PROJECT/navigation/patrol_env.sh"

if [[ ! "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ || ! -r "$CONFIG" ]]; then
  echo "missing or invalid visual initialization config: $CONFIG" >&2
  exit 1
fi

parameters=(--params-file "$CONFIG")
case "$TEST_ONLY" in
  0) ;;
  1)
    # Exercise exactly the production map parameters; a different diagnostic
    # acceptance profile used to mask differences on the next full startup.
    echo '[INFO] stationary relocalization test uses production localization parameters'
    ;;
  *) echo 'LITE3_RELOCALIZATION_TEST_ONLY must be 0 or 1' >&2; exit 1 ;;
esac

exec ros2 run lite3_patrol_nav visual_pose_initializer --ros-args \
  "${parameters[@]}"
