#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
source "$PROJECT/navigation/patrol_env.sh"
exec ros2 run lite3_patrol_nav patrol_runner \
  "$PROJECT/navigation/$MAP_ID/mission_draft.yaml"
