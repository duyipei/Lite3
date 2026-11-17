#!/usr/bin/env bash
# Reuse the NDT/odometry readiness guard; no visual frontend or motion command.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
python3 "$PROJECT/scripts/check_system_time.py" check
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
CONFIG=$PROJECT/navigation/$MAP_ID/visual_initialization.yaml
source "$PROJECT/navigation/patrol_env.sh"

if [[ ! "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ || ! -r "$CONFIG" ]]; then
  echo "missing or invalid initialization config: $CONFIG" >&2
  exit 1
fi

# Keep the established node name so the map-specific YAML and the existing
# readiness consumers retain their exact parameter/topic contracts.
exec ros2 run lite3_patrol_nav visual_pose_initializer --ros-args \
  --params-file "$CONFIG" \
  -p initialization_mode:=manual \
  -p auto_initialize:=false \
  -p supervise_visual_after_ready:=false
