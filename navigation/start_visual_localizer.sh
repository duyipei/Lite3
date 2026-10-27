#!/usr/bin/env bash
# Starts camera-only global pose estimation. It never publishes velocity commands.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
DATABASE=${LITE3_VISUAL_DATABASE:-$PROJECT/results/visual_maps/$MAP_ID/visual_map.db}
BACKEND=${LITE3_VISUAL_BACKEND:-orb}
case "$BACKEND" in orb|xfeat_lighterglue) ;; *) echo "invalid visual backend: $BACKEND" >&2; exit 1 ;; esac
SEARCH_STRATEGY=${LITE3_VISUAL_SEARCH_STRATEGY:-startup_then_global}
source "$PROJECT/navigation/patrol_env.sh"
set +u
source /home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash
set -u

if [[ ! "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ || ! -r "$DATABASE" ]]; then
  echo "missing or invalid visual database: $DATABASE" >&2
  exit 1
fi
if ros2 node list 2>/dev/null | grep -qx /rgbd_visual_localizer; then
  echo "refusing to start: /rgbd_visual_localizer already exists" >&2
  exit 1
fi

camera_sample=$(timeout 30s ros2 topic echo /camera/color/image_raw \
  sensor_msgs/msg/Image --no-arr 2>/dev/null | head -n 1 || true)
if [[ "$camera_sample" != "header:" ]]; then
  echo "refusing to start: /camera/color/image_raw has no live frames" >&2
  echo "restart realsense_ros2.service at the robot terminal and retry" >&2
  exit 1
fi

exec ros2 run lite3_patrol_nav rgbd_visual_localizer --ros-args \
  -p matching_backend:="$BACKEND" \
  -p database:="$DATABASE" \
  -p image_topic:=/camera/color/image_raw \
  -p map_frame:=visual_map \
  -p processing_rate_hz:=1.0 \
  -p max_input_age_s:=1.5 \
  -p max_query_age_s:=5.0 \
  -p candidate_count:=10 \
  -p pnp_candidate_count:=4 \
  -p learned_required_references:=1 \
  -p ratio_test:=0.80 \
  -p min_matches:=18 \
  -p min_inliers:=15 \
  -p search_strategy:="$SEARCH_STRATEGY" \
  -p startup_keyframe_count:=6 \
  -p startup_timeout_s:=8.0 \
  -p confirmation_samples:=1 \
  -p confirmation_max_position_m:=0.50 \
  -p confirmation_max_yaw_rad:=0.35
