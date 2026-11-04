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

camera_status=0
# Match the visual node's sensor QoS. Inspect a real Image instead of parsing
# CLI output; closing echo's pipe early can raise BrokenPipeError and stderr
# suppression previously hid discovery/import failures behind "no live frames".
timeout --kill-after=2s 30s python3 - <<'PY_CAMERA' || camera_status=$?
import sys
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('patrol_visual_camera_check')
received = []
def receive(message):
    if not received:
        received.append(message)
try:
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    subscription = node.create_subscription(Image, '/camera/color/image_raw', receive, qos)
    deadline = time.monotonic()+30.
    print('[CAMERA] Waiting for /camera/color/image_raw (up to 30 s)', flush=True)
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if not received:
        print('[FAIL] No RGB Image received; publishers={}'.format(
            node.count_publishers('/camera/color/image_raw')), file=sys.stderr, flush=True)
        raise SystemExit(1)
    image = received[0]
    print('[PASS] RGB Image received: {}x{} frame={}'.format(
        image.width, image.height, image.header.frame_id), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY_CAMERA
if (( camera_status != 0 )); then
  echo "refusing to start: RGB camera check failed (exit $camera_status); see diagnostics above" >&2
  echo "check realsense_ros2.service logs for stream/USB errors; service active alone does not confirm live frames" >&2
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
