#!/usr/bin/env bash
# Starts sensing and localization only. It never starts Nav2 or a velocity gate.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
NDT_RESOLUTION=${LITE3_NDT_RESOLUTION:-1.0}
DEFAULT_DOWNSAMPLE_RESOLUTION=0.3
if [[ "$MAP_ID" == outdoor_20260921_003_area_review ]]; then
  DEFAULT_DOWNSAMPLE_RESOLUTION=0.5
fi
DOWNSAMPLE_RESOLUTION=${LITE3_DOWNSAMPLE_RESOLUTION:-$DEFAULT_DOWNSAMPLE_RESOLUTION}
MAP_DIR=$PROJECT/navigation/$MAP_ID
source "$PROJECT/navigation/patrol_env.sh"

if [[ ! "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "invalid map id: $MAP_ID" >&2
  exit 1
fi
for artifact in patrol_map.yaml localization_map.pcd nav2_safe.yaml; do
  if [[ ! -r "$MAP_DIR/$artifact" ]]; then
    echo "missing map artifact: $MAP_DIR/$artifact" >&2
    exit 1
  fi
done

if pgrep -af 'livox_ros_driver2|hdl_localization|fastlivo_mapping|ros2 bag play' >/dev/null; then
  pgrep -af 'livox_ros_driver2|hdl_localization|fastlivo_mapping|ros2 bag play' >&2 || true
  echo "refusing to start: conflicting perception/localization process exists" >&2
  exit 1
fi

mkdir -p "$MAP_DIR/logs"
driver_pid=
leg_odom_bridge_pid=
localization_pid=
cleanup() {
  if [[ -n "${localization_pid:-}" ]]; then
    kill -INT -- "-$localization_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-$localization_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-$localization_pid" 2>/dev/null || true
    wait "$localization_pid" 2>/dev/null || true
  fi
  if [[ -n "${driver_pid:-}" ]]; then
    # start_livox.sh launches ros2 children.  Signal its whole private process
    # group so a failed preflight cannot leave an orphan LiDAR publisher.
    kill -INT -- "-$driver_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-$driver_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-$driver_pid" 2>/dev/null || true
    wait "$driver_pid" 2>/dev/null || true
  fi
  if [[ -n "${leg_odom_bridge_pid:-}" ]]; then
    kill -INT -- "-$leg_odom_bridge_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-$leg_odom_bridge_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-$leg_odom_bridge_pid" 2>/dev/null || true
    wait "$leg_odom_bridge_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

setsid bash /home/ysc/lite_cog_ros2/system/scripts/lidar/start_livox.sh \
  >"$MAP_DIR/logs/livox_live.log" 2>&1 &
driver_pid=$!
sleep 5

# This Jetson's Foxy `ros2 topic hz` can remain silent even while messages are
# arriving.  Wait for the first deserialized header instead.  `head` closes
# the pipe immediately after one message, so the check does not consume the
# full timeout when the sensor is healthy.
lidar_sample=$(timeout 45s ros2 topic echo /rslidar_points \
  sensor_msgs/msg/PointCloud2 --no-arr 2>&1 | head -n 1 || true)
if [[ "$lidar_sample" != "header:" ]]; then
  echo "refusing to start localization: /rslidar_points has no live data" >&2
  exit 1
fi
imu_sample=$(timeout 20s ros2 topic echo /imu/data \
  sensor_msgs/msg/Imu --no-arr 2>&1 | head -n 1 || true)
if [[ "$imu_sample" != "header:" ]]; then
  echo "refusing to start localization: /imu/data has no live data" >&2
  exit 1
fi

setsid python3 "$PROJECT/scripts/leg_odom_tf_bridge.py" \
  >"$MAP_DIR/logs/leg_odom_tf_bridge.log" 2>&1 &
leg_odom_bridge_pid=$!
sleep 1
if ! kill -0 "$leg_odom_bridge_pid" 2>/dev/null; then
  echo "refusing to start localization: leg odometry TF bridge failed" >&2
  exit 1
fi

# Bound OpenMP teams: FLANN status queries otherwise contend with scan matching
# and the ROS callbacks on the six-core Jetson. Keep all quality checks intact.
setsid env OMP_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0 \
  ros2 launch lite3_patrol_nav patrol_navigation.launch.py \
  map_yaml:="$MAP_DIR/patrol_map.yaml" \
  globalmap_pcd:="$MAP_DIR/localization_map.pcd" \
  params_file:="$MAP_DIR/nav2_safe.yaml" \
  start_localization:=True start_navigation:=False \
  specify_init_pose:=False use_imu:=False convert_utm_to_local:=False \
  ndt_resolution:="$NDT_RESOLUTION" \
  downsample_resolution:="$DOWNSAMPLE_RESOLUTION" &
localization_pid=$!
wait "$localization_pid"
