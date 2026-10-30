#!/usr/bin/env bash
# Indoor/outdoor visual-initial-pose + NDT-confirmed isolated Nav2 field launcher.
# Run this script on the Ubuntu development VM, not on the Jetson.
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
ROBOT=${ROBOT:-${LITE3_ROBOT_USER:-ysc}@${LITE3_ROBOT_HOST:-10.119.1.171}}
SSH_KEY=${SSH_KEY:-/home/duyipei/.ssh/id_ed25519_cyberdog}
REMOTE_PROJECT=/home/ysc/lite3_patrol_project
DEFAULT_MAP_ID=indoor_fusion_20260912_001_area_review
ACTION=${1:-start}
MAP_ID=${2:-$DEFAULT_MAP_ID}
DEFAULT_VISUAL_MAP_ID=$MAP_ID
if [[ "$MAP_ID" =~ ^(.+)_area(_v[0-9]+)?_review$ ]]; then
  DEFAULT_VISUAL_MAP_ID=${BASH_REMATCH[1]}
fi
VISUAL_MAP_ID=${LITE3_VISUAL_MAP_ID:-$DEFAULT_VISUAL_MAP_ID}
VISUAL_BACKEND=${LITE3_VISUAL_BACKEND:-orb}
VISUAL_SEARCH_STRATEGY=${LITE3_VISUAL_SEARCH_STRATEGY:-startup_then_global}
VISUAL_WAIT_TIMEOUT_S=${LITE3_VISUAL_WAIT_TIMEOUT_S:-240}
DYNAMIC_OBSTACLES=${LITE3_DYNAMIC_OBSTACLES:-0}
GROUND_MODE=${LITE3_GROUND_MODE:-map}
LOCALIZATION_RUNTIME=${LITE3_LOCALIZATION_RUNTIME:-vendor}
USE_VENDOR_VOA=${LITE3_USE_VENDOR_VOA:-$DYNAMIC_OBSTACLES}
RVIZ_SSH_BRIDGE=${LITE3_RVIZ_SSH_BRIDGE:-1}
RVIZ_LOCALHOST_ONLY=$RVIZ_SSH_BRIDGE
NDT_RESOLUTION=${NDT_RESOLUTION:-1.0}
FINE_REGISTRATION=${LITE3_NDT_FINE_REFINE:-false}
DEFAULT_DOWNSAMPLE_RESOLUTION=0.3
# Field-verified outdoor map: 0.3 m overloaded the Jetson scan matcher.
if [[ "$MAP_ID" == outdoor_20260921_003_area_review ]]; then
  DEFAULT_DOWNSAMPLE_RESOLUTION=0.5
fi
DOWNSAMPLE_RESOLUTION=${DOWNSAMPLE_RESOLUTION:-$DEFAULT_DOWNSAMPLE_RESOLUTION}

SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" "$ROBOT")
SSH_BG=(ssh -n -T -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" "$ROBOT")

LOCALIZATION_SSH_PID=
MAP_DISPLAY_SSH_PID=
NAV_SSH_PID=
VISUAL_LOCALIZER_SSH_PID=
VISUAL_INITIALIZER_SSH_PID=
GATE_SSH_PID=
GOAL_BRIDGE_SSH_PID=
RVIZ_SSH_PID=
RVIZ_POINTCLOUD_RELAY_PID=
RVIZ_SSH_BRIDGE_PID=
NAV_RESULT_NOTIFIER_PID=
SESSION_ACTIVE=0
LOG_DIR=$PROJECT_DIR/logs/lite3_indoor_nav_${MAP_ID}_$(date +%Y%m%d_%H%M%S)_$$

line() { printf '%s\n' '------------------------------------------------------------'; }
title() { line; printf '%s\n' "$1"; line; }
die() { printf '[FAIL] %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<EOF
用法（在本地 Ubuntu 执行）：
  $0 install
  $0 preflight [MAP_ID]
  $0 relocalize [MAP_ID]
  $0 start [MAP_ID]
  $0 arm [MAP_ID]
  $0 disarm [MAP_ID]
  $0 status [MAP_ID]
  $0 stop [MAP_ID]

默认 MAP_ID：$DEFAULT_MAP_ID

relocalize 仅测试视觉初始重定位和 NDT 连续确认，保持就绪 10 秒后
自动关闭本次定位节点；不启动 Nav2、目标桥或速度门。
静止测试使用与正式导航相同的地图定位参数。

start 自动完成：部署脚本 -> Mid360/NDT -> 视觉关键帧检索
-> 视觉粗位姿注入 -> NDT 连续确认 -> 临时地图显示 -> RViz
-> 隔离目标话题 /patrol/goal_request -> Jetson Foxy action 桥
-> 完整隔离 Nav2 -> DISARMED 速度门。脚本不会自动允许机器狗运动。

视觉地图可通过 LITE3_VISUAL_MAP_ID 指定，默认：$VISUAL_MAP_ID
LITE3_VISUAL_BACKEND=orb（默认）或 xfeat_lighterglue；XFeat 首次使用需运行 tools/deploy_xfeat_runtime.sh。
RViz 默认通过无线 SSH 转发地图、点云和目标，适用于 NAT 虚拟机。
LITE3_RVIZ_SSH_BRIDGE=0 可恢复直接 DDS 显示。
LITE3_DYNAMIC_OBSTACLES=1 启用实时全局绕行、等待和 VOA 近距离停车；需预先构建 VOA。
EOF
}

validate_inputs() {
  [[ "$DYNAMIC_OBSTACLES" == 0 || "$DYNAMIC_OBSTACLES" == 1 ]] || die 'LITE3_DYNAMIC_OBSTACLES must be 0 or 1'
  [[ "$GROUND_MODE" == map || "$GROUND_MODE" == local ]] || die 'LITE3_GROUND_MODE must be map or local'
  [[ "$LOCALIZATION_RUNTIME" == vendor || "$LOCALIZATION_RUNTIME" == audited ]] || die 'LITE3_LOCALIZATION_RUNTIME must be vendor or audited'
  if [[ "$DYNAMIC_OBSTACLES" == 1 && "$USE_VENDOR_VOA" != 1 ]]; then
    die '动态避障模式需要 VOA；不能关闭近距离速度检查'
  fi
  [[ "$USE_VENDOR_VOA" == 0 || "$USE_VENDOR_VOA" == 1 ]] || die 'LITE3_USE_VENDOR_VOA must be 0 or 1'
  [[ "$RVIZ_SSH_BRIDGE" == 0 || "$RVIZ_SSH_BRIDGE" == 1 ]] || die 'LITE3_RVIZ_SSH_BRIDGE must be 0 or 1'
  [[ -r "$SSH_KEY" ]] || die "SSH 密钥不存在: $SSH_KEY"
  [[ "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ ]] || die "地图名称含非法字符: $MAP_ID"
  [[ "$VISUAL_MAP_ID" =~ ^[A-Za-z0-9._-]+$ ]] || \
    die "视觉地图名称含非法字符: $VISUAL_MAP_ID"
  case "$VISUAL_BACKEND" in
    orb|xfeat_lighterglue) ;;
    *) die "LITE3_VISUAL_BACKEND 只能是 orb 或 xfeat_lighterglue" ;;
  esac
  case "$VISUAL_SEARCH_STRATEGY" in
    startup_then_global|startup_only|global) ;;
    *) die "LITE3_VISUAL_SEARCH_STRATEGY 只能是 startup_then_global、startup_only 或 global" ;;
  esac
  [[ "$VISUAL_WAIT_TIMEOUT_S" =~ ^[0-9]+$ ]] || \
    die "LITE3_VISUAL_WAIT_TIMEOUT_S 必须为整数秒"
  (( VISUAL_WAIT_TIMEOUT_S >= 30 && VISUAL_WAIT_TIMEOUT_S <= 300 )) || \
    die "LITE3_VISUAL_WAIT_TIMEOUT_S 必须为 30~300 秒"
  [[ "$FINE_REGISTRATION" == true || "$FINE_REGISTRATION" == false ]] || die "LITE3_NDT_FINE_REFINE 必须为 true/false"
  if [[ "$FINE_REGISTRATION" == true && "$ACTION" != relocalize ]]; then
    die '精配准尚未通过实机与低矮障碍验收，只允许 relocalize 静止测试'
  fi
  [[ "$NDT_RESOLUTION" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
    die "NDT_RESOLUTION 必须为正数"
  [[ "$DOWNSAMPLE_RESOLUTION" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
    die "DOWNSAMPLE_RESOLUTION 必须为正数"
  "${SSH[@]}" true || die "无法连接 Jetson: $ROBOT"
}

deploy_runtime() {
  title "同步导航运行文件"
  local local_map_dir=$PROJECT_DIR/navigation/$MAP_ID
  [[ -r "$local_map_dir/nav2_safe.yaml" ]] || \
    die "本地缺少导航参数: $local_map_dir/nav2_safe.yaml"
  [[ -r "$local_map_dir/visual_initialization.yaml" ]] || \
    die "本地缺少视觉初始化参数: $local_map_dir/visual_initialization.yaml"
  local summary_file
  local -a map_summaries=()
  for summary_file in traversability_summary.json patrol_map_summary.json; do
    [[ ! -r "$local_map_dir/$summary_file" ]] || map_summaries+=("$local_map_dir/$summary_file")
  done
  (( ${#map_summaries[@]} > 0 )) || \
    die "本地缺少地图边界摘要: $local_map_dir/{traversability_summary,patrol_map_summary}.json"
  "${SSH[@]}" \
    "mkdir -p '$REMOTE_PROJECT/scripts' '$REMOTE_PROJECT/navigation/$MAP_ID'"
  scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$SCRIPT_DIR/make_dynamic_nav2_config.py" \
    "$SCRIPT_DIR/costmap_ground_filter.py" \
    "$SCRIPT_DIR/local_ground_model.py" \
    "$SCRIPT_DIR/local_ground_detector.py" \
    "$SCRIPT_DIR/local_ground_costmap_bridge.py" \
    "$SCRIPT_DIR/capture_ground_reference.py" \
    "$SCRIPT_DIR/check_localization_runtime.py" \
    "$SCRIPT_DIR/navigation_recovery_budget.py" \
    "$SCRIPT_DIR/voa_camera_tf.py" \
    "$SCRIPT_DIR/voa_costmap_bridge.py" \
    "$SCRIPT_DIR/check_vendor_voa.py" \
    "$SCRIPT_DIR/leg_odom_tf_bridge.py" \
    "$SCRIPT_DIR/goal_pose_bridge.py" \
    "$SCRIPT_DIR/rviz_ssh_bridge.py" \
    "$ROBOT:$REMOTE_PROJECT/scripts/"
  scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/velocity_gate.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/voa_protocol.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/patrol_runner.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/rgbd_visual_localizer.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/visual_matching.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/learned_matching.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/xfeat_matching.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/visual_pose_initializer.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/localization_consistency.py" \
    "$ROBOT:$REMOTE_PROJECT/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/"
  scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$PROJECT_DIR/navigation/patrol_env.sh" \
    "$PROJECT_DIR/navigation/start_localization_only.sh" \
    "$PROJECT_DIR/navigation/start_map_display_only.sh" \
    "$PROJECT_DIR/navigation/start_visual_localizer.sh" \
    "$PROJECT_DIR/navigation/start_visual_initialization.sh" \
    "$PROJECT_DIR/navigation/start_nav_isolated.sh" \
    "$PROJECT_DIR/navigation/start_local_ground.sh" \
    "$PROJECT_DIR/navigation/start_gate_disarmed.sh" \
    "$PROJECT_DIR/navigation/start_vendor_voa.sh" \
    "$PROJECT_DIR/navigation/navigate_dynamic_obstacles.xml" \
    "$PROJECT_DIR/navigation/vendor_voa.yaml" \
    "$ROBOT:$REMOTE_PROJECT/navigation/"
  scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$local_map_dir/nav2_safe.yaml" \
    "$local_map_dir/visual_initialization.yaml" \
    "${map_summaries[@]}" \
    "$ROBOT:$REMOTE_PROJECT/navigation/$MAP_ID/"
  if [[ "$DYNAMIC_OBSTACLES" == 1 ]]; then
    [[ -r "$local_map_dir/traversability_evidence.npz" ]] || die '缺少当前地图的地面高度证据'
    scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
      "$local_map_dir/traversability_evidence.npz" "$ROBOT:$REMOTE_PROJECT/navigation/$MAP_ID/"
  fi
  scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/launch/patrol_navigation.launch.py" \
    "$PROJECT_DIR/patrol_nav_ws/src/lite3_patrol_nav/launch/map_display.launch.py" \
    "$ROBOT:$REMOTE_PROJECT/patrol_nav_ws/src/lite3_patrol_nav/launch/"
  "${SSH[@]}" "bash -s" <<'REMOTE'
set -Eeuo pipefail
PROJECT=/home/ysc/lite3_patrol_project
cp "$PROJECT/patrol_nav_ws/src/lite3_patrol_nav/launch/patrol_navigation.launch.py" \
  "$PROJECT/patrol_nav_ws/install_merged/share/lite3_patrol_nav/launch/patrol_navigation.launch.py"
cp "$PROJECT/patrol_nav_ws/src/lite3_patrol_nav/launch/map_display.launch.py" \
  "$PROJECT/patrol_nav_ws/install_merged/share/lite3_patrol_nav/launch/map_display.launch.py"
for module in velocity_gate voa_protocol rgbd_visual_localizer visual_matching learned_matching xfeat_matching visual_pose_initializer localization_consistency patrol_runner; do
  cp "$PROJECT/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/$module.py" \
    "$PROJECT/patrol_nav_ws/install_merged/lib/python3.8/site-packages/lite3_patrol_nav/$module.py"
done
chmod 0755 \
  "$PROJECT/scripts/leg_odom_tf_bridge.py" \
  "$PROJECT/scripts/goal_pose_bridge.py" \
  "$PROJECT/navigation/start_localization_only.sh" \
  "$PROJECT/navigation/start_map_display_only.sh" \
  "$PROJECT/navigation/start_visual_localizer.sh" \
  "$PROJECT/navigation/start_visual_initialization.sh" \
  "$PROJECT/navigation/start_nav_isolated.sh" \
  "$PROJECT/navigation/start_gate_disarmed.sh"
REMOTE
  echo "[PASS] 运行脚本已同步到 Jetson"
}

static_preflight() {
  title "启动前检查"
  "${SSH[@]}" "bash -s -- '$MAP_ID' '$VISUAL_MAP_ID' '$USE_VENDOR_VOA' '$DYNAMIC_OBSTACLES'" <<'REMOTE'
set -Eeuo pipefail
MAP_ID=$1
VISUAL_MAP_ID=$2
USE_VOA=$3
DYNAMIC_OBSTACLES=$4
PROJECT=/home/ysc/lite3_patrol_project
MAP_DIR=$PROJECT/navigation/$MAP_ID
fail=0
pass() { echo "[PASS] $*"; }
bad() { echo "[FAIL] $*"; fail=1; }

if [[ "$USE_VOA" == 1 ]]; then
  [[ -x "$PROJECT/voa_overlay_ws/install/voa/lib/voa/voa_composition" ]] || bad 'VOA 未构建；先执行 scripts/install_vendor_voa.sh'
  if systemctl is-active --quiet voa_ros2.service; then
    bad '原厂 VOA 服务仍占用公共速度链路，不能与项目 VOA 同时启动'
  fi
fi
if [[ "$DYNAMIC_OBSTACLES" == 1 ]]; then
  python3 "$PROJECT/scripts/navigation_recovery_budget.py" \
    "$PROJECT/navigation/navigate_dynamic_obstacles.xml" >/dev/null || bad '动态恢复等待预算无效'
  GROUND_ARGS=(--ground-relative)
  python3 "$PROJECT/scripts/costmap_ground_filter.py" "$MAP_DIR" --check || bad '地面高度证据校验失败' 
  python3 "$PROJECT/scripts/make_dynamic_nav2_config.py" "$MAP_DIR/nav2_safe.yaml" \
    "$MAP_DIR/nav2_dynamic.yaml" --behavior-tree "$PROJECT/navigation/navigate_dynamic_obstacles.xml" "${GROUND_ARGS[@]}" || bad '动态避障配置生成失败'
fi

for file in patrol_map.yaml patrol_map.pgm localization_map.pcd nav2_safe.yaml visual_initialization.yaml; do
  [[ -r "$MAP_DIR/$file" ]] && pass "地图文件可读: $file" || bad "缺少 $MAP_DIR/$file"
done
if python3 - "$MAP_DIR/nav2_safe.yaml" <<'PY'
import sys
import yaml

config = yaml.safe_load(open(sys.argv[1], "r"))
local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
controller = config["controller_server"]["ros__parameters"]
dwb = controller["FollowPath"]

assert local_costmap["stvl_obstacle_layer"]["rslidar_points"]["topic"] == "/aligned_points"
assert local_costmap["stvl_obstacle_layer"]["rslidar_points"]["sensor_frame"] == "rslidar"
assert local_costmap["stvl_obstacle_layer"]["rslidar_points"]["filter"] == "passthrough"
assert "stvl_obstacle_layer" not in global_costmap["plugins"]
assert "ObstacleFootprint" in dwb["critics"]
assert "static_layer" in local_costmap["plugins"]
yaw_tolerance = float(controller["goal_checker"]["yaw_goal_tolerance"])
assert 0.05 <= yaw_tolerance <= 3.14160
if yaw_tolerance < 3.0:
    assert "RotateToGoal" in dwb["critics"]
    turn_radius = float(dwb["xy_goal_tolerance"])
    acceptance_radius = float(controller["goal_checker"]["xy_goal_tolerance"])
    assert 0.0 < turn_radius <= acceptance_radius
    if turn_radius < acceptance_radius:
        assert controller["goal_checker"]["stateful"] is False
    assert 0.0 < float(dwb["trans_stopped_velocity"]) < float(dwb["max_vel_x"])
assert float(dwb["min_vel_y"]) == 0.0
assert float(dwb["max_vel_y"]) == 0.0
assert -0.08 <= float(dwb["min_vel_x"]) <= 0.0
assert 0.0 < float(dwb["max_vel_x"]) <= 0.15
assert 0.0 < float(dwb["max_vel_theta"]) <= 0.40
assert int(dwb["vx_samples"]) >= 9
PY
then
  pass "导航配置可解析，速度和避障边界有效"
else
  bad "nav2_safe.yaml 缺少必要的速度隔离或足迹避障配置"
fi
if [[ -r "$PROJECT/results/visual_maps/$VISUAL_MAP_ID/visual_map.db" ]]; then
  pass "视觉地图数据库可读: $VISUAL_MAP_ID/visual_map.db"
else
  bad "缺少视觉地图数据库: $PROJECT/results/visual_maps/$VISUAL_MAP_ID/visual_map.db"
fi
if python3 - "$MAP_DIR/visual_initialization.yaml" <<'PY'
import sys
import json
import math
from pathlib import Path
import yaml

config = yaml.safe_load(open(sys.argv[1], "r"))
params = config["visual_pose_initializer"]["ros__parameters"]
assert params["configured"] is True
assert params["visual_map_frame"] == "visual_map"
assert params["lidar_map_frame"] == "map"
if Path(sys.argv[1]).parent.name == 'indoor_fusion_20260912_001_area_review':
    binary = Path('/home/ysc/lite_cog_ros2/nav/install/hdl_localization/lib/hdl_localization/hdl_localization_composition')
    assert b'globalmap_downsample_resolution' in binary.read_bytes(), \
        'Update/rebuild the map server patch before using independent map downsampling'
    factory_path = '/home/ysc/lite_cog_ros2/driver/mid360_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json'
    factory = json.load(open(factory_path))['lidar_configs']
    expected = dict(roll=0, pitch=18, yaw=0, x=187, y=0, z=400)
    assert len(factory) == 1 and factory[0]['extrinsic_parameter'] == expected, \
        'Factory calibration changed; recompute the map registration reference conversion'
    pitch = math.radians(18)
    expected_xyz = (-.011-math.cos(pitch)*.187+math.sin(pitch)*(.4+.34751),
                    -.02329, .04412-math.sin(pitch)*.187-math.cos(pitch)*(.4+.34751))
    actual_xyz = params['ndt_seed_reference_xyz']
    assert len(actual_xyz) == 3 and max(abs(a-b) for a,b in zip(actual_xyz, expected_xyz)) < 1e-7
    assert params['ndt_seed_reference_rpy'] == [0., -pitch, 0.]
PY
then
  pass "视觉到雷达地图变换已配置"
else
  bad "visual_initialization.yaml 未正确配置"
fi
if [[ $(systemctl is-active realsense_ros2.service 2>/dev/null || true) == active ]]; then
  pass "RealSense 服务在线"
else
  bad "realsense_ros2.service 未运行"
fi
if [[ -r "$MAP_DIR/traversability_summary.json" ]]; then
  pass "地图边界文件: traversability_summary.json"
elif [[ -r "$MAP_DIR/patrol_map_summary.json" ]]; then
  pass "地图边界文件: patrol_map_summary.json"
else
  bad "缺少 traversability_summary.json 或 patrol_map_summary.json"
fi

conflicts=$(pgrep -af \
  'start_localization_only|start_map_display_only|start_visual_localizer|start_visual_initialization|start_nav_isolated|patrol_navigation.launch.py|hdl_localization|livox_ros_driver2|manual_ndt_guard|rgbd_visual_localizer|visual_pose_initializer|velocity_gate' \
  || true)
if [[ -n "$conflicts" ]]; then
  bad "发现上次遗留的定位/导航进程；先执行 navigation_start.sh stop"
  printf '%s\n' "$conflicts"
else
  pass "没有遗留的定位/导航进程"
fi

source "$PROJECT/navigation/patrol_env.sh" >/dev/null 2>&1
if [[ "$USE_VOA" == 1 ]]; then
  depth_sample=$(timeout 15s env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ros2 topic echo /camera/depth/color/points sensor_msgs/msg/PointCloud2 --no-arr \
    2>/dev/null | head -n 1 || true)
  if [[ "$depth_sample" == 'header:' ]]; then
    pass 'VOA 深度点云有实时数据'
  else
    bad 'VOA 缺少深度点云；检查相机 enable_pointcloud:=true 并重启 realsense_ros2.service'
  fi
fi
# Foxy on this Jetson can leave `ros2 topic hz` silent even when camera
# messages are arriving.  The online visual localizer only consumes RGB, so
# gate startup on one deserialized color frame instead of unnecessarily
# requiring the depth stream as well.
camera_sample=$(timeout 20s ros2 topic echo /camera/color/image_raw \
  sensor_msgs/msg/Image --no-arr 2>/dev/null | head -n 1 || true)
if [[ "$camera_sample" == "header:" ]]; then
  pass "/camera/color/image_raw 有实时帧"
else
  bad "/camera/color/image_raw 无实时帧；RealSense 服务可能假在线"
fi
for topic in /cmd_vel /cmd_vel_corrected; do
  count=$(timeout 8s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3}' || true)
  [[ "${count:-0}" == 0 ]] && pass "$topic 无发布者" || bad "$topic 已有 ${count} 个发布者"
done
exit "$fail"
REMOTE
}

wait_localization() {
  "${SSH[@]}" "bash -s" <<'REMOTE'
set +u
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh >/dev/null 2>&1
set -u
timeout 45s python3 - <<'PY_SENSORS'
import subprocess
import time
import rclpy
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node('patrol_localization_start_check')
received = {'lidar': 0, 'imu': 0}
def callback(key):
    def receive(msg):
        received[key] += 1
    return receive
subscriptions = [
    node.create_subscription(PointCloud2, '/rslidar_points', callback('lidar'), qos_profile_sensor_data),
    node.create_subscription(Imu, '/imu/data', callback('imu'), qos_profile_sensor_data),
]
patterns = (
    'hdl_localization_composition.*__node:=patrol_localization',
    'livox_ros_driver2_node.*__node:=livox_lidar_publisher',
    '/scripts/leg_odom_tf_bridge.py',
)
deadline = time.monotonic() + 35.0
ok = False
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if min(received.values()) >= 3 and all(
        subprocess.run(['pgrep', '-f', pattern], stdout=subprocess.DEVNULL,
                       timeout=2).returncode == 0 for pattern in patterns
    ):
        ok = True
        break
print(('[PASS]' if ok else '[FAIL]') +
      ' Mid360、IMU 实时数据与 NDT/里程计进程检查: ' + str(received), flush=True)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY_SENSORS
REMOTE
}

wait_visual_node() {
  local node_name=$1
  "${SSH[@]}" "bash -s -- '$node_name'" <<'REMOTE'
set +u
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh >/dev/null 2>&1
set -u
node_name=$1
for _ in $(seq 1 35); do
  nodes=$(timeout 5s ros2 node list --no-daemon 2>/dev/null || true)
  grep -qx "$node_name" <<<"$nodes" && exit 0
  sleep 1
done
exit 1
REMOTE
}

request_visual_initialization() {
  "${SSH[@]}" \
    "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; python3 -" <<'PY'
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

rclpy.init()
node = Node("navigation_visual_initialization_request")
client = node.create_client(Trigger, "/patrol/request_visual_initialization")
if not client.wait_for_service(timeout_sec=10.0):
    raise SystemExit("visual initialization service unavailable")
future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
response = future.result()
if response is None or not response.success:
    raise SystemExit("visual initialization request failed: {}".format(response))
print("[PASS] " + response.message)
node.destroy_node()
rclpy.shutdown()
PY
}

wait_visual_ready() {
  local stable_seconds=${1:-3}
  local phase=${2:-initialization}
  local attempt result
  for attempt in 1 2; do
    result=0
    "${SSH[@]}" \
    "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; python3 - '$VISUAL_WAIT_TIMEOUT_S' '$stable_seconds' '$phase'" <<'PY' || result=$?
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

timeout_s = float(sys.argv[1])
stable_seconds = float(sys.argv[2])
phase = sys.argv[3]
rclpy.init()
node = Node("navigation_visual_initialization_monitor")
latest = {"visual": "WAITING", "guard": "WAITING", "ready": False, "odom": None}
received = {"guard": float("-inf"), "ready": float("-inf"), "odom": float("-inf")}

ready_qos = QoSProfile(depth=1)
ready_qos.reliability = ReliabilityPolicy.RELIABLE
ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

def state_callback(key):
    def callback(message):
        if key in received:
            received[key] = time.monotonic()
        if latest[key] != message.data:
            latest[key] = message.data
            print("[STATE] {}: {}".format(key, message.data), flush=True)
    return callback

def ready_callback(message):
    latest["ready"] = bool(message.data)
    received["ready"] = time.monotonic()

def odom_callback(message):
    received["odom"] = time.monotonic()
    pose = message.pose.pose
    q = pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    latest["odom"] = (pose.position.x, pose.position.y, math.degrees(yaw))

node.create_subscription(String, "/visual_localization/state", state_callback("visual"), 10)
node.create_subscription(String, "/patrol/visual_init_state", state_callback("guard"), 10)
node.create_subscription(
    Bool,
    "/patrol/localization_ready",
    ready_callback,
    ready_qos,
)
node.create_subscription(Odometry, "/odom", odom_callback, 20)

started = time.monotonic()
ready_since = None
next_progress = started + 10.0
while rclpy.ok() and time.monotonic() - started < timeout_s:
    rclpy.spin_once(node, timeout_sec=0.1)
    now = time.monotonic()
    fresh = all(now - stamp <= 1.5 for stamp in received.values())
    if latest["ready"] and latest["guard"].startswith("READY:") and fresh:
        ready_since = ready_since or now
        if now - ready_since >= stable_seconds:
            print("[PASS] " + (
                "完整 Nav2 负载下 NDT + 里程计持续就绪 {:.0f} 秒".format(stable_seconds)
                if phase == "tracking" else "视觉粗定位已由 NDT 连续确认"
            ), flush=True)
            if latest["odom"] is not None:
                print(
                    "[POSE] x={:.3f}, y={:.3f}, yaw={:.1f}deg".format(*latest["odom"]),
                    flush=True,
                )
            node.destroy_node()
            rclpy.shutdown()
            raise SystemExit(0)
    else:
        ready_since = None
    if latest["guard"].startswith(("FAILED:", "BLOCKED:")):
        print("[FAIL] " + latest["guard"], flush=True)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    missing = [key for key, stamp in received.items() if not math.isfinite(stamp)]
    if (now - started >= 15.0 and missing and
            (latest["ready"] or latest["guard"].startswith("READY:"))):
        print('[WARN] 监测订阅未收到话题: ' + ','.join(missing), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)
    if now >= next_progress:
        print(
            "[WAIT] {:.0f}/{:.0f}s visual={} guard={}".format(
                now - started, timeout_s, latest["visual"], latest["guard"]
            ),
            flush=True,
        )
        next_progress += 10.0

print(
    "[FAIL] {} 秒内未完成{}；visual={} guard={} ready={}".format(
        int(timeout_s), "NDT + 里程计持续就绪检查" if phase == "tracking" else "视觉重定位",
        latest["visual"], latest["guard"], latest["ready"]
    ),
    flush=True,
)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
    if (( result == 0 )); then return 0; fi
    if (( result != 2 || attempt == 2 )); then return 1; fi
    echo "[WARN] 重建一次监测订阅；仍需重新通过完整就绪检查"
  done
  return 1
}

wait_map_display() {
  local map_log=$LOG_DIR/map_display.log
  for _ in $(seq 1 60); do
    if [[ -n "$MAP_DISPLAY_SSH_PID" ]] && \
       ! kill -0 "$MAP_DISPLAY_SSH_PID" 2>/dev/null; then
      echo "[FAIL] 临时 Map Server 后台进程已退出"
      return 1
    fi
    if grep -F 'Managed nodes are active' "$map_log" >/dev/null 2>&1; then
      echo "[PASS] 临时地图已激活"
      return 0
    fi
    sleep 0.5
  done
  echo "[FAIL] 临时 Map Server 30 秒内未进入 active"
  return 1
}

stop_map_display() {
  set +e
  "${SSH[@]}" "bash -s" <<'REMOTE'
wrapper=$(pgrep -f '/navigation/start_map_display_only.sh' | head -n 1 || true)
if [[ -n "$wrapper" ]]; then
  kill -TERM "$wrapper" 2>/dev/null || true
fi
for _ in $(seq 1 30); do
  pgrep -f 'manual_initial_pose_map_server' >/dev/null || exit 0
  sleep 0.2
done
pids=$(pgrep -f 'manual_initial_pose_map_server' || true)
for pid in $pids; do
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
  [[ -n "$pgid" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
done
REMOTE
  if [[ -n "$MAP_DISPLAY_SSH_PID" ]]; then
    kill -TERM "$MAP_DISPLAY_SSH_PID" 2>/dev/null || true
    wait "$MAP_DISPLAY_SSH_PID" 2>/dev/null || true
    MAP_DISPLAY_SSH_PID=
  fi
  set -e
}

wait_nav2() {
  local nav_log=$LOG_DIR/nav2.log
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if [[ -n "$NAV_SSH_PID" ]] && ! kill -0 "$NAV_SSH_PID" 2>/dev/null; then
      return 1
    fi
    if grep -E 'process has died|Failed to bring up all requested nodes' "$nav_log" >/dev/null 2>&1; then
      return 1
    fi
    if grep -E 'lifecycle_manager_navigation.*Managed nodes are active' "$nav_log" >/dev/null 2>&1 && \
       grep -F 'StaticLayer: Resizing costmap' "$nav_log" >/dev/null 2>&1; then
      # Use one fresh graph participant, not two daemon-backed CLI queries.
      # A timeout must not be interpreted as zero publishers; bound wall time.
      if "${SSH[@]}" "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; timeout 12s python3 - '$DYNAMIC_OBSTACLES' '$REMOTE_PROJECT/navigation/navigate_dynamic_obstacles.xml'" <<'PY'
import sys
import time
import rclpy
from rcl_interfaces.srv import GetParameters
rclpy.init()
node = rclpy.create_node('patrol_nav_output_check')
tree_future = None
if sys.argv[1] == '1':
    tree_client = node.create_client(GetParameters, '/bt_navigator/get_parameters')
    if not tree_client.wait_for_service(timeout_sec=2.0):
        print('[WAIT] bt_navigator parameter service not discovered', flush=True)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    request = GetParameters.Request()
    request.names = ['default_bt_xml_filename']
    tree_future = tree_client.call_async(request)
deadline = time.monotonic() + 8.0
stable_since = None
counts = None
ok = False
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    counts = tuple(node.count_publishers(t) for t in
                   ('/cmd_vel', '/patrol/cmd_vel_raw', '/cmd_vel_corrected'))
    tree_ok = tree_future is None
    if tree_future is not None and tree_future.done():
        reply = tree_future.result()
        tree_ok = bool(reply and reply.values and reply.values[0].string_value == sys.argv[2])
    if tree_ok and counts[0] == 0 and counts[1] >= 1 and counts[2] == 0:
        if stable_since is None:
            stable_since = time.monotonic()
        if time.monotonic()-stable_since >= 1.0:
            ok = True
            break
    else:
        stable_since = None
if not ok:
    print('[WAIT] Nav2 output graph not isolated/ready: cmd_vel,raw,corrected=' + str(counts), flush=True)
    if not tree_ok:
        print('[FAIL] Dynamic obstacle behavior tree not confirmed on running bt_navigator', flush=True)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY
      then
        echo "[PASS] Nav2 active，地图已载入，速度输出保持隔离"
        return 0
      fi
    fi
    sleep 1
  done
  echo "[FAIL] Nav2 在限定时间内未满足地图加载与速度输出隔离条件"
  return 1
}

wait_goal_bridge() {
  local bridge_log=$LOG_DIR/goal_bridge.log
  for _ in $(seq 1 60); do
    if [[ -n "$GOAL_BRIDGE_SSH_PID" ]] && \
       ! kill -0 "$GOAL_BRIDGE_SSH_PID" 2>/dev/null; then
      return 1
    fi
    if grep -F 'action server is ready' "$bridge_log" >/dev/null 2>&1; then
      echo "[PASS] 目标桥已连接 NavigateToPose"
      return 0
    fi
    sleep 0.5
  done
  return 1
}

status_snapshot() {
  echo "[INFO] 正在采集 Jetson 导航状态（通常需要 2~6 秒）……"
  "${SSH[@]}" "bash -s" <<'REMOTE'
set +u
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh >/dev/null 2>&1
set -u
timeout 15s python3 - <<'PY'
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

rclpy.init()
node = Node("patrol_status_snapshot")
state = {
    "localization_ready": None,
    "visual_state": None,
    "bridge_ready": None,
    "goal_active": None,
    "goal_pending": None,
    "bridge_state": None,
    "gate_state": None,
    "armed": None,
    "raw_command": None,
}

latched_qos = QoSProfile(depth=1)
latched_qos.reliability = ReliabilityPolicy.RELIABLE
latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

node.create_subscription(
    Bool,
    "/patrol/localization_ready",
    lambda message: state.update(localization_ready=bool(message.data)),
    latched_qos,
)
node.create_subscription(
    String,
    "/patrol/visual_init_state",
    lambda message: state.update(visual_state=message.data),
    10,
)
node.create_subscription(
    Bool,
    "/patrol/goal_bridge_ready",
    lambda message: state.update(bridge_ready=bool(message.data)),
    latched_qos,
)
node.create_subscription(
    Bool,
    "/patrol/goal_active",
    lambda message: state.update(goal_active=bool(message.data)),
    latched_qos,
)
node.create_subscription(
    Bool,
    "/patrol/goal_pending",
    lambda message: state.update(goal_pending=bool(message.data)),
    latched_qos,
)
node.create_subscription(
    String,
    "/patrol/goal_bridge_state",
    lambda message: state.update(bridge_state=message.data),
    latched_qos,
)
node.create_subscription(
    String,
    "/patrol/gate_state",
    lambda message: state.update(gate_state=message.data),
    10,
)
node.create_subscription(
    Bool,
    "/patrol/armed",
    lambda message: state.update(armed=bool(message.data)),
    10,
)

def raw_callback(message):
    state["raw_command"] = (
        float(message.linear.x),
        float(message.linear.y),
        float(message.angular.z),
    )

node.create_subscription(Twist, "/patrol/cmd_vel_raw", raw_callback, 20)

required = (
    "localization_ready",
    "visual_state",
    "bridge_ready",
    "goal_active",
    "goal_pending",
    "bridge_state",
    "gate_state",
    "armed",
)
deadline = time.monotonic() + 7.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if all(state[key] is not None for key in required):
        break

def boolean(value):
    if value is None:
        return "UNKNOWN"
    return "true" if value else "false"

print("[STATUS] localization_ready=" + boolean(state["localization_ready"]))
print("[STATUS] visual_init=" + str(state["visual_state"] or "UNKNOWN"))
print("[STATUS] goal_bridge_ready=" + boolean(state["bridge_ready"]))
print("[STATUS] goal_active=" + boolean(state["goal_active"]))
print("[STATUS] goal_pending=" + boolean(state["goal_pending"]))
print("[STATUS] goal_bridge=" + str(state["bridge_state"] or "UNKNOWN"))
print("[STATUS] velocity_gate=" + str(state["gate_state"] or "UNKNOWN"))
print("[STATUS] armed=" + boolean(state["armed"]))
if state["raw_command"] is None:
    print("[STATUS] nav_raw_cmd=NO_SAMPLE")
else:
    x, y, yaw = state["raw_command"]
    print("[STATUS] nav_raw_cmd=x:{:.3f},y:{:.3f},yaw:{:.3f}".format(x, y, yaw))

missing = [key for key in required if state[key] is None]
if missing:
    print("[WARN] 状态采集超时，未收到: " + ", ".join(missing))

node.destroy_node()
rclpy.shutdown()
raise SystemExit(2 if missing else 0)
PY
REMOTE
}

status_log_fallback() {
  local status_log_dir=$LOG_DIR
  if [[ ! -d "$status_log_dir" ]]; then
    status_log_dir=$(ls -td "$PROJECT_DIR/logs/lite3_indoor_nav_${MAP_ID}_"* "/tmp/lite3_indoor_nav_${MAP_ID}_"* 2>/dev/null | head -n 1 || true)
  fi
  if [[ -z "$status_log_dir" || ! -d "$status_log_dir" ]]; then
    echo "[WARN] 没有找到当前地图的本地会话日志"
    return 0
  fi

  echo "[FALLBACK] DDS 实时订阅不可用，显示日志中的最近状态："
  echo "[FALLBACK] 日志目录=$status_log_dir"
  local line_value
  line_value=$(grep -Ei 'READY:|FAILED:|BLOCKED:|WAITING_NDT:|COLLECTING:' \
    "$status_log_dir/visual_initializer.log" 2>/dev/null | tail -n 1 || true)
  echo "[FALLBACK] 视觉初始化=${line_value:-NO_LOG_ENTRY}"
  line_value=$(grep -Ei 'action server|accepted bridged goal|rejected|result|cancel' \
    "$status_log_dir/goal_bridge.log" 2>/dev/null | tail -n 1 || true)
  echo "[FALLBACK] 目标桥=${line_value:-NO_LOG_ENTRY}"
  line_value=$(grep -Ei 'Received goal|Navigation failed|Navigation succeeded|Aborting handle' \
    "$status_log_dir/nav2.log" 2>/dev/null | tail -n 1 || true)
  echo "[FALLBACK] Nav2=${line_value:-NO_LOG_ENTRY}"
  line_value=$(grep -Ei 'started DISARMED|velocity output ARMED|gate fault' \
    "$status_log_dir/velocity_gate.log" 2>/dev/null | tail -n 1 || true)
  echo "[FALLBACK] 速度门=${line_value:-NO_LOG_ENTRY}"
  echo "[WARN] 日志回退只用于诊断，不能作为 ARM 的放行依据。"
}

wait_gate_disarmed() {
  local gate_log=$LOG_DIR/velocity_gate.log
  for _ in $(seq 1 30); do
    if grep -F 'velocity gate started DISARMED' "$gate_log" >/dev/null 2>&1 && \
       grep -F 'zero-command heartbeat active on /cmd_vel_corrected' "$gate_log" >/dev/null 2>&1 && \
       "${SSH[@]}" "pgrep -f '/lite3_patrol_nav/velocity_gate' >/dev/null"; then
      if [[ "$USE_VENDOR_VOA" == 0 ]] || grep -F '[VOA] correlated sensor-valid response received' "$gate_log" >/dev/null 2>&1; then
        if [[ "$DYNAMIC_OBSTACLES" == 1 ]] && ! grep -F '[VOA_COSTMAP] fresh obstacle projection received' "$gate_log" >/dev/null 2>&1; then
          sleep 1
          continue
        fi
        echo "[PASS] 速度门已启动且为 DISARMED；仅向 /cmd_vel_corrected 发布零速心跳；VOA=$USE_VENDOR_VOA"
        return 0
      fi
    fi
    sleep 1
  done
  return 1
}

disarm() {
  "${SSH[@]}" \
    "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; timeout 8s ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: false}'" \
    2>/dev/null || true
}

stop_remote_runtime() {
  "${SSH[@]}" "bash -s" <<'REMOTE'
set +e
patterns=(
  '^bash -s -- /rgbd_visual_localizer$'
  '^bash -s -- /visual_pose_initializer$'
  '/scripts/leg_odom_tf_bridge.py'
  '/scripts/voa_costmap_bridge.py'
  '/scripts/goal_pose_bridge.py'
  '/navigation/start_visual_initialization.sh'
  '/lite3_patrol_nav/visual_pose_initializer'
  '/navigation/start_visual_localizer.sh'
  '/lite3_patrol_nav/rgbd_visual_localizer'
  '/navigation/start_visual_initialization.sh'
  '/lite3_patrol_nav/visual_pose_initializer'
  '/navigation/start_visual_localizer.sh'
  '/lite3_patrol_nav/rgbd_visual_localizer'
  '/navigation/start_gate_disarmed.sh'
  '/navigation/start_vendor_voa.sh'
  '/scripts/voa_camera_tf.py'
  '/lite3_patrol_project/voa_overlay_ws/install/voa/lib/voa/voa_composition'
  '/lite3_patrol_nav/velocity_gate'
  '/navigation/start_map_display_only.sh'
  'manual_initial_pose_map_server'
  '/navigation/start_nav_isolated.sh'
  '/navigation/start_localization_only.sh'
  'lite3_patrol_nav patrol_navigation.launch.py'
  'hdl_localization_composition'
  'livox_ros_driver2_node'
)
signal_groups() {
  local signal=$1 pattern pids pid pgid
  declare -A seen=()
  for pattern in "${patterns[@]}"; do
    pids=$(pgrep -f "$pattern" || true)
    for pid in $pids; do
      pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
      [[ -n "$pgid" && -z "${seen[$pgid]:-}" ]] || continue
      seen[$pgid]=1
      kill -"$signal" -- "-$pgid" 2>/dev/null || true
    done
  done
}
signal_groups INT
sleep 3
signal_groups TERM
sleep 2
signal_groups KILL
REMOTE
}

stop_local_sessions() {
  set +e
  disarm >/dev/null
  stop_remote_runtime
  stop_map_display
  # stop_map_display restores errexit. Children may already have exited after
  # remote shutdown; their missing PIDs must not turn a passed test into failure.
  set +e
  for pid in \
    "$GATE_SSH_PID" \
    "$GOAL_BRIDGE_SSH_PID" \
    "$NAV_SSH_PID" \
    "$VISUAL_INITIALIZER_SSH_PID" \
    "$VISUAL_LOCALIZER_SSH_PID" \
    "$LOCALIZATION_SSH_PID"; do
    [[ -n "$pid" ]] || continue
    kill -INT "$pid" 2>/dev/null
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.15
    done
    kill -TERM "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
  done
  [[ -n "$RVIZ_SSH_PID" ]] && kill -TERM "$RVIZ_SSH_PID" 2>/dev/null
  [[ -n "$RVIZ_POINTCLOUD_RELAY_PID" ]] && \
    kill -TERM "$RVIZ_POINTCLOUD_RELAY_PID" 2>/dev/null
  if [[ -n "$RVIZ_SSH_BRIDGE_PID" ]]; then
    kill -TERM "$RVIZ_SSH_BRIDGE_PID" 2>/dev/null
    wait "$RVIZ_SSH_BRIDGE_PID" 2>/dev/null
  fi
  if [[ -n "$NAV_RESULT_NOTIFIER_PID" ]]; then
    kill -TERM "$NAV_RESULT_NOTIFIER_PID" 2>/dev/null
    wait "$NAV_RESULT_NOTIFIER_PID" 2>/dev/null
  fi
  set -e
  return 0
}

cleanup() {
  if (( SESSION_ACTIVE == 1 )); then
    SESSION_ACTIVE=0
    trap - EXIT INT TERM HUP
    echo
    echo "[INFO] 正在 DISARM 并关闭本脚本启动的节点……"
    stop_local_sessions
  fi
}
trap cleanup EXIT
# An interrupt during preflight must exit, not resume startup after cleanup.
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP

show_failure_logs() {
  local file
  for file in "$LOG_DIR"/*.log; do
    [[ -r "$file" ]] || continue
    echo "===== $file ====="
    tail -n 60 "$file"
  done
}

start_rviz() {
  if [[ ${LITE3_RVIZ_ENABLED:-1} == 0 ]]; then
    echo '[INFO] RViz display/SSH forwarding disabled for saved-route execution'
    return 0
  fi
  local local_rviz_config=$LOG_DIR/dr_nav2_goal_topic.rviz
  [[ -x /opt/ros/galactic/bin/rviz2 ]] || \
    die "本地 ROS 2 Galactic RViz 不存在，不能使用低负载目标桥模式"
  scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$SSH_KEY" \
    "$ROBOT:/home/ysc/lite_cog_ros2/nav/src/dr_nav2/rviz/dr_nav2.rviz" \
    "$local_rviz_config" || die "无法取得 RViz 配置"

  # Remove the incompatible Galactic Nav2 action panel and replace its goal
  # tool with the distro-stable PoseStamped topic tool.  The Jetson bridge
  # publishes to an isolated request topic.  Nav2 itself subscribes to
  # /goal_pose, so using that name would bypass the ARM-delayed bridge.
  sed -i \
    '/^  - Class: nav2_rviz_plugins\/Navigation 2$/,+1d' \
    "$local_rviz_config"
  sed -i \
    '/^    - Class: nav2_rviz_plugins\/GoalTool$/c\    - Class: rviz_default_plugins/SetGoal\
      Topic:\
        Depth: 5\
        Durability Policy: Volatile\
        History Policy: Keep Last\
        Reliability Policy: Reliable\
        Value: /patrol/goal_request' \
    "$local_rviz_config"
  sed -i \
    's#Value: /goal_pose$#Value: /patrol/goal_request#' \
    "$local_rviz_config"
  if [[ "$RVIZ_SSH_BRIDGE" == 1 ]]; then
    (
      set +u
      source /opt/ros/galactic/setup.bash
      set -u
      exec env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1 \
        python3 "$SCRIPT_DIR/rviz_ssh_bridge.py" --robot "$ROBOT" --key "$SSH_KEY" \
          --parent-pid "$$"
    ) >"$LOG_DIR/rviz_ssh_bridge.log" 2>&1 &
    RVIZ_SSH_BRIDGE_PID=$!
    sleep 1
    kill -0 "$RVIZ_SSH_BRIDGE_PID" 2>/dev/null || die "RViz 无线 SSH 数据桥启动失败"
    echo "[PASS] RViz 使用无线 SSH 数据桥；本地显示与远端 DDS 分离"
  fi
  # Do not connect low-frame-rate RViz directly to the remote reliable
  # Mid360 stream: that previously caused DDS back-pressure during Nav2
  # bringup.  A local relay receives the already aligned/downsampled cloud
  # with sensor-data QoS, then republishes it reliably on the local host.
  if [[ ${LITE3_RVIZ_POINTCLOUD:-1} == 1 ]]; then
    sed -i \
      's#Value: /rslidar_points$#Value: /patrol/rviz_aligned_points#' \
      "$local_rviz_config"
    grep -q 'Value: /patrol/rviz_aligned_points$' "$local_rviz_config" || \
      die "RViz 点云显示未能切换到本地对齐点云中继"
    (
      set +u
      source /opt/ros/galactic/setup.bash
      set -u
      exec env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY="$RVIZ_LOCALHOST_ONLY" \
        python3 "$SCRIPT_DIR/rviz_pointcloud_relay.py" \
          --input-topic /aligned_points \
          --output-topic /patrol/rviz_aligned_points \
          --parent-pid "$$"
    ) >"$LOG_DIR/rviz_pointcloud_relay.log" 2>&1 &
    RVIZ_POINTCLOUD_RELAY_PID=$!
    sleep 1
    kill -0 "$RVIZ_POINTCLOUD_RELAY_PID" 2>/dev/null || \
      die "RViz 点云中继启动失败"
  else
    sed -i \
      's#Value: /rslidar_points$#Value: /patrol/rviz_raw_lidar_disabled#' \
      "$local_rviz_config"
  fi
  grep -q 'Value: /patrol/goal_request$' "$local_rviz_config" || \
    die "RViz 目标工具未能切换到隔离话题 /patrol/goal_request"

  if [[ -z "${DISPLAY:-}" ]]; then
    echo "[WARN] 当前终端没有 DISPLAY，无法自动打开 RViz。"
    echo "请在本地桌面终端执行："
    echo "source /opt/ros/galactic/setup.bash"
    echo "ROS_LOCALHOST_ONLY=$RVIZ_LOCALHOST_ONLY RMW_IMPLEMENTATION=rmw_cyclonedds_cpp rviz2 -d '$local_rviz_config'"
    return 0
  fi
  (
    set +u
    source /opt/ros/galactic/setup.bash
    set -u
    exec env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY="$RVIZ_LOCALHOST_ONLY" \
      rviz2 -d "$local_rviz_config"
  ) >"$LOG_DIR/rviz.log" 2>&1 &
  RVIZ_SSH_PID=$!
  echo "[PASS] RViz 在本地启动；目标仅通过 /patrol/goal_request 桥接"
  if [[ ${LITE3_RVIZ_POINTCLOUD:-1} == 1 ]]; then
    echo "[PASS] RViz 显示低负载 /aligned_points 点云"
  else
    echo "[PASS] RViz 点云显示已通过 LITE3_RVIZ_POINTCLOUD=0 关闭"
  fi
}

stop_visual_frontend_after_initialization() {
  # Only the one-shot visual frontend exits. NDT, odometry and the readiness
  # guard stay active. Reinitialization requires a fresh navigation startup.
  "${SSH[@]}" 'bash -s' <<'REMOTE'
set -e
pattern='^[^ ]*python3 /home/ysc/lite3_patrol_project/patrol_nav_ws/install_merged/lib/lite3_patrol_nav/rgbd_visual_localizer([[:space:]]|$)'
pids=$(pgrep -f "$pattern" || true)
for pid in $pids; do kill -TERM "$pid"; done
for _ in {1..20}; do
  pgrep -f "$pattern" >/dev/null || exit 0
  sleep 0.25
done
echo '[FAIL] Visual frontend did not exit; refusing extra navigation load' >&2
exit 1
REMOTE
  if [[ -n "$VISUAL_LOCALIZER_SSH_PID" ]]; then
    wait "$VISUAL_LOCALIZER_SSH_PID" || true
    VISUAL_LOCALIZER_SSH_PID=
  fi
  echo '[PASS] 初始视觉定位进程已退出，释放模型与特征缓存；NDT 和里程计继续定位'
}

start_session() {
  local mode=${1:-navigation}
  # The initializer has already confirmed accepted scans. Observe its first
  # fresh READY here; the one full-load stability window runs after Nav2 starts.
  local initial_stable_seconds=0
  local initialization_test_only=0
  if [[ "$mode" == relocalize ]]; then
    initial_stable_seconds=10
    initialization_test_only=1
  fi
  validate_inputs
  mkdir -p "$LOG_DIR"
  deploy_runtime
  static_preflight
  SESSION_ACTIVE=1

  title "启动 Mid360、IMU 和修正后的 NDT"
  "${SSH_BG[@]}" \
    "exec env LITE3_PATROL_MAP_ID='$MAP_ID' LITE3_NDT_RESOLUTION='$NDT_RESOLUTION' LITE3_NDT_FINE_REFINE='$FINE_REGISTRATION' LITE3_DOWNSAMPLE_RESOLUTION='$DOWNSAMPLE_RESOLUTION' LITE3_LOCALIZATION_RUNTIME='$LOCALIZATION_RUNTIME' bash '$REMOTE_PROJECT/navigation/start_localization_only.sh'" \
    >"$LOG_DIR/localization.log" 2>&1 &
  LOCALIZATION_SSH_PID=$!
  sleep 2
  if ! kill -0 "$LOCALIZATION_SSH_PID" 2>/dev/null || ! wait_localization; then
    show_failure_logs
    die "定位链路启动失败"
  fi

  title "启动视觉关键帧检索与初始化守卫"
  echo "[INFO] 视觉初始化期间请保持机器狗静止，并保证前置相机视野无遮挡。"
  "${SSH_BG[@]}" \
    "exec env LITE3_PATROL_MAP_ID='$VISUAL_MAP_ID' LITE3_VISUAL_SEARCH_STRATEGY='$VISUAL_SEARCH_STRATEGY' LITE3_VISUAL_BACKEND='$VISUAL_BACKEND' bash '$REMOTE_PROJECT/navigation/start_visual_localizer.sh'" \
    >"$LOG_DIR/visual_localizer.log" 2>&1 &
  VISUAL_LOCALIZER_SSH_PID=$!
  if ! wait_visual_node /rgbd_visual_localizer; then
    show_failure_logs
    die "视觉关键帧检索节点未启动"
  fi

  "${SSH_BG[@]}" \
    "exec env LITE3_PATROL_MAP_ID='$MAP_ID' LITE3_RELOCALIZATION_TEST_ONLY='$initialization_test_only' bash '$REMOTE_PROJECT/navigation/start_visual_initialization.sh'" \
    >"$LOG_DIR/visual_initializer.log" 2>&1 &
  VISUAL_INITIALIZER_SSH_PID=$!
  if ! wait_visual_node /visual_pose_initializer; then
    show_failure_logs
    die "视觉初始化守卫未启动"
  fi

  title "请求视觉粗定位并等待 NDT 确认"
  if ! request_visual_initialization || ! wait_visual_ready "$initial_stable_seconds"; then
    show_failure_logs
    die "视觉初始重定位未通过；机器狗保持静止，不要发送导航目标"
  fi

  if [[ "$mode" == relocalize ]]; then
    title "RELOCALIZATION PASSED（初始重定位测试通过）"
    echo "地图：$MAP_ID；视觉地图：$VISUAL_MAP_ID"
    echo "视觉粗定位已由 NDT 确认，且连续保持就绪 10 秒。"
    echo "日志：$LOG_DIR"
    return 0
  fi

  if [[ "$DYNAMIC_OBSTACLES" == 1 ]]; then
    stop_visual_frontend_after_initialization
  fi

  if [[ ${LITE3_RVIZ_ENABLED:-1} != 0 ]]; then
  title "启动临时地图显示与 RViz"
  "${SSH_BG[@]}" \
    "exec env LITE3_PATROL_MAP_ID='$MAP_ID' bash '$REMOTE_PROJECT/navigation/start_map_display_only.sh'" \
    >"$LOG_DIR/map_display.log" 2>&1 &
  MAP_DISPLAY_SSH_PID=$!
  sleep 2
  if ! kill -0 "$MAP_DISPLAY_SSH_PID" 2>/dev/null || ! wait_map_display; then
    show_failure_logs
    die "临时地图显示启动失败"
  fi

  start_rviz
  if [[ -n "$RVIZ_SSH_PID" ]]; then
    sleep 2
    if ! kill -0 "$RVIZ_SSH_PID" 2>/dev/null; then
      show_failure_logs
      die "本地 RViz 启动失败"
    fi
  fi
  else
    echo '[INFO] 无 RViz 模式：直接使用 Nav2 地图服务，跳过临时显示服务'
  fi

  title "切换到完整隔离 Nav2"
  stop_map_display
  "${SSH_BG[@]}" \
    "exec env LITE3_PATROL_MAP_ID='$MAP_ID' LITE3_DYNAMIC_OBSTACLES='$DYNAMIC_OBSTACLES' LITE3_GROUND_MODE='$GROUND_MODE' bash '$REMOTE_PROJECT/navigation/start_nav_isolated.sh'" \
    >"$LOG_DIR/nav2.log" 2>&1 &
  NAV_SSH_PID=$!
  sleep 2
  if ! kill -0 "$NAV_SSH_PID" 2>/dev/null || ! wait_nav2; then
    show_failure_logs
    die "定位已通过，但完整隔离 Nav2 启动失败"
  fi

  title "启动轻量 Foxy 导航目标桥"
  "${SSH_BG[@]}" \
    "source '$REMOTE_PROJECT/navigation/patrol_env.sh'; exec python3 '$REMOTE_PROJECT/scripts/goal_pose_bridge.py' --ros-args -p active_map_id:='$MAP_ID'" \
    >"$LOG_DIR/goal_bridge.log" 2>&1 &
  GOAL_BRIDGE_SSH_PID=$!
  sleep 1
  if ! kill -0 "$GOAL_BRIDGE_SSH_PID" 2>/dev/null || ! wait_goal_bridge; then
    show_failure_logs
    die "/patrol/goal_request 到 Foxy NavigateToPose 的桥接失败；禁止发送目标"
  fi

  title "启动默认 DISARMED 的速度门"
  "${SSH_BG[@]}" "LITE3_DYNAMIC_OBSTACLES='$DYNAMIC_OBSTACLES' LITE3_USE_VENDOR_VOA='$USE_VENDOR_VOA' exec bash '$REMOTE_PROJECT/navigation/start_gate_disarmed.sh'" \
    >"$LOG_DIR/velocity_gate.log" 2>&1 &
  GATE_SSH_PID=$!
  sleep 1
  if ! kill -0 "$GATE_SSH_PID" 2>/dev/null || ! wait_gate_disarmed; then
    show_failure_logs
    die "速度门未能以 DISARMED 状态启动"
  fi

  python3 "$SCRIPT_DIR/navigation_result_notifier.py" \
    --log "$LOG_DIR/goal_bridge.log" &
  NAV_RESULT_NOTIFIER_PID=$!
  sleep 0.2
  kill -0 "$NAV_RESULT_NOTIFIER_PID" 2>/dev/null || \
    die "导航结果提醒器启动失败"

  title "确认完整 Nav2 负载下定位持续稳定 10 秒"
  # Preserve the confirmed initial pose. The initializer keeps checking NDT
  # status and leg/filtered odometry, and revokes READY on failure. Observe
  # fresh readiness heartbeats under full load without resetting localization.
  if ! wait_visual_ready 10 tracking; then
    show_failure_logs
    die "Nav2 启动后定位未保持就绪；禁止 ARM"
  fi
  title "NAVIGATION READY（默认不会运动）"
  cat <<EOF
雷达地图：$MAP_ID
视觉地图：$VISUAL_MAP_ID
初始定位：视觉粗定位已由 NDT 确认
日志：$LOG_DIR

目标操作：
1. 先确认 RViz 中地图、点云和机器狗位置与现实一致；不要再点击 2D Pose Estimate；
2. 在 RViz 点击 2D Goal Pose，选择白色自由区内距当前位置 1~2 米的目标；
3. 确认预览路径正确，手持遥控器并在本终端输入一次 ARM；
4. 到达后终端会输出 [NAV DONE]；机器狗保持 ARMED 零速待命，直接点击下一个目标；
5. 结束测试输入 DISARM；故障后先 STATUS 检查。定位 FAILED 需 QUIT 后重新 start 完成视觉初始化；CLEAR 只清速度门故障。

控制命令：STATUS / CLEAR / ARM / DISARM / CANCEL / QUIT
EOF

  while true; do
    read -r -p "导航控制> " command || break
    case ${command^^} in
      STATUS)
        if ! status_snapshot; then
          echo "[WARN] 无法实时读取全部 Jetson 导航状态；机器狗状态未改变。"
          status_log_fallback
        fi
        ;;
      CLEAR)
        echo "清除速度门锁存故障；速度门仍保持 DISARMED。"
        "${SSH[@]}" "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; timeout 8s ros2 service call /patrol/clear_fault std_srvs/srv/Trigger '{}'"
        ;;
      ARM)
        echo "ARM 后机器狗会真实运动。确认目标只发送一次、路径正确、现场净空。"
        read -r -p "手持官方遥控器并准备急停，请再次输入 ARM: " confirm
        if [[ ${confirm^^} == ARM ]]; then
          "${SSH[@]}" "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; timeout 8s ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: true}'"
        else
          echo "已取消 ARM"
        fi
        ;;
      DISARM)
        disarm
        ;;
      CANCEL)
        disarm
        "${SSH[@]}" "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; timeout 8s ros2 service call /patrol/cancel_goal std_srvs/srv/Trigger '{}'" || true
        ;;
      QUIT|Q|EXIT)
        disarm >/dev/null
        break
        ;;
      *) echo "请输入 STATUS、CLEAR、ARM、DISARM、CANCEL 或 QUIT" ;;
    esac
  done
}

remote_status() {
  validate_inputs
  echo "MAP_ID=$MAP_ID"
  title "实时导航状态快照"
  if ! status_snapshot; then
    echo "[WARN] 15 秒内无法实时读取全部 Jetson 导航状态；机器狗状态未改变。"
    status_log_fallback
  fi
}

remote_arm() {
  validate_inputs
  title "首次目标：解锁连续导航会话"
  echo "ARM 后机器狗会真实运动；此后在 DISARM 前，新目标会直接执行。"
  read -r -p "手持官方遥控器并准备急停，请输入 ARM: " confirm
  [[ ${confirm^^} == ARM ]] || die "已取消 ARM；机器狗保持 DISARMED"
  "${SSH[@]}" \
    "source '$REMOTE_PROJECT/navigation/patrol_env.sh' >/dev/null 2>&1; timeout 8s ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: true}'"
}

remote_disarm() {
  validate_inputs
  title "结束连续导航会话"
  disarm
  echo "[PASS] 已请求 DISARM；现有目标会被取消，底盘只接收零速"
}

remote_stop() {
  validate_inputs
  title "停止室内定位与导航会话"
  disarm
  stop_remote_runtime
  echo "[PASS] 已停止本项目的定位、地图显示与导航进程；请用 status 确认"
}

case "$ACTION" in
  install)
    validate_inputs
    deploy_runtime
    ;;
  preflight)
    validate_inputs
    deploy_runtime
    static_preflight
    ;;
  relocalize) start_session relocalize ;;
  start) start_session ;;
  arm) remote_arm ;;
  disarm) remote_disarm ;;
  status) remote_status ;;
  stop) remote_stop ;;
  -h|--help|help) usage ;;
  *) usage; die "未知操作: $ACTION" ;;
esac
