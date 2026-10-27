#!/usr/bin/env bash
# Starts Nav2 with all velocity output remapped to /patrol/cmd_vel_raw.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
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

cmd_publishers=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count:/ {print $3}')
if [[ "${cmd_publishers:-0}" != 0 ]]; then
  echo "refusing to start: /cmd_vel already has ${cmd_publishers} publisher(s)" >&2
  exit 1
fi
corrected_publishers=$(ros2 topic info /cmd_vel_corrected 2>/dev/null | awk '/Publisher count:/ {print $3}')
if [[ "${corrected_publishers:-0}" != 0 ]]; then
  echo "refusing to start: /cmd_vel_corrected already has ${corrected_publishers} publisher(s)" >&2
  exit 1
fi
# Check both conditions in one bounded rclpy process.  Full Nav2 is never
# started before manual NDT initialization: costmaps require map -> base_link
# while activating and can otherwise block the lifecycle manager.
preflight=$(timeout 25s env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 - <<'PY' || true
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('patrol_nav_preflight')
buffer = Buffer()
listener = TransformListener(buffer, node)
state = {'ready': False}
ready_qos = QoSProfile(depth=1)
ready_qos.reliability = ReliabilityPolicy.RELIABLE
ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
subscription = node.create_subscription(
    Bool, '/patrol/localization_ready',
    lambda msg: state.update(ready=msg.data), ready_qos)
deadline = time.monotonic() + 20.0
ok = False
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        buffer.lookup_transform('map', 'base_link', Time())
        tf_ok = True
    except Exception:
        tf_ok = False
    if tf_ok and state['ready']:
        ok = True
        break
print('NAV_PREFLIGHT_OK' if ok else 'NAV_PREFLIGHT_FAIL')
node.destroy_node()
rclpy.shutdown()
PY
)
if [[ "$preflight" != *"NAV_PREFLIGHT_OK"* ]]; then
  echo "refusing to start: map TF or localization confirmation is unavailable" >&2
  exit 1
fi

PARAMS_FILE=$MAP_DIR/nav2_safe.yaml
BT_ARGS=()
DYNAMIC_OBSTACLES=${LITE3_DYNAMIC_OBSTACLES:-0}
[[ "$DYNAMIC_OBSTACLES" == 0 || "$DYNAMIC_OBSTACLES" == 1 ]] || exit 1
GROUND_ARGS=()
if [[ "$DYNAMIC_OBSTACLES" == 1 ]]; then
  PARAMS_FILE=$MAP_DIR/nav2_dynamic.yaml
  if [[ "$MAP_ID" == outdoor_20260921_003_area_review ]]; then
    GROUND_ARGS=(--ground-relative)
    python3 "$PROJECT/scripts/costmap_ground_filter.py" "$MAP_DIR" --check
  fi
  python3 "$PROJECT/scripts/make_dynamic_nav2_config.py" "$MAP_DIR/nav2_safe.yaml" \
    "$PARAMS_FILE" --behavior-tree "$PROJECT/navigation/navigate_dynamic_obstacles.xml" "${GROUND_ARGS[@]}"
  BT_ARGS=("default_bt_xml_filename:=$PROJECT/navigation/navigate_dynamic_obstacles.xml")
fi
nav_pid=
ground_pid=
cleanup() {
  if [[ -n "${nav_pid:-}" ]]; then
    kill -INT -- "-$nav_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-$nav_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-$nav_pid" 2>/dev/null || true
    wait "$nav_pid" 2>/dev/null || true
  fi
  if [[ -n "${ground_pid:-}" ]]; then
    kill -INT -- "-$ground_pid" 2>/dev/null || true
    sleep .2
    kill -TERM -- "-$ground_pid" 2>/dev/null || true
    wait "$ground_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

if (( ${#GROUND_ARGS[@]} )); then
  setsid env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
    python3 "$PROJECT/scripts/costmap_ground_filter.py" "$MAP_DIR" &
  ground_pid=$!
fi

# Keep localization and the velocity gate at normal priority while the Nav2
# processes load plugins/maps. Inherit bounded math thread pools into Nav2.
setsid nice -n 5 env OMP_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0 \
  OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  ros2 launch lite3_patrol_nav patrol_navigation.launch.py \
  map_yaml:="$MAP_DIR/patrol_map.yaml" \
  globalmap_pcd:="$MAP_DIR/localization_map.pcd" \
  params_file:="$PARAMS_FILE" \
  "${BT_ARGS[@]}" \
  start_localization:=False start_navigation:=True &
nav_pid=$!
if [[ -n "$ground_pid" ]]; then
  # A failed filter must not leave Nav2 running on expired obstacle evidence.
  while kill -0 "$nav_pid" 2>/dev/null; do
    if ! kill -0 "$ground_pid" 2>/dev/null; then
      echo 'ground obstacle filter exited; stopping Nav2' >&2
      exit 1
    fi
    sleep 1
  done
fi
wait "$nav_pid"
