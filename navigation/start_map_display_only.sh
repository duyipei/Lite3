#!/usr/bin/env bash
# Publishes only the static occupancy map needed for manual initial-pose input.
# No planner, controller, costmap, localization node, or velocity publisher is
# started here.
set -Eeuo pipefail

PROJECT=/home/ysc/lite3_patrol_project
MAP_ID=${LITE3_PATROL_MAP_ID:-site_fusion_20260909_001}
MAP_DIR=$PROJECT/navigation/$MAP_ID
source "$PROJECT/navigation/patrol_env.sh"

if [[ ! "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "invalid map id: $MAP_ID" >&2
  exit 1
fi
if [[ ! -r "$MAP_DIR/patrol_map.yaml" ]]; then
  echo "missing map artifact: $MAP_DIR/patrol_map.yaml" >&2
  exit 1
fi

if pgrep -f '[n]av2_map_server/map_server' >/dev/null || \
   pgrep -f '[m]anual_initial_pose_map_server' >/dev/null; then
  echo "refusing to start: a map server process already exists" >&2
  exit 1
fi

map_pid=
cleanup() {
  if [[ -n "${map_pid:-}" ]]; then
    kill -INT -- "-$map_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-$map_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-$map_pid" 2>/dev/null || true
    wait "$map_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

setsid ros2 launch lite3_patrol_nav map_display.launch.py \
  map_yaml:="$MAP_DIR/patrol_map.yaml" &
map_pid=$!
wait "$map_pid"
