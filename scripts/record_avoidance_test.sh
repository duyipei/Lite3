#!/usr/bin/env bash
# Jetson only. Recording cannot arm or start motion. Run before any ARM.
set -Eeuo pipefail
PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
OUT=${1:?Usage: bash scripts/record_avoidance_test.sh NEW_BAG_DIRECTORY [SECONDS]}
SECONDS_TO_RECORD=${2:-180}
[[ "$SECONDS_TO_RECORD" =~ ^[1-9][0-9]*$ && ! -e "$OUT" ]] || {
  echo '[FAIL] Use a new directory and positive integer duration' >&2; exit 1; }
source "$PROJECT/navigation/patrol_env.sh"
# Capture transient maps even if the recorder starts after map_server.
QOS_FILE=$(mktemp)
trap 'rm -f "$QOS_FILE"' EXIT
cat > "$QOS_FILE" <<'QOS'
/map:
  reliability: reliable
  durability: transient_local
/local_costmap/costmap:
  reliability: reliable
  durability: transient_local
/global_costmap/costmap:
  reliability: reliable
  durability: transient_local
/local_costmap/costmap_raw:
  reliability: reliable
  durability: transient_local
/global_costmap/costmap_raw:
  reliability: reliable
  durability: transient_local
/tf_static:
  reliability: reliable
  durability: transient_local
QOS
status=0
timeout --signal=INT "$SECONDS_TO_RECORD" ros2 bag record -o "$OUT" \
  --qos-profile-overrides-path "$QOS_FILE" \
  /patrol/voa/grid_map /patrol/voa/costmap_obstacles /patrol/voa/costmap_heartbeat \
  /patrol/cmd_vel_raw /cmd_vel_corrected \
  /patrol/gate_state /patrol/armed /patrol/localization_ready /patrol/registration_odom \
  /patrol/goal_bridge_state /patrol/goal_active /patrol/goal_request \
  /local_costmap/costmap /global_costmap/costmap /map /plan \
  /local_costmap/costmap_raw /global_costmap/costmap_raw \
  /tf /tf_static /odom /leg_odom2 /patrol/leg_odom_sampled /imu/data /rosout || status=$?
[[ "$status" == 0 || "$status" == 124 || "$status" == 130 ]] || exit "$status"
