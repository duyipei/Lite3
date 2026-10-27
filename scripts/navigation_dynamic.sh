#!/usr/bin/env bash
# Optional dynamic-obstacle navigation; startup always remains DISARMED.
set -Eeuo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-start} == check ]]; then
  MAP_ID=${2:-outdoor_20260921_003_area_review}
  [[ "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ ]] || { echo 'invalid map id' >&2; exit 1; }
  PROJECT=$(cd "$SCRIPT_DIR/.." && pwd)
  GROUND_ARGS=()
  if [[ "$MAP_ID" == outdoor_20260921_003_area_review ]]; then
    GROUND_ARGS=(--ground-relative)
    python3 "$SCRIPT_DIR/costmap_ground_filter.py" "$PROJECT/navigation/$MAP_ID" --check
  fi
  exec python3 "$SCRIPT_DIR/make_dynamic_nav2_config.py" \
    "$PROJECT/navigation/$MAP_ID/nav2_safe.yaml" \
    "$PROJECT/logs/dynamic_preflight/$MAP_ID/nav2_dynamic.yaml" \
    --behavior-tree "$PROJECT/navigation/navigate_dynamic_obstacles.xml" "${GROUND_ARGS[@]}"
fi
export LITE3_DYNAMIC_OBSTACLES=1
export LITE3_USE_VENDOR_VOA=1
exec bash "$SCRIPT_DIR/navigation_start.sh" "${1:-start}" "${2:-outdoor_20260921_003_area_review}"
