#!/usr/bin/env bash
# Optional dynamic-obstacle navigation; startup always remains DISARMED.
# LITE3_OBSTACLE_BACKEND=factory_only (default) uses vendor VOA perception
# only. private_combined retains the previous private VOA speed correction and
# Mid-360/camera costmap integration.
set -Eeuo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BACKEND=${LITE3_OBSTACLE_BACKEND:-factory_only}
case "$BACKEND" in
  factory_only|private_combined) ;;
  *) echo "invalid LITE3_OBSTACLE_BACKEND: $BACKEND" >&2; exit 1 ;;
esac
DEFAULT_MAP_ID=outdoor_20260921_003_area_review
if [[ ${1:-start} == check ]]; then
  MAP_ID=${2:-$DEFAULT_MAP_ID}
  [[ "$MAP_ID" =~ ^[A-Za-z0-9._-]+$ ]] || { echo 'invalid map id' >&2; exit 1; }
  PROJECT=$(cd "$SCRIPT_DIR/.." && pwd)
  GROUND_ARGS=()
  MODE_ARGS=(--factory-only)
  if [[ "$BACKEND" == private_combined ]]; then
    MODE_ARGS=()
    if [[ "$MAP_ID" == outdoor_20260921_003_area_review ]]; then
      GROUND_ARGS=(--ground-relative)
      python3 "$SCRIPT_DIR/costmap_ground_filter.py" "$PROJECT/navigation/$MAP_ID" --check
    fi
  fi
  echo "[INFO] obstacle backend: $BACKEND; check only, no robot process or motion is started"
  exec python3 "$SCRIPT_DIR/make_dynamic_nav2_config.py" \
    "$PROJECT/navigation/$MAP_ID/nav2_safe.yaml" \
    "$PROJECT/logs/dynamic_preflight/$MAP_ID/nav2_dynamic.yaml" \
    --behavior-tree "$PROJECT/navigation/navigate_dynamic_obstacles.xml" \
    "${MODE_ARGS[@]}" "${GROUND_ARGS[@]}"
fi
case "$BACKEND" in
  factory_only)
    export LITE3_USE_VENDOR_VOA=0
    ;;
  private_combined)
    export LITE3_USE_VENDOR_VOA=1
    ;;
esac
export LITE3_OBSTACLE_BACKEND="$BACKEND"
export LITE3_DYNAMIC_OBSTACLES=1
exec bash "$SCRIPT_DIR/navigation_start.sh" "${1:-start}" "${2:-$DEFAULT_MAP_ID}"
