#!/usr/bin/env bash
# Independent read-only camera bridge; does not start navigation or movement.
set -Eeuo pipefail
PROJECT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$PROJECT/navigation/patrol_env.sh"
exec python3 "$PROJECT/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/rtsp_camera.py" "$@"
