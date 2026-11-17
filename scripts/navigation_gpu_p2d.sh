#!/usr/bin/env bash
# Development VM entry; the existing launcher performs the normal safe startup.
set -Eeuo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export LITE3_LOCALIZATION_RUNTIME=audited
export LITE3_NDT_BACKEND=gpu_p2d
exec bash "$SCRIPT_DIR/navigation_start.sh" "$@"
