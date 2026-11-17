#!/usr/bin/env bash
# Stop the previous session before starting this CPU rollback entry.
set -Eeuo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export LITE3_LOCALIZATION_RUNTIME=audited
export LITE3_NDT_BACKEND=cpu_omp
exec bash "$SCRIPT_DIR/navigation_start.sh" "$@"
