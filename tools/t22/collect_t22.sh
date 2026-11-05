#!/usr/bin/env bash
# Local wrapper only; remote access occurs only with the operator's --host.
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$SCRIPT_DIR/collect_t22.py" "$@"
