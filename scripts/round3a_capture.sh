#!/usr/bin/env bash
# Compatibility entry for historical commands; use mapping_capture.sh.
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/mapping_capture.sh" "$@"
