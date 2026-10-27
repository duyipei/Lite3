#!/usr/bin/env bash
# Compatibility entry for historical commands; use navigation_start.sh.
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/navigation_start.sh" "$@"
