#!/usr/bin/env bash
# Sourced only by start_localization_only, after patrol_env.sh.
# Selects an already built release. Does not start a node or switch live state.
NDT_SELECTION=$(python3 "$PROJECT/scripts/select_ndt_backend.py" \
  --project "$PROJECT" --backend "${LITE3_NDT_BACKEND:-default}" --format lines)
mapfile -t NDT_FIELDS <<<"$NDT_SELECTION"
[[ ${#NDT_FIELDS[@]} == 8 ]] || { echo 'invalid NDT release selection' >&2; return 1; }
NDT_LABEL=${NDT_FIELDS[0]}
NDT_ROOT=${NDT_FIELDS[1]}
NDT_PREFIX=${NDT_FIELDS[2]}
NDT_REG_METHOD=${NDT_FIELDS[3]}
NDT_LIBRARY_PATH=${NDT_FIELDS[4]}
NDT_LAUNCH_ARGS=("${NDT_FIELDS[7]}")
set +u
if [[ "$NDT_LABEL" == CPU_NDT_OMP ]]; then
  source "$NDT_ROOT/ws/install/setup.bash"
else
  source "$NDT_ROOT/ws/install/local_setup.bash"
fi
set -u
export LD_LIBRARY_PATH="$NDT_LIBRARY_PATH"
export LD_PRELOAD=
NDT_ACTUAL_PREFIX=$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("hdl_localization"))')
[[ "$(readlink -f "$NDT_ACTUAL_PREFIX")" == "$(readlink -f "$NDT_PREFIX")" ]] || {
  echo 'selected hdl_localization overlay mismatch' >&2; return 1;
}
printf '[CONFIG] NDT_BACKEND=%s reg_method=%s build=%s binary=%s\n' \
  "$NDT_LABEL" "$NDT_REG_METHOD" "${NDT_FIELDS[6]}" "${NDT_FIELDS[5]}"
printf '[CONFIG] NDT_LIBRARY_PATH=%s\n' "$NDT_LIBRARY_PATH"
