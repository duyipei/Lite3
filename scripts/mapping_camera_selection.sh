#!/usr/bin/env bash
# Shared VM-side selection; sourcing this file never connects to the robot.
select_mapping_camera() {
  local selected=${1:-${LITE3_MAPPING_CAMERA:-}}
  if [[ -z "$selected" ]]; then
    printf '请选择扫图相机：\n  1) 深度相机 RealSense D435i（默认）\n  2) 广角相机 LRCP F1080P（预采集，仍保留 D435i）\n'
    while true; do
      printf '输入 1 或 2，回车默认 1，q 取消：'
      if ! IFS= read -r selected; then
        printf '\n[CANCELED] 未收到选择，未启动采集。\n' >&2
        return 1
      fi
      selected=${selected//[[:space:]]/}
      selected=${selected:-1}
      case "$selected" in
        1|2) break ;;
        q|Q) printf '[CANCELED] 已取消。\n'; return 1 ;;
        *) printf '输入无效，请选择 1 或 2。\n' ;;
      esac
    done
  fi
  [[ "$selected" == 1 || "$selected" == 2 ]] || {
    printf '[ERROR] CAMERA must be 1 or 2\n' >&2
    return 1
  }
  export LITE3_MAPPING_CAMERA=$selected
  if [[ "$selected" == 1 ]]; then
    printf '[CAMERA] 相机1：RealSense D435i 深度相机。\n'
  else
    printf '[CAMERA] 相机2：LRCP F1080P 广角相机。\n'
    printf '[CAMERA] 请先在另一终端执行扫图脚本的 camera2-start，并保持运行。\n'
    printf '[CAMERA] RAW DATA ONLY：同时保留 D435i；广角相机标定及视觉地图生成尚未完成。\n'
  fi
}
