#!/usr/bin/env bash
# VM entry: offline check by default; run requires explicit ARM confirmation.
set -Eeuo pipefail
PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ACTION=${1:-check}
MAP_ID=${2:-outdoor_20260921_003_area_review}
MISSION=${3:-mission_recovered_20261008.yaml}
[[ "$MAP_ID" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'invalid map id' >&2; exit 1; }
[[ "$MISSION" =~ ^[A-Za-z0-9_-]+\.yaml$ ]] || { echo 'mission must be a YAML basename' >&2; exit 1; }
case "$ACTION" in check|run) ;; *) echo 'Usage: bash scripts/patrol_route.sh check|run [MAP_ID] [MISSION.yaml]' >&2; exit 1;; esac
RUNNER=$PROJECT/patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/patrol_runner.py
FILE=$PROJECT/navigation/$MAP_ID/$MISSION
python3 "$RUNNER" "$FILE"
[[ "$ACTION" == run ]] || exit 0
echo '即将执行一次多点巡航；默认每个目标超时 120 秒，失败停止，结束锁定。'
echo '先在另一终端运行新版 indoor_nav_start.sh 并等待 NAVIGATION READY，保持 DISARMED。'
echo '本次 ARM 确认授权执行草稿，不修改 enabled/review_required；不增加人工路径复核。'
echo '运行期间不要在 RViz 点选新目标；现场持遥控器准备急停。'
while true; do
  if ! read -r -p '确认执行整条路线，请输入 ARM（取消输入 N）: ' confirm; then
    echo '[CANCELED] 未收到执行确认（输入已结束）。请在交互终端重新运行；未尝试连接或发送目标。' >&2
    exit 1
  fi
  if [[ "$confirm" =~ ^[[:space:]]*[Aa][Rr][Mm][[:space:]]*$ ]]; then
    break
  fi
  if [[ "$confirm" =~ ^[[:space:]]*([Nn]|[Nn][Oo]|[Qq]|[Cc][Aa][Nn][Cc][Ee][Ll])[[:space:]]*$ ]]; then
    echo '[CANCELED] 你已取消执行；未尝试连接或发送目标。这不是网络连接失败。'
    exit 1
  fi
  echo '[WAIT] 尚未确认执行。请输入 ARM 后回车，或输入 N 取消；不会自动解锁。'
done
echo '[CONFIRMED] 已收到本次巡航授权，开始连接 Jetson。'
ROBOT=${ROBOT:-ysc@192.168.1.103}
KEY=${SSH_KEY:-/home/duyipei/.ssh/id_ed25519_cyberdog}
REMOTE=/home/ysc/lite3_patrol_project
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=3 -o ServerAliveCountMax=2 -i "$KEY")
UPLOAD=$("${SSH[@]}" "$ROBOT" "mktemp -d /tmp/lite3_patrol_mission.XXXXXX")
[[ "$UPLOAD" =~ ^/tmp/lite3_patrol_mission\.[A-Za-z0-9]+$ ]] || exit 1
scp -q -o BatchMode=yes -o ConnectTimeout=8 -i "$KEY" "$FILE" "$RUNNER" "$ROBOT:$UPLOAD/"
# PTY forwards Ctrl+C/HUP; bridge independently supervises runner heartbeats.
mkdir -p "$PROJECT/logs"
RUN_LOG=$PROJECT/logs/patrol_route_${MAP_ID}_$(date +%Y%m%d_%H%M%S)_$$.log
echo "巡航逐点结果与运动诊断日志：$RUN_LOG"
"${SSH[@]}" -tt "$ROBOT" "source '$REMOTE/navigation/patrol_env.sh'; exec python3 '$UPLOAD/patrol_runner.py' '$UPLOAD/$MISSION' --map-yaml '$REMOTE/navigation/$MAP_ID/patrol_map.yaml' --execute --authorize-draft" 2>&1 | tee "$RUN_LOG"
