#!/bin/bash
# ============================================================================
# SLAM Bag 回放后处理 - 从 Mac 远程触发
# 用法: bash ~/Claude/Projects/rosmaster/run_replay.sh [rate] [output_name]
#   rate: 回放速度，默认 0.5 (越慢越精确)
#   output_name: 输出名称，默认 replay_<timestamp>
#
# 参数调优示例:
#   REPLAY_MAXURANGE=4 REPLAY_MINSCORE=300 bash ~/Claude/Projects/rosmaster/run_replay.sh 0.3
# ============================================================================

SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
HOST="pi@192.168.0.110"
CONTAINER="rosmaster_slam"

RATE="${1:-0.5}"
OUTPUT="${2:-replay_$(date +%Y%m%d_%H%M%S)}"

# 传递环境变量
ENV_VARS="REPLAY_RATE=$RATE"
[ -n "$REPLAY_MAXURANGE" ] && ENV_VARS="$ENV_VARS REPLAY_MAXURANGE=$REPLAY_MAXURANGE"
[ -n "$REPLAY_MAXRANGE" ] && ENV_VARS="$ENV_VARS REPLAY_MAXRANGE=$REPLAY_MAXRANGE"
[ -n "$REPLAY_MINSCORE" ] && ENV_VARS="$ENV_VARS REPLAY_MINSCORE=$REPLAY_MINSCORE"
[ -n "$REPLAY_PARTICLES" ] && ENV_VARS="$ENV_VARS REPLAY_PARTICLES=$REPLAY_PARTICLES"

echo "========================================"
echo "  SLAM Bag 回放后处理"
echo "  回放速度: ${RATE}x"
echo "  输出名称: $OUTPUT"
echo "========================================"

# Step 1: 部署回放脚本
echo ""
echo "[1] 部署回放脚本到机器人..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPLAY_SRC="$HOME/Documents/rosmaster_r2/robot/scripts/replay_slam.sh"

scp -i "$SSH_KEY" $SSH_OPTS "$REPLAY_SRC" "$HOST:/home/pi/temp/replay_slam.sh"
if [ $? -ne 0 ]; then
    echo "  SCP 失败"
    exit 1
fi
echo "  OK"

# Step 2: 在 Docker 内执行回放
echo ""
echo "[2] 启动回放 (这将需要较长时间)..."
echo "    按 Ctrl+C 中断"
echo ""

ssh -i "$SSH_KEY" $SSH_OPTS -t "$HOST" \
    "docker exec -e $ENV_VARS $CONTAINER bash /root/temp/replay_slam.sh /root/temp/bags $OUTPUT"

echo ""
echo "========================================"
echo "  回放完成！"
echo "  同步结果: bash ~/Claude/Projects/rosmaster/sync_data.sh"
echo "========================================"
