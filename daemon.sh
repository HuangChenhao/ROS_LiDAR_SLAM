#!/bin/bash
# ROSMaster 命令守护进程
# 启动一次后，Claude 写入 remote_commands.sh，守护进程自动执行并写入结果
# 用法: bash ~/Claude/Projects/rosmaster/daemon.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CMD_FILE="$SCRIPT_DIR/remote_commands.sh"
OUTPUT="$SCRIPT_DIR/cmd_output.txt"
TRIGGER="$SCRIPT_DIR/.trigger"
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
# IP/用户名从 robot_config.sh 读取, 可用 R2_HOST 环境变量覆盖
[ -f "$SCRIPT_DIR/robot_config.sh" ] && source "$SCRIPT_DIR/robot_config.sh"
HOST="${R2_HOST:-${ROBOT_USER:-pi}@${ROBOT_IP:-192.168.0.110}}"

echo "ROSMaster 守护进程已启动"
echo "监听命令文件: $CMD_FILE"
echo "按 Ctrl+C 退出"
echo ""

# 记录初始修改时间
LAST_MOD=0

while true; do
    if [ -f "$TRIGGER" ]; then
        echo "[$(date '+%H:%M:%S')] 检测到新命令，执行中..."
        rm -f "$TRIGGER"
        echo "RUNNING" > "$OUTPUT"
        ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash < "$CMD_FILE" > "$OUTPUT" 2>&1
        echo "__EXIT_CODE__=$?" >> "$OUTPUT"
        echo "[$(date '+%H:%M:%S')] 执行完成，结果已写入 cmd_output.txt"
    fi
    sleep 1
done
