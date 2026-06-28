#!/bin/bash
# 通用远程执行：将 stdin 或参数作为命令发送到小车
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"
OUTPUT="$SCRIPT_DIR/cmd_output.txt"

if [ -n "$1" ]; then
    ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash -c "$*" > "$OUTPUT" 2>&1
else
    ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash > "$OUTPUT" 2>&1
fi
