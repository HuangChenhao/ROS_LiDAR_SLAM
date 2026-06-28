#!/bin/bash
# 执行 remote_commands.sh 的内容到小车上，结果存入 cmd_output.txt
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"
OUTPUT="$SCRIPT_DIR/cmd_output.txt"

ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash < "$SCRIPT_DIR/remote_commands.sh" > "$OUTPUT" 2>&1
echo "done" > "$SCRIPT_DIR/cmd_done.txt"
