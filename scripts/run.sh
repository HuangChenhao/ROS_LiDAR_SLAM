#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"
OUTPUT="$SCRIPT_DIR/cmd_output.txt"

# 从 stdin 读取命令块
ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash > "$OUTPUT" 2>&1 << 'REMOTECMD'
COMMANDS_PLACEHOLDER
REMOTECMD
