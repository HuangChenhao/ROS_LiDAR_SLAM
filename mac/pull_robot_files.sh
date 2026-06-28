#!/bin/bash
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"
DEST="$HOME/Documents/rosmaster_r2"

echo "拉取小车端文件..."
mkdir -p "$DEST/robot"

scp -r -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/temp/robot_export/*" "$DEST/robot/" 2>/dev/null
echo "完成: $(ls -la $DEST/robot/ 2>/dev/null | wc -l) 项"
