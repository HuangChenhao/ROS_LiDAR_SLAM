#!/bin/bash
# Sync scan sessions from car to Mac — v5
#
# v5 语义 (配合小车上的 slam_supervisor):
#   - 扫描会话由小车自动管理: 1/2档激活=开始建图(建文件夹), 红灯/3档=结束并保存
#   - 文件夹以【扫描开始时间】命名 (在小车上创建时就定好, 不是拉取时间)
#   - 本脚本只负责拉取: 容器 -> Pi -> Mac scans/, 已有的跳过
#   - 硬关机保护: supervisor 每30秒自动存图, 最多丢30秒
#
# Usage: bash sync_scan.sh

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=6"
[ -f "$SCRIPT_DIR/robot_config.sh" ] && source "$SCRIPT_DIR/robot_config.sh"
HOST="${R2_HOST:-${ROBOT_USER:-pi}@${ROBOT_IP:-192.168.0.110}}"
ADDR="${HOST#*@}"; USERPART="${HOST%%@*}"
case "$ADDR" in *:*) SCP_HOST="${USERPART}@[${ADDR}]";; *) SCP_HOST="$HOST";; esac

echo "=== Sync scan sessions (v5) ==="

# 0. Connectivity
if ! ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" 'echo PI_OK' 2>/dev/null | grep -q PI_OK; then
    echo "ERROR: 小车不可达 ($HOST)。检查: 开机? 同一wifi? IP变了? (改 robot_config.sh)"
    exit 1
fi

# 1. Stage: container /root/rosmaster_maps -> Pi /home/pi/rosmaster_maps
echo "Staging sessions on Pi..."
ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" '
    mkdir -p /home/pi/rosmaster_maps
    docker cp rosmaster_ros2:/root/rosmaster_maps/. /home/pi/rosmaster_maps/ 2>/dev/null || true
    ls /home/pi/rosmaster_maps/ 2>/dev/null
' > /tmp/_sessions.txt
SESSIONS=$(cat /tmp/_sessions.txt)

if [ -z "$SESSIONS" ]; then
    echo "没有任何扫描会话。先用手柄建图 (1/2档激活)。"
    exit 0
fi

# 2. Pull each session not yet local
mkdir -p "$SCRIPT_DIR/scans"
PULLED=0; SKIPPED=0
for s in $SESSIONS; do
    # session dirs look like 20260715_183045
    case "$s" in [0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]_*) ;; *) continue;; esac
    if [ -s "$SCRIPT_DIR/scans/$s/map.pgm" ]; then
        SKIPPED=$((SKIPPED+1)); continue
    fi
    mkdir -p "$SCRIPT_DIR/scans/$s"
    if scp -i "$SSH_KEY" $SSH_OPTS -r "$SCP_HOST:/home/pi/rosmaster_maps/$s/*" "$SCRIPT_DIR/scans/$s/" 2>/dev/null; then
        if [ -s "$SCRIPT_DIR/scans/$s/map.pgm" ]; then
            echo "  ✓ $s ($(du -h "$SCRIPT_DIR/scans/$s/map.pgm" | cut -f1 | tr -d ' '))"
            PULLED=$((PULLED+1))
        else
            echo "  ⚠ $s 无 map.pgm (会话可能没建出图), 保留 metadata"
            PULLED=$((PULLED+1))
        fi
    else
        rm -rf "$SCRIPT_DIR/scans/$s"
        echo "  ✗ $s 拉取失败"
    fi
done

echo ""
echo "Done: 拉取 $PULLED 个新会话, 跳过 $SKIPPED 个已同步。"
ls "$SCRIPT_DIR/scans/"
