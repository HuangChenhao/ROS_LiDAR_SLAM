#!/bin/bash
# Sync scan data from Pi to Mac with timestamped folder
# Usage: bash sync_scan.sh [session_name]
#
# v4 语义:
#   - 文件夹总是以【扫描结束时间】(地图在 Pi 上保存的时刻) 命名
#   - 优先实时抓取当前 /map (小车正在建图时)
#   - 抓不到 (gmapping 没跑/刚重启) 则回退到 Pi 上持久化的上一次地图
#     (/home/pi/rosmaster_maps/last_scan.*)，用它的保存时间命名
#   - 小车完全不可达 → 立即报错退出，不创建任何文件夹
#   - 地图持久化到 /home/pi/rosmaster_maps/ (关机不丢)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=6"
# IP/用户名从 robot_config.sh 读取, 也可用环境变量覆盖: R2_HOST=pi@10.0.0.5 bash sync_scan.sh
[ -f "$SCRIPT_DIR/robot_config.sh" ] && source "$SCRIPT_DIR/robot_config.sh"
HOST="${R2_HOST:-${ROBOT_USER:-pi}@${ROBOT_IP:-192.168.0.110}}"
# scp 对 IPv6 地址要求加方括号: pi@[fe80::...%en0]:/path
ADDR="${HOST#*@}"; USERPART="${HOST%%@*}"
case "$ADDR" in *:*) SCP_HOST="${USERPART}@[${ADDR}]";; *) SCP_HOST="$HOST";; esac
SESSION_NAME="${1:-}"

echo "=== Sync scan ==="

# 0. 连通性检查 — 不可达直接退出，不留垃圾文件夹
if ! ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" 'echo PI_OK' 2>/dev/null | grep -q PI_OK; then
    echo "ERROR: 小车不可达 ($HOST)。请检查: 开机了吗? 同一 wifi? IP 变了? (改 robot_config.sh)"
    exit 1
fi

# 1. 在 Pi 上抓取: 优先实时 /map，失败则用持久化的上一次地图
echo "Saving map on Pi..."
SSH_OUT=$(ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash << 'REMOTE' 2>&1
set -u
C="rosmaster_ros2"
S="source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1"
PERSIST_DIR="/home/pi/rosmaster_maps"
mkdir -p "$PERSIST_DIR"

LIVE_OK=0
if docker ps --format '{{.Names}}' | grep -q "^${C}$"; then
    docker exec "$C" bash -c "pkill -f map_sync" 2>/dev/null || true
    docker exec "$C" bash -c "rm -f /tmp/final_map.*" 2>/dev/null || true

    # rclpy.spin() may hang after shutdown() in Foxy -> spin_once loop + hard timeout
    timeout 60 docker exec "$C" bash -c "$S && python3 -u -c \"
import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class M(Node):
    def __init__(self):
        super().__init__('map_sync')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, 1)
        self.done = False
    def cb(self, msg):
        if self.done: return
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data == 100] = 0
        img = np.flipud(img)
        with open('/tmp/final_map.pgm', 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            f.write(img.tobytes())
        with open('/tmp/final_map.yaml', 'w') as f:
            f.write(f'image: map.pgm\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        print(f'Saved: {w}x{h} res={res}', flush=True)
        self.done = True

rclpy.init()
n = M()
t0 = time.time()
while not n.done and time.time() - t0 < 35:
    rclpy.spin_once(n, timeout_sec=0.5)
if not n.done:
    print('WARN: 35s 内无 /map 数据 (gmapping 没在建图?)', flush=True)
\"" 2>&1

    if docker cp "$C:/tmp/final_map.pgm" /tmp/_grab.pgm 2>/dev/null && [ -s /tmp/_grab.pgm ]; then
        docker cp "$C:/tmp/final_map.yaml" /tmp/_grab.yaml 2>/dev/null
        # 持久化 (关机不丢) — mtime 即扫描结束时间
        mv /tmp/_grab.pgm  "$PERSIST_DIR/last_scan.pgm"
        mv /tmp/_grab.yaml "$PERSIST_DIR/last_scan.yaml"
        LIVE_OK=1
    fi
else
    echo "WARN: 容器未运行"
fi

if [ $LIVE_OK -eq 1 ]; then
    echo "SOURCE=live"
elif [ -s "$PERSIST_DIR/last_scan.pgm" ]; then
    echo "SOURCE=last_saved (实时抓取失败, 使用上一次持久化的扫描)"
else
    echo "SOURCE=none"
    exit 42
fi
echo "SAVE_OK size=$(stat -c%s "$PERSIST_DIR/last_scan.pgm")"
echo "MAP_EPOCH=$(stat -c %Y "$PERSIST_DIR/last_scan.pgm")"
REMOTE
)
SSH_RC=$?
echo "$SSH_OUT"

if [ $SSH_RC -eq 42 ]; then
    echo "ERROR: 既没有实时地图，Pi 上也没有历史扫描。先建图再同步。"
    exit 1
elif [ $SSH_RC -ne 0 ]; then
    echo "WARN: SSH 异常退出 (rc=$SSH_RC)，尝试继续拉取..."
fi

# 2. 用扫描结束时间命名 (= last_scan.pgm 的 mtime)
MAP_EPOCH=$(echo "$SSH_OUT" | grep -o 'MAP_EPOCH=[0-9]*' | cut -d= -f2 | tail -1)
if [ -z "${MAP_EPOCH:-}" ]; then
    MAP_EPOCH=$(ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" 'stat -c %Y /home/pi/rosmaster_maps/last_scan.pgm' 2>/dev/null || true)
fi
if [ -z "${MAP_EPOCH:-}" ]; then
    echo "ERROR: 无法确定扫描时间，中止 (不以当前时间乱命名)。"
    exit 1
fi

fmt_epoch() { date -r "$1" +%Y%m%d_%H%M%S 2>/dev/null || date -d "@$1" +%Y%m%d_%H%M%S; }
TIMESTAMP=$(fmt_epoch "$MAP_EPOCH")
MAP_AGE=$(( $(date +%s) - MAP_EPOCH ))
SOURCE=$(echo "$SSH_OUT" | grep -o 'SOURCE=[a-z_]*' | cut -d= -f2 | tail -1)

if [ -n "$SESSION_NAME" ]; then
    SCAN_DIR="$SCRIPT_DIR/scans/${TIMESTAMP}_${SESSION_NAME}"
else
    SCAN_DIR="$SCRIPT_DIR/scans/${TIMESTAMP}"
fi

if [ -d "$SCAN_DIR" ] && [ -s "$SCAN_DIR/map.pgm" ]; then
    echo "该扫描已同步过: $SCAN_DIR"
    exit 0
fi
mkdir -p "$SCAN_DIR"
echo "Session: $(basename "$SCAN_DIR") (扫描结束时间命名, 来源: ${SOURCE:-?}, ${MAP_AGE}s 前)"

# 3. 拉取 (带重试)
pull() {
    local i
    for i in 1 2 3; do
        if scp -i "$SSH_KEY" $SSH_OPTS "$SCP_HOST:$1" "$2" 2>/dev/null && [ -s "$2" ]; then
            echo "  $(basename "$2") OK ($(du -h "$2" | cut -f1 | tr -d ' '))"
            return 0
        fi
        echo "  $(basename "$2") attempt $i failed, retrying..."
        sleep 2
    done
    echo "  $(basename "$2") FAILED after 3 attempts"
    return 1
}

echo "Pulling files..."
PGM_OK=0
pull /home/pi/rosmaster_maps/last_scan.pgm  "$SCAN_DIR/map.pgm"  && PGM_OK=1
pull /home/pi/rosmaster_maps/last_scan.yaml "$SCAN_DIR/map.yaml"

# 4. Metadata
cat > "$SCAN_DIR/metadata.txt" << EOF
Session: $(basename "$SCAN_DIR")
Scan end (map saved on Pi): ${TIMESTAMP}
Source: ${SOURCE:-unknown}
Pulled at: $(date '+%Y-%m-%d %H:%M:%S')
Robot: ROSMaster R2 | ROS2 Foxy | gmapping | RPLidar A1
EOF

echo ""
if [ $PGM_OK -eq 1 ]; then
    echo "Done! Saved to: $SCAN_DIR"
else
    rm -f "$SCAN_DIR/metadata.txt"; rmdir "$SCAN_DIR" 2>/dev/null
    echo "FAILED: 拉取失败，已清理空文件夹。"
    exit 1
fi
ls -lh "$SCAN_DIR/"
