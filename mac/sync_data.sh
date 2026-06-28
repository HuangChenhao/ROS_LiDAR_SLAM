#!/bin/bash
# 从机器人同步所有 SLAM 数据到 Mac
# 用法: bash ~/Claude/Projects/rosmaster/sync_data.sh

SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"
LOCAL_DIR="$HOME/Documents/rosmaster_r2"

# 创建本地目录结构
mkdir -p "$LOCAL_DIR/maps/slices"
mkdir -p "$LOCAL_DIR/bags"
mkdir -p "$LOCAL_DIR/logs"

echo "========================================"
echo "  ROSMaster R2 数据同步"
echo "  目标: $LOCAL_DIR"
echo "========================================"

# 1. 同步地图（overall + slices）
echo ""
echo "[1/4] 同步地图..."
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/overall_map.*" "$LOCAL_DIR/maps/" 2>/dev/null && \
    echo "  overall_map 已同步" || echo "  overall_map 不存在"

scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/slices/*" "$LOCAL_DIR/maps/slices/" 2>/dev/null && \
    echo "  slices 已同步" || echo "  无新 slices"

# 2. 同步 rosbag
echo ""
echo "[2/4] 同步 rosbag..."
# 只同步已完成的 bag（不同步 .active 文件）
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/temp/bags/*.bag" "$LOCAL_DIR/bags/" 2>/dev/null && \
    echo "  bag 文件已同步" || echo "  无已完成的 bag"

# 3. 同步轨迹
echo ""
echo "[3/5] 同步轨迹..."
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/trajectory.csv" "$LOCAL_DIR/maps/" 2>/dev/null && \
    echo "  trajectory.csv 已同步" || echo "  无轨迹数据"

# 4. 同步日志
echo ""
echo "[4/5] 同步日志..."
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/autosave_service.log" "$LOCAL_DIR/logs/" 2>/dev/null || true
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/temp/bags/record.log" "$LOCAL_DIR/logs/rosbag_record.log" 2>/dev/null || true

# 5. 汇总 + 可视化
echo ""
echo "[5/5] 数据汇总："
echo "  地图:"
ls -lhS "$LOCAL_DIR/maps/overall_map."* 2>/dev/null | awk '{print "    " $5 " " $NF}'
echo "  切片: $(ls "$LOCAL_DIR/maps/slices/"*.pcd 2>/dev/null | wc -l) 个 PCD 文件"
echo "  Bag:  $(ls "$LOCAL_DIR/bags/"*.bag 2>/dev/null | wc -l) 个文件"
if ls "$LOCAL_DIR/bags/"*.bag >/dev/null 2>&1; then
    du -sh "$LOCAL_DIR/bags/" | awk '{print "    总大小: " $1}'
fi

# 生成轨迹可视化
if [ -f "$LOCAL_DIR/maps/trajectory.csv" ] && [ -f "$LOCAL_DIR/maps/overall_map.pgm" ]; then
    echo ""
    echo "  生成轨迹可视化..."
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    python3 "$SCRIPT_DIR/visualize_trajectory.py" "$LOCAL_DIR/maps" 2>/dev/null && \
        echo "  轨迹图已生成: $LOCAL_DIR/maps/map_with_trajectory.png" || \
        echo "  可视化脚本出错（需要 numpy + pyyaml）"
fi

echo ""
echo "========================================"
echo "  同步完成！数据在: $LOCAL_DIR"
echo "========================================"
