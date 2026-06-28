#!/bin/bash
# 从机器人同步所有 SLAM 数据到 Mac
# 每次同步创建带时间戳的文件夹，不覆盖历史数据
# 用法: bash ~/Claude/Projects/rosmaster/sync_data.sh

SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"
BASE_DIR="$HOME/Documents/rosmaster_r2"

# 时间戳文件夹
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
SESSION_DIR="$BASE_DIR/sessions/$TIMESTAMP"

# 创建目录结构
mkdir -p "$SESSION_DIR/maps/slices"
mkdir -p "$SESSION_DIR/bags"
mkdir -p "$SESSION_DIR/logs"

# 同时维护 latest 软链接
LATEST_LINK="$BASE_DIR/sessions/latest"

echo "========================================"
echo "  ROSMaster R2 数据同步"
echo "  会话: $TIMESTAMP"
echo "  目标: $SESSION_DIR"
echo "========================================"

# 检查连接
ssh -i "$SSH_KEY" $SSH_OPTS -o ConnectTimeout=5 "$HOST" "echo ok" >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo ""
    echo "  错误: 无法连接到机器人 (192.168.0.110)"
    echo "  请确认机器人在线且在同一网络"
    rmdir "$SESSION_DIR/logs" "$SESSION_DIR/bags" "$SESSION_DIR/maps/slices" "$SESSION_DIR/maps" "$SESSION_DIR" 2>/dev/null
    exit 1
fi

# 1. 同步地图
echo ""
echo "[1/5] 同步地图..."
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/overall_map.*" "$SESSION_DIR/maps/" 2>/dev/null && \
    echo "  overall_map 已同步" || echo "  overall_map 不存在"

scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/slices/*" "$SESSION_DIR/maps/slices/" 2>/dev/null && \
    echo "  slices 已同步" || echo "  无新 slices"

# 2. 同步 rosbag
echo ""
echo "[2/5] 同步 rosbag..."
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/temp/bags/*.bag" "$SESSION_DIR/bags/" 2>/dev/null && \
    echo "  bag 文件已同步" || echo "  无已完成的 bag"

# 3. 同步轨迹
echo ""
echo "[3/5] 同步轨迹..."
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/trajectory.csv" "$SESSION_DIR/maps/" 2>/dev/null && \
    echo "  trajectory.csv 已同步" || echo "  无轨迹数据"

# 4. 同步日志
echo ""
echo "[4/5] 同步日志..."
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_maps/autosave_service.log" "$SESSION_DIR/logs/" 2>/dev/null || true
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/temp/bags/record.log" "$SESSION_DIR/logs/rosbag_record.log" 2>/dev/null || true

# 5. 汇总
echo ""
echo "[5/5] 数据汇总："
echo "  地图:"
ls -lhS "$SESSION_DIR/maps/overall_map."* 2>/dev/null | awk '{print "    " $5 " " $NF}'
SLICE_COUNT=$(ls "$SESSION_DIR/maps/slices/"*.pcd 2>/dev/null | wc -l | tr -d ' ')
echo "  切片: $SLICE_COUNT 个 PCD 文件"
BAG_COUNT=$(ls "$SESSION_DIR/bags/"*.bag 2>/dev/null | wc -l | tr -d ' ')
echo "  Bag:  $BAG_COUNT 个文件"
if ls "$SESSION_DIR/bags/"*.bag >/dev/null 2>&1; then
    du -sh "$SESSION_DIR/bags/" | awk '{print "    总大小: " $1}'
fi

# 检查是否有实际数据（避免空文件夹）
FILE_COUNT=$(find "$SESSION_DIR" -type f | wc -l | tr -d ' ')
if [ "$FILE_COUNT" -eq 0 ]; then
    echo ""
    echo "  警告: 无数据同步，删除空目录"
    rm -rf "$SESSION_DIR"
    exit 0
fi

# 更新 latest 软链接
rm -f "$LATEST_LINK"
ln -s "$SESSION_DIR" "$LATEST_LINK"

# 生成轨迹可视化
if [ -f "$SESSION_DIR/maps/trajectory.csv" ] && [ -f "$SESSION_DIR/maps/overall_map.pgm" ]; then
    echo ""
    echo "  生成轨迹可视化..."
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    python3 "$SCRIPT_DIR/visualize_trajectory.py" "$SESSION_DIR/maps" 2>/dev/null && \
        echo "  轨迹图已生成: $SESSION_DIR/maps/map_with_trajectory.png" || \
        echo "  可视化脚本出错（需要 numpy + pyyaml）"
fi

echo ""
echo "========================================"
echo "  同步完成！"
echo "  本次数据: $SESSION_DIR"
echo "  最新链接: $LATEST_LINK"
echo ""
echo "  历史会话:"
ls -dt "$BASE_DIR/sessions"/20* 2>/dev/null | while read d; do
    COUNT=$(find "$d" -type f | wc -l | tr -d ' ')
    SIZE=$(du -sh "$d" 2>/dev/null | awk '{print $1}')
    echo "    $(basename $d)  ($COUNT 文件, $SIZE)"
done
echo "========================================"
