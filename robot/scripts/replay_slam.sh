#!/bin/bash
# ============================================================================
# SLAM Bag 回放后处理 - 在 Docker 容器内运行
# 用 rosbag play + gmapping 离线重建地图，支持参数调优
#
# 用法 (在 Pi host 上):
#   docker exec -it rosmaster_slam bash /root/replay_slam.sh [bag_dir] [output_name]
#
# 默认:
#   bag_dir:     /root/temp/bags
#   output_name: replay_<timestamp>
#
# 可通过环境变量覆盖 gmapping 参数:
#   REPLAY_MAXURANGE=6 REPLAY_MINSCORE=200 docker exec -it rosmaster_slam bash /root/replay_slam.sh
# ============================================================================

set -e

# --- 参数 ---
BAG_DIR="${1:-/root/temp/bags}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_NAME="${2:-replay_${TIMESTAMP}}"
OUTPUT_DIR="/home/pi/rosmaster_maps/replay/${OUTPUT_NAME}"

# gmapping 参数 (可通过环境变量覆盖)
MAXURANGE="${REPLAY_MAXURANGE:-6.0}"
MAXRANGE="${REPLAY_MAXRANGE:-8.0}"
MINSCORE="${REPLAY_MINSCORE:-200}"
PARTICLES="${REPLAY_PARTICLES:-80}"
LINEAR_UPDATE="${REPLAY_LINEAR_UPDATE:-0.2}"
ANGULAR_UPDATE="${REPLAY_ANGULAR_UPDATE:-0.15}"
TEMPORAL_UPDATE="${REPLAY_TEMPORAL_UPDATE:-1.0}"
MAP_UPDATE_INTERVAL="${REPLAY_MAP_INTERVAL:-2.0}"
DELTA="${REPLAY_DELTA:-0.03}"
PLAY_RATE="${REPLAY_RATE:-0.5}"  # 回放速度，<1 更慢更精确
XMIN="${REPLAY_XMIN:--15}"
XMAX="${REPLAY_XMAX:-15}"
YMIN="${REPLAY_YMIN:--15}"
YMAX="${REPLAY_YMAX:-15}"

# --- ROS 环境 ---
source /opt/ros/melodic/setup.bash
source /root/yahboomcar_ws/devel/setup.bash 2>/dev/null || true

echo "============================================================"
echo "  SLAM Bag 回放后处理"
echo "============================================================"
echo "  Bag 目录:   $BAG_DIR"
echo "  输出目录:   $OUTPUT_DIR"
echo "  回放速度:   ${PLAY_RATE}x"
echo ""
echo "  gmapping 参数:"
echo "    maxUrange:      $MAXURANGE"
echo "    maxRange:       $MAXRANGE"
echo "    minimumScore:   $MINSCORE"
echo "    particles:      $PARTICLES"
echo "    linearUpdate:   $LINEAR_UPDATE"
echo "    angularUpdate:  $ANGULAR_UPDATE"
echo "    temporalUpdate: $TEMPORAL_UPDATE"
echo "    delta:          $DELTA"
echo "    地图范围:       [${XMIN},${XMAX}] x [${YMIN},${YMAX}]"
echo "============================================================"

# --- 检查 bag 文件 ---
BAGS=$(ls -1 "$BAG_DIR"/*.bag 2>/dev/null | sort)
BAG_COUNT=$(echo "$BAGS" | grep -c ".bag" || true)

if [ "$BAG_COUNT" -eq 0 ]; then
    echo "错误: 未找到 bag 文件: $BAG_DIR"
    exit 1
fi

echo ""
echo "找到 $BAG_COUNT 个 bag 文件:"
for b in $BAGS; do
    SIZE=$(du -h "$b" | awk '{print $1}')
    echo "  $(basename $b) ($SIZE)"
done

# --- 准备输出目录 ---
mkdir -p "$OUTPUT_DIR"

# --- 停止当前运行的 gmapping (如果有) ---
echo ""
echo "[Step 1] 停止当前 SLAM 节点..."

# 记录当前运行的节点，以便后续恢复
RUNNING_GMAPPING=$(rosnode list 2>/dev/null | grep slam_gmapping) || true

if [ -n "$RUNNING_GMAPPING" ]; then
    echo "  杀死当前 gmapping: $RUNNING_GMAPPING"
    rosnode kill /slam_gmapping 2>/dev/null || true
    sleep 2
    # 确认已停止
    if rosnode list 2>/dev/null | grep -q slam_gmapping; then
        echo "  警告: gmapping 仍在运行，强制杀死"
        pkill -f slam_gmapping 2>/dev/null || true
        sleep 2
    fi
fi

# --- 启动 gmapping (离线模式) ---
echo ""
echo "[Step 2] 启动 gmapping (离线参数)..."

# 设置参数到 parameter server
rosparam set /slam_gmapping_replay/maxUrange "$MAXURANGE"
rosparam set /slam_gmapping_replay/maxRange "$MAXRANGE"
rosparam set /slam_gmapping_replay/minimumScore "$MINSCORE"
rosparam set /slam_gmapping_replay/particles "$PARTICLES"
rosparam set /slam_gmapping_replay/linearUpdate "$LINEAR_UPDATE"
rosparam set /slam_gmapping_replay/angularUpdate "$ANGULAR_UPDATE"
rosparam set /slam_gmapping_replay/temporalUpdate "$TEMPORAL_UPDATE"
rosparam set /slam_gmapping_replay/map_update_interval "$MAP_UPDATE_INTERVAL"
rosparam set /slam_gmapping_replay/delta "$DELTA"
rosparam set /slam_gmapping_replay/xmin "$XMIN"
rosparam set /slam_gmapping_replay/xmax "$XMAX"
rosparam set /slam_gmapping_replay/ymin "$YMIN"
rosparam set /slam_gmapping_replay/ymax "$YMAX"
rosparam set /slam_gmapping_replay/odom_frame "odom"
rosparam set /slam_gmapping_replay/base_frame "base_footprint"
rosparam set /slam_gmapping_replay/map_frame "map"

# 启动 gmapping 节点 (使用 replay 命名空间避免冲突)
rosrun gmapping slam_gmapping \
    __name:=slam_gmapping_replay \
    scan:=/scan \
    _maxUrange:="$MAXURANGE" \
    _maxRange:="$MAXRANGE" \
    _minimumScore:="$MINSCORE" \
    _particles:="$PARTICLES" \
    _linearUpdate:="$LINEAR_UPDATE" \
    _angularUpdate:="$ANGULAR_UPDATE" \
    _temporalUpdate:="$TEMPORAL_UPDATE" \
    _map_update_interval:="$MAP_UPDATE_INTERVAL" \
    _delta:="$DELTA" \
    _xmin:="$XMIN" \
    _xmax:="$XMAX" \
    _ymin:="$YMIN" \
    _ymax:="$YMAX" \
    _odom_frame:="odom" \
    _base_frame:="base_footprint" \
    _map_frame:="map" \
    _transform_publish_period:="0.0" \
    &

GMAPPING_PID=$!
sleep 3

# 确认启动
if ! kill -0 $GMAPPING_PID 2>/dev/null; then
    echo "错误: gmapping 启动失败"
    exit 1
fi
echo "  gmapping 已启动 (PID: $GMAPPING_PID)"

# --- 回放 bag ---
echo ""
echo "[Step 3] 回放 bag 文件 (rate=${PLAY_RATE})..."
echo "  这可能需要较长时间，请耐心等待..."

# 使用 --clock 让 gmapping 使用 bag 时间而非实时时间
# 需要先设置 use_sim_time
rosparam set /use_sim_time true

TOTAL_BAGS=$(echo "$BAGS" | wc -l | tr -d ' ')
CURRENT=0

for BAG in $BAGS; do
    CURRENT=$((CURRENT + 1))
    echo "  [$CURRENT/$TOTAL_BAGS] 回放: $(basename $BAG)"

    rosbag play "$BAG" \
        --clock \
        --rate "$PLAY_RATE" \
        --topics /scan /tf /tf_static /odom /imu/imu_data \
        --quiet \
        2>/dev/null

    echo "    完成"
done

echo "  所有 bag 回放完成"

# 等待 gmapping 处理完最后的数据
echo "  等待 gmapping 完成处理..."
sleep 5

# --- 保存地图 ---
echo ""
echo "[Step 4] 保存地图..."

rosrun map_server map_saver \
    -f "$OUTPUT_DIR/replay_map" \
    map:=/map \
    2>/dev/null

if [ -f "$OUTPUT_DIR/replay_map.pgm" ]; then
    MAP_SIZE=$(du -h "$OUTPUT_DIR/replay_map.pgm" | awk '{print $1}')
    echo "  地图已保存: $OUTPUT_DIR/replay_map.pgm ($MAP_SIZE)"
else
    echo "  警告: 地图保存失败，尝试备用方法..."
    # 用 rostopic 获取一帧 map 数据
    timeout 5 rostopic echo /map -n 1 > "$OUTPUT_DIR/map_raw.yaml" 2>/dev/null || true
fi

# --- 清理 ---
echo ""
echo "[Step 5] 清理..."

# 停止 replay gmapping
kill $GMAPPING_PID 2>/dev/null || true
wait $GMAPPING_PID 2>/dev/null || true
echo "  gmapping_replay 已停止"

# 恢复 use_sim_time
rosparam set /use_sim_time false

# 恢复原来的 gmapping (如果之前在运行)
if [ -n "$RUNNING_GMAPPING" ]; then
    echo "  注意: 原 gmapping 已被停止，需要重启 SLAM launch 恢复实时建图"
fi

# 保存使用的参数
cat > "$OUTPUT_DIR/params.txt" << EOF
# SLAM 回放参数 - $TIMESTAMP
maxUrange=$MAXURANGE
maxRange=$MAXRANGE
minimumScore=$MINSCORE
particles=$PARTICLES
linearUpdate=$LINEAR_UPDATE
angularUpdate=$ANGULAR_UPDATE
temporalUpdate=$TEMPORAL_UPDATE
delta=$DELTA
play_rate=$PLAY_RATE
map_range=[${XMIN},${XMAX}]x[${YMIN},${YMAX}]
bag_count=$BAG_COUNT
bag_dir=$BAG_DIR
EOF

# 复制到 host 可访问目录
cp "$OUTPUT_DIR"/* /home/pi/rosmaster_maps/replay/ 2>/dev/null || true

echo ""
echo "============================================================"
echo "  回放完成！"
echo "  输出: $OUTPUT_DIR/"
ls -lh "$OUTPUT_DIR/"
echo ""
echo "  同步到 Mac: bash ~/Claude/Projects/rosmaster/sync_data.sh"
echo "============================================================"
