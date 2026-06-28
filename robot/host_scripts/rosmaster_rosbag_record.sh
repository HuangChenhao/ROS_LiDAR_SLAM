#!/bin/bash
set -euo pipefail
BAG_DIR="/home/pi/temp/bags"
SPLIT_SEC="${1:-300}"
MAX_DISK_PERCENT="${2:-90}"
CONTAINER="${ROSMASTER_CONTAINER:-rosmaster_slam}"

mkdir -p "$BAG_DIR"

# 磁盘保护：删除最老的 bag 直到磁盘使用率低于阈值
cleanup_old_bags() {
    while true; do
        usage=$(df / --output=pcent | tail -1 | tr -d ' %')
        if [ "$usage" -lt "$MAX_DISK_PERCENT" ]; then
            break
        fi
        oldest=$(ls -t "$BAG_DIR"/*.bag 2>/dev/null | tail -1)
        if [ -z "$oldest" ]; then
            echo "磁盘 ${usage}%，无 bag 可删"
            break
        fi
        echo "磁盘 ${usage}%，删除最老: $(basename $oldest)"
        rm -f "$oldest"
        sleep 1
    done
}

echo "rosbag 录制: 分割=${SPLIT_SEC}s, 磁盘上限=${MAX_DISK_PERCENT}%"

# 先清理
cleanup_old_bags

docker exec rosmaster_slam mkdir -p /root/temp/bags

# 启动录制
docker exec rosmaster_slam bash -lc "\
source /opt/ros/melodic/setup.bash && source ~/yahboomcar_ws/devel/setup.bash && \
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 && \
cd /root/temp/bags && \
rosbag record /scan /tf /tf_static /odom /imu/imu_data \
  --split --duration=${SPLIT_SEC} \
  -O slam_session" &
RECORD_PID=$!

# 后台每60秒检查磁盘
while kill -0 $RECORD_PID 2>/dev/null; do
    sleep 60
    cleanup_old_bags
done
