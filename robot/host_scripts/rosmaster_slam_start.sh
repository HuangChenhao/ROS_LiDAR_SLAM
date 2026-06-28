#!/bin/bash
set -euo pipefail

CONTAINER="${ROSMASTER_CONTAINER:-rosmaster_slam}"
IMAGE="${ROSMASTER_IMAGE:-yahboomtechnology/ros-melodic:4.0.1}"
MAP_TYPE="${1:-gmapping}"
ROBOT_IP="${ROSMASTER_IP:-127.0.0.1}"
ROBOT_TYPE_VALUE="${ROBOT_TYPE:-X3}"
RPLIDAR_TYPE_VALUE="${RPLIDAR_TYPE:-a1}"
LOG_DIR="/root/rosmaster_slam_logs"
ROS_SETUP="source /opt/ros/melodic/setup.bash && source ~/yahboomcar_ws/devel/setup.bash && export ROS_MASTER_URI=http://${ROBOT_IP}:11311 ROS_IP=${ROBOT_IP} ROBOT_TYPE=${ROBOT_TYPE_VALUE} RPLIDAR_TYPE=${RPLIDAR_TYPE_VALUE}"

require_device() {
    if [ ! -e "$1" ]; then
        echo "Missing required device: $1" >&2
        exit 1
    fi
}

optional_device_args=()
add_optional_device() {
    if [ -e "$1" ]; then
        optional_device_args+=("--device=$1")
    fi
}

require_device /dev/myserial
require_device /dev/rplidar

add_optional_device /dev/astradepth
add_optional_device /dev/astrauvc
add_optional_device /dev/camera_depth
add_optional_device /dev/input
add_optional_device /dev/video0
add_optional_device /dev/video1

if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER"; then
    echo "Container already running: $CONTAINER"
elif docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER"; then
    echo "Starting existing container: $CONTAINER"
    docker start "$CONTAINER" >/dev/null
else
    echo "Creating container: $CONTAINER"
    docker run -dit \
        --name "$CONTAINER" \
        -m 4g \
        --memory-swap 4g \
        --net=host \
        --env="ROS_MASTER_URI=http://${ROBOT_IP}:11311" \
        --env="ROS_IP=${ROBOT_IP}" \
        --env="ROBOT_TYPE=${ROBOT_TYPE_VALUE}" \
        --env="YAHBOOM_BASE_TYPE=R2" \
        --env="RPLIDAR_TYPE=${RPLIDAR_TYPE_VALUE}" \
        --env="QT_X11_NO_MITSHM=1" \
        --security-opt apparmor:unconfined \
        -v /home/pi/temp:/root/temp \
        --device=/dev/myserial \
        --device=/dev/rplidar \
        "${optional_device_args[@]}" \
        "$IMAGE" \
        bash -lc "tail -f /dev/null" >/dev/null
fi

docker exec "$CONTAINER" bash -lc "mkdir -p '$LOG_DIR'"

start_if_missing() {
    local label="$1"
    local pattern="$2"
    local command="$3"
    local log_file="$4"

    if docker exec "$CONTAINER" bash -lc "pgrep -f '$pattern' >/dev/null"; then
        echo "$label already running"
    else
        echo "Starting $label"
        docker exec -d "$CONTAINER" bash -lc "$ROS_SETUP && $command > '$LOG_DIR/$log_file' 2>&1"
    fi
}

start_if_missing "roscore" "[r]oscore" "roscore" "01_roscore.log"

echo "Waiting for ROS master..."
docker exec "$CONTAINER" bash -lc "$ROS_SETUP && for i in \$(seq 1 20); do rostopic list >/dev/null 2>&1 && exit 0; sleep 1; done; exit 1"

start_if_missing "laser bringup" "[r]oslaunch yahboomcar_nav laser_bringup.launch" "roslaunch yahboomcar_nav laser_bringup.launch" "02_laser_bringup.log"

echo "Waiting for /scan and /odom..."
docker exec "$CONTAINER" bash -lc "$ROS_SETUP && for i in \$(seq 1 30); do rostopic list 2>/dev/null | grep -q '^/scan$' && rostopic list 2>/dev/null | grep -q '^/odom$' && exit 0; sleep 1; done; exit 1"

start_if_missing "gmapping" "[r]oslaunch yahboomcar_nav yahboomcar_map.launch" "roslaunch yahboomcar_nav yahboomcar_map.launch map_type:=$MAP_TYPE use_rviz:=false" "03_gmapping.log"

echo "Waiting for /map_metadata..."
docker exec "$CONTAINER" bash -lc "$ROS_SETUP && for i in \$(seq 1 30); do rostopic list 2>/dev/null | grep -q '^/map_metadata$' && exit 0; sleep 1; done; exit 1"

/home/pi/rosmaster_tools/rosmaster_joy_start.sh || echo "joy node was not started"

echo "SLAM stack is running."
echo "Container: $CONTAINER"
echo "Logs inside container: $LOG_DIR"
echo "Use rosmaster_slam_status.sh to inspect topics."
echo "Use rosmaster_map_save.sh my_map after driving the robot around."

# ========== 轨迹记录 ==========
echo "启动轨迹记录..."
docker exec -d $CONTAINER bash -lc "\
source /opt/ros/melodic/setup.bash && source ~/yahboomcar_ws/devel/setup.bash && \
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 && \
python /root/yahboomcar_ws/src/yahboomcar_bringup/scripts/trajectory_recorder.py \
_save_dir:=/root/rosmaster_slam_logs _save_interval:=30 _min_distance:=0.02 \
> /root/rosmaster_slam_logs/06_trajectory.log 2>&1"
echo "轨迹记录已启动"
