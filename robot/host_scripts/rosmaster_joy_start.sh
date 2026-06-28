#!/bin/bash
set -euo pipefail
CONTAINER="${ROSMASTER_CONTAINER:-rosmaster_slam}"
ROS_SETUP="source /opt/ros/melodic/setup.bash && source ~/yahboomcar_ws/devel/setup.bash && export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1"

# 1. 自动检测手柄 js 设备
JS_DEV=""
for js in /dev/input/js1 /dev/input/js0; do
    if [ -e "$js" ]; then
        name=$(cat /sys/class/input/$(basename $js)/device/name 2>/dev/null || echo "unknown")
        echo "检查 $js: $name"
        if echo "$name" | grep -iqE "xbox|x-box|gamepad|pad"; then
            JS_DEV="$js"
            break
        fi
    fi
done
if [ -z "$JS_DEV" ]; then
    [ -e /dev/input/js1 ] && JS_DEV="/dev/input/js1" || JS_DEV="/dev/input/js0"
fi
echo "手柄设备: $JS_DEV"

# 2. 清理旧 joy_node
docker exec "$CONTAINER" bash -lc "$ROS_SETUP && rosnode kill /joy_node 2>/dev/null" || true
sleep 2

# 3. 启动 joy_node（绑正确设备 + autorepeat）
docker exec -d "$CONTAINER" bash -lc "\
$ROS_SETUP && rosrun joy joy_node _dev:=$JS_DEV _autorepeat_rate:=20 _deadzone:=0.1 \
> /root/rosmaster_slam_logs/04_joy_node.log 2>&1"
sleep 3

# 4. 确保 yahboom_joy 在运行（处理 cmd_vel 输出）
if ! docker exec "$CONTAINER" bash -lc "pgrep -f 'yahboom_joy.py' >/dev/null 2>&1"; then
    echo "启动 yahboom_joy..."
    docker exec -d "$CONTAINER" bash -lc "\
    $ROS_SETUP && python /root/yahboomcar_ws/src/yahboomcar_ctrl/scripts/yahboom_joy.py \
    > /root/rosmaster_slam_logs/05_yahboom_joy.log 2>&1"
    sleep 2
else
    echo "yahboom_joy 已在运行"
fi

# 5. 验证
docker exec "$CONTAINER" bash -lc "$ROS_SETUP && timeout 3 rostopic echo /joy -n 1 >/dev/null 2>&1" \
    && echo "joy_node OK (/joy 有消息)" \
    || echo "WARNING: /joy 无消息，手柄可能未连接"
