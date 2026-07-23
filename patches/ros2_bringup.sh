#!/bin/bash
# ROSMaster R2 ROS2 boot script (v6 - serial-safe multi-SLAM)

set -Eeuo pipefail
trap 'echo "ERROR: bringup failed at line $LINENO" >&2' ERR

C="rosmaster_ros2"
TOOLS="/home/pi/rosmaster_tools"
IMG="yahboomtechnology/ros-foxy:4.0.7R2"
S="source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1 TZ=Europe/Berlin"
JOY_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_ctrl/lib/python3.8/site-packages/yahboomcar_ctrl/yahboom_joy_R2.py"
DRV_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/lib/python3.8/site-packages/yahboomcar_bringup/Ackman_driver_R2.py"
EKF_DST="/root/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/share/robot_localization/params/ekf_x1_x3.yaml"
BRINGUP_LAUNCH_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/launch/yahboomcar_bringup_R2_launch.py"
CARTO_LAUNCH_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_nav/share/yahboomcar_nav/launch/cartographer_launch.py"
LEGACY_AUTOSTART="/home/pi/.config/autostart/rosmaster.desktop"

disable_legacy_serial_owner() {
    # The legacy desktop app opens /dev/myserial before ROS2 and kills the
    # Rosmaster_Lib receive thread. Preserve its file as a reversible backup.
    if [[ -f "$LEGACY_AUTOSTART" ]]; then
        backup="${LEGACY_AUTOSTART}.disabled"
        [[ ! -e "$backup" ]] || backup="${backup}.$(date +%s)"
        mv "$LEGACY_AUTOSTART" "$backup"
        echo "disabled legacy autostart: $backup"
    fi

    for pid in $(fuser /dev/myserial 2>/dev/null || true); do
        cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
        if [[ "$cmdline" == *'/home/pi/Rosmaster/rosmaster/rosmaster_main.py'* ]]; then
            echo "stopping legacy serial owner pid=$pid"
            kill -TERM "$pid" 2>/dev/null || true
            for _ in $(seq 1 20); do
                [[ ! -d "/proc/$pid" ]] && break
                sleep 0.1
            done
            [[ ! -d "/proc/$pid" ]] || kill -KILL "$pid" 2>/dev/null || true
        fi
    done

    if fuser /dev/myserial >/dev/null 2>&1; then
        echo "ERROR: /dev/myserial is still owned before ROS2 driver start" >&2
        fuser -v /dev/myserial >&2 || true
        exit 1
    fi
}

modprobe joydev 2>/dev/null || true

# 1. Container lifecycle.
if ! docker ps -a --format '{{.Names}}' | grep -q "^${C}$"; then
    docker run -dit --restart=always --name "$C" --privileged --network host \
        -e ROBOT_TYPE=r2 -e RPLIDAR_TYPE=a1 -e TZ=Europe/Berlin -v /dev:/dev "$IMG"
    sleep 8
else
    docker start "$C" >/dev/null
    sleep 5
fi

for _ in $(seq 30); do
    docker exec "$C" true >/dev/null 2>&1 && break
    sleep 1
done
docker exec "$C" true >/dev/null

# 2. Deploy the versioned runtime files.
docker cp "$TOOLS/yahboom_joy_R2_patched.py" "$C:$JOY_DST"
docker cp "$TOOLS/Ackman_driver_R2_patched.py" "$C:$DRV_DST"
docker cp "$TOOLS/yahboomcar_bringup_R2_launch.py" "$C:$BRINGUP_LAUNCH_DST"
docker cp "$TOOLS/cartographer_launch.py" "$C:$CARTO_LAUNCH_DST"
docker cp "$TOOLS/ekf_r2.yaml" "$C:$EKF_DST"
docker exec "$C" mkdir -p /root/rosmaster_tools /root/rosmaster_maps
for file in slam_supervisor.py slam_toolbox_r2.yaml rtabmap_r2.yaml \
            rosmaster_odom.py robot_healthcheck.py; do
    docker cp "$TOOLS/$file" "$C:/root/rosmaster_tools/$file"
done
docker cp "$TOOLS/rosmaster_carto.lua" \
    "$C:/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_nav/share/yahboomcar_nav/params/rosmaster_carto.lua"
docker cp "$TOOLS/slam_gmapping.yaml" \
    "$C:/root/yahboomcar_ros2_ws/software/library_ws/install/slam_gmapping/share/slam_gmapping/params/slam_gmapping.yaml"
docker exec "$C" chmod +x /root/rosmaster_tools/rosmaster_odom.py /root/rosmaster_tools/robot_healthcheck.py

# 3. Stop the previous ROS graph, then prove the chassis serial is exclusive.
docker exec "$C" bash -c \
    "$S && timeout 1.0s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true" || true
docker exec "$C" bash -c \
    "pkill -9 -f 'ros2|joy_node|gmapping|sllidar|rplidar|driver|base_node|rosmaster_odom|ekf|imu|robot_state|joint_state|slam_supervisor|static_transform_publisher'" \
    2>/dev/null || true
sleep 3
disable_legacy_serial_owner

# 4. Core driver, stable odometry, IMU, EKF and controller. No LiDAR yet.
docker exec -d "$C" bash -c \
    "$S && ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py > /tmp/bringup.log 2>&1"
sleep 8

# 5. Fixed LiDAR transform, available before a mapping session starts.
docker exec -d "$C" bash -c \
    "$S && ros2 run tf2_ros static_transform_publisher 0.0435 5.258E-05 0.11 3.14 0 0 base_link laser > /tmp/tf.log 2>&1"

# 6. Reconnect the USB gamepad automatically.
docker exec -d "$C" bash -c \
    "while true; do while [ ! -e /dev/input/js0 ]; do sleep 2; done; $S && ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=20.0 > /tmp/joy_node.log 2>&1 || true; sleep 2; done"

# 7. SLAM lifecycle supervisor.
docker exec -d "$C" bash -c \
    "$S && python3 /root/rosmaster_tools/slam_supervisor.py > /tmp/slam_supervisor.log 2>&1"

# 8. Host-side live map display.
pkill -f map_kiosk.py 2>/dev/null || true
if ! sudo -u pi bash -c \
    'export DISPLAY=:0 XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-1; nohup python3 /home/pi/rosmaster_tools/map_kiosk.py > /tmp/map_kiosk.log 2>&1 &'; then
    echo "WARNING: map kiosk did not start; ROS2 core remains available" >&2
fi

# 9. Verify node presence and actual serial/IMU/odometry data.
sleep 10
NODES="$(docker exec "$C" bash -c "$S && ros2 node list")"
for node in /driver_node /base_node /ekf_filter_node /imu_filter_madgwick \
            /joy_ctrl /robot_state_publisher /slam_supervisor; do
    if ! grep -qx "$node" <<<"$NODES"; then
        echo "ERROR: required ROS2 node missing: $node" >&2
        echo "$NODES" >&2
        exit 1
    fi
done

docker exec "$C" bash -c \
    "$S && python3 /root/rosmaster_tools/robot_healthcheck.py --duration 4.0"

echo "bringup v6 done - serial, IMU and odometry healthy"
