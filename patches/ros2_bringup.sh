#!/bin/bash
# ROSMaster R2 ROS2 boot script (v2 — gear-driven SLAM)
# Boot default: driver + joy + slam_supervisor ONLY. No lidar, no gmapping.
# Lidar + gmapping are started/stopped by slam_supervisor based on /MappingState
# (gear 1/2 active = mapping ON; red/inactive or gear 3 = OFF).

C="rosmaster_ros2"
TOOLS="/home/pi/rosmaster_tools"
IMG="yahboomtechnology/ros-foxy:4.0.7R2"
S="source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1"
JOY_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_ctrl/lib/python3.8/site-packages/yahboomcar_ctrl/yahboom_joy_R2.py"
DRV_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/lib/python3.8/site-packages/yahboomcar_bringup/Ackman_driver_R2.py"

modprobe joydev 2>/dev/null || true

# 1. Container: create if missing, else start
if ! docker ps -a --format '{{.Names}}' | grep -q "^${C}$"; then
    docker run -dit --restart=always --name "$C" --privileged --network host \
        -e ROBOT_TYPE=r2 -e RPLIDAR_TYPE=a1 -v /dev:/dev "$IMG"
    sleep 8
else
    docker start "$C" 2>/dev/null
    sleep 5
fi

# 2. Deploy patches
docker cp "$TOOLS/yahboom_joy_R2_patched.py" "$C:$JOY_DST"
docker cp "$TOOLS/Ackman_driver_R2_patched.py" "$C:$DRV_DST"
docker exec "$C" mkdir -p /root/rosmaster_tools /root/rosmaster_maps
docker cp "$TOOLS/slam_supervisor.py" "$C:/root/rosmaster_tools/slam_supervisor.py"

# 3. Kill any old nodes
docker exec "$C" bash -c "pkill -9 -f 'ros2|joy_node|gmapping|sllidar|rplidar|driver|ekf|imu|robot_state|joint_state|slam_supervisor'" 2>/dev/null || true
sleep 3

# 4. Driver stack (NO lidar/gmapping)
docker exec -d "$C" bash -c "$S && ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py > /tmp/bringup.log 2>&1"
sleep 8

# 5. Static TF base_link -> laser (ready for when lidar starts)
docker exec -d "$C" bash -c "$S && ros2 run tf2_ros static_transform_publisher 0.0435 5.258E-05 0.11 3.14 0 0 base_link laser > /tmp/tf.log 2>&1"

# 6. Joy: wait for gamepad dongle (up to 4 min), then start
docker exec -d "$C" bash -c "for i in \$(seq 120); do [ -e /dev/input/js0 ] && break; sleep 2; done; $S && ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=20.0 > /tmp/joy_node.log 2>&1"
# NOTE: joy_ctrl (yahboom_joy_R2) is started by the R2 bringup launch itself — do not start it again.

# 7. SLAM supervisor
docker exec -d "$C" bash -c "$S && python3 /root/rosmaster_tools/slam_supervisor.py > /tmp/slam_supervisor.log 2>&1"

# 8. On-screen live map kiosk (host side, as user pi, via XWayland)
pkill -f map_kiosk.py 2>/dev/null || true
sudo -u pi bash -c 'export DISPLAY=:0 XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-1; nohup python3 /home/pi/rosmaster_tools/map_kiosk.py > /tmp/map_kiosk.log 2>&1 &'

echo "bringup v3 done"
