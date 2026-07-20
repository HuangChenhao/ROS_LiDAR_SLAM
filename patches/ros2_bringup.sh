#!/bin/bash
# ROSMaster R2 ROS2 boot script (v5 — reliable multi-SLAM)
# Boot default: driver + joy + slam_supervisor ONLY. No lidar or SLAM.
# Lidar + selected SLAM are started/stopped by slam_supervisor based on /MappingState
# (gear 1/2 active = mapping ON; red/inactive or gear 3 = OFF).

set -Eeuo pipefail
trap 'echo "ERROR: bringup failed at line $LINENO" >&2' ERR

C="rosmaster_ros2"
TOOLS="/home/pi/rosmaster_tools"
IMG="yahboomtechnology/ros-foxy:4.0.7R2"
S="source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1 TZ=Europe/Berlin"
JOY_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_ctrl/lib/python3.8/site-packages/yahboomcar_ctrl/yahboom_joy_R2.py"
DRV_DST="/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/lib/python3.8/site-packages/yahboomcar_bringup/Ackman_driver_R2.py"
EKF_DST="/root/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/share/robot_localization/params/ekf_x1_x3.yaml"

modprobe joydev 2>/dev/null || true

# 1. Container: create if missing, else start
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

# 2. Deploy patches
docker cp "$TOOLS/yahboom_joy_R2_patched.py" "$C:$JOY_DST"
docker cp "$TOOLS/Ackman_driver_R2_patched.py" "$C:$DRV_DST"
docker exec "$C" mkdir -p /root/rosmaster_tools /root/rosmaster_maps
docker cp "$TOOLS/slam_supervisor.py" "$C:/root/rosmaster_tools/slam_supervisor.py"
docker cp "$TOOLS/slam_toolbox_r2.yaml" "$C:/root/rosmaster_tools/slam_toolbox_r2.yaml"
docker cp "$TOOLS/rtabmap_r2.yaml" "$C:/root/rosmaster_tools/rtabmap_r2.yaml"
docker cp "$TOOLS/rosmaster_carto.lua" "$C:/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_nav/share/yahboomcar_nav/params/rosmaster_carto.lua"
docker cp "$TOOLS/slam_gmapping.yaml" "$C:/root/yahboomcar_ros2_ws/software/library_ws/install/slam_gmapping/share/slam_gmapping/params/slam_gmapping.yaml"
docker cp "$TOOLS/ekf_r2.yaml" "$C:$EKF_DST"

# 3. Kill any old nodes
docker exec "$C" bash -c "pkill -9 -f 'ros2|joy_node|gmapping|sllidar|rplidar|driver|ekf|imu|robot_state|joint_state|slam_supervisor|static_transform_publisher'" 2>/dev/null || true
sleep 3

# 4. Driver stack (NO lidar/gmapping)
docker exec -d "$C" bash -c "$S && ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py > /tmp/bringup.log 2>&1"
sleep 8

# 5. Static TF base_link -> laser (ready for when lidar starts)
docker exec -d "$C" bash -c "$S && ros2 run tf2_ros static_transform_publisher 0.0435 5.258E-05 0.11 3.14 0 0 base_link laser > /tmp/tf.log 2>&1"

# 6. Joy: survive USB dongle disconnects and automatically reconnect.
docker exec -d "$C" bash -c "while true; do while [ ! -e /dev/input/js0 ]; do sleep 2; done; $S && ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=20.0 > /tmp/joy_node.log 2>&1 || true; sleep 2; done"
# NOTE: joy_ctrl (yahboom_joy_R2) is started by the R2 bringup launch itself — do not start it again.

# 7. SLAM supervisor
docker exec -d "$C" bash -c "$S && python3 /root/rosmaster_tools/slam_supervisor.py > /tmp/slam_supervisor.log 2>&1"

# 8. On-screen live map kiosk (host side, as user pi, via XWayland)
pkill -f map_kiosk.py 2>/dev/null || true
if ! sudo -u pi bash -c 'export DISPLAY=:0 XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-1; nohup python3 /home/pi/rosmaster_tools/map_kiosk.py > /tmp/map_kiosk.log 2>&1 &'; then
    echo "WARNING: map kiosk did not start; ROS2 core remains available" >&2
fi

# 9. Verify core nodes after discovery settles.
sleep 10
NODES="$(docker exec "$C" bash -c "$S && ros2 node list")"
for node in /driver_node /joy_ctrl /robot_state_publisher /slam_supervisor; do
    if ! grep -qx "$node" <<<"$NODES"; then
        echo "ERROR: required ROS2 node missing: $node" >&2
        echo "$NODES" >&2
        exit 1
    fi
done

echo "bringup v5 done"
