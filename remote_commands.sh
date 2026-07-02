#!/bin/bash
echo "=== P0: 编码器诊断 + ROS2 镜像准备 ==="

echo "--- 1. 编码器诊断 (在 ROS1 容器内) ---"
docker exec rosmaster_slam bash -c '
source /opt/ros/melodic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
python2 << "PYEOF"
import time
from Rosmaster_Lib import Rosmaster

car = Rosmaster()
print("Setting car type to R2...")
car.set_car_type(car.CARTYPE_R2)
time.sleep(0.5)

# 1. PID
try:
    pid = car.get_motion_pid()
    print("PID params: {}".format(pid))
except Exception as e:
    print("PID error: {}".format(e))

# 2. Encoder test - drive forward slowly
print("\nDriving forward at 0.2 m/s for 2 seconds...")
car.set_car_motion(0.2, 0, 0)
time.sleep(2)

# Read encoder data 3 times
for i in range(3):
    try:
        vx, vy, angular = car.get_motion_data()
        print("Encoder read {}: vx={}, vy={}, angular={}".format(i+1, vx, vy, angular))
    except Exception as e:
        print("Encoder read {} error: {}".format(i+1, e))
    time.sleep(0.3)

car.set_car_motion(0, 0, 0)
time.sleep(0.5)

# 3. Version info
try:
    ver = car.get_version()
    print("\nFirmware version: {}".format(ver))
except:
    print("Cannot read firmware version")

# 4. Battery
try:
    bat = car.get_battery_voltage()
    print("Battery: {}V".format(bat))
except:
    print("Cannot read battery")

print("\nEncoder test complete.")
PYEOF
'

echo ""
echo "--- 2. 检查 ROS2 Docker 镜像 ---"
echo "已有镜像:"
docker images | grep -E "ros|yahboom|foxy"

echo ""
echo "磁盘空间:"
df -h / | tail -1

echo ""
echo "--- 3. 拉取 ROS2 镜像 (如果不存在) ---"
if docker images | grep -q "ros-foxy"; then
    echo "ROS2 镜像已存在!"
    docker images | grep ros-foxy
else
    echo "开始拉取 yahboomtechnology/ros-foxy:4.0.7R2 ..."
    echo "(这需要较长时间，镜像约 7GB)"
    docker pull yahboomtechnology/ros-foxy:4.0.7R2 2>&1 | tail -5
fi

echo ""
echo "--- 4. 检查 run_docker.sh ---"
if [ -f ~/run_docker.sh ]; then
    echo "run_docker.sh 存在:"
    cat ~/run_docker.sh
elif [ -f ~/Rosmaster/run_docker.sh ]; then
    echo "Rosmaster/run_docker.sh 存在:"
    cat ~/Rosmaster/run_docker.sh
else
    echo "未找到 run_docker.sh"
    find /home/pi -name "run_docker*" -o -name "set_R2*" 2>/dev/null | head -10
fi

echo ""
echo "--- 5. 检查 R2 配置脚本 ---"
find /home/pi -path "*/RobotType/set_R2*" -o -path "*/Rosmaster/set*" 2>/dev/null | head -10

echo ""
echo "=== DONE ==="
