#!/bin/bash
HOST="192.168.0.110"
USER="pi"
OUTPUT="$(dirname "$0")/fix_and_start_output.txt"

echo "正在尝试修复并启动 ROS..."

ssh "${USER}@${HOST}" bash << 'REMOTE_COMMANDS' > "$OUTPUT" 2>&1

echo "====== 1. 检查必需设备 ======"
echo -n "/dev/myserial: "; ls -la /dev/myserial 2>&1
echo -n "/dev/rplidar: "; ls -la /dev/rplidar 2>&1
echo -n "/dev/astradepth: "; ls -la /dev/astradepth 2>&1
echo -n "/dev/astrauvc: "; ls -la /dev/astrauvc 2>&1
echo ""

# 检查 rplidar 的 udev 规则
echo "====== 2. rplidar udev 规则 ======"
grep -r "rplidar\|lidar\|cp210x\|ch340" /etc/udev/rules.d/ 2>/dev/null
echo ""

# 检查串口映射
echo "====== 3. 串口设备映射 ======"
ls -la /dev/serial/by-id/ 2>/dev/null
ls -la /dev/serial/by-path/ 2>/dev/null
ls -la /dev/myserial 2>/dev/null
ls -la /dev/ttyUSB* 2>/dev/null
echo ""

# 检查所有 udev 自定义规则
echo "====== 4. 所有自定义 udev 规则 ======"
ls -la /etc/udev/rules.d/ 2>/dev/null
echo ""
for f in /etc/udev/rules.d/*.rules; do
    echo "--- $f ---"
    cat "$f" 2>/dev/null
    echo ""
done

# 如果 rplidar 不存在，尝试找到它
if [ ! -e /dev/rplidar ]; then
    echo "====== 5. rplidar 设备不存在，尝试定位 ======"
    echo "重新加载 udev 规则..."
    sudo udevadm control --reload-rules 2>&1
    sudo udevadm trigger 2>&1
    sleep 2
    echo -n "重新检查 /dev/rplidar: "; ls -la /dev/rplidar 2>&1
fi

echo ""

# 尝试删除旧容器并重启服务
echo "====== 6. 尝试重启 rosmaster_slam ======"
echo "停止旧容器..."
docker stop rosmaster_slam 2>&1
echo "删除旧容器（设备映射已过时）..."
docker rm rosmaster_slam 2>&1
echo ""

echo "重启 rosmaster-slam 服务..."
sudo systemctl restart rosmaster-slam.service 2>&1
sleep 5

echo "====== 7. 服务状态 ======"
systemctl status rosmaster-slam.service --no-pager 2>&1
echo ""

echo "====== 8. 容器状态 ======"
docker ps -a --filter name=rosmaster_slam 2>&1
echo ""

# 如果容器在运行，检查 ROS
if docker ps --format '{{.Names}}' | grep -Fxq rosmaster_slam; then
    echo "====== 9. ROS 检查 ======"
    docker exec rosmaster_slam bash -lc "source /opt/ros/melodic/setup.bash && source ~/yahboomcar_ws/devel/setup.bash && rosnode list" 2>&1
    echo ""
    docker exec rosmaster_slam bash -lc "source /opt/ros/melodic/setup.bash && source ~/yahboomcar_ws/devel/setup.bash && rostopic list" 2>&1
else
    echo "容器未运行，查看日志..."
    sudo journalctl -u rosmaster-slam.service --no-pager -n 30 2>&1
fi

echo ""
echo "====== 完成 ======"

REMOTE_COMMANDS

echo "完成！结果: $OUTPUT"
