#!/bin/bash
HOST="192.168.0.110"
USER="pi"
OUTPUT="$(dirname "$0")/full_check_output.txt"

echo "正在执行全面检查..."

ssh "${USER}@${HOST}" bash << 'REMOTE_COMMANDS' > "$OUTPUT" 2>&1

echo "========== 1. 深度相机检查 =========="
echo "--- /dev/astradepth ---"
ls -la /dev/astradepth 2>&1
echo ""

echo "--- USB 设备列表 ---"
lsusb 2>&1
echo ""

echo "--- 所有 /dev 下的相机/深度设备 ---"
ls -la /dev/video* /dev/astra* /dev/cam* 2>/dev/null || echo "未找到视频/相机设备"
echo ""

echo "--- udev 规则（查看 astradepth 如何映射）---"
grep -r "astradepth" /etc/udev/rules.d/ 2>/dev/null
grep -r "astra" /etc/udev/rules.d/ 2>/dev/null
echo ""

echo "--- USB 设备详情 ---"
lsusb -t 2>&1
echo ""

echo "========== 2. Docker 检查 =========="
echo "--- Docker 服务状态 ---"
systemctl status docker --no-pager 2>&1 | head -15
echo ""

echo "--- Docker 容器列表（全部）---"
docker ps -a 2>&1
echo ""

echo "--- rosmaster_slam 容器详情 ---"
docker inspect rosmaster_slam 2>&1 | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    if data:
        c = data[0]
        print('Image:', c.get('Config',{}).get('Image',''))
        print('Status:', c.get('State',{}).get('Status',''))
        print('Devices:', json.dumps(c.get('HostConfig',{}).get('Devices',[]), indent=2))
        print('Binds:', json.dumps(c.get('HostConfig',{}).get('Binds',[]), indent=2))
        print('Cmd:', c.get('Config',{}).get('Cmd',''))
        print('Entrypoint:', c.get('Config',{}).get('Entrypoint',''))
except:
    print('解析失败')
" 2>&1
echo ""

echo "========== 3. 启动脚本内容 =========="
echo "--- rosmaster_slam_start.sh ---"
cat /home/pi/rosmaster_tools/rosmaster_slam_start.sh 2>&1
echo ""

echo "--- rosmaster_tools 目录 ---"
ls -la /home/pi/rosmaster_tools/ 2>&1
echo ""

echo "========== 4. ROS 环境检查 =========="
echo "--- ROS 安装 ---"
ls /opt/ros/ 2>&1
echo ""

echo "--- catkin 工作空间 ---"
ls -la /home/pi/yahboomcar_ws/ 2>&1 | head -20
ls -la /home/pi/catkin_ws/ 2>&1 | head -10
echo ""

echo "--- ROS setup 测试 ---"
source /opt/ros/noetic/setup.bash 2>&1 && echo "ROS noetic OK" || echo "noetic 不存在"
source /opt/ros/humble/setup.bash 2>&1 && echo "ROS humble OK" || echo "humble 不存在"
source /opt/ros/melodic/setup.bash 2>&1 && echo "ROS melodic OK" || echo "melodic 不存在"
echo ""

echo "--- Python3 ROS 模块 ---"
python3 -c "import rospy; print('rospy OK')" 2>&1
python3 -c "import rclpy; print('rclpy OK')" 2>&1
echo ""

echo "--- .bashrc 中 ROS 相关 ---"
grep -i "ros\|catkin\|source.*setup" /home/pi/.bashrc 2>/dev/null
echo ""

echo "========== 5. 网络与系统 =========="
echo "--- 磁盘空间 ---"
df -h / 2>&1
echo ""

echo "--- 内存 ---"
free -h 2>&1
echo ""

echo "--- 系统负载 ---"
uptime 2>&1
echo ""

echo "========== 检查完成 =========="

REMOTE_COMMANDS

echo "检查完成！结果: $OUTPUT"
