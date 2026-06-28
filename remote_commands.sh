#!/bin/bash
CONTAINER="rosmaster_slam"

echo "====== 1. Wayland/X11 sockets ======"
ls -la /run/user/1000/wayland* 2>/dev/null || echo "无 wayland socket"
ls -la /tmp/.X11-unix/ 2>/dev/null || echo "无 X11 socket"

echo ""
echo "====== 2. XWayland 可用性 ======"
which Xwayland 2>/dev/null || echo "Xwayland 未安装"
dpkg -l | grep -i xwayland 2>/dev/null | head -3

echo ""
echo "====== 3. DRI/GPU 设备 ======"
ls -la /dev/dri/ 2>/dev/null || echo "无 /dev/dri"

echo ""
echo "====== 4. Docker 网络模式 ======"
docker inspect $CONTAINER --format='{{.HostConfig.NetworkMode}}'

echo ""
echo "====== 5. Docker 运行参数 (display相关) ======"
docker inspect $CONTAINER --format='{{json .Config.Env}}' | tr ',' '\n' | grep -iE "display|wayland|sdl|xdg" || echo "无显示相关环境变量"

echo ""
echo "====== 6. Pi host 上的 Python ======"
python3 --version 2>/dev/null
pip3 list 2>/dev/null | grep -iE "pygame|rospy|roslibpy" || echo "host无ROS/pygame"

echo ""
echo "====== 7. Docker 容器 pip 版本 ======"
docker exec $CONTAINER pip --version 2>/dev/null || echo "无pip"
docker exec $CONTAINER pip2 --version 2>/dev/null || echo "无pip2"
docker exec $CONTAINER python --version 2>/dev/null
docker exec $CONTAINER python2 --version 2>/dev/null

echo ""
echo "====== 8. Docker run 完整命令 ======"
# 尝试从 docker inspect 获取完整创建参数
docker inspect $CONTAINER --format='Privileged: {{.HostConfig.Privileged}}'
docker inspect $CONTAINER --format='PidMode: {{.HostConfig.PidMode}}'
docker inspect $CONTAINER --format='IPC: {{.HostConfig.IpcMode}}'

echo ""
echo "====== 9. roslibpy 可行性 (host→container ROS) ======"
# 检查 rosbridge 是否在运行
docker exec $CONTAINER rosnode list 2>/dev/null | grep -i bridge || echo "无 rosbridge"
# 检查 ROS master 端口
docker exec $CONTAINER bash -c 'echo $ROS_MASTER_URI' 2>/dev/null

echo ""
echo "====== DONE ======"
