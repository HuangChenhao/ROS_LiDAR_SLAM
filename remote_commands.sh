#!/bin/bash
CONTAINER="rosmaster_slam"

echo "====== 1. 检查 Pi 的显示环境 ======"
echo "--- DISPLAY ---"
echo "DISPLAY=$DISPLAY"
env | grep -i display || echo "无 DISPLAY 环境变量"

echo "--- X server ---"
pgrep -a Xorg 2>/dev/null || pgrep -a Xwayland 2>/dev/null || pgrep -a weston 2>/dev/null || echo "无 X server 进程"

echo "--- Desktop session ---"
pgrep -a lxsession 2>/dev/null || pgrep -a wayfire 2>/dev/null || pgrep -a labwc 2>/dev/null || pgrep -a openbox 2>/dev/null || echo "无桌面进程"

echo "--- framebuffer ---"
ls -la /dev/fb* 2>/dev/null || echo "无 framebuffer"

echo "--- 连接的屏幕 ---"
ls /sys/class/drm/card*/status 2>/dev/null | while read f; do echo "$f: $(cat $f)"; done
tvservice -s 2>/dev/null || echo "无 tvservice"

echo "--- loginctl ---"
loginctl show-session $(loginctl | grep pi | awk '{print $1}') 2>/dev/null | grep -E "Type|Remote|Display|TTY" || echo "无 loginctl session"

echo "--- 屏幕分辨率 ---"
cat /sys/class/graphics/fb0/virtual_size 2>/dev/null || echo "未知"

echo ""
echo "====== 2. 检查 Docker X11 挂载 ======"
docker inspect $CONTAINER --format='{{range .Mounts}}{{.Source}} -> {{.Destination}}{{"\n"}}{{end}}' 2>/dev/null | grep -i x11 || echo "无 X11 挂载"
docker inspect $CONTAINER --format='{{json .HostConfig.Devices}}' 2>/dev/null

echo ""
echo "====== 3. 检查容器内 pygame ======"
docker exec $CONTAINER pip2 list 2>/dev/null | grep -i pygame || echo "pygame 未安装"

echo ""
echo "====== DONE ======"
