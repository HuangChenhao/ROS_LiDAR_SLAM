#!/bin/bash
# ROSMaster R2 诊断脚本 - 在 Mac 上运行，自动 SSH 到小车执行检查

HOST="192.168.0.110"
USER="pi"
OUTPUT="$(dirname "$0")/diagnose_output.txt"

echo "正在连接小车 $HOST 并执行诊断..."
echo "(需要输入密码: yahboom)"

ssh "${USER}@${HOST}" bash << 'REMOTE_COMMANDS' > "$OUTPUT" 2>&1

echo "====== 系统信息 ======"
uname -a
echo ""

echo "====== ROS 环境 ======"
# 尝试加载 ROS
for f in /opt/ros/*/setup.bash; do
    echo "找到 ROS: $f"
    source "$f" 2>/dev/null
done
for f in ~/catkin_ws/devel/setup.bash ~/yahboomcar_ws/devel/setup.bash ~/ros_ws/devel/setup.bash; do
    [ -f "$f" ] && echo "找到工作空间: $f" && source "$f" 2>/dev/null
done
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo ""

echo "====== ROS 节点 ======"
rosnode list 2>&1 || echo "rosnode 不可用或 roscore 未运行"
echo ""

echo "====== ROS 话题 ======"
rostopic list 2>&1 || echo "rostopic 不可用"
echo ""

echo "====== 蜂鸣器/报警相关话题 ======"
rostopic list 2>/dev/null | grep -i -E "buzz|beep|alarm|volt|batt|warn|diag" || echo "未找到相关话题"
echo ""

echo "====== 运行中的 ROS/机器人进程 ======"
ps aux | grep -i -E "roscore|roslaunch|rosrun|yahboom|buzzer|beep|alarm|robot" | grep -v grep
echo ""

echo "====== systemd 服务（机器人相关）======"
systemctl list-units --state=running 2>/dev/null | grep -i -E "ros|yahboom|robot|car|buzzer"
echo ""

echo "====== 开机自启服务 ======"
systemctl list-unit-files --state=enabled 2>/dev/null | grep -i -E "ros|yahboom|robot|car|start"
ls /etc/rc.local 2>/dev/null && echo "--- rc.local 内容 ---" && cat /etc/rc.local
echo ""

echo "====== GPIO 状态（蜂鸣器可能用 GPIO）======"
for f in /sys/class/gpio/gpio*/value; do
    [ -f "$f" ] && echo "$f: $(cat $f)"
done
echo ""

echo "====== I2C 设备 ======"
i2cdetect -y 1 2>/dev/null || echo "i2cdetect 不可用"
echo ""

echo "====== 查找蜂鸣器相关代码 ======"
find /home/pi -maxdepth 4 -name "*.py" 2>/dev/null | xargs grep -l -i "buzzer\|beep" 2>/dev/null | head -20
echo ""

echo "====== dmesg 最近错误 ======"
dmesg | tail -30
echo ""

echo "====== 诊断完成 ======"

REMOTE_COMMANDS

echo ""
echo "诊断完成！结果已保存到: $OUTPUT"
echo "文件大小: $(wc -c < "$OUTPUT") bytes"
