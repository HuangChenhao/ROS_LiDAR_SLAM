#!/bin/bash
HOST="192.168.0.110"
USER="pi"
OUTPUT="$(dirname "$0")/diagnose2_output.txt"

echo "正在连接小车执行第二轮诊断..."

ssh "${USER}@${HOST}" bash << 'REMOTE_COMMANDS' > "$OUTPUT" 2>&1

echo "====== Rosmaster_Lib 蜂鸣器相关代码 ======"
grep -n -i -A5 "buzzer\|beep" /home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py 2>/dev/null | head -80
echo ""

echo "====== rosmaster_main.py 蜂鸣器相关 ======"
grep -n -i -A5 "buzzer\|beep" /home/pi/Rosmaster/rosmaster/rosmaster_main.py 2>/dev/null | head -80
echo ""

echo "====== OLED 脚本内容 ======"
cat /home/pi/software/oled_yahboom/yahboom_oled.py 2>/dev/null | head -100
echo ""

echo "====== rosmaster-slam.service 内容 ======"
cat /etc/systemd/system/rosmaster-slam.service 2>/dev/null || systemctl cat rosmaster-slam.service 2>/dev/null
echo ""

echo "====== rosmaster-slam 服务状态 ======"
systemctl status rosmaster-slam.service 2>&1
echo ""

echo "====== Rosmaster_Lib 完整内容（前200行）======"
head -200 /home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py 2>/dev/null
echo ""

echo "====== 尝试查找串口设备 ======"
ls -la /dev/ttyUSB* /dev/ttyACM* /dev/serial* /dev/ttyAMA* 2>/dev/null
echo ""

echo "====== 尝试直接关闭蜂鸣器 ======"
python3 -c "
import sys
sys.path.insert(0, '/home/pi/software/py_install/Rosmaster_Lib')
try:
    from Rosmaster_Lib import Rosmaster
    bot = Rosmaster()
    bot.set_beep(0)
    print('已尝试关闭蜂鸣器 (set_beep(0))')
except Exception as e:
    print(f'关闭失败: {e}')

try:
    bot.set_buzzer(0)
    print('已尝试关闭蜂鸣器 (set_buzzer(0))')
except Exception as e:
    print(f'set_buzzer 失败: {e}')
" 2>&1
echo ""

echo "====== 诊断2完成 ======"

REMOTE_COMMANDS

echo "诊断完成！结果: $OUTPUT"
