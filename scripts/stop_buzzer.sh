#!/bin/bash
HOST="192.168.0.110"
USER="pi"
OUTPUT="$(dirname "$0")/stop_buzzer_output.txt"

echo "正在尝试多种方式关闭蜂鸣器..."

ssh "${USER}@${HOST}" bash << 'REMOTE_COMMANDS' > "$OUTPUT" 2>&1

python3 << 'PYEOF'
import sys, time
sys.path.insert(0, '/home/pi/software/py_install/Rosmaster_Lib')
from Rosmaster_Lib import Rosmaster

bot = Rosmaster()
time.sleep(0.5)

# 方法1: set_beep(0) 多次
print("尝试1: set_beep(0)")
for i in range(5):
    bot.set_beep(0)
    time.sleep(0.1)

# 方法2: reset_car_state 完整重置
print("尝试2: reset_car_state()")
bot.reset_car_state()
time.sleep(0.5)

# 方法3: 直接发送串口指令关闭蜂鸣器
print("尝试3: 直接串口发送关闭命令")
import struct
HEAD = 0xFF
DEVICE_ID = 0xFC
COMPLEMENT = 257 - DEVICE_ID
FUNC_BEEP = 0x02
value = bytearray(struct.pack('h', 0))
cmd = [HEAD, DEVICE_ID, 0x05, FUNC_BEEP, value[0], value[1]]
checksum = sum(cmd, COMPLEMENT) & 0xff
cmd.append(checksum)
bot.ser.write(bytes(cmd))
time.sleep(0.1)
bot.ser.write(bytes(cmd))
time.sleep(0.1)
bot.ser.write(bytes(cmd))
print("串口命令已发送:", [hex(x) for x in cmd])

# 方法4: 再次 reset
print("尝试4: 再次 reset_car_state()")
bot.reset_car_state()
time.sleep(0.5)

# 方法5: 查看电池电压（低压报警可能是底板硬件级别的）
print("尝试5: 读取电池电压")
bot.create_receive_threading()
time.sleep(1)
voltage = bot.get_battery_voltage()
print(f"电池电压: {voltage}")

# 如果电压低，这可能是硬件低压报警，软件关不掉
if voltage and voltage > 0:
    if voltage < 10:
        print(f"!!! 电池电压过低 ({voltage}V)，蜂鸣器可能是硬件低压报警，需要充电才能停止 !!!")
    else:
        print(f"电池电压正常 ({voltage}V)")

print("所有尝试完成")
del bot
PYEOF

REMOTE_COMMANDS

echo "结果已保存: $OUTPUT"
cat "$(dirname "$0")/stop_buzzer_output.txt"
