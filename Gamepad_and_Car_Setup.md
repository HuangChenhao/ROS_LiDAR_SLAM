# ROSMaster R2 Gamepad & Car Setup Guide / 手柄与小车设置指南

## Overview / 概述

This document describes the Yahboom ROSMaster R2 setup with ROS2 Foxy, including gamepad control, LED gear indicators, and SLAM mapping.

本文档介绍 Yahboom ROSMaster R2 在 ROS2 Foxy 下的配置，包括手柄控制、LED 档位指示和 SLAM 建图。

---

## Hardware / 硬件

| Component / 组件 | Detail / 详情 |
|---|---|
| Robot | Yahboom ROSMaster R2 (Ackermann 2-wheel steering, wheelbase 0.25m) |
| SBC | Raspberry Pi 5 |
| Gamepad / 手柄 | Flydigi Direwolf 3 (Xbox 360 emulation mode, VID:045e PID:028e) |
| LiDAR | RPLidar A1 |
| Connection / 连接 | **USB 2.4GHz dongle** (plug into Pi USB port) / **USB 2.4GHz 接收器**（插入树莓派 USB 口） |

---

## Docker Container / Docker 容器

- **Image**: `yahboomtechnology/ros-foxy:4.0.7R2`
- **Key env vars / 关键环境变量**:
  - `ROBOT_TYPE=r2` (must be lowercase! / 必须小写!)
  - `RPLIDAR_TYPE=a1`
- **Devices**: `/dev/input/js0` (gamepad), `/dev/myserial` (STM32), `/dev/rplidar` (LiDAR), `/dev/astrapro` (optional camera)

### Start container / 启动容器

```bash
docker run -dit --restart=always --name rosmaster_ros2 \
  --privileged --network host \
  -e ROBOT_TYPE=r2 -e RPLIDAR_TYPE=a1 \
  -v /dev:/dev \
  yahboomtechnology/ros-foxy:4.0.7R2
```

---

## Gamepad Button Mapping / 手柄按键映射

The gamepad must be in **Xbox 360 emulation mode** (X-input). Plug the **USB 2.4GHz dongle** into a Pi USB port — no Bluetooth pairing needed.

手柄必须处于 **Xbox 360 模拟模式**（X-input）。将 **USB 2.4GHz 接收器**插入树莓派 USB 口即可，无需蓝牙配对。

### Axes / 摇杆

| Axis | Function / 功能 |
|---|---|
| Left Stick Y (`axes[1]`) | Forward / Backward — 前进/后退 |
| Right Stick X (`axes[3]`) | Steering — 转向 |

### Buttons / 按键

| Button / 按键 | Index | Function / 功能 |
|---|---|---|
| **Back** | 6 | Toggle joy control ON/OFF — 开关手柄控制 |
| **LB** | 4 | Cycle speed gear (1→2→3) + auto LED — 切换速度档位 + 自动 LED |
| **RB** | 5 | Cycle angular gear (1/4→1/2→3/4→1) — 切换转向灵敏度 |
| **Start** | 7 | Manual LED mode override — 手动切换 LED 模式 |
| **B** | 1 | Toggle buzzer — 开关蜂鸣器 |
| **A** | 0 | (unused / 未使用) |
| **X** | 2 | (unused / 未使用) |
| **Y** | 3 | (unused / 未使用) |

### Speed Gears & LED / 速度档位与 LED

| Gear / 档位 | Speed / 速度 | LED Effect / LED 效果 | Use / 用途 |
|---|---|---|---|
| — (inactive / 未激活) | 0 | **Solid red / 常亮红色** | Joy control OFF / 手柄控制关闭 |
| 1 (default) | 0.17 m/s | Breathing / 呼吸灯 | Mapping / 建图 |
| 2 | 0.33 m/s | Flowing / 流水灯 | Fast mapping / 快速建图 |
| 3 | 1.0 m/s (max) | Marquee / 跑马灯 | Transit only, NOT mapping / 仅赶路，不建图 |

Press **LB** to cycle gears. LED changes automatically to match the current gear. **Red LED = control inactive** — press Back to activate.

按 **LB** 切换档位，LED 自动随档位变化。**红灯常亮 = 控制未激活**，按 Back 激活。

Steering sensitivity defaults to lowest (1/4); press **RB** to cycle up (1/4→1/2→3/4→1).

转向灵敏度默认最低档（1/4），按 **RB** 可循环调高。

---

## ROS2 Nodes / ROS2 节点

### Launch bringup + SLAM / 启动底盘 + SLAM

```bash
# Inside container / 在容器内执行
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
export ROBOT_TYPE=r2
export RPLIDAR_TYPE=a1
ros2 launch yahboomcar_nav map_gmapping_launch.py
```

This starts: driver_node, base_node, IMU filter, EKF, joint_state_publisher, slam_gmapping, sllidar_node.

这会启动：驱动节点、底盘节点、IMU 滤波、EKF、关节状态发布、gmapping SLAM、雷达节点。

### Launch gamepad control / 启动手柄控制

```bash
# Start joy_node (reads gamepad device)
# 启动 joy_node（读取手柄设备）
ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=20.0 &

# Start patched joy_ctrl (converts joy → cmd_vel + LED)
# 启动修改版 joy_ctrl（转换手柄信号为速度指令 + LED）
ros2 run yahboomcar_ctrl yahboom_joy_R2 --ros-args \
  -p xspeed_limit:=0.5 -p angular_speed_limit:=5.0 &
```

### Key Topics / 关键话题

| Topic | Type | Description / 描述 |
|---|---|---|
| `/joy` | sensor_msgs/Joy | Raw gamepad data / 原始手柄数据 |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands / 速度指令 |
| `/RGBLight` | std_msgs/Int32 | LED effect index (0-6) / LED 效果编号 |
| `/JoyState` | std_msgs/Bool | Joy active state / 手柄激活状态 |
| `/scan` | sensor_msgs/LaserScan | LiDAR scan / 雷达扫描 |
| `/map` | nav_msgs/OccupancyGrid | SLAM map / SLAM 地图 |

---

## Usage / 使用方法

1. **Power on the robot / 开机** — container auto-starts, bringup + joy launches automatically.
   容器自动启动，底盘和手柄控制自动运行。

2. **Connect gamepad / 连接手柄** — Plug USB 2.4GHz dongle into Pi, then power on gamepad (hold center button).
   将 USB 2.4GHz 接收器插入树莓派 USB 口，然后长按手柄中间键开机。

3. **Press Back (6) / 按 Back 键** — activates joy control (toggle on/off).
   激活手柄控制（开/关切换）。

4. **Drive / 驾驶** — Left stick forward/back, Right stick left/right for steering.
   左摇杆前后 = 加减速，右摇杆左右 = 转向。

5. **Change gear / 换挡** — Press LB to cycle speed gears. LED reflects current gear.
   按 LB 切换速度档位，LED 自动变化。

---

## Patched Files / 修改的文件

The custom joy controller is at:
- **Source (Mac)**: `patches/yahboom_joy_R2_patched.py`
- **Deployed (Pi container)**: `/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_ctrl/lib/python3.8/site-packages/yahboomcar_ctrl/yahboom_joy_R2.py`

修改版手柄控制器位置：
- **源文件 (Mac)**：`patches/yahboom_joy_R2_patched.py`
- **部署位置 (Pi 容器内)**：`/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_ctrl/lib/python3.8/site-packages/yahboomcar_ctrl/yahboom_joy_R2.py`

Changes from original / 相对原版的修改:
- Safe `btn()`/`ax()` accessors (Flydigi has 11 buttons, not 15) — 安全的按键/摇杆访问（飞智有 11 个按键，非 15 个）
- Enabled `twist.angular.z` for Ackermann steering — 启用 angular.z 用于阿克曼转向
- Remapped buttons for Xbox 360 layout — 重映射为 Xbox 360 布局
- Added LED gear indicator via `/RGBLight` topic — 添加 LED 档位指示
- Solid red LED when inactive (custom `/RGBLight` index 7) — 未激活时红灯常亮（自定义编号 7）
- Absolute gear speeds: 0.17 / 0.33 / 1.0 m/s — 绝对档位速度
- Default steering sensitivity = lowest (1/4) — 默认最低转向灵敏度

The driver is also patched (`patches/Ackman_driver_R2_patched.py` on Pi at `/home/pi/rosmaster_tools/`):
- `/RGBLight` index 7 → stop current effect, then solid red via `set_colorful_lamps(0xFF, 255, 0, 0)`

驱动同样打了补丁：`/RGBLight` 编号 7 → 先停止当前特效，再设置纯红色。

---

## Known Issues / 已知问题

- **STM32 telemetry**: Returns Version=-1, Battery=0.0V, PID=[-1,-1,-1]. Motors work normally — this is a firmware limitation.
  STM32 返回 Version=-1, Battery=0.0V, PID=[-1,-1,-1]，但电机正常工作，属于固件限制。

- **IMU "free fall"**: Occasionally shows free fall warnings on container restart (serial init timing). Clears after a few seconds.
  容器重启时偶尔显示自由落体警告（串口初始化时序），几秒后消失。
