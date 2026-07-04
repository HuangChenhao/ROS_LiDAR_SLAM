# ROSMaster R2 — ROS2 Foxy Setup

Yahboom ROSMaster R2 (Ackermann steering) running ROS2 Foxy on Raspberry Pi 5.

## Features

- **ROS2 Foxy** in Docker (`yahboomtechnology/ros-foxy:4.0.7R2`)
- **Gamepad control** — Flydigi Direwolf 3 (Xbox 360 mode) via USB dongle
- **LED gear indicator** — LED effect auto-changes with speed gear
- **gmapping SLAM** — real-time mapping with RPLidar A1
- **Auto-start on boot** — systemd service launches container + all nodes

## Quick Start

1. Power on the robot (container + nodes start automatically)
2. Press **Back** on gamepad to activate control
3. Left stick = drive, Right stick = steer
4. **LB** = cycle speed gear (LED changes automatically)

## Files

| File | Description |
|---|---|
| `Gamepad_and_Car_Setup.md` | Bilingual (EN/CN) setup guide with button mapping |
| `R2_Issues_and_Fixes.md` | ROS1 issues analysis and fixes |
| `ROS_问题总结与迁移指南.md` | ROS1→ROS2 migration guide |
| `patches/yahboom_joy_R2_patched.py` | Custom joy controller with LED gear indicator |
| `daemon.sh` | Mac→Pi remote command daemon |
| `remote_commands.sh` | Current daemon command (overwritten per task) |

## Pi Setup

- **Container**: `rosmaster_ros2` with `--restart=always`
- **Systemd**: `rosmaster-ros2.service` (enabled, runs on boot)
- **Scripts**: `/home/pi/rosmaster_tools/ros2_bringup.sh`
- **Patch**: `/home/pi/rosmaster_tools/yahboom_joy_R2_patched.py` (auto-deployed on container start)

## Hardware

- Yahboom ROSMaster R2 (2-wheel Ackermann, wheelbase 0.25m)
- Raspberry Pi 5
- RPLidar A1
- Flydigi Direwolf 3 gamepad (USB 2.4GHz dongle)
