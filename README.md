# ROSMaster R2 — ROS2 Foxy Setup

Yahboom ROSMaster R2 (Ackermann steering) running ROS2 Foxy on Raspberry Pi 5.

## Features

- **ROS2 Foxy** in Docker (`yahboomtechnology/ros-foxy:4.0.7R2`)
- **Gamepad control** — Flydigi Direwolf 3 (Xbox 360 mode) via USB dongle
- **Global racing override** — X enters racing from any state; X again returns directly to inactive
- **Natural reverse steering** — left/right steering is inverted automatically while reversing
- **LED gear indicator** — LED effect auto-changes with speed gear
- **Dual SLAM** — switch between gmapping and Cartographer with the Y button
- **Gear-driven mapping** — gears 1/2 map; gear 3, racing mode, and inactive mode stop LiDAR
- **Session recording** — map, trajectory, metadata, logs, and rosbag saved together
- **Live map kiosk** — full-screen map preview on the Raspberry Pi display
- **Auto-start on boot** — systemd service launches container + all nodes
- **Recovery guards** — incomplete bags are retained, low disk stops mapping safely, and LiDAR gets one automatic retry
- **USB hotplug recovery** — the gamepad node automatically returns after dongle reconnection

## Quick Start

1. Power on the robot (container + nodes start automatically)
2. Press **Back** on gamepad to activate normal control, or press **X** from any
   state to enter racing mode directly
3. Left stick = drive, Right stick = steer
4. **LB** = cycle speed gear; gears 1/2 enable mapping and gear 3 disables it
5. **Y** = switch the SLAM algorithm for the next mapping session
6. Press **Back** again, select gear 3, or enter racing mode to save and stop mapping
7. In racing mode, press **X** again to stop and return to the inactive state

Mapping sessions are stored in `/root/rosmaster_maps/YYYYmmdd_HHMMSS/` inside
the container. Each complete session contains `map.pgm`, `map.yaml`,
`trajectory.csv`, `metadata.txt`, node logs, and a ROS2 bag.
The bag includes raw and fused odometry/IMU, LiDAR, TF, velocity commands,
mapping state, SLAM selection, and diagnostics so sensor-fusion problems can
be diagnosed after a drive.

## Files

| File | Description |
|---|---|
| `Gamepad_and_Car_Setup.md` | Bilingual (EN/CN) setup guide with button mapping |
| `R2_Issues_and_Fixes.md` | ROS1 issues analysis and fixes |
| `ROS_问题总结与迁移指南.md` | ROS1→ROS2 migration guide |
| `patches/yahboom_joy_R2_patched.py` | Custom joy controller with LED gear indicator |
| `patches/Ackman_driver_R2_patched.py` | R2 driver patch with custom LED effects |
| `patches/slam_supervisor.py` | LiDAR, SLAM, rosbag, map-save, retry, and disk-safety lifecycle |
| `patches/ros2_bringup.sh` | Reliable container and ROS2 startup script |
| `patches/slam_gmapping.yaml` | Tuned gmapping configuration |
| `patches/rosmaster_carto.lua` | Tuned Cartographer configuration |
| `patches/ekf_r2.yaml` | R2 sensor-fusion configuration using wheel velocity plus IMU yaw/yaw-rate |
| `patches/map_kiosk.py` | Raspberry Pi live-map display |
| `systemd/rosmaster-ros2.service` | Versioned boot service |
| `daemon.sh` | Mac→Pi remote command daemon |
| `remote_commands.sh` | Current daemon command (overwritten per task) |

## Pi Setup

- **Container**: `rosmaster_ros2` with `--restart=always`
- **Systemd**: `rosmaster-ros2.service` (enabled, runs on boot)
- **Scripts**: `/home/pi/rosmaster_tools/ros2_bringup.sh`
- **Patch**: `/home/pi/rosmaster_tools/yahboom_joy_R2_patched.py` (auto-deployed on container start)
- **Timezone**: `Europe/Berlin` is exported into the container so session names match the Pi clock

The supervisor requires at least 2 GiB free space to start a mapping session
and stops an active session if free space falls below 1 GiB.

## Verified on Robot

Validated on Raspberry Pi 5 with RPLidar A1:

- ROS2 driver, EKF, IMU, TF, joystick, and supervisor startup
- gmapping map generation and graceful shutdown
- Cartographer map generation and graceful shutdown
- RPLidar scan rate around 7.6 Hz
- raw/fused odometry and IMU recording at around 10 Hz
- stationary fused odometry with zero position drift after rejecting absolute wheel-pose jumps
- Cartographer TF output reduced from about 200 Hz to about 65 Hz total
- rosbag, map, metadata, log, and trajectory output
- no duplicate static TF publishers after service restart

## Hardware

- Yahboom ROSMaster R2 (2-wheel Ackermann, wheelbase 0.25m)
- Raspberry Pi 5
- RPLidar A1
- Flydigi Direwolf 3 gamepad (USB 2.4GHz dongle)
