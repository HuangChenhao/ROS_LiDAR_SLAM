# ROSMaster R2 — ROS2 Foxy Setup

Yahboom ROSMaster R2 (Ackermann steering) running ROS2 Foxy on Raspberry Pi 5.

<img width="970" height="600" alt="image" src="https://github.com/user-attachments/assets/3168bf38-5a0a-41bb-ac9f-d82c93a987c3" />


## Features

- **ROS2 Foxy** in Docker (`yahboomtechnology/ros-foxy:4.0.7R2`)
- **Gamepad control** — Flydigi Direwolf 3 (Xbox 360 mode) via USB dongle
- **Global racing override** — X enters racing from any state; X again returns directly to inactive
- **Natural reverse steering** — left/right steering is inverted automatically while reversing
- **LED gear indicator** — LED effect auto-changes with speed gear
- **Four-way SLAM** — Y cycles GMapping, Cartographer, SLAM Toolbox, and RTAB-Map 2D LiDAR
- **Loop-closure finalization** — graph-SLAM backends optimize and republish the corrected map before the final files are saved
- **Algorithm LED reminder** — while mapping, the active algorithm color appears for 1.5 seconds every 10 seconds
- **Gear-driven mapping** — gears 1/2 map; gear 3, racing mode, and inactive mode stop LiDAR
- **Session recording** — timestamp, algorithm, exact speed-gear events, map, optimized graph/path, logs, and rosbag saved together
- **Live map kiosk** — full map is continuously fit and centered on the Raspberry Pi display
- **Auto-start on boot** — systemd service launches container + all nodes
- **Recovery guards** — serial ownership is enforced, sensor health gates SLAM, incomplete bags are retained, low disk stops mapping safely, and LiDAR gets one automatic retry
- **USB hotplug recovery** — the gamepad node automatically returns after dongle reconnection

## Quick Start

1. Power on the robot (container + nodes start automatically)
2. Press **Back** on gamepad to activate normal control, or press **X** from any
   state to enter racing mode directly
3. Left stick = drive, Right stick = steer
4. **LB** = cycle speed gear; gears 1/2 enable mapping and gear 3 disables it
5. **Y** = select the SLAM algorithm for the next mapping session:
   blue=GMapping, amber=Cartographer, green=SLAM Toolbox, magenta=RTAB-Map
6. Press **Back** again, select gear 3, or enter racing mode to save and stop mapping
7. In racing mode, press **X** again to stop and return to the inactive state

Mapping sessions are stored in
`/root/rosmaster_maps/YYYYmmdd_HHMMSS_algorithm_gearN/` inside
the container. Each complete session contains `map.pgm`, `map.yaml`,
`trajectory.csv`, optional `trajectory_optimized.csv`/`pose_graph_edges.csv`,
`gear_events.csv`, `metadata.txt`, node logs, and a ROS2 bag.
The bag includes raw and fused odometry/IMU, measured and commanded velocity,
LiDAR, TF, chassis voltage/firmware, mapping state, SLAM selection, speed gear,
and optimized graph/path topics, plus
diagnostics so sensor-fusion and serial failures can be diagnosed after a drive.
The supervisor waits 0.5 seconds after a mapping-start request so the independent
algorithm, gear, and mapping-state topics are merged before a backend is chosen;
this prevents a folder label from disagreeing with the process that was started.

## SLAM Algorithms

| Y color | Algorithm | Sensor path | Notes |
|---|---|---|---|
| Blue | GMapping | LiDAR + EKF odometry | Particle-filter baseline |
| Amber | Cartographer | LiDAR + EKF odometry | Submaps and pose-graph optimization |
| Green | SLAM Toolbox | LiDAR + EKF odometry | ROS2-native asynchronous pose graph with robust loss |
| Magenta | RTAB-Map 2D LiDAR | LiDAR ICP + EKF odometry | Graph SLAM; also saves `rtabmap.db` |

The on-screen status bar shows the running algorithm and exact speed gear, and
large maps are scaled down so the complete map remains centered. If Y is pressed during
mapping, the current session continues safely and the status bar also shows
the algorithm selected for the next session.

Cartographer, SLAM Toolbox, and RTAB-Map use explicit pose-graph loop closure
and receive a final optimization window before the map is written. GMapping
uses particle-filter scan-to-map correlation and cannot globally rewrite old
poses, so it is best kept as the fast baseline rather than the preferred
algorithm for long closed-loop routes.

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
| `patches/slam_toolbox_r2.yaml` | R2-tuned asynchronous SLAM Toolbox configuration |
| `patches/rtabmap_r2.yaml` | R2-tuned RTAB-Map 2D LiDAR/ICP configuration |
| `patches/ekf_r2.yaml` | R2 sensor fusion using wheel velocity/steering plus IMU yaw rate |
| `patches/map_kiosk.py` | Raspberry Pi live-map display |
| `patches/rosmaster_odom.py` | Stable Ackermann odometry with first-sample and callback-gap rejection |
| `patches/robot_healthcheck.py` | Startup validation of serial, IMU, raw odometry, and EKF output |
| `patches/yahboomcar_bringup_R2_launch.py` | Respawning driver stack without duplicate joint-state publishers |
| `patches/cartographer_launch.py` | Cartographer launch with explicit LiDAR, odometry, and IMU remaps |
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
and stops an active session if free space falls below 1 GiB. It also stops and
labels a session if the selected SLAM process exits or if `/map` stops updating
for 10 seconds while the vehicle is moving, avoiding a silent dead session.

## Verified on Robot

Validated on Raspberry Pi 5 with RPLidar A1:

- ROS2 driver, EKF, IMU, TF, joystick, and supervisor startup
- gmapping map generation and graceful shutdown
- Cartographer map generation and graceful shutdown
- SLAM Toolbox map generation and graceful shutdown
- RTAB-Map 2D LiDAR map/database generation and graceful shutdown
- exact gear-labelled sessions and race-free algorithm selection
- Cartographer optimized pose-graph trajectory export and display
- four-color Y selection and algorithm-labeled session folders
- live display status including the running algorithm
- RPLidar scan rate around 7.6 Hz
- raw/fused odometry and IMU recording at around 10 Hz
- stationary fused odometry with zero position drift after rejecting absolute wheel-pose jumps
- chassis serial, IMU gravity norm, firmware version, and odometry are checked before startup succeeds
- legacy desktop control is disabled so it cannot steal `/dev/myserial` from ROS2
- Cartographer TF output reduced from about 200 Hz to about 65 Hz total
- rosbag, map, metadata, log, and trajectory output
- no duplicate static TF publishers after service restart

## Hardware

- Yahboom ROSMaster R2 (2-wheel Ackermann, wheelbase 0.25m)
- Raspberry Pi 5
- RPLidar A1
- Flydigi Direwolf 3 gamepad (USB 2.4GHz dongle)

## Experiment
Mapping:
<img width="743" height="1045" alt="image" src="https://github.com/user-attachments/assets/765aeb9e-ba3c-4a5d-96d2-70c6d40a78bb" />

Point-Cloud
<img width="1350" height="1224" alt="a6143283d74050aa1ac93841c3c4af23" src="https://github.com/user-attachments/assets/aeac49d2-a142-4b11-a576-b5e5772a3d2a" />

RGB-D Camera
new V-SLAM system under development
