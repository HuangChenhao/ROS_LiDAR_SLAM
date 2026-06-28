# Yahboom ROSMaster R2 LiDAR SLAM

2D LiDAR SLAM mapping with a Yahboom ROSMaster R2 robot. Runs RPLidar A1 + gmapping on a Raspberry Pi 5 via Docker (ROS Melodic), controlled by an Xbox 360 wireless gamepad.

> 中文说明请见 [README_zh.md](README_zh.md)

---

## Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Software Architecture](#software-architecture)
- [Data Flow](#data-flow)
- [Known Issues & Fixes](#known-issues--fixes)
- [Parameter Tuning](#parameter-tuning)
- [Usage Guide](#usage-guide)
- [ROS Topics](#ros-topics)
- [Repository Structure](#repository-structure)
- [Development Log](#development-log)

---

## Overview

This project turns a Yahboom ROSMaster R2 (two-wheel Ackermann steering) into a gamepad-controlled SLAM mapping platform. Core workflow:

1. Boot → Docker container `rosmaster_slam` starts ROS Melodic automatically
2. Gamepad drives the robot; RPLidar A1 scans in real time
3. gmapping receives `/scan` + `/odom` (EKF-fused) and builds the map online
4. Map auto-saves a slice every 60 s; an overall map is maintained continuously
5. One command syncs all data to Mac + generates trajectory visualization

**Current status**: gmapping running normally, Scan Matching Score stable at ~894, map resolution 3 cm/pixel.

---

## Hardware

| Component | Spec | Notes |
|-----------|------|-------|
| Chassis | Yahboom ROSMaster R2 | Two-wheel Ackermann steering |
| SBC | Raspberry Pi 5, 8 GB RAM | aarch64, Debian 12 (Bookworm) |
| LiDAR | RPLidar A1 | ~8 Hz scan, 12 m range, `/dev/rplidar` → `/dev/ttyUSB1` |
| Base board | Rosmaster Board v3.3.9 | Motor control + IMU (accel + gyro + mag) |
| Gamepad | Xbox 360 Wireless | xpad driver, `/dev/input/js1` |
| Network | WiFi | IP: `192.168.0.110` |

### Connection Diagram

```
Mac (SSH) ──── WiFi ──── Pi 5 (192.168.0.110)
                           ├── Docker: rosmaster_slam (ROS Melodic, --net=host)
                           ├── USB: RPLidar A1 (/dev/ttyUSB1)
                           ├── Serial: Rosmaster Board (motors + IMU)
                           └── USB: Xbox 360 Wireless Receiver (/dev/input/js1)
```

SSH:

```bash
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110
```

---

## Software Architecture

### Environment Variables

| Variable | Value | Purpose |
|----------|-------|---------|
| `ROBOT_TYPE` | `X3` | Launch file compatibility (selects correct launch branch) |
| `YAHBOOM_BASE_TYPE` | `R2` | Driver logic (chassis type check in `Mcnamu_driver.py`) |

### Docker Container

Container `rosmaster_slam`, based on ROS Melodic, running with `--net=host`:

```bash
# Enter the container
docker exec -it rosmaster_slam bash

# Inside the container
source /root/yahboomcar_ws/devel/setup.bash
```

### Core Nodes

| Node | File | Function |
|------|------|----------|
| `Mcnamu_driver` | `Mcnamu_driver.py` | Chassis driver: receives `/cmd_vel`, controls motors, publishes `/vel_raw` |
| `base_node` | C++ (yahboomcar_bringup) | Converts `/vel_raw` → `/odom_raw`, handles TF |
| `yahboom_joy` | `yahboom_joy.py` | Gamepad → `/cmd_vel`, with deadzone and speed limits |
| `rplidarNode` | rplidar_ros | LiDAR driver, publishes `/scan` |
| `imu_filter_madgwick` | imu_filter_madgwick | IMU filter: `/imu/imu_raw` → `/imu/imu_data` |
| `ekf_localization` | robot_localization | EKF: fuses odom + IMU → `/odom` |
| `slam_gmapping` | gmapping | SLAM mapping, publishes `/map` |

---

## Data Flow

Complete sensor-to-map pipeline:

```
┌─────────┐     /joy      ┌──────────────┐    /cmd_vel    ┌─────────────────┐
│ Gamepad │──────────────→│ yahboom_joy  │──────────────→│ Mcnamu_driver   │
│ (Xbox)  │               │   .py        │               │   .py           │
└─────────┘               └──────────────┘               └────────┬────────┘
                                                                  │
                                                    feedback_vx (from cmd_vel)
                                                                  │
                                                          /vel_raw (Twist)
                                                                  │
                                                                  ▼
                                                         ┌────────────────┐
                                                         │  base_node     │
                                                         │  (C++)         │
                                                         └───────┬────────┘
                                                                 │
                                                          /odom_raw
                                                                 │
┌─────────┐    /imu/imu_raw   ┌─────────────┐  /imu/imu_data    │
│  IMU    │──────────────────→│  Madgwick   │────────────┐       │
│ (Board) │                   │  Filter     │            │       │
└─────────┘                   └─────────────┘            │       │
                                                         ▼       ▼
                                                    ┌────────────────┐
                                                    │  EKF           │
                                                    │ (robot_local.) │
                                                    └───────┬────────┘
                                                            │
                                                         /odom
                                                            │
┌─────────┐     /scan                                       │
│ RPLidar │──────────────────────────────────────┐          │
│   A1    │                                      │          │
└─────────┘                                      ▼          ▼
                                            ┌────────────────────┐
                                            │   gmapping         │
                                            │                    │
                                            └─────────┬──────────┘
                                                      │
                                                   /map
                                                      │
                                                      ▼
                                              ┌───────────────┐
                                              │  map_saver    │
                                              │  (autosave)   │
                                              └───────────────┘
```

### TF Tree

```
map → odom → base_footprint → base_link → laser
```

- `map → odom`: provided by gmapping
- `odom → base_footprint`: provided by EKF (robot_localization)
- `base_link → laser`: static TF (defined in launch file)

---

## Known Issues & Fixes

### Critical Bug: R2 Chassis Encoder Always Returns Zero

**Problem**: `Rosmaster_Lib.get_motion_data()` always returns `(0, 0, 0)` for the R2 chassis. The library defines `CARTYPE_X3`, `CARTYPE_X1`, etc., but `CARTYPE_R2` simply does not exist, so the firmware never sends back wheel encoder data.

**Symptoms**:
- `/odom` publishes all zeros → robot appears stationary to gmapping
- gmapping relies entirely on scan matching → produces starburst/radial noise patterns
- Scan Matching Score drops to ~20 (normal is >200)

**Fix** (`Mcnamu_driver.py`):

Replace the broken encoder feedback with the commanded velocity from `/cmd_vel`:

```python
# Before (original):
# vx, vy, angular = self.car.get_motion_data()  # always (0,0,0) on R2

# After (fix):
# Use cmd_vel command as velocity feedback
feedback_vx = self.last_cmd_linear_x      # from /cmd_vel callback
feedback_angular = self.last_cmd_angular_z
```

**Result**:
- `/odom` now publishes real values
- Scan Matching Score jumps from ~20 to ~894
- Map changes from radial noise to clean indoor/outdoor contours

> **Note**: This is open-loop estimation — no real encoder feedback. Drift accumulates over long runs, but it is accurate enough for gmapping's scan matching.

### Other Fixes

| Issue | Fix |
|-------|-----|
| Gamepad jitter at low speed | Added deadzone filter in `yahboom_joy.py` |
| Steering range too narrow | Adjusted angular velocity mapping ratio |
| rosbag fills disk | Added disk guard: auto-delete oldest bag when >90% full |
| Gamepad device number drifts | Startup script auto-detects `/dev/input/js*` |

---

## Parameter Tuning

### gmapping (`gmapping.launch`)

| Parameter | Tuned | Default | Notes |
|-----------|-------|---------|-------|
| `delta` | **0.03** | 0.05 | Map resolution (3 cm/pixel) |
| `minimumScore` | **20** | 50 | Minimum accepted scan match score |
| `particles` | **50** | 30 | More particles = more stable but slower |
| `temporalUpdate` | **2.0** | -1.0 (off) | Update every 2 s even when stationary — **critical** |
| `linearUpdate` | **0.05** | 0.1 | Update after 5 cm movement |
| `angularUpdate` | **0.15** | 0.3 | Update after 0.15 rad (~8.6°) rotation |
| `iterations` | **5** | 1 | Scan matching iterations |
| `maxUrange` | **10.0** | 8.0 | Maximum usable laser range (m) |
| `xmin/ymin` | -15.0 | -10.0 | Map extent 30 m × 30 m |
| `xmax/ymax` | 15.0 | 10.0 | Map extent 30 m × 30 m |

Map size: `30 m / 0.03 m = 1000 cells`, actual grid ~992×992.

### Speed Limits (`yahboom_joy.py`)

| Parameter | Value | Default | Notes |
|-----------|-------|---------|-------|
| Linear speed limit | **0.4 m/s** | 1.0 | Slower speed → better map quality |
| Angular speed limit | **2.0 rad/s** | 5.0 | Slower turns → less scan distortion |

### EKF (`robot_localization.yaml`)

Fuses two sources:
- `/odom_raw`: provides x, y position and yaw
- `/imu/imu_data`: provides angular velocity and linear acceleration

---

## Usage Guide

### Auto-start on Boot

These systemd services start automatically:

| Service | Function |
|---------|----------|
| `rosmaster-slam.service` | Starts Docker + SLAM + gamepad control |
| `rosmaster-autosave.service` | Saves map slice every 60 s |
| `rosmaster-rosbag.service` | Records rosbag (with disk guard) |

```bash
sudo systemctl status rosmaster-slam
sudo systemctl restart rosmaster-slam
sudo journalctl -u rosmaster-slam -f
```

### Manual Start (debugging)

```bash
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110
sudo systemctl stop rosmaster-slam rosmaster-autosave rosmaster-rosbag
bash /home/pi/rosmaster_tools/rosmaster_slam_start.sh
```

### Debug Inside Docker

```bash
docker exec -it rosmaster_slam bash
source /root/yahboomcar_ws/devel/setup.bash
rostopic list
rostopic hz /scan
rostopic echo /odom -n 1
rostopic echo /slam_gmapping/entropy -n 1
```

### Mac-side Tools

```bash
# Start SSH daemon (keep terminal open)
bash ~/Claude/Projects/rosmaster/daemon.sh

# Sync all data from robot
bash ~/Claude/Projects/rosmaster/sync_data.sh

# Generate trajectory overlay image
python3 ~/Claude/Projects/rosmaster/visualize_trajectory.py ~/Documents/rosmaster_r2/maps
```

Sync output goes to `~/Documents/rosmaster_r2/`: maps, bags, trajectory CSV, logs.

### Map Output

Auto-saved to `/home/pi/rosmaster_maps/`:
- `overall_map.pgm/.yaml/.pcd` — cumulative map (overwritten each time)
- `slices/slam_map_YYYYMMDD_HHMMSS.*` — timestamped slice every 60 s

---

## ROS Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/scan` | `LaserScan` | ~8 Hz | RPLidar A1 scan |
| `/imu/imu_raw` | `Imu` | 20 Hz | Raw IMU from base board |
| `/imu/imu_data` | `Imu` | 20 Hz | Madgwick-filtered IMU |
| `/joy` | `Joy` | ~20 Hz | Raw gamepad input |
| `/cmd_vel` | `Twist` | on demand | Velocity command |
| `/vel_raw` | `Twist` | 20 Hz | Velocity feedback (cmd_vel-based) |
| `/odom_raw` | `Odometry` | 20 Hz | Raw odometry |
| `/odom` | `Odometry` | 20 Hz | EKF-fused odometry |
| `/map` | `OccupancyGrid` | ~0.5 Hz | SLAM map |
| `/tf` | `TFMessage` | continuous | `map→odom→base_footprint→base_link→laser` |

---

## Repository Structure

```
rosmaster_r2/
├── README.md                          # This file (English)
├── README_zh.md                       # Chinese documentation
├── .gitignore
├── robot/                             # Robot-side files
│   ├── scripts/
│   │   ├── Mcnamu_driver.py           # Chassis driver (patched: cmd_vel replaces encoder)
│   │   ├── yahboom_joy.py             # Gamepad control (tuned: deadzone + speed limits)
│   │   ├── trajectory_recorder.py     # Trajectory recorder (/odom → CSV)
│   │   └── slam_visualizer.py         # Real-time SLAM visualization on robot screen
│   ├── launch/
│   │   └── gmapping.launch            # gmapping config (tuned)
│   ├── config/
│   │   └── robot_localization.yaml    # EKF fusion parameters
│   ├── host_scripts/                  # Pi host scripts (outside Docker)
│   │   ├── rosmaster_slam_start.sh
│   │   ├── rosmaster_joy_start.sh
│   │   ├── rosmaster_slam_autosave.sh
│   │   └── rosmaster_rosbag_record.sh
│   └── systemd/
│       ├── rosmaster-slam.service
│       ├── rosmaster-autosave.service
│       └── rosmaster-rosbag.service
├── mac/                               # Mac-side tools
│   ├── daemon.sh
│   ├── sync_data.sh
│   ├── visualize_trajectory.py
│   └── pull_robot_files.sh
├── maps/                              # Map data (.pgm/.pcd gitignored)
├── bags/                              # rosbag data (all gitignored)
└── logs/                              # Log files
```

---

## Development Log

| Phase | Work | Status |
|-------|------|--------|
| Foundation | Docker ROS Melodic, RPLidar driver, Rosmaster Board comms | Done |
| Gamepad | Xbox 360 driver, yahboom_joy.py tuning, deadzone filter | Done |
| SLAM startup | gmapping base config, systemd auto-start | Done |
| Gamepad fixes | Disconnect fix, low-speed jitter, steering range | Done |
| Map storage | Overall map + 60 s slices, timestamp naming | Done |
| **Odom fix** | **R2 encoder zero bug → cmd_vel workaround → Score 20→894** | **Done (key turning point)** |
| gmapping tuning | 3 cm resolution, temporalUpdate, 50 particles, 5 iterations | Done |
| Disk guard | rosbag + auto-delete when >90% full | Done |
| Data pipeline | Mac sync scripts, trajectory visualization, SSH daemon | Done |
| Live visualization | Real-time SLAM map + trajectory on robot screen | Done |

---

## Quick Reference

```bash
# SSH connect
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110

# Check if SLAM is running
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 \
  "docker exec rosmaster_slam bash -c 'source /root/yahboomcar_ws/devel/setup.bash && rostopic hz /map'"

# Emergency stop
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 \
  "docker exec rosmaster_slam bash -c 'source /root/yahboomcar_ws/devel/setup.bash && rostopic pub /cmd_vel geometry_msgs/Twist \"{}\" -1'"

# Disk usage
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 "df -h /"
```

| Path on Robot | Description |
|---------------|-------------|
| `/home/pi/rosmaster_tools/` | Host startup scripts |
| `/home/pi/rosmaster_maps/` | Map output |
| `/home/pi/temp/bags/` | rosbag recording |
| `/root/yahboomcar_ws/` | ROS workspace (inside Docker) |
