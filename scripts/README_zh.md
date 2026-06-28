# Yahboom ROSMaster R2 LiDAR SLAM 项目

基于 Yahboom ROSMaster R2 小车的 2D LiDAR SLAM 建图项目。使用 RPLidar A1 + gmapping 在 Raspberry Pi 5 上通过 Docker 运行 ROS Melodic，实现手柄遥控建图。

> For English documentation, see [README.md](README.md)

---

## 目录

- [项目概述](#项目概述)
- [硬件配置](#硬件配置)
- [软件架构](#软件架构)
- [数据流](#数据流)
- [已知问题与修复](#已知问题与修复)
- [参数配置](#参数配置)
- [使用指南](#使用指南)
- [ROS Topic 说明](#ros-topic-说明)
- [仓库结构](#仓库结构)
- [开发日志](#开发日志)

---

## 项目概述

本项目将 Yahboom ROSMaster R2（两轮 Ackermann 转向小车）改造为一台可通过 Xbox 360 无线手柄遥控的 SLAM 建图平台。核心工作流程：

1. 开机自启 → Docker 容器 `rosmaster_slam` 启动 ROS Melodic
2. 手柄控制小车运动，RPLidar A1 实时扫描
3. gmapping 接收 `/scan` + `/odom`（EKF 融合）进行在线建图
4. 地图每 60 秒自动保存切片，同时维护一张 overall 总图
5. Mac 端一键同步数据 + 轨迹可视化

**当前状态**: gmapping 建图正常运行，Scan Matching Score 稳定在 ~894，地图分辨率 3cm/pixel。

---

## 硬件配置

| 组件 | 型号/规格 | 备注 |
|------|----------|------|
| 小车底盘 | Yahboom ROSMaster R2 | 两轮 Ackermann 转向 |
| 主控 | Raspberry Pi 5, 8GB RAM | aarch64, Debian 12 (Bookworm) |
| LiDAR | RPLidar A1 | ~8Hz scan, 12m range, `/dev/rplidar` → `/dev/ttyUSB1` |
| 底板 | Rosmaster Board v3.3.9 | 电机控制 + IMU（加速度计+陀螺仪+磁力计） |
| 手柄 | Xbox 360 Wireless | xpad driver, `/dev/input/js1` |
| 网络 | WiFi | IP: `192.168.0.110` |

### 连接方式

```
Mac (SSH) ──── WiFi ──── Pi 5 (192.168.0.110)
                           ├── Docker: rosmaster_slam (ROS Melodic, --net=host)
                           ├── USB: RPLidar A1 (/dev/ttyUSB1)
                           ├── Serial: Rosmaster Board (电机 + IMU)
                           └── USB: Xbox 360 Wireless Receiver (/dev/input/js1)
```

SSH 连接：

```bash
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110
```

---

## 软件架构

### 环境变量

| 变量 | 值 | 用途 |
|------|---|------|
| `ROBOT_TYPE` | `X3` | launch 文件兼容性（选择正确的 launch 分支） |
| `YAHBOOM_BASE_TYPE` | `R2` | 驱动逻辑（Mcnamu_driver.py 中判断底盘类型） |

### Docker 容器

容器名 `rosmaster_slam`，基于 ROS Melodic，以 `--net=host` 模式运行：

```bash
# 进入容器
docker exec -it rosmaster_slam bash

# 容器内 ROS workspace
source /root/yahboomcar_ws/devel/setup.bash
```

### 核心节点

| 节点 | 文件 | 功能 |
|------|------|------|
| `Mcnamu_driver` | `Mcnamu_driver.py` | 底盘驱动：接收 `/cmd_vel`，控制电机，发布 `/vel_raw` |
| `base_node` | C++ (yahboomcar_bringup) | 将 `/vel_raw` 转换为 `/odom_raw`，处理 TF |
| `yahboom_joy` | `yahboom_joy.py` | 手柄输入 → `/cmd_vel`，含 deadzone 和速度限制 |
| `rplidarNode` | rplidar_ros | LiDAR 驱动，发布 `/scan` |
| `imu_filter_madgwick` | imu_filter_madgwick | IMU 滤波，`/imu/imu_raw` → `/imu/imu_data` |
| `ekf_localization` | robot_localization | EKF 融合 odom + IMU → `/odom` |
| `slam_gmapping` | gmapping | SLAM 建图，发布 `/map` |

---

## 数据流

从传感器到地图的完整数据通路：

```
┌─────────┐     /joy      ┌──────────────┐    /cmd_vel    ┌─────────────────┐
│ Gamepad │──────────────→│ yahboom_joy  │──────────────→│ Mcnamu_driver   │
│ (Xbox)  │               │   .py        │               │   .py           │
└─────────┘               └──────────────┘               └────────┬────────┘
                                                                  │
                                                    feedback_vx (来自 cmd_vel)
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

- `map → odom`: gmapping 提供
- `odom → base_footprint`: EKF (robot_localization) 提供
- `base_link → laser`: 静态 TF（launch 文件定义）

---

## 已知问题与修复

### 关键 Bug: R2 底盘编码器反馈全零

**问题**: `Rosmaster_Lib.get_motion_data()` 对 R2 底盘始终返回 `(0, 0, 0)`。库中定义了 `CARTYPE_X3`、`CARTYPE_X1` 等常量，但根本不存在 `CARTYPE_R2`，导致底层 firmware 无法正确返回轮子编码器数据。

**症状**:
- `/odom` 发布全零 → 机器人在 gmapping 看来永远静止不动
- gmapping 完全依赖 scan matching，产生放射状星爆图案
- Scan Matching Score 低至 ~20（正常应 > 200）

**修复方案** (`Mcnamu_driver.py`):

用 `/cmd_vel` 的命令速度作为 feedback 替代损坏的编码器反馈：

```python
# 修复前（原版）:
# vx, vy, angular = self.car.get_motion_data()  # R2 始终返回 (0,0,0)

# 修复后:
feedback_vx = self.last_cmd_linear_x  # 来自 /cmd_vel callback
feedback_angular = self.last_cmd_angular_z
```

**效果**:
- `/odom` 恢复正常数值
- Scan Matching Score 从 ~20 跳升至 ~894
- 地图从放射状噪声变为清晰的室内/室外轮廓

> **注意**: 这是开环估计，没有真实编码器闭环。长时间运行会累积漂移，但对 gmapping 的 scan matching 来说已足够好用。

### 其他修复

| 问题 | 修复 |
|------|------|
| 手柄低速抖动 | `yahboom_joy.py` 增加 deadzone 过滤 |
| 转向幅度过小 | 调整角速度映射比例 |
| rosbag 撑满磁盘 | 添加磁盘保护（>90% 自动删除最旧的 bag） |
| 手柄设备号漂移 | 启动脚本自动检测 `/dev/input/js*` |

---

## 参数配置

### gmapping 参数 (`gmapping.launch`)

| 参数 | 调优后 | 原始值 | 说明 |
|------|--------|--------|------|
| `delta` | **0.03** | 0.05 | 地图分辨率（3cm/pixel） |
| `minimumScore` | **20** | 50 | scan matching 最低接受分数 |
| `particles` | **50** | 30 | 粒子数，越多越稳定但越慢 |
| `temporalUpdate` | **2.0** | -1.0 (禁用) | 即使不动也每 2 秒更新一次，**关键参数** |
| `linearUpdate` | **0.05** | 0.1 | 移动 5cm 触发一次更新 |
| `angularUpdate` | **0.15** | 0.3 | 旋转 0.15rad (~8.6deg) 触发一次更新 |
| `iterations` | **5** | 1 | scan matching 迭代次数 |
| `maxUrange` | **10.0** | 8.0 | 最大可用激光距离（米） |
| `xmin/ymin` | -15.0 | -10.0 | 地图范围 30m x 30m |
| `xmax/ymax` | 15.0 | 10.0 | 地图范围 30m x 30m |

### 速度限制 (`yahboom_joy.py`)

| 参数 | 值 | 原始值 | 说明 |
|------|---|--------|------|
| 线速度上限 | **0.4 m/s** | 1.0 | 降低速度提高建图质量 |
| 角速度上限 | **2.0 rad/s** | 5.0 | 降低转弯速度减少 scan 畸变 |

### EKF 参数 (`robot_localization.yaml`)

融合两个数据源：
- `/odom_raw`: 提供 x, y 位置和 yaw 角
- `/imu/imu_data`: 提供角速度和线加速度

---

## 使用指南

### 开机自启

| 服务 | 功能 |
|------|------|
| `rosmaster-slam.service` | 启动 Docker + SLAM + 手柄控制 |
| `rosmaster-autosave.service` | 每 60 秒保存地图切片 |
| `rosmaster-rosbag.service` | 录制 rosbag（含磁盘保护） |

```bash
sudo systemctl status rosmaster-slam
sudo systemctl restart rosmaster-slam
sudo journalctl -u rosmaster-slam -f
```

### 手动启动（调试用）

```bash
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110
sudo systemctl stop rosmaster-slam rosmaster-autosave rosmaster-rosbag
bash /home/pi/rosmaster_tools/rosmaster_slam_start.sh
```

### 进入 Docker 容器调试

```bash
docker exec -it rosmaster_slam bash
source /root/yahboomcar_ws/devel/setup.bash
rostopic list
rostopic hz /scan
rostopic echo /odom -n 1
rostopic echo /slam_gmapping/entropy -n 1
```

### Mac 端操作

```bash
# 启动守护进程
bash ~/Claude/Projects/rosmaster/daemon.sh

# 同步所有数据
bash ~/Claude/Projects/rosmaster/sync_data.sh

# 生成轨迹叠加图
python3 ~/Claude/Projects/rosmaster/visualize_trajectory.py ~/Documents/rosmaster_r2/maps
```

### 地图输出

自动保存到 `/home/pi/rosmaster_maps/`：
- `overall_map.pgm/.yaml/.pcd` — 总图（持续覆盖更新）
- `slices/slam_map_YYYYMMDD_HHMMSS.*` — 每 60 秒一个时间戳切片

---

## ROS Topic 说明

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/scan` | `LaserScan` | ~8Hz | RPLidar A1 激光扫描 |
| `/imu/imu_raw` | `Imu` | 20Hz | 底板 IMU 原始数据 |
| `/imu/imu_data` | `Imu` | 20Hz | Madgwick 滤波后的 IMU |
| `/joy` | `Joy` | ~20Hz | 手柄原始输入 |
| `/cmd_vel` | `Twist` | 按需 | 速度指令 |
| `/vel_raw` | `Twist` | 20Hz | 速度反馈（基于 cmd_vel） |
| `/odom_raw` | `Odometry` | 20Hz | 原始里程计 |
| `/odom` | `Odometry` | 20Hz | EKF 融合里程计 |
| `/map` | `OccupancyGrid` | ~0.5Hz | SLAM 地图 |
| `/tf` | `TFMessage` | 持续 | `map→odom→base_footprint→base_link→laser` |

---

## 仓库结构

```
rosmaster_r2/
├── README.md                          # 英文说明
├── README_zh.md                       # 中文说明（本文件）
├── .gitignore
├── robot/                             # 小车端文件
│   ├── scripts/
│   │   ├── Mcnamu_driver.py           # 底盘驱动（已修补：cmd_vel 替代编码器）
│   │   ├── yahboom_joy.py             # 手柄控制（已调参）
│   │   ├── trajectory_recorder.py     # 轨迹记录
│   │   └── slam_visualizer.py         # 机器人屏幕实时 SLAM 可视化
│   ├── launch/
│   │   └── gmapping.launch            # gmapping 配置（已调优）
│   ├── config/
│   │   └── robot_localization.yaml    # EKF 融合参数
│   ├── host_scripts/                  # Pi 宿主机脚本
│   │   ├── rosmaster_slam_start.sh
│   │   ├── rosmaster_joy_start.sh
│   │   ├── rosmaster_slam_autosave.sh
│   │   └── rosmaster_rosbag_record.sh
│   └── systemd/
│       ├── rosmaster-slam.service
│       ├── rosmaster-autosave.service
│       └── rosmaster-rosbag.service
├── mac/                               # Mac 端工具
│   ├── daemon.sh
│   ├── sync_data.sh
│   ├── visualize_trajectory.py
│   └── pull_robot_files.sh
├── maps/                              # 地图数据（.pgm/.pcd 已 gitignore）
├── bags/                              # rosbag（全部 gitignore）
└── logs/                              # 日志文件
```

---

## 开发日志

| 阶段 | 工作内容 | 状态 |
|------|---------|------|
| 基础搭建 | Docker ROS Melodic 环境、RPLidar 驱动、Rosmaster Board 通信 | 完成 |
| 手柄控制 | Xbox 360 手柄驱动、yahboom_joy.py 调参、deadzone 过滤 | 完成 |
| SLAM 启动 | gmapping 基础配置、systemd 自启服务 | 完成 |
| 手柄修复 | 修复断连问题、低速抖动、转向幅度 | 完成 |
| 地图存储 | 整体图 + 60 秒切片、时间戳命名、防覆盖 | 完成 |
| **Odom 修复** | **发现 R2 编码器全零 bug → cmd_vel 替代方案 → Score 20→894** | **完成（关键转折点）** |
| gmapping 调优 | 分辨率 3cm、temporalUpdate 启用、粒子数 50、迭代 5 | 完成 |
| 磁盘保护 | rosbag 录制 + >90% 自动清理最旧文件 | 完成 |
| 数据通路 | Mac 端同步脚本、轨迹可视化、SSH 守护进程 | 完成 |
| 实时可视化 | 机器人屏幕显示实时 SLAM 地图 + 轨迹 | 完成 |

---

## 快速参考

```bash
# SSH 连接
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110

# 查看 SLAM 是否在运行
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 \
  "docker exec rosmaster_slam bash -c 'source /root/yahboomcar_ws/devel/setup.bash && rostopic hz /map'"

# 紧急停车
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 \
  "docker exec rosmaster_slam bash -c 'source /root/yahboomcar_ws/devel/setup.bash && rostopic pub /cmd_vel geometry_msgs/Twist \"{}\" -1'"

# 查看磁盘使用
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 "df -h /"
```

| 小车端路径 | 说明 |
|-----------|------|
| `/home/pi/rosmaster_tools/` | 宿主机启动脚本 |
| `/home/pi/rosmaster_maps/` | 地图输出目录 |
| `/home/pi/temp/bags/` | rosbag 录制目录 |
| `/root/yahboomcar_ws/` | ROS workspace（Docker 内） |
