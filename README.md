# Yahboom ROSMaster R2 LiDAR SLAM 项目

基于 Yahboom ROSMaster R2 小车的 2D LiDAR SLAM 建图项目。使用 RPLidar A1 + gmapping 在 Raspberry Pi 5 上通过 Docker 运行 ROS Melodic，实现手柄遥控建图。

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

这是本项目最核心的架构图，展示从传感器到地图的完整数据通路：

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
# 使用 cmd_vel 命令值作为 velocity feedback
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
| `minimumScore` | **20** | 50 | scan matching 最低接受分数，降低后容忍较差匹配 |
| `particles` | **50** | 30 | 粒子数，越多越稳定但越慢 |
| `temporalUpdate` | **2.0** | -1.0 (禁用) | 即使不动也每 2 秒更新一次，**关键参数** |
| `linearUpdate` | **0.05** | 0.1 | 移动 5cm 触发一次更新（原 10cm） |
| `angularUpdate` | **0.15** | 0.3 | 旋转 0.15rad (~8.6deg) 触发一次更新 |
| `iterations` | **5** | 1 | scan matching 迭代次数，提高精度 |
| `maxUrange` | **10.0** | 8.0 | 最大可用激光距离（米） |
| `xmin/ymin` | -15.0 | -10.0 | 地图范围 30m x 30m |
| `xmax/ymax` | 15.0 | 10.0 | 地图范围 30m x 30m |

地图尺寸：`30m / 0.03m = 1000 cells`，实际 grid 约 992x992。

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

小车开机后，以下 systemd 服务自动启动：

| 服务 | 功能 |
|------|------|
| `rosmaster-slam.service` | 启动 Docker + SLAM + 手柄控制 |
| `rosmaster-autosave.service` | 每 60 秒保存地图切片 |
| `rosmaster-rosbag.service` | 录制 rosbag（含磁盘保护） |

```bash
# 查看服务状态
sudo systemctl status rosmaster-slam
sudo systemctl status rosmaster-autosave
sudo systemctl status rosmaster-rosbag

# 手动重启 SLAM
sudo systemctl restart rosmaster-slam

# 查看实时日志
sudo journalctl -u rosmaster-slam -f
```

### 手动启动（调试用）

```bash
# SSH 到小车
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110

# 停掉自启服务
sudo systemctl stop rosmaster-slam rosmaster-autosave rosmaster-rosbag

# 手动启动 SLAM
bash /home/pi/rosmaster_tools/rosmaster_slam_start.sh

# 在另一个终端启动手柄（如果需要单独启动）
bash /home/pi/rosmaster_tools/rosmaster_joy_start.sh

# 手动启动自动保存
bash /home/pi/rosmaster_tools/rosmaster_slam_autosave.sh

# 手动启动 rosbag 录制
bash /home/pi/rosmaster_tools/rosmaster_rosbag_record.sh
```

### 进入 Docker 容器调试

```bash
# 进入正在运行的容器
docker exec -it rosmaster_slam bash

# 容器内查看 ROS topic
source /root/yahboomcar_ws/devel/setup.bash
rostopic list
rostopic hz /scan
rostopic echo /odom -n 1

# 检查 gmapping 状态
rostopic echo /slam_gmapping/entropy -n 1

# 查看 TF tree
rosrun tf tf_echo map base_link

# 手动保存地图
rosrun map_server map_saver -f /root/yahboomcar_ws/src/yahboomcar_nav/maps/my_map
```

### Mac 端操作

#### 远程命令守护进程

```bash
# 启动守护进程（保持终端打开）
bash ~/Claude/Projects/rosmaster/daemon.sh

# 守护进程会监听 remote_commands.sh 的变化并通过 SSH 执行
```

#### 数据同步

```bash
# 一键从小车同步所有数据（地图、bag、轨迹、日志）
bash ~/Claude/Projects/rosmaster/sync_data.sh
```

同步内容：
- 地图文件（overall + slices）→ `~/Documents/rosmaster_r2/maps/`
- rosbag → `~/Documents/rosmaster_r2/bags/`
- 轨迹 CSV → `~/Documents/rosmaster_r2/maps/trajectory.csv`
- 日志 → `~/Documents/rosmaster_r2/logs/`
- 自动生成轨迹可视化叠加图

#### 轨迹可视化

```bash
# 生成轨迹叠加在地图上的可视化图
python3 ~/Claude/Projects/rosmaster/visualize_trajectory.py ~/Documents/rosmaster_r2/maps
# 输出: ~/Documents/rosmaster_r2/maps/map_with_trajectory.png
```

#### 拉取机器人配置文件

```bash
# 将小车上的脚本和配置同步到本地仓库
bash ~/Claude/Projects/rosmaster/pull_robot_files.sh
```

### 停止 SLAM

```bash
# 方法 1: 停止 systemd 服务
sudo systemctl stop rosmaster-slam rosmaster-autosave rosmaster-rosbag

# 方法 2: 停止 Docker 容器
docker stop rosmaster_slam
```

### 保存最终地图

地图自动保存到 `/home/pi/rosmaster_maps/`：
- `overall_map.pgm` / `.yaml` / `.pcd` — 总图（持续覆盖更新）
- `slices/slam_map_YYYYMMDD_HHMMSS.*` — 每 60 秒一个时间戳切片（带防覆盖）

---

## ROS Topic 说明

### 传感器数据

| Topic | 类型 | 频率 | 发布者 | 说明 |
|-------|------|------|--------|------|
| `/scan` | `sensor_msgs/LaserScan` | ~8Hz | rplidarNode | RPLidar A1 激光扫描数据 |
| `/imu/imu_raw` | `sensor_msgs/Imu` | 20Hz | Mcnamu_driver | 底板 IMU 原始数据 |
| `/imu/imu_data` | `sensor_msgs/Imu` | 20Hz | imu_filter_madgwick | Madgwick 滤波后的 IMU |

### 控制指令

| Topic | 类型 | 频率 | 发布者 | 说明 |
|-------|------|------|--------|------|
| `/joy` | `sensor_msgs/Joy` | ~20Hz | joy_node | 手柄原始按钮和摇杆 |
| `/cmd_vel` | `geometry_msgs/Twist` | 按需 | yahboom_joy | 速度指令（线速度 + 角速度） |

### 里程计

| Topic | 类型 | 频率 | 发布者 | 说明 |
|-------|------|------|--------|------|
| `/vel_raw` | `geometry_msgs/Twist` | 20Hz | Mcnamu_driver | 速度反馈（基于 cmd_vel） |
| `/odom_raw` | `nav_msgs/Odometry` | 20Hz | base_node | 原始里程计（积分 vel_raw） |
| `/odom` | `nav_msgs/Odometry` | 20Hz | ekf_localization | EKF 融合里程计（odom_raw + IMU） |

### 建图

| Topic | 类型 | 频率 | 发布者 | 说明 |
|-------|------|------|--------|------|
| `/map` | `nav_msgs/OccupancyGrid` | ~0.5Hz | slam_gmapping | 占据栅格地图 |
| `/tf` | `tf2_msgs/TFMessage` | 持续 | 多节点 | 坐标变换：`map→odom→base_footprint→base_link→laser` |

---

## 仓库结构

```
rosmaster_r2/
├── README.md                          # 本文件
├── .gitignore
├── robot/                             # 小车端文件
│   ├── scripts/                       # Python 脚本
│   │   ├── Mcnamu_driver.py           # 底盘驱动（已修补：cmd_vel 替代编码器）
│   │   ├── yahboom_joy.py             # 手柄控制（已调参：deadzone + 速度限制）
│   │   ├── trajectory_recorder.py     # 轨迹记录（订阅 /odom 写 CSV）
│   │   └── slam_visualizer.py         # 机器人屏幕实时 SLAM 可视化
│   ├── launch/                        # ROS launch 文件
│   │   └── gmapping.launch            # gmapping 配置（已调优）
│   ├── config/                        # 配置文件
│   │   └── robot_localization.yaml    # EKF 融合参数
│   ├── host_scripts/                  # Pi 宿主机脚本（Docker 外）
│   │   ├── rosmaster_slam_start.sh    # 主 SLAM 启动脚本
│   │   ├── rosmaster_joy_start.sh     # 手柄启动（自动检测 js 设备）
│   │   ├── rosmaster_slam_autosave.sh # 60 秒自动保存（带防覆盖）
│   │   └── rosmaster_rosbag_record.sh # rosbag 录制（磁盘 >90% 自动清理）
│   └── systemd/                       # systemd 服务文件
│       ├── rosmaster-slam.service     # 开机自启 SLAM
│       ├── rosmaster-autosave.service # 开机自启地图保存
│       └── rosmaster-rosbag.service   # 开机自启 rosbag 录制
├── mac/                               # Mac 端工具
│   ├── daemon.sh                      # SSH 命令守护进程
│   ├── sync_data.sh                   # 一键数据同步
│   ├── visualize_trajectory.py        # 轨迹叠加可视化
│   └── pull_robot_files.sh            # 从小车拉取配置到本地
├── maps/                              # 地图数据
│   ├── overall_map.yaml               # 总图元数据（tracked）
│   ├── overall_map.pgm                # 总图栅格（gitignored, 大文件）
│   ├── overall_map.pcd                # 总图点云（gitignored）
│   └── slices/                        # 时间切片（yaml tracked, pgm/pcd ignored）
│       └── slam_map_YYYYMMDD_HHMMSS.*
├── bags/                              # rosbag 数据（全部 gitignored）
├── logs/                              # 日志文件
└── docs/                              # 额外文档
    └── parameters.md                  # 参数详解
```

### .gitignore 策略

- `.pgm` / `.pcd` / `.bag` — 二进制大文件，不入库
- `.yaml`（地图元数据）— 入库，体积小且含重要参数
- `logs/` — 不入库

---

## 开发日志

按时间顺序记录关键里程碑：

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
| 仓库整理 | 目录结构、文件归档、README 文档 | 进行中 |
| Git 发布 | 初始化 Git、推送 GitHub | 待完成 |

---

## 快速参考

### 常用 SSH 命令

```bash
# SSH 连接
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110

# 查看 SLAM 是否在运行
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 \
  "docker exec rosmaster_slam bash -c 'source /root/yahboomcar_ws/devel/setup.bash && rostopic hz /map'"

# 查看当前 Scan Matching Score
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 \
  "docker exec rosmaster_slam bash -c 'source /root/yahboomcar_ws/devel/setup.bash && rostopic echo /slam_gmapping/entropy -n 1'"

# 紧急停车
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 \
  "docker exec rosmaster_slam bash -c 'source /root/yahboomcar_ws/devel/setup.bash && rostopic pub /cmd_vel geometry_msgs/Twist \"{}\" -1'"

# 查看磁盘使用
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110 "df -h /"
```

### 小车端关键路径

| 路径 | 说明 |
|------|------|
| `/home/pi/rosmaster_tools/` | 宿主机启动脚本 |
| `/home/pi/rosmaster_maps/` | 地图输出目录 |
| `/home/pi/temp/bags/` | rosbag 录制目录 |
| `/dev/rplidar` → `/dev/ttyUSB1` | LiDAR 设备 |
| `/dev/input/js1` | 手柄设备 |

### Docker 内关键路径

| 路径 | 说明 |
|------|------|
| `/root/yahboomcar_ws/` | ROS workspace |
| `/root/yahboomcar_ws/src/yahboomcar_bringup/scripts/` | 驱动脚本 |
| `/root/yahboomcar_ws/src/yahboomcar_nav/launch/library/` | launch 文件 |
| `/root/yahboomcar_ws/src/yahboomcar_bringup/param/` | 参数文件 |
| `/root/yahboomcar_ws/src/yahboomcar_nav/maps/` | 地图保存位置 |
