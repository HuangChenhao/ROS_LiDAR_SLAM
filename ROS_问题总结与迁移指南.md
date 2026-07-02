# ROSMaster R2 — ROS 问题总结与 ROS2 迁移指南

> 最后更新: 2026-07-02  
> 硬件: Yahboom ROSMaster R2 (Ackermann 2轮转向), Raspberry Pi 5, RPLidar A1  
> 当前环境: ROS1 Melodic (Docker: `yahboomtechnology/ros-melodic:4.0.1`)

---

## 一、当前 ROS1 环境遇到的问题

### 1. gmapping 僵尸节点 (反复发生)

**现象**: gmapping 进程存在但完全断开 ROS master，`rosnode info /slam_gmapping` 显示 Publications: None, Subscriptions: None。地图停止更新但进程不退出。

**原因**: ROS1 的 rosmaster 是中心化架构，节点注册信息保存在 rosmaster 进程内存中。当 rosmaster 重启而 gmapping 没有重启时，gmapping 变成僵尸 — 进程在跑，但 rosmaster 不认识它了。

**修复**: `pkill -9 -f slam_gmapping` 然后重新启动 gmapping。

**影响**: 已导致至少两次建图会话数据丢失。好在 rosbag 持续录制，可以用 bag 离线重建地图。

**根本原因**: ROS1 架构固有缺陷。ROS2 使用 DDS 去中心化通信，不存在此问题。

### 2. ROBOT_TYPE 配置错误 (X3 vs R2)

**现象**: 手柄行为异常，驱动使用了 X3 (麦克纳姆轮) 模式而非 R2 (Ackermann 转向)。

**原因**: ROS1 的 Yahboom 官方 launch 文件只对 X3 有条件分支:
```xml
<!-- bringup.launch -->
<node pkg="yahboomcar_bringup" type="Mcnamu_driver.py" name="driver_node"
      if="$(eval arg('robot_type') == 'X3')">
```
如果设 `ROBOT_TYPE=R2`，驱动节点不会启动，激光 TF 也不会发布。所以被迫使用 `ROBOT_TYPE=X3`，再通过 `YAHBOOM_BASE_TYPE=R2` 环境变量让驱动内部切换到 R2 模式。

**我们的补丁**:
- `Mcnamu_driver.py`: 添加了 `YAHBOOM_BASE_TYPE` 检测，R2 时使用 `set_car_type(CARTYPE_R2)` 和 Ackermann 转向逻辑
- `yahboom_joy.py`: 添加了速度限制参数、竞速模式切换

**问题**: 这是一个 hack，每次容器重建都需要重新部署补丁脚本，容易遗漏。

### 3. autosave 脚本循环调用 bug

**现象**: autosave 启动后杀掉了 joy 节点，系统行为异常。

**原因**: `rosmaster_slam_autosave.sh` 开头调用了 `rosmaster_slam_start.sh`（完整启动脚本），启动脚本内部又杀掉现有 autosave 并启动新的，形成循环。启动脚本还会杀掉并重启 joy 节点。

**修复**: 从 autosave 脚本中删除了对 start script 的调用。

### 4. 手柄设备检测 (Docker bind mount)

**现象**: 开机后手柄无法使用，需要手动重启容器。

**原因**: Docker `--device=/dev/input/js0` 只在容器创建时快照设备。USB 手柄接收器开机后才被识别，创建容器时设备不存在。

**修复**: 改用 bind mount + cgroup 规则:
```bash
-v /dev/input:/dev/input
--device-cgroup-rule='c 13:* rmw'
```
这样容器内能实时看到新出现的输入设备。还需要确保 `joydev` 内核模块已加载 (`modprobe joydev`)。

### 5. 地图质量问题

**问题 1 — 放射状伪影 (Starburst)**: 户外远距离回波 >10m 被当作障碍物画入地图。  
**修复**: `maxUrange=6.0` (限制有效激光距离)

**问题 2 — 转弯处地图撕裂**: cmd_vel 反馈非真实编码器数据，急转弯时里程计漂移。  
**修复**: `minimumScore=20` + 驾驶速度 ≤0.3 m/s

**问题 3 — 无回环检测**: gmapping 不做 loop closure，长距离累积漂移。  
**修复**: 只能通过离线后处理 (postprocess_slam.py) 或换用 cartographer。

### 6. 其他问题

| 问题 | 修复 |
|------|------|
| map_saver 无限挂起 | gmapping 僵尸导致 /map topic 不存在，先修复 gmapping |
| autosave 不保存 | 启动脚本提前退出未执行到 autosave 部分，手动补启 |
| rosbag 未录制 | 同上，手动补启 |
| slices 无限增长 | 添加 `prune_slices()` 函数，配合 `SLICE_KEEP=30` |
| `_tmp_save_*` 残留文件 | 添加 `cleanup_tmp()` 函数 |
| zsh 命令问题 | `#` 注释被当作命令，`*` 通配符需要引号包裹 |

---

## 二、当前 ROS1 配置体系

### 文件结构 (Pi 上)

```
/home/pi/rosmaster_tools/
├── rosmaster_slam_start.sh      # 主启动脚本 (246行)
├── slam_config.sh               # 用户可编辑配置文件
├── rosmaster_slam_autosave.sh   # 自动保存脚本
├── rosmaster_map_save.sh        # 单次保存脚本
├── rosmaster_kiosk.sh           # 桌面显示脚本
└── custom_scripts/
    ├── yahboom_joy.py            # R2 定制手柄 (含竞速模式)
    └── map_web_viewer.py         # 地图 web 可视化

/home/pi/rosmaster_maps/
├── overall_map.pgm/yaml/pcd     # 当前整体地图
├── slices/                      # 每60s快照 (最多30个)
├── archive/                     # 历史会话归档 (最多5个)
└── autosave.log

/home/pi/temp/bags/              # rosbag 录制目录
```

### slam_config.sh 关键参数

```bash
# gmapping
SLAM_DELTA=0.05          # 分辨率 (m/pixel)
SLAM_MIN_SCORE=20        # scan match 最低分
SLAM_PARTICLES=50        # 粒子数
SLAM_MAX_URANGE=6.0      # 最大可用激光距离
SLAM_MAX_RANGE=8.0       # 最大测量距离

# 速度限制
JOY_MAX_LINEAR=0.3       # 制图模式 (m/s)
JOY_MAX_ANGULAR=2.0      # 制图模式 (rad/s)
JOY_RACING_LINEAR=1.0    # 竞速模式
JOY_RACING_ANGULAR=4.0   # 竞速模式

# 归档
ARCHIVE_KEEP=5
SLICE_KEEP=30
```

### 启动流程

```
rosmaster_slam_start.sh
├── 加载 slam_config.sh
├── 归档上次会话数据
├── 创建/启动 Docker 容器 (含 bind mount 修复)
├── 部署定制脚本到容器
├── roscore
├── laser_bringup.launch (rplidar + bringup + IMU + EKF)
├── gmapping (全参数从 config 读取)
├── 手柄 (30s 自动等待 + 速度限制)
├── 轨迹记录
├── rosbag 录制 (split=64MB)
├── autosave (每60s + 自动裁剪)
└── 地图 web viewer
```

---

## 三、ROS2 官方支持情况

### Yahboom 官方 ROS2 资源

| 项目 | 详情 |
|------|------|
| Docker 镜像 | `yahboomtechnology/ros-foxy` (~14.2 GB) |
| ROS2 版本 | Foxy (容器内), PC 端推荐 Humble |
| R2 专用驱动 | `Ackman_driver_R2` (非 Mcnamu 打补丁) |
| R2 专用手柄 | `yahboom_joy_R2` |
| SLAM 算法 | gmapping + **cartographer** (有 loop closure) |
| 导航 | Nav2 (DWA / TEB 路径规划) |
| 教程 | 完整 17-29 章，涵盖遥控、硬件、SLAM、导航、视觉、多机 |
| 系统镜像 | Google Drive 提供 Pi 5 ROS2 系统镜像下载 |

### ROS2 关键命令

```bash
# 进入 Docker 容器
~/run_docker.sh

# 设置机器人类型 (一次性)
sh ~/Rosmaster/RobotType/set_R2_A1.sh

# gmapping 建图
ros2 launch yahboomcar_nav map_gmapping_launch.py

# cartographer 建图 (推荐，有 loop closure)
ros2 launch yahboomcar_nav map_cartographer_launch.py

# 保存地图
ros2 launch yahboomcar_nav save_map_launch.py

# 导航
ros2 launch yahboomcar_nav navigation_dwa_launch.py
```

### ROS2 工作空间结构 (容器内)

```
~/yahboomcar_ros2_ws/yahboomcar_ws/src/
├── yahboomcar_bringup/    # 驱动 (Ackman_driver_R2)
├── yahboomcar_ctrl/       # 控制 (yahboom_joy_R2)
├── yahboomcar_nav/        # 导航 + SLAM launch 文件
│   └── maps/              # 地图保存目录
├── yahboomcar_description/ # URDF 模型
└── ...
```

---

## 四、ROS1 vs ROS2 对比 (针对 R2 场景)

| 维度 | ROS1 Melodic | ROS2 Foxy |
|------|-------------|-----------|
| 节点发现 | 中心化 rosmaster (单点故障) | DDS 去中心化 (无单点故障) |
| R2 驱动 | Mcnamu + 补丁 hack | 原生 Ackman_driver_R2 |
| R2 手柄 | 通用 joy + 补丁 | 原生 yahboom_joy_R2 |
| SLAM | gmapping only | gmapping + cartographer |
| Loop closure | 无 | cartographer 支持 |
| 节点生命周期 | 无管理 | Lifecycle nodes |
| Python | Python 2 | Python 3 |
| 官方维护 | EOL (2023年停止) | Foxy EOL 2023, 但可升级 Humble (2027) |
| 导航 | move_base | Nav2 (功能更强) |
| gmapping 僵尸问题 | 反复发生 | 不存在 |
| 配置复杂度 | 需要大量 hack | 一行命令配置 R2 |

---

## 五、迁移建议

### 推荐路径

1. **下载 ROS2 Docker 镜像**: `docker pull yahboomtechnology/ros-foxy:latest` 或从 Yahboom Google Drive 下载 tar 文件
2. **在 Pi 上加载镜像并配置**: `~/run_docker.sh` → `set_R2_A1.sh`
3. **验证基本功能**: 手柄控制、LiDAR 数据、gmapping 建图
4. **测试 cartographer**: 对比 gmapping 效果，尤其是 loop closure
5. **移植定制功能**: autosave、config 系统、rosbag 自动录制、地图 web viewer (API 变化: `rospy` → `rclpy`)
6. **可选**: 升级到 ROS2 Humble (如果 Yahboom 提供镜像)

### 可以保留的

- `slam_config.sh` 配置体系 (参数名可能需要调整)
- autosave 逻辑 (ROS2 命令从 `rosrun map_server map_saver` 变为 `ros2 launch ... save_map_launch.py`)
- rosbag 录制 (`rosbag record` → `ros2 bag record`)
- postprocess_slam.py (离线处理，与 ROS 版本无关)
- daemon.sh 远程部署模式

### 需要重写的

- 启动脚本 (ROS2 launch 是 Python 文件，不是 XML)
- 手柄定制 (直接修改容器内的 `yahboom_joy_R2.py`)
- 地图 web viewer (订阅接口从 `rospy.Subscriber` 变为 `rclpy` 订阅)

---

## 六、数据流架构

### ROS1 当前数据流

```
轮编码器 + IMU
    ↓
ekf_localization (EKF 融合)
    ↓
/odom
    ↓
gmapping ← /scan (RPLidar A1, ~8Hz)
    ↓
TF: map → odom → base_footprint → laser
    ↓
/map (OccupancyGrid) → autosave → slices/ + overall_map
                     → map_web_viewer (HTTP :8080)
                     → rosbag 录制
```

### 传感器频率

| Topic | 频率 | 来源 |
|-------|------|------|
| /scan | ~8 Hz | RPLidar A1 |
| /odom | 20 Hz | EKF (编码器 + IMU 融合) |
| /imu/imu_data | 14-20 Hz | IMU + Madgwick 滤波 |
| /joy | 20 Hz | 手柄 (autorepeat) |
| /map | ~1 Hz | gmapping |

---

## 七、常用操作速查

### ROS1 (当前)

```bash
# SSH 连接
ssh -i ~/.ssh/rosmaster_codex_nopass pi@192.168.0.110

# 启动全套
/home/pi/rosmaster_tools/rosmaster_slam_start.sh

# 重建容器 (改了 docker 配置后)
docker rm -f rosmaster_slam
/home/pi/rosmaster_tools/rosmaster_slam_start.sh

# 修改参数
nano /home/pi/rosmaster_tools/slam_config.sh

# 拷贝地图到 Mac
scp -i ~/.ssh/rosmaster_codex_nopass "pi@192.168.0.110:/home/pi/rosmaster_maps/overall_map.*" ~/Documents/rosmaster_r2/maps_$(date +%Y%m%d)/

# 拷贝 rosbag
scp -i ~/.ssh/rosmaster_codex_nopass "pi@192.168.0.110:/home/pi/temp/bags/slam_session_*.bag" ~/Documents/rosmaster_r2/maps_$(date +%Y%m%d)/

# 杀僵尸 gmapping
docker exec rosmaster_slam bash -c "pkill -9 -f slam_gmapping"
```

### Mac 端离线后处理

```bash
python3 postprocess_slam.py ~/Documents/rosmaster_r2/maps_YYYYMMDD/bags -o ~/Documents/rosmaster_r2/postprocess -r 0.03
```
