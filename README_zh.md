# ROSMaster R2 — LiDAR SLAM 工具集

这是一套在 Mac 端运行的脚本，用于通过 SSH 操作、诊断 **Yahboom ROSMaster R2** 机器人并收集 SLAM 数据。

> For English documentation, see [README.md](README.md)

---

## 硬件配置

| 组件 | 说明 |
|------|------|
| 机器人 | Yahboom ROSMaster R2 |
| 单板计算机 | Raspberry Pi（IP: `192.168.0.110`，用户: `pi`）|
| 激光雷达 | RPLidar（设备: `/dev/rplidar`）|
| 深度相机 | Astra（设备: `/dev/astradepth`, `/dev/astrauvc`）|
| 电机控制器 | `/dev/myserial` |
| ROS 环境 | Melodic，运行于 Docker 容器 `rosmaster_slam` 内 |

---

## 仓库结构

```
rosmaster/
├── robot.sh              # 核心 SSH 封装 — 在小车上执行任意命令
├── setup_ssh.sh          # 一次性配置免密 SSH
├── daemon.sh             # 命令守护进程：监听触发文件，自动执行远程命令
├── cmd.sh                # 向小车发送命令块
├── run.sh                # 快速执行助手
├── remote_exec.sh        # 远程执行助手
├── diagnose.sh           # 完整 ROS / 系统诊断
├── diagnose2.sh          # 显示环境、Docker X11、pygame 诊断
├── fix_and_start.sh      # 修复设备映射，重启 ROS / Docker
├── full_check.sh         # 全面系统检查（设备、Docker、ROS、网络）
├── stop_buzzer.sh        # 关闭机器人蜂鸣器
├── sync_data.sh          # 将 SLAM 地图、bag 包、轨迹从小车同步到 Mac
├── pull_robot_files.sh   # 拉取小车导出文件
├── remote_commands.sh    # 远程命令文件（由守护进程工作流写入）
└── visualize_trajectory.py  # 将录制的轨迹叠加到 SLAM 地图上
```

---

## 快速开始

### 1. 配置免密 SSH（只需执行一次）

```bash
bash ~/Claude/Projects/rosmaster/setup_ssh.sh
```

脚本会将 SSH 密钥复制到 `~/.ssh/`，并在 `~/.ssh/config` 中添加 `rosmaster` 主机别名。执行过程中需要输入一次机器人密码（`yahboom`）。

### 2. 在小车上执行命令

```bash
bash ~/Claude/Projects/rosmaster/robot.sh "rosnode list"
```

### 3. 将 SLAM 数据同步到 Mac

```bash
bash ~/Claude/Projects/rosmaster/sync_data.sh
```

数据同步到 `~/Documents/rosmaster_r2/`：

```
rosmaster_r2/
├── maps/
│   ├── overall_map.pgm          # SLAM 占用栅格地图
│   ├── overall_map.yaml         # 地图元数据（分辨率、原点）
│   ├── trajectory.csv           # 录制的机器人轨迹（x, y, theta）
│   ├── slices/                  # 增量 PCD 切片
│   └── map_with_trajectory.png  # 可视化输出图像
├── bags/                        # ROS bag 文件
└── logs/                        # 服务日志和录制日志
```

### 4. 可视化轨迹

同步完成后，生成轨迹叠加图像：

```bash
python3 ~/Claude/Projects/rosmaster/visualize_trajectory.py ~/Documents/rosmaster_r2/maps
```

输出 `map_with_trajectory.png`（以及 `.ppm` 备用格式）。红点 = 轨迹路径，绿色 = 起点，蓝色 = 终点。

---

## 诊断脚本说明

| 脚本 | 用途 |
|------|------|
| `diagnose.sh` | ROS 节点 / 话题、进程、systemd 服务、GPIO、I2C |
| `diagnose2.sh` | 显示环境、Docker X11 挂载、pygame 安装情况 |
| `full_check.sh` | 设备、Docker 容器详情、ROS 环境、磁盘 / 内存 / 负载 |
| `fix_and_start.sh` | 重新加载 udev 规则，重建 Docker 容器，重启 `rosmaster-slam.service` |
| `stop_buzzer.sh` | 通过 ROS 话题或 GPIO 关闭蜂鸣器 |

所有诊断脚本的输出保存为 `*_output.txt` 文件（已通过 `.gitignore` 排除）。

---

## 守护进程工作流

用于无需每次重新建立 SSH 连接的重复命令执行：

```bash
# 终端 1 — 启动守护进程
bash ~/Claude/Projects/rosmaster/daemon.sh

# 终端 2 — 将命令写入 remote_commands.sh，然后触发执行
touch ~/Claude/Projects/rosmaster/.trigger
# 守护进程检测到 .trigger 后，在小车上执行 remote_commands.sh，
# 并将结果写入 cmd_output.txt
```

---

## 依赖要求

- macOS，需要 `bash`、`ssh`、`scp`
- Python 3，需要 `numpy`、`pyyaml`（用于 `visualize_trajectory.py`）
- 可选：`Pillow` 用于生成 PNG（`pip3 install Pillow`）
- 机器人与 Mac 在同一局域网，可通过 `192.168.0.110` 访问

---

## 安全说明

包含 SSH 私钥的 `.ssh/` 目录已通过 `.gitignore` 排除，不会提交到仓库。**请勿将 SSH 私钥提交到 git。**
