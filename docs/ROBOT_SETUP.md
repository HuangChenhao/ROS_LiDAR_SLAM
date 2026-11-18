# 机器人部署与操作说明

本页整理原项目 README 中的运行入口。项目成果、实验结果与验证范围见[展示首页](../README.md)和[实验盘点](EXPERIMENTS.md)。

## 1. 平台与运行环境

| 项目 | 原部署记录 |
|---|---|
| 底盘 | Yahboom ROSMASTER R2，阿克曼转向，轴距约 0.25 m |
| 计算平台 | Raspberry Pi 5 |
| 雷达 | RPLIDAR A1 |
| 手柄 | Flydigi Direwolf 3，USB 2.4 GHz 接收器 |
| ROS | ROS2 Foxy，Docker 容器 |
| 镜像 | `yahboomtechnology/ros-foxy:4.0.7R2` |
| 容器 | `rosmaster_ros2` |
| 主机工具目录 | `/home/pi/rosmaster_tools` |
| 服务 | `rosmaster-ros2.service` |

部署行为以 [ros2_bringup.sh](../patches/ros2_bringup.sh) 和 [systemd 服务](../systemd/rosmaster-ros2.service)为准。代码依赖机器人上的 Yahboom ROS2 工作空间和已安装后端；首次部署应核对硬件、工作空间及设备路径。

## 2. 手柄操作

| 操作 | 设计行为 |
|---|---|
| Back | 启用普通控制；再次按下结束普通控制并保存/停止建图 |
| 左摇杆 / 右摇杆 | 行驶 / 转向 |
| LB | 切换速度档位；1、2 档用于建图，3 档停止建图 |
| Y | 选择下次会话的算法；当前会话保留已启动的算法 |
| X | 切换竞速状态，停止建图；再次按下回到未激活状态 |

| Y 键提示色 | 算法 | 参数入口 |
|---|---|---|
| 蓝 | GMapping | [slam_gmapping.yaml](../patches/slam_gmapping.yaml) |
| 黄 | Cartographer | [rosmaster_carto.lua](../patches/rosmaster_carto.lua) |
| 绿 | SLAM Toolbox | [slam_toolbox_r2.yaml](../patches/slam_toolbox_r2.yaml) |
| 紫 | RTAB-Map 2D | [rtabmap_r2.yaml](../patches/rtabmap_r2.yaml) |

更完整的按钮说明见[手柄与小车设置](../Gamepad_and_Car_Setup.md)。

## 3. 启动与状态

原部署由 systemd 启动容器和节点；手工排查时可在机器人主机上查看：

```bash
systemctl status rosmaster-ros2.service
docker ps --filter name=rosmaster_ros2
journalctl -u rosmaster-ros2.service -n 100 --no-pager
```

启动脚本包含串口占用检查、节点启动和传感器健康检查。SLAM 监督器在启动建图前检查磁盘空间，并监测后端退出和移动时地图停更。

## 4. 会话产物

会话保存路径：

```text
/root/rosmaster_maps/YYYYmmdd_HHMMSS_algorithm_gearN/
├── metadata.txt
├── gear_events.csv
├── nodes.log
├── trajectory.csv
├── map.pgm
├── map.yaml
└── bag/
```

根据后端和版本，还可能保存优化轨迹、图边和 RTAB-Map 数据库。部分历史会话缺少产物；以[逐次审计](EXPERIMENTS.md)为准。

## 5. 代码与研究版本

[`patches/`](../patches/)保持本次展示整理之前的线上运行版本。新增的 [`research/`](../research/)辅助模块用于结束流程、离线回放与 RGB-D 研究，没有由此次文档更新部署到机器人。

旧研发日志描述当时的实现和排查判断；操作方式和参数可能随版本变化。ROS1 问题与迁移过程见 [R2 问题记录](../R2_Issues_and_Fixes.md)及[迁移指南](../ROS_问题总结与迁移指南.md)。
