# 2026-11-18 ROS2 启动修复与静态验证

关联问题：[SLAM-025](ISSUES_AND_FIXES.md#slam-025开机服务-203exec启动脚本没有执行权限)。本记录描述用户批准后的在线修复；不修改已发布的 v2.1.0-research 标签或附件。

## 问题与直接证据

小车 SSH 和 Docker 可用，但 `rosmaster-ros2.service` 为 failed，退出码为 `203/EXEC`，检查的 ROS2 环境没有运行节点。当前启动日志明确报 `Failed to locate executable ... ros2_bringup.sh: Permission denied`。

主机启动脚本权限为 `0644`，没有执行位；父目录可访问，脚本没有 CRLF，`bash -n` 通过。这里确认的是执行权限这一直接阻碍，没有证据证明由哪一次传输或编辑操作移除了执行位。

## 修复动作

全量实验备份已完成后，经用户批准：

1. 确认启动文件 SHA-256 与备份一致。
2. 给文件所有者恢复执行位，权限由 `0644` 变为 `0744`。
3. 重启现有 `rosmaster-ros2.service`，等待启动完成。
4. 通过只读订阅检查真实数据、TF、串口持有者和控制状态。

文件 SHA-256 保持 `05ebadfa4a33ba72bb6174d0037de26b06d05fa9d2eba38bd92ff74dd51298c9`，本次未修改该脚本内容或部署算法参数。小车已部署脚本与仓库 `patches/` 内容不同，因此这些结果不代表完整仓库版本已经在车上重新部署。

## 检查结果

| 项目 | 观察结果 |
|---|---|
| systemd | `active (exited)`，`Result=success`，`ExecMainStatus=0` |
| 开机启用标记 | `enabled` |
| 关键节点 | driver、base、EKF、IMU filter、joy_ctrl、robot_state_publisher、slam_supervisor 均可发现 |
| 原始/融合里程计 | 约 10 Hz，10 秒窗口各 100 个样本 |
| 原始/滤波 IMU | 约 10 Hz，重力加速度模长中位数约 9.87 m/s² |
| 电压与固件遥测 | 电压约 10.6 V；固件版本字段 3.5 |
| 底盘串口 | 单个驱动进程持有 |
| TF | `odom → base_footprint`、`base_footprint → base_link`、`base_link → laser` 均可查询 |
| 静止观测 | 原始/融合位置在该窗口无位移，实测轮速为 0 |
| 建图状态 | `MappingState=false`；雷达由监督器保持待机，无 `/scan` 发布者 |
| 手柄输入 | `/dev/input/js0` 未出现，USB/输入枚举没有手柄；joydev 已加载，重连循环等待设备 |

启动脚本自带健康检查的六项条件全部通过；后续 10 秒观察确认数据持续到达。原始指标和验证范围见[检查 JSON](../reports/robot_startup_verification_20261118.json)。

## 防止再次部署时丢失权限

仓库的启动脚本增加 Git 可执行位记录；从 Windows 传输文件时仍需显式检查目标权限：

```bash
chmod u+x /home/pi/rosmaster_tools/ros2_bringup.sh
test -x /home/pi/rosmaster_tools/ros2_bringup.sh
bash -n /home/pi/rosmaster_tools/ros2_bringup.sh
```

以上命令只处理和检查启动文件。重启服务会改变驱动、遥控和监督器状态，应在操作者确认后执行。

## 验证边界

`active (exited)` 是 oneshot 启动完成的状态，不能替代持续健康监测。此次没有进行驾驶、现场建图、相机流验证或整机断电重启；因此不延伸为导航、动态 SLAM 或完整开机流程的验证结论。
