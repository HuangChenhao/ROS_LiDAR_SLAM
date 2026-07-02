# ROSMaster R2 — 当前问题与修复计划

> 基于 Yahboom 官方 PDF 教程 (GitHub ROSMASTER-R2-main) 与实际代码库对比分析
>
> 更新日期: 2026年7月2日

---

## 1. 轴距/URDF 错误 — 最严重

| | 官方值 | 我们的值 |
|---|-------|---------|
| 轴距 (wheelbase) | **0.25 m** (TEB 文档) | 0.16 m (X3 URDF) |
| URDF 模型 | yahboomcar_R2.urdf | yahboomcar_X3.urdf |
| 最小转弯半径 | 0.768 m | 未配置 |

**影响:** 所有 TF 变换、EKF 里程计积分都基于错误的物理尺寸。转弯时误差最大 — 这是地图撕裂的系统性原因之一。

**来源:** `03.ROS1-R2 Car Tutorial/12.Lidar course/14.TEB path planning algorithm/TEB path planning algorithm.pdf` 明确写了 R2 参数 `wheelbase: 0.25`, `min_turning_radius: 0.768`。

**修复方案:**
1. 在容器内找到 `yahboomcar_R2.urdf`（应在 `yahboomcar_description/urdf/` 下）
2. 将 `ROBOT_TYPE` 改回 `R2`
3. 修改 `bringup.launch`，使 driver_node 和 LiDAR TF 不再限定 X3
4. 或直接修改 `laser_bringup.launch` 移除 `if="$(eval arg('robot_type') == 'X3')"` 条件

---

## 2. 编码器不工作 — 开环里程计

**现象:** `Mcnamu_driver.py` 中 `self.car.get_motion_data()` 返回的数据无效，代码已注释掉，改用 cmd_vel 命令速度作为假里程计。

**官方设计:**
- 正交编码器，STM32 Timer 编码器模式 (TI1+TI2 = 4倍频)
- 定时器分配: M1→TIM2, M2→TIM4, M3→TIM5, M4→TIM3
- 采样间隔: 10ms，串口输出间隔: 100ms
- PID 出厂已调好（增量式 PID），不建议自行调整

**可能原因:**
1. 容器内原版驱动没有调用 `set_car_type(CARTYPE_R2)` — STM32 仍在 X3 模式下解读编码器
2. STM32 固件版本不匹配 R2 底盘
3. 硬件接线问题（可能性较低，电机能正常转）

**诊断步骤 (下次连车时):**
```python
# 在容器内 Python 中测试
from Rosmaster_Lib import Rosmaster
car = Rosmaster()
car.set_car_type(car.CARTYPE_R2)

# 1. 读 PID 参数
print(car.get_motion_pid())  # 正常应返回 (kp, ki, kd)，错误返回 (-1,-1,-1)

# 2. 让电机转，读编码器
car.set_car_motion(0.3, 0, 0)
import time; time.sleep(1)
vx, vy, angular = car.get_motion_data()
print(f"vx={vx}, vy={vy}, angular={angular}")  # 如果全是 0 或乱码则确认编码器有问题
car.set_car_motion(0, 0, 0)

# 3. 重置出厂参数 (最后手段)
# car.reset_flash_value()  # 或长按扩展板 KEY1 约 10 秒
```

**修复方案:**
- 如果诊断后编码器数据有效 → 部署 R2 补丁驱动，去掉 cmd_vel 替代方案，恢复闭环
- 如果编码器确实坏了 → 切换到 Hector SLAM（不需要里程计）

---

## 3. ROBOT_TYPE=X3 兼容性 Hack

**现状:** `slam_config.sh` 中 `ROBOT_TYPE=X3`，因为 `bringup.launch` 中：
- `Mcnamu_driver.py` 只在 `robot_type == 'X3'` 时启动
- LiDAR 的 static TF (`base_link → laser`) 只在 `robot_type == 'X3'` 时发布
- URDF 按 `robot_type` 加载 → 加载了 X3 的麦轮模型

同时 Docker 环境变量 `YAHBOOM_BASE_TYPE=R2` 让驱动内部以 R2 阿克曼模式运行。

**问题:** 驱动模式正确 (R2)，但 URDF、TF、launch 条件全是 X3 的。

**修复方案 (改 launch 文件):**

`bringup.launch` — 让 driver_node 对 R2 也启动:
```xml
<!-- 原来: if="$(eval arg('robot_type') == 'X3')" -->
<!-- 改为: -->
<node pkg="yahboomcar_bringup" type="Mcnamu_driver.py" name="driver_node"
      required="true" output="screen"
      if="$(eval arg('robot_type') in ['X3', 'R2'])">
```

`laser_bringup.launch` — LiDAR TF 对 R2 也发布:
```xml
<!-- 原来: if="$(eval arg('robot_type') == 'X3')" -->
<!-- 改为 (R2 的 LiDAR 安装位置可能不同，需实测确认): -->
<node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
      args="0.0435 5.258E-05 0.11 3.1416 0 0 /base_link /laser 30"
      if="$(eval arg('robot_type') in ['X3', 'R2'])"/>
```

然后将 `ROBOT_TYPE` 改回 `R2`。

---

## 4. PID 被注释掉

**现状:** `Mcnamu_driver.py` 的 `dynamic_reconfigure_callback` 中：
```python
# self.car.set_pid_param(config['Kp'], config['Ki'], config['Kd'])
```
PID 调用被注释掉了，意味着无法通过 `rqt_reconfigure` 动态调参。

**官方设计:** PID 在 STM32 端执行（增量式），出厂已调好。Python 端通过 `set_pid_param()` 可以修改，`get_motion_pid()` 可以读取。

**修复:** 编码器恢复工作后，取消注释 PID 调用，允许在线调参。

---

## 5. 导航规划器不适用

**官方文档明确指出:** DWA 算法不适用于 R2 阿克曼模型，因为阿克曼无法原地旋转。

**R2 专用规划器:** TEB (Timed Elastic Band)，关键参数:
```yaml
min_turning_radius: 0.768
wheelbase: 0.25
max_vel_x: 0.4
max_vel_x_backwards: 0.4
max_vel_y: 0.0        # 阿克曼不能横移
max_vel_theta: 1.0
acc_lim_x: 0.5
acc_lim_theta: 0.5
```

**修复:** 未来做导航时，使用 `costmap_common_params_R2.yaml` 和 `dwa_local_planner_params_R2.yaml`（实际应为 TEB 参数文件）。

---

## 6. SLAM 算法已预装

官方系统镜像已预装以下算法，无需从源码编译：

| 算法 | 启动命令 | 备注 |
|------|---------|------|
| gmapping | `roslaunch yahboomcar_nav yahboomcar_map.launch map_type:=gmapping` | 当前使用 |
| Hector SLAM | `roslaunch yahboomcar_nav yahboomcar_map.launch map_type:=hector` | 不需要里程计 |
| Karto | `roslaunch yahboomcar_nav yahboomcar_map.launch map_type:=karto` | 图优化 |
| Cartographer | 需先运行 `copy_carto.sh` | 回环检测，精度最高 |

**建议:** 如果编码器无法修复，优先尝试 Hector SLAM。

---

## 7. 校准程序不适用 R2

官方校准教程（10.7 Robot calibration）明确标注：线速度和角速度校准 **不适用于 R2 (阿克曼模型)**，原因是运动方向控制原理不同。

R2 的校准需要单独处理阿克曼转向几何。

---

## 8. STM32 固件

**烧录方法:**
1. 工具: mcuisp 或 FlyMcu，通过 CH340 USB 串口连接
2. 进入烧录模式: 按住 BOOT0 → 按下 RESET → 松开 BOOT0
3. 设置: "DTR 低电平复位，RTS 高电平进入 BootLoader"
4. 固件文件: `Rosmaster_XXX.hex`（在 Yahboom 网盘下载，需提取码）

**恢复出厂:** `reset_flash_value()` 或长按 KEY1 约 10 秒。

---

## 修复优先级

| 优先级 | 任务 | 前置条件 | 预估影响 |
|--------|------|---------|---------|
| **P0** | 诊断编码器 (get_motion_data 测试) | 连接小车 | 确认根因 |
| **P0** | 部署 R2 驱动补丁 | 连接小车 | 启用 R2 阿克曼模式 |
| **P1** | 修改 launch 文件 + ROBOT_TYPE=R2 | 找到 R2 URDF | 修复 TF 和尺寸 |
| **P1** | 找到并加载 yahboomcar_R2.urdf | 检查容器内文件 | 正确轴距 0.25m |
| **P2** | 测试 Hector SLAM | 编码器确认坏 | 无里程计建图 |
| **P2** | 测试 Cartographer | 有足够算力 | 回环检测 |
| **P3** | 固件更新 | 获取固件+烧录工具 | 可能修复编码器 |

---

## 诊断脚本 (待部署)

下次连车时执行的完整诊断流程已准备好，包括：
1. 编码器数据读取测试
2. PID 参数读取
3. R2 驱动补丁部署
4. URDF 文件查找
5. Hector SLAM 快速切换测试

脚本位置: 待写入 `remote_commands.sh` 并通过 daemon 执行。

---

> 参考资料:
> - Yahboom 官方 GitHub: `/Users/chenhao/Downloads/ROSMASTER-R2-main/`
> - 关键 PDF: `14.2 Kinematics Analysis of Ackermann Car.pdf`, `13.Timer captures the encoder data.pdf`, `15. PID control robot movement.pdf`, `TEB path planning algorithm.pdf`
> - 当前代码库: `/Users/chenhao/Downloads/Rosmaster_R2_LiDAR_SLAM-main/`
