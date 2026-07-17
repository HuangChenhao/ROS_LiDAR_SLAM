# 2026-07-17 工作日志 — Cartographer 双算法、扫描问题排查、离线重建管线

## 1. 首次户外扫描复盘（07-15 下午）

**低压报警**：扫描以"滴滴滴"结束 — 扩展板硬件低压保护（切电机+鸣叫），电压遥测坏（0.0V）但硬件保护独立工作。之后 TF 断裂（STM32 断电→无里程计），gmapping 疯狂丢帧。**充电时关电源开关**可避免持续鸣叫。

**地图碎裂根因**（gmapping）：
- `minimum_score: 0.0`（容器默认）— 任何烂匹配都接受 → 位姿传送
- `maxUrange: 4.0` 户外太短 → 已调优为 minimum_score=100, maxUrange=8, particles=50
- 调优后仍碎 → 户外稀疏特征是 gmapping 的结构性弱点

**架空对照实验**：车架起、轮子空转 → 地图也碎。证明 gmapping 的运动先验完全依赖 odom（编码器），激光匹配窗口只有 ±0.2m/几度，救不回"说谎的里程计"。编码器本身验证过是好的（指令0.1 vs 实测0.096 m/s）。

## 2. 双 SLAM 算法（Y 键切换）

- **Y 键**：蓝闪3下 = gmapping，黄闪3下 = cartographer（下次建图生效，metadata 记录算法）
- 驱动新增 LED 编号：8=蓝闪、9=黄闪（先停特效再闪，闪完 joy 恢复状态灯）
- `rosmaster_carto.lua` 关键调优：
  - 实时相关匹配 + 大搜索窗口（0.3m / 25°）— 抗里程计说谎
  - 回环检测 optimize_every_n_nodes=30
  - max_range 10m 适配户外

## 3. 关键 bug：cartographer 地图"没墙"

存图/预览只把占用值 `==100` 画成墙，但 cartographer 输出概率值（~55-100），墙全被丢弃 → cartographer 显得"效果差"。**修复：阈值渲染（≥55=墙，0-25=空闲）**，三处渲染器同步修正。

## 4. 对比结论（同场地 4-5 分钟各一次）

| | gmapping | cartographer |
|---|---|---|
| 结构 | 碎成多块，轨迹跳跃 | **一整块连贯** |
| 花坛圆弧 | 无法重建 | **可见** |
| 结论 | 只适合室内小场景 | **户外首选** |

## 5. 离线重建管线（鲁棒性兜底）

- 每次建图会话自动 `ros2 bag record`（/scan /odom /imu /tf /tf_static，~6MB/min）
- Foxy `bag play` 无 `--clock` → 新增 `bag_clock_pub.py`（读 bag 时间范围发布 /clock）
- 重放流程：cartographer(use_sim_time) + bag_clock_pub + bag play → 抓 /map
- 已验证：07-15 的 cartographer bag 离线重建成功

## 6. 其他新功能（本批一起部署）

- 屏幕实时预览彩色化：红色轨迹 + 蓝色车位置，0.5s 轮询
- X 键飙车模式：最高速 + 星光灯 + 暂停建图
- 空会话自动删除（误触不留垃圾）
- trajectory.csv 每会话保存

## 7. 出门检查单

1. 电池充满（低压报警=立即回家）
2. 开手柄 → 开车 → 红灯待命
3. Y 键切 cartographer（黄闪3下）
4. Back 激活 → 屏幕弹图 → 1档慢扫，贴近特征 2-4m，重点区域走两遍
5. Back 结束 → 回家 `bash sync_scan.sh` 拉取（含 bag）
