# 更新日志

这里按实际 Git 标签、提交和实验记录整理项目演进。发布日期、代码提交日期与实验发生日期分别记录；未发布的开发节点不会补造为历史 Release。

详细材料：[版本演进](docs/VERSION_HISTORY.md) · [问题与修复台账](docs/ISSUES_AND_FIXES.md) · [完整提交清单](reports/version_history_20261118.json)。

## 未发布 — 2026-11-18 在线启动修复

- 实际小车因启动脚本为 `0644`，systemd 以 `203/EXEC` 失败。用户批准后恢复执行位为 `0744`，脚本 SHA-256 不变，重启成功。
- 10 秒静态观察确认驱动、IMU、里程计和 TF 工作；建图关闭、轮速为 0，未进行驾驶或现场建图。
- 当前未检测到手柄输入设备，重连循环等待设备；遥控驾驶尚未验证。
- 仓库启动脚本记录可执行位，并增加部署权限检查，补充 [SLAM-025 与验证记录](docs/STARTUP_REPAIR_20261118.md)。已发布的 v2.1.0 标签与数据附件保持原样。

## v2.1.0-research — 2026-11-18

**类型：研究预发布。** 在 v2.0.0 ROS2 运行实现上，固定可展示、可下载、可复查的研究快照。

### 新增

- 公开三份原始传输归档：早期 ROS1 导出、2026-07-22 的 6 个 ROS2 会话、2026-07-23 的 3 个验证会话。
- 原始归档作为 [Release 附件](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/tag/v2.1.0-research)提供；增加逐文件 SHA-256、归档成员清单与下载恢复工具。
- 逐次实验总表、17 张地图预览与叠加图、原始日志和轨迹、SQLite 审计结果及研究报告提纲。
- 31 个已有提交、3 个历史标签与中间开发节点的版本记录，以及按症状、证据、处理、验证和剩余问题编写的台账。
- 后续研究辅助模块：有界结束流程、离线输入回放和 RGB-D + LiDAR 配置；这些模块独立保存在 `research/`。
- 后续版本变更、问题和实验记录模板。

### 修正说明

- 将 SLAM Toolbox 的历史 `complete` 标签与后端致命异常分开判断。
- 将 RTAB-Map 日志中的关键词匹配与成功闭环约束分开，避免把“闭环被禁用”提示计为成功。
- 明确 7 月 23 日是静止验证，档位标签不等于实际行驶速度。
- 明确在线轨迹、优化轨迹与外部真值的区别；首尾距离不直接称作 ATE/RPE。
- 给早期问题文档增加历史阶段说明，保留原文并指出后续判断应查看当前台账。

### 验证范围

原始归档与本地持久数据按成员 SHA-256 对照；9 个 rosbag SQLite 可读，其中 8 个非空，累计约 33.7 分钟、14,882 条 `/scan` 消息。结束流程研究模块有 4 项单元测试。下载恢复工具另行检查归档校验、路径边界和文件冲突。

`patches/` 与 `systemd/` 的运行内容继承 v2.0.0；本版没有向机器人部署新代码，也没有新增动态闭环精度或 RGB-D 融合实测结论。

## v2.0.0-ros2-mapping — 2026-07-24

**实际标签：** [`v2.0.0-ros2-mapping`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/tag/v2.0.0-ros2-mapping)，指向 [`31c2bb2`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/31c2bb26f9339d903ebcff899ae55bd52a922d6d)。GitHub 历史状态为正式发布，保留不变。

### 功能与实现

- 通过 Y 键选择 GMapping、Cartographer、SLAM Toolbox、RTAB-Map 2D。
- 增加 R2 稳定里程计、启动健康检查、驱动与 launch 修订，处理串口占用和重复节点。
- 增加算法/档位命名、档位事件、后端退出和地图停更检查、磁盘空间保护及雷达重试。
- 调整 SLAM Toolbox 搜索网格与队列、RTAB-Map 邻近约束/ICP 参数。
- Cartographer 显示与导出接入优化图轨迹；大地图适配屏幕。

### 对应实验与边界

7 月 22 日保留了故障和行驶会话，7 月 23 日保留了三次修复后静止验证。代码实现与当时的 Release 说明不能替代长距离闭环、轮滑或算法精度对照。详见 [SLAM-013—SLAM-022](docs/ISSUES_AND_FIXES.md)。

## 2026-07-20 — EKF 与四后端扩展开发节点

此阶段没有另建 Release。

- [`1cfbb84`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/1cfbb84881fe79e12a909110088afd32f8bfdd60)：加入平面 EKF 配置；不直接融合轮式里程计绝对位姿，处理大位姿跳变传播问题；完善启动错误处理、手柄重连和 systemd 入口。
- [`bfec5c5`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/bfec5c5)：加入 SLAM Toolbox 与 RTAB-Map 配置及监督器分支，扩展算法显示和操作说明。

这些代码变更的日期不能作为 7 月 23 日验证会话的发生日期。

## 2026-07-17 — Cartographer、阈值渲染与离线重建

开发节点 [`e7ebf86`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/e7ebf86)，无独立 Release。

- 增加 GMapping / Cartographer 双算法切换与 Cartographer 配置。
- 修正地图渲染只接受占用值 `100` 导致部分墙体消失的问题，改为概率阈值渲染。
- 增加 ROS2 bag 记录、时钟发布和离线重建流程。
- 记录户外低压、TF 中断、架空空转和地图碎裂现象；日志中场景优劣的判断属于有限试验观察。

证据：[当日研发日志](logs/2026-07-17_cartographer与问题排查.md)。当时的原始重建 bag 不在本次 9 个会话中。

## 2026-07-15 — 手柄、档位、监督器与实时地图

开发节点 [`c05efa0`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/c05efa0) 与 [`14f4a19`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/14f4a19)，无独立 Release。

- 增加未激活红灯、档位指示、绝对速度上限和默认转向灵敏度。
- 增加常驻监督器、雷达 DTR 管理、会话命名和屏幕地图显示。
- 处理手柄晚启动、重复 joy 节点和 LED 动画覆盖。
- 档位联动行为曾有中间调整：当日日志记录过“激活即建图”；后续恢复为低档建图、高档/竞速结束。查看旧记录时必须同时核对代码版本。

证据：[手柄与档位日志](logs/2026-07-15_红灯指示与档位调整.md)、[生命周期与显示日志](logs/2026-07-15b_档位联动建图与屏幕实时显示.md)。

## 2026-07-04 — ROS2 Foxy 迁移开发节点

提交 [`3904b75`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/3904b75)，提交标题中的 `v1.0.0-ros2` 没有对应独立 Git 标签。

- 仓库切换为 ROS2 Foxy 的补丁与操作说明结构。
- 增加 R2 驱动、手柄和 bringup 等补丁入口。
- 整理 ROS1 问题和迁移指南，历史 ROS1 源文件仍可从旧标签查看。

## v1.0.0-ros1-alpha — 2026-07-03

实际标签 [`v1.0.0-ros1-alpha`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/tag/v1.0.0-ros1-alpha)，指向 [`73b380c`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/73b380c23561eed95e6b9160dca95d23ee534898)。标签名包含 alpha，但历史 GitHub `prerelease` 标志为 false，不重新解释为现今稳定性保证。

- 固定 ROS1 Melodic 阶段的系统与问题记录，随后转入 ROS2。
- 增加底盘模式、URDF/轴距、里程计、PID、校准与导航约束的排查笔记。
- 记录 autosave 重入、手柄设备映射、地图质量和保存流程问题。
- 部分记录属于诊断假设与待实施方案，例如“固件限制导致遥测异常”，未由该版本证实。

## v0.1.0-demo_test — 2026-06-29

实际标签 [`v0.1.0-demo_test`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/tag/v0.1.0-demo_test)，指向 [`b24e46d`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/b24e46df2c23351b2dabd14e9d0f7576eff20d2f)，历史状态为预发布。

- 初步集成 RPLIDAR A1、ROS1 GMapping、遥控、定时地图导出和同步工具。
- 对里程计全零现象使用命令速度反馈补丁，加入轨迹记录和离线后处理。
- 标签中有早期整体地图；命令速度回退不等于恢复真实编码器测量，也不能证明实际轨迹精度。
- 原发布说明中的扫描匹配分数与“v0.2”等实验标记保留为当时记录，不提升为独立正式版本或跨算法精度指标。

## 发布之后的文档更新

2026-08-26、09-16 的提交更新了照片和 README；2026-11-18 的 [`fab4cbc`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/fab4cbcbb915c4114ab077bd4e16b89b26bc84aa) 和 [`487ee43`](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/commit/487ee436e6c8fc587aeca26468bd059ee36c07fb) 将首页改为展示页并补充轻量实验材料。这些不是新增 SLAM 精度实验。
