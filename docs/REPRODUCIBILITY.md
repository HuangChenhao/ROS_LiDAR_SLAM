# 实验复查与版本说明

## 1. 展示版本的来源

本次整理基于线上 `main` 提交 `8feb75bb25c3ba6817624edecf062e0ed0d48155`，将项目首页改为成果展示，并补充本地采集材料的轻量证据与审计。线上已有的 `patches/`、`systemd/`、操作文档和研发日志保留原内容。

本地盘点包含 2026-07-22、07-23 的 9 个 ROS2 会话和早期 ROS1 导出。线上 7 月 15—17 日的研发日志补充了开发过程；日志中的现场判断不自动等同于本次原始数据审计结论。

## 2. 数据分层

| 内容 | 仓库中的位置 | 用途 |
|---|---|---|
| 展示与结论 | `README.md`、`docs/` | 项目汇报与阅读导航 |
| 轻量实验证据 | `scan_archives/` | 会话日志、轨迹、元数据和地图预览 |
| 审计快照 | `reports/` | SQLite 统计、会话总表、原始文件校验清单 |
| 运行代码 | `patches/`、`systemd/` | 原线上 ROS2 部署入口 |
| 后续研究代码 | `research/` | 结束流程、激光回放和 RGB-D 配置快照 |
| 原始 bag、数据库、PGM/PCD 与压缩包 | [Release 附件](DATASET.md)，按清单恢复 | 完整重算、地图恢复与算法回放 |

原始归档和成员的相对路径、字节数、下载地址与 SHA-256 见[发布清单](../reports/raw_release_manifest_20261118.json)。旧[本地盘点清单](../reports/raw_data_manifest_20261117.json)还包含分析环境的 SQLite 缓存；其来源区别见[数据说明](DATASET.md)。PNG 预览不能代替包含分辨率、原点和占据值的完整导航地图。

## 3. 在普通电脑上复查

在仓库根目录运行，无需 ROS 或机器人：

```bash
python tools/download_raw_data.py --extract
python -m pip install -r requirements-analysis.txt
python -B tools/audit_experiments.py --audit-date 2026-11-18
python -B research/optimization/test_finalization.py
python -B -m unittest discover -s tools/tests
```

原始数据来自同一仓库的 v2.1.0-research Release，下载后核对摘要再恢复。审计写入 `audit_output/20261118/`，拒绝覆盖已有目录。再次核对时指定新目录：

```bash
python -B tools/audit_experiments.py --audit-date 2026-11-18 --out audit_output/recheck_01
```

只克隆代码而未下载附件时，脚本仍可复查日志、轨迹和元数据，但会明确记录 `raw SQLite data absent locally`。恢复附件后可重算完整会话统计。GitHub 自动生成的 Source code ZIP 不包含独立原始附件。

恢复工具会校验每个成员并拒绝覆盖不同内容的现有文件。原始传输归档中的数据库已核对，当前本地 WAL 均为空，分析生成的 SHM 缓存不作为采集附件；未来非空 WAL 必须共同归档。SQLite 审计以只读连接打开数据库，WAL 模式可能触及本地共享内存侧文件。核对期间不会发布 ROS 话题。

## 4. 本次检查结果与边界

- 完整本地数据中，9 个 bag 的 `PRAGMA quick_check` 均返回 `ok`，其中 8 个有消息。
- 累计数据库时间跨度约 2019.828 秒，`/scan` 消息 14,882 条，与原索引交叉核对一致。
- 结束流程研究模块的 4 项单元测试通过，覆盖响应记录、超时、图文件缺失和取消幂等性。
- 已验证无原始数据时，审计会明确记录缺失，仍能识别日志中的后端异常。
- 数据恢复工具另有 4 项检查：重复恢复、目录穿越、已有文件冲突和链接成员拒绝；使用原始归档恢复后重新核对消息统计。

[恢复验证记录](../reports/restoration_verification_20261118.json)保存实际恢复的 104 个文件成员、9 个 bag 的复查统计和 8 项单元测试结果。

盘点环境为 Python 3.13.9 / Windows。数据库可读不证明传感器同步或建图精度；单元测试通过不代表后续研究代码已完成机器人实测。

## 5. 离线 SLAM 回放

[`research/optimization/replay_slam.py`](../research/optimization/replay_slam.py)需要已安装相关后端的 Linux ROS2 Foxy 环境。设置独立的非零 ROS Domain，并在 `PYTHONPATH` 中包含 `research/`，以加载 `slam_finalization.py`。

工具仅回放 `/scan`、`/odom`、`/imu/data`、`/tf`、`/tf_static`，过滤原 SLAM 的 map 变换，不回放运动命令。运行环境应与机器人硬件域分开。

它支持四类激光后端，尚不覆盖 RGB-D 输入、所有后端的统一优化轨迹和完整内存/延迟指标。当前没有配套的完整参数对照结果，不能据此报告优化收益。GMapping 的历史 bag YAML 为空；使用标准播放工具时应在数据副本上重建元数据。

## 6. 版本和实验记录建议

每次实验关联 Git 提交、未提交改动、容器镜像摘要、后端包版本、实际 SLAM/EKF/TF 配置和 SHA-256，并记录路线、起终点、实际速度、成功标准与失败原因。

历史会话没有当前研究代码中的 `finalization.json` 和配置校验快照；不能用现在的参数文件冒充当时的精确配置。旧 `metadata_status` 标签、服务响应和闭环精度也应分别核验。

完整的发布节点、改动原因和问题状态见[版本演进](VERSION_HISTORY.md)、[更新日志](../CHANGELOG.md)和[问题台账](ISSUES_AND_FIXES.md)。后续记录使用[版本模板](templates/VERSION_CHANGE.md)及[实验模板](templates/EXPERIMENT_RECORD.md)。
