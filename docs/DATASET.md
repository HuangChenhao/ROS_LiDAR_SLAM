# 原始实验数据与恢复

原始数据发布于 [v2.1.0-research Release](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/tag/v2.1.0-research)。仓库中的预览、CSV 和日志便于浏览，Release 附件保留原始传输归档字节，以便重新核对与回放。

## 1. 下载内容

| 附件 | 字节数 | 内容 | 恢复位置 |
|---|---:|---|---|
| [latest_scans_20260722.tar](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/download/v2.1.0-research/latest_scans_20260722.tar) | 283,084,800 | 6 个 ROS2 会话，含原始 bag、地图、轨迹、日志及 RTAB-Map 数据库 | `scan_archives/20260722_latest/` |
| [slam_validation_20260723.tar](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/download/v2.1.0-research/slam_validation_20260723.tar) | 35,461,120 | 3 个修复后静止验证会话及对应 bag、地图、图轨迹和数据库 | `scan_archives/20260723_validation/` |
| [rosmaster_export_current_full.tar.gz](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/releases/download/v2.1.0-research/rosmaster_export_current_full.tar.gz) | 27,294 | 早期 ROS1 地图、二维 PCD、脚本及日志备份，共 37 个文件成员 | 仓库根下的 `rosmaster_export_current_full/` |

三份附件合计 **318,573,214 字节**。07-22 和 07-23 的压缩包分别有 40、27 个文件成员，保持原始归档格式。它们覆盖本次核对的全部 9 个 ROS2 SQLite bag、8 份 ROS2 PGM 和两份 RTAB-Map 数据库；ROS1 备份内另有 5 份地图及 PCD。

早期标签中还有当时提交的整体地图，可从 [v0.1.0 数据树](https://github.com/HuangChenhao/ROS_LiDAR_SLAM/tree/v0.1.0-demo_test/maps)访问；这些历史产物不计入本次 9 个 ROS2 会话。

## 2. 校验与来源

[发布数据清单](../reports/raw_release_manifest_20261118.json)记录每份附件和每个文件成员的 SHA-256、大小、原路径及解压目录；[SHA256SUMS](../reports/SHA256SUMS-raw-v2.1.0-research.txt)用于手工校验。上传时还核对 GitHub 返回的附件摘要与大小。

原始 bag 包含失败会话、不完整的元数据和空轨迹；这些状态被保留。特别是 GMapping 的空 bag YAML 和含 NUL 的会话元数据没有在归档中被改写，详见[问题台账](ISSUES_AND_FIXES.md)。

SQLite 在本地分析时生成的 `-shm` 是共享内存缓存；本次核对的 `-wal` 均为空。它们不在原始传输归档中，也不作为独立采集数据发布。旧[本地文件清单](../reports/raw_data_manifest_20261117.json)包含分析环境的侧文件，复原原始采集时应以发布清单为准。若将来采集存在非空 WAL，则必须将其与数据库共同归档。

## 3. 一条命令下载并恢复

在完整克隆本仓库后运行，建议使用 Python 3.10 或更新版本：

```bash
python tools/download_raw_data.py --extract
```

工具使用 Python 标准库；逐份下载、检查字节数与 SHA-256，再恢复缺失文件。它不会覆盖内容不同的现有文件，会拒绝目录穿越、链接和不符合清单的成员。若有冲突，选择新的 `--root`，或先人工检查差异；不要直接覆盖已有实验。

只获取 7 月 22 日数据：

```bash
python tools/download_raw_data.py --dataset ros2_20260722 --extract
```

只检查已经下载的归档：

```bash
python tools/download_raw_data.py --verify-only
```

`--root` 可指定新的数据目录，默认是仓库根目录。中断下载保留 `.download` 文件，不会在重试时悄悄截断它；检查后将其另存或移除，再重新下载。

## 4. 恢复后复查

```bash
python -m pip install -r requirements-analysis.txt
python -B tools/audit_experiments.py --audit-date 2026-11-18 --out audit_output/restored_audit
```

在仓库根恢复全部原始会话后，应能核对到 9 个 bag、8 个非空 bag、14,882 条 `/scan` 消息与约 2019.828 秒的累计采集时间跨度。输出目录必须尚不存在。

本次已用恢复工具实际恢复并检查 104 个归档文件成员，重新得到上述统计，9 个数据库结构检查均通过。过程摘要见[恢复验证记录](../reports/restoration_verification_20261118.json)。

结构可读和消息数一致不代表 SLAM 精度已验证。标准 ROS2 播放工具还依赖 bag 元数据；GMapping 的空 YAML 应在副本上重建。地图 YAML 可能含原容器的绝对路径，实际导航使用前在工作副本中调整图像路径。

## 5. 为什么放在 Release 附件

最大的原始归档超过普通 Git 的 100 MiB 文件限制。GitHub 建议通过 Release 分发大型文件，单个 Release 附件需小于 2 GiB。因此代码、清单和说明进入 Git，原始归档仍发布在同一 GitHub 仓库的 Release 下载区。

依据：[普通文件限制](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)、[Release 附件说明](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)。直接下载 GitHub 自动生成的 Source code ZIP 不会包含这些单独上传的原始附件，请使用上面的入口或恢复工具。
