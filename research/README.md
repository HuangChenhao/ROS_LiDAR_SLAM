# 后续研究辅助代码

本目录保存本地研究快照，未由本次展示整理合入 `patches/` 或部署到机器人。

- `slam_finalization.py`：有界结束流程、服务响应与配置记录。
- `optimization/test_finalization.py`：无需 ROS 的四项单元测试。
- `optimization/replay_slam.py`：需要 ROS2 Foxy 的四类激光 SLAM 回放工具。
- `rtabmap_rgbd_r2.yaml`：待采集验证的 RGB-D + LiDAR 研究配置。

使用条件见[复查说明](../docs/REPRODUCIBILITY.md)，验证范围见[实验盘点](../docs/EXPERIMENTS.md)。
