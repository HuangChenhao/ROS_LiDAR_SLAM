#!/bin/bash
CONTAINER="rosmaster_slam"

echo "====== 1. 保存当前地图快照 ======"
docker exec $CONTAINER bash -c '
source /opt/ros/melodic/setup.bash
source /root/yahboomcar_ws/devel/setup.bash
rosrun map_server map_saver -f /root/temp/overall_map map:=/map 2>/dev/null
' && echo "地图已保存" || echo "保存失败"

# 确保 host 能访问
ls -la /home/pi/temp/overall_map.* 2>/dev/null
cp /home/pi/temp/overall_map.* /home/pi/rosmaster_maps/ 2>/dev/null

echo ""
echo "====== 2. 确认当前 bag 数据 ======"
ls -lhS /home/pi/temp/bags/*.bag 2>/dev/null | tail -5
echo "bag 总数: $(ls /home/pi/temp/bags/*.bag 2>/dev/null | wc -l)"

echo ""
echo "====== 3. 轨迹数据 ======"
wc -l /home/pi/rosmaster_maps/trajectory.csv 2>/dev/null || echo "无轨迹"

echo ""
echo "====== READY - 可以同步了 ======"
