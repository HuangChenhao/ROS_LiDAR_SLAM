#!/bin/bash
CONTAINER="rosmaster_slam"

echo "====== 查找 gmapping 相关文件 ======"
docker exec $CONTAINER find /root/yahboomcar_ws -name "*.launch" 2>/dev/null | grep -iE "gmap|slam|map"
echo "---"
docker exec $CONTAINER find /root/yahboomcar_ws -name "*.launch" 2>/dev/null
echo "---"
echo "检查当前 gmapping 参数:"
docker exec $CONTAINER bash -c 'source /opt/ros/melodic/setup.bash && source /root/yahboomcar_ws/devel/setup.bash && rosparam get /slam_gmapping 2>/dev/null' | grep -E "maxUrange|maxRange|minimumScore|particles|linearUpdate|angularUpdate|temporalUpdate"

echo ""
echo "====== DONE ======"
