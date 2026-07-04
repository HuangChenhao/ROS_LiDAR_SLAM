#!/bin/bash
# Sync scan data from Pi to Mac with timestamped folder
# Usage: bash sync_scan.sh [session_name]
#   e.g. bash sync_scan.sh office_floor2
#        bash sync_scan.sh   (uses default timestamp-only name)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"
CONTAINER="rosmaster_ros2"
ROS2_SETUP="source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SESSION_NAME="${1:-}"
if [ -n "$SESSION_NAME" ]; then
    SCAN_DIR="$SCRIPT_DIR/scans/${TIMESTAMP}_${SESSION_NAME}"
else
    SCAN_DIR="$SCRIPT_DIR/scans/${TIMESTAMP}"
fi

mkdir -p "$SCAN_DIR"

echo "=== Sync scan data ==="
echo "Target: $SCAN_DIR"
echo ""

# 1. Save current map on Pi
echo "--- 1. Saving map on Pi ---"
ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash << 'REMOTE'
CONTAINER="rosmaster_ros2"
ROS2_SETUP="source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1"

# Save map (use integer timeout to avoid Foxy bug)
docker exec "$CONTAINER" bash -c "$ROS2_SETUP && ros2 run nav2_map_server map_saver_cli -f /tmp/final_map --ros-args -p save_map_timeout:=10" 2>&1 || \
docker exec "$CONTAINER" bash -c "$ROS2_SETUP && ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \"{map_topic: '/map', map_url: '/tmp/final_map', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.25, occupied_thresh: 0.65}\"" 2>&1 || \
echo "WARNING: map_saver failed, will try to copy existing map"

# Copy from container to host
docker cp "$CONTAINER:/tmp/final_map.pgm" /tmp/final_map.pgm 2>/dev/null || true
docker cp "$CONTAINER:/tmp/final_map.yaml" /tmp/final_map.yaml 2>/dev/null || true

ls -lh /tmp/final_map.* 2>/dev/null || echo "No final_map files"
REMOTE

echo ""

# 2. Pull files from Pi to Mac
echo "--- 2. Pulling map files ---"
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/tmp/final_map.pgm" "$SCAN_DIR/map.pgm" 2>/dev/null && echo "  map.pgm OK" || echo "  map.pgm FAILED"
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/tmp/final_map.yaml" "$SCAN_DIR/map.yaml" 2>/dev/null && echo "  map.yaml OK" || echo "  map.yaml FAILED"

# 3. Also pull any logs
echo ""
echo "--- 3. Pulling logs ---"
scp -i "$SSH_KEY" $SSH_OPTS "$HOST:/home/pi/rosmaster_tools/ros2_bringup.log" "$SCAN_DIR/bringup.log" 2>/dev/null || true

# 4. Save metadata
echo ""
echo "--- 4. Saving metadata ---"
cat > "$SCAN_DIR/metadata.txt" << EOF
Scan session: $(basename "$SCAN_DIR")
Date: $(date '+%Y-%m-%d %H:%M:%S')
Robot: Yahboom ROSMaster R2
ROS: ROS2 Foxy
SLAM: gmapping
LiDAR: RPLidar A1
EOF

# 5. Summary
echo ""
echo "=== Done ==="
echo "Scan saved to: $SCAN_DIR"
ls -lh "$SCAN_DIR/" 2>/dev/null
echo ""
echo "To view the map: open $SCAN_DIR/map.pgm"
