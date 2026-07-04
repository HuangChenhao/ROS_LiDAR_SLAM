#!/bin/bash
set -e

C="rosmaster_ros2"

echo "=== Clear all old scan data ==="

# 1. Clear container /tmp map files
echo "--- 1. container /tmp ---"
docker exec "$C" bash -c "rm -f /tmp/*.pgm /tmp/*.yaml /tmp/*.pcd /tmp/test_map.* /tmp/final_map.*" 2>/dev/null || true
echo "cleared"

# 2. Clear container /root/temp (all old map/tmp files, keep nothing)
echo "--- 2. container /root/temp ---"
docker exec "$C" bash -c "rm -rf /root/temp/_tmp_save_* /root/temp/test_save.* /root/temp/overall_map.* /root/temp/current_map_* /root/temp/pre_restart_map.* /root/temp/replay_output /root/temp/trajectory /root/temp/bags/*" 2>/dev/null || true
# Also remove old scripts/patches that are no longer needed
docker exec "$C" bash -c "rm -f /root/temp/Mcnamu_driver.py* /root/temp/patch_*.py /root/temp/yahboom_joy*.py* /root/temp/map_web_viewer*.py /root/temp/replay_slam* /root/temp/trajectory_recorder.py /root/temp/robot_export* /root/temp/rosmaster-slam.service" 2>/dev/null || true
echo "cleared"

# 3. Clear host /home/pi/temp
echo "--- 3. host /home/pi/temp ---"
rm -rf /home/pi/temp/_tmp_save_* /home/pi/temp/test_save.* /home/pi/temp/overall_map.* /home/pi/temp/current_map_* /home/pi/temp/pre_restart_map.* /home/pi/temp/replay_output /home/pi/temp/trajectory /home/pi/temp/bags/* 2>/dev/null || true
rm -f /home/pi/temp/Mcnamu_driver.py* /home/pi/temp/patch_*.py /home/pi/temp/yahboom_joy*.py* /home/pi/temp/map_web_viewer*.py /home/pi/temp/replay_slam* /home/pi/temp/trajectory_recorder.py /home/pi/temp/robot_export* /home/pi/temp/rosmaster-slam.service 2>/dev/null || true
echo "cleared"

# 4. Clear host /home/pi/rosmaster_maps
echo "--- 4. host /home/pi/rosmaster_maps ---"
rm -rf /home/pi/rosmaster_maps/slices 2>/dev/null || true
rm -f /home/pi/rosmaster_maps/*.pgm /home/pi/rosmaster_maps/*.yaml /home/pi/rosmaster_maps/*.pcd /home/pi/rosmaster_maps/*.png 2>/dev/null || true
echo "cleared"

# 5. Verify everything is clean
echo ""
echo "--- 5. verify ---"
echo "Container /root/temp:"
docker exec "$C" bash -c "ls /root/temp/ 2>/dev/null" || echo "  empty"
echo "Host /home/pi/temp:"
ls /home/pi/temp/ 2>/dev/null || echo "  empty"
echo "Host /home/pi/rosmaster_maps:"
ls /home/pi/rosmaster_maps/ 2>/dev/null || echo "  empty"

echo ""
echo "=== DONE ==="
echo "All old scan data cleared. Ready for a fresh scan session."
