#!/bin/bash
echo "=== Clear test sessions ==="
docker exec rosmaster_ros2 bash -c "rm -rf /root/rosmaster_maps/2026*" 2>/dev/null
rm -rf /home/pi/rosmaster_maps/2026* 2>/dev/null
echo "car sessions: $(docker exec rosmaster_ros2 bash -c 'ls /root/rosmaster_maps/ | wc -l') (want 0)"
echo "pi staged: $(ls /home/pi/rosmaster_maps/ 2>/dev/null | wc -l) (want 0)"
echo "=== DONE ==="
