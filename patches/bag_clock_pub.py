#!/usr/bin/env python3
# Foxy 'ros2 bag play' has no --clock. This node publishes /clock synced to a bag's
# recorded time range so cartographer(use_sim_time) can replay old bags.
# Usage: bag_clock_pub.py <bag_dir> [rate]
import sqlite3, sys, time, glob
import rclpy
from rosgraph_msgs.msg import Clock

bag_dir = sys.argv[1]
rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
db = glob.glob(bag_dir + '/*.db3')[0]
conn = sqlite3.connect(db)
t_min, t_max = conn.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
conn.close()

rclpy.init()
n = rclpy.create_node('bag_clock_pub')
pub = n.create_publisher(Clock, '/clock', 10)
t0_wall = time.time()
print(f'clock: {t_min} .. {t_max} ({(t_max-t_min)/1e9:.0f}s) rate={rate}', flush=True)
while rclpy.ok():
    sim_ns = int(t_min + (time.time() - t0_wall) * rate * 1e9)
    if sim_ns > t_max + int(5e9):
        break
    msg = Clock()
    msg.clock.sec = sim_ns // 1000000000
    msg.clock.nanosec = sim_ns % 1000000000
    pub.publish(msg)
    time.sleep(0.02)
print('clock done', flush=True)
