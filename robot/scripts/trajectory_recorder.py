#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
轨迹记录节点：记录 /odom 轨迹并周期性保存为 CSV
可用于后期叠加到地图上（红色轨迹）
"""
import rospy
import csv
import os
import time
import threading
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TrajectoryRecorder:
    def __init__(self):
        rospy.init_node('trajectory_recorder', anonymous=False)

        self.save_dir = rospy.get_param('~save_dir', '/root/rosmaster_slam_logs')
        self.save_interval = rospy.get_param('~save_interval', 30)  # 30秒保存一次
        self.min_distance = rospy.get_param('~min_distance', 0.02)  # 2cm 最小移动距离

        self.trajectory = []  # [(timestamp, x, y, theta)]
        self.last_x = None
        self.last_y = None
        self.lock = threading.Lock()

        # CSV 文件路径
        self.csv_path = os.path.join(self.save_dir, 'trajectory.csv')
        self.csv_overall = os.path.join(self.save_dir, 'trajectory_overall.csv')

        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)

        # 周期保存线程
        self.save_thread = threading.Thread(target=self.periodic_save)
        self.save_thread.daemon = True
        self.save_thread.start()

        rospy.on_shutdown(self.save_trajectory)
        rospy.loginfo("轨迹记录器启动: 保存到 %s", self.save_dir)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 过滤微小移动
        if self.last_x is not None:
            dist = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
            if dist < self.min_distance:
                return

        self.last_x = x
        self.last_y = y

        t = msg.header.stamp.to_sec()
        with self.lock:
            self.trajectory.append((t, x, y, theta))

    def periodic_save(self):
        while not rospy.is_shutdown():
            time.sleep(self.save_interval)
            self.save_trajectory()

    def save_trajectory(self):
        with self.lock:
            if not self.trajectory:
                return
            points = list(self.trajectory)

        try:
            with open(self.csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'theta'])
                for row in points:
                    writer.writerow(['{:.6f}'.format(row[0]),
                                   '{:.6f}'.format(row[1]),
                                   '{:.6f}'.format(row[2]),
                                   '{:.6f}'.format(row[3])])

            # 也保存到 maps 目录供同步
            maps_csv = '/home/pi/rosmaster_maps/trajectory.csv'
            with open(maps_csv, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'theta'])
                for row in points:
                    writer.writerow(['{:.6f}'.format(row[0]),
                                   '{:.6f}'.format(row[1]),
                                   '{:.6f}'.format(row[2]),
                                   '{:.6f}'.format(row[3])])

            rospy.loginfo("轨迹已保存: %d 个点", len(points))
        except Exception as e:
            rospy.logerr("保存轨迹失败: %s", str(e))

if __name__ == '__main__':
    try:
        recorder = TrajectoryRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
