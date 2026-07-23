#!/usr/bin/env python3
"""Stable Ackermann odometry for ROSMaster R2.

The vendor node integrates the first velocity sample against an uninitialised
timestamp and accepts arbitrarily large callback gaps.  Both can create huge
pose jumps.  This replacement uses the measured forward speed and reported
steering angle, rejects invalid timing, and publishes realistic covariance.
"""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class RosmasterOdom(Node):
    def __init__(self):
        super().__init__('base_node')
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('steering_scale', 1.0)
        self.declare_parameter('max_dt', 0.25)
        self.declare_parameter('publish_tf', False)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.steering_scale = float(self.get_parameter('steering_scale').value)
        self.max_dt = float(self.get_parameter('max_dt').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_rx = None
        self.pub = self.create_publisher(Odometry, '/odom_raw', 50)
        self.sub = self.create_subscription(Twist, '/vel_raw', self.on_velocity, 50)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.get_logger().info('stable R2 odometry ready (wheelbase={:.3f} m)'.format(self.wheelbase))

    def on_velocity(self, msg):
        now_mono = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        speed = float(msg.linear.x) * self.linear_scale
        steering = math.radians(float(msg.linear.y) * self.steering_scale)
        steering = max(math.radians(-45.0), min(math.radians(45.0), steering))
        yaw_rate = speed * math.tan(steering) / self.wheelbase

        dt = None if self.last_rx is None else now_mono - self.last_rx
        self.last_rx = now_mono
        if dt is not None and 0.0 < dt <= self.max_dt:
            delta_yaw = yaw_rate * dt
            if abs(yaw_rate) > 1.0e-6:
                radius = speed / yaw_rate
                self.x += radius * (math.sin(self.yaw + delta_yaw) - math.sin(self.yaw))
                self.y -= radius * (math.cos(self.yaw + delta_yaw) - math.cos(self.yaw))
            else:
                self.x += speed * math.cos(self.yaw) * dt
                self.y += speed * math.sin(self.yaw) * dt
            self.yaw = math.atan2(math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))
        elif dt is not None and dt > self.max_dt:
            self.get_logger().warn(
                'velocity gap {:.3f}s rejected (limit {:.3f}s)'.format(dt, self.max_dt))

        half = 0.5 * self.yaw
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = 1.0e6
        odom.pose.covariance[21] = 1.0e6
        odom.pose.covariance[28] = 1.0e6
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 1.0e6
        odom.twist.covariance[14] = 1.0e6
        odom.twist.covariance[21] = 1.0e6
        odom.twist.covariance[28] = 1.0e6
        odom.twist.covariance[35] = 0.05
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = odom.child_frame_id
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = RosmasterOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
