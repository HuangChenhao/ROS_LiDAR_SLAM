#!/usr/bin/env python3
"""Fail-fast health check for the R2 serial, IMU and odometry pipeline."""

import argparse
import json
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class HealthCollector(Node):
    def __init__(self):
        super().__init__('rosmaster_healthcheck')
        self.voltage = []
        self.edition = []
        self.accel_norm = []
        self.imu_count = 0
        self.odom_raw_count = 0
        self.odom_count = 0
        self.create_subscription(Float32, '/voltage', lambda m: self.voltage.append(float(m.data)), 20)
        self.create_subscription(Float32, '/edition', lambda m: self.edition.append(float(m.data)), 20)
        self.create_subscription(Imu, '/imu/data_raw', self.on_raw_imu, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.create_subscription(Odometry, '/odom_raw', self.on_odom_raw, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)

    def on_raw_imu(self, msg):
        a = msg.linear_acceleration
        self.accel_norm.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))

    def on_imu(self, _msg):
        self.imu_count += 1

    def on_odom_raw(self, _msg):
        self.odom_raw_count += 1

    def on_odom(self, _msg):
        self.odom_count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=4.0)
    args = parser.parse_args()

    rclpy.init()
    node = HealthCollector()
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    median_voltage = statistics.median(node.voltage) if node.voltage else 0.0
    median_accel = statistics.median(node.accel_norm) if node.accel_norm else 0.0
    edition = max(node.edition) if node.edition else 0.0
    minimum_samples = max(8, int(args.duration * 4.0))
    checks = {
        'voltage_valid': 8.0 <= median_voltage <= 15.0,
        'firmware_seen': edition > 0.0,
        'raw_imu_live': len(node.accel_norm) >= minimum_samples and 5.0 <= median_accel <= 15.0,
        'filtered_imu_live': node.imu_count >= minimum_samples,
        'odom_raw_live': node.odom_raw_count >= minimum_samples,
        'fused_odom_live': node.odom_count >= minimum_samples,
    }
    report = {
        'ok': all(checks.values()),
        'checks': checks,
        'samples': {
            'voltage': len(node.voltage),
            'edition': len(node.edition),
            'imu_raw': len(node.accel_norm),
            'imu': node.imu_count,
            'odom_raw': node.odom_raw_count,
            'odom': node.odom_count,
        },
        'median_voltage_v': round(median_voltage, 3),
        'firmware_version': edition,
        'median_acceleration_norm': round(median_accel, 3),
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if report['ok'] else 1)


if __name__ == '__main__':
    main()
