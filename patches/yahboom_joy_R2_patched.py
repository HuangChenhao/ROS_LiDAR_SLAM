#!/usr/bin/env python
# encoding: utf-8
# Patched for Flydigi Direwolf 3 + R2 Ackermann + LED gear indicator

import os
import time
import getpass
import threading
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy

# LED effects: 0=off, 1=flowing, 2=marquee, 3=breathing, 4=gradient, 5=starlight, 6=battery
# 7 = solid red (custom, patched into Ackman_driver_R2.py) — means joy control INACTIVE
GEAR_LED = {
    1: (3, 'breathing'),    # 0.17 m/s — mapping (calm breathing)
    2: (1, 'flowing'),      # 0.33 m/s — mapping fast (flowing)
    3: (2, 'marquee'),      # MAX speed (marquee)
}
INACTIVE_LED = 7  # solid red

class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        self.Joy_active = False
        self.Buzzer_active = False
        self.RGBLight_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_Gear_idx = 1  # 1=slow, 2=med, 3=fast
        self.angular_Gear = 1.0 / 4  # default lowest steering sensitivity
        self.racing = False
        self.racing_time = time.time()
        self.slam_algo = 'gmapping'  # or 'cartographer', toggled by Y
        self.algo_time = time.time()

        self.pub_goal = self.create_publisher(GoalID, "move_base/cancel", 10)
        self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_Buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight = self.create_publisher(Int32, "RGBLight", 10)
        _qos = QoSProfile(depth=1)
        _qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_MapState = self.create_publisher(Bool, "MappingState", _qos)
        _qos2 = QoSProfile(depth=1)
        _qos2.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_SlamAlgo = self.create_publisher(String, "SlamAlgo", _qos2)

        self.sub_Joy = self.create_subscription(Joy, 'joy', self.buttonCallback, 1)

        self.declare_parameter('xspeed_limit', 1.0)
        self.declare_parameter('yspeed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)
        self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
        self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

        # Gear speeds (absolute m/s): 1=mapping, 2=mapping-fast, 3=transit (NOT for mapping)
        self.gear_map = {1: 0.17, 2: 0.33, 3: self.xspeed_limit}

        # Inactive at startup -> solid red LED (retry a few times, driver may still be starting)
        self._init_led_count = 0
        self._init_led_timer = self.create_timer(2.0, self._init_led_cb)
        self.update_mapping_state()  # boot default: OFF (no lidar, no mapping)
        self.publish_algo(blink=False)
        self.get_logger().info('R2 Joy started: xspeed={}, angular={}, INACTIVE (red LED)'.format(
            self.xspeed_limit, self.angular_speed_limit))

    def publish_algo(self, blink=True):
        msg = String()
        msg.data = self.slam_algo
        self.pub_SlamAlgo.publish(msg)
        self.get_logger().info('SLAM algo -> {}'.format(self.slam_algo))
        if blink:
            b = Int32()
            b.data = 8 if self.slam_algo == 'gmapping' else 9
            self.pub_RGBLight.publish(b)
            # after blink (~1.6s in driver), restore current LED state
            m = Int32()
            if not self.Joy_active:
                m.data = 7  # red
            elif self.racing:
                m.data = 5  # starlight
            else:
                m.data = GEAR_LED.get(self.linear_Gear_idx, (3, ''))[0]
            self.pub_RGBLight.publish(m)

    def update_mapping_state(self):
        """Mapping ON when active and NOT racing. Red/inactive or racing -> OFF (lidar stops)."""
        state = bool(self.Joy_active and not self.racing)
        msg = Bool()
        msg.data = state
        self.pub_MapState.publish(msg)
        self.get_logger().info('MappingState -> {}'.format(state))

    def _init_led_cb(self):
        """Republish inactive LED a few times at startup, then stop"""
        if not self.Joy_active:
            self.set_inactive_led(log=False)
        self._init_led_count += 1
        if self._init_led_count >= 5:
            self._init_led_timer.cancel()

    def set_inactive_led(self, log=True):
        """Solid red = joy control inactive"""
        msg = Int32()
        msg.data = INACTIVE_LED
        for _ in range(3):
            self.pub_RGBLight.publish(msg)
        if log:
            self.get_logger().info('Joy INACTIVE -> LED solid red')

    def set_gear_led(self, gear_idx):
        """Set LED effect to match current gear"""
        effect, name = GEAR_LED.get(gear_idx, (3, 'breathing'))
        msg = Int32()
        msg.data = effect
        for _ in range(3):
            self.pub_RGBLight.publish(msg)
        self.get_logger().info('Gear {}/3 -> LED: {} (effect {})'.format(gear_idx, name, effect))

    def btn(self, joy_data, idx):
        if idx < len(joy_data.buttons):
            return joy_data.buttons[idx]
        return 0

    def ax(self, joy_data, idx):
        if idx < len(joy_data.axes):
            return joy_data.axes[idx]
        return 0.0

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        self.user_r2(joy_data)

    def user_r2(self, joy_data):
        """R2 Ackermann joy control

        Xbox 360 mapping (Flydigi X-input):
          axes[1] = left stick Y (forward/back)
          axes[3] = right stick X (steering)
          buttons[0]=A, [1]=B, [2]=X, [3]=Y
          buttons[4]=LB, [5]=RB, [6]=Back, [7]=Start
          buttons[8]=Guide, [9]=L3, [10]=R3
        """
        # Back (6) = toggle joy active
        if self.btn(joy_data, 6) == 1:
            self.cancel_nav()

        # Start (7) = cycle LED manually (override gear LED)
        if self.btn(joy_data, 7) == 1:
            self.RGBLight_index = (self.RGBLight_index + 1) % 7
            msg = Int32()
            msg.data = self.RGBLight_index
            for _ in range(3):
                self.pub_RGBLight.publish(msg)

        # Y (3) = toggle SLAM algorithm: blue blink x3 = gmapping, yellow x3 = cartographer
        if self.btn(joy_data, 3) == 1:
            now = time.time()
            if now - self.algo_time > 1.5:
                self.algo_time = now
                self.slam_algo = 'cartographer' if self.slam_algo == 'gmapping' else 'gmapping'
                self.publish_algo(blink=True)

        # X (2) = racing mode toggle: max speed + starlight LED + mapping OFF
        if self.btn(joy_data, 2) == 1:
            now = time.time()
            if self.Joy_active and now - self.racing_time > 1:
                self.racing_time = now
                self.racing = not self.racing
                if self.racing:
                    msg = Int32()
                    msg.data = 5  # starlight
                    for _ in range(3):
                        self.pub_RGBLight.publish(msg)
                    self.get_logger().info('RACING ON: max speed, starlight LED, mapping OFF')
                else:
                    self.set_gear_led(self.linear_Gear_idx)
                    self.get_logger().info('RACING OFF: back to gear {}/3'.format(self.linear_Gear_idx))
                self.update_mapping_state()

        # B (1) = buzzer toggle
        if self.btn(joy_data, 1) == 1:
            Buzzer_ctrl = Bool()
            self.Buzzer_active = not self.Buzzer_active
            Buzzer_ctrl.data = self.Buzzer_active
            for _ in range(3):
                self.pub_Buzzer.publish(Buzzer_ctrl)

        # LB (4) = cycle speed gear (1/3 -> 2/3 -> 3/3) + auto LED (only when active)
        if self.btn(joy_data, 4) == 1:
            if self.racing:
                self.racing = False
                self.get_logger().info('RACING OFF (gear change)')
            self.linear_Gear_idx = (self.linear_Gear_idx % 3) + 1
            if self.Joy_active:
                self.set_gear_led(self.linear_Gear_idx)
            else:
                self.get_logger().info('Gear {}/3 (inactive, LED stays red)'.format(self.linear_Gear_idx))
            self.update_mapping_state()

        # RB (5) = angular gear cycle
        if self.btn(joy_data, 5) == 1:
            if self.angular_Gear == 1.0:
                self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4:
                self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2:
                self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4:
                self.angular_Gear = 1.0
            self.get_logger().info('Angular gear: {:.2f}'.format(self.angular_Gear))

        # R2 Ackermann: left stick Y = speed, right stick X = steering
        if self.racing:
            gear_speed = self.xspeed_limit
        else:
            gear_speed = min(self.gear_map.get(self.linear_Gear_idx, 0.17), self.xspeed_limit)
        xlinear_speed = self.filter_data(self.ax(joy_data, 1)) * gear_speed
        angular_speed = self.filter_data(self.ax(joy_data, 3)) * self.angular_speed_limit * self.angular_Gear

        xlinear_speed = max(-self.xspeed_limit, min(self.xspeed_limit, xlinear_speed))
        angular_speed = max(-self.angular_speed_limit, min(self.angular_speed_limit, angular_speed))

        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.angular.z = angular_speed

        if self.Joy_active:
            for _ in range(3):
                self.pub_cmdVel.publish(twist)

    def filter_data(self, value):
        if abs(value) < 0.2:
            value = 0
        return value

    def cancel_nav(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            Joy_ctrl = Bool()
            self.Joy_active = not self.Joy_active
            if not self.Joy_active:
                self.racing = False
            Joy_ctrl.data = self.Joy_active
            self.get_logger().info('Joy active: {}'.format(self.Joy_active))
            for _ in range(3):
                self.pub_JoyState.publish(Joy_ctrl)
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = now_time
            # LED reflects state: active -> gear LED, inactive -> solid red
            if self.Joy_active:
                self.set_gear_led(self.linear_Gear_idx)
            else:
                self.set_inactive_led()
            self.update_mapping_state()

def main():
    rclpy.init()
    joy_ctrl = JoyTeleop('joy_ctrl')
    rclpy.spin(joy_ctrl)
