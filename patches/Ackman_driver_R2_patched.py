#!/usr/bin/env python
# encoding: utf-8

#public lib
import sys
import math
import os
import time
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from rclpy.clock import Clock

#from dynamic_reconfigure.server import Server
car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		self.car = Rosmaster()
		self.car.set_car_type(5)
		#get parameter
		self.declare_parameter('car_type', 'R2')
		self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
		print (self.car_type)
		self.declare_parameter('imu_link', 'imu_link')
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('angular_limit', 1.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)
		self.declare_parameter('nav_use_rotvel', False)
		self.nav_use_rotvel = self.get_parameter('nav_use_rotvel').get_parameter_value().bool_value
		print (self.nav_use_rotvel)

		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

		#create publisher
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.volPublisher = self.create_publisher(Float32,"voltage",100)
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"imu/data_raw",100)

		#create timer
		self.timer = self.create_timer(0.1, self.pub_data)

		#create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		self.car.create_receive_threading()
		self._started_at = time.monotonic()
		self._last_cmd_at = self._started_at
		self._last_telemetry_change = self._started_at
		self._last_telemetry = None
		self._watchdog_stopped = False
		self._wheel_position = 0.0
		self._last_publish_at = self._started_at
		self._edition_publish_tick = 0
		self._steady_led = 7
		self._led_blink_color = None
		self._led_blink_phase = -1
		self._led_next_step = 0.0
	#callback function
	def cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x*1.0
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y*1.0
		angular = msg.angular.z*1.0    # wait for change
		self.car.set_car_motion(vx, vy, angular)
		self._last_cmd_at = time.monotonic()
		self._watchdog_stopped = False
		# self.car.set_car_motion(vx*1.8, vy, angular)
		# self.get_logger().info("cmd_vel_callback: vx = {}, vy = {}, angular = {},".format(vx, vy, angular))
        #print(self.nav_use_rotvel)
	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		code = int(msg.data)
		if code in (8, 9, 10, 11):
			self._led_blink_color = {
				8: (0, 80, 255),
				9: (255, 160, 0),
				10: (0, 255, 80),
				11: (200, 0, 255),
			}[code]
			self._led_blink_phase = -1
			self._led_next_step = time.monotonic()
			return

		# A normal state/gear LED immediately cancels an in-progress reminder.
		self._led_blink_color = None
		self._steady_led = code
		self._apply_steady_led(code)

	def _apply_steady_led(self, code):
		if code == 7:
			for _ in range(3): self.car.set_colorful_effect(0, 6, parm=1)
			time.sleep(0.1)
			for _ in range(3): self.car.set_colorful_lamps(0xFF, 255, 0, 0)
		else:
			for _ in range(3): self.car.set_colorful_effect(code, 6, parm=1)

	def _update_algorithm_led(self, now_mono):
		"""Advance one reminder phase without sleeping or concurrent serial I/O."""
		if self._led_blink_color is None or now_mono < self._led_next_step:
			return
		if self._led_blink_phase == -1:
			for _ in range(3): self.car.set_colorful_effect(0, 6, parm=1)
			self._led_blink_phase = 0
			self._led_next_step = now_mono + 0.10
		elif self._led_blink_phase < 6:
			if self._led_blink_phase % 2 == 0:
				self.car.set_colorful_lamps(0xFF, *self._led_blink_color)
				self._led_next_step = now_mono + 0.28
			else:
				self.car.set_colorful_lamps(0xFF, 0, 0, 0)
				self._led_next_step = now_mono + 0.18
			self._led_blink_phase += 1
		else:
			self._led_blink_color = None
			self._apply_steady_led(self._steady_led)
	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	#pub data
	def pub_data(self):
		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		if len(self.Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
		
		#print ("mag: ",self.car.get_magnetometer_data())		
		edition.data = self.car.get_version()*1.0
		battery.data = self.car.get_battery_voltage()*1.0
		ax, ay, az = self.car.get_accelerometer_data()
		# self.get_logger().info("ax = {}, ay = {}, az = {} ".format(ax,ay,az))
		gx, gy, gz = self.car.get_gyroscope_data()
		# self.get_logger().info("gx = {}, gy = {}, gz = {} ".format(gx,gy,gz))
		vx, vy, angular = self.car.get_motion_data()
		now_mono = time.monotonic()
		self._update_algorithm_led(now_mono)
		telemetry = (battery.data, ax, ay, az, gx, gy, gz, vx, vy, angular)
		if telemetry != self._last_telemetry:
			self._last_telemetry = telemetry
			self._last_telemetry_change = now_mono
		accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
		if now_mono - self._started_at > 3.0:
			if battery.data < 1.0 or accel_norm < 2.0:
				self.get_logger().fatal(
					'no valid serial telemetry (voltage={:.1f}, accel_norm={:.2f})'.format(
						battery.data, accel_norm))
				self.car.set_car_motion(0.0, 0.0, 0.0)
				os._exit(2)
			if now_mono - self._last_telemetry_change > 2.5:
				self.get_logger().fatal('serial telemetry stopped changing for 2.5s')
				self.car.set_car_motion(0.0, 0.0, 0.0)
				os._exit(3)
		if now_mono - self._last_cmd_at > 0.5 and not self._watchdog_stopped:
			self.car.set_car_motion(0.0, 0.0, 0.0)
			self._watchdog_stopped = True
		# self.get_logger().info("get_motion_data: vx = {}, vy = {}, angular = {},".format(vx, vy, angular))
		
		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.orientation.w = 1.0
		imu.orientation_covariance[0] = -1.0
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		imu.angular_velocity.z = gz*1.0
		for i in (0, 4, 8):
			imu.angular_velocity_covariance[i] = 0.0004
			imu.linear_acceleration_covariance[i] = 0.04

		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx*1.0    #velocity in axis 
		twist.linear.y = vy*1000*1.0   #steer angle
		#twist.linear.y = vy   #steer angle
		#twist.angular.z = angular
		twist.angular.z = angular*1.0    #this is invalued
		self.velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rospy.loginfo("battery: {}".format(battery))
		# rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
		self.imuPublisher.publish(imu)
		self.volPublisher.publish(battery)
		self._edition_publish_tick += 1
		if self._edition_publish_tick >= 10:
			self._edition_publish_tick = 0
			self.EdiPublisher.publish(edition)
		
		#turn to radis
		steer_radis = vy*1000.0*3.1416/180.0
		dt = max(0.0, min(0.2, now_mono - self._last_publish_at))
		self._last_publish_at = now_mono
		wheel_speed = vx / 0.0325
		self._wheel_position = math.atan2(
			math.sin(self._wheel_position + wheel_speed * dt),
			math.cos(self._wheel_position + wheel_speed * dt))
		state.position = [self._wheel_position, self._wheel_position,
			steer_radis, self._wheel_position, steer_radis, self._wheel_position]
		state.velocity = [wheel_speed, wheel_speed, 0.0, wheel_speed, 0.0, wheel_speed]
		self.staPublisher.publish(state)
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

'''if __name__ == '__main__':
	main()'''

		
		
