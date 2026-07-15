#!/usr/bin/env python3
# encoding: utf-8
# SLAM session supervisor for ROSMaster R2
#
# Listens to /MappingState (from patched joy_ctrl):
#   True  (active + gear 1/2) -> start lidar + gmapping, create session folder
#                                named by SCAN START time, autosave map every 30s
#   False (red/inactive or gear 3) -> save final map, stop lidar (motor off) + gmapping
#
# Sessions: /root/rosmaster_maps/<YYYYmmdd_HHMMSS>/  (start-time named)
# Autosave protects against hard power-off (last <=30s of mapping lost at most).

import os
import time
import signal
import subprocess

import numpy as np
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid

ENV = ("source /opt/ros/foxy/setup.bash && "
       "source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && "
       "export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1")
MAPS_DIR = '/root/rosmaster_maps'
PREVIEW_SEC = 0.5      # kiosk refresh poll — follows gmapping /map rate
AUTOSAVE_TICKS = 60    # full session autosave every ~30s
LIDAR_PORT = '/dev/rplidar'


class SlamSupervisor(Node):
    def __init__(self):
        super().__init__('slam_supervisor')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/MappingState', self.state_cb, qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.mapping = False
        self.session = None
        self.start_ts = None
        self.procs = []
        self.last_map = None
        self._tick = 0
        self._map_dirty = False
        self.create_timer(PREVIEW_SEC, self.tick)
        os.makedirs(MAPS_DIR, exist_ok=True)
        # kill any orphan lidar/gmapping from previous supervisor instance
        subprocess.call(['bash', '-c', "pkill -9 -f 'sllidar|slam_gmapping' 2>/dev/null; true"])
        time.sleep(1)
        self.lidar_ser = None
        self.lidar_motor_off()
        self.get_logger().info('slam_supervisor ready — mapping OFF (lidar motor off)')

    def lidar_motor_off(self):
        """A1 motor is controlled by serial DTR: DTR high = motor OFF.
        Keep the port open to hold DTR, otherwise the motor restarts."""
        if self.lidar_ser is not None:
            return
        try:
            self.lidar_ser = serial.Serial(LIDAR_PORT, 115200)
            self.lidar_ser.dtr = True
            self.get_logger().info('lidar motor OFF (DTR held)')
        except Exception as e:
            self.lidar_ser = None
            self.get_logger().warn('cannot stop lidar motor: {}'.format(e))

    def lidar_release(self):
        """Release the serial port so sllidar_node can open it."""
        if self.lidar_ser is not None:
            try:
                self.lidar_ser.close()
            except Exception:
                pass
            self.lidar_ser = None
            time.sleep(0.5)

    def state_cb(self, msg):
        self.get_logger().info('MappingState received: {} (mapping={})'.format(msg.data, self.mapping))
        if msg.data and not self.mapping:
            self.start_mapping()
        elif not msg.data and self.mapping:
            self.stop_mapping()

    def map_cb(self, msg):
        if self.mapping:
            self.last_map = msg
            self._map_dirty = True

    def start_mapping(self):
        self.start_ts = time.strftime('%Y%m%d_%H%M%S')
        self.session = os.path.join(MAPS_DIR, self.start_ts)
        os.makedirs(self.session, exist_ok=True)
        with open(os.path.join(self.session, 'metadata.txt'), 'w') as f:
            f.write('scan_start: {}\n'.format(self.start_ts))
        self.last_map = None
        self.lidar_release()
        log = open(os.path.join(self.session, 'nodes.log'), 'w')
        self.procs = [
            subprocess.Popen(['bash', '-c', ENV + ' && ros2 launch sllidar_ros2 sllidar_launch.py'],
                             preexec_fn=os.setsid, stdout=log, stderr=subprocess.STDOUT),
            subprocess.Popen(['bash', '-c', ENV + ' && ros2 launch slam_gmapping slam_gmapping.launch.py'],
                             preexec_fn=os.setsid, stdout=log, stderr=subprocess.STDOUT),
        ]
        self.mapping = True
        self._tick = 0
        self.get_logger().info('MAPPING START -> session {}'.format(self.start_ts))

    def stop_mapping(self):
        self.get_logger().info('MAPPING STOP -> saving final map...')
        self.mapping = False
        self.save_map(final=True)
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGINT)   # graceful: lidar motor stop
            except Exception:
                pass
        time.sleep(4)
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except Exception:
                pass
        subprocess.call(['bash', '-c', "pkill -9 -f 'sllidar|slam_gmapping' 2>/dev/null; true"])
        self.procs = []
        time.sleep(1)
        self.lidar_motor_off()
        try:
            os.remove(os.path.join(MAPS_DIR, 'live_preview.pgm'))
        except OSError:
            pass
        self.get_logger().info('lidar + gmapping stopped, motor off')

    def tick(self):
        if not self.mapping:
            return
        self._tick += 1
        if self._map_dirty:  # only re-render when gmapping published a new map
            self._map_dirty = False
            self.write_preview()
        if self._tick % AUTOSAVE_TICKS == 0:
            self.save_map()

    def render_pgm(self, m, pgm_path):
        w, h = m.info.width, m.info.height
        data = np.array(m.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data == 100] = 0
        img = np.flipud(img)
        tmp = pgm_path + '.tmp'
        with open(tmp, 'wb') as f:
            f.write('P5\n{} {}\n255\n'.format(w, h).encode())
            f.write(img.tobytes())
        os.replace(tmp, pgm_path)

    def write_preview(self):
        if self.last_map is None:
            return
        try:
            self.render_pgm(self.last_map, os.path.join(MAPS_DIR, 'live_preview.pgm'))
        except Exception as e:
            self.get_logger().warn('preview failed: {}'.format(e))

    def save_map(self, final=False):
        m = self.last_map
        if m is None or self.session is None:
            if final:
                self.get_logger().warn('no /map received this session — nothing to save')
            return
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = np.array(m.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data == 100] = 0
        img = np.flipud(img)
        with open(os.path.join(self.session, 'map.pgm'), 'wb') as f:
            f.write('P5\n{} {}\n255\n'.format(w, h).encode())
            f.write(img.tobytes())
        with open(os.path.join(self.session, 'map.yaml'), 'w') as f:
            f.write('image: map.pgm\nresolution: {}\norigin: [{}, {}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'.format(res, ox, oy))
        if final:
            with open(os.path.join(self.session, 'metadata.txt'), 'a') as f:
                f.write('scan_end: {}\nmap: {}x{} res={}\n'.format(
                    time.strftime('%Y%m%d_%H%M%S'), w, h, res))
            self.get_logger().info('final map saved: {}x{}'.format(w, h))


def main():
    rclpy.init()
    node = SlamSupervisor()
    try:
        rclpy.spin(node)
    finally:
        if node.mapping:
            node.stop_mapping()


if __name__ == '__main__':
    main()
