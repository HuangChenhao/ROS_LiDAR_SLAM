#!/usr/bin/env python3
# encoding: utf-8
# SLAM session supervisor for ROSMaster R2
#
# Listens to /MappingState (from patched joy_ctrl):
#   True  (active + gear 1/2) -> start lidar + selected SLAM, create session folder
#                                named by SCAN START time, autosave map every 30s
#   False (red/inactive or gear 3) -> save final map, stop lidar (motor off) + gmapping
#
# Sessions: /root/rosmaster_maps/<YYYYmmdd_HHMMSS_algorithm>/
# Autosave protects against hard power-off (last <=30s of mapping lost at most).

import os
import time
import signal
import subprocess
import shutil
import json

import numpy as np
import serial
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

ENV = ("source /opt/ros/foxy/setup.bash && "
       "source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && "
       "export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1 TZ=Europe/Berlin")
MAPS_DIR = '/root/rosmaster_maps'
PREVIEW_SEC = 0.5      # kiosk refresh poll — follows gmapping /map rate
AUTOSAVE_TICKS = 60    # full session autosave every ~30s
LIDAR_PORT = '/dev/rplidar'
MIN_START_FREE_BYTES = 2 * 1024 ** 3
MIN_RUNTIME_FREE_BYTES = 1 * 1024 ** 3
SCAN_START_TIMEOUT_TICKS = 30  # 15 seconds at PREVIEW_SEC=0.5
ALGORITHMS = ('gmapping', 'cartographer', 'slam_toolbox', 'rtabmap')
ALGO_DISPLAY = {
    'gmapping': 'GMapping',
    'cartographer': 'Cartographer',
    'slam_toolbox': 'SLAM Toolbox',
    'rtabmap': 'RTAB-Map 2D LiDAR',
}
LIVE_STATUS = os.path.join(MAPS_DIR, 'live_status.json')


class SlamSupervisor(Node):
    def __init__(self):
        super().__init__('slam_supervisor')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/MappingState', self.state_cb, qos)
        qos2 = QoSProfile(depth=1)
        qos2.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/SlamAlgo', self.algo_cb, qos2)
        self.algo = 'gmapping'
        self.active_algo = None
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.mapping = False
        self.session = None
        self.start_ts = None
        self.procs = []
        self.nodes_log = None
        self.last_map = None
        self.traj = []
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self._tick = 0
        self._map_dirty = False
        self.scan_count = 0
        self.startup_retries = 0
        self.create_timer(PREVIEW_SEC, self.tick)
        os.makedirs(MAPS_DIR, exist_ok=True)
        # kill any orphan lidar/gmapping from previous supervisor instance
        subprocess.call([
            'bash', '-c',
            "pkill -9 -f 'sllidar|slam_gmapping|cartographer|slam_toolbox|rtabmap' "
            "2>/dev/null; true"
        ])
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
        """Explicitly enable the A1 motor, then release the serial port."""
        if self.lidar_ser is not None:
            try:
                self.lidar_ser.dtr = False
                time.sleep(0.25)
                self.lidar_ser.close()
            except Exception:
                pass
            self.lidar_ser = None
            time.sleep(0.75)

    def algo_cb(self, msg):
        if msg.data in ALGORITHMS and msg.data != self.algo:
            self.algo = msg.data
            if self.mapping:
                self.get_logger().info('SLAM algo -> {} (applies to NEXT session)'.format(self.algo))
                self.write_live_status()
            else:
                self.get_logger().info('SLAM algo -> {}'.format(self.algo))

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

    def scan_cb(self, _msg):
        if self.mapping:
            self.scan_count += 1

    def write_live_status(self):
        if not self.mapping or self.active_algo is None:
            return
        status = {
            'algorithm': self.active_algo,
            'algorithm_display': ALGO_DISPLAY[self.active_algo],
            'selected_next': self.algo,
            'session': os.path.basename(self.session) if self.session else '',
            'scan_count': self.scan_count,
            'started': self.start_ts,
        }
        tmp = LIVE_STATUS + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(status, f, ensure_ascii=False)
        os.replace(tmp, LIVE_STATUS)

    def start_mapping(self, retry=False):
        free = shutil.disk_usage(MAPS_DIR).free
        if free < MIN_START_FREE_BYTES:
            self.get_logger().error(
                'mapping refused: only {:.2f} GiB free (need 2 GiB)'.format(free / 1024 ** 3))
            return
        self.start_ts = time.strftime('%Y%m%d_%H%M%S')
        self.active_algo = self.algo
        session_name = '{}_{}'.format(self.start_ts, self.active_algo)
        self.session = os.path.join(MAPS_DIR, session_name)
        os.makedirs(self.session, exist_ok=True)
        with open(os.path.join(self.session, 'metadata.txt'), 'w') as f:
            f.write(
                'session: {}\nscan_start: {}\nalgo: {}\nalgo_display: {}\n'.format(
                    session_name, self.start_ts, self.active_algo,
                    ALGO_DISPLAY[self.active_algo]))
        self.last_map = None
        self.scan_count = 0
        if not retry:
            self.startup_retries = 0
        self.lidar_release()
        self.nodes_log = open(os.path.join(self.session, 'nodes.log'), 'w')
        self.traj = []
        bag_path = os.path.join(self.session, 'bag')
        if self.active_algo == 'cartographer':
            slam_cmd = ('ros2 launch yahboomcar_nav cartographer_launch.py '
                        'configuration_basename:=rosmaster_carto.lua')
        elif self.active_algo == 'slam_toolbox':
            slam_cmd = (
                'ros2 run slam_toolbox async_slam_toolbox_node --ros-args '
                '--params-file /root/rosmaster_tools/slam_toolbox_r2.yaml')
        elif self.active_algo == 'rtabmap':
            slam_cmd = (
                'ros2 run rtabmap_slam rtabmap --ros-args '
                '--params-file /root/rosmaster_tools/rtabmap_r2.yaml '
                '-p database_path:={}/rtabmap.db'.format(self.session))
        else:
            slam_cmd = 'ros2 launch slam_gmapping slam_gmapping.launch.py'
        lidar_proc = subprocess.Popen(
            ['bash', '-c', ENV + ' && ros2 launch sllidar_ros2 sllidar_launch.py'],
            preexec_fn=os.setsid, stdout=self.nodes_log, stderr=subprocess.STDOUT)
        self.procs = [lidar_proc]
        # Give USB serial and the A1 motor time to settle before SLAM subscribes.
        time.sleep(2.0)
        self.procs.extend([
            subprocess.Popen(['bash', '-c', ENV + ' && ' + slam_cmd],
                              preexec_fn=os.setsid, stdout=self.nodes_log, stderr=subprocess.STDOUT),
            subprocess.Popen(['bash', '-c', ENV + ' && ros2 bag record -o {} '
                              '/scan /odom /odom_raw /tf /tf_static '
                              '/imu/data /imu/data_raw /cmd_vel '
                              '/MappingState /SlamAlgo /diagnostics'.format(bag_path)],
                              preexec_fn=os.setsid, stdout=self.nodes_log, stderr=subprocess.STDOUT),
        ])
        self.mapping = True
        self._tick = 0
        self.write_live_status()
        self.get_logger().info(
            'MAPPING START ({}) -> session {}'.format(self.active_algo, session_name))

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
        subprocess.call([
            'bash', '-c',
            "pkill -9 -f 'sllidar|slam_gmapping|cartographer|slam_toolbox|rtabmap' "
            "2>/dev/null; true"
        ])
        self.procs = []
        if self.nodes_log is not None:
            try:
                self.nodes_log.close()
            except Exception:
                pass
            self.nodes_log = None
        time.sleep(1)
        self.lidar_motor_off()
        for ext in ('ppm', 'pgm'):
            try:
                os.remove(os.path.join(MAPS_DIR, 'live_preview.' + ext))
            except OSError:
                pass
        try:
            os.remove(LIVE_STATUS)
        except OSError:
            pass
        if self.traj and self.session and os.path.isdir(self.session):
            try:
                with open(os.path.join(self.session, 'trajectory.csv'), 'w') as f:
                    f.write('x,y\n')
                    for (x, y) in self.traj:
                        f.write('{:.3f},{:.3f}\n'.format(x, y))
            except Exception:
                pass
        # Keep incomplete sessions: their rosbag and logs can recover or diagnose a failed map.
        if self.session and not os.path.exists(os.path.join(self.session, 'map.pgm')):
            try:
                with open(os.path.join(self.session, 'metadata.txt'), 'a') as f:
                    f.write('scan_end: {}\nstatus: incomplete_no_map\n'.format(
                        time.strftime('%Y%m%d_%H%M%S')))
            except Exception:
                pass
            self.get_logger().warn('incomplete session kept for recovery: {}'.format(self.session))
        self.active_algo = None
        self.get_logger().info('lidar + SLAM stopped, motor off')

    def tick(self):
        if not self.mapping:
            return
        self._tick += 1
        if self._tick % 2 == 0:
            self.write_live_status()
        if self._tick == SCAN_START_TIMEOUT_TICKS and self.scan_count == 0:
            if self.startup_retries < 1:
                self.startup_retries += 1
                self.get_logger().warn('no /scan after 15s — restarting LiDAR + SLAM once')
                self.stop_mapping()
                self.start_mapping(retry=True)
            else:
                self.get_logger().error('no /scan after retry — stopping mapping for safety')
                self.stop_mapping()
            return
        if self._tick % 20 == 0:
            free = shutil.disk_usage(MAPS_DIR).free
            if free < MIN_RUNTIME_FREE_BYTES:
                self.get_logger().error(
                    'disk safety stop: only {:.2f} GiB free'.format(free / 1024 ** 3))
                self.stop_mapping()
                return
        self.update_pose()
        if self._map_dirty or (self._tick % 2 == 0):  # re-render on new map or every 1s (traj moves)
            self._map_dirty = False
            self.write_preview()
        if self._tick % AUTOSAVE_TICKS == 0:
            self.save_map()

    def update_pose(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint', RclTime())
            x = t.transform.translation.x
            y = t.transform.translation.y
            if not self.traj or (x - self.traj[-1][0]) ** 2 + (y - self.traj[-1][1]) ** 2 > 0.0025:
                self.traj.append((x, y))
        except Exception:
            pass

    def render_preview(self, m, ppm_path):
        """Color preview: map + trajectory (red) + robot position (blue)."""
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = np.array(m.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[(data >= 0) & (data <= 25)] = 254   # free
        img[data >= 55] = 0                     # occupied (gmapping=100, carto>=55)
        img = np.flipud(img)
        rgb = np.stack([img, img, img], axis=-1)

        def px(x, y):
            c = int((x - ox) / res)
            r = h - 1 - int((y - oy) / res)
            return r, c

        for (x, y) in self.traj:
            r, c = px(x, y)
            if 0 <= r < h and 0 <= c < w:
                rgb[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = (220, 40, 40)
        if self.traj:
            r, c = px(*self.traj[-1])
            if 0 <= r < h and 0 <= c < w:
                rgb[max(0, r - 2):r + 3, max(0, c - 2):c + 3] = (0, 90, 255)

        tmp = ppm_path + '.tmp'
        with open(tmp, 'wb') as f:
            f.write('P6\n{} {}\n255\n'.format(w, h).encode())
            f.write(rgb.astype(np.uint8).tobytes())
        os.replace(tmp, ppm_path)

    def render_pgm(self, m, pgm_path):
        w, h = m.info.width, m.info.height
        data = np.array(m.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[(data >= 0) & (data <= 25)] = 254   # free
        img[data >= 55] = 0                     # occupied (gmapping=100, carto>=55)
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
            self.render_preview(self.last_map, os.path.join(MAPS_DIR, 'live_preview.ppm'))
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
        self.render_pgm(m, os.path.join(self.session, 'map.pgm'))
        yaml_path = os.path.join(self.session, 'map.yaml')
        yaml_tmp = yaml_path + '.tmp'
        with open(yaml_tmp, 'w') as f:
            f.write('image: map.pgm\nresolution: {}\norigin: [{}, {}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'.format(res, ox, oy))
        os.replace(yaml_tmp, yaml_path)
        if final:
            with open(os.path.join(self.session, 'metadata.txt'), 'a') as f:
                f.write('scan_end: {}\nstatus: complete\nmap: {}x{} res={}\n'.format(
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
