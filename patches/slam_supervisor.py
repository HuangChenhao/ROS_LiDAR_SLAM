#!/usr/bin/env python3
# encoding: utf-8
# SLAM session supervisor for ROSMaster R2
#
# Listens to /MappingState (from patched joy_ctrl):
#   True  (active + gear 1/2) -> start lidar + selected SLAM, create session folder
#                                named by SCAN START time, autosave map every 30s
#   False (red/inactive or gear 3) -> save final map, stop lidar (motor off) + gmapping
#
# Sessions: /root/rosmaster_maps/<YYYYmmdd_HHMMSS_algorithm_gear>/
# Autosave protects against hard power-off (last <=30s of mapping lost at most).

import os
import time
import signal
import subprocess
import shutil
import json
import math

import numpy as np
import serial
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Int32, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from visualization_msgs.msg import MarkerArray

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
SENSOR_TIMEOUT_SEC = 2.5
MAP_STALL_TIMEOUT_SEC = 10.0
MAX_POSE_STEP_M = 1.0
FINAL_OPTIMIZE_SEC = 4.0
START_SETTLE_SEC = 0.5  # collect algo + gear before honoring MappingState=true
ALGORITHMS = ('gmapping', 'cartographer', 'slam_toolbox', 'rtabmap')
ALGO_DISPLAY = {
    'gmapping': 'GMapping',
    'cartographer': 'Cartographer',
    'slam_toolbox': 'SLAM Toolbox',
    'rtabmap': 'RTAB-Map 2D LiDAR',
}
LOOP_CLOSURE_MODE = {
    'gmapping': 'implicit_particle_scan_to_map',
    'cartographer': 'explicit_pose_graph',
    'slam_toolbox': 'explicit_pose_graph',
    'rtabmap': 'explicit_pose_graph_icp',
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
        qos3 = QoSProfile(depth=1)
        qos3.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Int32, '/SpeedGear', self.gear_cb, qos3)
        self.algo = 'gmapping'
        self.gear = 0
        self.session_gear = 0
        self.gear_events = []
        self.active_algo = None
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Float32, '/voltage', self.voltage_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Path, '/rtabmap/mapPath', self.rtab_path_cb, 1)
        self.create_subscription(
            MarkerArray, '/trajectory_node_list', self.carto_markers_cb, 1)
        self.create_subscription(
            MarkerArray, '/slam_toolbox/graph_visualization',
            self.toolbox_markers_cb, 1)
        self.mapping = False
        self.session = None
        self.start_ts = None
        self.procs = []
        self.slam_proc = None
        self.nodes_log = None
        self.last_map = None
        self.traj = []
        self.optimized_traj = []
        self.graph_segments = []
        self.trajectory_source = 'live_tf'
        self.pose_frame = ''
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self._tick = 0
        self._map_dirty = False
        self.scan_count = 0
        self.last_odom_rx = 0.0
        self.last_imu_rx = 0.0
        self.last_voltage_rx = 0.0
        self.last_map_rx = 0.0
        self.last_motion_rx = 0.0
        self.command_speed = 0.0
        self.start_mono = 0.0
        self.voltage = 0.0
        self.pose_jump_count = 0
        self.stopping = False
        self.stop_deadline = 0.0
        self.stop_reason = 'requested'
        self.start_pending_since = None
        self.startup_retries = 0
        self.create_timer(PREVIEW_SEC, self.tick)
        os.makedirs(MAPS_DIR, exist_ok=True)
        # kill any orphan lidar/gmapping from previous supervisor instance
        subprocess.call([
            'bash', '-c',
            "pkill -9 -f 'ros2 (launch sllidar_ros2|launch slam_gmapping|"
            "launch yahboomcar_nav cartographer_launch.py|run slam_toolbox|"
            "run rtabmap_slam)|/opt/ros/foxy/lib/(sllidar_ros2|slam_gmapping|"
            "cartographer_ros|slam_toolbox|rtabmap_slam)/' "
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

    def gear_cb(self, msg):
        new_gear = int(msg.data)
        if new_gear not in (1, 2, 3, 4):
            return
        changed = new_gear != self.gear
        self.gear = new_gear
        if changed and self.mapping:
            elapsed = max(0.0, time.monotonic() - self.start_mono)
            self.gear_events.append((elapsed, new_gear))
            self.append_metadata(
                'gear_event: {:.3f}s,{}\n'.format(elapsed, self.gear_display(new_gear)))
            self.write_live_status()

    @staticmethod
    def gear_display(gear):
        return 'racing' if gear == 4 else ('gear{}'.format(gear) if gear else 'unknown')

    def state_cb(self, msg):
        self.get_logger().info('MappingState received: {} (mapping={})'.format(msg.data, self.mapping))
        if msg.data and not self.mapping:
            # Algorithm, gear, and mapping state are independent DDS topics;
            # their cross-topic delivery order is not defined.  Debounce the
            # start so the latest algorithm and gear label the real process.
            if self.start_pending_since is None:
                self.start_pending_since = time.monotonic()
        elif not msg.data:
            self.start_pending_since = None
            if self.mapping:
                self.stop_mapping(reason='requested')

    def map_cb(self, msg):
        if self.mapping:
            self.last_map = msg
            self.last_map_rx = time.monotonic()
            self._map_dirty = True

    def scan_cb(self, _msg):
        if self.mapping:
            self.scan_count += 1

    def odom_cb(self, _msg):
        self.last_odom_rx = time.monotonic()

    def imu_cb(self, _msg):
        self.last_imu_rx = time.monotonic()

    def voltage_cb(self, msg):
        self.last_voltage_rx = time.monotonic()
        self.voltage = float(msg.data)

    def cmd_vel_cb(self, msg):
        self.command_speed = abs(float(msg.linear.x))
        if self.command_speed > 0.04:
            self.last_motion_rx = time.monotonic()

    def rtab_path_cb(self, msg):
        if not self.mapping or self.active_algo != 'rtabmap':
            return
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if len(points) >= 2:
            self.optimized_traj = points
            self.graph_segments = list(zip(points, points[1:]))
            self.trajectory_source = 'rtabmap_optimized_path'

    @staticmethod
    def marker_points(marker):
        q = marker.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)
        tx, ty = marker.pose.position.x, marker.pose.position.y
        return [
            (tx + cy * point.x - sy * point.y,
             ty + sy * point.x + cy * point.y)
            for point in marker.points
        ]

    def update_marker_graph(self, msg, source):
        segments = []
        longest_path = []
        for marker in msg.markers:
            if marker.action in (2, 3):  # DELETE / DELETEALL
                continue
            points = self.marker_points(marker)
            if marker.type == 4 and len(points) >= 2:  # LINE_STRIP
                segments.extend(zip(points, points[1:]))
                if len(points) > len(longest_path):
                    longest_path = points
            elif marker.type == 5 and len(points) >= 2:  # LINE_LIST
                segments.extend(
                    (points[index], points[index + 1])
                    for index in range(0, len(points) - 1, 2))
        if segments:
            self.graph_segments = list(segments)
            self.optimized_traj = longest_path
            self.trajectory_source = source

    def carto_markers_cb(self, msg):
        if self.mapping and self.active_algo == 'cartographer':
            self.update_marker_graph(msg, 'cartographer_pose_graph')

    def toolbox_markers_cb(self, msg):
        if self.mapping and self.active_algo == 'slam_toolbox':
            self.update_marker_graph(msg, 'slam_toolbox_pose_graph')

    def sensors_healthy(self):
        now = time.monotonic()
        return (
            8.0 <= self.voltage <= 15.0
            and now - self.last_voltage_rx <= SENSOR_TIMEOUT_SEC
            and now - self.last_odom_rx <= SENSOR_TIMEOUT_SEC
            and now - self.last_imu_rx <= SENSOR_TIMEOUT_SEC
        )

    def append_metadata(self, text):
        """Durably append session metadata so power loss cannot leave NUL holes."""
        if not self.session:
            return
        path = os.path.join(self.session, 'metadata.txt')
        try:
            with open(path, 'a') as stream:
                stream.write(text)
                stream.flush()
                os.fsync(stream.fileno())
        except Exception as error:
            self.get_logger().warn('metadata append failed: {}'.format(error))

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
            'gear': self.gear,
            'gear_display': self.gear_display(self.gear),
            'sensors_ok': self.sensors_healthy(),
            'voltage': round(self.voltage, 1),
            'pose_jumps': self.pose_jump_count,
            'loop_closure': LOOP_CLOSURE_MODE[self.active_algo],
            'trajectory_source': self.trajectory_source,
            'slam_alive': bool(self.slam_proc and self.slam_proc.poll() is None),
            'finalizing': self.stopping,
        }
        tmp = LIVE_STATUS + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(status, f, ensure_ascii=False)
        os.replace(tmp, LIVE_STATUS)

    def start_mapping(self, retry=False):
        self.start_pending_since = None
        free = shutil.disk_usage(MAPS_DIR).free
        if free < MIN_START_FREE_BYTES:
            self.get_logger().error(
                'mapping refused: only {:.2f} GiB free (need 2 GiB)'.format(free / 1024 ** 3))
            return
        self.start_ts = time.strftime('%Y%m%d_%H%M%S')
        self.active_algo = self.algo
        self.session_gear = self.gear
        session_name = '{}_{}_{}'.format(
            self.start_ts, self.active_algo, self.gear_display(self.session_gear))
        self.session = os.path.join(MAPS_DIR, session_name)
        os.makedirs(self.session, exist_ok=True)
        metadata_path = os.path.join(self.session, 'metadata.txt')
        metadata_tmp = metadata_path + '.tmp'
        with open(metadata_tmp, 'w') as f:
            f.write(
                'session: {}\nscan_start: {}\nalgo: {}\nalgo_display: {}\n'
                'loop_closure: {}\nstart_gear: {}\n'.format(
                    session_name, self.start_ts, self.active_algo,
                    ALGO_DISPLAY[self.active_algo],
                    LOOP_CLOSURE_MODE[self.active_algo],
                    self.gear_display(self.session_gear)))
            f.flush()
            os.fsync(f.fileno())
        os.replace(metadata_tmp, metadata_path)
        self.last_map = None
        self.last_map_rx = 0.0
        self.scan_count = 0
        self.pose_jump_count = 0
        self.stopping = False
        self.stop_deadline = 0.0
        if not retry:
            self.startup_retries = 0
        self.lidar_release()
        self.nodes_log = open(os.path.join(self.session, 'nodes.log'), 'w')
        self.traj = []
        self.optimized_traj = []
        self.graph_segments = []
        self.trajectory_source = 'live_tf'
        self.pose_frame = ''
        self.gear_events = [(0.0, self.session_gear)]
        self.start_mono = time.monotonic()
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
            start_new_session=True, stdout=self.nodes_log, stderr=subprocess.STDOUT)
        self.procs = [lidar_proc]
        # Give USB serial and the A1 motor time to settle before SLAM subscribes.
        time.sleep(2.0)
        self.procs.extend([
            subprocess.Popen(['bash', '-c', ENV + ' && ' + slam_cmd],
                              start_new_session=True, stdout=self.nodes_log,
                              stderr=subprocess.STDOUT),
            subprocess.Popen(['bash', '-c', ENV + ' && ros2 bag record -o {} '
                              '/scan /odom /odom_raw /tf /tf_static '
                              '/imu/data /imu/data_raw /vel_raw /cmd_vel '
                              '/voltage /edition '
                              '/MappingState /SlamAlgo /SpeedGear /diagnostics '
                              '/trajectory_node_list '
                              '/slam_toolbox/graph_visualization '
                              '/rtabmap/mapPath'.format(bag_path)],
                              start_new_session=True, stdout=self.nodes_log,
                              stderr=subprocess.STDOUT),
        ])
        self.slam_proc = self.procs[1]
        self.mapping = True
        self._tick = 0
        self.write_live_status()
        self.get_logger().info(
            'MAPPING START ({}) -> session {}'.format(self.active_algo, session_name))

    def request_final_optimization(self):
        """Ask graph-SLAM backends to settle before the final map is saved."""
        if self.nodes_log is None:
            output = subprocess.DEVNULL
        else:
            output = self.nodes_log
        command = None
        if self.active_algo == 'cartographer':
            command = (
                'timeout 3s ros2 service call /finish_trajectory '
                'cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"')
        elif self.active_algo == 'rtabmap':
            command = (
                'timeout 3s ros2 service call /publish_map '
                'rtabmap_msgs/srv/PublishMap '
                '"{global_map: true, optimized: true, graph_only: false}"')
        if command is not None:
            subprocess.Popen(
                ['bash', '-c', ENV + ' && ' + command],
                stdout=output, stderr=subprocess.STDOUT)

    def stop_mapping(self, reason='requested', finalize=True):
        if self.stopping:
            if not finalize:
                self._complete_stop(reason)
            return
        if finalize and self.active_algo in ('cartographer', 'slam_toolbox', 'rtabmap'):
            self.stopping = True
            self.stop_reason = reason
            self.stop_deadline = time.monotonic() + FINAL_OPTIMIZE_SEC
            self.request_final_optimization()
            self.write_live_status()
            if self.session:
                self.append_metadata('final_optimization: requested\n')
            self.get_logger().info(
                'FINAL OPTIMIZATION ({}) - waiting {:.1f}s before map save'.format(
                    self.active_algo, FINAL_OPTIMIZE_SEC))
            return
        self._complete_stop(reason)

    def _complete_stop(self, reason):
        self.get_logger().info('MAPPING STOP -> saving final map...')
        self.stopping = False
        self.mapping = False
        self.save_map(final=True)
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGINT)   # graceful: lidar motor stop
            except Exception:
                pass
        # rosbag needs time to write metadata.yaml and close SQLite cleanly.
        # Poll instead of blindly killing it after four seconds.
        graceful_deadline = time.monotonic() + 8.0
        while time.monotonic() < graceful_deadline:
            if all(p.poll() is not None for p in self.procs):
                break
            time.sleep(0.2)
        for p in self.procs:
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except Exception:
                    pass
        subprocess.call([
            'bash', '-c',
            "pkill -9 -f 'ros2 (launch sllidar_ros2|launch slam_gmapping|"
            "launch yahboomcar_nav cartographer_launch.py|run slam_toolbox|"
            "run rtabmap_slam)|/opt/ros/foxy/lib/(sllidar_ros2|slam_gmapping|"
            "cartographer_ros|slam_toolbox|rtabmap_slam)/' "
            "2>/dev/null; true"
        ])
        self.procs = []
        self.slam_proc = None
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
        if self.optimized_traj and self.session and os.path.isdir(self.session):
            try:
                with open(os.path.join(self.session, 'trajectory_optimized.csv'), 'w') as f:
                    f.write('x,y\n')
                    for (x, y) in self.optimized_traj:
                        f.write('{:.3f},{:.3f}\n'.format(x, y))
            except Exception:
                pass
        if self.graph_segments and self.session and os.path.isdir(self.session):
            try:
                with open(os.path.join(self.session, 'pose_graph_edges.csv'), 'w') as f:
                    f.write('x1,y1,x2,y2\n')
                    for ((x1, y1), (x2, y2)) in self.graph_segments:
                        f.write('{:.3f},{:.3f},{:.3f},{:.3f}\n'.format(
                            x1, y1, x2, y2))
            except Exception:
                pass
        if self.gear_events and self.session and os.path.isdir(self.session):
            try:
                with open(os.path.join(self.session, 'gear_events.csv'), 'w') as f:
                    f.write('elapsed_s,gear\n')
                    for elapsed, gear in self.gear_events:
                        f.write('{:.3f},{}\n'.format(elapsed, self.gear_display(gear)))
            except Exception:
                pass
        # Keep incomplete sessions: their rosbag and logs can recover or diagnose a failed map.
        if self.session and not os.path.exists(os.path.join(self.session, 'map.pgm')):
            self.append_metadata('scan_end: {}\nstatus: incomplete_no_map\n'.format(
                time.strftime('%Y%m%d_%H%M%S')))
            self.get_logger().warn('incomplete session kept for recovery: {}'.format(self.session))
        if self.session and os.path.isdir(self.session):
            try:
                final_traj = self.optimized_traj or self.traj
                endpoint_distance = 0.0
                if len(final_traj) >= 2:
                    endpoint_distance = math.sqrt(
                        (final_traj[-1][0] - final_traj[0][0]) ** 2
                        + (final_traj[-1][1] - final_traj[0][1]) ** 2)
                self.append_metadata(
                    'stop_reason: {}\npose_jumps: {}\n'
                    'trajectory_source: {}\n'
                    'trajectory_endpoint_distance_m: {:.3f}\n'.format(
                        reason, self.pose_jump_count, self.trajectory_source,
                        endpoint_distance))
            except Exception:
                pass
        self.active_algo = None
        self.get_logger().info('lidar + SLAM stopped, motor off')

    def tick(self):
        if self.stopping:
            if time.monotonic() >= self.stop_deadline:
                self._complete_stop(self.stop_reason)
            else:
                self.write_live_status()
            return
        if not self.mapping:
            if (self.start_pending_since is not None
                    and time.monotonic() - self.start_pending_since >= START_SETTLE_SEC):
                self.start_pending_since = None
                if self.sensors_healthy():
                    self.start_mapping()
                else:
                    self.get_logger().error(
                        'mapping refused: odometry/IMU/voltage health check failed')
            return
        self._tick += 1
        if self.slam_proc is not None and self.slam_proc.poll() is not None:
            exit_code = self.slam_proc.returncode
            self.get_logger().error(
                'SLAM process exited unexpectedly with code {}'.format(exit_code))
            self.append_metadata('slam_exit_code: {}\n'.format(exit_code))
            self.stop_mapping(reason='slam_process_exited', finalize=False)
            return
        if self._tick % 2 == 0 and not self.sensors_healthy():
            self.get_logger().error(
                'sensor health lost during mapping - saving and stopping SLAM')
            self.stop_mapping(reason='sensor_health_lost', finalize=False)
            return
        now = time.monotonic()
        moving_recently = now - self.last_motion_rx < 2.0
        map_age = now - self.last_map_rx if self.last_map_rx else float('inf')
        if (now - self.start_mono > 20.0 and moving_recently
                and map_age > MAP_STALL_TIMEOUT_SEC):
            self.get_logger().error(
                'map update stalled for {:.1f}s while moving'.format(map_age))
            self.append_metadata('map_stall_seconds: {:.3f}\n'.format(map_age))
            self.stop_mapping(reason='map_update_stalled', finalize=False)
            return
        if self._tick % 2 == 0:
            self.write_live_status()
        if self._tick == SCAN_START_TIMEOUT_TICKS and self.scan_count == 0:
            if self.startup_retries < 1:
                self.startup_retries += 1
                self.get_logger().warn('no /scan after 15s — restarting LiDAR + SLAM once')
                self.stop_mapping(reason='lidar_start_timeout_retry', finalize=False)
                self.start_mapping(retry=True)
            else:
                self.get_logger().error('no /scan after retry — stopping mapping for safety')
                self.stop_mapping(reason='lidar_start_failed', finalize=False)
            return
        if self._tick % 20 == 0:
            free = shutil.disk_usage(MAPS_DIR).free
            if free < MIN_RUNTIME_FREE_BYTES:
                self.get_logger().error(
                    'disk safety stop: only {:.2f} GiB free'.format(free / 1024 ** 3))
                self.stop_mapping(reason='low_disk', finalize=False)
                return
        self.update_pose()
        if self._map_dirty or (self._tick % 2 == 0):  # re-render on new map or every 1s (traj moves)
            self._map_dirty = False
            self.write_preview()
        if self._tick % AUTOSAVE_TICKS == 0:
            self.save_map()

    def update_pose(self):
        t = None
        for frame in ('base_footprint', 'base_link', 'laser'):
            try:
                t = self.tf_buf.lookup_transform('map', frame, RclTime())
                self.pose_frame = frame
                break
            except Exception:
                continue
        if t is None:
            return
        try:
            x = t.transform.translation.x
            y = t.transform.translation.y
            if self.traj:
                step = math.sqrt(
                    (x - self.traj[-1][0]) ** 2 + (y - self.traj[-1][1]) ** 2)
                if step > MAX_POSE_STEP_M:
                    self.pose_jump_count += 1
                    self.get_logger().error(
                        'SLAM pose jump detected: {:.2f} m'.format(step))
            if not self.traj or (x - self.traj[-1][0]) ** 2 + (y - self.traj[-1][1]) ** 2 > 0.0025:
                self.traj.append((x, y))
        except Exception:
            return

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

        segments = self.graph_segments
        if not segments:
            segments = list(zip(self.traj, self.traj[1:]))

        # Draw either the optimized pose graph/path or, until one is
        # available, the live TF history. Rasterize each line so graph
        # corrections replace the complete old trajectory on the next frame.
        for ((x1, y1), (x2, y2)) in segments:
            r1, c1 = px(x1, y1)
            r2, c2 = px(x2, y2)
            steps = max(abs(r2 - r1), abs(c2 - c1), 1)
            for step in range(steps + 1):
                fraction = float(step) / steps
                r = int(round(r1 + (r2 - r1) * fraction))
                c = int(round(c1 + (c2 - c1) * fraction))
                if 0 <= r < h and 0 <= c < w:
                    rgb[max(0, r - 1):r + 2,
                        max(0, c - 1):c + 2] = (220, 40, 40)
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
            self.append_metadata(
                'scan_end: {}\nstatus: complete\nmap: {}x{} res={}\n'.format(
                    time.strftime('%Y%m%d_%H%M%S'), w, h, res))
            self.get_logger().info('final map saved: {}x{}'.format(w, h))


def main():
    rclpy.init()
    node = SlamSupervisor()
    try:
        rclpy.spin(node)
    finally:
        if node.mapping:
            node.stop_mapping(reason='supervisor_shutdown', finalize=False)


if __name__ == '__main__':
    main()
