#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SLAM 后处理工具 - 从 rosbag 提取轨迹 + 激光扫描，重建修正地图和点云
Post-process SLAM data from rosbag files: extract trajectory, rebuild map & point cloud.

用法:
  pip3 install rosbags numpy pyyaml Pillow
  python3 postprocess_slam.py [bag_dir] [--output output_dir]

默认:
  bag_dir:  ~/Documents/rosmaster_r2/bags/
  output:   ~/Documents/rosmaster_r2/postprocess/

功能:
  1. 从所有 rosbag 提取 /odom 轨迹 → trajectory.csv
  2. 从所有 rosbag 提取 /scan 激光数据
  3. 用轨迹 + scan 重建占栅格地图 (OccupancyGrid PGM)
  4. 用轨迹 + scan 生成修正点云 (PCD)
  5. 生成带轨迹叠加的可视化图
"""

import sys
import os
import math
import csv
import struct
import time
import argparse
from pathlib import Path
from collections import namedtuple

import numpy as np

# ---------------------------------------------------------------------------
# Try rosbags library first, fall back to manual parser
# ---------------------------------------------------------------------------
USE_ROSBAGS = False
try:
    from rosbags.rosbag1 import Reader as Rosbag1Reader
    from rosbags.serde import deserialize_cdr, ros1_to_cdr
    USE_ROSBAGS = True
    print("[info] 使用 rosbags 库解析 bag 文件")
except ImportError:
    print("[info] rosbags 未安装，使用内置解析器 (pip3 install rosbags 可获得更好支持)")


# ---------------------------------------------------------------------------
# Built-in ROS1 bag parser (fallback)
# ---------------------------------------------------------------------------
class BagMessage:
    """Simple container for a deserialized bag message."""
    __slots__ = ['topic', 'timestamp', 'data']
    def __init__(self, topic, timestamp, data):
        self.topic = topic
        self.timestamp = timestamp
        self.data = data


def read_ros1_bag_manual(bag_path, topics=None):
    """
    Minimalist ROS1 bag parser. Two-pass: first find connections, then parse chunks.
    Returns list of BagMessage with raw bytes in .data
    """
    messages = []

    with open(bag_path, 'rb') as f:
        # Verify header
        magic = f.readline()
        if not magic.startswith(b'#ROSBAG'):
            raise ValueError(f"Not a ROS bag file: {bag_path}")

        # First pass: find all connection records
        connections = {}  # conn_id -> {topic, type, md5sum}
        f.seek(0)
        f.readline()  # skip magic

        while True:
            pos = f.tell()
            # Read header length
            buf = f.read(4)
            if len(buf) < 4:
                break
            header_len = struct.unpack('<I', buf)[0]
            header_data = f.read(header_len)
            if len(header_data) < header_len:
                break

            # Read data length
            buf = f.read(4)
            if len(buf) < 4:
                break
            data_len = struct.unpack('<I', buf)[0]
            data_start = f.tell()

            # Parse header fields
            fields = {}
            offset = 0
            while offset < header_len:
                field_len = struct.unpack('<I', header_data[offset:offset+4])[0]
                offset += 4
                field_bytes = header_data[offset:offset+field_len]
                offset += field_len
                eq_pos = field_bytes.find(b'=')
                if eq_pos >= 0:
                    key = field_bytes[:eq_pos].decode('ascii', errors='replace')
                    val = field_bytes[eq_pos+1:]
                    fields[key] = val

            op = fields.get('op', b'')
            if isinstance(op, bytes) and len(op) == 1:
                op_val = op[0]
            else:
                f.seek(data_start + data_len)
                continue

            # op=0x07: connection record
            if op_val == 0x07:
                conn_id = struct.unpack('<I', fields.get('conn', b'\x00\x00\x00\x00'))[0]
                topic = fields.get('topic', b'').decode('utf-8', errors='replace')
                # Parse connection header from data
                conn_header = f.read(data_len)
                conn_fields = {}
                off2 = 0
                while off2 < len(conn_header):
                    if off2 + 4 > len(conn_header):
                        break
                    fl = struct.unpack('<I', conn_header[off2:off2+4])[0]
                    off2 += 4
                    fb = conn_header[off2:off2+fl]
                    off2 += fl
                    eq = fb.find(b'=')
                    if eq >= 0:
                        conn_fields[fb[:eq].decode('ascii', errors='replace')] = fb[eq+1:]

                msg_type = conn_fields.get('type', b'').decode('utf-8', errors='replace')
                connections[conn_id] = {
                    'topic': topic,
                    'type': msg_type,
                }
                continue

            f.seek(data_start + data_len)

        # Determine which connections we care about
        if topics:
            target_conns = {cid for cid, info in connections.items()
                           if info['topic'] in topics}
        else:
            target_conns = set(connections.keys())

        if not target_conns:
            return messages

        # Second pass: read message data
        f.seek(0)
        f.readline()  # skip magic

        while True:
            buf = f.read(4)
            if len(buf) < 4:
                break
            header_len = struct.unpack('<I', buf)[0]
            header_data = f.read(header_len)
            if len(header_data) < header_len:
                break

            buf = f.read(4)
            if len(buf) < 4:
                break
            data_len = struct.unpack('<I', buf)[0]
            data_start = f.tell()

            # Parse header
            fields = {}
            offset = 0
            while offset < header_len:
                field_len = struct.unpack('<I', header_data[offset:offset+4])[0]
                offset += 4
                field_bytes = header_data[offset:offset+field_len]
                offset += field_len
                eq_pos = field_bytes.find(b'=')
                if eq_pos >= 0:
                    key = field_bytes[:eq_pos].decode('ascii', errors='replace')
                    val = field_bytes[eq_pos+1:]
                    fields[key] = val

            op = fields.get('op', b'')
            if isinstance(op, bytes) and len(op) == 1:
                op_val = op[0]
            else:
                f.seek(data_start + data_len)
                continue

            # op=0x02: message data
            if op_val == 0x02:
                conn_id = struct.unpack('<I', fields.get('conn', b'\x00\x00\x00\x00'))[0]
                if conn_id in target_conns:
                    time_bytes = fields.get('time', b'\x00' * 8)
                    if len(time_bytes) >= 8:
                        secs = struct.unpack('<I', time_bytes[:4])[0]
                        nsecs = struct.unpack('<I', time_bytes[4:8])[0]
                        timestamp = secs + nsecs * 1e-9
                    else:
                        timestamp = 0.0

                    msg_data = f.read(data_len)
                    topic = connections[conn_id]['topic']
                    msg_type = connections[conn_id]['type']
                    messages.append(BagMessage(topic, timestamp,
                                              {'raw': msg_data, 'type': msg_type}))
                    continue

            f.seek(data_start + data_len)

    messages.sort(key=lambda m: m.timestamp)
    return messages


# ---------------------------------------------------------------------------
# ROS message deserializers (manual, for fallback parser)
# ---------------------------------------------------------------------------
def deserialize_odom(raw_bytes):
    """Deserialize nav_msgs/Odometry from raw bytes."""
    off = 0
    # Header
    seq = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    stamp_sec = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    stamp_nsec = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    frame_len = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    frame_id = raw_bytes[off:off+frame_len].decode('utf-8', errors='replace'); off += frame_len
    # child_frame_id
    child_len = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    child_frame = raw_bytes[off:off+child_len].decode('utf-8', errors='replace'); off += child_len
    # Pose
    px, py, pz = struct.unpack_from('<ddd', raw_bytes, off); off += 24
    qx, qy, qz, qw = struct.unpack_from('<dddd', raw_bytes, off); off += 32
    # Pose covariance (36 doubles)
    off += 36 * 8
    # Twist
    vx, vy, vz = struct.unpack_from('<ddd', raw_bytes, off); off += 24
    wx, wy, wz = struct.unpack_from('<ddd', raw_bytes, off); off += 24

    return {
        'timestamp': stamp_sec + stamp_nsec * 1e-9,
        'x': px, 'y': py, 'z': pz,
        'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
        'vx': vx, 'vy': vy, 'vz': vz,
        'wx': wx, 'wy': wy, 'wz': wz,
    }


def deserialize_laserscan(raw_bytes):
    """Deserialize sensor_msgs/LaserScan from raw bytes."""
    off = 0
    # Header
    seq = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    stamp_sec = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    stamp_nsec = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    frame_len = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    frame_id = raw_bytes[off:off+frame_len].decode('utf-8', errors='replace'); off += frame_len
    # Scan params
    angle_min, angle_max, angle_inc = struct.unpack_from('<fff', raw_bytes, off); off += 12
    time_inc, scan_time = struct.unpack_from('<ff', raw_bytes, off); off += 8
    range_min, range_max = struct.unpack_from('<ff', raw_bytes, off); off += 8
    # Ranges array
    n_ranges = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    ranges = list(struct.unpack_from(f'<{n_ranges}f', raw_bytes, off)); off += n_ranges * 4
    # Intensities array
    n_int = struct.unpack_from('<I', raw_bytes, off)[0]; off += 4
    intensities = list(struct.unpack_from(f'<{n_int}f', raw_bytes, off)) if n_int > 0 else []

    return {
        'timestamp': stamp_sec + stamp_nsec * 1e-9,
        'angle_min': angle_min,
        'angle_max': angle_max,
        'angle_increment': angle_inc,
        'range_min': range_min,
        'range_max': range_max,
        'ranges': ranges,
    }


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Bag processing
# ---------------------------------------------------------------------------
def process_bags_rosbags(bag_files):
    """Process bags using rosbags library."""
    odom_data = []
    scan_data = []

    for bag_path in bag_files:
        print(f"  处理: {bag_path.name}")
        try:
            with Rosbag1Reader(bag_path) as reader:
                for conn, timestamp, rawdata in reader.messages():
                    ts = timestamp / 1e9  # nanoseconds to seconds

                    if conn.topic == '/odom':
                        try:
                            msg = deserialize_cdr(ros1_to_cdr(rawdata, conn.msgtype), conn.msgtype)
                            odom_data.append({
                                'timestamp': ts,
                                'x': msg.pose.pose.position.x,
                                'y': msg.pose.pose.position.y,
                                'z': msg.pose.pose.position.z,
                                'qx': msg.pose.pose.orientation.x,
                                'qy': msg.pose.pose.orientation.y,
                                'qz': msg.pose.pose.orientation.z,
                                'qw': msg.pose.pose.orientation.w,
                                'vx': msg.twist.twist.linear.x,
                                'wz': msg.twist.twist.angular.z,
                            })
                        except Exception as e:
                            pass

                    elif conn.topic == '/scan':
                        try:
                            msg = deserialize_cdr(ros1_to_cdr(rawdata, conn.msgtype), conn.msgtype)
                            scan_data.append({
                                'timestamp': ts,
                                'angle_min': msg.angle_min,
                                'angle_max': msg.angle_max,
                                'angle_increment': msg.angle_increment,
                                'range_min': msg.range_min,
                                'range_max': msg.range_max,
                                'ranges': list(msg.ranges),
                            })
                        except Exception as e:
                            pass
        except Exception as e:
            print(f"    错误: {e}")

    return odom_data, scan_data


def process_bags_manual(bag_files):
    """Process bags using built-in parser."""
    odom_data = []
    scan_data = []

    for bag_path in bag_files:
        print(f"  处理: {bag_path.name}")
        try:
            messages = read_ros1_bag_manual(str(bag_path), topics={'/odom', '/scan'})
            for msg in messages:
                if msg.topic == '/odom':
                    try:
                        odom = deserialize_odom(msg.data['raw'])
                        odom_data.append(odom)
                    except Exception as e:
                        pass
                elif msg.topic == '/scan':
                    try:
                        scan = deserialize_laserscan(msg.data['raw'])
                        scan_data.append(scan)
                    except Exception as e:
                        pass
        except Exception as e:
            print(f"    错误: {e}")

    return odom_data, scan_data


# ---------------------------------------------------------------------------
# Trajectory interpolation
# ---------------------------------------------------------------------------
def interpolate_pose(odom_data, target_time):
    """Find the closest odom pose for a given timestamp via linear interpolation."""
    if not odom_data:
        return None

    # Binary search for closest
    lo, hi = 0, len(odom_data) - 1

    def _add_yaw(d):
        """Ensure dict has 'yaw' key computed from quaternion."""
        if 'yaw' not in d:
            d = dict(d)
            d['yaw'] = quaternion_to_yaw(d.get('qx',0), d.get('qy',0),
                                          d.get('qz',0), d.get('qw',1))
        return d

    if target_time <= odom_data[lo]['timestamp']:
        return _add_yaw(odom_data[lo])
    if target_time >= odom_data[hi]['timestamp']:
        return _add_yaw(odom_data[hi])

    while hi - lo > 1:
        mid = (lo + hi) // 2
        if odom_data[mid]['timestamp'] <= target_time:
            lo = mid
        else:
            hi = mid

    # Linear interpolation
    t0 = odom_data[lo]['timestamp']
    t1 = odom_data[hi]['timestamp']
    dt = t1 - t0
    if dt < 1e-9:
        return _add_yaw(odom_data[lo])

    alpha = (target_time - t0) / dt
    result = {}
    for key in ['x', 'y', 'z']:
        result[key] = odom_data[lo][key] + alpha * (odom_data[hi][key] - odom_data[lo][key])

    # Interpolate yaw
    yaw0 = quaternion_to_yaw(odom_data[lo]['qx'], odom_data[lo]['qy'],
                              odom_data[lo]['qz'], odom_data[lo]['qw'])
    yaw1 = quaternion_to_yaw(odom_data[hi]['qx'], odom_data[hi]['qy'],
                              odom_data[hi]['qz'], odom_data[hi]['qw'])
    # Handle angle wrapping
    dyaw = yaw1 - yaw0
    while dyaw > math.pi: dyaw -= 2 * math.pi
    while dyaw < -math.pi: dyaw += 2 * math.pi
    result['yaw'] = yaw0 + alpha * dyaw
    result['timestamp'] = target_time

    return result


# ---------------------------------------------------------------------------
# Map building
# ---------------------------------------------------------------------------
def build_occupancy_grid(odom_data, scan_data, resolution=0.05, max_range=10.0):
    """
    Build an occupancy grid from trajectory + laser scans.
    Uses simple ray-casting (Bresenham) for each scan.
    """
    print(f"\n  构建占栅格地图 (分辨率={resolution}m)...")

    # First pass: determine map bounds
    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')

    for odom in odom_data:
        x, y = odom['x'], odom['y']
        min_x = min(min_x, x - max_range)
        max_x = max(max_x, x + max_range)
        min_y = min(min_y, y - max_range)
        max_y = max(max_y, y + max_range)

    # Add margin
    margin = 2.0
    min_x -= margin; min_y -= margin
    max_x += margin; max_y += margin

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))

    print(f"  地图尺寸: {width}x{height} ({(max_x-min_x):.1f}m x {(max_y-min_y):.1f}m)")

    # Limit map size
    if width > 10000 or height > 10000:
        print("  警告: 地图过大，缩小范围")
        resolution *= 2
        width = int(math.ceil((max_x - min_x) / resolution))
        height = int(math.ceil((max_y - min_y) / resolution))

    # Log-odds grid
    log_odds = np.zeros((height, width), dtype=np.float32)
    l_occ = 0.85   # log-odds for occupied
    l_free = -0.40  # log-odds for free
    l_max = 5.0
    l_min = -5.0

    def world_to_grid(wx, wy):
        gx = int((wx - min_x) / resolution)
        gy = int((wy - min_y) / resolution)
        return gx, gy

    def bresenham(x0, y0, x1, y1):
        """Yield grid cells along a line."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    # Process each scan
    processed = 0
    total_scans = len(scan_data)
    t_start = time.time()

    for idx, scan in enumerate(scan_data):
        if idx % 100 == 0 and idx > 0:
            elapsed = time.time() - t_start
            rate = idx / elapsed
            eta = (total_scans - idx) / rate
            print(f"    进度: {idx}/{total_scans} ({idx*100//total_scans}%), ETA {eta:.0f}s")

        pose = interpolate_pose(odom_data, scan['timestamp'])
        if pose is None:
            continue

        rx, ry = pose['x'], pose['y']
        ryaw = pose['yaw']
        rgx, rgy = world_to_grid(rx, ry)

        angle = scan['angle_min']
        for r in scan['ranges']:
            if scan['range_min'] < r < min(scan['range_max'], max_range):
                # Hit point in world frame
                hx = rx + r * math.cos(ryaw + angle)
                hy = ry + r * math.sin(ryaw + angle)
                hgx, hgy = world_to_grid(hx, hy)

                # Ray-cast: free cells along ray
                for gx, gy in bresenham(rgx, rgy, hgx, hgy):
                    if 0 <= gx < width and 0 <= gy < height:
                        if gx == hgx and gy == hgy:
                            log_odds[gy, gx] = np.clip(log_odds[gy, gx] + l_occ, l_min, l_max)
                        else:
                            log_odds[gy, gx] = np.clip(log_odds[gy, gx] + l_free, l_min, l_max)

            angle += scan['angle_increment']
        processed += 1

    print(f"  处理完成: {processed}/{total_scans} scans, 耗时 {time.time()-t_start:.1f}s")

    # Convert log-odds to occupancy probability
    prob = 1.0 - 1.0 / (1.0 + np.exp(log_odds))

    # Convert to PGM values: 0=occupied(black), 254=free(white), 205=unknown(gray)
    pgm = np.full((height, width), 205, dtype=np.uint8)
    pgm[prob > 0.65] = 0      # occupied
    pgm[prob < 0.35] = 254    # free

    # Flip Y for PGM (origin at bottom-left in ROS)
    pgm_flipped = np.flipud(pgm)

    map_meta = {
        'resolution': resolution,
        'origin_x': min_x,
        'origin_y': min_y,
        'width': width,
        'height': height,
    }

    return pgm_flipped, map_meta


# ---------------------------------------------------------------------------
# Point cloud building
# ---------------------------------------------------------------------------
def build_point_cloud(odom_data, scan_data, max_range=10.0, downsample=1):
    """Build a 2D point cloud from trajectory + laser scans."""
    print(f"\n  构建点云 (downsample={downsample})...")

    points = []
    for idx, scan in enumerate(scan_data):
        if idx % downsample != 0:
            continue

        pose = interpolate_pose(odom_data, scan['timestamp'])
        if pose is None:
            continue

        rx, ry = pose['x'], pose['y']
        ryaw = pose['yaw']

        angle = scan['angle_min']
        for r in scan['ranges']:
            if scan['range_min'] < r < min(scan['range_max'], max_range):
                hx = rx + r * math.cos(ryaw + angle)
                hy = ry + r * math.sin(ryaw + angle)
                points.append((hx, hy, 0.0))
            angle += scan['angle_increment']

    print(f"  点云总数: {len(points)}")
    return points


# ---------------------------------------------------------------------------
# Output functions
# ---------------------------------------------------------------------------
def save_trajectory_csv(odom_data, output_path):
    """Save trajectory to CSV."""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'x', 'y', 'theta', 'vx', 'wz'])
        prev_x, prev_y = None, None
        for odom in odom_data:
            x, y = odom['x'], odom['y']
            # Filter very close points
            if prev_x is not None:
                dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                if dist < 0.01:
                    continue
            yaw = quaternion_to_yaw(odom.get('qx', 0), odom.get('qy', 0),
                                     odom.get('qz', 0), odom.get('qw', 1))
            vx = odom.get('vx', 0)
            wz = odom.get('wz', odom.get('wx', 0))
            writer.writerow([f"{odom['timestamp']:.6f}", f"{x:.4f}", f"{y:.4f}",
                           f"{yaw:.4f}", f"{vx:.4f}", f"{wz:.4f}"])
            prev_x, prev_y = x, y
    print(f"  轨迹已保存: {output_path}")


def save_pgm(pgm_data, output_path, map_meta):
    """Save occupancy grid as PGM + YAML."""
    h, w = pgm_data.shape
    with open(output_path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(pgm_data.tobytes())

    yaml_path = output_path.with_suffix('.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f"image: {output_path.name}\n")
        f.write(f"resolution: {map_meta['resolution']}\n")
        f.write(f"origin: [{map_meta['origin_x']}, {map_meta['origin_y']}, 0.0]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.196\n")

    print(f"  地图已保存: {output_path}")
    print(f"  YAML已保存: {yaml_path}")


def save_pcd(points, output_path):
    """Save point cloud as ASCII PCD file."""
    with open(output_path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
    print(f"  点云已保存: {output_path}")


def save_visualization(pgm_data, map_meta, odom_data, output_path):
    """Save map with trajectory overlay as PNG."""
    try:
        from PIL import Image
    except ImportError:
        print("  [跳过] 可视化需要 Pillow: pip3 install Pillow")
        return

    h, w = pgm_data.shape
    # Convert PGM to RGB
    rgb = np.stack([pgm_data, pgm_data, pgm_data], axis=-1).copy()

    res = map_meta['resolution']
    ox = map_meta['origin_x']
    oy = map_meta['origin_y']
    mh = map_meta['height']

    # Draw trajectory
    prev_px, prev_py = None, None
    for odom in odom_data:
        px = int((odom['x'] - ox) / res)
        py = mh - 1 - int((odom['y'] - oy) / res)

        if 0 <= px < w and 0 <= py < h:
            # Draw 2x2 red dot
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    ppx, ppy = px + dx, py + dy
                    if 0 <= ppx < w and 0 <= ppy < h:
                        rgb[ppy, ppx] = [255, 40, 40]

    # Start (green) and end (blue) markers
    if odom_data:
        for label, odom, color in [("start", odom_data[0], [0, 255, 0]),
                                     ("end", odom_data[-1], [0, 100, 255])]:
            px = int((odom['x'] - ox) / res)
            py = mh - 1 - int((odom['y'] - oy) / res)
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    ppx, ppy = px + dx, py + dy
                    if 0 <= ppx < w and 0 <= ppy < h:
                        rgb[ppy, ppx] = color

    img = Image.fromarray(rgb)
    img.save(str(output_path))
    print(f"  可视化已保存: {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='SLAM 后处理工具')
    parser.add_argument('bag_dir', nargs='?',
                        default=os.path.expanduser('~/Documents/rosmaster_r2/bags'),
                        help='rosbag 文件目录')
    parser.add_argument('--output', '-o',
                        default=os.path.expanduser('~/Documents/rosmaster_r2/postprocess'),
                        help='输出目录')
    parser.add_argument('--resolution', '-r', type=float, default=0.05,
                        help='地图分辨率 (m/pixel), 默认 0.05')
    parser.add_argument('--max-range', type=float, default=10.0,
                        help='激光最大有效距离 (m), 默认 10.0')
    parser.add_argument('--downsample', '-d', type=int, default=2,
                        help='点云降采样 (每N帧取1帧), 默认 2')
    args = parser.parse_args()

    bag_dir = Path(args.bag_dir)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("  SLAM 后处理工具 - ROSMaster R2")
    print("=" * 60)

    # Find bag files
    bag_files = sorted(bag_dir.glob('*.bag'))
    if not bag_files:
        print(f"\n错误: 未找到 bag 文件: {bag_dir}")
        sys.exit(1)

    print(f"\n找到 {len(bag_files)} 个 bag 文件:")
    total_size = sum(f.stat().st_size for f in bag_files)
    print(f"  总大小: {total_size / 1e6:.0f} MB")
    for f in bag_files:
        print(f"  - {f.name} ({f.stat().st_size / 1e6:.1f} MB)")

    # Step 1: Extract data from bags
    print(f"\n{'='*60}")
    print("  Step 1: 从 rosbag 提取数据")
    print(f"{'='*60}")

    t0 = time.time()
    if USE_ROSBAGS:
        odom_data, scan_data = process_bags_rosbags(bag_files)
    else:
        odom_data, scan_data = process_bags_manual(bag_files)

    print(f"\n  提取完成 ({time.time()-t0:.1f}s):")
    print(f"    /odom 消息: {len(odom_data)}")
    print(f"    /scan 消息: {len(scan_data)}")

    if not odom_data:
        print("\n错误: 未找到 /odom 数据！")
        sys.exit(1)

    # Sort by timestamp
    odom_data.sort(key=lambda x: x['timestamp'])
    scan_data.sort(key=lambda x: x['timestamp'])

    # Time range
    t_start = odom_data[0]['timestamp']
    t_end = odom_data[-1]['timestamp']
    duration = t_end - t_start
    print(f"    时间跨度: {duration:.1f}s ({duration/60:.1f}min)")

    # Step 2: Save trajectory
    print(f"\n{'='*60}")
    print("  Step 2: 保存轨迹")
    print(f"{'='*60}")
    save_trajectory_csv(odom_data, output_dir / 'trajectory.csv')

    # Step 3: Build occupancy grid map
    print(f"\n{'='*60}")
    print("  Step 3: 重建占栅格地图")
    print(f"{'='*60}")
    pgm_data, map_meta = build_occupancy_grid(
        odom_data, scan_data,
        resolution=args.resolution,
        max_range=args.max_range
    )
    save_pgm(pgm_data, output_dir / 'postprocess_map.pgm', map_meta)

    # Step 4: Build point cloud
    print(f"\n{'='*60}")
    print("  Step 4: 构建修正点云")
    print(f"{'='*60}")
    points = build_point_cloud(odom_data, scan_data,
                                max_range=args.max_range,
                                downsample=args.downsample)
    save_pcd(points, output_dir / 'postprocess_cloud.pcd')

    # Step 5: Visualization
    print(f"\n{'='*60}")
    print("  Step 5: 生成可视化")
    print(f"{'='*60}")
    save_visualization(pgm_data, map_meta, odom_data,
                       output_dir / 'postprocess_map_with_trajectory.png')

    # Summary
    print(f"\n{'='*60}")
    print("  后处理完成！")
    print(f"{'='*60}")
    print(f"  输出目录: {output_dir}")
    print(f"  文件:")
    for f in sorted(output_dir.iterdir()):
        print(f"    {f.name} ({f.stat().st_size / 1e3:.1f} KB)")


if __name__ == '__main__':
    main()
