#!/usr/bin/env python3
"""Replay only SLAM inputs in an isolated ROS domain; never publish commands.

Run inside the sourced Foxy container. Outputs are new files under --out.
Supports SQLite rosbag2 splits, synchronized /clock, filtered TF and map metrics.
"""
import argparse
import glob
import heapq
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import resource
import sqlite3
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from tf2_msgs.msg import TFMessage

INPUTS = {'/scan', '/odom', '/imu/data', '/tf', '/tf_static'}


def filtered_tf(msg):
    # Recorded map->odom belongs to the original SLAM, never the replay.
    msg.transforms = [t for t in msg.transforms
                      if t.header.frame_id.lstrip('/') != 'map'
                      and t.child_frame_id.lstrip('/') != 'map']
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--algorithm', choices=['rtabmap', 'slam_toolbox', 'cartographer', 'gmapping'], required=True)
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--rate', type=float, default=1)
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        parser.error('Set a dedicated nonzero ROS_DOMAIN_ID to isolate hardware.')
    if args.rate <= 0:
        parser.error('--rate must be positive')
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.config, out / Path(args.config).name)
    usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    wall_start = time.monotonic()
    dbs = sorted(glob.glob(str(Path(args.bag) / '*.db3')))
    if not dbs:
        parser.error('No SQLite rosbag2 files found')
    connections, streams, types = [], [], {}
    for db in dbs:
        conn = sqlite3.connect('file:' + db + '?mode=ro', uri=True)
        connections.append(conn)
        for name, typename in conn.execute('SELECT name,type FROM topics'):
            if name in INPUTS:
                types[name] = typename
        streams.append(iter(conn.execute(
            'SELECT timestamp,name,data FROM messages JOIN topics ON topic_id=topics.id '
            'WHERE name IN (?,?,?,?,?) ORDER BY timestamp', tuple(sorted(INPUTS)))))
    records = list(heapq.merge(*streams, key=lambda row: row[0]))
    if not records:
        parser.error('No input messages')
    missing = {'/scan', '/odom', '/tf', '/tf_static'} - set(types)
    if missing:
        parser.error('Bag lacks required inputs: ' + ', '.join(sorted(missing)))
    t0 = records[0][0]
    if args.duration:
        records = [r for r in records if r[0] <= t0 + args.duration * 1e9]
    rclpy.init()
    node = rclpy.create_node('slam_replay_probe')
    pubs, classes = {}, {}
    for topic, typename in types.items():
        cls = get_message(typename)
        classes[topic] = cls
        qos = QoSProfile(depth=100)
        if topic == '/tf_static':
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pubs[topic] = node.create_publisher(cls, topic, qos)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    maps, paths, positions = [], [], []
    def on_map(msg):
        maps[:] = [msg]
        stats['map_messages'] += 1
    def on_path(msg):
        paths[:] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
                positions.append((t.transform.translation.x, t.transform.translation.y))
    node.create_subscription(OccupancyGrid, '/map', on_map, 10)
    node.create_subscription(NavPath, '/rtabmap/mapPath', on_path, 10)
    node.create_subscription(NavPath, '/mapPath', on_path, 10)
    node.create_subscription(TFMessage, '/tf', on_tf, 100)
    stats = {'algorithm': args.algorithm, 'bag': args.bag, 'config': args.config,
             'domain': os.environ['ROS_DOMAIN_ID'], 'rate': args.rate,
             'duration_s': (records[-1][0]-t0)/1e9, 'published': {}, 'map_messages': 0}
    def clock(ns):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(int(ns), 10**9)
        clock_pub.publish(msg)
    # Only the static tree needs replay before the first scan. Accumulate all
    # static edges because rosbag2 may have recorded multiple publishers.
    static_edges = {}
    for _, topic, data in records:
        if topic == '/tf_static':
            for transform in filtered_tf(deserialize_message(data, classes[topic])).transforms:
                static_edges[transform.child_frame_id] = transform
    def prime():
        clock(t0)
        if '/tf_static' in pubs:
            msg = TFMessage()
            msg.transforms = list(static_edges.values())
            pubs['/tf_static'].publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)
    rosargs = ['--ros-args', '-p', 'use_sim_time:=true']
    extra = []
    if args.algorithm == 'rtabmap':
        command = ['ros2', 'run', 'rtabmap_slam', 'rtabmap', '--ros-args', '--params-file', args.config,
                   '-p', 'use_sim_time:=true', '-p', 'database_path:=' + str(out/'rtabmap.db')]
    elif args.algorithm == 'slam_toolbox':
        command = ['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node', '--ros-args',
                   '--params-file', args.config, '-p', 'use_sim_time:=true']
    elif args.algorithm == 'gmapping':
        command = ['ros2', 'run', 'slam_gmapping', 'slam_gmapping', '--ros-args',
                   '--params-file', args.config, '-p', 'use_sim_time:=true']
    else:
        config = Path(args.config).resolve()
        command = ['ros2', 'run', 'cartographer_ros', 'cartographer_node',
                   '-configuration_directory', str(config.parent), '-configuration_basename', config.name,
                   '--ros-args', '-p', 'use_sim_time:=true', '-r', 'imu:=/imu/data', '-r', 'odom:=/odom']
        extra = ['ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
                 '-resolution', '0.05', '-publish_period_sec', '1.0'] + rosargs
    log = open(out/'nodes.log', 'w')
    processes = []
    try:
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        if extra:
            processes.append(subprocess.Popen(extra, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            prime()
        start = time.monotonic()
        for timestamp, topic, data in records:
            target = start + (timestamp-t0)/1e9/args.rate
            while time.monotonic() < target:
                rclpy.spin_once(node, timeout_sec=min(0.01, max(0, target-time.monotonic())))
            if processes[0].poll() is not None:
                raise RuntimeError('SLAM exited during replay')
            clock(timestamp)
            msg = deserialize_message(data, classes[topic])
            if topic in ('/tf', '/tf_static'):
                msg = filtered_tf(msg)
                if not msg.transforms:
                    continue
            pubs[topic].publish(msg)
            stats['published'][topic] = stats['published'].get(topic, 0) + 1
            rclpy.spin_once(node, timeout_sec=0)
        # Advance simulated time after input ends to let map timers finish.
        drain_start = time.monotonic()
        while time.monotonic()-drain_start < 5:
            clock(records[-1][0] + (time.monotonic()-drain_start)*1e9)
            rclpy.spin_once(node, timeout_sec=0.02)
        if args.algorithm != 'gmapping':
            from slam_finalization import SessionFinalizer
            node.map_cb = on_map
            node.append_metadata = lambda value: None
            finalizer = SessionFinalizer(node, args.algorithm, str(out))
            while not finalizer.done:
                clock(records[-1][0] + (time.monotonic()-drain_start)*1e9)
                rclpy.spin_once(node, timeout_sec=0.02)
            settle = time.monotonic() + 2
            while time.monotonic() < settle:
                clock(records[-1][0] + (time.monotonic()-drain_start)*1e9)
                rclpy.spin_once(node, timeout_sec=0.02)
        if maps:
            msg = maps[0]
            grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
            np.save(out/'map.npy', grid)
            pixels = np.full(grid.shape, 205, dtype=np.uint8)
            pixels[(grid >= 0) & (grid <= 25)] = 254
            pixels[grid >= 55] = 0
            with open(out/'map.pgm', 'wb') as stream:
                stream.write(('P5\n%d %d\n255\n' % (msg.info.width, msg.info.height)).encode())
                stream.write(np.flipud(pixels).tobytes())
            stats.update(map_size=[msg.info.width, msg.info.height], resolution=msg.info.resolution,
                         occupied_cells=int((grid >= 55).sum()), known_cells=int((grid >= 0).sum()))
        stats['optimized_path_nodes'] = len(paths)
        if len(paths) >= 2:
            stats['endpoint_separation_m'] = math.hypot(paths[-1][0]-paths[0][0], paths[-1][1]-paths[0][1])
        stats['map_odom_max_step_m'] = max((math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(positions,positions[1:])), default=0)
        (out/'path.json').write_text(json.dumps(paths))
    finally:
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        log.close()
        text = (out/'nodes.log').read_text(errors='replace')
        stats['negative_hessian_index_warnings'] = text.count('negative hessian index')
        stats['fatal_logs'] = len(re.findall(r'\bFATAL\b', text))
        stats['scan_drops'] = text.count('Message Filter dropping message')
        stats['rtabmap_processing_seconds'] = [float(v) for v in re.findall(r'RTAB-Map=([0-9.]+)s', text)]
        usage_after = resource.getrusage(resource.RUSAGE_CHILDREN)
        stats['child_cpu_seconds'] = (usage_after.ru_utime + usage_after.ru_stime
                                     - usage_before.ru_utime - usage_before.ru_stime)
        stats['wall_seconds'] = time.monotonic() - wall_start
        if (out/'rtabmap.db').exists():
            with sqlite3.connect(str(out/'rtabmap.db')) as conn:
                stats['database_links_by_type'] = dict(conn.execute('SELECT type,count(*) FROM Link GROUP BY type'))
                stats['database_nodes'] = conn.execute('SELECT count(*) FROM Node').fetchone()[0]
        (out/'metrics.json').write_text(json.dumps(stats, indent=2))
        print(json.dumps({k:v for k,v in stats.items() if k != 'rtabmap_processing_seconds'}, indent=2), flush=True)
        for conn in connections:
            conn.close()
        node.destroy_node()
        rclpy.shutdown()
    if not maps:
        raise RuntimeError('No map received; replay is not a valid comparison')


if __name__ == '__main__':
    main()
