#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SLAM 实时可视化工具 - Yahboom ROSMaster R2 (Host版)
在 Pi host 上运行，通过 rosbridge websocket 连接 Docker 内的 ROS.

用法:
  export XDG_RUNTIME_DIR=/run/user/1000
  python3 slam_visualizer_host.py [--host localhost] [--port 9090]

键盘控制:
  +/=    放大 / Zoom in
  -      缩小 / Zoom out
  c      重新居中到机器人 / Re-center on robot
  f      切换跟随模式 / Toggle follow mode
  t      切换轨迹显示 / Toggle trajectory display
  q/ESC  退出 / Quit

依赖 (Pi host):
  pip3 install pygame roslibpy
"""

import sys
import os
import math
import time
import threading
import signal
import argparse
import struct

# Ensure display is available
if 'XDG_RUNTIME_DIR' not in os.environ:
    os.environ['XDG_RUNTIME_DIR'] = '/run/user/1000'

try:
    import pygame
except ImportError:
    print("ERROR: pygame not installed. Run: pip3 install pygame")
    sys.exit(1)

try:
    import roslibpy
except ImportError:
    print("ERROR: roslibpy not installed. Run: pip3 install roslibpy")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 600
FPS_LIMIT = 30

# Colors (R, G, B)
COLOR_BG = (128, 128, 128)
COLOR_FREE = (255, 255, 255)
COLOR_OCCUPIED = (0, 0, 0)
COLOR_UNKNOWN = (128, 128, 128)
COLOR_ROBOT = (0, 200, 0)
COLOR_TRAIL = (200, 40, 40)
COLOR_SCAN = (40, 100, 255)
COLOR_TEXT = (220, 220, 220)
COLOR_STATUS_OK = (80, 255, 80)
COLOR_STATUS_WARN = (255, 200, 40)
COLOR_STATUS_ERR = (255, 60, 60)

ROBOT_SIZE = 12
MAX_TRAIL_POINTS = 5000
MIN_TRAIL_DISTANCE = 0.02


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class SlamVisualizer:
    def __init__(self, host='localhost', port=9090):
        self.ros_host = host
        self.ros_port = port
        self.lock = threading.Lock()

        # Map
        self.map_surface = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_updated = False

        # Odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.odom_received = False
        self.odom_time = 0.0

        # Laser scan
        self.scan_points = []
        self.scan_count = 0
        self.scan_received = False
        self.scan_time = 0.0

        # Trajectory
        self.trail_points = []

        # View
        self.zoom = 1.0
        self.pixels_per_meter = 20.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.follow_robot = True
        self.show_trail = True

        # Pygame
        self.screen = None
        self.font = None
        self.font_small = None
        self.running = False
        self.screen_w = SCREEN_WIDTH
        self.screen_h = SCREEN_HEIGHT

        # ROS connection
        self.ros_client = None
        self.connected = False
        self.connect_error = ""

    # -------------------------------------------------------------------
    # roslibpy callbacks
    # -------------------------------------------------------------------
    def _map_callback(self, msg):
        info = msg['info']
        w = info['width']
        h = info['height']
        res = info['resolution']
        ox = info['origin']['position']['x']
        oy = info['origin']['position']['y']
        data = msg['data']

        surf = pygame.Surface((w, h))
        try:
            pxa = pygame.PixelArray(surf)
            for j in range(h):
                row_off = j * w
                py = h - 1 - j
                for i in range(w):
                    val = data[row_off + i]
                    if val < 0:
                        color = COLOR_UNKNOWN
                    elif val < 50:
                        g = 255 - int(val * 3)
                        color = (g, g, g)
                    else:
                        g = max(0, int((100 - val) * 2.5))
                        color = (g, g, g)
                    pxa[i][py] = surf.map_rgb(color)
            pxa.close()
        except Exception:
            for j in range(h):
                for i in range(w):
                    val = data[j * w + i]
                    if val < 0:
                        color = COLOR_UNKNOWN
                    elif val < 50:
                        g = 255 - int(val * 3)
                        color = (g, g, g)
                    else:
                        g = max(0, int((100 - val) * 2.5))
                        color = (g, g, g)
                    surf.set_at((i, h - 1 - j), color)

        with self.lock:
            self.map_surface = surf
            self.map_width = w
            self.map_height = h
            self.map_resolution = res
            self.map_origin_x = ox
            self.map_origin_y = oy
            self.map_updated = True

    def _odom_callback(self, msg):
        pose = msg['pose']['pose']
        px = pose['position']['x']
        py = pose['position']['y']
        q = pose['orientation']
        yaw = quaternion_to_yaw(q['x'], q['y'], q['z'], q['w'])
        vx = msg['twist']['twist']['linear']['x']
        wz = msg['twist']['twist']['angular']['z']

        with self.lock:
            self.robot_x = px
            self.robot_y = py
            self.robot_yaw = yaw
            self.robot_vx = vx
            self.robot_wz = wz
            self.odom_received = True
            self.odom_time = time.time()

            if len(self.trail_points) == 0:
                self.trail_points.append((px, py))
            else:
                lx, ly = self.trail_points[-1]
                dist = math.sqrt((px - lx) ** 2 + (py - ly) ** 2)
                if dist >= MIN_TRAIL_DISTANCE:
                    self.trail_points.append((px, py))
                    if len(self.trail_points) > MAX_TRAIL_POINTS:
                        drop = MAX_TRAIL_POINTS // 10
                        self.trail_points = self.trail_points[drop:]

    def _scan_callback(self, msg):
        with self.lock:
            rx = self.robot_x
            ry = self.robot_y
            ryaw = self.robot_yaw

        angle_min = msg['angle_min']
        angle_inc = msg['angle_increment']
        range_min = msg['range_min']
        range_max = msg['range_max']
        ranges = msg['ranges']

        points = []
        angle = angle_min
        for r in ranges:
            if range_min < r < range_max:
                wx = rx + r * math.cos(ryaw + angle)
                wy = ry + r * math.sin(ryaw + angle)
                points.append((wx, wy))
            angle += angle_inc

        with self.lock:
            self.scan_points = points
            self.scan_count = len(points)
            self.scan_received = True
            self.scan_time = time.time()

    # -------------------------------------------------------------------
    # Coordinate transforms
    # -------------------------------------------------------------------
    def world_to_screen(self, wx, wy):
        scale = self.pixels_per_meter * self.zoom
        sx = int((wx - self.center_x) * scale + self.screen_w / 2.0)
        sy = int((self.center_y - wy) * scale + self.screen_h / 2.0)
        return sx, sy

    # -------------------------------------------------------------------
    # Drawing
    # -------------------------------------------------------------------
    def _draw_map(self, screen):
        with self.lock:
            surf = self.map_surface
            mw = self.map_width
            mh = self.map_height
            res = self.map_resolution
            ox = self.map_origin_x
            oy = self.map_origin_y

        if surf is None:
            return

        scale = self.pixels_per_meter * self.zoom
        cell_screen_size = res * scale
        scaled_w = max(1, int(mw * cell_screen_size))
        scaled_h = max(1, int(mh * cell_screen_size))

        if scaled_w > 4000 or scaled_h > 4000:
            return

        try:
            scaled = pygame.transform.scale(surf, (scaled_w, scaled_h))
        except Exception:
            return

        top_left_wx = ox
        top_left_wy = oy + mh * res
        sx, sy = self.world_to_screen(top_left_wx, top_left_wy)
        screen.blit(scaled, (sx, sy))

    def _draw_scan(self, screen):
        with self.lock:
            points = list(self.scan_points)
        if not points:
            return
        for wx, wy in points:
            sx, sy = self.world_to_screen(wx, wy)
            if 0 <= sx < self.screen_w and 0 <= sy < self.screen_h:
                screen.set_at((sx, sy), COLOR_SCAN)
                if sx + 1 < self.screen_w:
                    screen.set_at((sx + 1, sy), COLOR_SCAN)
                if sy + 1 < self.screen_h:
                    screen.set_at((sx, sy + 1), COLOR_SCAN)

    def _draw_trail(self, screen):
        if not self.show_trail:
            return
        with self.lock:
            points = list(self.trail_points)
        if len(points) < 2:
            return
        screen_points = [self.world_to_screen(wx, wy) for wx, wy in points]
        try:
            pygame.draw.lines(screen, COLOR_TRAIL, False, screen_points, 2)
        except Exception:
            pass

    def _draw_robot(self, screen):
        with self.lock:
            rx = self.robot_x
            ry = self.robot_y
            yaw = self.robot_yaw
            has_odom = self.odom_received
        if not has_odom:
            return

        cx, cy = self.world_to_screen(rx, ry)
        size = ROBOT_SIZE
        tip_x = cx + int(size * 1.5 * math.cos(-yaw))
        tip_y = cy + int(size * 1.5 * math.sin(-yaw))
        left_x = cx + int(size * math.cos(-yaw + 2.5))
        left_y = cy + int(size * math.sin(-yaw + 2.5))
        right_x = cx + int(size * math.cos(-yaw - 2.5))
        right_y = cy + int(size * math.sin(-yaw - 2.5))

        try:
            pygame.draw.polygon(screen, COLOR_ROBOT,
                                [(tip_x, tip_y), (left_x, left_y), (right_x, right_y)])
            pygame.draw.polygon(screen, (0, 100, 0),
                                [(tip_x, tip_y), (left_x, left_y), (right_x, right_y)], 1)
        except Exception:
            pass

    def _draw_status(self, screen):
        bar_h = 40
        bar_y = self.screen_h - bar_h

        bar_surf = pygame.Surface((self.screen_w, bar_h))
        bar_surf.set_alpha(200)
        bar_surf.fill((20, 20, 20))
        screen.blit(bar_surf, (0, bar_y))

        now = time.time()
        with self.lock:
            rx = self.robot_x
            ry = self.robot_y
            vx = self.robot_vx
            wz = self.robot_wz
            mw = self.map_width
            mh = self.map_height
            sc = self.scan_count
            has_map = self.map_surface is not None
            has_odom = self.odom_received
            has_scan = self.scan_received
            odom_age = now - self.odom_time if has_odom else 999
            scan_age = now - self.scan_time if has_scan else 999
            trail_len = len(self.trail_points)

        line1 = f"Pos: ({rx:.2f}, {ry:.2f})  v={abs(vx):.2f} m/s  w={math.degrees(wz):.1f} d/s  Trail: {trail_len} pts"
        map_str = f"Map: {mw}x{mh}" if has_map else "Map: waiting..."
        scan_str = f"Scan: {sc} pts" if has_scan else "Scan: waiting..."
        conn_str = "WS:OK" if self.connected else f"WS:ERR"
        zoom_str = f"x{self.zoom:.1f}"
        follow_str = "[F]" if self.follow_robot else ""
        trail_str = "[T]" if self.show_trail else ""
        line2 = f"{map_str}  {scan_str}  {conn_str}  {zoom_str} {follow_str} {trail_str}"

        y1 = bar_y + 4
        y2 = bar_y + 22

        if self.font_small:
            try:
                t1 = self.font_small.render(line1, True, COLOR_TEXT)
                screen.blit(t1, (8, y1))
                t2 = self.font_small.render(line2, True, COLOR_TEXT)
                screen.blit(t2, (8, y2))
            except Exception:
                pass

        # Health indicators (top-right)
        ix = self.screen_w - 12
        iy = 6
        r = 5
        c = COLOR_STATUS_OK if has_map else COLOR_STATUS_ERR
        pygame.draw.circle(screen, c, (ix - 30, iy + r), r)
        c = COLOR_STATUS_OK if (has_odom and odom_age < 2.0) else (
            COLOR_STATUS_WARN if has_odom else COLOR_STATUS_ERR)
        pygame.draw.circle(screen, c, (ix - 15, iy + r), r)
        c = COLOR_STATUS_OK if (has_scan and scan_age < 2.0) else (
            COLOR_STATUS_WARN if has_scan else COLOR_STATUS_ERR)
        pygame.draw.circle(screen, c, (ix, iy + r), r)

    def _draw_waiting_screen(self, screen):
        screen.fill(COLOR_BG)
        if self.font:
            lines = [
                "SLAM Visualizer - ROSMaster R2",
                "",
                f"Connecting to rosbridge ws://{self.ros_host}:{self.ros_port} ...",
                f"Status: {'Connected' if self.connected else self.connect_error or 'Connecting...'}",
                "",
                "Waiting for data on topics:",
                "  /map  (nav_msgs/OccupancyGrid)",
                "  /odom (nav_msgs/Odometry)",
                "  /scan (sensor_msgs/LaserScan)",
                "",
                "Keys: +/- zoom, c center, f follow, t trail, q quit",
            ]
            y = 60
            for line in lines:
                try:
                    t = self.font.render(line, True, COLOR_TEXT)
                    screen.blit(t, (30, y))
                except Exception:
                    pass
                y += 24

    # -------------------------------------------------------------------
    # ROS connection (background thread)
    # -------------------------------------------------------------------
    def _ros_thread(self):
        while self.running:
            try:
                self.connect_error = "Connecting..."
                client = roslibpy.Ros(host=self.ros_host, port=self.ros_port)
                client.run(timeout=5)

                self.ros_client = client
                self.connected = True
                self.connect_error = ""
                print(f"[rosbridge] Connected to ws://{self.ros_host}:{self.ros_port}")

                # Subscribe to topics
                map_sub = roslibpy.Topic(client, '/map', 'nav_msgs/OccupancyGrid',
                                         queue_size=1)
                map_sub.subscribe(self._map_callback)

                odom_sub = roslibpy.Topic(client, '/odom', 'nav_msgs/Odometry',
                                          queue_size=1)
                odom_sub.subscribe(self._odom_callback)

                scan_sub = roslibpy.Topic(client, '/scan', 'sensor_msgs/LaserScan',
                                          queue_size=1)
                scan_sub.subscribe(self._scan_callback)

                print("[rosbridge] Subscribed to /map, /odom, /scan")

                # Keep alive
                while self.running and client.is_connected:
                    time.sleep(0.5)

                # Disconnected
                self.connected = False
                self.connect_error = "Disconnected, reconnecting..."
                print("[rosbridge] Disconnected, will retry...")
                try:
                    client.terminate()
                except Exception:
                    pass

            except Exception as e:
                self.connected = False
                self.connect_error = f"Error: {e}"
                print(f"[rosbridge] Connection error: {e}")
                time.sleep(3)

    # -------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------
    def run(self):
        pygame.init()

        # Use full screen on Pi's display
        try:
            self.screen = pygame.display.set_mode(
                (SCREEN_WIDTH, SCREEN_HEIGHT), pygame.NOFRAME)
        except pygame.error:
            self.screen = pygame.display.set_mode(
                (SCREEN_WIDTH, SCREEN_HEIGHT))

        pygame.display.set_caption("SLAM Visualizer - ROSMaster R2")
        self.screen_w = self.screen.get_width()
        self.screen_h = self.screen.get_height()

        pygame.font.init()
        try:
            font_name = pygame.font.match_font("dejavusansmono,liberationmono,mono")
            if font_name:
                self.font = pygame.font.Font(font_name, 16)
                self.font_small = pygame.font.Font(font_name, 14)
            else:
                self.font = pygame.font.SysFont(None, 18)
                self.font_small = pygame.font.SysFont(None, 16)
        except Exception:
            self.font = pygame.font.SysFont(None, 18)
            self.font_small = pygame.font.SysFont(None, 16)

        clock = pygame.time.Clock()
        self.running = True

        # Start ROS connection thread
        ros_t = threading.Thread(target=self._ros_thread, daemon=True)
        ros_t.start()

        print(f"[visualizer] pygame initialized ({self.screen_w}x{self.screen_h})")
        print(f"[visualizer] Connecting to rosbridge ws://{self.ros_host}:{self.ros_port}")

        try:
            while self.running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        self._handle_key(event.key)

                if self.follow_robot:
                    with self.lock:
                        if self.odom_received:
                            self.center_x = self.robot_x
                            self.center_y = self.robot_y

                self.screen.fill(COLOR_BG)

                with self.lock:
                    has_any_data = (self.map_surface is not None or
                                   self.odom_received or
                                   self.scan_received)

                if has_any_data:
                    self._draw_map(self.screen)
                    self._draw_trail(self.screen)
                    self._draw_scan(self.screen)
                    self._draw_robot(self.screen)
                    self._draw_status(self.screen)
                else:
                    self._draw_waiting_screen(self.screen)

                pygame.display.flip()
                clock.tick(FPS_LIMIT)

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            pygame.quit()
            if self.ros_client:
                try:
                    self.ros_client.terminate()
                except Exception:
                    pass
            print("[visualizer] Shutdown.")

    def _handle_key(self, key):
        if key in (pygame.K_q, pygame.K_ESCAPE):
            self.running = False
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.zoom = min(self.zoom * 1.3, 20.0)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.zoom = max(self.zoom / 1.3, 0.1)
        elif key == pygame.K_c:
            with self.lock:
                if self.odom_received:
                    self.center_x = self.robot_x
                    self.center_y = self.robot_y
        elif key == pygame.K_f:
            self.follow_robot = not self.follow_robot
            print(f"[visualizer] Follow mode: {'ON' if self.follow_robot else 'OFF'}")
        elif key == pygame.K_t:
            self.show_trail = not self.show_trail
            print(f"[visualizer] Trail display: {'ON' if self.show_trail else 'OFF'}")


def main():
    parser = argparse.ArgumentParser(description='SLAM Visualizer (Host)')
    parser.add_argument('--host', default='localhost', help='rosbridge host')
    parser.add_argument('--port', type=int, default=9090, help='rosbridge port')
    args = parser.parse_args()

    signal.signal(signal.SIGINT, lambda s, f: None)
    viz = SlamVisualizer(host=args.host, port=args.port)
    viz.run()


if __name__ == "__main__":
    main()
