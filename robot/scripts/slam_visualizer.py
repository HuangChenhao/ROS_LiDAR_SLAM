#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SLAM 实时可视化工具 - Yahboom ROSMaster R2
Real-time SLAM visualizer using pygame for Raspberry Pi 5 small screen.

用法（在 Pi 上）:
  export DISPLAY=:0
  docker exec -e DISPLAY=:0 rosmaster_slam python /root/yahboomcar_ws/src/yahboomcar_bringup/scripts/slam_visualizer.py

键盘控制 / Keyboard controls:
  +/=    放大 / Zoom in
  -      缩小 / Zoom out
  c      重新居中到机器人 / Re-center on robot
  f      切换跟随模式 / Toggle follow mode
  t      切换轨迹显示 / Toggle trajectory display
  q/ESC  退出 / Quit

依赖 / Dependencies (inside Docker):
  pip install pygame
  (rospy, sensor_msgs, nav_msgs should already be available)
"""

from __future__ import print_function, division

import sys
import math
import time
import threading
import signal

try:
    import pygame
except ImportError:
    print("ERROR: pygame not installed. Run: pip install pygame")
    sys.exit(1)

try:
    import rospy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
except ImportError:
    print("ERROR: ROS packages not found. Make sure ROS environment is sourced.")
    print("  source /opt/ros/melodic/setup.bash")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
DEFAULT_WIDTH = 480
DEFAULT_HEIGHT = 320
FPS_LIMIT = 30
OVERLAY_FPS = 15  # Rate for scan/odom overlay redraws between map updates

# Colors (R, G, B)
COLOR_BG = (128, 128, 128)       # Gray background (unknown space)
COLOR_FREE = (255, 255, 255)     # White (free space)
COLOR_OCCUPIED = (0, 0, 0)       # Black (occupied/wall)
COLOR_UNKNOWN = (128, 128, 128)  # Gray (unknown)
COLOR_ROBOT = (0, 200, 0)        # Green (robot triangle)
COLOR_TRAIL = (200, 40, 40)      # Red (trajectory)
COLOR_SCAN = (40, 100, 255)      # Blue (laser scan points)
COLOR_TEXT_BG = (0, 0, 0, 180)   # Semi-transparent black for status bar
COLOR_TEXT = (220, 220, 220)     # Light gray text
COLOR_STATUS_OK = (80, 255, 80)  # Green status indicator
COLOR_STATUS_WARN = (255, 200, 40)  # Yellow status indicator
COLOR_STATUS_ERR = (255, 60, 60)   # Red status indicator

# Robot triangle size in pixels
ROBOT_SIZE = 10

# Trajectory: keep at most this many points to limit memory
MAX_TRAIL_POINTS = 5000

# Minimum distance (meters) between trail points to avoid clutter
MIN_TRAIL_DISTANCE = 0.02


# ---------------------------------------------------------------------------
# Helper: quaternion to yaw (avoid tf dependency which may not be available)
# ---------------------------------------------------------------------------
def quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw angle from a quaternion. Returns radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Main Visualizer Class
# ---------------------------------------------------------------------------
class SlamVisualizer(object):
    def __init__(self):
        # --- State protected by lock ---
        self.lock = threading.Lock()

        # Map data
        self.map_data = None        # raw OccupancyGrid message
        self.map_surface = None     # pygame.Surface of rendered map
        self.map_width = 0          # grid cells
        self.map_height = 0
        self.map_resolution = 0.05  # meters per cell
        self.map_origin_x = 0.0     # meters (world frame)
        self.map_origin_y = 0.0
        self.map_updated = False

        # Odometry
        self.robot_x = 0.0   # meters, world frame
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # radians
        self.robot_vx = 0.0   # m/s linear
        self.robot_wz = 0.0   # rad/s angular
        self.odom_received = False
        self.odom_time = 0.0

        # Laser scan
        self.scan_points = []   # list of (x, y) in world frame
        self.scan_count = 0
        self.scan_received = False
        self.scan_time = 0.0

        # Trajectory
        self.trail_points = []  # list of (x, y) in world frame

        # --- View state (main thread only) ---
        self.zoom = 1.0          # pixels per meter = zoom / resolution
        self.pixels_per_meter = 20.0  # base scale
        self.center_x = 0.0     # world x at screen center
        self.center_y = 0.0
        self.follow_robot = True
        self.show_trail = True

        # --- Timing ---
        self.last_overlay_time = 0.0

        # --- pygame ---
        self.screen = None
        self.font = None
        self.font_small = None
        self.running = False
        self.screen_w = DEFAULT_WIDTH
        self.screen_h = DEFAULT_HEIGHT

    # -------------------------------------------------------------------
    # ROS Callbacks (called from rospy spinner threads)
    # -------------------------------------------------------------------
    def _map_callback(self, msg):
        """Handle /map (OccupancyGrid) messages."""
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # Build a pygame surface from occupancy data
        # OccupancyGrid data: -1 = unknown, 0 = free, 100 = occupied
        surf = pygame.Surface((w, h))
        # Use a pixel array for speed
        try:
            pxa = pygame.PixelArray(surf)
        except Exception:
            # Fallback: set_at (slower)
            for j in range(h):
                for i in range(w):
                    val = msg.data[j * w + i]
                    if val < 0:
                        color = COLOR_UNKNOWN
                    elif val < 50:
                        # Gradient: 0=white, 49=light gray
                        g = 255 - int(val * 3)
                        color = (g, g, g)
                    else:
                        # 50-100: dark gray to black
                        g = max(0, int((100 - val) * 2.5))
                        color = (g, g, g)
                    surf.set_at((i, h - 1 - j), color)
        else:
            for j in range(h):
                row_offset = j * w
                # Flip Y: ROS map origin is bottom-left, pygame is top-left
                py = h - 1 - j
                for i in range(w):
                    val = msg.data[row_offset + i]
                    if val < 0:
                        color = COLOR_UNKNOWN
                    elif val < 50:
                        g = 255 - int(val * 3)
                        color = (g, g, g)
                    else:
                        g = max(0, int((100 - val) * 2.5))
                        color = (g, g, g)
                    pxa[i][py] = color
            pxa.close()

        with self.lock:
            self.map_surface = surf
            self.map_width = w
            self.map_height = h
            self.map_resolution = res
            self.map_origin_x = ox
            self.map_origin_y = oy
            self.map_updated = True

    def _odom_callback(self, msg):
        """Handle /odom (Odometry) messages."""
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        with self.lock:
            self.robot_x = px
            self.robot_y = py
            self.robot_yaw = yaw
            self.robot_vx = vx
            self.robot_wz = wz
            self.odom_received = True
            self.odom_time = time.time()

            # Add to trail
            if len(self.trail_points) == 0:
                self.trail_points.append((px, py))
            else:
                lx, ly = self.trail_points[-1]
                dist = math.sqrt((px - lx) ** 2 + (py - ly) ** 2)
                if dist >= MIN_TRAIL_DISTANCE:
                    self.trail_points.append((px, py))
                    if len(self.trail_points) > MAX_TRAIL_POINTS:
                        # Drop oldest 10% to avoid frequent trimming
                        drop = MAX_TRAIL_POINTS // 10
                        self.trail_points = self.trail_points[drop:]

    def _scan_callback(self, msg):
        """Handle /scan (LaserScan) messages. Convert to world-frame points."""
        with self.lock:
            rx = self.robot_x
            ry = self.robot_y
            ryaw = self.robot_yaw

        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Transform from laser frame to world frame
                # Assuming laser is at robot center (common for this platform)
                wx = rx + r * math.cos(ryaw + angle)
                wy = ry + r * math.sin(ryaw + angle)
                points.append((wx, wy))
            angle += msg.angle_increment

        with self.lock:
            self.scan_points = points
            self.scan_count = len(points)
            self.scan_received = True
            self.scan_time = time.time()

    # -------------------------------------------------------------------
    # Coordinate transforms
    # -------------------------------------------------------------------
    def world_to_screen(self, wx, wy):
        """Convert world coordinates (meters) to screen pixel coordinates."""
        scale = self.pixels_per_meter * self.zoom
        sx = int((wx - self.center_x) * scale + self.screen_w / 2.0)
        sy = int((self.center_y - wy) * scale + self.screen_h / 2.0)  # Y flipped
        return sx, sy

    # -------------------------------------------------------------------
    # Drawing
    # -------------------------------------------------------------------
    def _draw_map(self, screen):
        """Draw the occupancy grid map."""
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

        # Scale the map surface to screen size
        scaled_w = max(1, int(mw * cell_screen_size))
        scaled_h = max(1, int(mh * cell_screen_size))

        # Don't scale if extremely large (memory protection)
        if scaled_w > 4000 or scaled_h > 4000:
            return

        try:
            scaled = pygame.transform.scale(surf, (scaled_w, scaled_h))
        except Exception:
            return

        # Map origin in screen coords
        # Map origin is bottom-left in ROS, but our surface is already flipped
        # So top-left of surface corresponds to (ox, oy + mh*res) in world
        top_left_wx = ox
        top_left_wy = oy + mh * res
        sx, sy = self.world_to_screen(top_left_wx, top_left_wy)

        screen.blit(scaled, (sx, sy))

    def _draw_scan(self, screen):
        """Draw laser scan points as blue dots."""
        with self.lock:
            points = list(self.scan_points)

        if not points:
            return

        for wx, wy in points:
            sx, sy = self.world_to_screen(wx, wy)
            if 0 <= sx < self.screen_w and 0 <= sy < self.screen_h:
                # Draw 2px dot for visibility on small screen
                screen.set_at((sx, sy), COLOR_SCAN)
                if sx + 1 < self.screen_w:
                    screen.set_at((sx + 1, sy), COLOR_SCAN)
                if sy + 1 < self.screen_h:
                    screen.set_at((sx, sy + 1), COLOR_SCAN)

    def _draw_trail(self, screen):
        """Draw robot trajectory as a red line."""
        if not self.show_trail:
            return

        with self.lock:
            points = list(self.trail_points)

        if len(points) < 2:
            return

        screen_points = []
        for wx, wy in points:
            sx, sy = self.world_to_screen(wx, wy)
            screen_points.append((sx, sy))

        # Draw as connected line segments
        try:
            pygame.draw.lines(screen, COLOR_TRAIL, False, screen_points, 1)
        except Exception:
            pass  # Can fail if fewer than 2 points on screen

    def _draw_robot(self, screen):
        """Draw robot as a green triangle pointing in heading direction."""
        with self.lock:
            rx = self.robot_x
            ry = self.robot_y
            yaw = self.robot_yaw
            has_odom = self.odom_received

        if not has_odom:
            return

        cx, cy = self.world_to_screen(rx, ry)

        # Triangle: tip at front, two corners at back
        size = ROBOT_SIZE
        tip_x = cx + int(size * 1.5 * math.cos(-yaw))
        tip_y = cy + int(size * 1.5 * math.sin(-yaw))

        left_angle = -yaw + 2.5  # ~143 degrees back
        right_angle = -yaw - 2.5
        left_x = cx + int(size * math.cos(left_angle))
        left_y = cy + int(size * math.sin(left_angle))
        right_x = cx + int(size * math.cos(right_angle))
        right_y = cy + int(size * math.sin(right_angle))

        try:
            pygame.draw.polygon(screen, COLOR_ROBOT,
                                [(tip_x, tip_y), (left_x, left_y), (right_x, right_y)])
            pygame.draw.polygon(screen, (0, 100, 0),
                                [(tip_x, tip_y), (left_x, left_y), (right_x, right_y)], 1)
        except Exception:
            pass

    def _draw_status(self, screen):
        """Draw status bar at bottom of screen."""
        bar_h = 36
        bar_y = self.screen_h - bar_h

        # Semi-transparent background
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

        # Line 1: Position and speed
        speed = abs(vx)
        line1 = "Pos: (%.2f, %.2f)  v=%.2f m/s  w=%.1f d/s" % (
            rx, ry, speed, math.degrees(wz))

        # Line 2: Map info and status indicators
        if has_map:
            map_str = "Map: %dx%d" % (mw, mh)
        else:
            map_str = "Map: waiting..."

        scan_str = "Scan: %d pts" % sc if has_scan else "Scan: waiting..."
        zoom_str = "x%.1f" % self.zoom
        follow_str = "F" if self.follow_robot else ""

        line2 = "%s  %s  %s %s" % (map_str, scan_str, zoom_str, follow_str)

        # Status indicator dots
        y1 = bar_y + 2
        y2 = bar_y + 18

        if self.font_small:
            try:
                t1 = self.font_small.render(line1, True, COLOR_TEXT)
                screen.blit(t1, (4, y1))
            except Exception:
                pass
            try:
                t2 = self.font_small.render(line2, True, COLOR_TEXT)
                screen.blit(t2, (4, y2))
            except Exception:
                pass

        # Topic health indicators (top-right corner)
        indicator_x = self.screen_w - 10
        indicator_y = 4
        radius = 4
        # Map
        c = COLOR_STATUS_OK if has_map else COLOR_STATUS_ERR
        pygame.draw.circle(screen, c, (indicator_x - 24, indicator_y + radius), radius)
        # Odom
        c = COLOR_STATUS_OK if (has_odom and odom_age < 2.0) else (
            COLOR_STATUS_WARN if has_odom else COLOR_STATUS_ERR)
        pygame.draw.circle(screen, c, (indicator_x - 12, indicator_y + radius), radius)
        # Scan
        c = COLOR_STATUS_OK if (has_scan and scan_age < 2.0) else (
            COLOR_STATUS_WARN if has_scan else COLOR_STATUS_ERR)
        pygame.draw.circle(screen, c, (indicator_x, indicator_y + radius), radius)

    def _draw_waiting_screen(self, screen):
        """Draw a waiting screen when no data is available yet."""
        screen.fill(COLOR_BG)
        if self.font:
            lines = [
                "SLAM Visualizer - Waiting for data...",
                "",
                "Topics needed:",
                "  /map  (nav_msgs/OccupancyGrid)",
                "  /odom (nav_msgs/Odometry)",
                "  /scan (sensor_msgs/LaserScan)",
                "",
                "Keys: +/- zoom, c center, f follow, t trail, q quit",
            ]
            y = 40
            for line in lines:
                try:
                    t = self.font_small.render(line, True, COLOR_TEXT)
                    screen.blit(t, (20, y))
                except Exception:
                    pass
                y += 16

    # -------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------
    def run(self):
        """Initialize pygame and ROS, then run the main display loop."""
        # --- Initialize ROS ---
        rospy.init_node("slam_visualizer", anonymous=True, disable_signals=True)

        rospy.Subscriber("/map", OccupancyGrid, self._map_callback,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber("/odom", Odometry, self._odom_callback,
                         queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._scan_callback,
                         queue_size=1)

        rospy.loginfo("SLAM Visualizer: subscribed to /map, /odom, /scan")

        # --- Initialize pygame ---
        pygame.init()

        # Try to use the framebuffer directly if no X display
        try:
            self.screen = pygame.display.set_mode(
                (DEFAULT_WIDTH, DEFAULT_HEIGHT), pygame.RESIZABLE)
        except pygame.error:
            # Try framebuffer driver for headless Pi
            import os
            for driver in ['fbcon', 'directfb', 'svgalib']:
                os.environ['SDL_VIDEODRIVER'] = driver
                try:
                    pygame.display.init()
                    self.screen = pygame.display.set_mode(
                        (DEFAULT_WIDTH, DEFAULT_HEIGHT))
                    break
                except pygame.error:
                    continue
            else:
                rospy.logerr("No suitable video driver found!")
                return

        pygame.display.set_caption("SLAM Visualizer - ROSMaster R2")
        self.screen_w = self.screen.get_width()
        self.screen_h = self.screen.get_height()

        # Initialize fonts
        pygame.font.init()
        try:
            # Try to find a monospace font
            font_name = pygame.font.match_font("dejavusansmono,liberationmono,mono")
            if font_name:
                self.font = pygame.font.Font(font_name, 14)
                self.font_small = pygame.font.Font(font_name, 12)
            else:
                self.font = pygame.font.SysFont(None, 16)
                self.font_small = pygame.font.SysFont(None, 14)
        except Exception:
            self.font = pygame.font.SysFont(None, 16)
            self.font_small = pygame.font.SysFont(None, 14)

        clock = pygame.time.Clock()
        self.running = True

        rospy.loginfo("SLAM Visualizer: pygame initialized (%dx%d)",
                      self.screen_w, self.screen_h)

        try:
            while self.running and not rospy.is_shutdown():
                # --- Handle events ---
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        self._handle_key(event.key)
                    elif event.type == pygame.VIDEORESIZE:
                        self.screen_w = event.w
                        self.screen_h = event.h
                        self.screen = pygame.display.set_mode(
                            (event.w, event.h), pygame.RESIZABLE)

                # --- Update view center if following robot ---
                if self.follow_robot:
                    with self.lock:
                        if self.odom_received:
                            self.center_x = self.robot_x
                            self.center_y = self.robot_y

                # --- Draw ---
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
            pygame.quit()
            rospy.loginfo("SLAM Visualizer: shutdown.")

    def _handle_key(self, key):
        """Handle keyboard input."""
        if key in (pygame.K_q, pygame.K_ESCAPE):
            self.running = False
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.zoom = min(self.zoom * 1.3, 20.0)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.zoom = max(self.zoom / 1.3, 0.1)
        elif key == pygame.K_c:
            # Re-center on robot
            with self.lock:
                if self.odom_received:
                    self.center_x = self.robot_x
                    self.center_y = self.robot_y
        elif key == pygame.K_f:
            self.follow_robot = not self.follow_robot
            rospy.loginfo("Follow mode: %s", "ON" if self.follow_robot else "OFF")
        elif key == pygame.K_t:
            self.show_trail = not self.show_trail
            rospy.loginfo("Trail display: %s", "ON" if self.show_trail else "OFF")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, lambda s, f: None)

    viz = SlamVisualizer()
    viz.run()


if __name__ == "__main__":
    main()
