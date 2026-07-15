#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
view_scan.py — 读取/查看 ROSMaster R2 扫描数据 (纯标准库，无需安装依赖)

用法:
  python3 view_scan.py                # 列出本地所有扫描会话并打开最新一次
  python3 view_scan.py --list         # 只列出会话
  python3 view_scan.py 20260707       # 打开名称包含该关键词的会话
  python3 view_scan.py --live         # 实时连接小车，抓取当前 /map 并显示 (不存档)
  python3 view_scan.py --scale 4      # 放大倍数 (默认自动)

本地会话目录: scans/<扫描开始时间>/ (map.pgm + map.yaml + metadata.txt)
"""

import os
import re
import struct
import subprocess
import sys
import tempfile
import zlib

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCANS_DIR = os.path.join(SCRIPT_DIR, "scans")
SSH_KEY = os.path.expanduser("~/.ssh/rosmaster_codex_nopass")
SSH_OPTS = [
    "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
    "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=10",
    "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=6",
]
def _load_host():
    """R2_HOST 环境变量 > robot_config.sh > 默认值"""
    if os.environ.get("R2_HOST"):
        return os.environ["R2_HOST"]
    ip, user = "192.168.0.110", "pi"
    cfg = os.path.join(SCRIPT_DIR, "robot_config.sh")
    if os.path.exists(cfg):
        for line in open(cfg):
            m = re.match(r'^ROBOT_IP="?([^"\s]+)"?', line.strip())
            if m: ip = m.group(1)
            m = re.match(r'^ROBOT_USER="?([^"\s]+)"?', line.strip())
            if m: user = m.group(1)
    return f"{user}@{ip}"

HOST = _load_host()

def _scp_host():
    """scp 对 IPv6 地址要求方括号: pi@[fe80::...%en0]"""
    user, _, addr = HOST.partition("@")
    return f"{user}@[{addr}]" if ":" in addr else HOST


# ---------- PGM / YAML 解析 ----------

def read_pgm(path):
    """解析 P5 二进制 PGM，返回 (width, height, bytes)。"""
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(b"P5"):
        raise ValueError(f"{path} 不是 P5 PGM 文件")
    # 头部: P5 <w> <h> <maxval>，允许注释行
    tokens, pos = [], 2
    while len(tokens) < 3:
        while pos < len(data) and data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            while data[pos : pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos : pos + 1].isspace():
            pos += 1
        tokens.append(int(data[start:pos]))
    pos += 1  # 单个空白符后即像素数据
    w, h, _maxval = tokens
    pixels = data[pos : pos + w * h]
    if len(pixels) != w * h:
        raise ValueError(f"像素数据不完整: 期望 {w*h}, 实际 {len(pixels)}")
    return w, h, pixels


def read_map_yaml(path):
    """简易解析 map.yaml，返回 dict。"""
    info = {}
    if not os.path.exists(path):
        return info
    with open(path) as f:
        for line in f:
            m = re.match(r"^(\w+):\s*(.+)$", line.strip())
            if m:
                info[m.group(1)] = m.group(2)
    return info


# ---------- PNG 输出 (纯标准库) ----------

def write_png(path, w, h, gray_bytes, scale=1):
    """把 8-bit 灰度图写成 PNG，可选整数倍放大。"""
    if scale > 1:
        rows = []
        for y in range(h):
            row = gray_bytes[y * w : (y + 1) * w]
            big = b"".join(bytes([p]) * scale for p in row)
            rows.extend([big] * scale)
        w, h = w * scale, h * scale
    else:
        rows = [gray_bytes[y * w : (y + 1) * w] for y in range(h)]

    raw = b"".join(b"\x00" + r for r in rows)  # filter type 0 per row

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)  # 8-bit grayscale
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


# ---------- 会话管理 ----------

def list_sessions():
    if not os.path.isdir(SCANS_DIR):
        return []
    out = []
    for name in sorted(os.listdir(SCANS_DIR)):
        d = os.path.join(SCANS_DIR, name)
        if not os.path.isdir(d):
            continue
        pgm = next((os.path.join(d, f) for f in ("map.pgm", "map_STALE.pgm")
                    if os.path.exists(os.path.join(d, f))), None)
        out.append((name, d, pgm))
    return out


def print_sessions(sessions):
    if not sessions:
        print(f"scans/ 目录下还没有扫描会话 ({SCANS_DIR})")
        print("先运行: bash sync_scan.sh [会话名]")
        return
    print(f"本地扫描会话 ({len(sessions)} 个):")
    for name, _d, pgm in sessions:
        if pgm:
            size = os.path.getsize(pgm)
            tag = " [STALE!]" if "STALE" in os.path.basename(pgm) else ""
            print(f"  {name}  ({size//1024} KB){tag}")
        else:
            print(f"  {name}  (无地图文件 — 同步失败?)")


# ---------- 查看 ----------

def show_map(pgm_path, yaml_path=None, title="", scale=None, open_image=True):
    w, h, px = read_pgm(pgm_path)
    info = read_map_yaml(yaml_path) if yaml_path else {}
    res = float(info.get("resolution", 0.05))

    total = w * h
    occ = px.count(0)
    free = px.count(254)
    unknown = total - occ - free

    print(f"\n=== {title or os.path.basename(pgm_path)} ===")
    print(f"  像素尺寸 : {w} x {h}")
    print(f"  分辨率   : {res} m/px")
    print(f"  实际尺寸 : {w*res:.1f} m x {h*res:.1f} m")
    if "origin" in info:
        print(f"  原点     : {info['origin']}")
    print(f"  占用/空闲/未知 : {occ} ({100*occ/total:.1f}%) / "
          f"{free} ({100*free/total:.1f}%) / {unknown} ({100*unknown/total:.1f}%)")

    if scale is None:
        scale = max(1, min(8, 1200 // max(w, h)))  # 自动放大到 ~1200px
    png_path = os.path.join(tempfile.gettempdir(),
                            os.path.basename(os.path.dirname(pgm_path) or "map") + ".png")
    write_png(png_path, w, h, px, scale=scale)
    print(f"  已生成 PNG (x{scale}): {png_path}")

    if open_image:
        try:
            if sys.platform == "darwin":
                subprocess.run(["open", png_path], check=False)
            elif sys.platform.startswith("linux"):
                subprocess.run(["xdg-open", png_path], check=False)
        except Exception:
            pass
    return png_path


def view_session(keyword=None, scale=None):
    sessions = [s for s in list_sessions() if s[2]]
    if not sessions:
        sys.exit(1)
    if keyword:
        matches = [s for s in sessions if keyword in s[0]]
        if not matches:
            print(f"找不到包含 '{keyword}' 的会话。")
            print_sessions(sessions)
            sys.exit(1)
        sel = matches[-1]
    else:
        sel = sessions[-1]  # 最新
    name, d, pgm = sel
    meta = os.path.join(d, "metadata.txt")
    if os.path.exists(meta):
        print(open(meta).read().strip())
    show_map(pgm, os.path.join(d, "map.yaml"), title=name, scale=scale)


# ---------- 实时模式 ----------

GRAB_SNIPPET = r'''
C="rosmaster_ros2"
S="source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROBOT_TYPE=r2 RPLIDAR_TYPE=a1"
docker exec "$C" bash -c "pkill -f map_live_grab" 2>/dev/null || true
docker exec "$C" bash -c "rm -f /tmp/live_map.*" 2>/dev/null || true
rm -f /tmp/live_map.* 2>/dev/null || true

# 建图中 (gmapping 在跑) -> 抓实时 /map; 等 12s 抓不到就走回退
MAPPING=0
if docker exec "$C" bash -c "pgrep -f slam_gmapping" > /dev/null 2>&1; then
    MAPPING=1
    timeout 30 docker exec "$C" bash -c "$S && python3 -u -c \"
import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
class M(Node):
    def __init__(self):
        super().__init__('map_live_grab')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, 1)
        self.done = False
    def cb(self, msg):
        if self.done: return
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data == 100] = 0
        img = np.flipud(img)
        with open('/tmp/live_map.pgm', 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            f.write(img.tobytes())
        with open('/tmp/live_map.yaml', 'w') as f:
            f.write(f'image: map.pgm\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        self.done = True
rclpy.init()
n = M()
t0 = time.time()
while not n.done and time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.5)
if not n.done:
    print('No /map data after 12s', flush=True)
\"" 2>&1
    docker cp "$C:/tmp/live_map.pgm" /tmp/live_map.pgm 2>/dev/null
    docker cp "$C:/tmp/live_map.yaml" /tmp/live_map.yaml 2>/dev/null
fi

if [ -s /tmp/live_map.pgm ]; then
    echo "SOURCE=live"
    echo LIVE_OK
else
    # 未激活 (红灯) 或抓取失败 -> 回退到小车上最新保存的会话
    [ $MAPPING -eq 0 ] && echo "NOTE: 未在建图 (红灯状态), 使用最新保存的会话"
    LATEST=$(docker exec "$C" bash -c "ls -1 /root/rosmaster_maps/ 2>/dev/null | grep -E '^[0-9]{8}_' | sort | tail -1")
    if [ -n "$LATEST" ] && docker exec "$C" bash -c "[ -s /root/rosmaster_maps/$LATEST/map.pgm ]"; then
        docker cp "$C:/root/rosmaster_maps/$LATEST/map.pgm" /tmp/live_map.pgm 2>/dev/null
        docker cp "$C:/root/rosmaster_maps/$LATEST/map.yaml" /tmp/live_map.yaml 2>/dev/null
        echo "SOURCE=saved:$LATEST"
        [ -s /tmp/live_map.pgm ] && echo LIVE_OK || echo LIVE_FAILED
    else
        echo "SOURCE=none"
        echo LIVE_FAILED
    fi
fi
'''


def view_live(scale=None):
    print(f"连接小车 {HOST} 抓取当前 /map ...")
    r = subprocess.run(["ssh", "-i", SSH_KEY, *SSH_OPTS, HOST, "bash"],
                       input=GRAB_SNIPPET, capture_output=True, text=True, timeout=90)
    out = r.stdout + r.stderr
    if "LIVE_OK" not in out:
        print("抓取失败。输出:")
        print(out.strip()[-2000:])
        print("\n检查: 小车开机? 同一 wifi? 或者还没有任何扫描会话?")
        sys.exit(1)

    m = re.search(r"SOURCE=(live|saved:(\S+))", out)
    source = m.group(1) if m else "?"
    if source == "live":
        title = "LIVE — 实时建图中"
    elif source.startswith("saved:"):
        title = f"最新保存的会话 — {source.split(':', 1)[1]} (当前未在建图)"
    else:
        title = "小车地图"

    tmp = tempfile.mkdtemp(prefix="live_scan_")
    for fn in ("live_map.pgm", "live_map.yaml"):
        subprocess.run(["scp", "-i", SSH_KEY, *SSH_OPTS,
                        f"{_scp_host()}:/tmp/{fn}", os.path.join(tmp, fn)],
                       capture_output=True, timeout=60)
    pgm = os.path.join(tmp, "live_map.pgm")
    if not os.path.exists(pgm) or os.path.getsize(pgm) == 0:
        print("scp 拉取失败")
        sys.exit(1)
    show_map(pgm, os.path.join(tmp, "live_map.yaml"), title=title, scale=scale)
    if source == "live":
        print("\n提示: 实时预览未存档。结束建图 (切红灯) 会自动保存, 之后 bash sync_scan.sh 拉取。")
    else:
        print("\n提示: 当前未在建图 (红灯)。按 Back 激活即开始新会话。")


# ---------- main ----------

def main():
    args = sys.argv[1:]
    scale = None
    if "--scale" in args:
        i = args.index("--scale")
        scale = int(args[i + 1])
        del args[i : i + 2]

    if "--list" in args:
        print_sessions(list_sessions())
    elif "--live" in args:
        view_live(scale=scale)
    else:
        keyword = args[0] if args else None
        print_sessions(list_sessions())
        view_session(keyword, scale=scale)


if __name__ == "__main__":
    main()
