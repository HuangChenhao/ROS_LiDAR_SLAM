#!/usr/bin/env python3
# encoding: utf-8
# map_kiosk.py — 小车屏幕实时建图显示 (跑在 Pi host 上, 用户 pi, XWayland)
#
# 逻辑:
#   - 每 0.5s 从容器拉取 /root/rosmaster_maps/live_preview.pgm (supervisor 建图时生成)
#   - 文件存在   -> 全屏窗口显示地图 (跟随 gmapping 刷新) + 屏幕常亮
#   - 文件不存在 -> 隐藏窗口 + 恢复原来的熄屏设置
#
# 常亮实现: 临时把 wayfire.ini [idle] 的 dpms_timeout/screensaver_timeout 设为 -1,
#           wayfire 会热加载配置; 退出显示时恢复原值。

import os
import re
import shutil
import subprocess
import time
import tkinter as tk
import json
import math

C = 'rosmaster_ros2'
SRC = '/root/rosmaster_maps/live_preview.ppm'
LOCAL = '/tmp/kiosk_map.ppm'
STATUS_SRC = '/root/rosmaster_maps/live_status.json'
STATUS_LOCAL = '/tmp/kiosk_status.json'
POLL_MS = 500
WAYFIRE_INI = os.path.expanduser('~/.config/wayfire.ini')
IDLE_BACKUP = '/tmp/kiosk_idle_backup.txt'


def docker(*args):
    for pre in ([], ['sudo', '-n']):
        r = subprocess.run(pre + ['docker'] + list(args), capture_output=True)
        if r.returncode == 0:
            return True
        if b'permission denied' not in (r.stderr or b'').lower():
            return False
    return False


def fetch_preview():
    """Returns True if a preview map exists (mapping in progress)."""
    if os.path.exists(LOCAL):
        os.remove(LOCAL)
    ok = docker('cp', '{}:{}'.format(C, SRC), LOCAL)
    return ok and os.path.exists(LOCAL) and os.path.getsize(LOCAL) > 0


def fetch_status():
    """Return supervisor status; an empty dict is safe during file rotation."""
    try:
        if os.path.exists(STATUS_LOCAL):
            os.remove(STATUS_LOCAL)
        if not docker('cp', '{}:{}'.format(C, STATUS_SRC), STATUS_LOCAL):
            return {}
        with open(STATUS_LOCAL, encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return {}


def set_keep_awake(on):
    """Toggle wayfire idle timeouts (wayfire hot-reloads its config)."""
    try:
        if not os.path.exists(WAYFIRE_INI):
            return
        txt = open(WAYFIRE_INI).read()
        if on:
            saved = {}
            def repl(m):
                saved[m.group(1)] = m.group(2)
                return '{} = -1'.format(m.group(1))
            new = re.sub(r'^(dpms_timeout|screensaver_timeout)\s*=\s*(\S+)',
                         repl, txt, flags=re.M)
            if saved:
                with open(IDLE_BACKUP, 'w') as f:
                    for k, v in saved.items():
                        f.write('{}={}\n'.format(k, v))
                shutil.copy(WAYFIRE_INI, WAYFIRE_INI + '.kiosk_bak')
                open(WAYFIRE_INI, 'w').write(new)
        else:
            if os.path.exists(WAYFIRE_INI + '.kiosk_bak'):
                shutil.move(WAYFIRE_INI + '.kiosk_bak', WAYFIRE_INI)
                if os.path.exists(IDLE_BACKUP):
                    os.remove(IDLE_BACKUP)
    except Exception:
        pass


class Kiosk:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('R2 LIVE MAP')
        self.root.configure(bg='black')
        self.root.attributes('-fullscreen', True)
        self.label = tk.Label(self.root, bg='black')
        self.label.pack(expand=True)
        self.status = tk.Label(self.root, fg='#00ff00', bg='black',
                               font=('DejaVu Sans Mono', 14))
        self.status.pack(side='bottom', fill='x')
        self.img = None
        self.slam_status = {}
        self.showing = False
        self.root.withdraw()
        self.sw = self.root.winfo_screenwidth()
        self.sh = self.root.winfo_screenheight()
        self.poll()
        self.root.mainloop()

    def poll(self):
        try:
            if fetch_preview():
                self.slam_status = fetch_status()
                self.show()
            else:
                self.hide()
        except Exception:
            pass
        self.root.after(POLL_MS, self.poll)

    def show(self):
        try:
            img = tk.PhotoImage(file=LOCAL)
            raw_w, raw_h = img.width(), img.height()
            # Always fit the complete live map inside the display. The old
            # code only zoomed small maps and left a 1120x1024 GMapping map at
            # native size, so the newest cells and robot marker left screen.
            available_w = max(1, self.sw - 12)
            available_h = max(1, self.sh - 52)
            fit = min(available_w / max(raw_w, 1),
                      available_h / max(raw_h, 1))
            if fit < 1.0:
                shrink = max(1, int(math.ceil(1.0 / fit)))
                img = img.subsample(shrink, shrink)
            else:
                grow = max(1, int(math.floor(fit)))
                if grow > 1:
                    img = img.zoom(grow, grow)
            shown_w, shown_h = img.width(), img.height()
            self.img = img
            self.label.configure(image=self.img)
            algo = self.slam_status.get('algorithm_display', 'Unknown')
            selected = self.slam_status.get('selected_next')
            running = self.slam_status.get('algorithm')
            sensors = 'OK' if self.slam_status.get('sensors_ok') else 'FAULT'
            voltage = self.slam_status.get('voltage', 0.0)
            jumps = self.slam_status.get('pose_jumps', 0)
            next_text = ''
            if selected and running and selected != running:
                next_text = '  |  Next: {}'.format(selected)
            self.status.configure(
                text=' MAPPING LIVE  |  SLAM: {}  |  Gear: {}  |  Sensors: {} {:.1f}V  |  Jumps: {}  |  {}x{} -> {}x{}  |  {}{}'.format(
                    algo, self.slam_status.get('gear_display', '?'), sensors,
                    voltage, jumps, raw_w, raw_h, shown_w, shown_h,
                    time.strftime('%H:%M:%S'), next_text))
        except Exception:
            return
        if not self.showing:
            self.showing = True
            set_keep_awake(True)
            self.root.deiconify()
            self.root.attributes('-fullscreen', True)
            self.root.lift()

    def hide(self):
        if self.showing:
            self.showing = False
            self.root.withdraw()
            set_keep_awake(False)


if __name__ == '__main__':
    Kiosk()
