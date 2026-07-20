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
            w, h = img.width(), img.height()
            f = max(1, min(self.sw // max(w, 1), (self.sh - 40) // max(h, 1)))
            if f > 1:
                img = img.zoom(f)
            self.img = img
            self.label.configure(image=self.img)
            algo = self.slam_status.get('algorithm_display', '未知算法')
            selected = self.slam_status.get('selected_next')
            running = self.slam_status.get('algorithm')
            next_text = ''
            if selected and running and selected != running:
                next_text = '  |  下次: {}'.format(selected)
            self.status.configure(
                text=' 建图中 LIVE  |  算法: {}  |  {}x{}  |  {}{}'.format(
                    algo, w, h, time.strftime('%H:%M:%S'), next_text))
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
