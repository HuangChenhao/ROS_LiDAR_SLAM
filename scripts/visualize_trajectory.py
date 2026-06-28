#!/usr/bin/env python3
"""
地图 + 轨迹可视化：将红色车辆轨迹叠加到 SLAM 地图上
用法: python3 ~/Claude/Projects/rosmaster/visualize_trajectory.py [map_dir]
默认: ~/Documents/rosmaster_r2/maps/
"""
import sys
import os
import csv
import yaml
import numpy as np
from pathlib import Path

def load_pgm(pgm_path):
    """加载 PGM 文件为 numpy 数组"""
    with open(pgm_path, 'rb') as f:
        # P5 binary PGM
        magic = f.readline().strip()
        assert magic == b'P5', f"不是 P5 PGM: {magic}"
        # 跳过注释
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline().strip())
        data = np.frombuffer(f.read(), dtype=np.uint8)
        return data.reshape(h, w), w, h, maxval

def main():
    map_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/Documents/rosmaster_r2/maps")
    map_dir = Path(map_dir)

    # 查找文件
    pgm_path = map_dir / "overall_map.pgm"
    yaml_path = map_dir / "overall_map.yaml"
    traj_path = map_dir / "trajectory.csv"

    if not pgm_path.exists():
        print(f"地图不存在: {pgm_path}")
        sys.exit(1)

    # 加载地图 yaml
    with open(yaml_path) as f:
        map_meta = yaml.safe_load(f)
    resolution = map_meta['resolution']  # m/pixel
    origin_x = map_meta['origin'][0]
    origin_y = map_meta['origin'][1]
    print(f"地图: {pgm_path}")
    print(f"  分辨率: {resolution} m/px, 原点: ({origin_x}, {origin_y})")

    # 加载 PGM
    pgm_data, w, h, maxval = load_pgm(str(pgm_path))
    print(f"  尺寸: {w}x{h}")

    # 转为 RGB
    # PGM: 205=unknown(灰), 254=free(白), 0=occupied(黑)
    rgb = np.stack([pgm_data, pgm_data, pgm_data], axis=-1)

    # 加载轨迹
    if not traj_path.exists():
        print(f"轨迹不存在: {traj_path}")
        print("保存无轨迹的地图...")
    else:
        trajectory = []
        with open(traj_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                trajectory.append((float(row['x']), float(row['y']), float(row['theta'])))
        print(f"  轨迹点数: {len(trajectory)}")

        if trajectory:
            # 世界坐标 → 像素坐标
            for x, y, theta in trajectory:
                px = int((x - origin_x) / resolution)
                py = h - 1 - int((y - origin_y) / resolution)  # PGM 上下翻转

                # 画 3x3 红色点（增加可见性）
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        ppx, ppy = px + dx, py + dy
                        if 0 <= ppx < w and 0 <= ppy < h:
                            rgb[ppy, ppx] = [255, 0, 0]  # 红色

            # 画起点（绿色，5x5）和终点（蓝色，5x5）
            sx, sy = trajectory[0][0], trajectory[0][1]
            ex, ey = trajectory[-1][0], trajectory[-1][1]
            for label, (wx, wy), color in [("起点", (sx, sy), [0, 255, 0]),
                                            ("终点", (ex, ey), [0, 0, 255])]:
                px = int((wx - origin_x) / resolution)
                py = h - 1 - int((wy - origin_y) / resolution)
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        ppx, ppy = px + dx, py + dy
                        if 0 <= ppx < w and 0 <= ppy < h:
                            rgb[ppy, ppx] = color
                print(f"  {label}: ({wx:.2f}, {wy:.2f}) → 像素({px}, {py})")

    # 保存为 PPM (不依赖 PIL)
    output_path = map_dir / "map_with_trajectory.ppm"
    with open(output_path, 'wb') as f:
        f.write(f'P6\n{w} {h}\n255\n'.encode())
        f.write(rgb.tobytes())
    print(f"\n已保存: {output_path}")

    # 尝试用 PIL 转 PNG（如果可用）
    try:
        from PIL import Image
        img = Image.fromarray(rgb)
        png_path = map_dir / "map_with_trajectory.png"
        img.save(str(png_path))
        print(f"已保存 PNG: {png_path}")
    except ImportError:
        print("提示: 安装 Pillow 可生成 PNG: pip3 install Pillow")

if __name__ == '__main__':
    main()
