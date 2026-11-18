#!/usr/bin/env python3
"""Inspect downloaded ROS2 scan sessions without requiring a ROS installation."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
import math
import re
import sqlite3
import statistics
import struct
from pathlib import Path

from PIL import Image, ImageDraw


DEFAULT_ROOT = Path(__file__).resolve().parent / "20260722_latest"
ROOT = DEFAULT_ROOT


def parse_metadata(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    text = path.read_bytes().replace(b"\x00", b"").decode("utf-8", "replace")
    result: dict[str, str] = {}
    for line in text.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            result[key.strip()] = value.strip()
    return result


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction)))
    return ordered[index]


def unpack_twist(data: bytes) -> tuple[float, ...] | None:
    # ROS2 CDR messages have a four-byte encapsulation header. Fast-CDR in
    # Foxy places the first double either immediately after it or at byte 8.
    endian = "<" if len(data) > 1 and data[1] == 1 else ">"
    candidates = []
    for offset in (4, 8):
        if len(data) < offset + 48:
            continue
        values = struct.unpack_from(endian + "6d", data, offset)
        if all(math.isfinite(value) and abs(value) < 100.0 for value in values):
            candidates.append(values)
    if not candidates:
        return None
    # A real Twist from this car has unused y/z components equal to zero.
    return min(candidates, key=lambda values: sum(abs(value) for value in values[1:3]))


def bag_stats(db_path: Path) -> dict[str, object]:
    if not db_path.exists():
        return {"topics": {}, "cmd": [], "vel": [], "duration_s": 0.0, "start_ns": 0}
    connection = sqlite3.connect(f"file:{db_path.as_posix()}?mode=ro", uri=True)
    try:
        topics = {
            row[0]: {"id": row[1], "type": row[2]}
            for row in connection.execute("SELECT name, id, type FROM topics")
        }
        counts = {
            row[0]: row[1]
            for row in connection.execute(
                "SELECT topics.name, COUNT(messages.id) "
                "FROM topics LEFT JOIN messages ON messages.topic_id=topics.id "
                "GROUP BY topics.id"
            )
        }
        bounds = connection.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
        duration_s = 0.0 if not bounds or bounds[0] is None else (bounds[1] - bounds[0]) / 1e9

        def read_twists(name: str) -> list[tuple[int, tuple[float, ...]]]:
            topic = topics.get(name)
            if not topic:
                return []
            rows = connection.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic["id"],),
            )
            result = []
            for timestamp, data in rows:
                values = unpack_twist(data)
                if values is not None:
                    result.append((timestamp, values))
            return result

        return {
            "topics": counts,
            "cmd": read_twists("/cmd_vel"),
            "vel": read_twists("/vel_raw"),
            "duration_s": duration_s,
            "start_ns": 0 if not bounds or bounds[0] is None else bounds[0],
        }
    finally:
        connection.close()


def infer_gear(
    cmd: list[tuple[int, tuple[float, ...]]], start_ns: int
) -> tuple[str, float, float, str]:
    speeds = [abs(values[0]) for _, values in cmd if abs(values[0]) > 0.015]
    if not speeds:
        return "stationary/unknown", 0.0, 0.0, ""
    maximum = max(speeds)
    p99 = percentile(speeds, 0.99)
    # Published gear limits: gear 1=0.17 m/s, gear 2=0.33 m/s. Use the
    # dominant category, then preserve brief X/racing bursts as secondary
    # evidence instead of mislabelling the whole session by its maximum.
    categories: dict[str, list[int]] = {"1": [], "2": [], "3/racing": []}
    cap_counts = {"1": 0, "2": 0, "3/racing": 0}
    for timestamp, values in cmd:
        speed = abs(values[0])
        if speed <= 0.015:
            continue
        category = "1" if speed <= 0.205 else ("2" if speed <= 0.38 else "3/racing")
        categories[category].append(timestamp)
        if abs(speed - 0.17) <= 0.006:
            cap_counts["1"] += 1
        if abs(speed - 0.33) <= 0.006:
            cap_counts["2"] += 1
        if speed > 0.38:
            cap_counts["3/racing"] += 1
    primary = max(cap_counts, key=cap_counts.get)
    order = {"1": 1, "2": 2, "3/racing": 3}
    secondary = [
        key for key, timestamps in categories.items()
        if order[key] > order[primary] and len(timestamps) >= 50
    ]
    gear = primary + (" + brief " + "/".join(secondary) if secondary else "")
    intervals = []
    for key, timestamps in categories.items():
        if not timestamps:
            continue
        first = (timestamps[0] - start_ns) / 1e9
        last = (timestamps[-1] - start_ns) / 1e9
        intervals.append("{}:{:.1f}-{:.1f}s(n={})".format(key, first, last, len(timestamps)))
    return gear, maximum, p99, "; ".join(intervals)


def recorded_gear(session: Path, metadata: dict[str, str], fallback: str) -> tuple[str, str]:
    """Prefer the exact post-upgrade gear record over speed inference."""
    labels: list[str] = []
    events_path = session / "gear_events.csv"
    if events_path.exists():
        with events_path.open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                label = row.get("gear", "").strip()
                if label and (not labels or label != labels[-1]):
                    labels.append(label)
    if labels:
        return " -> ".join(labels), "gear_events.csv"
    if metadata.get("start_gear"):
        return metadata["start_gear"], "metadata"
    return fallback, "cmd_vel inference"


def trajectory_stats(path: Path) -> dict[str, float | int]:
    if not path.exists() or path.stat().st_size == 0:
        return {"points": 0, "path_m": 0.0, "endpoint_m": 0.0, "max_step_m": 0.0}
    points = []
    with path.open(newline="", encoding="utf-8", errors="replace") as stream:
        for row in csv.reader(stream):
            try:
                points.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    if not points:
        return {"points": 0, "path_m": 0.0, "endpoint_m": 0.0, "max_step_m": 0.0}
    steps = [math.dist(a, b) for a, b in zip(points, points[1:])]
    return {
        "points": len(points),
        "path_m": sum(steps),
        "endpoint_m": math.dist(points[0], points[-1]),
        "max_step_m": max(steps, default=0.0),
    }


def log_stats(path: Path, algo: str) -> dict[str, int | float]:
    text = path.read_text("utf-8", "replace") if path.exists() else ""
    result: dict[str, int | float] = {
        "warnings": len(re.findall(r"\bWARN(?:ING)?\b", text, re.I)),
        "errors": len(re.findall(r"\bERROR\b|\bFATAL\b", text, re.I)),
    }
    if algo == "cartographer":
        matches = re.findall(
            r"differs by translation ([0-9.]+) rotation ([0-9.]+) with score ([0-9.]+)%",
            text,
        )
        translations = [float(item[0]) for item in matches]
        result.update(
            {
                "constraints": len(matches),
                "constraints_gt_1m": sum(value > 1.0 for value in translations),
                "max_constraint_translation_m": max(translations, default=0.0),
            }
        )
    elif algo == "slam_toolbox":
        result.update(
            {
                "scan_drops": len(re.findall(r"dropp", text, re.I)),
                "failed_matches": len(re.findall(r"fail.*match|match.*fail", text, re.I)),
                "transform_errors": len(re.findall(r"transform|extrapolat", text, re.I)),
            }
        )
    elif algo == "rtabmap":
        result.update(
            {
                "loop_events": len(re.findall(r"loop closure|loopClosureId", text, re.I)),
                "rejected": len(re.findall(r"reject", text, re.I)),
                "no_map_events": len(re.findall(r"no map|empty.*map", text, re.I)),
            }
        )
    elif algo == "gmapping":
        scores = [float(value) for value in re.findall(r"Average Scan Matching Score=([0-9.]+)", text)]
        result.update(
            {
                "score_samples": len(scores),
                "score_median": statistics.median(scores) if scores else 0.0,
                "scan_drops": len(re.findall(r"dropping message", text, re.I)),
            }
        )
    return result


def read_trajectory(path: Path) -> list[tuple[float, float]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    points = []
    with path.open(newline="", encoding="utf-8", errors="replace") as stream:
        for row in csv.reader(stream):
            try:
                points.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return points


def convert_map(session: Path) -> tuple[int, int]:
    pgm = session / "map.pgm"
    if not pgm.exists():
        return 0, 0
    image = Image.open(pgm)
    image.save(session / "map_preview.png")
    yaml_text = (session / "map.yaml").read_text("utf-8", "replace")
    resolution_match = re.search(r"resolution:\s*([-+0-9.eE]+)", yaml_text)
    origin_match = re.search(r"origin:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)", yaml_text)
    trajectory = read_trajectory(session / "trajectory.csv")
    if resolution_match and origin_match and trajectory:
        resolution = float(resolution_match.group(1))
        origin_x, origin_y = float(origin_match.group(1)), float(origin_match.group(2))
        overlay = image.convert("RGB")
        draw = ImageDraw.Draw(overlay)
        pixels = [
            ((x - origin_x) / resolution, overlay.height - 1 - (y - origin_y) / resolution)
            for x, y in trajectory
        ]
        if len(pixels) >= 2:
            draw.line(pixels, fill=(230, 35, 35), width=3)
        for point, color in ((pixels[0], (0, 180, 40)), (pixels[-1], (0, 90, 255))):
            x, y = point
            draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color)
        overlay.save(session / "diagnostic_overlay.png")
    return image.size


def make_contact_sheet(rows: list[dict[str, object]]) -> None:
    cell_w, cell_h = 520, 430
    sheet = Image.new("RGB", (cell_w * 2, cell_h * 3), "white")
    draw = ImageDraw.Draw(sheet)
    for index, row in enumerate(rows):
        session = ROOT / str(row["session"])
        source = session / "diagnostic_overlay.png"
        if not source.exists():
            source = session / "map_preview.png"
        x0, y0 = (index % 2) * cell_w, (index // 2) * cell_h
        title = "{} | {} | {}".format(
            row["session"], row["algorithm"], row["gear_inferred"]
        )
        draw.text((x0 + 10, y0 + 8), title, fill="black")
        if source.exists():
            image = Image.open(source).convert("RGB")
            image.thumbnail((cell_w - 20, cell_h - 45))
            left = x0 + (cell_w - image.width) // 2
            top = y0 + 35 + (cell_h - 40 - image.height) // 2
            sheet.paste(image, (left, top))
    sheet.save(ROOT / "scan_contact_sheet.png")


def main() -> None:
    global ROOT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root", nargs="?", type=Path, default=DEFAULT_ROOT,
        help="directory containing extracted scan-session folders")
    args = parser.parse_args()
    ROOT = args.root.resolve()
    rows = []
    for session in sorted(path for path in ROOT.iterdir() if path.is_dir()):
        meta = parse_metadata(session / "metadata.txt")
        algo = meta.get("algo", session.name.split("_", 2)[-1])
        bag = bag_stats(session / "bag" / "bag_0.db3")
        gear, max_cmd, p99_cmd, gear_intervals = infer_gear(
            bag["cmd"], int(bag["start_ns"]))
        gear, gear_source = recorded_gear(session, meta, gear)
        trajectory = trajectory_stats(session / "trajectory.csv")
        width, height = convert_map(session)
        logs = log_stats(session / "nodes.log", algo)
        row = {
            "session": session.name,
            "scan_start": meta.get("scan_start", ""),
            "scan_end": meta.get("scan_end", ""),
            "algorithm": algo,
            "gear_inferred": gear,
            "gear_source": gear_source,
            "gear_evidence_max_cmd_mps": round(max_cmd, 4),
            "gear_evidence_p99_cmd_mps": round(p99_cmd, 4),
            "gear_intervals_inferred": gear_intervals,
            "bag_duration_s": round(float(bag["duration_s"]), 3),
            "scan_messages": bag["topics"].get("/scan", 0),
            "odom_messages": bag["topics"].get("/odom", 0),
            "imu_messages": bag["topics"].get("/imu/data", 0),
            "map_width": width,
            "map_height": height,
            "status": meta.get("status", ""),
            **trajectory,
            **logs,
        }
        command_histogram = Counter(round(values[0], 3) for _, values in bag["cmd"])
        row["dominant_cmd_linear_x"] = "; ".join(
            "{}:{}".format(speed, count)
            for speed, count in command_histogram.most_common(8)
        )
        rows.append(row)

    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with (ROOT / "scan_index.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    make_contact_sheet(rows)

    print("\t".join(["session", "algo", "gear", "max_cmd", "duration", "scan", "odom", "traj_pts", "path_m", "endpoint_m", "map"]))
    for row in rows:
        print(
            "\t".join(
                [
                    str(row["session"]),
                    str(row["algorithm"]),
                    str(row["gear_inferred"]),
                    str(row["gear_evidence_max_cmd_mps"]),
                    str(row["bag_duration_s"]),
                    str(row["scan_messages"]),
                    str(row["odom_messages"]),
                    str(row["points"]),
                    str(round(float(row["path_m"]), 2)),
                    str(round(float(row["endpoint_m"]), 2)),
                    f"{row['map_width']}x{row['map_height']}",
                ]
            )
        )
        print("  cmd_linear_x: {}".format(row["dominant_cmd_linear_x"]))
    print(f"\nWrote {ROOT / 'scan_index.csv'}")


if __name__ == "__main__":
    main()
