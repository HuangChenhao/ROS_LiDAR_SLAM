#!/usr/bin/env python3
"""Audit local SLAM evidence without ROS, hardware access, or data modification."""

from __future__ import annotations

import argparse
import csv
from datetime import date
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sqlite3
import tarfile

from PIL import Image
import yaml


def trajectory(path: Path) -> dict:
    points = []
    if path.exists():
        for row in csv.reader(path.read_text('utf-8-sig', errors='replace').splitlines()):
            try:
                points.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return {
        'samples': len(points),
        'path_m': sum(math.dist(a, b) for a, b in zip(points, points[1:])),
        'endpoint_separation_m': math.dist(points[0], points[-1]) if points else None,
    }


def map_stats(image: Image.Image) -> dict:
    histogram = image.convert('L').histogram()
    return {
        'width': image.width,
        'height': image.height,
        'occupied_black_pixels': histogram[0],
        'unknown_205_pixels': histogram[205],
        'free_254_pixels': histogram[254],
    }


def read_bag(folder: Path) -> dict:
    databases = sorted(folder.glob('*.db3'))
    if not databases:
        return {'available': False, 'reason': 'raw SQLite data absent locally'}
    counts = {}
    checks = {}
    starts, ends = [], []
    for database in databases:
        connection = sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True)
        try:
            for name, count in connection.execute(
                'SELECT topics.name, COUNT(messages.id) FROM topics '
                'LEFT JOIN messages ON messages.topic_id=topics.id GROUP BY topics.id'
            ):
                counts[name] = counts.get(name, 0) + count
            start, end = connection.execute(
                'SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
            if start is not None:
                starts.append(start)
                ends.append(end)
            checks[database.name] = [row[0] for row in connection.execute('PRAGMA quick_check')]
        finally:
            connection.close()
    metadata_path = folder / 'metadata.yaml'
    parsed = yaml.safe_load(metadata_path.read_text('utf-8')) if metadata_path.exists() else {}
    bag_metadata = (parsed or {}).get('rosbag2_bagfile_information') or {}
    return {
        'available': True,
        'quick_check': checks,
        'duration_s': (max(ends) - min(starts)) / 1e9 if starts else 0.0,
        'message_count': sum(counts.values()),
        'metadata_message_count': bag_metadata.get('message_count'),
        'metadata_duration_s': (bag_metadata.get('duration') or {}).get('nanoseconds', 0) / 1e9,
        'topic_counts': counts,
        'camera_topics_with_data': {
            key: value for key, value in counts.items()
            if value and any(token in key for token in ('image', 'camera', 'rgbd'))
        },
    }


def read_session(metadata_path: Path) -> dict:
    folder = metadata_path.parent
    raw = metadata_path.read_bytes()
    metadata = {}
    for line in raw.replace(b'\0', b'').decode('utf-8', 'replace').splitlines():
        if ':' in line:
            key, value = line.split(':', 1)
            metadata[key.strip()] = value.strip()
    log_path = folder / 'nodes.log'
    log = log_path.read_text('utf-8', errors='replace') if log_path.exists() else ''
    result = {
        'session': folder.name,
        'group': folder.parent.name,
        'metadata': metadata,
        'metadata_nul_bytes': raw.count(b'\0'),
        'trajectory': trajectory(folder / 'trajectory.csv'),
        'optimized_trajectory': trajectory(folder / 'trajectory_optimized.csv'),
        'bag': read_bag(folder / 'bag'),
        'log': {
            'negative_hessian_index_warnings': log.lower().count('negative hessian index'),
            'probability_search_fatal': 'unable to get pointer in probability search' in log,
            # Text matches are evidence to inspect, never a successful closure count.
            'loop_related_lines': [line for line in log.splitlines()
                                   if re.search(r'loop closure|loopClosureId', line, re.I)],
        },
        'new_evidence_files': {
            name: (folder / name).exists() for name in (
                'finalization.json', 'configuration/sha256.json', 'map.pbstream',
                'pose_graph.posegraph', 'trajectory_optimized.csv', 'pose_graph_edges.csv')
        },
    }
    pgm = folder / 'map.pgm'
    if pgm.exists():
        with Image.open(pgm) as image:
            result['map'] = map_stats(image)
    database = folder / 'rtabmap.db'
    if database.exists():
        connection = sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True)
        try:
            result['rtabmap_database'] = {
                'link_types': dict(connection.execute('SELECT type, COUNT(*) FROM Link GROUP BY type')),
                'nodes': connection.execute('SELECT COUNT(*) FROM Node').fetchone()[0],
            }
        finally:
            connection.close()
    return result


def historical_exports(path: Path) -> list:
    if not path.exists():
        return []
    result = []
    with tarfile.open(path, 'r:gz') as archive:
        for member in archive.getmembers():
            if not member.isfile() or not member.name.endswith('.pgm'):
                continue
            data = archive.extractfile(member).read()
            with Image.open(io.BytesIO(data)) as image:
                info = map_stats(image)
            stem = member.name[:-4]
            pcd = archive.extractfile(stem + '.pcd').read().decode('ascii', 'replace')
            match = re.search(r'^POINTS\s+(\d+)', pcd, re.M)
            info.update(
                name=Path(stem).name,
                pgm_sha256=hashlib.sha256(data).hexdigest(),
                pcd_points=int(match.group(1)) if match else None,
                yaml=archive.extractfile(stem + '.yaml').read().decode('utf-8'),
            )
            result.append(info)
    return result


def manifest(root: Path) -> list:
    files = [path for path in (root / 'scan_archives').rglob('*') if path.is_file()
             and (path.suffix in ('.db3', '.db', '.pgm', '.pcd', '.tar')
                  or path.name.endswith(('.tar.gz', '.db3-wal', '.db3-shm')))]
    archive = root / 'rosmaster_export_current_full.tar.gz'
    if archive.exists():
        files.append(archive)
    result = []
    for path in sorted(files):
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        result.append({'path': path.relative_to(root).as_posix(),
                       'bytes': path.stat().st_size, 'sha256': digest.hexdigest()})
    return result


def write_summary(path: Path, sessions: list) -> None:
    fields = ['session', 'algorithm', 'metadata_status', 'raw_bag_available',
              'bag_duration_s', 'scan_messages', 'trajectory_samples',
              'trajectory_path_m', 'endpoint_separation_m', 'optimized_trajectory_samples',
              'metadata_nul_bytes', 'negative_hessian_index_warnings', 'probability_search_fatal']
    with path.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for session in sessions:
            bag = session['bag']
            writer.writerow({
                'session': session['session'],
                'algorithm': session['metadata'].get('algo'),
                'metadata_status': session['metadata'].get('status', ''),
                'raw_bag_available': bag['available'],
                'bag_duration_s': round(bag['duration_s'], 3) if bag['available'] else '',
                'scan_messages': bag.get('topic_counts', {}).get('/scan', '') if bag['available'] else '',
                'trajectory_samples': session['trajectory']['samples'],
                'trajectory_path_m': session['trajectory']['path_m'],
                'endpoint_separation_m': session['trajectory']['endpoint_separation_m'],
                'optimized_trajectory_samples': session['optimized_trajectory']['samples'],
                'metadata_nul_bytes': session['metadata_nul_bytes'],
                'negative_hessian_index_warnings': session['log']['negative_hessian_index_warnings'],
                'probability_search_fatal': session['log']['probability_search_fatal'],
            })


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--audit-date', type=date.fromisoformat, default=date.today())
    parser.add_argument('--out', type=Path, help='New output folder; default: audit_output/<audit-date>/')
    args = parser.parse_args()
    root = args.root.resolve()
    stamp = args.audit_date.strftime('%Y%m%d')
    out = args.out.resolve() if args.out else root / 'audit_output' / stamp
    # Refuse to overwrite an existing snapshot, including the versioned report.
    out.mkdir(parents=True, exist_ok=False)
    sessions = [read_session(path) for path in sorted(
        (root / 'scan_archives').glob('*/*/metadata.txt'))]
    report = {
        'audit_date': args.audit_date.isoformat(),
        'scope': 'local workspace; no live robot or remote repository accessed',
        'sessions': sessions,
        'historical_map_exports': historical_exports(root / 'rosmaster_export_current_full.tar.gz'),
        'totals': {
            'sessions': len(sessions),
            'bags_available': sum(s['bag']['available'] for s in sessions),
            'nonempty_bags': sum(s['bag'].get('message_count', 0) > 0 for s in sessions),
            'bag_duration_s': sum(s['bag'].get('duration_s', 0) for s in sessions),
            'raw_scan_messages': sum(s['bag'].get('topic_counts', {}).get('/scan', 0) for s in sessions),
        },
    }
    (out / f'experiment_audit_{stamp}.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    (out / f'raw_data_manifest_{stamp}.json').write_text(
        json.dumps(manifest(root), ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    write_summary(out / f'session_summary_{stamp}.csv', sessions)
    print(json.dumps(report['totals'], indent=2))
    print(f'Wrote audit snapshot: {out}')


if __name__ == '__main__':
    main()
