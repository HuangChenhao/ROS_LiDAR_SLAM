#!/usr/bin/env python3
"""Download checksummed Release datasets and optionally restore original files."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import urllib.request


MANIFEST_NAME = 'raw_release_manifest_20261118.json'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def verify_archive(path: Path, record: dict) -> None:
    if path.stat().st_size != record['bytes'] or sha256(path) != record['sha256']:
        raise ValueError('Archive size or SHA-256 mismatch: ' + path.name)


def download_archive(root: Path, record: dict, verify_only: bool = False) -> Path:
    target = (root / record['source_path']).resolve()
    if not target.is_relative_to(root):
        raise ValueError('Download path leaves dataset root')
    if target.exists():
        verify_archive(target, record)
        print('Verified existing archive: ' + record['asset_name'])
        return target
    if verify_only:
        raise FileNotFoundError('Archive not found: ' + str(target))
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + '.download')
    # Never truncate a previous interrupted download or an unrelated file.
    if partial.exists():
        raise FileExistsError('Download temporary file already exists: ' + str(partial))
    request = urllib.request.Request(record['download_url'], headers={
        'User-Agent': 'ROSMASTER-R2-dataset-downloader',
        'Accept': 'application/octet-stream',
    })
    print('Downloading: ' + record['download_url'], flush=True)
    with urllib.request.urlopen(request, timeout=60) as response, partial.open('xb') as stream:
        shutil.copyfileobj(response, stream, length=1024 * 1024)
    verify_archive(partial, record)
    # Preserve a concurrently created archive instead of overwriting it.
    if target.exists():
        raise FileExistsError('Target appeared during download: ' + str(target))
    partial.rename(target)
    return target


def extract_archive(path: Path, destination: Path, expected_members: list[dict]) -> dict:
    """Preflight every member; refuse traversal, links and nonidentical files."""
    destination = destination.resolve()
    expected = {row['path']: row for row in expected_members}
    with tarfile.open(path, 'r:*') as archive:
        members = archive.getmembers()
        seen = set()
        pending = []
        skipped = 0
        for member in members:
            name = PurePosixPath(member.name)
            if name.is_absolute() or '..' in name.parts or '\\' in member.name:
                raise ValueError('Unsafe archive member: ' + member.name)
            target = (destination / member.name).resolve()
            if not target.is_relative_to(destination):
                raise ValueError('Archive member leaves destination: ' + member.name)
            if member.isdir():
                continue
            if not member.isfile() or member.name not in expected or member.name in seen:
                raise ValueError('Unexpected or linked archive member: ' + member.name)
            seen.add(member.name)
            record = expected[member.name]
            if member.size != record['bytes']:
                raise ValueError('Archive member size mismatch: ' + member.name)
            if target.exists():
                if not target.is_file() or sha256(target) != record['sha256']:
                    raise FileExistsError('Refusing to overwrite a different file: ' + str(target))
                skipped += 1
            else:
                pending.append((member, target, record))
        if seen != set(expected):
            raise ValueError('Archive member inventory mismatch')
        for member, target, record in pending:
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.extractfile(member) as source, target.open('xb') as stream:
                shutil.copyfileobj(source, stream, length=1024 * 1024)
            if sha256(target) != record['sha256']:
                raise ValueError('Restored member checksum mismatch: ' + member.name)
    return {'restored_files': len(pending), 'already_matching_files': skipped}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--dataset', default='all',
                        choices=['all', 'ros1_export', 'ros2_20260722', 'validation_20260723'])
    parser.add_argument('--extract', action='store_true', help='Restore missing original files after verification')
    parser.add_argument('--verify-only', action='store_true', help='Verify local archives without downloading')
    args = parser.parse_args()
    root = args.root.resolve()
    manifest_path = args.manifest or Path(__file__).resolve().parents[1] / 'reports' / MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text('utf-8'))
    for record in manifest['archives']:
        if args.dataset != 'all' and args.dataset != record['dataset']:
            continue
        archive = download_archive(root, record, args.verify_only)
        if args.extract:
            destination = (root / record['extract_to']).resolve()
            if not destination.is_relative_to(root):
                raise ValueError('Extraction destination leaves dataset root')
            result = extract_archive(archive, destination, record['members'])
            print(record['dataset'] + ': ' + json.dumps(result))
    print('Selected dataset archives verified.')


if __name__ == '__main__':
    main()
