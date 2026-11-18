"""Verify restoration preserves files and cannot escape the requested root."""
import hashlib
import importlib.util
import io
from pathlib import Path
import tarfile
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('download_raw_data', Path(__file__).parents[1] / 'download_raw_data.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RestoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.archive = self.root / 'data.tar'

    def make_archive(self, name='session/file.txt', symlink=False):
        value = b'original experiment data\n'
        with tarfile.open(self.archive, 'w') as stream:
            member = tarfile.TarInfo(name)
            if symlink:
                member.type = tarfile.SYMTYPE
                member.linkname = '../outside'
                stream.addfile(member)
            else:
                member.size = len(value)
                stream.addfile(member, io.BytesIO(value))
        return [{'path': name, 'bytes': len(value), 'sha256': hashlib.sha256(value).hexdigest()}]

    def test_identical_restoration_is_idempotent(self):
        members = self.make_archive()
        destination = self.root / 'restored'
        first = module.extract_archive(self.archive, destination, members)
        second = module.extract_archive(self.archive, destination, members)
        self.assertEqual(first['restored_files'], 1)
        self.assertEqual(second['restored_files'], 0)
        self.assertEqual(second['already_matching_files'], 1)

    def test_path_traversal_cannot_write_outside_destination(self):
        members = self.make_archive('../outside.txt')
        with self.assertRaises(ValueError):
            module.extract_archive(self.archive, self.root / 'restored', members)
        self.assertFalse((self.root / 'outside.txt').exists())

    def test_different_existing_file_is_preserved(self):
        members = self.make_archive()
        destination = self.root / 'restored'
        existing = destination / 'session' / 'file.txt'
        existing.parent.mkdir(parents=True)
        existing.write_bytes(b'newer work')
        with self.assertRaises(FileExistsError):
            module.extract_archive(self.archive, destination, members)
        self.assertEqual(existing.read_bytes(), b'newer work')

    def test_archive_symlink_is_rejected(self):
        members = self.make_archive(symlink=True)
        with self.assertRaises(ValueError):
            module.extract_archive(self.archive, self.root / 'restored', members)
        self.assertFalse((self.root / 'restored').exists())


if __name__ == '__main__':
    unittest.main()
