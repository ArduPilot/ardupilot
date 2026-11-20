#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).resolve().parents[2] / 'scripts' / 'param_unpack.py'


class ParamUnpackTest(unittest.TestCase):
    def unpack(self, data):
        with tempfile.TemporaryDirectory() as directory:
            packed = pathlib.Path(directory) / 'param.pck'
            packed.write_bytes(data)
            return subprocess.run([sys.executable, str(SCRIPT), str(packed)],
                                  capture_output=True, text=True)

    def test_formats(self):
        for magic in (0x671b, 0x671c):
            for ptype, fmt, value, default in ((1, 'b', -12, 7), (2, 'h', -1234, 200),
                                               (3, 'i', 123456789, -456), (4, 'f', 1.25, -2.5)):
                with self.subTest(magic=magic, ptype=ptype):
                    # The second record shares its prefix and has no explicit default.
                    flags = 0x10 if magic == 0x671c else 0
                    record = bytes((ptype | flags, 0x50)) + b'TEST_A' + struct.pack('<' + fmt, value)
                    if flags:
                        record += struct.pack('<' + fmt, default)
                    record += b'\x00\x00' + bytes((ptype, 5)) + b'B' + struct.pack('<' + fmt, value)
                    result = self.unpack(struct.pack('<HHH', magic, 2, 10) + record)
                    self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
                    expected = ['%-16s %f' % (name, value) for name in ('TEST_A', 'TEST_B')]
                    self.assertEqual(result.stdout.splitlines(), expected)

    def test_defaults_header_without_explicit_defaults(self):
        data = struct.pack('<HHH', 0x671c, 1, 1) + b'\x01\x00A\x05'
        result = self.unpack(data)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), 'A                5.000000')

    def test_unknown_magic(self):
        result = self.unpack(struct.pack('<HHH', 0x1234, 0, 0))
        self.assertEqual(result.returncode, 1)
        self.assertIn('Bad magic', result.stdout)


if __name__ == '__main__':
    unittest.main()
