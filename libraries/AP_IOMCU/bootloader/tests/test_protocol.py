#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Run the actual C++ protocol with a native UART/flash model: python3 -m unittest discover -s <this directory>."""
import ctypes
import struct
import subprocess
import tempfile
import unittest
import zlib

from pathlib import Path

APP_BASE = 0x08001000
APP_SIZE = 60 * 1024
OK = b'\x12\x10'
INVALID = b'\x12\x13'
FAILED = b'\x12\x11'


class ProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        directory = Path(__file__).resolve().parent
        library = Path(cls.tmp.name) / 'protocol.so'
        subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all', '-g',
                        str(directory / 'harness.cpp'), str(directory.parent / 'protocol.cpp'),
                        '-o', str(library)], check=True)
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.flash_word.restype = ctypes.c_uint32
        cls.lib.active.restype = ctypes.c_bool
        cls.lib.valid_vectors.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
        cls.lib.valid_vectors.restype = ctypes.c_bool
        cls.lib.exchange.argtypes = [ctypes.c_char_p, ctypes.c_uint, ctypes.c_void_p]

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.lib.reset(True)

    def exchange(self, packet):
        reply = ctypes.create_string_buffer(4096)
        size = self.lib.exchange(packet, len(packet), reply)
        return reply.raw[:size]

    def sync(self):
        self.assertEqual(self.exchange(b'\0' * 254 + b'\x21\x20'), OK)

    def erase(self):
        self.sync()
        self.assertEqual(self.exchange(b'\x23\x20'), OK)

    def program(self, image):
        for pos in range(0, len(image), 248):
            block = image[pos:pos + 248]
            self.assertEqual(self.exchange(b'\x27' + bytes([len(block)]) + block + b'\x20'), OK)

    def crc(self):
        reply = self.exchange(b'\x29\x20')
        self.assertEqual(reply[4:], OK)
        return struct.unpack('<I', reply[:4])[0]

    def test_legacy_info_and_stray_bytes(self):
        for arg, expected in ((1, 5), (2, 10), (3, 0), (4, APP_SIZE)):
            self.assertEqual(self.exchange(bytes([0x22, arg, 0x20])), struct.pack('<I', expected) + OK)
        self.assertEqual(self.exchange(b'\x20\0'), b'')
        self.assertEqual(self.exchange(b'\x22\x05\x20'), b'\xff' * 16 + OK)
        self.assertEqual(self.exchange(b'\x22\x06\x20'), INVALID)

    def test_shared_f100_f103_application_vectors(self):
        for sp in (0x20000008, 0x20002000, 0x20005000):
            self.assertTrue(self.lib.valid_vectors(sp, APP_BASE + 1))
        for sp in (0x1ffffff8, 0x20000000, 0x20002004, 0x20005008, 0xffffffff):
            self.assertFalse(self.lib.valid_vectors(sp, APP_BASE + 1))
        for pc in (APP_BASE - 1, APP_BASE, APP_BASE + APP_SIZE + 1, 0xffffffff):
            self.assertFalse(self.lib.valid_vectors(0x20002000, pc))
        self.assertTrue(self.lib.valid_vectors(0x20005000, APP_BASE + APP_SIZE - 1))

    def test_fmu_upload_sequence_and_crc(self):
        self.sync()
        self.assertEqual(self.exchange(b'\x22\x01\x20'), struct.pack('<I', 5) + OK)
        self.assertEqual(self.exchange(b'\x23\x20'), OK)
        image = struct.pack('<II', 0x20002000, 0x08001101) + bytes(range(256)) * 100
        self.program(image)
        self.assertEqual(self.lib.flash_word(0), 0xffffffff)
        self.assertEqual(self.lib.flash_word(4), 0x08001101)
        # verify_rev3 sends an extra EOC after the flash-size query.
        self.assertEqual(self.exchange(b'\x22\x04\x20\x20'), struct.pack('<I', APP_SIZE) + OK)
        padded = image.ljust(APP_SIZE, b'\xff')
        self.assertEqual(self.crc(), zlib.crc32(padded, 0xffffffff) ^ 0xffffffff)
        self.assertEqual(self.exchange(b'\x30\x20'), OK)
        self.assertEqual(self.lib.flash_word(0), 0x20002000)
        self.assertEqual(self.lib.boot_count(), 1)
        self.lib.reset(False)
        self.assertEqual(self.crc(), zlib.crc32(padded, 0xffffffff) ^ 0xffffffff)

    def test_erase_requires_sync_and_program_requires_erase(self):
        self.assertEqual(self.exchange(b'\x23\x20'), INVALID)
        self.sync()
        self.assertEqual(self.exchange(b'\x27\x04\0\0\0\0\x20'), INVALID)
        self.assertFalse(self.lib.boot_count())

    def test_partial_upload_reset_keeps_first_word_erased(self):
        self.erase()
        self.program(struct.pack('<II', 0x20002000, 0x08001101))
        self.lib.reset(False)
        self.assertEqual(self.lib.flash_word(0), 0xffffffff)

    def test_crc_required_before_commit(self):
        self.erase()
        self.program(struct.pack('<II', 0x20002000, 0x08001101))
        self.assertEqual(self.exchange(b'\x30\x20'), INVALID)
        self.assertEqual(self.lib.flash_word(0), 0xffffffff)
        self.sync()
        self.crc()
        self.program(b'\x11\x22\x33\x44')
        self.assertEqual(self.exchange(b'\x30\x20'), INVALID)
        self.assertEqual(self.lib.flash_word(0), 0xffffffff)

    def test_second_erase_discards_pending_first_word(self):
        self.erase()
        self.program(struct.pack('<II', 0x20002000, 0x08001101))
        self.crc()
        self.assertEqual(self.exchange(b'\x23\x20'), OK)
        self.assertEqual(self.exchange(b'\x30\x20'), OK)
        self.assertEqual(self.lib.flash_word(0), 0xffffffff)
        self.program(struct.pack('<II', 0x20001000, 0x08001201))
        self.crc()
        self.assertEqual(self.exchange(b'\x30\x20'), OK)
        self.assertEqual(self.lib.flash_word(0), 0x20001000)

    def test_full_flash_and_bounds(self):
        self.erase()
        self.program(b'\x11' * APP_SIZE)
        self.assertEqual(self.exchange(b'\x27\x04\0\0\0\0\x20'), INVALID)
        self.assertEqual(self.lib.flash_word(APP_SIZE - 4), 0x11111111)

    def test_bad_lengths_and_missing_terminator_do_not_write(self):
        for packet in (b'\x27\0\x20', b'\x27\x03\0\0\0\x20', b'\x27\xff',
                       b'\x27\x04\x01\x02', b'\x27\x04\x01\x02\x03\x04\0'):
            self.erase()
            self.assertIn(INVALID, self.exchange(packet))
            self.assertEqual(self.lib.flash_word(0), 0xffffffff)
            self.assertEqual(self.lib.flash_word(4), 0xffffffff)

    def test_flash_failure_requires_fresh_erase(self):
        self.erase()
        self.lib.inject_write_failure(4)
        self.assertEqual(self.exchange(b'\x27\x08' + struct.pack('<II', 0x20002000, 0x08001101) + b'\x20'), FAILED)
        self.lib.inject_write_failure(-1)
        self.crc()
        self.assertEqual(self.exchange(b'\x30\x20'), OK)
        self.assertEqual(self.lib.flash_word(0), 0xffffffff)
        self.assertEqual(self.exchange(b'\x27\x04\0\0\0\0\x20'), INVALID)

    def test_erase_failure_discards_pending_commit(self):
        self.erase()
        self.program(struct.pack('<II', 0x20002000, 0x08001101))
        self.crc()
        self.lib.inject_erase_failure(True)
        self.assertEqual(self.exchange(b'\x23\x20'), FAILED)
        self.assertEqual(self.exchange(b'\x30\x20'), OK)
        self.assertEqual(self.lib.flash_word(0), 0xffffffff)

    def test_only_valid_commands_disable_startup_timeout(self):
        self.assertFalse(self.lib.active())
        self.assertEqual(self.exchange(b'\0\x20\x21\0'), INVALID)
        self.assertFalse(self.lib.active())
        self.sync()
        self.assertTrue(self.lib.active())


if __name__ == '__main__':
    unittest.main()
