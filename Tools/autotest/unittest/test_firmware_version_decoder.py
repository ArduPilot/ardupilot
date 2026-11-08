"""Tests for packed and naturally aligned firmware metadata.

AP_FLAKE8_CLEAN
"""

import importlib.util
import io
import struct
import unittest

from pathlib import Path
from types import SimpleNamespace

decoder_path = Path(__file__).resolve().parents[2] / "scripts" / "firmware_version_decoder.py"
spec = importlib.util.spec_from_file_location("firmware_version_decoder", decoder_path)
decoder_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(decoder_module)


class TestFirmwareVersionDecoder(unittest.TestCase):
    def decode(self, version, pointer_size, byteorder, null_strings=False):
        # v2 is packed and may start at an unaligned offset in the binary.
        prefix = b"\0" * (3 if version == 2 else 16)
        metadata = bytearray(struct.pack(
            byteorder + "QHBBBBHBBBBI",
            decoder_module.FWVersion.header, version << 8, pointer_size,
            0, 2, 3, 65535, 4, 8, 0, 0, 20261108,
        ))
        padding = (-(len(metadata) + 2 * pointer_size + 4)) % pointer_size if version == 3 else 0
        metadata_size = len(metadata) + 8 * pointer_size + 4 + padding
        strings = bytearray()

        def pointer(value):
            address = 0
            if value is not None:
                address = 0x8000 + len(prefix) + metadata_size + len(strings)
                strings.extend(value.encode() + b"\0")
            return struct.pack(byteorder + ("Q" if pointer_size == 8 else "I"), address)

        metadata.extend(pointer("ArduCopter test"))
        metadata.extend(pointer("1234abcd"))
        metadata.extend(struct.pack(byteorder + "I", 0x1234abcd))
        metadata.extend(b"\0" * padding)
        for value in ("Original firmware", "Short firmware", "Middleware", "middleware-hash", "ChibiOS", "os-hash"):
            metadata.extend(pointer(None if null_strings else value))

        decoder = decoder_module.Decoder()
        decoder.byteorder = byteorder
        decoder.bytesio = io.BytesIO(prefix + metadata + strings)
        decoder.bytesio.seek(len(prefix))
        decoder.elffile = SimpleNamespace(address_offsets=lambda address: iter([address - 0x8000]))
        decoder.unpack_fwversion()
        return decoder.fwversion

    def test_v2_and_v3_metadata(self):
        for version in (2, 3):
            for pointer_size in (4, 8):
                for byteorder in ("<", ">"):
                    with self.subTest(version=version, pointer_size=pointer_size, byteorder=byteorder):
                        firmware = self.decode(version, pointer_size, byteorder)
                        self.assertEqual(firmware.header_version, version << 8)
                        self.assertEqual(firmware.pointer_size, pointer_size)
                        self.assertEqual(firmware.firmware_string, "ArduCopter test")
                        self.assertEqual(firmware.firmware_hash_string, "1234abcd")
                        self.assertEqual(firmware.firmware_hash, 0x1234abcd)
                        self.assertEqual(firmware.middleware_name, "Middleware")
                        self.assertEqual(firmware.middleware_hash_string, "middleware-hash")
                        self.assertEqual(firmware.os_name, "ChibiOS")
                        self.assertEqual(firmware.os_hash_string, "os-hash")

    def test_null_metadata_strings(self):
        for version in (2, 3):
            for pointer_size in (4, 8):
                for byteorder in ("<", ">"):
                    with self.subTest(version=version, pointer_size=pointer_size, byteorder=byteorder):
                        firmware = self.decode(version, pointer_size, byteorder, null_strings=True)
                        self.assertEqual(firmware.firmware_hash, 0x1234abcd)
                        self.assertEqual(firmware.middleware_name, "")
                        self.assertEqual(firmware.middleware_hash_string, "")
                        self.assertEqual(firmware.os_name, "")
                        self.assertEqual(firmware.os_hash_string, "")


if __name__ == "__main__":
    unittest.main()
