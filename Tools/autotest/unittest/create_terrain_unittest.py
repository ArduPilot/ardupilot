'''
Unit tests for the terrain data creation tool.

AP_FLAKE8_CLEAN
'''

import sys
import tempfile
import types
import unittest

from pathlib import Path
from unittest.mock import patch


def crc16xmodem(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xffff
            else:
                crc = (crc << 1) & 0xffff
    return crc


def load_create_terrain_namespace():
    mavproxy = types.ModuleType('MAVProxy')
    modules = types.ModuleType('MAVProxy.modules')
    mavproxy_map = types.ModuleType('MAVProxy.modules.mavproxy_map')
    srtm = types.ModuleType('MAVProxy.modules.mavproxy_map.srtm')
    mavproxy_map.srtm = srtm
    modules.mavproxy_map = mavproxy_map
    mavproxy.modules = modules

    crc16 = types.ModuleType('crc16')
    crc16.crc16xmodem = crc16xmodem

    fake_modules = {
        'MAVProxy': mavproxy,
        'MAVProxy.modules': modules,
        'MAVProxy.modules.mavproxy_map': mavproxy_map,
        'MAVProxy.modules.mavproxy_map.srtm': srtm,
        'crc16': crc16,
    }

    root = Path(__file__).resolve().parents[3]
    source = root.joinpath('libraries', 'AP_Terrain', 'tools', 'create_terrain.py').read_text(encoding='utf-8')
    namespace = {}
    with patch.dict(sys.modules, fake_modules):
        exec(source.split('from argparse import ArgumentParser')[0], namespace)
    return namespace


class TestCreateTerrain(unittest.TestCase):

    def test_pack_writes_current_minor_version(self):
        namespace = load_create_terrain_namespace()

        class Args:
            pass

        with tempfile.TemporaryDirectory() as tmpdir:
            args = Args()
            args.directory = tmpdir
            namespace['args'] = args

            block = namespace['GridBlock'](47, 18, 47.0, 18.0)
            data_file = namespace['DataFile'](47, 18)
            try:
                data_file.write(block)
                data_file.fh.flush()

                data_file.fh.seek(block.blocknum() * namespace['IO_BLOCK_SIZE'])
                buf = data_file.fh.read(namespace['IO_BLOCK_SIZE'])
                self.assertEqual(len(buf), namespace['IO_BLOCK_SIZE'])
                self.assertEqual(namespace['get_version_minor'](buf), namespace['TERRAIN_VERSION_MINOR_MIN'])

                crc_stored = int.from_bytes(buf[16:18], 'little')
                crc_buf = buf[:16] + b'\x00\x00' + buf[18:]
                self.assertEqual(crc16xmodem(crc_buf[:namespace['IO_BLOCK_DATA_SIZE']]), crc_stored)
                self.assertTrue(data_file.check_filled(block))

                old_buf = bytearray(buf)
                old_buf[namespace['IO_BLOCK_VERSION_MINOR_OFS']] = 0
                data_file.fh.seek(block.blocknum() * namespace['IO_BLOCK_SIZE'])
                data_file.fh.write(old_buf)
                data_file.fh.flush()
                self.assertFalse(data_file.check_filled(block))
            finally:
                data_file.fh.close()


if __name__ == '__main__':
    unittest.main()
