#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import unittest
from dronecan.dsdl import common


class TestCRC16FromBytes(unittest.TestCase):
    def test_str(self):
        self.assertEqual(common.crc16_from_bytes('123456789'), 0x29B1)

    def test_bytes(self):
        self.assertEqual(common.crc16_from_bytes(b'123456789'), 0x29B1)

    def test_bytearray(self):
        self.assertEqual(
            common.crc16_from_bytes(bytearray('123456789', 'utf-8')),
            0x29B1)


class TestBytesFromCRC64(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(common.bytes_from_crc64(0),
                         b"\x00\x00\x00\x00\x00\x00\x00\x00")

    def test_check_val(self):
        self.assertEqual(common.bytes_from_crc64(0x62EC59E3F1A4F00A),
                         b"\x0A\xF0\xA4\xF1\xE3\x59\xEC\x62")


if __name__ == '__main__':
    unittest.main()
