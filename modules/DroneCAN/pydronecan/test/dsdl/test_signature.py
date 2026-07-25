#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import unittest
from dronecan.dsdl import signature


class TestSignature(unittest.TestCase):
    def test_add(self):
        s = signature.Signature()
        s.add('123456789')
        self.assertEqual(s.get_value(), 0x62EC59E3F1A4F00A)

    def test_add_multiple(self):
        s = signature.Signature()
        for c in '123456789':
            s.add(c)
        self.assertEqual(s.get_value(), 0x62EC59E3F1A4F00A)

    def test_add_bytes(self):
        s = signature.Signature()
        s.add(b'123456789')
        self.assertEqual(s.get_value(), 0x62EC59E3F1A4F00A)

    def test_add_bytearray(self):
        s = signature.Signature()
        s.add(bytearray('123456789', 'utf-8'))
        self.assertEqual(s.get_value(), 0x62EC59E3F1A4F00A)


class TestComputeSignature(unittest.TestCase):
    def test_str(self):
        self.assertEqual(signature.compute_signature('123456789'),
                         0x62EC59E3F1A4F00A)

    def test_bytes(self):
        self.assertEqual(signature.compute_signature(b'123456789'),
                         0x62EC59E3F1A4F00A)

    def test_bytearray(self):
        self.assertEqual(
            signature.compute_signature(bytearray('123456789', 'utf-8')),
            0x62EC59E3F1A4F00A)


if __name__ == '__main__':
    unittest.main()
