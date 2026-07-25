#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#         Ben Dyer <ben_dyer@mac.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals

#
# CRC-64-WE
# Description: http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
# Initial value: 0xFFFFFFFFFFFFFFFF
# Poly: 0x42F0E1EBA9EA3693
# Reverse: no
# Output xor: 0xFFFFFFFFFFFFFFFF
# Check: 0x62EC59E3F1A4F00A
#


class Signature:
    '''
    This class implements the UAVCAN DSDL signature hash function. Please refer to the specification for details.
    '''
    MASK64 = 0xFFFFFFFFFFFFFFFF
    POLY = 0x42F0E1EBA9EA3693

    def __init__(self, extend_from=None):
        '''
        extend_from    Initial value (optional)
        '''
        if extend_from is not None:
            self._crc = (int(extend_from) & Signature.MASK64) ^ Signature.MASK64
        else:
            self._crc = Signature.MASK64

    def add(self, data_bytes):
        '''Feed ASCII string or bytes to the signature function'''
        try:
            if isinstance(data_bytes, basestring):  # Python 2.7 compatibility
                data_bytes = map(ord, data_bytes)
        except NameError:
            if isinstance(data_bytes, str):  # This branch will be taken on Python 3
                data_bytes = map(ord, data_bytes)

        for b in data_bytes:
            self._crc ^= (b << 56) & Signature.MASK64
            for _ in range(8):
                if self._crc & (1 << 63):
                    self._crc = ((self._crc << 1) & Signature.MASK64) ^ Signature.POLY
                else:
                    self._crc <<= 1

    def get_value(self):
        '''Returns integer signature value'''
        return (self._crc & Signature.MASK64) ^ Signature.MASK64


def compute_signature(data):
    '''
    One-shot signature computation for ASCII string or bytes.
    Returns integer signture value.
    '''
    s = Signature()
    s.add(data)
    return s.get_value()


# if __name__ == '__main__':
if 1:
    s = Signature()
    s.add(b'123')
    s.add('456789')
    assert s.get_value() == 0x62EC59E3F1A4F00A
