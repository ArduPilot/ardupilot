#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#         Ben Dyer <ben_dyer@mac.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import os
import struct
from dronecan import UAVCANException


class DsdlException(UAVCANException):
    '''
    This exception is raised in case of a parser failure.
    Fields:
        file    Source file path where the error has occurred. Optional, will be None if unknown.
        line    Source file line number where the error has occurred. Optional, will be None if unknown.
    '''

    def __init__(self, text, file=None, line=None):
        Exception.__init__(self, text)
        self.file = file
        self.line = line

    def __str__(self):
        '''Returns nicely formatted error string in GCC-like format (can be parsed by e.g. Eclipse error parser)'''
        if self.file and self.line:
            return '%s:%d: %s' % (pretty_filename(self.file), self.line, Exception.__str__(self))
        if self.file:
            return '%s: %s' % (pretty_filename(self.file), Exception.__str__(self))
        return Exception.__str__(self)


def pretty_filename(filename):
    '''Returns a nice human readable path to 'filename'.'''
    try:
        a = os.path.abspath(filename)
        r = os.path.relpath(filename)
    except ValueError:
        # Catch relpath exception. Happens, because it can not produce relative path
        # if wroking directory is on different drive.
        a = r = filename
    return a if '..' in r else r


def crc16_from_bytes(bytes, initial=0xFFFF):
    # CRC-16-CCITT
    # Initial value: 0xFFFF
    # Poly: 0x1021
    # Reverse: no
    # Output xor: 0
    # Check string: '123456789'
    # Check value: 0x29B1

    try:
        if isinstance(bytes, basestring):  # Python 2.7 compatibility
            bytes = map(ord, bytes)
    except NameError:
        if isinstance(bytes, str):  # This branch will be taken on Python 3
            bytes = map(ord, bytes)

    crc = initial
    for byte in bytes:
        crc ^= byte << 8
        for bit in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def bytes_from_crc64(crc64):
    # Cast to str explicitly for Python 2.7 compatibility when
    # unicode_literals is enabled
    return bytes(struct.pack(str("<Q"), crc64))
