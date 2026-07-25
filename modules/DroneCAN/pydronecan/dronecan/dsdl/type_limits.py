#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#         Ben Dyer <ben_dyer@mac.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
from .common import DsdlException


def get_unsigned_integer_range(bitlen):
    if not 1 <= bitlen <= 64:
        raise DsdlException('Invalid bit length for integer type: %d' % bitlen)
    return 0, (1 << bitlen) - 1


def get_signed_integer_range(bitlen):
    _, uint_max = get_unsigned_integer_range(bitlen)
    return -int(uint_max / 2) - 1, int(uint_max / 2)


def get_float_range(bitlen):
    try:
        maxvalue = {
            16: 65504.0,
            32: 3.40282346638528859812e+38,
            64: 1.79769313486231570815e+308
        }[bitlen]
    except KeyError:
        raise DsdlException('Invalid bit length for float type: %d' % bitlen)
    return -maxvalue, maxvalue
