#!/usr/bin/env python3


"""
test for trimming under Python 3
"""

import unittest
import os
import sys
from pymavlink import mavutil

class PayLoadTrimZeros(unittest.TestCase):
    '''Trivial test for trimming zeros from end of messages'''

    def test_dump_length(self):
        mavutil.mavlink.WIRE_PROTOCOL_VERSION = 2
        mav = mavutil.mavudp(":12345")

        ts = [ ((1, 1), 14),
              ((1, 0), 13),
              ((0, 0), 13)
              ]
        for t in ts:
            ((sysid, compid), result) = t
            m = mavutil.mavlink.MAVLink_param_request_list_message(sysid, compid)
            packed = m.pack(mav.mav)
            print("(%u/%u) should be %u" % (sysid,compid, result))
            self.assertEqual(len(packed), result)

if __name__ == '__main__':
    unittest.main()
