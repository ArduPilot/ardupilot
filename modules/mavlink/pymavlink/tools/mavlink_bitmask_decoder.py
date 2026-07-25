#!/usr/bin/env python3

from pymavlink import mavutil

import argparse

def print_decode(messagetype, field, value):
    decoded = mavutil.decode_bitmask(messagetype, field, value)
    for bit_value in decoded:
        value = bit_value.value
        offset = bit_value.offset
        name = bit_value.name
        svalue = " "
        if not value:
            svalue = "!"
        if name is None:
            name = "[UNKNOWN]"
        print("%s %s" % (svalue, name))


parser = argparse.ArgumentParser(description=__doc__)

parser.add_argument("message")
parser.add_argument("field")
parser.add_argument("value")

args = parser.parse_args()

#        print("Usage: decode-mavlink-bitmask.py MESSAGETYPE FIELDNAME VALUE")
#        print("e.g: decode-mavlink-bitmask.py SYS_STATUS onboard_control_sensors_health 12531")
#        sys.exit(1)

print_decode(args.message, args.field, int(args.value))
