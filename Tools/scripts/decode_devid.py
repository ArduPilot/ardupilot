#!/usr/bin/env python
'''
decode a device ID, such as used for COMPASS_DEV_ID, INS_ACC_ID etc

To understand the devtype you should look at the backend headers for
the sensor library, such as libraries/AP_Compass/AP_Compass_Backend.h
'''

import sys

devid=int(sys.argv[1])

bus_type=devid & 0x07
bus=(devid>>3) & 0x1F
address=(devid>>8)&0xFF
devtype=(devid>>16)

print("bus_type:%u  bus:%u address:%u devtype:%u(0x%x)" % (
             bus_type, bus, address, devtype, devtype))



