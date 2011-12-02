#!/usr/bin/env python
'''
useful extra functions for use by mavlink clients

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

from math import *


def norm_heading(RAW_IMU, ATTITUDE, declination):
    '''calculate heading from RAW_IMU and ATTITUDE'''
    xmag = RAW_IMU.xmag
    ymag = RAW_IMU.ymag
    zmag = RAW_IMU.zmag
    pitch = ATTITUDE.pitch
    roll  = ATTITUDE.roll

    headX = xmag*cos(pitch) + ymag*sin(roll)*sin(pitch) + zmag*cos(roll)*sin(pitch)
    headY = ymag*cos(roll) - zmag*sin(roll)
    heading = atan2(-headY, headX)
    heading = fmod(degrees(heading) + declination + 360, 360)
    return heading

def TrueHeading(SERVO_OUTPUT_RAW):
    rc3_min = 1060
    rc3_max = 1850
    p = float(SERVO_OUTPUT_RAW.servo3_raw - rc3_min) / (rc3_max - rc3_min)
    return 172 + (1.0-p)*(326 - 172)

def kmh(mps):
    '''convert m/s to Km/h'''
    return mps*3.6
