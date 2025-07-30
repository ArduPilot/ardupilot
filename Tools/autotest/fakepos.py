#!/usr/bin/env python3

import errno
import socket
import struct
import sys
import time
from math import cos, fabs, radians, sin, sqrt


class udp_out(object):
    """A UDP output socket."""
    def __init__(self, device):
        a = device.split(':')
        if len(a) != 2:
            print("UDP ports must be specified as host:port")
            sys.exit(1)
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination_addr = (a[0], int(a[1]))
        self.port.setblocking(0)
        self.last_address = None

    def recv(self, n=None):
        try:
            data, self.last_address = self.port.recvfrom(300)
        except socket.error as e:
            if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK]:
                return ""
            raise
        return data

    def write(self, buf):
        try:
            self.port.sendto(buf, self.destination_addr)
        except socket.error:
            pass


udp = udp_out("127.0.0.1:5501")

latitude = -35
longitude = 149
altitude = 600.0
heading = 0.0
speedN = 0
speedE = 0.0
speedD = 0.0
rollRate = 0.0
yawRate = 0.0
yawDeg = 0.0
airspeed = 0
magic = 0x4c56414f

deltaT = 0.005
rollDeg = 45
pitchDeg = 0

pitchMax = None
rollMax = None

if True:
    pitchRate = 1
    pitchMax = 45
    rollMax = 45

while True:

    xAccel = sin(radians(pitchDeg))
    yAccel = -sin(radians(rollDeg)) * cos(radians(pitchDeg))
    zAccel = -cos(radians(rollDeg)) * cos(radians(pitchDeg))
    scale = 9.81 / sqrt((xAccel*xAccel) + (yAccel*yAccel) + (zAccel*zAccel))
    xAccel *= scale
    yAccel *= scale
    zAccel *= scale

    struc_buf = struct.pack('<17dI',
                            latitude, longitude, altitude, heading,
                            speedN, speedE, speedD,
                            xAccel, yAccel, zAccel,
                            rollRate, pitchRate, yawRate,
                            rollDeg, pitchDeg, yawDeg,
                            airspeed, magic)
    udp.write(struc_buf)
    time.sleep(deltaT)

    yawDeg += yawRate * deltaT
    if yawDeg > 180:
        yawDeg -= 360
    if yawDeg < -180:
        yawDeg += 360
    heading = yawDeg

    if pitchMax is not None and fabs(pitchDeg) > pitchMax:
        pitchRate = -pitchRate

    if rollMax is not None and fabs(rollDeg) > rollMax:
        rollRate = -rollRate

    pitchDeg += pitchRate * deltaT
    if pitchDeg > 180:
        pitchDeg -= 360
    if pitchDeg < -180:
        pitchDeg += 360

    rollDeg += rollRate * deltaT
    if rollDeg > 180:
        rollDeg -= 360
    if rollDeg < -180:
        rollDeg += 360
