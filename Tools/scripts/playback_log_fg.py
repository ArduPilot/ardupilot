#!/usr/bin/env python

'''
play back an onboard log as a FlightGear FG NET stream

Useful for visualising flights
'''

from builtins import object

import sys
import time

from pymavlink import fgFDM
from pymavlink import mavutil

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--fgout", action='append', default=['127.0.0.1:5503'], help="flightgear FDM NET output (IP:port)")
parser.add_argument("--attmsg", default='ATT', help="msg to use for attitude")
parser.add_argument("--speedup", type=float, default=1.0, help="playback speedup")
parser.add_argument("--chute-slowdown", type=float, default=1.0, help="slowdown after parachute release")
parser.add_argument("log", metavar="LOG")
args = parser.parse_args()

filename = args.log


class Playback(object):
    def __init__(self, filename):
        self.mlog = mavutil.mavlink_connection(filename)
        self.fgout = []
        for f in args.fgout:
            self.fgout.append(mavutil.mavudp(f, input=False))

        self.fdm = fgFDM.fgFDM()

        self.chute_released = False

        self.msg = self.next_msg()
        if self.msg is None:
            sys.exit(1)
        self.last_timestamp = self.msg.TimeUS*1.0e-6

        while True:
            self.next_message()
            if self.msg is None:
                break

    def next_msg(self):
        while True:
            msg = self.mlog.recv_match(condition=args.condition)
            if msg is None:
                return None
            if msg.get_type() not in ['PARM', 'FMT', 'CMD']:
                return msg

    def next_message(self):
        '''called as each msg is ready'''

        msg = self.msg
        if msg is None:
            return

        timestamp = msg.TimeUS*1.0e-6

        dt = timestamp - self.last_timestamp
        dt = max(dt, 0)
        dt = min(dt, 1)
        if dt > 0.01:
            self.last_timestamp = timestamp
            if self.chute_released:
                dt *= args.chute_slowdown
            else:
                dt /= args.speedup
            while dt > 0:
                time.sleep(0.01)
                dt -= 0.01
                if self.fdm.get('latitude') != 0:
                    for f in self.fgout:
                        f.write(self.fdm.pack())

        while True:
            self.msg = self.next_msg()
            if self.msg is None:
                return
            if self.msg is not None and self.msg.get_type() != "BAD_DATA":
                break

        if msg.get_type() == "HEARTBEAT":
            self.is_mavlink = True

        if msg.get_type() == "GPS":
            self.fdm.set('latitude', msg.Lat, units='degrees')
            self.fdm.set('longitude', msg.Lng, units='degrees')
            self.fdm.set('altitude', msg.Alt, units='meters')

        if msg.get_type() == args.attmsg:
            self.fdm.set('phi', msg.Roll, units='degrees')
            self.fdm.set('theta', msg.Pitch, units='degrees')
            self.fdm.set('psi', msg.Yaw, units='degrees')

        if msg.get_type() == "IMU":
            self.fdm.set('phidot', msg.GyrX, units='rps')
            self.fdm.set('thetadot', msg.GyrY, units='rps')
            self.fdm.set('psidot', msg.GyrZ, units='rps')

        if msg.get_type() == "RCOU":
            self.fdm.set('num_engines', 4)
            self.fdm.set('rpm', (msg.C1-1060), 0)
            self.fdm.set('rpm', (msg.C2-1060), 1)
            self.fdm.set('rpm', (msg.C3-1060), 2)
            self.fdm.set('rpm', (msg.C4-1060), 3)

        if msg.get_type() == 'MSG':
            print("APM: %s" % msg.Message)

        if msg.get_type() == 'EV':
            if msg.Id == 51:
                self.chute_released = True
                print("PARACHTE RELEASED")

        if self.fdm.get('latitude') != 0:
            for f in self.fgout:
                f.write(self.fdm.pack())


playback = Playback(filename)
