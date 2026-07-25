#!/usr/bin/env python3
'''
extract non-default parameters for publishing
'''

from pymavlink import mavparm
import fnmatch

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("defaults", metavar="defaults")
parser.add_argument("params", metavar="params")

args = parser.parse_args()

p1 = mavparm.MAVParmDict()
p2 = mavparm.MAVParmDict()
p1.load(args.defaults)
p2.load(args.params)

include_list = [ 'Q_ENABLE', 'Q_FRAME*', 'INS_HN*' ]
exclude_list = [ 'AHRS_TRIM*', 'RC[0-9]*', 'BARO*', 'INS*',
                 'COMPASS*', 'ARMING_*', '*DEVID', 'ARSPD_TYPE', 'ARSPD_RATIO',
                     'ADSB*', 'FLTMODE*', 'EK3*', 'ARSPD_PIN', 'ARSPD_USE', 'ARSPD_OPTIONS', '*WIND*',
                     'AVD*', 'BATT*', 'BRD*', 'CAN*', 'CRASH*', 'FS*', 'THR_FS*', '*NODEID*', 'GPS*', 'LGR*',
                     'NTF*', 'ONESH*', 'FENCE*', 'FOLL*', 'Q_AUTOTUNE*', 'Q_OPTIONS', 'RC_OPTIONS',
                     'RELAY*', 'RNGFND*', 'RSSI*', 'SCR*', 'SERIAL*', 'SR[0-9]*', 'STAT*']

def in_list(p, lst):
    for e in lst:
        if fnmatch.fnmatch(p, e):
            return True
    return False

def vstring(v):
    s = str(v)
    if s.find('.'):
        while s[-1] == '0':
            s = s[:-1]
    if s[-1] == '.':
        s = s[:-1]
    return s

for p in p2:
    if in_list(p, exclude_list) and not in_list(p, include_list):
        continue
    if not p in p1 or (p1[p] == p2[p] and not in_list(p, include_list)):
        continue
    print("%-16s %s" % (p, vstring(p2[p])))
