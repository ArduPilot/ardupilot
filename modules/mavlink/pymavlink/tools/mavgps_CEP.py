#!/usr/bin/env python3

'''
calculate GPS CEP from DF or mavlink log for all present GPS modules

This assumes the GPS modules were not moving during the test
'''

import os

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.mavextra import *

# full set of data, indexed by sysId and
DATA = {}

class Sample:
    def __init__(self, lat, lon, alt, fix_type):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.fix_type = fix_type

def add_data(sysid, gps_id, lat, lon, alt, fix_type):
    if not sysid in DATA:
        DATA[sysid] = {}
    if not gps_id in DATA[sysid]:
        DATA[sysid][gps_id] = []
    DATA[sysid][gps_id].append(Sample(lat, lon, alt, fix_type))

def process_log(logfile):
    '''process GPS logs'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    while True:
        m = mlog.recv_match(type=['GPS', 'GPS2', 'GPS_RAW_INT', 'GPS2_RAW'])
        if m is None:
            break
        mtype = m.get_type()
        if mtype in ['GPS_RAW_INT', 'GPS2_RAW']:
            (lat, lon, alt, fix_type) = (m.lat*1.0e-7, m.lon*1.0e-7, m.alt*1.0e-3, m.fix_type)
        else:
            (lat, lon, alt, fix_type) = (m.Lat, m.Lng, m.Alt, m.Status)
        if fix_type < 3:
            continue
        if mtype in ['GPS', 'GPS_RAW_INT']:
            gps_id = 1
        else:
            gps_id = 2

        if hasattr(m, 'get_srcSystem'):
            sysid = m.get_srcSystem()
        else:
            sysid = int(mlog.params.get('SYSID_THISMAV',0))

        add_data(sysid, gps_id, lat, lon, alt, fix_type)

for filename in args.logs:
    process_log(filename)

def calc_cep(data, pct):
    '''calculate CEP horizontally and alt vertically'''
    # get median pos
    count = len(data)
    mid = count//2
    mid2 = count*pct//100

    median_lat = sorted(data, key=lambda x: x.lat)[mid].lat
    median_lon = sorted(data, key=lambda x: x.lon)[mid].lon
    median_alt = sorted(data, key=lambda x: x.alt)[mid].alt

    # find point that is half way through list sorted by distance from median pos
    d1 = sorted(data, key=lambda x: distance_lat_lon(x.lat, x.lon, median_lat, median_lon))[mid2]

    # find point that is half way through list sorted by alt from median alt
    d2 = sorted(data, key=lambda x: abs(x.alt-median_alt))[mid2]

    # cep is distance from median point to the point that 50% of points are within CEP
    cep = distance_lat_lon(d1.lat, d1.lon, median_lat, median_lon)

    # alt accuracy is alt difference from median alt to point where 50% of points are within accuracy
    hep = abs(d2.alt - median_alt)

    return (cep, hep)
    
def process_CEP(sysid, gps_id, data):
    rtk5 = []
    rtk6 = []
    non_rtk = []
    for d in data:
        if d.fix_type >= 6:
            rtk6.append(d)
        elif d.fix_type == 5:
            rtk5.append(d)
        elif d.fix_type <= 4:
            non_rtk.append(d)

    # HEP50
    # CEP50
    # CEP99
    # HEP99

    # process if we have at least 100 points
    if len(rtk5) > 100:
        (cep50, hep50) = calc_cep(rtk5, 50)
        (cep99, hep99) = calc_cep(rtk5, 99)
        print("GPS-RTK5 %s:%u CEP50:%.3fm CEP99:%.2fm HEP50:%.3fm HEP99:%.3fm (%u points)" % (sysid, gps_id,
                                                                                             cep50, cep99, hep50, hep99,
                                                                                             len(rtk5)))
    if len(rtk6) > 100:
        (cep50, hep50) = calc_cep(rtk6, 50)
        (cep99, hep99) = calc_cep(rtk6, 99)
        print("GPS-RTK6 %s:%u CEP50:%.3fm CEP99:%.2fm HEP50:%.3fm HEP99:%.3fm (%u points)" % (sysid, gps_id,
                                                                                             cep50, cep99, hep50, hep99,
                                                                                             len(rtk6)))
    if len(non_rtk) > 100:
        (cep50, hep50) = calc_cep(non_rtk, 50)
        (cep99, hep99) = calc_cep(non_rtk, 99)
        print("GPS-NORM %s:%u CEP50:%.3fm CEP99:%.2fm HEP50:%.3fm HEP99:%.3fm (%u points)" % (sysid, gps_id,
                                                                                             cep50, cep99, hep50, hep99,
                                                                                             len(non_rtk)))


for sysid in sorted(DATA.keys()):
    for gps_id in sorted(DATA[sysid].keys()):
        process_CEP(sysid, gps_id, DATA[sysid][gps_id])
