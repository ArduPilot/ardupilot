#!/usr/bin/env python3

'''In ArduPilot 4.7 the distance values were moved to float to
facilitate supporting rangefinders with more than 327m of range.

The dataflash Replay log messages changes to support this.

This script will take an older log and convert those messages such
that older logs can be replayed with newer code.

AP_FLAKE8_CLEAN
'''

from argparse import ArgumentParser

import struct

from pymavlink import DFReader
from pymavlink import mavutil

new_RRNH_format = "ffB"
new_RRNI_format = "ffffBBB"


class Rewrite():
    def __init__(self, rewrite_fmt, rewrite_fmtu, rewrite_instance):
        self.rewrite_fmt = rewrite_fmt
        self.rewrite_fmtu = rewrite_fmtu
        self.rewrite_instance = rewrite_instance

    @staticmethod
    def format_to_struct(fmt):
        ret = bytes("<", 'ascii')
        for c in fmt:
            (s, _mul, _type) = DFReader.FORMAT_TO_STRUCT[c]
            ret += bytes(s, 'ascii')
        return bytes(ret)


rewrites = {
    # convert RRNI.distance_cm > RRNI.distance
    "RRNH": Rewrite(
        rewrite_fmt=lambda buf, m : buf[:3] + struct.pack(
            Rewrite.format_to_struct("BBnNZ"),
            m.Type,
            struct.calcsize(Rewrite.format_to_struct(new_RRNH_format)) + 3,  # m.Length
            bytes(m.Name, 'ascii'),
            bytes(new_RRNH_format, 'ascii'),
            bytes(m.Columns, 'ascii')
        ),
        rewrite_fmtu=lambda buf, m : buf[:3] + struct.pack(
            Rewrite.format_to_struct("QBNN"),
            m.TimeUS,
            m.FmtType,
            bytes("mm-", 'ascii'),  # new units
            bytes("00-", 'ascii')  # new mults
        ),
        rewrite_instance=lambda buf, m : buf[:3] + struct.pack(
            Rewrite.format_to_struct(new_RRNH_format),
            m.GCl * 0.01,
            m.MaxD * 0.01,
            m.NumSensors
        ),
    ),
    "RRNI": Rewrite(
        rewrite_fmt=lambda buf, m : buf[:3] + struct.pack(
            Rewrite.format_to_struct("BBnNZ"),
            m.Type,
            struct.calcsize(Rewrite.format_to_struct(new_RRNI_format)) + 3,  # m.Length
            bytes(m.Name, 'ascii'),
            bytes(new_RRNI_format, 'ascii'),
            bytes(m.Columns, 'ascii')
        ),
        rewrite_fmtu=lambda buf, m : buf[:3] + struct.pack(
            Rewrite.format_to_struct("QBNN"),
            m.TimeUS,
            m.FmtType,
            bytes("???m--#", 'ascii'),  # new units
            bytes("???0---", 'ascii')   # new mults
        ),
        rewrite_instance=lambda buf, m : buf[:3] + struct.pack(
            Rewrite.format_to_struct(new_RRNI_format),
            m.PX,
            m.PY,
            m.PZ,
            m.Dist * 0.01,
            m.Orient,
            m.Status,
            m.I
        ),
    ),
}


parser = ArgumentParser(description=__doc__)

parser.add_argument("login")
parser.add_argument("logout")

args = parser.parse_args()

login = mavutil.mavlink_connection(args.login)
output = open(args.logout, mode='wb')

type_name_map = {}


def rewrite_message(m):
    buf = bytearray(m.get_msgbuf())

    mtype = m.get_type()
    if mtype == 'FMT':
        type_name_map[m.Type] = m.Name
        if m.Name in rewrites:
            return rewrites[m.Name].rewrite_fmt(buf, m)

    if mtype == 'FMTU':
        if m.FmtType not in type_name_map:
            raise ValueError(f"Have not seen format for ID {m.FmtType}")
        name = type_name_map[m.FmtType]
        if name in rewrites:
            return rewrites[name].rewrite_fmtu(buf, m)

    if mtype in rewrites:
        return rewrites[mtype].rewrite_instance(buf, m)

    return buf


while True:
    m = login.recv_msg()
    if m is None:
        break
    output.write(rewrite_message(m))
