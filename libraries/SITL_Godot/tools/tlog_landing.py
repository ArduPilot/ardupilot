#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Print the landings in a MAVLink telemetry log, once a second from each flare
until the vehicle disarms: altitude, ground and air speed, throttle, attitude.
The autotest suite leaves a .tlog for each test at the top of the tree.

  ./tlog_landing.py autotest-1234.tlog [--seconds 70]

AP_FLAKE8_CLEAN
'''

import argparse

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("tlog")
    parser.add_argument("--seconds", type=float, default=70, help="longest to follow each landing for")
    args = parser.parse_args()

    mav = mavutil.mavlink_connection(args.tlog)
    state = {}
    flare = None
    last = 0
    while True:
        msg = mav.recv_match(type=["STATUSTEXT", "VFR_HUD", "ATTITUDE", "GLOBAL_POSITION_INT", "HEARTBEAT"])
        if msg is None:
            break
        if msg.get_srcSystem() != 1:
            continue
        t = getattr(msg, "_timestamp", 0)
        if msg.get_type() == "STATUSTEXT":
            text = msg.text
            if "Flare" in text or "disarm" in text.lower() or "Land" in text:
                print("%.1f %s" % (t, text))
            if "Flare" in text:
                flare = t
            continue
        state[msg.get_type()] = msg
        if flare is None or len(state) < 4 or t - last < 1.0:
            continue
        last = t
        vfr = state["VFR_HUD"]
        att = state["ATTITUDE"]
        pos = state["GLOBAL_POSITION_INT"]
        armed = state["HEARTBEAT"].base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        print("t+%5.1f alt=%6.2f gs=%5.2f as=%5.2f thr=%3d pitch=%6.1f roll=%6.1f armed=%d" % (
            t - flare, pos.relative_alt * 1e-3, vfr.groundspeed, vfr.airspeed, vfr.throttle,
            att.pitch * 57.3, att.roll * 57.3, 1 if armed else 0))
        if not armed or t - flare > args.seconds:
            flare = None


if __name__ == "__main__":
    main()
