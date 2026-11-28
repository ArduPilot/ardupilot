# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Fly a vehicle's check mission over MAVLink, for the checks in this directory.
The copter takes off in GUIDED, holds, flies 20 m north and lands; the plane
flies an AUTO runway takeoff, a 600 m x 300 m circuit and an automatic landing.

AP_FLAKE8_CLEAN
'''

import math
import sys
import time

from pymavlink import mavutil

state = {}
failures = []
mav = None
overall_timeout = 600.0


def check(ok, what):
    print("%s: %s" % ("PASS" if ok else "FAIL", what))
    if not ok:
        failures.append(what)


def pump(seconds):
    '''process incoming messages for a while, keeping the latest of each type'''
    end = time.time() + seconds
    while time.time() < end:
        m = mav.recv_match(blocking=True, timeout=0.1)
        # ignore other ground stations, such as MAVProxy, forwarded by the vehicle
        if m is None or m.get_srcSystem() != mav.target_system:
            continue
        t = m.get_type()
        state[t] = m
        if t == "STATUSTEXT":
            print("  AP: %s" % m.text)


def wait_until(cond, timeout, what, each=None):
    end = time.time() + timeout
    while time.time() < end:
        pump(0.2)
        if each:
            each()
        if cond():
            return True
    print("timed out waiting for %s" % what)
    return False


def armed():
    hb = state.get("HEARTBEAT")
    return hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0


def rel_alt():
    gpi = state.get("GLOBAL_POSITION_INT")
    return gpi.relative_alt * 0.001 if gpi else 0.0


def local_ne():
    lp = state.get("LOCAL_POSITION_NED")
    return (lp.x, lp.y) if lp else (0.0, 0.0)


def arm(mode):
    '''arming succeeds once the EKF has converged and GPS has a fix'''
    deadline = time.time() + overall_timeout
    while not armed():
        if time.time() > deadline:
            check(False, "arm")
            sys.exit(1)
        mav.set_mode(mode)
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(2)
    check(True, "armed in %s" % mode)


def fly_copter(alt=10.0):
    arm("GUIDED")
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    check(wait_until(lambda: rel_alt() > alt - 0.5, 60, "takeoff"), "takeoff to %.0fm" % alt)

    # hold position and record how steady it is
    alts = []
    tilt = 0.0
    end = time.time() + 10
    while time.time() < end:
        pump(0.1)
        alts.append(rel_alt())
        att = state.get("ATTITUDE")
        if att:
            tilt = max(tilt, math.degrees(max(abs(att.roll), abs(att.pitch))))
    print("hover: alt %.2f..%.2fm, max tilt %.1f deg" % (min(alts), max(alts), tilt))
    check(max(alts) - min(alts) < 1.0 and abs(sum(alts) / len(alts) - alt) < 1.0, "altitude hold within 1m")
    check(tilt < 5.0, "hover tilt under 5 degrees")

    # fly 20m north at the same altitude
    n0, e0 = local_ne()
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000110111111000, n0 + 20, e0, -alt, 0, 0, 0, 0, 0, 0, 0, 0)
    arrived = wait_until(lambda: math.hypot(local_ne()[0] - n0 - 20, local_ne()[1] - e0) < 1.0, 60,
                         "position target")
    check(arrived, "flew 20m north to within 1m")

    mav.set_mode("LAND")
    check(wait_until(lambda: not armed(), 90, "landing and disarm"), "landed and disarmed")


def upload_mission(items):
    '''items are (command, lat, lon, alt, param1)'''
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    pump(0.5)
    mav.mav.mission_count_send(mav.target_system, mav.target_component, len(items))
    sent = set()
    end = time.time() + 30
    while time.time() < end:
        m = mav.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"], blocking=True, timeout=1)
        if m is None or m.get_srcSystem() != mav.target_system or m.target_system != mav.source_system:
            continue
        if m.get_type() == "MISSION_ACK":
            return m.type == mavutil.mavlink.MAV_MISSION_ACCEPTED and len(sent) == len(items)
        cmd, lat, lon, alt, p1 = items[m.seq]
        mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component, m.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1,
            p1, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), alt)
        sent.add(m.seq)
    return False


def fly_plane():
    # wait for a position so the mission can be laid out around home
    wait_until(lambda: state.get("GPS_RAW_INT") is not None and state["GPS_RAW_INT"].fix_type >= 3, 120, "GPS")
    pump(1)
    gpi = state["GLOBAL_POSITION_INT"]
    lat0, lon0 = gpi.lat * 1e-7, gpi.lon * 1e-7

    def offset(north, east):
        return (lat0 + math.degrees(north / 6378137.0),
                lon0 + math.degrees(east / (6378137.0 * math.cos(math.radians(lat0)))))

    cmd = mavutil.mavlink
    circuit = [(300, 0, 40), (300, 300, 40), (-300, 300, 30), (-300, 0, 25)]
    items = [(cmd.MAV_CMD_NAV_WAYPOINT, lat0, lon0, 0, 0),
             (cmd.MAV_CMD_NAV_TAKEOFF, lat0, lon0, 30, 12)]
    items += [(cmd.MAV_CMD_NAV_WAYPOINT,) + offset(n, e) + (alt, 0) for n, e, alt in circuit]
    items += [(cmd.MAV_CMD_NAV_LAND,) + offset(50, 0) + (0, 0)]
    land_seq = len(items) - 1
    check(upload_mission(items), "mission uploaded")

    arm("AUTO")
    stats = {"min_airspeed": 1e9, "max_roll": 0.0}

    def track():
        hud = state.get("VFR_HUD")
        att = state.get("ATTITUDE")
        seq = state["MISSION_CURRENT"].seq if "MISSION_CURRENT" in state else 0
        if hud and 2 <= seq < land_seq and rel_alt() > 10:
            stats["min_airspeed"] = min(stats["min_airspeed"], hud.airspeed)
        if att and 2 <= seq < land_seq:
            stats["max_roll"] = max(stats["max_roll"], abs(math.degrees(att.roll)))

    def seq():
        return state["MISSION_CURRENT"].seq if "MISSION_CURRENT" in state else 0

    check(wait_until(lambda: seq() >= 2, 90, "takeoff", track), "took off and climbed to 30m")
    check(wait_until(lambda: seq() >= land_seq, 300, "circuit", track), "flew the circuit")
    print("circuit: min airspeed %.1f m/s, max roll %.0f deg" % (stats["min_airspeed"], stats["max_roll"]))
    check(stats["min_airspeed"] > 10.0, "airspeed stayed above 10 m/s")
    check(wait_until(lambda: not armed(), 180, "landing and disarm", track), "landed and disarmed")
    n, e = local_ne()
    print("stopped %.0fm north, %.0fm east of home" % (n, e))
    check(abs(e) < 15 and -100 < n < 200, "landed on the runway line")


def run(vehicle, connect, alt=10.0, timeout=600.0):
    '''connect and fly the vehicle's mission, returning the number of failures'''
    global mav, overall_timeout
    state.clear()
    del failures[:]
    overall_timeout = timeout
    mav = mavutil.mavlink_connection(connect, source_system=254)
    while True:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=60)
        if hb is None:
            print("no heartbeat from the vehicle")
            return 1
        if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            mav.target_system = hb.get_srcSystem()
            mav.target_component = hb.get_srcComponent()
            break
    print("connected to system %u" % mav.target_system)
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    if vehicle == "copter":
        fly_copter(alt)
    else:
        fly_plane()
    print("%d failure(s)" % len(failures))
    return len(failures)
