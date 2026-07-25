#!/usr/bin/env python3
"""Minimal MAVLink helper to connect to the running ArduRover SITL on an
auxiliary TCP port and inspect / fix arming, then drive test manoeuvres."""
import os
import sys
import time
from pymavlink import mavutil


def _logdir():
    """Directory for run logs, next to this script (created on demand)."""
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(d, exist_ok=True)
    return d + os.sep


def connect():
    for port in (5762, 5763, 5760):
        try:
            m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=250, source_component=1)
            m.wait_heartbeat(timeout=6)
            print(f"[ok] connected on tcp:{port}  sys={m.target_system} comp={m.target_component}")
            return m
        except Exception as e:
            print(f"[--] tcp:{port} failed: {e}")
    sys.exit("no MAVLink port available")


def get_params(m, names, timeout=6):
    for n in names:
        m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
    out, t = {}, time.time()
    while time.time() - t < timeout and len(out) < len(names):
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg and msg.param_id in names:
            out[msg.param_id] = msg.param_value
    return out


def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value), ptype)
    t = time.time()
    while time.time() - t < 4:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg and msg.param_id == name:
            print(f"[set] {name} = {msg.param_value}")
            return True
    print(f"[!!] {name} set not confirmed")
    return False


def drain_statustext(m, secs=3.0):
    t = time.time()
    while time.time() - t < secs:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg:
            print(f"    AP: {msg.text}")


def cmd_status(m):
    p = get_params(m, ["ARMING_SKIPCHK", "SCHED_LOOP_RATE", "FS_EKF_THRESH"])
    print("params:", p)
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"mode={mavutil.mode_string_v10(hb)} armed={armed}")
    print("recent messages:")
    drain_statustext(m, 3)


def cmd_fix(m):
    # Match ArduPilot's loop-rate target to what the (rendering-heavy) sim can
    # sustain, so the loop/gyro pre-arm checks pass. Needs a SITL restart.
    set_param(m, "SCHED_LOOP_RATE", 30, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
    # -1 = skip all non-mandatory checks (ARMING_CHECK was renamed
    # ARMING_SKIPCHK in ArduPilot 4.7 with inverted bitmask semantics: bits
    # are checks to SKIP, so 0 - the old "disable everything" value - now
    # means the opposite, "skip nothing").
    set_param(m, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    print(">> params set. RESTART SITL for SCHED_LOOP_RATE to take effect.")


def set_mode(m, name):
    m.set_mode(m.mode_mapping()[name])
    time.sleep(0.5)


def arm(m):
    set_param(m, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_mode(m, "MANUAL")
    # force-arm (param2 = 21196) bypasses any remaining pre-arm checks
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    t = time.time()
    while time.time() - t < 6:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[armed]")
            return True
        drain_statustext(m, 0.2)
    print("[!!] arm failed")
    return False


def set_reverse(m, reverse):
    """Engage reverse ('gear') so Rover BACKS UP to a target instead of doing a
    forward K-turn. Needs a reversible ESC (see rover.parm). 1=reverse, 0=fwd."""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, 0,
        1 if reverse else 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.3)
    print(f"    [gear] {'REVERSE' if reverse else 'forward'}")


def guided_goto_local(m, north, east, wait=15, reverse=False):
    """Drive to a local NED offset (metres) from current position in GUIDED.
    With reverse=True the rover backs up to it (throttle < neutral)."""
    set_mode(m, "GUIDED")
    set_reverse(m, reverse)
    # type_mask: use position only (ignore vel/accel/yaw)
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        0b0000111111111000, north, east, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    t = time.time()
    while time.time() - t < wait:
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        if msg:
            print(f"    pos N={msg.x:6.2f} E={msg.y:6.2f}  v={msg.vx:5.2f}")
    set_reverse(m, False)  # always leave it in forward gear


import math

HOME_LAT, HOME_LON = 28.5016472, 77.3921611


def _offset(n, e):
    dlat = n / 111320.0
    dlon = e / (111320.0 * math.cos(math.radians(HOME_LAT)))
    return HOME_LAT + dlat, HOME_LON + dlon


def _offset_from(lat0, lon0, n, e):
    dlat = n / 111320.0
    dlon = e / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def get_pos(m, timeout=5):
    """Return (lat, lon, north_m, east_m): current global + EKF-local NED."""
    ll = ne = None
    t = time.time()
    while time.time() - t < timeout and (ll is None or ne is None):
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
                           blocking=True, timeout=1)
        if not msg:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            ll = (msg.lat / 1e7, msg.lon / 1e7)
        else:
            ne = (msg.x, msg.y)
    return (ll[0], ll[1], ne[0], ne[1]) if ll and ne else None


def upload_square(m, side=10.0, origin=None):
    # Square corners as NED metres from the origin. seq 0 is the start corner
    # (current=1); anchoring the origin at the rover's CURRENT position makes
    # every run start on corner 0 (no long approach leg polluting the metric).
    lat0, lon0 = (origin[0], origin[1]) if origin else (HOME_LAT, HOME_LON)
    corners = [(side, 0), (side, side), (0, side), (0, 0)]
    items = [(lat0, lon0, 1)]  # seq 0 = origin corner (current=1)
    for n, e in corners:
        lat, lon = _offset_from(lat0, lon0, n, e)
        items.append((lat, lon, 0))

    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = set()
    t = time.time()
    while time.time() - t < 20:
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                           blocking=True, timeout=2)
        if not msg:
            continue
        if msg.get_type() == "MISSION_ACK":
            if len(sent) >= len(items):
                print(f"[mission] ack type={msg.type} ({len(items)} items)")
                return msg.type == 0
            continue  # ignore stray acks before all items are sent
        seq = msg.seq
        lat, lon, cur = items[seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, cur, 1,
            0, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), 0)
        sent.add(seq)
    print("[mission] upload timed out")
    return False


def stop(m):
    set_mode(m, "HOLD")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    print("[stop] HOLD + disarm sent")


def tune(m):
    """Tighter waypoint tracking for crisp corners in a small square."""
    set_param(m, "WP_RADIUS", 1.5)       # was 3.0 - arrive closer to each corner
    set_param(m, "CRUISE_SPEED", 1.3)    # was 2.0 - slower = less corner overshoot
    set_param(m, "TURN_RADIUS", 1.85)    # rover's physical min (wheelbase/tan(maxsteer))
    set_param(m, "ATC_STR_RAT_FF", 0.9)  # a touch more steering feed-forward for tracking


def run_square(m, side=10.0):
    tune(m)
    print(f"-> uploading {side} m square ({len(range(5))} legs)")
    if not upload_square(m, side):
        return
    arm(m)
    set_mode(m, "AUTO")
    last_wp = -1
    t = time.time()
    while time.time() - t < 120:
        msg = m.recv_match(type=["MISSION_CURRENT", "LOCAL_POSITION_NED", "STATUSTEXT"],
                           blocking=True, timeout=1)
        if not msg:
            continue
        ty = msg.get_type()
        if ty == "MISSION_CURRENT" and msg.seq != last_wp:
            last_wp = msg.seq
            print(f"    >> heading to waypoint {msg.seq}")
        elif ty == "LOCAL_POSITION_NED":
            print(f"    pos N={msg.x:6.2f} E={msg.y:6.2f}")
        elif ty == "STATUSTEXT" and ("ission" in msg.text or "omplete" in msg.text):
            print(f"    AP: {msg.text}")
            if "omplete" in msg.text:
                break


# --------------------- tracking-quality measurement ---------------------

def _seg_dist(px, py, ax, ay, bx, by):
    """Perpendicular distance from point P to segment AB (metres)."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def setp(m, pairs):
    """setp NAME VALUE NAME VALUE ...  (INT params inferred from int-looking values)."""
    for i in range(0, len(pairs) - 1, 2):
        name, val = pairs[i], float(pairs[i + 1])
        ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32
                 if float(val).is_integer() and name.split("_")[0] in ("NAVL1", "WP", "SCHED", "ARMING")
                 else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # NAVL1_PERIOD/DAMPING are REAL; only genuinely-int params use INT
        if name in ("NAVL1_PERIOD", "NAVL1_DAMPING", "WP_RADIUS", "WP_SPEED", "WP_OVERSHOOT"):
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        set_param(m, name, val, ptype)


def run_track(m, side=12.0):
    """Run the square in AUTO, record actual path, report cross-track error vs
    the planned square legs (the tracking-quality metric we optimise).

    Does NOT change tuning params - set those via `setp` first so each run
    measures exactly the configuration under test."""
    print(f"-> tracking run: {side} m square")
    # request a steady position stream, then anchor the square under the rover
    for f in (33, 32):  # GLOBAL_POSITION_INT, LOCAL_POSITION_NED
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, f, 100000, 0, 0, 0, 0, 0)
    pos = get_pos(m)
    if not pos:
        print("no position fix"); return
    lat0, lon0, n0, e0 = pos
    print(f"   anchored at N={n0:.1f} E={e0:.1f}")
    if not upload_square(m, side, origin=(lat0, lon0)):
        return
    # planned legs in the SAME EKF-local NED frame as LOCAL_POSITION_NED,
    # offset by the rover's start position (which is corner 0).
    C = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    legs = [((n0 + a[0], e0 + a[1]), (n0 + b[0], e0 + b[1])) for a, b in zip(C[:-1], C[1:])]
    corners = [(n0 + c[0], e0 + c[1]) for c in C]  # for straight-vs-corner split
    # enable steering-rate PID telemetry so we can watch desired vs achieved
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 194, 50000, 0, 0, 0, 0, 0)  # PID_TUNING
    arm(m)
    set_mode(m, "AUTO")
    samples, errs = [], []
    des, ach = [], []          # steering-rate desired / achieved (deg/s)
    last_wp, done = -1, False
    t0 = time.time()
    while time.time() - t0 < 180 and not done:
        msg = m.recv_match(type=["MISSION_CURRENT", "LOCAL_POSITION_NED", "STATUSTEXT", "PID_TUNING"],
                           blocking=True, timeout=1)
        if not msg:
            continue
        ty = msg.get_type()
        if ty == "LOCAL_POSITION_NED":
            n, e = msg.x, msg.y
            # only score once actually moving through the mission (skip start)
            if last_wp >= 1:
                ct = min(_seg_dist(n, e, a[0], a[1], b[0], b[1]) for a, b in legs)
                cd = min(math.hypot(n - cx, e - cy) for cx, cy in corners)  # dist to nearest corner
                errs.append(ct)
                samples.append((round(time.time() - t0, 2), round(n, 2), round(e, 2), round(ct, 3), round(cd, 2)))
        elif ty == "PID_TUNING" and getattr(msg, "axis", None) == 5 and last_wp >= 1:  # 5 = steering
            des.append(msg.desired); ach.append(msg.achieved)  # rad/s
        elif ty == "MISSION_CURRENT" and msg.seq != last_wp:
            last_wp = msg.seq
            print(f"    >> waypoint {msg.seq}")
        elif ty == "STATUSTEXT" and "omplete" in msg.text:
            print(f"    AP: {msg.text}")
            done = True
    stop(m)
    base = _logdir()
    with open(base + "track_last.csv", "w") as f:
        f.write("t,N,E,cross_track_m,corner_dist_m\n")
        for s in samples:
            f.write(",".join(map(str, s)) + "\n")
    if not errs:
        print("no samples scored"); return
    rms = math.sqrt(sum(x * x for x in errs) / len(errs))
    # Split straight-leg (bowing) from corner error. STRAIGHT = far from any
    # corner (> WP_RADIUS + margin); CORNER = within the rounding zone.
    ZONE = 4.0
    straight = [s[3] for s in samples if s[4] > ZONE]
    corner = [s[3] for s in samples if s[4] <= ZONE]
    def rms_max(v):
        return (math.sqrt(sum(x * x for x in v) / len(v)), max(v)) if v else (0, 0)
    sr, sm = rms_max(straight)
    cr, cm = rms_max(corner)
    print(f"\n==== TRACKING n={len(errs)}  RMS={rms:.3f} m  MAX={max(errs):.3f} m ====")
    print(f"     STRAIGHT-leg (bowing): RMS={sr:.3f} MAX={sm:.3f} m  (n={len(straight)})")
    print(f"     CORNER-zone          : RMS={cr:.3f} MAX={cm:.3f} m  (n={len(corner)})")


def cmd_radar(m, duration=15.0):
    """Show live HLK-LD2451 proximity status end to end: PRX1 config,
    SYS_STATUS health bits, and any OBSTACLE_DISTANCE sectors ArduPilot has
    actually built from the injected radar frames. This confirms the full
    chain (Webots radar -> ardupilot_bridge.py -> radar_mav.py -> SITL UART
    -> AP_Proximity_HLK_LD2451 -> AP_Proximity database), not just that
    frames were sent."""
    p = get_params(m, ["PRX1_TYPE", "AVOID_ENABLE", "OA_TYPE"])
    print(f"params: {p}")
    if p.get("PRX1_TYPE") != 19:
        print("[!!] PRX1_TYPE is not 19 (HLK-LD2451) - see rover.parm")

    PROXIMITY_BIT = 1 << 20  # MAV_SYS_STATUS_SENSOR_PROXIMITY / OBSTACLE_AVOIDANCE
    for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
                       (mavutil.mavlink.MAVLINK_MSG_ID_OBSTACLE_DISTANCE, 5)):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    print(f"watching for {duration:.0f}s (Ctrl-C to stop early) ...")
    t0 = time.time()
    seen_hit = False
    while time.time() - t0 < duration:
        msg = m.recv_match(type=["SYS_STATUS", "OBSTACLE_DISTANCE"], blocking=True, timeout=1)
        if not msg:
            continue
        if msg.get_type() == "SYS_STATUS":
            present = bool(msg.onboard_control_sensors_present & PROXIMITY_BIT)
            enabled = bool(msg.onboard_control_sensors_enabled & PROXIMITY_BIT)
            healthy = bool(msg.onboard_control_sensors_health & PROXIMITY_BIT)
            print(f"    [sys_status] proximity present={present} enabled={enabled} healthy={healthy}")
        else:
            hits = [(i, d) for i, d in enumerate(msg.distances) if d < msg.max_distance]
            if hits:
                seen_hit = True
                i, d = min(hits, key=lambda h: h[1])
                inc = getattr(msg, "increment_f", None) or msg.increment
                print(f"    [obstacle_distance] nearest sector {i} (~{i * inc:.0f}deg): "
                      f"{d / 100:.2f} m  ({len(hits)} sector(s) with a hit)")

    print("[ok] AP_Proximity reported live obstacles" if seen_hit else
          "[!!] no OBSTACLE_DISTANCE hits seen - check radar_mav.py is running, "
          "target_car is moving within radar range, and PRX1_TYPE=19")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    mav = connect()
    if cmd == "status":
        cmd_status(mav)
    elif cmd == "fix":
        cmd_fix(mav)
    elif cmd == "arm":
        arm(mav)
    elif cmd == "fwdback":
        arm(mav)
        print("-> 5 m forward"); guided_goto_local(mav, 5, 0)
        # back up in REVERSE gear (not a forward turn-around)
        print("-> 5 m back (reverse)"); guided_goto_local(mav, -5, 0, reverse=True)
    elif cmd == "back":
        dist = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
        arm(mav)
        print(f"-> {dist} m back (reverse)"); guided_goto_local(mav, -dist, 0, reverse=True)
    elif cmd == "square":
        side = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
        run_square(mav, side)
    elif cmd == "track":
        side = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
        run_track(mav, side)
    elif cmd == "setp":
        setp(mav, sys.argv[2:])
    elif cmd == "getp":
        print(get_params(mav, sys.argv[2:]))
    elif cmd == "radar":
        duration = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
        cmd_radar(mav, duration)
    elif cmd == "turntest":
        # From a north-facing spawn, drive to a point 6 m North / 10 m East.
        # A correct rover steers RIGHT (heading should climb toward ~060 deg).
        arm(mav)
        print("-> GUIDED to N=6 E=10 (correct = steer right, heading increases)")
        guided_goto_local(mav, 6, 10, wait=18)
        stop(mav)
    elif cmd == "stop":
        stop(mav)
    elif cmd == "steertest":
        # Controlled test: full-right steering + gentle forward throttle, in
        # MANUAL, while reporting ArduPilot's steering output and the rover's
        # actual heading response. Compare with bridge_log.csv (c0_steer,
        # gps_yaw) to determine the steering sign empirically.
        arm(mav)
        print("commanding steering RIGHT (rc1=1900) + throttle fwd (rc3=1600) for 6s")
        t0 = time.time()
        yaw0 = None
        while time.time() - t0 < 6:
            mav.mav.rc_channels_override_send(mav.target_system, mav.target_component,
                                              1900, 0, 1600, 0, 0, 0, 0, 0)
            msg = mav.recv_match(type="VFR_HUD", blocking=True, timeout=0.5)
            if msg:
                if yaw0 is None:
                    yaw0 = msg.heading
                print(f"    heading={msg.heading:3d}deg  gspd={msg.groundspeed:.2f}")
            time.sleep(0.1)
        mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, 0, 0, 0, 0, 0, 0, 0, 0)
        stop(mav)
        print(f"heading change: {yaw0} -> last (right turn should INCREASE heading / turn clockwise)")
