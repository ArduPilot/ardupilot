#!/usr/bin/env python3
"""
mavlink_xplane.py — ArduPilot fmuv3-hil ↔ X-Plane bridge.

Mirrors SIM_XPlane.cpp protocol for real hardware via MAVLink.

Data injected into the flight controller
─────────────────────────────────────────
  HIL_SENSOR              gyro, accel, baro, diff-pressure  @50 Hz
  GPS_INPUT               lat, lon, alt, vel NED             @5 Hz
  VISION_POSITION_ESTIMATE roll, pitch, yaw                  @50 Hz

Data received from the flight controller
─────────────────────────────────────────
  SERVO_OUTPUT_RAW        → X-Plane DREFs (control surfaces)

X-Plane wire protocol
─────────────────────
  Same as SIM_XPlane.cpp: DSEL / DREF / RREF / DATA@ rows

Board setup
───────────
  Flash ArduPlane built for fmuv3-hil (safety switch disabled).
  All required parameters are set automatically via defaults.parm.

Usage:
    python3 mavlink_xplane.py --pixhawk /dev/tty.usbmodem1101
    python3 mavlink_xplane.py --xplane-host 192.168.1.10 --auto --debug
"""

import argparse
import math
import select
import socket
import struct
import time

from pymavlink import mavutil

# ── constants ────────────────────────────────────────────────────────────────
KNOTS_TO_MS  = 0.514444
FEET_TO_M    = 0.3048
GRAVITY_MSS  = 9.80665
DEG_TO_RAD   = math.pi / 180.0

# Standard atmosphere — for baro synthesis from altitude
SEA_PRESSURE_HPA = 1013.25
LAPSE_RATE       = 0.0065   # K/m
T0_K             = 288.15   # sea-level temperature K


def alt_to_pressure(alt_m: float) -> float:
    """Return absolute pressure (hPa) from altitude (m) via ISA model."""
    return SEA_PRESSURE_HPA * ((1.0 - LAPSE_RATE * alt_m / T0_K) ** 5.2561)


def airspeed_to_diff_pressure(airspeed_ms: float) -> float:
    """Return differential pressure (hPa) from true airspeed (m/s)."""
    rho = 1.225  # kg/m³ at sea level
    return 0.5 * rho * airspeed_ms * airspeed_ms / 100.0   # Pa → hPa


# ── HIL_SENSOR fields_updated bitmask (MAVLink spec) ────────────────────────
HIL_SENSOR_ACCEL    = 0x007   # xacc | yacc | zacc
HIL_SENSOR_GYRO     = 0x038   # xgyro | ygyro | zgyro
HIL_SENSOR_MAG      = 0x1C0   # xmag | ymag | zmag
HIL_SENSOR_BARO     = 0xE00   # abs_pressure | diff_pressure | pressure_alt
HIL_SENSOR_TEMP     = 0x1000
HIL_FIELDS = (HIL_SENSOR_ACCEL | HIL_SENSOR_GYRO | HIL_SENSOR_MAG |
              HIL_SENSOR_BARO  | HIL_SENSOR_TEMP)

# ── GPS time ─────────────────────────────────────────────────────────────────
# GPS epoch: 6 Jan 1980. Leap seconds as of 2024: 18.
_GPS_EPOCH_UNIX = 315964800
_GPS_LEAP_SECONDS = 18

def gps_time() -> tuple:
    """Return (time_week, time_week_ms) from current UTC system time."""
    t = time.time() + _GPS_LEAP_SECONDS - _GPS_EPOCH_UNIX
    week    = int(t / 604800)
    tow_ms  = int((t % 604800) * 1000)
    return week, tow_ms


# ── Magnetic field helpers (mirrors SIM_Aircraft::update_mag_field_bf) ───────

def igrf_dipole_ned_gauss(lat_deg: float, lon_deg: float) -> list:
    """
    IGRF-14 (2025) degree-1 dipole approximation of Earth magnetic field in NED,
    returned in Gauss (HIL_SENSOR units).  Matches AP_Declination intensity to
    within ~20% globally; direction error <5° — sufficient for HIL compass.

    Mirrors the field model used by Aircraft::update_mag_field_bf() in
    SIM_Aircraft.cpp, without requiring the binary AP_Declination table.
    """
    # IGRF-14 epoch 2025.0 degree-1 coefficients, nT
    g10, g11, h11 = -29351.0, -1410.0, 4545.0
    lat   = math.radians(lat_deg)
    lon   = math.radians(lon_deg)
    colat = math.pi / 2.0 - lat          # geocentric colatitude ≈ geographic
    sc, cc = math.sin(colat), math.cos(colat)
    sl, cl = math.sin(lon),   math.cos(lon)
    gsl    = g11 * cl + h11 * sl         # g11·cosλ + h11·sinλ
    # NED components, nT → Gauss (* 1e-5)
    bn = -(g10 * sc - gsl * cc) * 1e-5
    be =  (g11 * sl - h11 * cl) * 1e-5
    bd = -2.0 * (g10 * cc + gsl * sc) * 1e-5
    return [bn, be, bd]


def ned_to_body(vec_ned: list, roll_r: float, pitch_r: float, yaw_r: float) -> list:
    """
    Rotate a NED vector to body frame using ZYX Euler angles (rad).
    Mirrors dcm.transposed() * mag_ef in SIM_Aircraft::update_mag_field_bf().
    """
    sr, cr = math.sin(roll_r),  math.cos(roll_r)
    sp, cp = math.sin(pitch_r), math.cos(pitch_r)
    sy, cy = math.sin(yaw_r),   math.cos(yaw_r)
    n, e, d = vec_ned
    bx = cp * cy * n + cp * sy * e - sp * d
    by = (sr * sp * cy - cr * sy) * n + (sr * sp * sy + cr * cy) * e + sr * cp * d
    bz = (cr * sp * cy + sr * sy) * n + (cr * sp * sy - sr * cy) * e + cr * cp * d
    return [bx, by, bz]

# ── X-Plane DATA@ row codes (SIM_XPlane.cpp enum) ───────────────────────────
ROW_TIMES          = 1
ROW_SPEED          = 3
ROW_GLOAD          = 4
ROW_ANG_VEL        = 16
ROW_PITCH_ROLL_HDG = 17
ROW_LAT_LON_ALT    = 20
ROW_LOC_VEL_DIST   = 21
REQUIRED_ROWS = [
    ROW_TIMES, ROW_SPEED, ROW_GLOAD,
    ROW_ANG_VEL, ROW_PITCH_ROLL_HDG,
    ROW_LAT_LON_ALT, ROW_LOC_VEL_DIST,
]

RREF_VERSION = 1


# ── X-Plane UDP wire protocol ────────────────────────────────────────────────

def xp_dsel(sock, addr, rows):
    padded = (list(rows) + [0] * 8)[:8]
    sock.sendto(b'DSEL\x00' + struct.pack('<8I', *padded), addr)


def xp_dref(sock, addr, name: str, value: float):
    name_b = name.encode() + b'\x00' * (500 - len(name))
    sock.sendto(b'DREF\x00' + struct.pack('<f', value) + name_b, addr)


def xp_rref(sock, addr, name: str, code: int, rate_hz: int):
    name_b = name.encode() + b'\x00' * (400 - len(name))
    sock.sendto(b'RREF\x00' + struct.pack('<II', rate_hz, code) + name_b, addr)


def xp_parse(pkt: bytes) -> dict:
    """Parse DATA@ packet → {row_code: (v0…v7)}.  values[N] == C++ data[N+1]."""
    if len(pkt) < 5 or pkt[:4] != b'DATA':
        return {}
    rows = {}
    off = 5
    while off + 36 <= len(pkt):
        code   = struct.unpack_from('<I', pkt, off)[0]
        values = struct.unpack_from('<8f', pkt, off + 4)
        rows[code] = values
        off += 36
    return rows


# ── DREF / servo map ──────────────────────────────────────────────────────────
def _angle(pwm, r=1.0): return r * (pwm - 1500) / 500.0
def _range(pwm, r=1.0): return r * (pwm - 1000) / 1000.0

SERVO_DREFS = [
    ('sim/joystick/yoke_roll_ratio',             0, _angle),
    ('sim/joystick/yoke_pitch_ratio',            1, _angle),
    ('sim/flightmodel/engine/ENGN_thro_use[0]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[1]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[2]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[3]',  2, _range),
    ('sim/joystick/yoke_heading_ratio',           3, _angle),
    ('sim/cockpit2/controls/flap_ratio',          4, _range),
]


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='fmuv3-hil ↔ X-Plane bridge (HIL_SENSOR + GPS_INPUT + RC override)'
    )
    ap.add_argument('--pixhawk',     default='udpin:0.0.0.0:14560',
                    help='MAVLink connection  (default: udpin:0.0.0.0:14560)')
    ap.add_argument('--xplane-host', default='127.0.0.1')
    ap.add_argument('--xplane-port', type=int, default=49000)
    ap.add_argument('--bind-port',   type=int, default=49005,
                    help='Local port for X-Plane DATA@ (default: 49005)')
    ap.add_argument('--gps-rate',    type=int, default=5,
                    help='GPS_INPUT rate Hz (default: 5)')
    ap.add_argument('--hil-rate',    type=int, default=100,
                    help='Maximum HIL_SENSOR rate Hz (default: 100); '
                         'actual rate follows X-Plane physics rate')
    # ap.add_argument('--auto',        action='store_true',
    #                 help='Set flight mode to AUTO once ArduPilot is ready')
    ap.add_argument('--debug',       action='store_true')
    args = ap.parse_args()

    xp_addr = (args.xplane_host, args.xplane_port)

    # ── MAVLink ──────────────────────────────────────────────────────────────
    print(f'[MAV] Connecting to {args.pixhawk} …')
    mav = mavutil.mavlink_connection(args.pixhawk, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] Heartbeat  sysid={mav.target_system}  '
          f'compid={mav.target_component}')

    # request SERVO_OUTPUT_RAW stream
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 50, 1)

    # ── X-Plane socket ───────────────────────────────────────────────────────
    xp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    xp_sock.bind(('0.0.0.0', args.bind_port))
    xp_sock.setblocking(False)
    print(f'[XP]  Listening on :{args.bind_port}  →  {xp_addr}')

    xp_dsel(xp_sock, xp_addr, REQUIRED_ROWS)
    xp_rref(xp_sock, xp_addr, 'sim/version/xplane_internal_version', RREF_VERSION, 1)

    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_joystick',  1.0)
    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_throttles', 1.0)

    xp_dref(xp_sock, xp_addr, 'sim/flightmodel/controls/parkbrake', 1.0)
    print('[XP]  Parking brake SET')
    print(f'[XP]  DSEL rows {REQUIRED_ROWS}')

    # ── state ────────────────────────────────────────────────────────────────
    xp_time    = 0.0
    lat        = 0.0
    lon        = 0.0
    alt_m      = 0.0
    roll_r     = 0.0
    pitch_r    = 0.0
    yaw_r      = 0.0
    # gyro  — ArduPilot body frame [x, y, z] rad/s
    gyro       = [0.0, 0.0, 0.0]
    # accel — ArduPilot body frame [x, y, z] m/s²
    accel_body = [0.0, 0.0, -GRAVITY_MSS]
    # velocity NED m/s
    vel_ned    = [0.0, 0.0, 0.0]
    airspeed   = 0.0

    xp_version      = 0
    pos_valid       = False
    xp_sensor_updated = False   # True when fresh IMU/attitude rows arrive from XP

    gps_ivl     = 1.0 / args.gps_rate
    hil_min_ivl = 1.0 / args.hil_rate   # minimum interval between HIL sends

    last_gps   = 0.0
    last_hil   = 0.0
    last_dsel  = time.monotonic()
    last_debug = time.monotonic()
    last_srv   = time.monotonic()
    srv_count  = 0     # SERVO_OUTPUT_RAW messages received since last print
    dref_sent  = False # True after first DREF batch is sent to X-Plane

    print('\nRunning — Ctrl+C to stop\n')

    while True:
        now = time.monotonic()

        # Re-subscribe DSEL + refresh overrides every 5 s.
        # X-Plane resets override flags on scene reload / unpause.
        if now - last_dsel >= 5.0:
            xp_dsel(xp_sock, xp_addr, REQUIRED_ROWS)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_joystick',  1.0)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_throttles', 1.0)
            last_dsel = now

        # ── receive X-Plane DATA@ ────────────────────────────────────────────
        # Block until XP sends data or the next GPS/HIL deadline arrives.
        # This replaces busy-polling and lets the OS schedule other processes.
        timeout = max(0.0, min(
            (last_gps + gps_ivl) - now,
            hil_min_ivl,
        ))
        readable, _, _ = select.select([xp_sock], [], [], timeout)

        xp_sensor_updated = False
        try:
            while readable:   # only drain if select() said data is available
                pkt, _ = xp_sock.recvfrom(65536)

                if pkt[:4] == b'RREF' and len(pkt) >= 13:
                    code, val = struct.unpack_from('<If', pkt, 5)
                    if code == RREF_VERSION and xp_version == 0:
                        xp_version = int(val)
                        print(f'[XP]  X-Plane {xp_version // 10000} '
                              f'(build {xp_version})')
                    continue

                rows = xp_parse(pkt)
                if not rows:
                    continue

                is_xp12 = (xp_version // 10000) >= 12

                # Row 1 — Times: values[2] = data[3] = elapsed sim time (s)
                if ROW_TIMES in rows:
                    xp_time = rows[ROW_TIMES][2]

                # Row 3 — Speed: values[0] = data[0] = IAS kts  (matches SIM_XPlane.cpp data[0])
                if ROW_SPEED in rows:
                    airspeed = rows[ROW_SPEED][0] * KNOTS_TO_MS

                # Row 4 — Gload → body-frame accel  (SIM_XPlane.cpp)
                # accel_body.z = -data[5]*g  values[4]
                # accel_body.x =  data[6]*g  values[5]
                # accel_body.y =  data[7]*g  values[6]
                if ROW_GLOAD in rows:
                    d = rows[ROW_GLOAD]
                    accel_body = [
                         d[5] * GRAVITY_MSS,    # x
                         d[6] * GRAVITY_MSS,    # y
                        -d[4] * GRAVITY_MSS,    # z
                    ]
                    xp_sensor_updated = True

                # Row 16 — AngularVelocities
                # XP12 deg/s  data[1..3] → values[0..2]
                # XP11 rad/s  axes swapped: gyro.x=data[2], gyro.y=data[1]
                if ROW_ANG_VEL in rows:
                    d = rows[ROW_ANG_VEL]
                    if is_xp12:
                        gyro = [d[0] * DEG_TO_RAD,
                                d[1] * DEG_TO_RAD,
                                d[2] * DEG_TO_RAD]
                    else:
                        gyro = [d[1], d[0], d[2]]
                    xp_sensor_updated = True

                # Row 17 — PitchRollHeading
                # data[1]=pitch°  data[2]=roll°  data[3]=hdg°
                if ROW_PITCH_ROLL_HDG in rows:
                    d = rows[ROW_PITCH_ROLL_HDG]
                    pitch_r = d[0] * DEG_TO_RAD
                    roll_r  = d[1] * DEG_TO_RAD
                    yaw_r   = d[2] * DEG_TO_RAD
                    xp_sensor_updated = True

                # Row 20 — LatLonAlt
                # data[1]=lat°  data[2]=lon°  data[3]=alt_ft_MSL
                if ROW_LAT_LON_ALT in rows:
                    d = rows[ROW_LAT_LON_ALT]
                    lat   = d[0]
                    lon   = d[1]
                    alt_m = d[2] * FEET_TO_M
                    pos_valid = True

                # Row 21 — LocVelDistTraveled  (SIM_XPlane.cpp)
                # velocity_ef: y=data[4] E, z=−data[5] D, x=−data[6] N
                if ROW_LOC_VEL_DIST in rows:
                    d = rows[ROW_LOC_VEL_DIST]
                    vel_ned = [-d[5], d[3], -d[4]]   # [N, E, D]


        except BlockingIOError:
            pass

        if not pos_valid:
            # drain MAVLink to avoid stale messages building up
            mav.recv_match(blocking=False)
            if args.debug and now - last_debug >= 5.0:
                last_debug = now
                print('[wait] No X-Plane DATA@ — is X-Plane running and unpaused?')
            continue

        t_us = int(now * 1e6)

        # ── HIL_SENSOR ────────────────────────────────────────────────────────
        # Send whenever X-Plane delivers fresh sensor rows, subject to the
        # maximum rate cap (hil_min_ivl).  This follows X-Plane's physics
        # rate rather than an independent fixed timer.
        if xp_sensor_updated and (now - last_hil) >= hil_min_ivl:
            last_hil = now
            abs_p  = alt_to_pressure(alt_m)
            diff_p = airspeed_to_diff_pressure(airspeed)
            temp_c = 15.0 - 0.0065 * alt_m   # ISA temperature

            # Magnetometer: IGRF-14 dipole NED → rotate to body frame (Gauss).
            # Mirrors Aircraft::update_mag_field_bf() in SIM_Aircraft.cpp.
            mag_ned  = igrf_dipole_ned_gauss(lat, lon)
            mag_body = ned_to_body(mag_ned, roll_r, pitch_r, yaw_r)

            # HIL_SENSOR — gyro/accel/baro/mag injected into sitl->state via
            # handle_hil_sensor() in GCS_Common.cpp (fmuv3-hil build only).
            mav.mav.hil_sensor_send(
                t_us,
                accel_body[0], accel_body[1], accel_body[2],  # m/s²
                gyro[0],       gyro[1],       gyro[2],         # rad/s
                mag_body[0],   mag_body[1],   mag_body[2],     # Gauss
                abs_p,                                          # hPa
                diff_p,                                         # hPa
                alt_m,                                          # pressure_alt m
                temp_c,                                         # °C
                HIL_FIELDS,                                     # fields_updated
                0,                                              # sensor id
            )

        # ── GPS_INPUT ─────────────────────────────────────────────────────────
        # Works on standard firmware with GPS1_TYPE=14.
        if now - last_gps >= gps_ivl:
            last_gps = now
            hdg_cd = int((yaw_r % (2 * math.pi)) * 18000 / math.pi) or 36000  # 0 = unknown; 36000 = North

            gps_week, gps_tow_ms = gps_time()
            mav.mav.gps_input_send(
                t_us,
                0,                       # gps_id
                0,                       # ignore_flags — use all fields
                gps_tow_ms, gps_week,    # time_week_ms, time_week
                3,                       # fix_type: 3D
                int(lat * 1e7),          # lat  degE7
                int(lon * 1e7),          # lon  degE7
                alt_m,                   # alt  m MSL
                1.0, 1.0,                # hdop, vdop
                vel_ned[0], vel_ned[1], vel_ned[2],   # vn, ve, vd  m/s
                0.2,                     # speed_accuracy  m/s
                0.3,                     # horiz_accuracy  m
                0.5,                     # vert_accuracy   m
                10,                      # satellites_visible
                hdg_cd,                  # yaw  cdeg (0 = unknown)
            )

        # ── SERVO_OUTPUT_RAW → X-Plane DREFs ─────────────────────────────────
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() != 'SERVO_OUTPUT_RAW':
                continue
            servos = [
                msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                msg.servo4_raw, msg.servo5_raw,
                msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
            ]
            srv_count += 1
            if args.debug:
                labels = ['ail', 'elev', 'thr', 'rud', 'ch5', 'ch6', 'ch7', 'ch8']
                parts  = [f'{l}={v}' for l, v in zip(labels, servos) if v > 0]
                print(f'[SRV] {" ".join(parts)}')
           
            if not dref_sent:
                dref_sent = True
                print(f'[XP]  First DREF → {xp_addr}')
            for dref_name, ch, conv in SERVO_DREFS:
                val = conv(servos[ch])
                xp_dref(xp_sock, xp_addr, dref_name, val)
                if args.debug:
                    print(f'  → {dref_name} = {val:.3f}')

        # ── periodic 1 s status ────────────────────────────────────────────────
        if now - last_srv >= 1.0:
            last_srv  = now
            hdg = math.degrees(yaw_r) % 360.0
            print(f'[GPS] lat={lat:.6f}  lon={lon:.6f}  '
                  f'alt={alt_m:.1f} m  hdg={hdg:.1f}°  IAS={airspeed:.1f} m/s')
            if not args.debug:
                labels = ['ail', 'elev', 'thr', 'rud', 'ch5', 'ch6', 'ch7', 'ch8']
                parts  = [f'{l}={v}' for l, v in zip(labels, servos) if v > 0]
                print(f'[SRV] {" ".join(parts) or "(none)"}  '
                      f'({srv_count} msg/s)')
            srv_count = 0

        # ── set AUTO mode once position is valid ──────────────────────────────
        # if args.auto and not mode_set:
        #     mav.mav.set_mode_send(
        #         mav.target_system,
        #         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        #         PLANE_MODE_AUTO,
        #     )
        #     mode_set = True
        #     print('[MAV] Flight mode → AUTO')

        # ── debug ─────────────────────────────────────────────────────────────
        if args.debug and now - last_debug >= 5.0:
            last_debug = now
            hdg = math.degrees(yaw_r) % 360
            print(f'[xp]   lat={lat:.5f}  lon={lon:.5f}  '
                  f'alt={alt_m:.1f} m  t={xp_time:.1f} s')
            print(f'[att]  pitch={math.degrees(pitch_r):+.1f}°  '
                  f'roll={math.degrees(roll_r):+.1f}°  hdg={hdg:.1f}°')
            print(f'[vel]  N={vel_ned[0]:.1f}  E={vel_ned[1]:.1f}  '
                  f'D={vel_ned[2]:.1f} m/s  airspeed={airspeed:.1f} m/s')
            print(f'[gyro] {[round(g, 3) for g in gyro]} rad/s')
            print(f'[acc]  {[round(a, 2) for a in accel_body]} m/s²')
            p = alt_to_pressure(alt_m)
            print(f'[baro] {p:.2f} hPa  '
                  f'diff={airspeed_to_diff_pressure(airspeed):.4f} hPa')
            print()


if __name__ == '__main__':
    main()
