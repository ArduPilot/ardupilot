#!/usr/bin/env python3
"""Live monitor for what data is actually arriving from the Webots rover,
via ArduPilot's own MAVLink telemetry.

Connects to a running SITL instance and prints a refreshed snapshot every
second: mode/armed state, GPS fix, position, attitude, raw IMU, groundspeed,
and per-radar proximity health + nearest detected obstacle. This is exactly
the FDM data path documented in docs/HLK-LD2451/README.md - Webots sends
gyro/accel/attitude/velocity/position, ArduPilot decodes it through GPS/EKF,
and the 4 corner radars show up as PRX1-4 proximity data.

Usage (run against a live sim_vehicle.py/ardurover instance):
    python3 docs/HLK-LD2451/webots_monitor.py
    python3 docs/HLK-LD2451/webots_monitor.py --host 127.0.0.1 --port 5762

Ctrl-C to stop.
"""
import argparse
import sys
import time

from pymavlink import mavutil

PROXIMITY_BIT = 1 << 20  # MAV_SYS_STATUS_SENSOR_PROXIMITY / OBSTACLE_AVOIDANCE


def connect(host, port):
    m = mavutil.mavlink_connection(f"tcp:{host}:{port}", source_system=250, source_component=1)
    print(f"connecting to {host}:{port} ...")
    m.wait_heartbeat(timeout=15)
    print(f"connected: sys={m.target_system} comp={m.target_component}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    # REQUEST_DATA_STREAM doesn't reliably cover OBSTACLE_DISTANCE - ask for it explicitly.
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_OBSTACLE_DISTANCE, int(1e6 / 4), 0, 0, 0, 0, 0)
    return m


def fmt_hit(msg):
    """Nearest OBSTACLE_DISTANCE sector, or None if nothing detected."""
    hits = [(i, d) for i, d in enumerate(msg.distances) if d < msg.max_distance]
    if not hits:
        return None
    i, d = min(hits, key=lambda h: h[1])
    return f"sector {i} @ {d / 100.0:.1f}m"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5762, help="SITL MAVLink TCP port (default: 5762)")
    args = parser.parse_args()

    m = connect(args.host, args.port)

    state = {}
    last_print = 0.0

    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=1)
            if msg is not None:
                state[msg.get_type()] = msg

            now = time.time()
            if now - last_print < 1.0:
                continue
            last_print = now

            hb = state.get("HEARTBEAT")
            mode_name = m.flightmode
            gps = state.get("GPS_RAW_INT")
            pos = state.get("GLOBAL_POSITION_INT")
            att = state.get("ATTITUDE")
            imu = state.get("RAW_IMU")
            vfr = state.get("VFR_HUD")
            sys_status = state.get("SYS_STATUS")
            obs = state.get("OBSTACLE_DISTANCE")

            print("=" * 70)
            if hb:
                armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print(f"mode={mode_name:10s} armed={armed}")
            if gps:
                print(f"GPS      fix_type={gps.fix_type} sats={gps.satellites_visible} "
                      f"lat={gps.lat / 1e7:.7f} lon={gps.lon / 1e7:.7f}")
            if pos:
                print(f"Position lat={pos.lat / 1e7:.7f} lon={pos.lon / 1e7:.7f} "
                      f"alt={pos.relative_alt / 1000.0:.2f}m hdg={pos.hdg / 100.0:.1f}deg")
            if att:
                import math
                print(f"Attitude roll={math.degrees(att.roll):+.1f} pitch={math.degrees(att.pitch):+.1f} "
                      f"yaw={math.degrees(att.yaw):+.1f} deg")
            if imu:
                print(f"IMU      accel=({imu.xacc},{imu.yacc},{imu.zacc}) mg  "
                      f"gyro=({imu.xgyro},{imu.ygyro},{imu.zgyro}) mrad/s")
            if vfr:
                print(f"Speed    groundspeed={vfr.groundspeed:.2f} m/s  throttle={vfr.throttle}%")
            if sys_status:
                present = bool(sys_status.onboard_control_sensors_present & PROXIMITY_BIT)
                enabled = bool(sys_status.onboard_control_sensors_enabled & PROXIMITY_BIT)
                healthy = bool(sys_status.onboard_control_sensors_health & PROXIMITY_BIT)
                print(f"Radar    present={present} enabled={enabled} healthy={healthy}", end="")
                if obs:
                    hit = fmt_hit(obs)
                    print(f"  nearest={hit or 'clear'}")
                else:
                    print("  (no OBSTACLE_DISTANCE received yet)")
    except KeyboardInterrupt:
        print("\nstopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()
