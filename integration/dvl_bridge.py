#!/usr/bin/env python3
"""Forward valid A50 bottom-track velocity to local ArduSub over MAVLink.

By default this bridge emits VISION_POSITION_DELTA (body FRD displacement) so
ArduSub EKF3 enters AID_RELATIVE (optical-flow/body-odom equivalent) cleanly
without requiring global GPS coordinates or causing AID_NONE ground-drift
faults. Use --mode ned_vel for VISION_SPEED_ESTIMATE when global ExternalNav
position/origin is active. It cannot arm or command motors.
"""

import argparse
import json
import math
import os
from pathlib import Path
import socket
import time

from dvl_velocity import (
    mavlink_covariance,
    parse_position,
    parse_velocity,
    to_body_delta,
    to_ned,
)


def measurement(line, received_s, attitude, config):
    """Return (NED velocity, uncertainty, validity_us), or None when input is unusable."""
    if attitude is None or received_s - attitude[0] > 0.5 or received_s < attitude[0]:
        return None
    sample = parse_velocity(line, received_s, config["fom_max_mps"])
    if sample is None:
        return None
    try:
        velocity = to_ned(sample, attitude[1], config["mount_quaternion_wxyz"],
                          config["offset_m_frd"], attitude[2], received_s,
                          config["velocity_timeout_s"])
    except ValueError:
        return None
    return velocity, mavlink_covariance(sample.velocity_error_mps), sample.validity_unix_us


def delta_measurement(line, received_s, attitude, config,
                      prev_validity_us=None, prev_received_s=None):
    """Return (pos_delta, ang_delta, time_delta_usec, confidence, validity_us), or None."""
    if attitude is None or received_s - attitude[0] > 0.5 or received_s < attitude[0]:
        return None
    sample = parse_velocity(line, received_s, config["fom_max_mps"])
    if sample is None:
        return None
    if sample.validity_unix_us is not None and prev_validity_us is not None:
        if sample.validity_unix_us <= prev_validity_us:
            return None
        dt_s = (sample.validity_unix_us - prev_validity_us) / 1_000_000.0
    elif sample.interval_s is not None:
        dt_s = sample.interval_s
    elif prev_received_s is not None:
        dt_s = received_s - prev_received_s
    else:
        return None
    try:
        pos_delta, ang_delta, time_delta_usec, confidence = to_body_delta(
            sample, dt_s, config["mount_quaternion_wxyz"],
            config["offset_m_frd"], attitude[2], config["fom_max_mps"],
            received_s, config["velocity_timeout_s"],
        )
    except ValueError:
        return None
    return pos_delta, ang_delta, time_delta_usec, confidence, sample.validity_unix_us


def extract_dvl_status(line, received_s):
    """Parse lightweight DVL health telemetry for dashboard display (never navigation)."""
    try:
        report = json.loads(line)
        if not isinstance(report, dict) or report.get("type", "velocity") != "velocity":
            return None
        locked = report.get("velocity_valid") is True and report.get("status") == 0
        altitude = report.get("altitude")
        fom = report.get("fom")
        return {
            "type": "atal_dvl_status",
            "dvl_locked": bool(locked),
            "dvl_altitude": float(altitude) if isinstance(altitude, (int, float)) and math.isfinite(altitude) else None,
            "dvl_fom": float(fom) if isinstance(fom, (int, float)) and math.isfinite(fom) else None,
            "ts": received_s,
        }
    except (ValueError, TypeError, json.JSONDecodeError):
        return None


def update_attitude(master, attitude):
    """Drain telemetry; retain only a fresh, finite attitude and body-rate sample."""
    heartbeat = False
    while (msg := master.recv_match(blocking=False)) is not None:
        if (msg.get_type() == "HEARTBEAT" and
                msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1):
            heartbeat = True
        if (msg.get_type() != "ATTITUDE_QUATERNION" or
                msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1):
            continue
        quat = (msg.q1, msg.q2, msg.q3, msg.q4)
        rates = (msg.rollspeed, msg.pitchspeed, msg.yawspeed)
        if all(math.isfinite(v) for v in quat + rates):
            attitude = (time.monotonic(), quat, rates)
    return attitude, heartbeat


def request_attitude(master, mavutil):
    """Ask ArduSub for 10 Hz quaternion/rates; the default stream omits it."""
    master.mav.command_long_send(
        1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100_000,
        0, 0, 0, 0, 0,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("vehicle_defaults.json"))
    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14551",
                        help="loopback MAVLink endpoint; match ArduSub --serial0 udp:127.0.0.1:14551")
    parser.add_argument("--connect-a50", action="store_true",
                        help="explicitly enable the live A50 TCP sensor connection")
    parser.add_argument("--observe-position", action="store_true",
                        help="print raw A50 dead-reckoning reports; never forward position to ArduSub")
    parser.add_argument("--mode", choices=("body_delta", "ned_vel", "both"), default="body_delta",
                        help="MAVLink velocity/odometry output mode (default: body_delta for AID_RELATIVE)")
    parser.add_argument("--status-udp", default="",
                        help="optional host:port UDP target for DVL lock/altitude/FOM dashboard status")
    args = parser.parse_args()
    if not args.connect_a50:
        parser.error("live sensor access requires --connect-a50")
    if args.mavlink != "udpin:127.0.0.1:14551":
        parser.error("this development bridge only permits the loopback MAVLink endpoint")
    status_sock = None
    status_target = None
    if args.status_udp:
        host, port_str = args.status_udp.rsplit(":", 1)
        status_target = (host, int(port_str))
        status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    config = json.loads(args.config.read_text())["dvl_a50"]
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil
    master = mavutil.mavlink_connection(args.mavlink, source_system=200,
                                       source_component=191, dialect="ardupilotmega")
    attitude = None
    heard_ardusub = False
    next_attitude_request = 0.0
    last_validity = None
    last_received_s = None
    with socket.create_connection((config["ip"], config["tcp_port"]), timeout=3) as dvl:
        dvl.settimeout(0.1)
        buffer = b""
        while True:
            attitude, heartbeat = update_attitude(master, attitude)
            heard_ardusub |= heartbeat
            now = time.monotonic()
            if (heard_ardusub and now >= next_attitude_request and
                    (attitude is None or now - attitude[0] > 0.5)):
                request_attitude(master, mavutil)
                next_attitude_request = now + 2.0
            try:
                chunk = dvl.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("A50 TCP connection closed")
            buffer += chunk
            if len(buffer) > 65536:
                raise ValueError("A50 line exceeds 64 KiB")
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                now = time.monotonic()
                if status_sock is not None:
                    dvl_status = extract_dvl_status(line, now)
                    if dvl_status is not None:
                        status_sock.sendto(json.dumps(dvl_status).encode(), status_target)
                if args.observe_position:
                    position = parse_position(line, now)
                    if position is not None:
                        print(json.dumps({"type": "a50_position_observation",
                                          "frame": "a50_reset_local_unaligned",
                                          "xyz_m": position.local_m,
                                          "std_m": position.position_std_m,
                                          "report_time_s": position.report_time_s}), flush=True)
                sent = False
                if args.mode in ("body_delta", "both"):
                    delta = delta_measurement(line, now, attitude, config,
                                              prev_validity_us=last_validity,
                                              prev_received_s=last_received_s)
                    if delta is not None:
                        pos_d, ang_d, dt_us, conf, validity = delta
                        master.mav.vision_position_delta_send(
                            time.time_ns() // 1000, dt_us, ang_d, pos_d, conf
                        )
                        if validity is not None:
                            last_validity = validity
                        last_received_s = now
                        sent = True
                if args.mode in ("ned_vel", "both"):
                    result = measurement(line, now, attitude, config)
                    if result is not None:
                        velocity, covariance, validity = result
                        if args.mode == "ned_vel":
                            if validity is not None and last_validity is not None and validity <= last_validity:
                                continue
                            if validity is not None:
                                last_validity = validity
                        master.mav.vision_speed_estimate_send(time.time_ns() // 1000, *velocity,
                                                              covariance=covariance)
                        sent = True
                if not sent:
                    parsed = parse_velocity(line, now, config["fom_max_mps"])
                    if parsed is not None:
                        if parsed.validity_unix_us is not None:
                            if last_validity is None or parsed.validity_unix_us > last_validity:
                                last_validity = parsed.validity_unix_us
                        last_received_s = now


if __name__ == "__main__":
    main()
