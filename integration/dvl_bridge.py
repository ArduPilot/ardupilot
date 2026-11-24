#!/usr/bin/env python3
"""Forward valid A50 bottom-track velocity to local ArduSub over MAVLink.

This process sends only VISION_SPEED_ESTIMATE. It cannot arm or command motors.
It requires fresh ArduSub attitude to convert A50 body FRD into local NED.
"""

import argparse
import json
import math
import os
from pathlib import Path
import socket
import time

from dvl_velocity import mavlink_covariance, parse_position, parse_velocity, to_ned


def measurement(line, received_s, attitude, config):
    """Return (NED velocity, uncertainty), or None when input is unusable."""
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
    args = parser.parse_args()
    if not args.connect_a50:
        parser.error("live sensor access requires --connect-a50")
    if args.mavlink != "udpin:127.0.0.1:14551":
        parser.error("this development bridge only permits the loopback MAVLink endpoint")
    config = json.loads(args.config.read_text())["dvl_a50"]
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil
    master = mavutil.mavlink_connection(args.mavlink, source_system=200,
                                       source_component=191, dialect="ardupilotmega")
    attitude = None
    heard_ardusub = False
    next_attitude_request = 0.0
    last_validity = None
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
                if args.observe_position:
                    position = parse_position(line, now)
                    if position is not None:
                        print(json.dumps({"type": "a50_position_observation",
                                          "frame": "a50_reset_local_unaligned",
                                          "xyz_m": position.local_m,
                                          "std_m": position.position_std_m,
                                          "report_time_s": position.report_time_s}), flush=True)
                result = measurement(line, now, attitude, config)
                if result is None:
                    continue
                velocity, covariance, validity = result
                if validity is not None and last_validity is not None and validity <= last_validity:
                    continue
                if validity is not None:
                    last_validity = validity
                master.mav.vision_speed_estimate_send(time.time_ns() // 1000, *velocity,
                                                      covariance=covariance)


if __name__ == "__main__":
    main()
