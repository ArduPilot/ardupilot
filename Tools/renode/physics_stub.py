#!/usr/bin/env python3

"""Deterministic stationary physics sidecar for exercising the wire protocol."""

# AP_FLAKE8_CLEAN

import argparse
import math
import socket
import sys

from physics_protocol import MessageType
from physics_protocol import ProtocolError
from physics_protocol import StepFrame
from physics_protocol import TruthFrame
from physics_protocol import pack_json
from physics_protocol import recv_message
from physics_protocol import send_message
from physics_protocol import unpack_json

MODELS = ("constant-motion", "stationary")


def _expect_message(connection, expected_type):
    message = recv_message(connection)
    if message is None:
        raise EOFError(f"connection closed while waiting for {expected_type.name}")
    message_type, payload = message
    if message_type != expected_type:
        raise ProtocolError(f"expected {expected_type.name}, received {message_type.name}")
    return payload


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProtocolError(f"{name} must be a finite number")
    return float(value)


def _configure(payload):
    request = unpack_json(payload)
    if request.get("model") not in MODELS:
        raise ProtocolError("the stub does not support the requested model")
    location = request.get("location")
    if not isinstance(location, dict):
        raise ProtocolError("location must be an object")
    configured = {
        "latitude_deg": _number(location.get("latitude_deg"), "location.latitude_deg"),
        "longitude_deg": _number(location.get("longitude_deg"), "location.longitude_deg"),
        "altitude_m": _number(location.get("altitude_m"), "location.altitude_m"),
        "heading_deg": _number(location.get("heading_deg", 0.0), "location.heading_deg"),
        "model": request["model"],
        "rate_hz": _number(request.get("rate_hz"), "rate_hz"),
    }
    if not -90.0 <= configured["latitude_deg"] <= 90.0:
        raise ProtocolError("location.latitude_deg must be from -90 to 90")
    if not -180.0 <= configured["longitude_deg"] <= 180.0:
        raise ProtocolError("location.longitude_deg must be from -180 to 180")
    if configured["rate_hz"] <= 0.0:
        raise ProtocolError("rate_hz must be positive")
    return configured


def _truth_for_step(configuration, step):
    yaw = math.radians(configuration["heading_deg"] % 360.0)
    half_yaw = 0.5 * yaw
    elapsed_s = step.timestamp_us * 1e-6
    if configuration["model"] == "constant-motion":
        velocity_ned_m_s = (10.0, 5.0, -1.0)
        position_ned_m = tuple(value * elapsed_s for value in velocity_ned_m_s)
        latitude_deg = max(-89.999, min(
            89.999, configuration["latitude_deg"] + position_ned_m[0] / 111319.5))
        longitude_scale = 111319.5 * max(0.01, abs(math.cos(math.radians(latitude_deg))))
        longitude_deg = configuration["longitude_deg"] + position_ned_m[1] / longitude_scale
        longitude_deg = (longitude_deg + 180.0) % 360.0 - 180.0
        altitude_m = configuration["altitude_m"] - position_ned_m[2]
        airspeed_m_s = 12.5
    else:
        velocity_ned_m_s = (0.0, 0.0, 0.0)
        position_ned_m = (0.0, 0.0, 0.0)
        latitude_deg = configuration["latitude_deg"]
        longitude_deg = configuration["longitude_deg"]
        altitude_m = configuration["altitude_m"]
        airspeed_m_s = 0.0
    atmosphere_altitude_m = min(max(altitude_m, -1000.0), 44300.0)
    atmosphere_base = max(0.01, 1.0 - 2.25577e-5 * atmosphere_altitude_m)
    pressure_pa = 101325.0 * atmosphere_base ** 5.25588
    temperature_k = max(1.0, 288.15 - 0.0065 * atmosphere_altitude_m)
    magnetic_north = 400.0
    magnetic_east = 40.0
    magnetic_down = 120.0
    magnetic_body = (
        math.cos(yaw) * magnetic_north + math.sin(yaw) * magnetic_east,
        -math.sin(yaw) * magnetic_north + math.cos(yaw) * magnetic_east,
        magnetic_down,
    )
    return TruthFrame(
        timestamp_us=step.timestamp_us,
        sequence=step.sequence,
        flags=0,
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        position_ned_m=position_ned_m,
        quaternion=(math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)),
        gyro_rad_s=(0.0, 0.0, 0.0),
        specific_force_m_s2=(0.0, 0.0, -9.80665),
        velocity_ned_m_s=velocity_ned_m_s,
        airspeed_m_s=airspeed_m_s,
        magnetic_field_body_mgauss=magnetic_body,
        pressure_pa=pressure_pa,
        temperature_k=temperature_k,
        battery_voltage_v=12.0,
        battery_current_a=0.0,
        rpm=(0.0,) * 32,
        rangefinder_m=(0.0,) * 10,
    )


def serve_connection(connection, trace_actuators=False, truth_provider=None):
    """Serve one already-connected Renode client until it disconnects."""
    if truth_provider is None:
        truth_provider = _truth_for_step
    try:
        hello = unpack_json(_expect_message(connection, MessageType.HELLO))
        if hello.get("role") != "renode":
            raise ProtocolError("HELLO role must be renode")
        send_message(connection, MessageType.HELLO_REPLY, pack_json({"models": list(MODELS), "role": "physics"}))

        configuration = _configure(_expect_message(connection, MessageType.CONFIGURE))
        send_message(connection, MessageType.CONFIGURE_REPLY, pack_json({
            "model": configuration["model"],
            "status": "configured",
        }))

        previous_sequence = None
        previous_timestamp_us = None
        previous_actuators = None
        while True:
            message = recv_message(connection)
            if message is None:
                return
            message_type, payload = message
            if message_type != MessageType.STEP:
                raise ProtocolError(f"expected STEP, received {message_type.name}")
            step = StepFrame.unpack(payload)
            if previous_sequence is not None and step.sequence <= previous_sequence:
                raise ProtocolError("STEP sequence must increase")
            if previous_timestamp_us is not None and step.timestamp_us <= previous_timestamp_us:
                raise ProtocolError("STEP timestamp must increase")
            previous_sequence = step.sequence
            previous_timestamp_us = step.timestamp_us
            if trace_actuators:
                active = tuple(
                    f"{index + 1}:{actuator.protocol.name}:"
                    f"{actuator.value}:0x{actuator.flags:02x}"
                    for index, actuator in enumerate(step.actuators)
                    if actuator.protocol.value != 0
                )
                if active != previous_actuators:
                    print(f"ACTUATORS {step.sequence} {' '.join(active)}", flush=True)
                    previous_actuators = active
            truth = truth_provider(configuration, step)
            if not isinstance(truth, TruthFrame):
                raise ProtocolError("truth provider did not return a TruthFrame")
            send_message(connection, MessageType.STATE, truth.pack())
    except (EOFError, OSError):
        return
    except ProtocolError as ex:
        try:
            send_message(connection, MessageType.ERROR, pack_json({"error": str(ex)}))
        except OSError:
            pass


def _parse_listen(value):
    host, separator, port_text = value.rpartition(":")
    if not separator or not host:
        raise argparse.ArgumentTypeError("listen address must be HOST:PORT")
    try:
        port = int(port_text)
    except ValueError as ex:
        raise argparse.ArgumentTypeError("listen port must be an integer") from ex
    if not 0 <= port <= 65535:
        raise argparse.ArgumentTypeError("listen port must be from 0 to 65535")
    return host, port


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen", type=_parse_listen, default=("127.0.0.1", 0), metavar="HOST:PORT")
    parser.add_argument("--trace-actuators", action="store_true",
                        help="print each non-unused actuator received in STEP")
    args = parser.parse_args()
    with socket.create_server(args.listen, family=socket.AF_INET, backlog=1) as server:
        print(f"PHYSICS_PORT {server.getsockname()[1]}", flush=True)
        connection, _ = server.accept()
        with connection:
            serve_connection(connection, args.trace_actuators)
    return 0


if __name__ == "__main__":
    sys.exit(main())
