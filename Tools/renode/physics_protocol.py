#!/usr/bin/env python3

"""Wire protocol shared by the Renode launcher and physics sidecars."""

# AP_FLAKE8_CLEAN

import json
import math
import struct

from dataclasses import dataclass
from enum import IntEnum

MAGIC = b"APRP"
PROTOCOL_VERSION = 1
MAX_PAYLOAD = 1024 * 1024
ACTUATOR_COUNT = 32
RPM_COUNT = 32
RANGEFINDER_COUNT = 10

_ENVELOPE = struct.Struct("<4sHHI")
_STEP_HEADER = struct.Struct("<QIHH")
_ACTUATOR = struct.Struct("<HBB")
_TRUTH = struct.Struct("<QII6d63f")


class ProtocolError(ValueError):
    """The peer supplied a malformed or unexpected protocol value."""


class MessageType(IntEnum):
    HELLO = 1
    HELLO_REPLY = 2
    CONFIGURE = 3
    CONFIGURE_REPLY = 4
    STEP = 5
    STATE = 6
    ERROR = 7


class ActuatorProtocol(IntEnum):
    UNUSED = 0
    PWM = 1
    DSHOT = 2


ACTUATOR_FLAG_VALID = 1 << 0


def _validate_uint(name, value, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ProtocolError(f"{name} must be an integer from 0 to {maximum}")


def _validate_float(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProtocolError(f"{name} must be a finite number")


def _validate_values(name, values, count):
    if len(values) != count:
        raise ProtocolError(f"{name} must contain {count} values")
    for value in values:
        _validate_float(name, value)


@dataclass(frozen=True)
class Actuator:
    value: int = 0
    protocol: ActuatorProtocol = ActuatorProtocol.UNUSED
    flags: int = 0

    def validate(self):
        _validate_uint("actuator value", self.value, 0xFFFF)
        try:
            ActuatorProtocol(self.protocol)
        except ValueError as ex:
            raise ProtocolError(f"unknown actuator protocol {self.protocol}") from ex
        _validate_uint("actuator flags", self.flags, 0xFF)


@dataclass(frozen=True)
class StepFrame:
    timestamp_us: int
    sequence: int
    actuators: tuple

    def pack(self):
        _validate_uint("timestamp_us", self.timestamp_us, 0xFFFFFFFFFFFFFFFF)
        _validate_uint("sequence", self.sequence, 0xFFFFFFFF)
        if len(self.actuators) != ACTUATOR_COUNT:
            raise ProtocolError(f"actuators must contain {ACTUATOR_COUNT} values")
        payload = bytearray(_STEP_HEADER.pack(self.timestamp_us, self.sequence, ACTUATOR_COUNT, 0))
        for actuator in self.actuators:
            if not isinstance(actuator, Actuator):
                raise ProtocolError("each actuator must be an Actuator")
            actuator.validate()
            payload.extend(_ACTUATOR.pack(actuator.value, int(actuator.protocol), actuator.flags))
        return bytes(payload)

    @classmethod
    def unpack(cls, payload):
        expected_size = _STEP_HEADER.size + ACTUATOR_COUNT * _ACTUATOR.size
        if len(payload) != expected_size:
            raise ProtocolError(f"STEP payload is {len(payload)} bytes, expected {expected_size}")
        timestamp_us, sequence, actuator_count, reserved = _STEP_HEADER.unpack_from(payload)
        if actuator_count != ACTUATOR_COUNT:
            raise ProtocolError(f"STEP has {actuator_count} actuators, expected {ACTUATOR_COUNT}")
        if reserved != 0:
            raise ProtocolError("STEP reserved field is non-zero")
        actuators = []
        offset = _STEP_HEADER.size
        for _ in range(ACTUATOR_COUNT):
            value, protocol_value, flags = _ACTUATOR.unpack_from(payload, offset)
            offset += _ACTUATOR.size
            try:
                protocol = ActuatorProtocol(protocol_value)
            except ValueError as ex:
                raise ProtocolError(f"unknown actuator protocol {protocol_value}") from ex
            actuator = Actuator(value, protocol, flags)
            actuator.validate()
            actuators.append(actuator)
        return cls(timestamp_us, sequence, tuple(actuators))


@dataclass(frozen=True)
class TruthFrame:
    timestamp_us: int
    sequence: int
    flags: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    position_ned_m: tuple
    quaternion: tuple
    gyro_rad_s: tuple
    specific_force_m_s2: tuple
    velocity_ned_m_s: tuple
    airspeed_m_s: float
    magnetic_field_body_mgauss: tuple
    pressure_pa: float
    temperature_k: float
    battery_voltage_v: float
    battery_current_a: float
    rpm: tuple
    rangefinder_m: tuple

    def _packed_values(self):
        _validate_uint("timestamp_us", self.timestamp_us, 0xFFFFFFFFFFFFFFFF)
        _validate_uint("sequence", self.sequence, 0xFFFFFFFF)
        _validate_uint("truth flags", self.flags, 0xFFFFFFFF)
        for name, value in (
            ("latitude_deg", self.latitude_deg),
            ("longitude_deg", self.longitude_deg),
            ("altitude_m", self.altitude_m),
            ("airspeed_m_s", self.airspeed_m_s),
            ("pressure_pa", self.pressure_pa),
            ("temperature_k", self.temperature_k),
            ("battery_voltage_v", self.battery_voltage_v),
            ("battery_current_a", self.battery_current_a),
        ):
            _validate_float(name, value)
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ProtocolError("latitude_deg must be from -90 to 90")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ProtocolError("longitude_deg must be from -180 to 180")
        fields = (
            ("position_ned_m", self.position_ned_m, 3),
            ("quaternion", self.quaternion, 4),
            ("gyro_rad_s", self.gyro_rad_s, 3),
            ("specific_force_m_s2", self.specific_force_m_s2, 3),
            ("velocity_ned_m_s", self.velocity_ned_m_s, 3),
            ("magnetic_field_body_mgauss", self.magnetic_field_body_mgauss, 3),
            ("rpm", self.rpm, RPM_COUNT),
            ("rangefinder_m", self.rangefinder_m, RANGEFINDER_COUNT),
        )
        for name, values, count in fields:
            _validate_values(name, values, count)
        doubles = (
            float(self.latitude_deg),
            float(self.longitude_deg),
            float(self.altitude_m),
            *(float(value) for value in self.position_ned_m),
        )
        floats = (
            *(float(value) for value in self.quaternion),
            *(float(value) for value in self.gyro_rad_s),
            *(float(value) for value in self.specific_force_m_s2),
            *(float(value) for value in self.velocity_ned_m_s),
            float(self.airspeed_m_s),
            *(float(value) for value in self.magnetic_field_body_mgauss),
            float(self.pressure_pa),
            float(self.temperature_k),
            float(self.battery_voltage_v),
            float(self.battery_current_a),
            *(float(value) for value in self.rpm),
            *(float(value) for value in self.rangefinder_m),
        )
        return doubles, floats

    def pack(self):
        doubles, floats = self._packed_values()
        return _TRUTH.pack(self.timestamp_us, self.sequence, self.flags, *doubles, *floats)

    @classmethod
    def unpack(cls, payload):
        if len(payload) != _TRUTH.size:
            raise ProtocolError(f"STATE payload is {len(payload)} bytes, expected {_TRUTH.size}")
        values = _TRUTH.unpack(payload)
        timestamp_us, sequence, flags = values[:3]
        doubles = values[3:9]
        floats = values[9:]
        frame = cls(
            timestamp_us=timestamp_us,
            sequence=sequence,
            flags=flags,
            latitude_deg=doubles[0],
            longitude_deg=doubles[1],
            altitude_m=doubles[2],
            position_ned_m=tuple(doubles[3:6]),
            quaternion=tuple(floats[0:4]),
            gyro_rad_s=tuple(floats[4:7]),
            specific_force_m_s2=tuple(floats[7:10]),
            velocity_ned_m_s=tuple(floats[10:13]),
            airspeed_m_s=floats[13],
            magnetic_field_body_mgauss=tuple(floats[14:17]),
            pressure_pa=floats[17],
            temperature_k=floats[18],
            battery_voltage_v=floats[19],
            battery_current_a=floats[20],
            rpm=tuple(floats[21:53]),
            rangefinder_m=tuple(floats[53:63]),
        )
        frame._packed_values()
        return frame


def pack_message(message_type, payload=b""):
    try:
        message_type = MessageType(message_type)
    except ValueError as ex:
        raise ProtocolError(f"unknown message type {message_type}") from ex
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise ProtocolError("payload must be bytes-like")
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload exceeds the {MAX_PAYLOAD}-byte limit")
    return _ENVELOPE.pack(MAGIC, PROTOCOL_VERSION, int(message_type), len(payload)) + payload


def read_exact(connection, length, allow_initial_eof=False):
    data = bytearray()
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        if not chunk:
            if allow_initial_eof and not data:
                return None
            raise EOFError(f"connection closed after {len(data)} of {length} bytes")
        data.extend(chunk)
    return bytes(data)


def recv_message(connection):
    header = read_exact(connection, _ENVELOPE.size, allow_initial_eof=True)
    if header is None:
        return None
    magic, version, message_type_value, payload_length = _ENVELOPE.unpack(header)
    if magic != MAGIC:
        raise ProtocolError(f"invalid protocol magic {magic!r}")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    try:
        message_type = MessageType(message_type_value)
    except ValueError as ex:
        raise ProtocolError(f"unknown message type {message_type_value}") from ex
    if payload_length > MAX_PAYLOAD:
        raise ProtocolError(f"payload exceeds the {MAX_PAYLOAD}-byte limit")
    return message_type, read_exact(connection, payload_length)


def send_message(connection, message_type, payload=b""):
    connection.sendall(pack_message(message_type, payload))


def pack_json(value):
    if not isinstance(value, dict):
        raise ProtocolError("JSON control payload must be an object")
    try:
        return json.dumps(value, allow_nan=False, separators=(",", ":"), sort_keys=True).encode("utf-8")
    except (TypeError, ValueError) as ex:
        raise ProtocolError(f"invalid JSON control payload: {ex}") from ex


def unpack_json(payload):
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as ex:
        raise ProtocolError(f"invalid JSON control payload: {ex}") from ex
    if not isinstance(value, dict):
        raise ProtocolError("JSON control payload must be an object")
    return value
