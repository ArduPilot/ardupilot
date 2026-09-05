#!/usr/bin/env python3

"""Tests for the Renode physics sidecar protocol."""

# AP_FLAKE8_CLEAN

import socket
import struct
import sys
import threading

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import physics_protocol as protocol  # noqa: E402
import physics_stub  # noqa: E402


def empty_actuators():
    return (protocol.Actuator(),) * protocol.ACTUATOR_COUNT


def sample_truth():
    return protocol.TruthFrame(
        timestamp_us=123456,
        sequence=17,
        flags=3,
        latitude_deg=-35.363261,
        longitude_deg=149.165230,
        altitude_m=584.0,
        position_ned_m=(1.0, 2.0, 3.0),
        quaternion=(1.0, 0.0, 0.0, 0.0),
        gyro_rad_s=(0.1, 0.2, 0.3),
        specific_force_m_s2=(0.0, 0.0, -9.80665),
        velocity_ned_m_s=(4.0, 5.0, 6.0),
        airspeed_m_s=7.0,
        magnetic_field_body_mgauss=(400.0, 40.0, 120.0),
        pressure_pa=95000.0,
        temperature_k=284.0,
        battery_voltage_v=12.2,
        battery_current_a=3.4,
        rpm=tuple(float(value) for value in range(protocol.RPM_COUNT)),
        rangefinder_m=tuple(float(value) for value in range(protocol.RANGEFINDER_COUNT)),
    )


def test_step_frame_round_trip():
    actuators = list(empty_actuators())
    actuators[2] = protocol.Actuator(1500, protocol.ActuatorProtocol.PWM, protocol.ACTUATOR_FLAG_VALID)
    actuators[9] = protocol.Actuator(777, protocol.ActuatorProtocol.DSHOT, protocol.ACTUATOR_FLAG_VALID)
    frame = protocol.StepFrame(123456, 42, tuple(actuators))
    assert protocol.StepFrame.unpack(frame.pack()) == frame


def test_step_frame_rejects_invalid_values():
    with pytest.raises(protocol.ProtocolError, match="32"):
        protocol.StepFrame(1, 1, ()).pack()
    invalid = (protocol.Actuator(70000),) + empty_actuators()[1:]
    with pytest.raises(protocol.ProtocolError, match="65535"):
        protocol.StepFrame(1, 1, invalid).pack()
    payload = bytearray(protocol.StepFrame(1, 1, empty_actuators()).pack())
    payload[14:16] = struct.pack("<H", 1)
    with pytest.raises(protocol.ProtocolError, match="reserved"):
        protocol.StepFrame.unpack(payload)
    payload = bytearray(protocol.StepFrame(1, 1, empty_actuators()).pack())
    payload[12:14] = struct.pack("<H", protocol.ACTUATOR_COUNT - 1)
    with pytest.raises(protocol.ProtocolError, match="31 actuators"):
        protocol.StepFrame.unpack(payload)


def test_truth_frame_round_trip():
    original = sample_truth()
    decoded = protocol.TruthFrame.unpack(original.pack())
    assert decoded.timestamp_us == original.timestamp_us
    assert decoded.sequence == original.sequence
    assert decoded.flags == original.flags
    assert decoded.latitude_deg == original.latitude_deg
    assert decoded.longitude_deg == original.longitude_deg
    assert decoded.altitude_m == original.altitude_m
    assert decoded.position_ned_m == original.position_ned_m
    assert decoded.quaternion == pytest.approx(original.quaternion)
    assert decoded.rpm == pytest.approx(original.rpm)
    assert decoded.rangefinder_m == pytest.approx(original.rangefinder_m)


def test_truth_frame_validates_shape_and_location():
    truth = sample_truth()
    with pytest.raises(protocol.ProtocolError, match="latitude"):
        protocol.TruthFrame(**{**truth.__dict__, "latitude_deg": 91.0}).pack()
    with pytest.raises(protocol.ProtocolError, match="quaternion"):
        protocol.TruthFrame(**{**truth.__dict__, "quaternion": (1.0,)}).pack()


class FragmentedReader:
    def __init__(self, connection):
        self.connection = connection

    def recv(self, length):
        return self.connection.recv(min(length, 1))


def test_fragmented_envelope_read():
    sender, receiver = socket.socketpair()
    try:
        sender.sendall(protocol.pack_message(protocol.MessageType.HELLO, b"hello"))
        assert protocol.recv_message(FragmentedReader(receiver)) == (protocol.MessageType.HELLO, b"hello")
    finally:
        sender.close()
        receiver.close()


@pytest.mark.parametrize(
    "header,match",
    (
        (struct.pack("<4sHHI", b"BAD!", 1, 1, 0), "magic"),
        (struct.pack("<4sHHI", protocol.MAGIC, 99, 1, 0), "version"),
        (struct.pack("<4sHHI", protocol.MAGIC, 1, 99, 0), "message type"),
        (struct.pack("<4sHHI", protocol.MAGIC, 1, 1, protocol.MAX_PAYLOAD + 1), "limit"),
    ),
)
def test_invalid_envelope(header, match):
    sender, receiver = socket.socketpair()
    try:
        sender.sendall(header)
        with pytest.raises(protocol.ProtocolError, match=match):
            protocol.recv_message(receiver)
    finally:
        sender.close()
        receiver.close()


def test_truncated_envelope_and_clean_eof():
    sender, receiver = socket.socketpair()
    sender.sendall(b"AP")
    sender.close()
    with pytest.raises(EOFError, match="2 of"):
        protocol.recv_message(receiver)
    receiver.close()

    sender, receiver = socket.socketpair()
    sender.close()
    assert protocol.recv_message(receiver) is None
    receiver.close()


def test_json_payload_must_be_an_object():
    with pytest.raises(protocol.ProtocolError, match="object"):
        protocol.pack_json([])
    with pytest.raises(protocol.ProtocolError, match="object"):
        protocol.unpack_json(b"[]")
    with pytest.raises(protocol.ProtocolError, match="invalid JSON"):
        protocol.unpack_json(b"{")


def test_pack_message_rejects_oversized_payload():
    with pytest.raises(protocol.ProtocolError, match="limit"):
        protocol.pack_message(protocol.MessageType.HELLO, bytes(protocol.MAX_PAYLOAD + 1))


def start_stub():
    client, server = socket.socketpair()
    thread = threading.Thread(target=physics_stub.serve_connection, args=(server,))
    thread.start()
    return client, server, thread


def configure_stub(client, model="stationary"):
    protocol.send_message(client, protocol.MessageType.HELLO, protocol.pack_json({"role": "renode"}))
    message_type, payload = protocol.recv_message(client)
    assert message_type == protocol.MessageType.HELLO_REPLY
    assert protocol.unpack_json(payload) == {
        "models": ["constant-motion", "stationary"],
        "role": "physics",
    }
    configuration = {
        "location": {
            "latitude_deg": -35.363261,
            "longitude_deg": 149.165230,
            "altitude_m": 584.0,
            "heading_deg": 90.0,
        },
        "model": model,
        "rate_hz": 400.0,
    }
    protocol.send_message(client, protocol.MessageType.CONFIGURE, protocol.pack_json(configuration))
    message_type, payload = protocol.recv_message(client)
    assert message_type == protocol.MessageType.CONFIGURE_REPLY
    assert protocol.unpack_json(payload)["status"] == "configured"


def stop_stub(client, server, thread):
    client.close()
    thread.join(timeout=2)
    server.close()
    assert not thread.is_alive()


def test_stationary_stub_lockstep():
    client, server, thread = start_stub()
    try:
        configure_stub(client)
        step = protocol.StepFrame(2500, 11, empty_actuators())
        protocol.send_message(client, protocol.MessageType.STEP, step.pack())
        message_type, payload = protocol.recv_message(client)
        assert message_type == protocol.MessageType.STATE
        truth = protocol.TruthFrame.unpack(payload)
        assert truth.timestamp_us == step.timestamp_us
        assert truth.sequence == step.sequence
        assert truth.latitude_deg == -35.363261
        assert truth.longitude_deg == 149.165230
        assert truth.altitude_m == 584.0
        assert truth.quaternion == pytest.approx((2 ** -0.5, 0.0, 0.0, 2 ** -0.5))
        assert truth.velocity_ned_m_s == (0.0, 0.0, 0.0)
    finally:
        stop_stub(client, server, thread)


def test_constant_motion_stub_has_nonzero_velocity_and_airspeed():
    client, server, thread = start_stub()
    try:
        configure_stub(client, model="constant-motion")
        step = protocol.StepFrame(2_000_000, 11, empty_actuators())
        protocol.send_message(client, protocol.MessageType.STEP, step.pack())
        message_type, payload = protocol.recv_message(client)
        assert message_type == protocol.MessageType.STATE
        truth = protocol.TruthFrame.unpack(payload)
        assert truth.position_ned_m == pytest.approx((20.0, 10.0, -2.0))
        assert truth.velocity_ned_m_s == pytest.approx((10.0, 5.0, -1.0))
        assert truth.airspeed_m_s == pytest.approx(12.5)
        assert truth.latitude_deg > -35.363261
        assert truth.longitude_deg > 149.165230
        assert truth.altitude_m == pytest.approx(586.0)
    finally:
        stop_stub(client, server, thread)


def test_stub_accepts_a_truth_provider():
    client, server = socket.socketpair()

    def provider(configuration, step):
        truth = physics_stub._truth_for_step(configuration, step)
        return protocol.TruthFrame(**{
            **truth.__dict__,
            "latitude_deg": -34.0,
            "magnetic_field_body_mgauss": (300.0, -100.0, 50.0),
        })

    thread = threading.Thread(
        target=physics_stub.serve_connection,
        args=(server,),
        kwargs={"truth_provider": provider},
    )
    thread.start()
    try:
        configure_stub(client)
        step = protocol.StepFrame(2500, 11, empty_actuators())
        protocol.send_message(client, protocol.MessageType.STEP, step.pack())
        message_type, payload = protocol.recv_message(client)
        assert message_type == protocol.MessageType.STATE
        truth = protocol.TruthFrame.unpack(payload)
        assert truth.latitude_deg == -34.0
        assert truth.magnetic_field_body_mgauss == (300.0, -100.0, 50.0)
    finally:
        stop_stub(client, server, thread)


def test_stationary_stub_rejects_out_of_order_steps():
    client, server, thread = start_stub()
    try:
        configure_stub(client)
        step = protocol.StepFrame(2500, 11, empty_actuators())
        protocol.send_message(client, protocol.MessageType.STEP, step.pack())
        assert protocol.recv_message(client)[0] == protocol.MessageType.STATE
        protocol.send_message(client, protocol.MessageType.STEP, step.pack())
        message_type, payload = protocol.recv_message(client)
        assert message_type == protocol.MessageType.ERROR
        assert "sequence" in protocol.unpack_json(payload)["error"]
    finally:
        stop_stub(client, server, thread)


def test_stationary_stub_rejects_timestamp_regression():
    client, server, thread = start_stub()
    try:
        configure_stub(client)
        protocol.send_message(client, protocol.MessageType.STEP, protocol.StepFrame(2500, 11, empty_actuators()).pack())
        assert protocol.recv_message(client)[0] == protocol.MessageType.STATE
        protocol.send_message(client, protocol.MessageType.STEP, protocol.StepFrame(2500, 12, empty_actuators()).pack())
        message_type, payload = protocol.recv_message(client)
        assert message_type == protocol.MessageType.ERROR
        assert "timestamp" in protocol.unpack_json(payload)["error"]
    finally:
        stop_stub(client, server, thread)


@pytest.mark.parametrize(
    "first_type,payload,error",
    (
        (protocol.MessageType.STEP, protocol.StepFrame(1, 1, empty_actuators()).pack(), "expected HELLO"),
        (protocol.MessageType.HELLO, protocol.pack_json({"role": "other"}), "role must be renode"),
    ),
)
def test_stationary_stub_rejects_invalid_handshake(first_type, payload, error):
    client, server, thread = start_stub()
    try:
        protocol.send_message(client, first_type, payload)
        message_type, response = protocol.recv_message(client)
        assert message_type == protocol.MessageType.ERROR
        assert error in protocol.unpack_json(response)["error"]
    finally:
        stop_stub(client, server, thread)


@pytest.mark.parametrize(
    "configuration,error",
    (
        ({"model": "moving", "location": {}, "rate_hz": 400}, "requested model"),
        ({"model": "stationary", "location": [], "rate_hz": 400}, "location must be an object"),
        (
            {
                "model": "stationary",
                "location": {"latitude_deg": 91, "longitude_deg": 0, "altitude_m": 0},
                "rate_hz": 400,
            },
            "latitude_deg",
        ),
        (
            {
                "model": "stationary",
                "location": {"latitude_deg": 0, "longitude_deg": 181, "altitude_m": 0},
                "rate_hz": 400,
            },
            "longitude_deg",
        ),
        (
            {
                "model": "stationary",
                "location": {"latitude_deg": 0, "longitude_deg": 0, "altitude_m": 0},
                "rate_hz": 0,
            },
            "rate_hz",
        ),
    ),
)
def test_stationary_stub_rejects_invalid_configuration(configuration, error):
    client, server, thread = start_stub()
    try:
        protocol.send_message(client, protocol.MessageType.HELLO, protocol.pack_json({"role": "renode"}))
        assert protocol.recv_message(client)[0] == protocol.MessageType.HELLO_REPLY
        protocol.send_message(client, protocol.MessageType.CONFIGURE, protocol.pack_json(configuration))
        message_type, payload = protocol.recv_message(client)
        assert message_type == protocol.MessageType.ERROR
        assert error in protocol.unpack_json(payload)["error"]
    finally:
        stop_stub(client, server, thread)


def test_stationary_stub_handles_extreme_finite_location_values():
    configuration = {
        "altitude_m": -1e300,
        "heading_deg": 1e300,
        "latitude_deg": 0.0,
        "longitude_deg": 0.0,
        "model": "stationary",
        "rate_hz": 400.0,
    }
    truth = physics_stub._truth_for_step(configuration, protocol.StepFrame(1, 1, empty_actuators()))
    assert protocol.TruthFrame.unpack(truth.pack()).altitude_m == -1e300
