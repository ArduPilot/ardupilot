#!/usr/bin/env python3

"""Integration test for the standalone ArduPilot physics sidecar."""

# AP_FLAKE8_CLEAN

import os
import queue
import socket
import subprocess
import sys
import threading

from pathlib import Path

import pytest

RENODE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = RENODE_ROOT.parents[1]
sys.path.insert(0, str(RENODE_ROOT))

import physics_protocol as protocol  # noqa: E402


def _unused_actuators():
    return (protocol.Actuator(),) * protocol.ACTUATOR_COUNT


def _find_free_port():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def _read_lines(stream, lines):
    for line in stream:
        lines.put(line)


def _expect(connection, expected_type):
    message_type, payload = protocol.recv_message(connection)
    assert message_type is expected_type
    return payload


def _hello(connection):
    protocol.send_message(connection, protocol.MessageType.HELLO, protocol.pack_json({"role": "renode"}))
    return protocol.unpack_json(_expect(connection, protocol.MessageType.HELLO_REPLY))


def _configure(connection, configuration):
    protocol.send_message(
        connection, protocol.MessageType.CONFIGURE, protocol.pack_json(configuration)
    )
    return protocol.unpack_json(_expect(connection, protocol.MessageType.CONFIGURE_REPLY))


def test_quad_model_responds_to_pwm(tmp_path):
    configured_binary = os.environ.get("RENODE_PHYSICS_BINARY")
    binary = Path(configured_binary) if configured_binary else REPOSITORY_ROOT / "build/sitl/tool/renode-physics"
    if not binary.is_file():
        pytest.skip("build tool/renode-physics or set RENODE_PHYSICS_BINARY")

    port = _find_free_port()
    process = subprocess.Popen(
        [str(binary.resolve()), "--physics-port", str(port), "--model", "quad"],
        cwd=tmp_path,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    lines = queue.Queue()
    reader = threading.Thread(target=_read_lines, args=(process.stdout, lines), daemon=True)
    reader.start()
    try:
        while True:
            line = lines.get(timeout=10)
            if line.startswith("PHYSICS_PORT "):
                assert int(line.split()[1]) == port
                break

        with socket.create_connection(("127.0.0.1", port), timeout=5) as connection:
            connection.settimeout(5)
            assert _hello(connection) == {"models": ["quad"], "role": "physics"}

            configuration = {
                "location": {
                    "altitude_m": 584.0,
                    "heading_deg": 90.0,
                    "latitude_deg": -35.363261,
                    "longitude_deg": 149.165230,
                },
                "model": "quad",
                "rate_hz": 400,
            }
            assert _configure(connection, configuration) == {"model": "quad", "status": "configured"}

            motor = protocol.Actuator(
                2000, protocol.ActuatorProtocol.PWM, protocol.ACTUATOR_FLAG_VALID
            )
            actuators = (motor,) * 4 + _unused_actuators()[4:]
            first_truth = None
            for sequence in range(1, 401):
                step = protocol.StepFrame(sequence * 2500, sequence, actuators)
                protocol.send_message(connection, protocol.MessageType.STEP, step.pack())
                truth = protocol.TruthFrame.unpack(_expect(connection, protocol.MessageType.STATE))
                if first_truth is None:
                    first_truth = truth

            assert first_truth.sequence == 1
            assert truth.sequence == 400
            assert truth.timestamp_us == 1000000
            assert truth.altitude_m > first_truth.altitude_m + 0.5
            assert truth.battery_current_a > 0.0

        # Reconnecting represents an FC/bridge reset, not a physical vehicle
        # reset. The model continues, but STEP time restarts at zero.
        with socket.create_connection(("127.0.0.1", port), timeout=5) as connection:
            connection.settimeout(5)
            _hello(connection)
            _configure(connection, configuration)
            step = protocol.StepFrame(2500, 1, _unused_actuators())
            protocol.send_message(connection, protocol.MessageType.STEP, step.pack())
            resumed = protocol.TruthFrame.unpack(_expect(connection, protocol.MessageType.STATE))
            assert resumed.sequence == 1
            assert resumed.altitude_m >= truth.altitude_m

            protocol.send_message(connection, protocol.MessageType.STEP, step.pack())
            error = protocol.unpack_json(_expect(connection, protocol.MessageType.ERROR))
            assert "increase" in error["error"]

        with socket.create_connection(("127.0.0.1", port), timeout=5) as connection:
            connection.settimeout(5)
            protocol.send_message(
                connection, protocol.MessageType.HELLO, protocol.pack_json({"role": "invalid"})
            )
            error = protocol.unpack_json(_expect(connection, protocol.MessageType.ERROR))
            assert "role" in error["error"]

        with socket.create_connection(("127.0.0.1", port), timeout=5) as connection:
            connection.settimeout(5)
            _hello(connection)
            invalid_configuration = dict(configuration)
            invalid_configuration["rate_hz"] = 10000
            protocol.send_message(
                connection,
                protocol.MessageType.CONFIGURE,
                protocol.pack_json(invalid_configuration),
            )
            error = protocol.unpack_json(_expect(connection, protocol.MessageType.ERROR))
            assert "model rate" in error["error"]
    finally:
        process.terminate()
        process.wait(timeout=10)
