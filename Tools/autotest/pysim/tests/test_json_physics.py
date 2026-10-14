"""Tests for Tools/autotest/pysim/json_physics.py."""

import json
import socket
import struct

from collections.abc import Iterator

import pytest

from Tools.autotest.pysim.json_physics import JSONPhysics


def pkt(frame_count: int, rate: int = 400) -> bytes:
    return struct.pack('<HHI16H', 18458, rate, frame_count, *([1500] * 16))


def reply(physics: JSONPhysics, data: bytes) -> bytes:
    r = physics.handle(data)
    assert r is not None
    return r


def test_lockstep_replies() -> None:
    p = JSONPhysics(dt=0.01, start_time=5.0)
    p.set(velocity=[1.0, 0.0, 0.0])
    first = json.loads(reply(p, pkt(0)))
    assert first['timestamp'] == 5.0
    assert first['position'][0] == pytest.approx(0.01)
    # a resent frame gets the same answer, time does not advance
    assert p.handle(pkt(0)) == p.last_reply and p.replies == 1
    second = json.loads(reply(p, pkt(1)))
    assert second['timestamp'] == pytest.approx(5.01)
    assert p.frame_rate == 400 and p.pwm == tuple([1500] * 16)


def test_servo_packets() -> None:
    p = JSONPhysics()
    assert p.handle(b'\x00\x00') is None  # bad magic
    assert p.handle(pkt(0)[:-2]) is None  # bad length
    reply(p, struct.pack('<HHI32H', 29569, 400, 0, *([1100] * 32)))
    assert p.pwm == tuple([1100] * 32)


def test_fields() -> None:
    p = JSONPhysics()
    # SIM_JSON rejects booleans after '": ', so no spaces
    p.set(no_lockstep=True)
    assert b'"no_lockstep":true' in reply(p, pkt(0))
    p.set(attitude=None)
    assert b'attitude' not in reply(p, pkt(1))


def test_paused() -> None:
    p = JSONPhysics()
    p.paused = True
    assert p.handle(pkt(0)) is None and p.pwm == tuple([1500] * 16)
    assert p.replies == 0


@pytest.fixture
def sitl() -> Iterator[socket.socket]:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1)
    yield s
    s.close()


def started(physics: JSONPhysics) -> tuple[str, int]:
    physics.start()
    assert physics.sock is not None
    host, port = physics.sock.getsockname()
    return host, port


def test_udp_lockstep(sitl: socket.socket) -> None:
    p = JSONPhysics(port=0)
    addr = started(p)
    try:
        for frame in range(3):
            sitl.sendto(pkt(frame), addr)
            msg = json.loads(sitl.recv(4096))
            assert msg['timestamp'] == pytest.approx(frame * p.dt)
    finally:
        p.stop()


def test_udp_free_run(sitl: socket.socket) -> None:
    p = JSONPhysics(port=0, dt=0.01, free_run=True)
    addr = started(p)
    try:
        # one packet tells the backend where SITL is, then it sends on its own
        sitl.sendto(pkt(0), addr)
        timestamps = [json.loads(sitl.recv(4096))['timestamp'] for _ in range(5)]
        assert timestamps == pytest.approx([i * p.dt for i in range(5)])
    finally:
        p.stop()
