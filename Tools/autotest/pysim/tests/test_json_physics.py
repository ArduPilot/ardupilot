"""Tests for Tools/autotest/pysim/json_physics.py."""

import json
import socket
import statistics
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
    assert p.handle(pkt(0)) == p.last_reply and p.replies == 1 and p.resends == 1
    second = json.loads(reply(p, pkt(1)))
    assert second['timestamp'] == pytest.approx(5.01)
    assert p.frame_rate == 400 and p.pwm == tuple([1500] * 16)


def test_servo_packets() -> None:
    p = JSONPhysics()
    assert p.handle(b'\x00\x00') is None  # bad magic
    assert p.handle(pkt(0)[:-2]) is None  # bad length
    reply(p, struct.pack('<HHI32H', 29569, 400, 0, *([1100] * 32)))
    assert p.pwm == tuple([1100] * 32)


def test_frame_rates() -> None:
    p = JSONPhysics()
    # every servo packet counts, resent or paused, not a bad one
    reply(p, pkt(0, rate=400))
    p.handle(pkt(0, rate=400))
    p.handle(b'\x00\x00')
    p.paused = True
    p.handle(pkt(1, rate=401))
    assert list(p.frame_rates) == [400, 400, 401]
    # one odd rate does not move the median the tests take
    assert statistics.median(p.frame_rates) == 400
    # only the newest 100 are kept
    p.paused = False
    for n in range(2, 152):
        p.handle(pkt(n, rate=n))
    assert len(p.frame_rates) == 100 and p.frame_rates[0] == 52 and p.frame_rates[-1] == 151


def test_fields() -> None:
    p = JSONPhysics()
    p.set(no_lockstep=True)
    assert b'"no_lockstep":true' in reply(p, pkt(0))
    p.set(attitude=None)
    assert b'attitude' not in reply(p, pkt(1))
    # json.dumps writes infinity as Infinity, which SIM_JSON accepts
    p.set(rng_1=float('inf'))
    assert b'"rng_1":Infinity' in reply(p, pkt(2))


def test_raw_fields() -> None:
    p = JSONPhysics()
    p.set(rng_1=5.0)
    p.set_raw(rng_1='null', rng_2='-inf')
    msg = reply(p, pkt(0))
    assert msg.endswith(b',"rng_1":null,"rng_2":-inf}\n') and msg.count(b'rng_1') == 1
    # set() and set_raw() replace each other's value for a field
    p.set(rng_1=4.0)
    p.set_raw(rng_2=None)
    msg = reply(p, pkt(1))
    assert json.loads(msg)['rng_1'] == 4.0 and b'rng_2' not in msg


def test_injected() -> None:
    p = JSONPhysics(dt=0.01)
    reply(p, pkt(0))
    p.inject(b'bad 1', b'bad 2')
    # one per frame, without advancing time; SITL sending the frame
    # again gets a normal reply, not taken as a resend
    assert p.handle(pkt(1)) == b'bad 1'
    assert json.loads(reply(p, pkt(1)))['timestamp'] == pytest.approx(0.01)
    assert p.handle(pkt(2)) == b'bad 2'
    assert json.loads(reply(p, pkt(2)))['timestamp'] == pytest.approx(0.02)
    assert json.loads(reply(p, pkt(3)))['timestamp'] == pytest.approx(0.03)
    assert p.resends == 0 and p.replies == 4


def test_framing() -> None:
    p = JSONPhysics()
    p.framing = lambda text: text + b'\0'
    msg = reply(p, pkt(0))
    assert msg.startswith(b'{') and msg.endswith(b'}\0')


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
