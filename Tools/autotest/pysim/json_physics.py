"""
Minimal physics backend for SITL's JSON model (--model JSON), for autotest.

SITL sends one servo packet per physics step to UDP 9002 (+10*instance)
and blocks until it gets a JSON state back. This answers each packet
with self.state plus a timestamp advanced by a fixed dt, so tests
control exactly what the vehicle senses and when.

See libraries/SITL/examples/JSON/readme.md for the protocol.

AP_FLAKE8_CLEAN
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

from typing import Any

# magic -> struct format for SIM_JSON's servo_packet_16 / servo_packet_32
SERVO_PACKETS: dict[int, str] = {
    18458: '<HHI16H',
    29569: '<HHI32H',
}


class JSONPhysics(object):

    def __init__(self, dt: float = 0.0025, start_time: float = 0.0, port: int = 9002, free_run: bool = False) -> None:
        self.dt = dt
        self.timestamp = start_time
        self.port = port
        # send a state every dt of wall-clock time instead of answering
        # each servo packet, as a backend using no_lockstep does
        self.free_run = free_run
        # stop sending, as a stalled or crashed backend
        self.paused = False
        # vehicle at rest at home, level, facing north; tests may
        # change or add any field SIM_JSON understands
        self.state: dict[str, Any] = {
            'imu': {
                'gyro': [0.0, 0.0, 0.0],
                'accel_body': [0.0, 0.0, -9.80665],
            },
            'position': [0.0, 0.0, 0.0],
            'attitude': [0.0, 0.0, 0.0],
            'velocity': [0.0, 0.0, 0.0],
        }
        self.lock = threading.Lock()
        self.frame_rate: int | None = None
        self.frame_count: int | None = None
        self.pwm: tuple[int, ...] | None = None
        self.packets = 0
        self.replies = 0
        self.last_reply: bytes | None = None
        self.sock: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self.running = False

    def start(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', self.port))
        self.sock.settimeout(self.dt / 4 if self.free_run else 0.1)
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join()
            self.thread = None
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def set(self, **fields: Any) -> None:
        '''replace top-level fields of the state sent to SITL, None removes one'''
        with self.lock:
            for name, value in fields.items():
                if value is None:
                    self.state.pop(name, None)
                else:
                    self.state[name] = value

    def step(self) -> bytes:
        '''advance the physics by dt, return the message for SITL; call with lock held'''
        if self.last_reply is not None:
            self.timestamp += self.dt
        # position follows velocity, so a test only sets velocity
        pos = self.state['position']
        vel = self.state['velocity']
        self.state['position'] = [p + v * self.dt for p, v in zip(pos, vel)]
        msg = dict(self.state, timestamp=self.timestamp)
        # compact separators: SIM_JSON expects the value right after
        # '":', a space makes it reject booleans such as no_lockstep
        self.last_reply = ('\n' + json.dumps(msg, separators=(',', ':')) + '\n').encode('ascii')
        self.replies += 1
        return self.last_reply

    def handle(self, data: bytes) -> bytes | None:
        '''decode one servo packet, return the reply to send or None'''
        if len(data) < 2:
            return None
        (magic,) = struct.unpack_from('<H', data)
        fmt = SERVO_PACKETS.get(magic)
        if fmt is None or len(data) != struct.calcsize(fmt):
            return None
        fields = struct.unpack(fmt, data)
        frame_count = fields[2]
        with self.lock:
            self.packets += 1
            self.frame_rate = fields[1]
            self.pwm = fields[3:]
            if self.paused or self.free_run:
                return None
            if frame_count == self.frame_count and self.last_reply is not None:
                # SITL resends servos when it has waited ~1s for us;
                # answer with the same state so time does not jump
                return self.last_reply
            self.frame_count = frame_count
            return self.step()

    def run(self) -> None:
        sock = self.sock
        assert sock is not None
        addr = None
        next_send = time.monotonic()
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                reply = self.handle(data)
                if reply is not None:
                    sock.sendto(reply, addr)
            except socket.timeout:
                pass
            except OSError:
                break
            now = time.monotonic()
            if not self.free_run or addr is None or now < next_send:
                continue
            # no catching up after a pause or before SITL connected
            next_send = max(next_send, now - self.dt) + self.dt
            with self.lock:
                if self.paused:
                    continue
                msg = self.step()
            sock.sendto(msg, addr)
