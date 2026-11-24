#!/usr/bin/env python3
"""Native ArduSub RCOutput -> guarded ESP32 serial, opt-in and default disabled."""

import argparse
import json
from pathlib import Path
import re
import secrets
import select
import signal
import socket
import time

from actuator_adapter import ActuatorAdapter
from rcoutput_mapping import load_motor_order


SESSION = re.compile(rb"A1,SESSION,([0-9A-Fa-f]{8}),([0-9A-F]{8})\r?\n")


class StatusMonitor:
    """Bound MCU status buffering so a noisy USB port cannot starve RCOutput."""

    def __init__(self):
        self.pending = b""

    def ingest(self, data):
        self.pending += data
        if len(self.pending) > 256:
            raise RuntimeError("ESP32 status stream overflow; restart bridge")
        while b"\n" in self.pending:
            line, self.pending = self.pending.split(b"\n", 1)
            line = line.rstrip(b"\r")
            if line == b"A1,FAULT":
                raise RuntimeError("ESP32 guard fault; inspect and restart")
            if line.startswith(b"A1,SESSION,"):
                raise RuntimeError("ESP32 session changed; restart bridge")
            if line:
                raise RuntimeError("Unexpected ESP32 status; restart bridge")


def read_session(serial_port, timeout_s=5):
    challenge = f"{secrets.randbits(32):08X}".encode("ascii")
    deadline = time.monotonic() + timeout_s
    next_request = 0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_request:
            serial_port.write(b"A1,HELLO," + challenge + b"\n")
            next_request = time.monotonic() + 0.5
        line = serial_port.readline()
        match = SESSION.fullmatch(line)
        if match and match.group(2) == challenge:
            return int(match.group(1), 16)
    raise TimeoutError("ESP32 session announcement not received")


def run(serial_port, udp_port, session, stop, order=None):
    adapter = ActuatorAdapter(load_motor_order() if order is None else order,
                              session, pwm_interval_ms=20)
    status = StatusMonitor()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", udp_port))
        receiver.settimeout(0.05)
        try:
            while not stop():
                available = serial_port.in_waiting
                if available:
                    status.ingest(serial_port.read(min(available, 256)))
                try:
                    packet = receiver.recv(256)
                    for _ in range(64):
                        if not select.select([receiver], [], [], 0)[0]:
                            break
                        packet = receiver.recv(256)
                    frames = adapter.ingest(packet, time.monotonic_ns() // 1_000_000)
                except socket.timeout:
                    frames = adapter.tick(time.monotonic_ns() // 1_000_000)
                for frame in frames:
                    serial_port.write((frame + "\n").encode("ascii"))
        finally:
            for frame in adapter._stop():
                serial_port.write((frame + "\n").encode("ascii"))
            serial_port.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="ESP32 serial device")
    parser.add_argument("--port", type=int, default=14660, help="localhost RCOutput UDP port")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("vehicle_defaults.json"))
    parser.add_argument("--enable-hardware", action="store_true")
    args = parser.parse_args()
    if not args.enable_hardware or not json.loads(args.config.read_text())["actuators"]["enabled"]:
        parser.error("serial bridge disabled: require --enable-hardware and actuators.enabled=true")
    order = load_motor_order(args.config, allow_enabled=True)
    import serial
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, request_stop)
    with serial.Serial(args.device, 115200, timeout=0.1, write_timeout=0.2) as serial_port:
        session = read_session(serial_port)
        run(serial_port, args.port, session, lambda: stopping, order)


if __name__ == "__main__":
    main()
