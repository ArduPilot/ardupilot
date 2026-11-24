#!/usr/bin/env python3
"""Translate native ArduSub RCOutput observations into guarded A1 frames.

This software-only adapter writes frames to stdout; it never opens a motor port.
"""

import argparse
import secrets
import select
import signal
import socket
import sys
import time

from actuator_frames import encode_frame
from rcoutput_mapping import load_motor_order, motors_in_esp_order, parse_observation


class ActuatorAdapter:
    TIMEOUT_MS = 200  # Shorter than ActuatorGuard's 250 ms output watchdog.

    def __init__(self, order, session, pwm_interval_ms=0):
        self.order = order
        self.session = session
        self.pwm_interval_ms = pwm_interval_ms
        self.sequence = 0
        self.ready = False
        self.armed = False
        self.last_observation_ms = None
        self.last_pwm_ms = None

    def _frame(self, command, pwms=None):
        if self.sequence == 0xFFFFFFFF:
            raise RuntimeError("Actuator sequence exhausted; restart with a new session")
        self.sequence += 1
        return encode_frame(command, self.session, self.sequence, pwms)

    def _stop(self):
        self.ready = False
        self.last_pwm_ms = None
        if not self.armed:
            return ()
        self.armed = False
        return (self._frame("DISARM"),)

    def tick(self, now_ms):
        if self.last_observation_ms is not None and now_ms - self.last_observation_ms > self.TIMEOUT_MS:
            self.last_observation_ms = None
            return self._stop()
        return ()

    def ingest(self, packet, now_ms):
        expired = self.tick(now_ms)
        try:
            armed, channels = parse_observation(packet)
        except ValueError:
            self.last_observation_ms = None
            return expired + self._stop()
        self.last_observation_ms = now_ms
        if not armed:
            was_armed = self.armed
            frames = self._stop() if was_armed else ()
            self.ready = True
            return expired + frames
        if not self.ready:
            return expired
        mapped = motors_in_esp_order(channels, self.order)
        frames = ()
        if not self.armed:
            self.armed = True
            frames = (self._frame("ARM"),)
        if self.last_pwm_ms is not None and now_ms - self.last_pwm_ms < self.pwm_interval_ms:
            return expired + frames
        self.last_pwm_ms = now_ms
        return expired + frames + (self._frame("PWM", mapped),)


def main():
    parser = argparse.ArgumentParser(description="Software-only ArduSub RCOutput to A1 frame adapter")
    parser.add_argument("--port", type=int, default=14660, help="localhost observation UDP port")
    args = parser.parse_args()
    adapter = ActuatorAdapter(load_motor_order(), secrets.randbits(32))
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, request_stop)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", args.port))
        receiver.settimeout(0.05)
        print("Waiting for disarmed ArduSub RCOutput observation", file=sys.stderr)
        try:
            while not stop_requested:
                try:
                    packet = receiver.recv(256)
                    # A slow consumer must act on the newest mixer cycle, never
                    # replay queued motor outputs that have already expired.
                    while select.select([receiver], [], [], 0)[0]:
                        packet = receiver.recv(256)
                    frames = adapter.ingest(packet, time.monotonic_ns() // 1_000_000)
                except socket.timeout:
                    frames = adapter.tick(time.monotonic_ns() // 1_000_000)
                for frame in frames:
                    print(frame, flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            for frame in adapter._stop():
                print(frame, flush=True)


if __name__ == "__main__":
    main()
