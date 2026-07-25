#!/usr/bin/env python3
"""
HLK-LD2451 24GHz Radar Sensor SITL Simulator for ArduPilot.

Connects to ArduPilot SITL via TCP (default: 127.0.0.1:5763, which maps to
SERIAL2) and sends valid LD2451 data frames at 10 Hz.

Protocol reference: Hi-Link HLK-LD2451 Serial Communication Protocol v1.03

Frame structure:
  Header : F4 F3 F2 F1                    (4 bytes)
  Length : 2 bytes little-endian = 2 + N_targets * 5
  Payload:
    count  : 1 byte   number of targets (0-3)
    alarm  : 1 byte   0x01 if any target is approaching, else 0x00
    Per target (5 bytes each):
      angle_raw : 1 byte  actual_degrees = raw_byte - 0x80
                          (0x80=0deg, 0x8A=+10deg, 0x76=-10deg)
      dist_m    : 1 byte  unsigned integer metres
      direction : 1 byte  0x00=moving away, 0x01=approaching
      speed_kmh : 1 byte  unsigned km/h
      snr       : 1 byte  0-100
  Tail   : F8 F7 F6 F5                    (4 bytes)

Example: 1 target at 30m, 0 deg, approaching at 60 km/h, SNR=80
  count=1, alarm=0x01
  angle_raw=0x80, dist=0x1E, dir=0x01, speed=0x3C, snr=0x50
  length=7
  Frame: F4 F3 F2 F1  07 00  01 01  80 1E 01 3C 50  F8 F7 F6 F5
"""

import argparse
import socket
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
FRAME_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
FRAME_TAIL   = bytes([0xF8, 0xF7, 0xF6, 0xF5])
RATE_HZ      = 10
PERIOD_S     = 1.0 / RATE_HZ


# ---------------------------------------------------------------------------
# Frame builder
# ---------------------------------------------------------------------------
def make_frame(targets):
    """
    Build a complete LD2451 data frame.

    Parameters
    ----------
    targets : list[dict]
        Each dict has keys:
          angle_deg  : float  angle in degrees (negative = left, positive = right)
          dist_m     : int    distance in metres (0-255)
          approaching: bool   True if target is approaching
          speed_kmh  : int    speed in km/h (0-255)
          snr        : int    signal-to-noise ratio (0-100)

    Returns
    -------
    bytes
        Complete framed byte sequence ready to send over serial/TCP.
    """
    n = len(targets)
    if n > 3:
        raise ValueError("LD2451 supports at most 3 targets")

    # alarm byte: 0x01 if any target is approaching
    alarm = 0x01 if any(t.get("approaching", False) for t in targets) else 0x00

    payload = bytes([n, alarm])
    for t in targets:
        angle_deg  = t.get("angle_deg", 0.0)
        dist_m     = int(t.get("dist_m", 0))
        approaching = t.get("approaching", False)
        speed_kmh  = int(t.get("speed_kmh", 0))
        snr        = int(t.get("snr", 50))

        # Clamp values to valid byte ranges
        dist_m    = max(0, min(255, dist_m))
        speed_kmh = max(0, min(255, speed_kmh))
        snr       = max(0, min(100, snr))

        # Encode angle: raw = round(degrees) + 0x80, clamped to [0, 255]
        angle_raw = int(round(angle_deg)) + 0x80
        angle_raw = max(0, min(255, angle_raw))

        direction = 0x01 if approaching else 0x00

        payload += bytes([angle_raw, dist_m, direction, speed_kmh, snr])

    # Length field = 2 (count + alarm) + 5 * N_targets
    length = 2 + 5 * n
    length_bytes = struct.pack("<H", length)

    frame = FRAME_HEADER + length_bytes + payload + FRAME_TAIL
    return frame


# ---------------------------------------------------------------------------
# Scenario generators
# ---------------------------------------------------------------------------
def scenario_approach(elapsed):
    """
    Single target starting at 30 m, approaching to 5 m over 30 seconds,
    then resetting.
    """
    cycle = elapsed % 30.0
    # linear interpolation: 30m -> 5m
    dist = 30.0 - (25.0 * cycle / 30.0)
    targets = [
        {
            "angle_deg":   0.0,
            "dist_m":      int(round(dist)),
            "approaching": True,
            "speed_kmh":   int(round(25.0 / 30.0 * 3.6)),  # ~3 km/h closing
            "snr":         80,
        }
    ]
    return targets


def scenario_static(elapsed):
    """Single stationary target at 15 m, dead ahead."""
    targets = [
        {
            "angle_deg":   0.0,
            "dist_m":      15,
            "approaching": False,
            "speed_kmh":   0,
            "snr":         75,
        }
    ]
    return targets


def scenario_multi(elapsed):
    """
    Three targets:
      - Centre (0 deg): approaching from 25 m
      - Left (-20 deg): stationary at 10 m
      - Right (+15 deg): moving away at 20 m
    """
    cycle = elapsed % 25.0
    dist_centre = max(5, int(round(25.0 - cycle)))
    targets = [
        {
            "angle_deg":   0.0,
            "dist_m":      dist_centre,
            "approaching": True,
            "speed_kmh":   36,
            "snr":         82,
        },
        {
            "angle_deg":   -20.0,
            "dist_m":      10,
            "approaching": False,
            "speed_kmh":   0,
            "snr":         60,
        },
        {
            "angle_deg":   15.0,
            "dist_m":      20,
            "approaching": False,
            "speed_kmh":   10,
            "snr":         55,
        },
    ]
    return targets


SCENARIOS = {
    "approach": scenario_approach,
    "static":   scenario_static,
    "multi":    scenario_multi,
}


# ---------------------------------------------------------------------------
# TCP connection helpers
# ---------------------------------------------------------------------------
def connect_tcp(host, port, retry_interval=2.0):
    """
    Block until a TCP connection to host:port is established.
    Returns the connected socket.
    """
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.connect((host, port))
            print(f"[LD2451-SIM] Connected to {host}:{port}")
            return sock
        except (ConnectionRefusedError, OSError) as exc:
            print(f"[LD2451-SIM] Connection failed ({exc}), retrying in {retry_interval}s ...")
            time.sleep(retry_interval)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def run(host, port, scenario_name):
    scenario_fn = SCENARIOS[scenario_name]
    print(f"[LD2451-SIM] Starting scenario '{scenario_name}' -> {host}:{port} at {RATE_HZ} Hz")

    sock = connect_tcp(host, port)
    start_time = time.monotonic()
    frame_count = 0

    while True:
        elapsed = time.monotonic() - start_time
        targets = scenario_fn(elapsed)

        frame = make_frame(targets)

        # Pretty-print for debugging
        hex_str = " ".join(f"{b:02X}" for b in frame)
        t_info = ", ".join(
            f"[{t['angle_deg']:+.0f}deg {t['dist_m']}m {'->>' if t['approaching'] else '<<-'} {t['speed_kmh']}km/h SNR={t['snr']}]"
            for t in targets
        )
        print(f"[LD2451-SIM] t={elapsed:6.1f}s  frame#{frame_count:04d}  {t_info}  hex={hex_str}")

        try:
            sock.sendall(frame)
        except (BrokenPipeError, ConnectionResetError, OSError) as exc:
            print(f"[LD2451-SIM] Send error: {exc} — reconnecting ...")
            try:
                sock.close()
            except OSError:
                pass
            sock = connect_tcp(host, port)
            start_time = time.monotonic()
            frame_count = 0
            continue

        frame_count += 1
        time.sleep(PERIOD_S)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="HLK-LD2451 24GHz radar SITL simulator for ArduPilot"
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="SITL TCP host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5763,
        help="SITL TCP port for SERIAL2 (default: 5763)",
    )
    parser.add_argument(
        "--scenario",
        choices=list(SCENARIOS.keys()),
        default="approach",
        help="Simulation scenario (default: approach)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        run(args.host, args.port, args.scenario)
    except KeyboardInterrupt:
        print("\n[LD2451-SIM] Stopped by user.")
        sys.exit(0)
