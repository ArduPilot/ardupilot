#!/usr/bin/env python3
"""HLK-LD2451 radar -> ArduPilot bridge (runs in WSL, next to SITL).

The Webots controller (ardupilot_bridge.py, on Windows) streams the nearest
front/rear radar target - distance (m) and |radial speed| (m/s) - over UDP.
This process turns that into real Hi-Link HLK-LD2451 UART frames and feeds
them straight into SITL's serial port, so ArduPilot's actual
AP_Proximity_HLK_LD2451 backend (PRX1_TYPE=19) parses them - not a generic
MAVLink DISTANCE_SENSOR standing in for it.

    Webots radar -> bridge (Win) -> UDP :9005 -> [this] -> HLK-LD2451 UART -> SITL

Frame format matches libraries/AP_Proximity/AP_Proximity_HLK_LD2451.cpp
exactly (verified against Tools/autotest/hlk_ld2451_sim.py in the ArduPilot
tree, which the real driver already passes):

  Header : F4 F3 F2 F1                    (4 bytes)
  Length : 2 bytes little-endian = 2 + N_targets * 5
  Payload:
    count  : 1 byte   number of targets (0 or 1 here - we only ever report
                       the single nearest target on the selected side)
    alarm  : 1 byte   0x01 if the target is approaching, else 0x00
    target : 5 bytes  angle_raw, dist_m, direction, speed_kmh, snr
  Tail   : F8 F7 F6 F5                    (4 bytes)

Direction (approaching/departing) is derived here from the distance trend
between consecutive UDP samples, not from Webots' Target.speed sign, which
isn't documented; the speed magnitude Webots does give us is used as-is.
SNR has no Webots equivalent, so it's synthesised from distance (closer =
higher confidence), matching real-radar intuition.

Note: this instance represents ONE physical HLK-LD2451 (PRX1, one --side).
The Webots rig has 4 corner radars but ArduPilot's proximity backends are
1 UART = 1 sensor; to model front AND rear, run two copies of this script
against two SITL serial ports and configure PRX1/PRX2 accordingly.

Run in WSL alongside SITL (front sensor, matching rover.parm's SERIAL2
default):
    python3 tools/radar_mav.py --side front --port 5763
"""
import argparse
import socket
import struct
import sys
import time

FRAME_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
FRAME_TAIL = bytes([0xF8, 0xF7, 0xF6, 0xF5])

UDP_PORT = 9005                # must match RADAR_OUT_PORT in ardupilot_bridge.py
RATE_HZ = 10
PERIOD_S = 1.0 / RATE_HZ
STALE_S = 0.5                  # no UDP data for this long -> report "clear"
DIST_MIN_M, DIST_MAX_M = 0.5, 200.0   # matches HLK_LD2451_DIST_MIN/MAX_M
TREND_EPS_M = 0.02             # ignore sub-2cm jitter when judging direction


def make_frame(target):
    """Build one HLK-LD2451 frame. target is None (no detection) or a dict
    with angle_deg, dist_m, approaching, speed_kmh, snr."""
    if target is None:
        # count=0, alarm=0 - the driver requires these 2 overhead bytes even
        # for "no target" (payload_len must be >= HLK_LD2451_PAYLOAD_OVERHEAD),
        # unlike the bare header+00 00+tail framing described in the ICD doc.
        payload = bytes([0, 0])
        return FRAME_HEADER + struct.pack("<H", len(payload)) + payload + FRAME_TAIL

    angle_raw = max(0, min(255, int(round(target["angle_deg"])) + 0x80))
    dist_m = max(0, min(255, int(round(target["dist_m"]))))
    direction = 0x01 if target["approaching"] else 0x00
    speed_kmh = max(0, min(255, int(round(target["speed_kmh"]))))
    snr = max(0, min(100, int(round(target["snr"]))))

    # payload layout: [count, alarm, angle, dist, direction, speed, snr].
    # alarm mirrors this single target's direction (matches the real
    # sensor's documented behaviour: alarm tracks approaching targets).
    payload = bytes([1, direction]) + bytes([angle_raw, dist_m, direction, speed_kmh, snr])

    length = 2 + 5
    return FRAME_HEADER + struct.pack("<H", length) + payload + FRAME_TAIL


def connect_tcp(host, port, retry_interval=2.0):
    """Block until a TCP connection to SITL's serial port is established."""
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.connect((host, port))
            print(f"radar_mav: connected to SITL at {host}:{port}")
            return sock
        except (ConnectionRefusedError, OSError) as exc:
            print(f"radar_mav: connection failed ({exc}), retrying in {retry_interval}s ...")
            time.sleep(retry_interval)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="SITL TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5763,
                         help="SITL TCP port for the serial this HLK-LD2451 instance is wired to "
                              "(default: 5763 = SERIAL2, matching rover.parm)")
    parser.add_argument("--side", choices=["front", "rear"], default="front",
                         help="which Webots corner-radar pair to report (default: front)")
    parser.add_argument("--udp-port", type=int, default=UDP_PORT,
                         help=f"UDP port to receive radar data from ardupilot_bridge.py (default: {UDP_PORT})")
    args = parser.parse_args()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(("0.0.0.0", args.udp_port))
    udp.settimeout(0.05)

    print(f"radar_mav: side={args.side}  listening for Webots radar on UDP {args.udp_port}, "
          f"feeding HLK-LD2451 frames to {args.host}:{args.port}")

    sock = connect_tcp(args.host, args.port)

    dist = -1.0
    speed_mps = 0.0
    last_rx = 0.0
    prev_dist = None
    printed = 0.0
    frame_count = 0

    front_idx = 0  # ardupilot_bridge.py sends "front_dist,front_speed,rear_dist,rear_speed"
    idx = front_idx if args.side == "front" else 2

    while True:
        try:
            data, _ = udp.recvfrom(64)
            fields = [float(x) for x in data.decode().split(",")]
            dist, speed_mps = fields[idx], fields[idx + 1]
            last_rx = time.time()
        except socket.timeout:
            pass
        except (ValueError, IndexError, OSError):
            continue

        if time.time() - last_rx > STALE_S:
            dist = -1.0

        if dist <= 0:
            target = None
            prev_dist = None
        else:
            dist_m = max(DIST_MIN_M, min(DIST_MAX_M, dist))
            approaching = prev_dist is not None and dist_m < prev_dist - TREND_EPS_M
            snr = max(10, min(100, round(100 - dist_m / 2)))
            target = {
                "angle_deg": 0.0,          # dead-ahead of this radar's own mount axis
                "dist_m": dist_m,
                "approaching": approaching,
                "speed_kmh": speed_mps * 3.6,
                "snr": snr,
            }
            prev_dist = dist_m

        frame = make_frame(target)
        try:
            sock.sendall(frame)
        except (BrokenPipeError, ConnectionResetError, OSError) as exc:
            print(f"radar_mav: send error: {exc} - reconnecting ...")
            try:
                sock.close()
            except OSError:
                pass
            sock = connect_tcp(args.host, args.port)
            continue

        frame_count += 1
        now = time.time()
        if target is not None and now - printed > 0.5:
            printed = now
            arrow = "->>" if target["approaching"] else "<<-"
            print(f"radar_mav: [{args.side}] {target['dist_m']:.1f}m {arrow} "
                  f"{target['speed_kmh']:.0f}km/h SNR={target['snr']}  frame#{frame_count}")

        time.sleep(PERIOD_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nradar_mav: stopped by user.")
        sys.exit(0)
