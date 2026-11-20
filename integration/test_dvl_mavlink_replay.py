#!/usr/bin/env python3
"""Exercise the A50 bridge with loopback TCP and MAVLink only."""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "modules" / "mavlink"))
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil


def main():
    with tempfile.TemporaryDirectory(prefix="atal-dvl-") as directory:
        tcp = socket.socket()
        tcp.bind(("127.0.0.1", 0))
        tcp.listen(1)
        tcp.settimeout(4)
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind(("127.0.0.1", 0))
        udp.settimeout(0.1)
        config = json.loads((ROOT / "integration/vehicle_defaults.json").read_text())
        config["dvl_a50"].update(ip="127.0.0.1", tcp_port=tcp.getsockname()[1])
        config_path = Path(directory) / "defaults.json"
        config_path.write_text(json.dumps(config))
        process = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dvl_bridge.py"),
             "--config", str(config_path), "--connect-a50"],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        device = None
        try:
            device, _ = tcp.accept()
            encoder = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            decoder = mavutil.mavlink.MAVLink(None)
            report = {
                "type": "velocity", "velocity_valid": True, "status": 0,
                "altitude": 2.0, "fom": 0.1, "vx": 1.0, "vy": -0.5,
                "vz": 0.25, "time_of_validity": 123456789,
            }

            def attitude():
                frame = encoder.attitude_quaternion_encode(
                    1000, 1, 0, 0, 0, 0, 0, 0).pack(encoder)
                udp.sendto(frame, ("127.0.0.1", 14551))

            def receive_for(seconds, send_attitude=True):
                found = []
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    if send_attitude:
                        attitude()
                    try:
                        data, _ = udp.recvfrom(4096)
                    except socket.timeout:
                        continue
                    for byte in data:
                        message = decoder.parse_char(bytes([byte]))
                        if message and message.get_type() == "VISION_SPEED_ESTIMATE":
                            found.append(message)
                return found

            receive_for(0.2)  # establish the loopback peer and fresh attitude
            device.sendall((json.dumps(dict(report, velocity_valid=False)) + "\n").encode())
            if receive_for(0.3):
                raise AssertionError("invalid A50 bottom lock emitted MAVLink velocity")
            device.sendall((json.dumps(report) + "\n").encode())
            messages = receive_for(0.7)
            if len(messages) != 1:
                raise AssertionError(f"expected one velocity packet, received {len(messages)}")
            message = messages[0]
            if any(abs(actual - expected) > 1e-4 for actual, expected in
                   zip((message.x, message.y, message.z), (1.0, -0.5, 0.25))):
                raise AssertionError("A50 velocity changed before MAVLink transmission")
            expected_variance = 0.1 ** 2 / 3
            if any(abs(message.covariance[index] - expected_variance) > 1e-5
                   for index in (0, 4, 8)):
                raise AssertionError("A50 uncertainty was not sent in MAVLink covariance")
            device.sendall((json.dumps(report) + "\n").encode())
            if receive_for(0.3):
                raise AssertionError("duplicate A50 timestamp emitted another packet")
            receive_for(0.6, send_attitude=False)
            device.sendall((json.dumps(dict(report, time_of_validity=123456790)) + "\n").encode())
            if receive_for(0.2, send_attitude=False):
                raise AssertionError("stale attitude emitted MAVLink velocity")
            receive_for(0.2)
            device.sendall((json.dumps(dict(report, time_of_validity=123456791)) + "\n").encode())
            if len(receive_for(0.7)) != 1:
                raise AssertionError("A50 velocity did not recover after fresh attitude")
            print("A50 bridge loopback MAVLink replay passed")
        finally:
            if device is not None:
                device.close()
            tcp.close()
            udp.close()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            if process.returncode not in (None, 0, -signal.SIGTERM):
                print(process.stderr.read().decode(errors="replace"), file=sys.stderr)
            process.stderr.close()


if __name__ == "__main__":
    main()
