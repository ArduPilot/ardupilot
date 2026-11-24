#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Run a clearly labelled, non-actuating ArduSub/dashboard software demo."""

import argparse
import os
import pty
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from test_xsens_native_startup import BINARY, ROOT, data_item, packet


def check_local_port(port):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", port))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--http-port", type=int, default=8089)
    args = parser.parse_args()
    if not BINARY.is_file():
        parser.error("build the native Linux Sub first: ./waf configure --board=linux && ./waf sub -j4")
    check_local_port(args.http_port)
    with tempfile.TemporaryDirectory(prefix="atal-demo-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("127.0.0.1", 0))
            dashboard_port = sock.getsockname()[1]
        defaults = Path(directory) / "demo.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SERIAL3_PROTOCOL 2\nSERIAL3_BAUD 115\n"
            "SR3_EXTRA1 10\nSR3_EXTRA2 5\nSR3_RAW_SENS 5\nSR3_EXT_STAT 2\n"
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )
        dashboard = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dashboard_api.py"), "--demo-mode",
             "--mavlink", f"udpin:127.0.0.1:{dashboard_port}",
             "--http-port", str(args.http_port),
             "--recording-dir", str(Path(directory) / "recordings")],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        vehicle = subprocess.Popen(
            [str(BINARY), "--serial0", "udp:127.0.0.1:14551",
             "--serial1", os.ttyname(xsens_slave), "--serial2", os.ttyname(depth_slave),
             "--serial3", f"udp:127.0.0.1:{dashboard_port}",
             "--defaults", str(defaults), "--log-directory", directory,
             "--storage-directory", directory, "--terrain-directory", directory],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        imu = (data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
               + data_item(0x8020, struct.pack(">3f", 0.1, 0.2, 0.3))
               + data_item(0x4020, struct.pack(">3f", 1, 2, -9.81)))
        print(f"Software demo: http://127.0.0.1:{args.http_port}/", flush=True)
        print("Synthetic Xsens/Bar30 inputs; no vehicle or actuator connected. Press Ctrl-C to stop.", flush=True)
        counter = 0
        last_depth = 0.0
        try:
            while vehicle.poll() is None and dashboard.poll() is None:
                os.write(xsens_master, packet(0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if time.monotonic() - last_depth >= 0.05:
                    os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                    last_depth = time.monotonic()
                time.sleep(0.01)
            raise RuntimeError(f"demo process exited: Sub={vehicle.poll()}, dashboard={dashboard.poll()}")
        except KeyboardInterrupt:
            pass
        finally:
            for process in (vehicle, dashboard):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
