#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Run a clearly labelled, non-actuating ArduSub/dashboard software demo.

Runs native Linux ArduSub with virtual Xsens, Bar30, and Water Linked A50
synthetic inputs (via UnderwaterDynamicsSim), and opens the interactive
`/classic` ArduSub dashboard with `--enable-control` on localhost (no hardware
actuators are connected or driven).
"""

import argparse
import json
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
from vehicle_dynamics_sim import UnderwaterDynamicsSim


def check_local_port(port):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", port))


def unused_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--http-port", type=int, default=8089)
    parser.add_argument("--read-only", action="store_true",
                        help="start dashboard without --enable-control")
    args = parser.parse_args()
    if not BINARY.is_file():
        parser.error("build the native Linux Sub first: ./waf configure --board=linux && ./waf sub -j4")
    check_local_port(args.http_port)
    with tempfile.TemporaryDirectory(prefix="atal-demo-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        dashboard_port = unused_udp_port()
        dvl_status_port = unused_udp_port()
        tcp = socket.socket()
        tcp.bind(("127.0.0.1", 0))
        tcp.listen(1)
        tcp.settimeout(5)
        cfg = json.loads((ROOT / "integration/vehicle_defaults.json").read_text())
        cfg["dvl_a50"].update(ip="127.0.0.1", tcp_port=tcp.getsockname()[1])
        cfg_path = Path(directory) / "vehicle.json"
        cfg_path.write_text(json.dumps(cfg))

        defaults = Path(directory) / "demo.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SERIAL3_PROTOCOL 2\nSERIAL3_BAUD 115\n"
            "SR3_EXTRA1 10\nSR3_EXTRA2 5\nSR3_RAW_SENS 5\n"
            "SR3_EXT_STAT 2\nSR3_POSITION 10\n"
            + (ROOT / "integration/atal_dvl_ekf_replay.parm").read_text()
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )

        dash_cmd = [
            sys.executable, str(ROOT / "integration/dashboard_api.py"), "--demo-mode",
            "--mavlink", f"udpin:127.0.0.1:{dashboard_port}",
            "--http-port", str(args.http_port),
            "--dvl-status-udp", f"127.0.0.1:{dvl_status_port}",
            "--recording-dir", str(Path(directory) / "recordings"),
        ]
        if not args.read_only:
            dash_cmd.append("--enable-control")

        dashboard = subprocess.Popen(
            dash_cmd, cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        dvl_bridge = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dvl_bridge.py"),
             "--config", str(cfg_path), "--connect-a50",
             "--status-udp", f"127.0.0.1:{dvl_status_port}"],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        dvl_sock, _ = tcp.accept()
        vehicle = subprocess.Popen(
            [str(BINARY), "--serial0", "udp:127.0.0.1:14551",
             "--serial1", os.ttyname(xsens_slave), "--serial2", os.ttyname(depth_slave),
             "--serial3", f"udp:127.0.0.1:{dashboard_port}",
             "--defaults", str(defaults), "--log-directory", directory,
             "--storage-directory", directory, "--terrain-directory", directory],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        sim = UnderwaterDynamicsSim(cfg_path)
        print(f"Classic ROS-layout ArduSub dashboard: http://127.0.0.1:{args.http_port}/classic", flush=True)
        print(f"Interim operator dashboard:         http://127.0.0.1:{args.http_port}/", flush=True)
        print("Synthetic Xsens/Bar30/A50 inputs; no physical actuator connected. Ctrl-C to stop.", flush=True)
        counter = 0
        last_depth = 0.0
        last_dvl = 0.0
        try:
            while vehicle.poll() is None and dashboard.poll() is None:
                q, rates, accel = sim.xsens_mtdata2_items()
                imu = (data_item(0x2014, struct.pack(">4f", *q))
                       + data_item(0x8020, struct.pack(">3f", *rates))
                       + data_item(0x4020, struct.pack(">3f", *accel)))
                os.write(xsens_master, packet(0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                now = time.monotonic()
                if now - last_depth >= 0.05:
                    os.write(depth_master, sim.depth_line())
                    last_depth = now
                if now - last_dvl >= 0.1:
                    dvl_sock.sendall((json.dumps(sim.a50_report(time.time_ns() // 1000, 100.0)) + "\n").encode())
                    last_dvl = now
                time.sleep(0.01)
            raise RuntimeError(f"demo process exited: Sub={vehicle.poll()}, dashboard={dashboard.poll()}")
        except KeyboardInterrupt:
            pass
        finally:
            for process in (vehicle, dvl_bridge, dashboard):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            dvl_sock.close()
            tcp.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
