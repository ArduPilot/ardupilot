#!/usr/bin/env python3
"""Prove native ArduSub telemetry reaches dashboard HTTP, SSE and local CSV."""

import csv
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
import urllib.error
import urllib.request
from pathlib import Path

from test_xsens_native_startup import BINARY, ROOT, data_item, mavutil, packet


def unused_port(socktype):
    with socket.socket(socket.AF_INET, socktype) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    if not BINARY.is_file():
        raise SystemExit("Build the normal native Linux Sub first")
    with tempfile.TemporaryDirectory(prefix="atal-dashboard-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        dashboard_port = unused_port(socket.SOCK_DGRAM)
        http_port = unused_port(socket.SOCK_STREAM)
        recording_dir = Path(directory) / "recordings"
        dashboard = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dashboard_api.py"),
             "--mavlink", f"udpin:127.0.0.1:{dashboard_port}",
             "--http-port", str(http_port), "--recording-dir", str(recording_dir)],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        gcs_link = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        gcs_port = gcs_link.port.getsockname()[1]
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SERIAL3_PROTOCOL 2\nSERIAL3_BAUD 115\n"
            "SR3_EXTRA1 10\nSR3_EXTRA2 5\nSR3_RAW_SENS 5\nSR3_EXT_STAT 2\n"
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )
        process = subprocess.Popen(
            [str(BINARY), "--serial0", f"udp:127.0.0.1:{gcs_port}",
             "--serial1", os.ttyname(xsens_slave),
             "--serial2", os.ttyname(depth_slave),
             "--serial3", f"udp:127.0.0.1:{dashboard_port}",
             "--defaults", str(defaults),
             "--log-directory", directory,
             "--storage-directory", directory,
             "--terrain-directory", directory],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        imu = (data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
               + data_item(0x8020, struct.pack(">3f", 0.1, 0.2, 0.3))
               + data_item(0x4020, struct.pack(">3f", 1, 2, -9.81)))
        counter = 0
        last_depth = 0.0
        state = None
        recording_started = False
        live_since = None
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and process.poll() is None and dashboard.poll() is None:
                now = time.monotonic()
                os.write(xsens_master, packet(0x36, imu +
                         data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if now - last_depth > 0.05:
                    os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                    last_depth = now
                try:
                    with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/state", timeout=0.2) as response:
                        state = json.load(response)
                    if not recording_started:
                        request = urllib.request.Request(
                            f"http://127.0.0.1:{http_port}/api/recording/start",
                            method="POST", headers={"X-ATAL-Local-Action": "recording"},
                        )
                        with urllib.request.urlopen(request, timeout=0.2) as response:
                            assert json.load(response)["active"]
                        recording_started = True
                    if (state["link_alive"] and state["imu_alive"] and state["pressure_alive"]
                            and state["mission_alive"] and state["battery_alive"]):
                        if live_since is None:
                            live_since = time.monotonic()
                        if time.monotonic() - live_since < 0.3:
                            time.sleep(0.01)
                            continue
                        assert abs(state["pressure_mbar"] - 1025.5) < 0.2
                        assert state["mission_state"] == "No mission"
                        with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/stream", timeout=0.2) as response:
                            event = json.loads(response.readline().removeprefix(b"data: "))
                        assert event["link_alive"] and event["imu_alive"]
                        with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/", timeout=0.2) as response:
                            assert b"AUV-IITK Control" in response.read()
                        request = urllib.request.Request(
                            f"http://127.0.0.1:{http_port}/api/recording/stop",
                            method="POST", headers={"X-ATAL-Local-Action": "recording"},
                        )
                        with urllib.request.urlopen(request, timeout=0.2) as response:
                            recording = json.load(response)
                        with (recording_dir / recording["file"]).open(newline="", encoding="utf-8") as file:
                            rows = list(csv.DictReader(file))
                        if any(row["link_alive"] == "True" and row["imu_alive"] == "True" and
                               row["battery_alive"] == "True" and
                               row["mission_alive"] == "True" and row["mission_state"] == "No mission" and
                               abs(float(row["pressure_mbar"]) - 1025.5) < 0.2
                               for row in rows if row["pressure_mbar"]):
                            print("Native ArduSub serial3 → dashboard HTTP, SSE and CSV passed")
                            return
                        raise AssertionError(f"Recorded CSV lacks live native telemetry: {recording}")
                except (urllib.error.URLError, TimeoutError):
                    pass
                time.sleep(0.01)
            raise AssertionError(f"Dashboard HTTP telemetry incomplete: {state}; process={process.poll()}, api={dashboard.poll()}")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            if dashboard.poll() is None:
                os.killpg(dashboard.pid, signal.SIGTERM)
                dashboard.wait(timeout=2)
            gcs_link.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
