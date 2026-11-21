#!/usr/bin/env python3
"""Exercise the dashboard API process with synthetic MAVLink over loopback."""

import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

from pymavlink import mavutil


def unused_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    mavlink_port = unused_port()
    http_port = unused_port()
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).with_name("dashboard_api.py")),
         "--mavlink", f"udpin:127.0.0.1:{mavlink_port}",
         "--http-port", str(http_port)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    sender = mavutil.mavlink_connection(f"udpout:127.0.0.1:{mavlink_port}",
                                       source_system=1, source_component=1)
    try:
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(f"dashboard API exited: {process.returncode}")
            sender.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_SUBMARINE,
                                      mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                      0, 0, 4)
            sender.mav.attitude_send(0, 0.1, 0.2, 0.3, 0, 0, 0)
            sender.mav.scaled_pressure_send(0, 1025.5, 0, 2340)
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/state", timeout=0.5) as response:
                    state = json.load(response)
                if state["link_alive"] and state["imu_alive"] and state["pressure_alive"]:
                    assert abs(state["pressure_mbar"] - 1025.5) < 0.2
                    assert abs(state["roll"] - 5.72958) < 0.01
                    with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/", timeout=0.5) as response:
                        page = response.read().decode("utf-8")
                    assert "Vehicle telemetry" in page and "Motor output / SERVO channels" in page
                    print("Dashboard view and API HTTP + MAVLink loopback passed")
                    return
            except (urllib.error.URLError, TimeoutError):
                pass
            time.sleep(0.1)
        raise AssertionError("dashboard API did not receive loopback telemetry")
    finally:
        sender.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2)


if __name__ == "__main__":
    main()
