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

os.environ.setdefault("MAVLINK20", "1")
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
                                      1, 0, 4)
            sender.mav.attitude_send(0, 0.1, 0.2, 0.3, 0, 0, 0)
            sender.mav.scaled_pressure_send(0, 1025.5, 0, 2340)
            sender.mav.mission_current_send(2, 4, 3, 1)
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/state", timeout=0.5) as response:
                    state = json.load(response)
                if state["link_alive"] and state["imu_alive"] and state["pressure_alive"] and state["mission_alive"]:
                    assert abs(state["pressure_mbar"] - 1025.5) < 0.2
                    assert abs(state["roll"] - 5.72958) < 0.01
                    assert state["vehicle_mode"] == "STABILIZE"
                    assert (state["mission_seq"], state["mission_total"], state["mission_state"]) == (2, 4, "Active")
                    with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/", timeout=0.5) as response:
                        page = response.read().decode("utf-8")
                    assert "AUV-IITK Control" in page and "Motor output / SERVO channels" in page and "Mission Status" in page
                    for asset in ("ros_dashboard.css", "operator_dashboard.css", "operator_dashboard.js", "img/auv_logo.svg"):
                        with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/static/{asset}", timeout=0.5) as response:
                            assert response.status == 200 and response.read()
                    with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/diagnostics", timeout=0.5) as response:
                        assert "Vehicle telemetry" in response.read().decode()
                    with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/stream", timeout=0.5) as response:
                        stream_state = json.loads(response.readline().removeprefix(b"data: "))
                    assert stream_state["link_alive"] and stream_state["imu_alive"]
                    print("Dashboard view, API and SSE + MAVLink loopback passed")
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
