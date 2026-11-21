#!/usr/bin/env python3
"""Prove a second native ArduSub MAVLink link feeds read-only dashboard state."""

import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

from dashboard_telemetry import DashboardTelemetry
from test_xsens_native_startup import BINARY, ROOT, data_item, mavutil, packet


def main():
    if not BINARY.is_file():
        raise SystemExit("Build the normal native Linux Sub first")
    with tempfile.TemporaryDirectory(prefix="atal-dashboard-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        dashboard_link = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        dashboard_port = dashboard_link.port.getsockname()[1]
        gcs_link = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        gcs_port = gcs_link.port.getsockname()[1]
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SERIAL3_PROTOCOL 2\nSERIAL3_BAUD 115\n"
            "SR3_EXTRA1 10\nSR3_EXTRA2 5\nSR3_RAW_SENS 5\n"
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
        telemetry = DashboardTelemetry()
        imu = (data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
               + data_item(0x8020, struct.pack(">3f", 0.1, 0.2, 0.3))
               + data_item(0x4020, struct.pack(">3f", 1, 2, -9.81)))
        counter = 0
        last_depth = 0.0
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and process.poll() is None:
                now = time.monotonic()
                os.write(xsens_master, packet(0x36, imu +
                         data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if now - last_depth > 0.05:
                    os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                    last_depth = now
                while (msg := dashboard_link.recv_match(blocking=False)) is not None:
                    telemetry.ingest(msg)
                state = telemetry.snapshot()
                if state["link_alive"] and state["imu_alive"] and state["pressure_alive"]:
                    assert abs(state["pressure_mbar"] - 1025.5) < 0.2
                    print("Native ArduSub serial3 → dashboard telemetry passed")
                    return
                time.sleep(0.01)
            raise AssertionError(f"Dashboard telemetry incomplete: {telemetry.snapshot()}; process={process.poll()}")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            dashboard_link.close()
            gcs_link.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
