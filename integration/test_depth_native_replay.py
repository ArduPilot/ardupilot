#!/usr/bin/env python3
"""Replay virtual Xsens and ESP32 Bar30 streams into native ArduSub."""

import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

from test_xsens_native_startup import BINARY, HWDEF, ROOT, data_item, mavutil, packet


def main():
    if not BINARY.is_file() or not HWDEF.is_file():
        raise SystemExit("Build the native Linux Sub first")
    if "#define HAL_BARO_ALLOW_INIT_NO_BARO 1" in HWDEF.read_text():
        raise SystemExit("Reconfigure without the no-barometer test hwdef first")

    with tempfile.TemporaryDirectory(prefix="atal-depth-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "BARO_SPEC_GRAV 1.0\n"
        )
        connection = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        telemetry_port = connection.port.getsockname()[1]
        command = [
            str(BINARY),
            "--serial0", f"udp:127.0.0.1:{telemetry_port}",
            "--serial1", os.ttyname(xsens_slave),
            "--serial2", os.ttyname(depth_slave),
            "--defaults", str(defaults),
            "--log-directory", directory,
            "--storage-directory", directory,
            "--terrain-directory", directory,
        ]
        process = subprocess.Popen(
            command, cwd=ROOT, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True,
        )
        imu_payload = (
            data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
            + data_item(0x8020, struct.pack(">3f", 0.1, 0.2, 0.3))
            + data_item(0x4020, struct.pack(">3f", 1.0, 2.0, -9.81))
        )
        counter = 0
        last_depth = 0.0
        raw_imu = False
        scaled_pressure = False
        pressure_health = False
        stale_pressure_detected = False
        valid_phase = True
        pressure_mask = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and process.poll() is None:
                now = time.monotonic()
                os.write(xsens_master, packet(0x36, imu_payload +
                         data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if now - last_depth >= 0.05:
                    last_depth = now
                    depth_line = (b"DEPTH:1025.50,23.40,1.2500\n" if valid_phase
                                  else b"DEPTH:nan,23.40,1.2500\n")
                    os.write(depth_master, depth_line)
                message = connection.recv_match(blocking=False)
                if message is not None:
                    kind = message.get_type()
                    if kind == "RAW_IMU":
                        raw_imu |= abs(message.xgyro - 100) <= 5
                    elif kind == "SCALED_PRESSURE":
                        scaled_pressure |= (
                            abs(message.press_abs - 1025.5) < 0.2
                            and abs(message.temperature - 2340) <= 2
                        )
                    elif kind == "SYS_STATUS":
                        healthy = bool(message.onboard_control_sensors_health & pressure_mask)
                        if valid_phase:
                            pressure_health |= healthy
                        elif not healthy:
                            stale_pressure_detected = True
                if raw_imu and scaled_pressure and pressure_health:
                    valid_phase = False
                if stale_pressure_detected:
                    print("Virtual Xsens and Bar30 telemetry passed; malformed depth went stale")
                    return
                time.sleep(0.01)
            raise SystemExit(
                f"Replay incomplete: imu={raw_imu}, pressure={scaled_pressure}, "
                f"pressure_health={pressure_health}, stale={stale_pressure_detected}, "
                f"process={process.poll()}"
            )
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            connection.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
