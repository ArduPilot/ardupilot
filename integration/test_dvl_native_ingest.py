#!/usr/bin/env python3
"""Check that native ArduSub logs synthetic DVL velocity via VisualOdom."""

import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

from test_xsens_native_startup import BINARY, ROOT, data_item, mavutil, packet


def run_replay(velocity_source):
    if not BINARY.is_file():
        raise SystemExit("Build native Linux ArduSub first")
    with tempfile.TemporaryDirectory(prefix="atal-dvl-native-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        defaults = Path(directory) / "defaults.parm"
        replay_params = (ROOT / "integration/atal_dvl_ekf_replay.parm").read_text()
        replay_params = replay_params.replace("EK3_SRC1_VELXY 6", f"EK3_SRC1_VELXY {velocity_source}")
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "LOG_DISARMED 1\n"
            + replay_params
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )
        connection = mavutil.mavlink_connection(
            "udpin:127.0.0.1:0", source_system=200, source_component=191
        )
        port = connection.port.getsockname()[1]
        process = subprocess.Popen(
            [str(BINARY), "--serial0", f"udp:127.0.0.1:{port}",
             "--serial1", os.ttyname(xsens_slave),
             "--serial2", os.ttyname(depth_slave),
             "--defaults", str(defaults),
             "--log-directory", directory,
             "--storage-directory", directory,
             "--terrain-directory", directory],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        imu = (data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
               + data_item(0x8020, struct.pack(">3f", 0, 0, 0))
               + data_item(0x4020, struct.pack(">3f", 0, 0, -9.81)))
        counter = 0
        last_depth = 0.0
        last_velocity = 0.0
        telemetry = False
        try:
            deadline = time.monotonic() + 9
            while time.monotonic() < deadline and process.poll() is None:
                now = time.monotonic()
                os.write(xsens_master, packet(
                    0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if now - last_depth >= 0.05:
                    os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                    last_depth = now
                message = connection.recv_match(blocking=False)
                telemetry |= message is not None and message.get_type() == "HEARTBEAT"
                if telemetry and now - last_velocity >= 0.2:
                    last_velocity = now
                    variance = 0.1 ** 2 / 3
                    connection.mav.vision_speed_estimate_send(
                        time.time_ns() // 1000, 0.4, -0.2, 0.1,
                        covariance=(variance, 0, 0, 0, variance, 0, 0, 0, variance),
                    )
                time.sleep(0.01)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            connection.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)

        if not telemetry:
            raise AssertionError("No native MAVLink heartbeat; DVL injection did not start")
        log_files = list(Path(directory).rglob("*.BIN"))
        if not log_files:
            raise AssertionError("Native ArduSub produced no binary log")
        found = False
        ekf_velocities = []
        for log_file in log_files:
            log = mavutil.mavlink_connection(str(log_file))
            while message := log.recv_match(type=["VISV", "XKF1"]):
                if message.get_type() == "XKF1":
                    ekf_velocities.append((message.VN, message.VE, message.VD))
                elif (abs(message.VX - 0.4) < 0.01 and
                        abs(message.VY + 0.2) < 0.01 and
                        abs(message.VZ - 0.1) < 0.01 and
                        abs(message.VErr - 0.1) < 0.01 and
                        message.Ign == 0 and message.Q == 0):
                    found = True
            log.close()
        if not found:
            raise AssertionError("Native VISV did not contain the expected velocity, error and acceptance")
        if not ekf_velocities:
            raise AssertionError("Native EKF produced no XKF1 state")
        return ekf_velocities[-1]


def main():
    control = run_replay(0)
    external = run_replay(6)
    if abs(control[0]) > 0.1 or external[0] < 0.2 or external[0] - control[0] < 0.2:
        raise AssertionError(f"DVL velocity did not affect native EKF as expected: "
                             f"control={control}, external={external}")
    print(f"Native DVL ingest and EKF velocity-source comparison passed: "
          f"control={control}, external={external}")


if __name__ == "__main__":
    main()
