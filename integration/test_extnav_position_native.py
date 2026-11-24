#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Test-only virtual ExternalNav pose/origin into native ArduSub; no actuators."""

import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from test_xsens_native_startup import BINARY, ROOT, data_item, mavutil, packet  # noqa: E402


def main():
    if not BINARY.is_file():
        raise SystemExit("Build native Linux ArduSub first")
    with tempfile.TemporaryDirectory(prefix="atal-extnav-pos-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SR0_EXTRA2 5\nLOG_DISARMED 1\n"
            "VISO_TYPE 1\nEK3_SRC1_POSXY 6\nEK3_SRC1_VELXY 0\n"
            "EK3_SRC1_POSZ 1\nEK3_SRC1_VELZ 0\nEK3_SRC1_YAW 6\n"
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )
        connection = mavutil.mavlink_connection(
            "udpin:127.0.0.1:0", source_system=200, source_component=191,
            dialect="ardupilotmega",
        )
        port = connection.port.getsockname()[1]
        process = subprocess.Popen(
            [str(BINARY), "--serial0", f"udp:127.0.0.1:{port}",
             "--serial1", os.ttyname(xsens_slave),
             "--serial2", os.ttyname(depth_slave),
             "--defaults", str(defaults), "--log-directory", directory,
             "--storage-directory", directory, "--terrain-directory", directory],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        imu = (data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
               + data_item(0x8020, struct.pack(">3f", 0, 0, 0))
               + data_item(0x4020, struct.pack(">3f", 0, 0, -9.81)))
        flags = set()
        last_flags = None
        saw_origin = False
        guided = False
        heard_native = False
        precheck_sent = False
        guided_before_pose = False
        counter = 0
        last_depth = last_pose = last_origin = last_mode = 0.0
        try:
            started = time.monotonic()
            deadline = started + 18
            while time.monotonic() < deadline and process.poll() is None:
                now = time.monotonic()
                os.write(xsens_master, packet(0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if now - last_depth >= 0.05:
                    os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                    last_depth = now
                message = connection.recv_match(blocking=False)
                if message is not None:
                    if message.get_type() == "EKF_STATUS_REPORT":
                        flags.add(message.flags)
                        last_flags = message.flags
                    elif message.get_type() == "GPS_GLOBAL_ORIGIN":
                        saw_origin = True
                    elif message.get_type() == "HEARTBEAT":
                        heard_native = True
                        if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                            raise AssertionError("Virtual position test unexpectedly armed")
                        if message.custom_mode == 4:
                            guided = True
                            guided_before_pose |= now - started < 3
                if heard_native and now - started >= 1 and not precheck_sent:
                    connection.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                    precheck_sent = True
                if now - started >= 3 and now - last_origin >= 1:
                    connection.mav.set_gps_global_origin_send(1, 1000000, 1000000, 0)
                    last_origin = now
                if now - started >= 3 and now - last_pose >= 0.2:
                    covariance = [0.0] * 21
                    for index in (0, 6, 11):
                        covariance[index] = 0.01
                    for index in (15, 18, 20):
                        covariance[index] = 0.01
                    connection.mav.vision_position_estimate_send(
                        time.time_ns() // 1000, 0, 0, 1.25, 0, 0, 0,
                        covariance=covariance,
                    )
                    last_pose = now
                if saw_origin and any(flag & 16 for flag in flags) and not guided and now - last_mode >= 1:
                    connection.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                    last_mode = now
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
        print(f"Virtual ExternalNav pose/origin replay: origin_report={saw_origin}, "
              f"EKF flags={sorted(flags)}, final_flags={last_flags}, guided={guided}, "
              f"guided_before_pose={guided_before_pose}")
        if not precheck_sent or guided_before_pose:
            raise AssertionError("Disarmed GUIDED precondition check was absent or accepted too early")
        if not saw_origin or not any(flag & 16 for flag in flags) or not guided:
            raise AssertionError("Native ArduSub did not accept synthetic position for disarmed GUIDED")


if __name__ == "__main__":
    main()
