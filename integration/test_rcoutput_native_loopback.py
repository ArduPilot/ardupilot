#!/usr/bin/env python3
"""Observe real ArduSub RCOutput cycles using virtual sensors and localhost UDP."""

import os
import pty
import sys
import signal
import socket
import struct
import subprocess
import tempfile
import time
from pathlib import Path

from test_xsens_native_startup import BINARY, HWDEF, ROOT, data_item, mavutil, packet
from rcoutput_mapping import load_motor_order, motors_in_esp_order, parse_observation


def main():
    active = "--active" in sys.argv[1:]
    motor_order = load_motor_order()
    if not BINARY.is_file() or "#define HAL_ATAL_RCOUT_LOOPBACK_ENABLED 1" not in HWDEF.read_text():
        raise SystemExit("Build with integration/atal_loopback_output.hwdef first")
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 14660))
    receiver.settimeout(0.2)
    with tempfile.TemporaryDirectory(prefix="atal-rcoutput-") as directory:
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "BARO_SPEC_GRAV 1.0\n"
            + ("ARMING_CHECK 0\n" if active else "")
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )
        connection = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        telemetry_port = connection.port.getsockname()[1]
        process = subprocess.Popen(
            [str(BINARY), "--serial0", f"udp:127.0.0.1:{telemetry_port}",
             "--serial1", os.ttyname(xsens_slave),
             "--serial2", os.ttyname(depth_slave), "--defaults", str(defaults),
             "--log-directory", directory, "--storage-directory", directory,
             "--terrain-directory", directory],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        payload = (
            data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
            + data_item(0x8020, struct.pack(">3f", 0.1, 0.2, 0.3))
            + data_item(0x4020, struct.pack(">3f", 1.0, 2.0, -9.81))
        )
        count = 0
        seen = 0
        arm_requested = False
        disarm_requested = False
        try:
            deadline = time.monotonic() + (12 if active else 8)
            while time.monotonic() < deadline and process.poll() is None:
                os.write(xsens_master, packet(0x36, payload + data_item(0x1020, count.to_bytes(2, "big"))))
                os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                message = connection.recv_match(blocking=False)
                if active and seen >= 10 and message is not None and not arm_requested:
                    connection.mav.command_long_send(
                        1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0, 1, 0, 0, 0, 0, 0, 0,
                    )
                    arm_requested = True
                if active and arm_requested and not disarm_requested:
                    connection.mav.manual_control_send(1, 500, 0, 500, 0, 0)
                count = (count + 1) & 0xFFFF
                try:
                    frame = receiver.recv(256)
                except socket.timeout:
                    continue
                armed, values = parse_observation(frame)
                if armed and not active:
                    raise SystemExit(f"Unexpected arming state in RCOutput frame: {frame!r}")
                seen += 1
                if active and armed and not disarm_requested:
                    if (values[0] > 1500 and values[1] > 1500 and
                            values[2] < 1500 and values[3] < 1500):
                        mapped = motors_in_esp_order(values, motor_order)
                        if not (mapped[5] > 1500 and mapped[6] > 1500 and
                                mapped[0] < 1500 and mapped[2] < 1500):
                            raise SystemExit(f"Invalid ESP32 motor permutation: {mapped}")
                        connection.mav.command_long_send(
                            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 0, 0, 0, 0, 0, 0, 0,
                        )
                        disarm_requested = True
                if active and disarm_requested and not armed and values[:8] == (1500,) * 8:
                    print("Observed native mixer response and neutral output after disarm on localhost")
                    return
                if not active and seen >= 10 and any(values[:8]):
                    print(f"Observed {seen} disarmed native RCOutput frames on localhost; motor channels={values[:8]}")
                    return
            raise SystemExit(f"No expected RCOutput stream; frames={seen}, arm_requested={arm_requested}, disarm_requested={disarm_requested}, process={process.poll()}")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            receiver.close()
            connection.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
