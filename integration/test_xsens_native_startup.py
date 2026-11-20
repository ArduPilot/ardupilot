#!/usr/bin/env python3
"""Software-only Xsens startup check using a pseudo-terminal, never a device."""

import os
import pty
import selectors
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "build/linux/bin/ardusub"
HWDEF = ROOT / "build/linux/hwdef.h"
sys.path.insert(0, str(ROOT / "modules/mavlink"))
from pymavlink import mavutil  # noqa: E402


def packet(mid, payload=b""):
    header = bytes((0xFA, 0xFF, mid, len(payload)))
    checksum = -(sum(header[1:]) + sum(payload)) & 0xFF
    return header + payload + bytes((checksum,))


def data_item(data_id, value):
    return data_id.to_bytes(2, "big") + bytes((len(value),)) + value


def replay_imu(master, connection):
    payload = (
        data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
        + data_item(0x8020, struct.pack(">3f", 0.1, 0.2, 0.3))
        + data_item(0x4020, struct.pack(">3f", 1.0, 2.0, -9.81))
    )
    deadline = time.monotonic() + 4
    counter = 0
    while time.monotonic() < deadline:
        sample = data_item(0x1020, counter.to_bytes(2, "big"))
        os.write(master, packet(0x36, payload + sample))
        counter += 1
        message = connection.recv_match(blocking=False)
        if message is not None and message.get_type() == "RAW_IMU":
            values = (message.xacc, message.yacc, message.zacc,
                      message.xgyro, message.ygyro, message.zgyro)
            expected = (102, 204, -1000, 100, 200, 300)
            if all(abs(actual - target) <= 5 for actual, target in zip(values, expected)):
                print("Synthetic Xsens sample observed in RAW_IMU telemetry")
                return
        time.sleep(0.01)
    raise SystemExit("Xsens RAW_IMU telemetry did not match the injected sample")


def main():
    if not BINARY.is_file() or not HWDEF.is_file():
        raise SystemExit("Build the native Linux Sub with the test hwdef first")
    if "#define HAL_BARO_ALLOW_INIT_NO_BARO 1" not in HWDEF.read_text():
        raise SystemExit("This check requires integration/atal_linux_test.hwdef")

    output = bytes.fromhex(
        "20 14 00 64 80 20 00 64 40 20 00 64 10 20 00 64"
    )
    expected = (
        packet(0x30),
        packet(0xC0, output),
        packet(0x10),
    )
    with tempfile.TemporaryDirectory(prefix="atal-xsens-") as directory:
        connection = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        telemetry_port = connection.port.getsockname()[1]
        master, slave = pty.openpty()
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
        )
        command = [
            str(BINARY),
            "--serial0", f"udp:127.0.0.1:{telemetry_port}",
            "--serial1", os.ttyname(slave),
            "--defaults", str(defaults),
            "--log-directory", directory,
            "--storage-directory", directory,
            "--terrain-directory", directory,
        ]
        process = subprocess.Popen(
            command, cwd=ROOT, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True,
        )
        received = bytearray()
        selector = selectors.DefaultSelector()
        selector.register(master, selectors.EVENT_READ)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and process.poll() is None:
                for _key, _events in selector.select(timeout=0.2):
                    received.extend(os.read(master, 4096))
                if all(item in received for item in expected):
                    print("Xsens configuration commands observed on virtual serial")
                    break
            else:
                seen = [item in received for item in expected]
                raise SystemExit(
                    f"Xsens startup incomplete: config/output/measurement={seen}; "
                    f"received={len(received)} bytes; process={process.poll()}"
                )
            replay_imu(master, connection)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            selector.close()
            connection.close()
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    main()
