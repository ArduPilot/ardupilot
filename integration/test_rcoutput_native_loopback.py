#!/usr/bin/env python3
"""Observe real ArduSub RCOutput cycles using virtual sensors and localhost UDP."""

import os
import pty
import signal
import socket
import struct
import subprocess
import tempfile
import time
from pathlib import Path

from test_xsens_native_startup import BINARY, HWDEF, ROOT, data_item, packet


def main():
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
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )
        process = subprocess.Popen(
            [str(BINARY), "--serial1", os.ttyname(xsens_slave),
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
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and process.poll() is None:
                os.write(xsens_master, packet(0x36, payload + data_item(0x1020, count.to_bytes(2, "big"))))
                os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                count = (count + 1) & 0xFFFF
                try:
                    frame = receiver.recv(256).decode("ascii")
                except socket.timeout:
                    continue
                fields = frame.split(",")
                if len(fields) != 19 or fields[:3] != ["ATAL_RCOUT", "1", "0"]:
                    raise SystemExit(f"Invalid or armed RCOutput frame: {frame!r}")
                values = [int(value) for value in fields[3:]]
                if any(value < 0 or value > 2100 for value in values):
                    raise SystemExit(f"Invalid RCOutput values: {values}")
                seen += 1
                if seen >= 10 and any(values[:8]):
                    print(f"Observed {seen} disarmed native RCOutput frames on localhost; motor channels={values[:8]}")
                    return
            raise SystemExit(f"No populated RCOutput stream; frames={seen}, process={process.poll()}")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            receiver.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
