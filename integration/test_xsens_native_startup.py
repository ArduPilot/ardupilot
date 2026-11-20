#!/usr/bin/env python3
"""Software-only Xsens startup check using a pseudo-terminal, never a device."""

import os
import pty
import selectors
import signal
import subprocess
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "build/linux/bin/ardusub"
HWDEF = ROOT / "build/linux/hwdef.h"


def packet(mid, payload=b""):
    header = bytes((0xFA, 0xFF, mid, len(payload)))
    checksum = -(sum(header[1:]) + sum(payload)) & 0xFF
    return header + payload + bytes((checksum,))


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
        master, slave = pty.openpty()
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
        )
        command = [
            str(BINARY),
            "--serial0", "udp:127.0.0.1:14551",
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
                    return
            seen = [item in received for item in expected]
            raise SystemExit(
                f"Xsens startup incomplete: config/output/measurement={seen}; "
                f"received={len(received)} bytes; process={process.poll()}"
            )
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            selector.close()
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    main()
