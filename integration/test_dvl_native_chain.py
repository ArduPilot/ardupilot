#!/usr/bin/env python3
"""Replay a loopback A50 stream through the bridge into native ArduSub EKF."""

import json
import os
import pty
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from test_xsens_native_startup import BINARY, ROOT, data_item, mavutil, packet


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    if not BINARY.is_file():
        raise SystemExit("Build native Linux ArduSub first")
    with tempfile.TemporaryDirectory(prefix="atal-dvl-chain-") as directory:
        tcp = socket.socket()
        tcp.bind(("127.0.0.1", 0))
        tcp.listen(1)
        tcp.settimeout(4)
        config = json.loads((ROOT / "integration/vehicle_defaults.json").read_text())
        config["dvl_a50"].update(ip="127.0.0.1", tcp_port=tcp.getsockname()[1])
        config_path = Path(directory) / "vehicle.json"
        config_path.write_text(json.dumps(config))
        defaults = Path(directory) / "defaults.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SERIAL3_PROTOCOL 2\nSR3_EXTRA2 5\n"
            "LOG_DISARMED 1\n"
            + (ROOT / "integration/atal_dvl_ekf_replay.parm").read_text()
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )
        xsens_master, xsens_slave = pty.openpty()
        depth_master, depth_slave = pty.openpty()
        status_link = mavutil.mavlink_connection("udpin:127.0.0.1:0")
        status_port = status_link.port.getsockname()[1]
        bridge = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dvl_bridge.py"),
             "--config", str(config_path), "--connect-a50"],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        native = None
        dvl = None
        ekf_flags = None
        try:
            dvl, _ = tcp.accept()
            native = subprocess.Popen(
                [str(BINARY), "--serial0", "udp:127.0.0.1:14551",
                 "--serial1", os.ttyname(xsens_slave),
                 "--serial2", os.ttyname(depth_slave),
                 "--serial3", f"udp:127.0.0.1:{status_port}",
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
            last_report = 0.0
            started = time.monotonic()
            deadline = started + 15
            while time.monotonic() < deadline:
                if bridge.poll() is not None or native.poll() is not None:
                    raise AssertionError(f"Process exited: bridge={bridge.poll()}, native={native.poll()}")
                now = time.monotonic()
                os.write(xsens_master, packet(
                    0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if now - last_depth >= 0.05:
                    os.write(depth_master, b"DEPTH:1025.50,23.40,1.2500\n")
                    last_depth = now
                if now - last_report >= 0.2:
                    bottom_lock = not 7 <= now - started < 10
                    report = {"type": "velocity", "velocity_valid": bottom_lock,
                              "status": 0, "altitude": 2.0, "fom": 0.1,
                              "vx": 0.4, "vy": -0.2, "vz": 0.1,
                              "time_of_validity": time.time_ns() // 1000}
                    dvl.sendall((json.dumps(report) + "\n").encode())
                    last_report = now
                message = status_link.recv_match(blocking=False)
                if message is not None and message.get_type() == "EKF_STATUS_REPORT":
                    ekf_flags = message.flags
                time.sleep(0.01)
        finally:
            if native is not None:
                stop(native)
            stop(bridge)
            if dvl is not None:
                dvl.close()
            tcp.close()
            status_link.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)
            bridge_error = bridge.stderr.read().decode(errors="replace")
            bridge.stderr.close()

        log_files = list(Path(directory).rglob("*.BIN"))
        if not log_files:
            raise AssertionError(f"No native log; bridge stderr: {bridge_error}")
        accepted_times = []
        ekf_velocity = None
        for log_file in log_files:
            log = mavutil.mavlink_connection(str(log_file))
            while message := log.recv_match(type=["VISV", "XKF1"]):
                if message.get_type() == "VISV" and message.Ign == 0:
                    if abs(message.VX - 0.4) < 0.05 and abs(message.VY + 0.2) < 0.05:
                        accepted_times.append(message.TimeUS)
                elif message.get_type() == "XKF1":
                    ekf_velocity = (message.VN, message.VE)
            log.close()
        gap_s = max(((b - a) / 1e6 for a, b in zip(accepted_times, accepted_times[1:])),
                    default=0.0)
        if len(accepted_times) < 6 or gap_s < 2.5 or ekf_velocity is None or ekf_velocity[0] < 0.2:
            raise AssertionError(f"Chain incomplete: accepted={len(accepted_times)}, "
                                 f"lock-loss gap={gap_s:.2f}s, EKF={ekf_velocity}, "
                                 f"bridge stderr={bridge_error}")
        if ekf_flags is None or ekf_flags & 16:
            raise AssertionError(f"Unexpected absolute-position status for velocity-only DVL replay: {ekf_flags}")
        print(f"Loopback A50 → bridge → native ArduSub EKF passed: "
              f"accepted={len(accepted_times)}, lock-loss gap={gap_s:.2f}s, "
              f"EKF velocity={ekf_velocity}, status flags={ekf_flags}")


if __name__ == "__main__":
    main()
