#!/usr/bin/env python3
"""End-to-end native ArduSub + DVL + Dashboard HTTP Control verification.

Runs the actual build/linux/bin/ardusub binary with virtual Xsens, Bar30, and
A50 DVL streams, connected to dashboard_api.py --enable-control, and verifies:
  1. EKF3 initializes into AID_RELATIVE (EKF_POS_HORIZ_REL, no EKF_UNINITIALIZED).
  2. Real ArduSub mode transitions via HTTP /api/set_mode (GUIDED, ALT_HOLD, STABILIZE).
  3. Real ArduSub parameter read & write via /api/get_params and /api/set_param.
  4. Real ArduSub MAVLink mission upload & read-back via /api/mission/upload.
  5. Safe disarm / kill via /api/kill_all (no physical actuators attached).
"""

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
import urllib.error
import urllib.request
from pathlib import Path

from test_xsens_native_startup import BINARY, ROOT, data_item, packet


def unused_port(socktype):
    with socket.socket(socket.AF_INET, socktype) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def http_json(url, method="GET", payload=None, timeout=4.0):
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    headers = {"Content-Type": "application/json"} if payload is not None else {}
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.load(resp)
    except urllib.error.HTTPError as err:
        return err.code, json.loads(err.read().decode("utf-8"))


def main():
    if not BINARY.is_file():
        raise SystemExit("Build native Linux ArduSub first")
    with tempfile.TemporaryDirectory(prefix="atal-e2e-") as directory:
        d = Path(directory)
        xm, xs = pty.openpty()
        dm, ds = pty.openpty()
        dash_udp = unused_port(socket.SOCK_DGRAM)
        gcs_udp = unused_port(socket.SOCK_DGRAM)
        http_port = unused_port(socket.SOCK_STREAM)
        dvl_status_udp = unused_port(socket.SOCK_DGRAM)

        tcp = socket.socket()
        tcp.bind(("127.0.0.1", 0))
        tcp.listen(1)
        tcp.settimeout(5)
        cfg = json.loads((ROOT / "integration/vehicle_defaults.json").read_text())
        cfg["dvl_a50"].update(ip="127.0.0.1", tcp_port=tcp.getsockname()[1])
        cfg_path = d / "vehicle.json"
        cfg_path.write_text(json.dumps(cfg))

        defaults = d / "e2e.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SERIAL3_PROTOCOL 2\nSERIAL3_BAUD 115\n"
            "SR3_EXTRA1 10\nSR3_EXTRA2 5\nSR3_RAW_SENS 5\n"
            "SR3_EXT_STAT 2\nSR3_POSITION 10\n"
            + (ROOT / "integration/atal_dvl_ekf_replay.parm").read_text()
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )

        dashboard = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dashboard_api.py"),
             "--mavlink", f"udpin:127.0.0.1:{dash_udp}",
             "--http-port", str(http_port),
             "--dvl-status-udp", f"127.0.0.1:{dvl_status_udp}",
             "--recording-dir", str(d / "rec"),
             "--enable-control"],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        dvl_bridge = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dvl_bridge.py"),
             "--config", str(cfg_path), "--connect-a50",
             "--status-udp", f"127.0.0.1:{dvl_status_udp}"],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        dvl_sock, _ = tcp.accept()
        ardusub = subprocess.Popen(
            [str(BINARY), "--serial0", "udp:127.0.0.1:14551",
             "--serial1", os.ttyname(xs),
             "--serial2", os.ttyname(ds),
             "--serial3", f"udp:127.0.0.1:{dash_udp}",
             "--defaults", str(defaults),
             "--log-directory", str(d),
             "--storage-directory", str(d),
             "--terrain-directory", str(d)],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        imu = (data_item(0x2014, struct.pack(">4f", 1, 0, 0, 0))
               + data_item(0x8020, struct.pack(">3f", 0, 0, 0))
               + data_item(0x4020, struct.pack(">3f", 0, 0, -9.81)))
        counter = 0
        last_depth = 0.0
        last_dvl = 0.0
        base_url = f"http://127.0.0.1:{http_port}"
        ekf_rel_ready = False
        try:
            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline and ardusub.poll() is None and dashboard.poll() is None:
                now = time.monotonic()
                os.write(xm, packet(0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                counter = (counter + 1) & 0xFFFF
                if now - last_depth >= 0.05:
                    os.write(dm, b"DEPTH:1025.50,23.40,1.2500\n")
                    last_depth = now
                if now - last_dvl >= 0.1:
                    rep = {"type": "velocity", "velocity_valid": True, "status": 0,
                           "altitude": 2.0, "fom": 0.04, "time": 100.0,
                           "vx": 0.0, "vy": 0.0, "vz": 0.0,
                           "time_of_validity": time.time_ns() // 1000}
                    dvl_sock.sendall((json.dumps(rep) + "\n").encode())
                    last_dvl = now
                if now - last_dvl < 0.02:
                    try:
                        code, st = http_json(f"{base_url}/api/state", timeout=0.3)
                        if (code == 200 and st.get("link_alive") and st.get("ekf_status_alive")
                                and st.get("ekf_relative_position_ok") and not st.get("ekf_uninitialized")
                                and st.get("dvl_locked") is True):
                            ekf_rel_ready = True
                            break
                    except Exception:
                        pass
                time.sleep(0.01)

            if not ekf_rel_ready:
                raise AssertionError("Native ArduSub EKF3 did not reach relative-position readiness with DVL")

            # Pump virtual sensors in background thread while running HTTP API roundtrips
            stop_sensors = False

            def pump_sensors():
                nonlocal counter, last_depth, last_dvl
                while not stop_sensors:
                    now_s = time.monotonic()
                    os.write(xm, packet(0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                    counter = (counter + 1) & 0xFFFF
                    if now_s - last_depth >= 0.05:
                        os.write(dm, b"DEPTH:1025.50,23.40,1.2500\n")
                        last_depth = now_s
                    if now_s - last_dvl >= 0.1:
                        rep = {"type": "velocity", "velocity_valid": True, "status": 0,
                               "altitude": 2.0, "fom": 0.04, "time": 100.0,
                               "vx": 0.0, "vy": 0.0, "vz": 0.0,
                               "time_of_validity": time.time_ns() // 1000}
                        dvl_sock.sendall((json.dumps(rep) + "\n").encode())
                        last_dvl = now_s
                    time.sleep(0.01)

            import threading
            pump_thread = threading.Thread(target=pump_sensors, daemon=True)
            pump_thread.start()
            try:
                # 1. Mode switches on real ArduSub
                for mode_name in ("GUIDED", "ALT_HOLD", "STABILIZE"):
                    c, r = http_json(f"{base_url}/api/set_mode", "POST", {"mode": mode_name})
                    assert c == 200 and r.get("ok"), f"Mode {mode_name} failed: {c} {r}"
                    time.sleep(0.25)
                    _, st = http_json(f"{base_url}/api/state")
                    assert st["vehicle_mode"] == mode_name, f"Expected {mode_name}, got {st['vehicle_mode']}"

                # 2. Parameter read/write roundtrip on real ArduSub
                c, pr = http_json(f"{base_url}/api/get_params", timeout=8.0)
                assert c == 200 and pr.get("count", 0) > 50, f"Param list failed: {pr}"
                c, sr = http_json(f"{base_url}/api/set_param", "POST", {"param": "WP_SPD", "value": 0.55})
                assert c == 200 and sr.get("ok") and abs(sr["value"] - 0.55) < 0.01, f"Param set failed: {sr}"

                # 3. Mission upload & readback against real ArduSub
                wps = [{"north": 2.0, "east": 1.0, "depth": 1.2, "yaw": 45.0,
                        "max_vel": 0.4, "tolerance": 0.3, "hold_sec": 2.0}]
                c, mr = http_json(f"{base_url}/api/mission/upload", "POST", {"wps": wps}, timeout=8.0)
                assert c == 200 and mr.get("ok") and mr["readback_count"] == 4, f"Mission upload/readback failed: {mr}"

                # 4. Kill / disarm safety check
                c, kr = http_json(f"{base_url}/api/kill_all", "POST", {})
                assert c == 200 and kr.get("ok"), f"Kill failed: {kr}"
                _, st = http_json(f"{base_url}/api/state")
                assert st["armed_known"] and st["armed"] is False

                print("End-to-end native ArduSub + DVL + Dashboard HTTP Control passed: "
                      f"mode/param/mission_readback({mr['readback_count']})/kill verified")
            finally:
                stop_sensors = True
                pump_thread.join(timeout=1.0)
        finally:
            for proc in (ardusub, dvl_bridge, dashboard):
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
            dvl_sock.close()
            tcp.close()
            for fd in (xm, xs, dm, ds):
                os.close(fd)


if __name__ == "__main__":
    main()
