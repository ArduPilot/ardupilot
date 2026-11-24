#!/usr/bin/env python3
"""Closed-loop 6-DoF dynamics test against native ArduSub + RCOutput loopback.

Requires building ArduSub with `integration/atal_loopback_output.hwdef`:
    ./waf configure --board=linux --extra-hwdef integration/atal_loopback_output.hwdef --no-submodule-update
    ./waf sub --no-submodule-update -j4

Verifies STABILIZE attitude recovery, ALT_HOLD depth stabilization, GUIDED
mission waypoint arrival via GuidedMissionRunner, and neutral outputs on disarm.
"""

import json
import os
import pty
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
from pathlib import Path

from pymavlink import mavutil

from ardusub_control import ArduSubCommander
from dashboard_telemetry import DashboardTelemetry
from mission_client import GuidedMissionRunner
from rcoutput_mapping import parse_observation
from test_xsens_native_startup import BINARY, HWDEF, ROOT, data_item, packet
from vehicle_dynamics_sim import UnderwaterDynamicsSim


def _unused_port(socktype):
    with socket.socket(socket.AF_INET, socktype) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    if not BINARY.is_file() or "#define HAL_ATAL_RCOUT_LOOPBACK_ENABLED 1" not in HWDEF.read_text():
        raise SystemExit("Build with integration/atal_loopback_output.hwdef first")

    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rc_sock.bind(("127.0.0.1", 14660))
    rc_sock.setblocking(False)

    sim = UnderwaterDynamicsSim()
    sim.pos_ned[2] = 0.0  # Calibrate surface Baro at 0.0m depth during boot
    sim_lock = threading.Lock()

    with tempfile.TemporaryDirectory(prefix="atal-closed-loop-") as directory:
        d = Path(directory)
        xm, xs = pty.openpty()
        dm, ds = pty.openpty()
        for fd in (xm, xs, dm, ds):
            tty.setraw(fd)
        ctrl_udp = _unused_port(socket.SOCK_DGRAM)

        tcp = socket.socket()
        tcp.bind(("127.0.0.1", 0))
        tcp.listen(1)
        tcp.settimeout(5)
        cfg = json.loads((ROOT / "integration/vehicle_defaults.json").read_text())
        cfg["dvl_a50"].update(ip="127.0.0.1", tcp_port=tcp.getsockname()[1])
        cfg_path = d / "vehicle.json"
        cfg_path.write_text(json.dumps(cfg))

        defaults = d / "closed_loop.parm"
        defaults.write_text(
            "EAHRS_TYPE 12\nEAHRS_RATE 100\nEAHRS_SENSORS 2\n"
            "SERIAL1_PROTOCOL 36\nSERIAL1_BAUD 115\n"
            "SERIAL2_PROTOCOL 51\nSERIAL2_BAUD 115\n"
            "SERIAL3_PROTOCOL 2\nSERIAL3_BAUD 115\n"
            "SR3_EXTRA1 20\nSR3_EXTRA2 10\nSR3_RAW_SENS 10\n"
            "SR3_EXT_STAT 5\nSR3_POSITION 20\n"
            "ARMING_CHECK 0\n"
            "FS_PILOT_INPUT 0\nFS_GCS_ENABLE 0\n"
            + (ROOT / "integration/atal_dvl_ekf_replay.parm").read_text()
            + (ROOT / "integration/atal_motor_defaults.parm").read_text()
        )

        dvl_bridge = subprocess.Popen(
            [sys.executable, str(ROOT / "integration/dvl_bridge.py"),
             "--config", str(cfg_path), "--connect-a50"],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        dvl_sock, _ = tcp.accept()
        ardusub = subprocess.Popen(
            [str(BINARY), "--serial0", "udp:127.0.0.1:14551",
             "--serial1", os.ttyname(xs),
             "--serial2", os.ttyname(ds),
             "--serial3", f"udp:127.0.0.1:{ctrl_udp}",
             "--defaults", str(defaults),
             "--log-directory", str(d),
             "--storage-directory", str(d),
             "--terrain-directory", str(d)],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        link = mavutil.mavlink_connection(f"udpin:127.0.0.1:{ctrl_udp}", autoreconnect=True)
        telemetry = DashboardTelemetry(system_id=1)
        telem_lock = threading.Lock()
        send_lock = threading.Lock()
        commander = ArduSubCommander(link, send_lock=send_lock, system_id=1, allow_arming=True)
        commander.start_watchdog()

        def snapshot():
            with telem_lock:
                return telemetry.snapshot()

        runner = GuidedMissionRunner(commander, snapshot)
        stop_ev = threading.Event()
        latest_rc = {"armed": False, "channels": (1500,) * 8}

        def mav_loop():
            while not stop_ev.is_set():
                msg = link.recv_match(blocking=True, timeout=0.1)
                if msg is not None and msg.get_type() != "BAD_DATA":
                    with telem_lock:
                        telemetry.ingest(msg)
                    commander.handle_message(msg)

        def plant_loop():
            counter = 0
            last_depth = 0.0
            last_dvl = 0.0
            last_step = time.monotonic()
            while not stop_ev.is_set():
                while True:
                    try:
                        pkt, _ = rc_sock.recvfrom(512)
                        armed_obs, ch = parse_observation(pkt)
                        latest_rc["armed"] = armed_obs
                        latest_rc["channels"] = ch[:8]
                    except BlockingIOError:
                        break
                now = time.monotonic()
                dt = min(0.05, max(0.002, now - last_step))
                last_step = now
                with sim_lock:
                    sim.step(latest_rc["channels"], latest_rc["armed"], dt)
                    q, gyro, accel = sim.xsens_mtdata2_items()
                    d_line = sim.depth_line()
                    dvl_rep = sim.a50_report(time.time_ns() // 1000, dt_ms=100.0)
                imu = (data_item(0x2014, struct.pack(">4f", *q))
                       + data_item(0x8020, struct.pack(">3f", *gyro))
                       + data_item(0x4020, struct.pack(">3f", *accel)))
                if select.select([], [xm], [], 0)[1]:
                    os.write(xm, packet(0x36, imu + data_item(0x1020, counter.to_bytes(2, "big"))))
                    counter = (counter + 1) & 0xFFFF
                if now - last_depth >= 0.05 and select.select([], [dm], [], 0)[1]:
                    os.write(dm, d_line)
                    last_depth = now
                if now - last_dvl >= 0.1:
                    dvl_sock.sendall((json.dumps(dvl_rep) + "\n").encode())
                    last_dvl = now
                time.sleep(0.015)

        t_mav = threading.Thread(target=mav_loop, daemon=True)
        t_plant = threading.Thread(target=plant_loop, daemon=True)
        t_mav.start()
        t_plant.start()

        try:
            # 1. Wait for EKF relative readiness
            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline:
                snap = snapshot()
                if (snap.get("link_alive") and snap.get("ekf_relative_position_ok")
                        and not snap.get("ekf_uninitialized")):
                    break
                time.sleep(0.1)
            else:
                raise AssertionError("EKF3 did not become ready in closed-loop sim")

            # 2. Set origin & arm in STABILIZE
            commander.set_ekf_origin(26.512339, 80.232900, 0.0)
            res = commander.set_mode("STABILIZE")
            assert res.get("ok"), f"STABILIZE mode failed: {res}"
            arm = commander.arm(True, force=True)
            assert arm.get("ok"), f"Arm failed: {arm}"
            commander.set_manual_control(0, 0, 500, 0, lease_s=5.0)
            time.sleep(0.3)

            # 3. Attitude disturbance recovery in STABILIZE
            with sim_lock:
                sim.rpy[0] = 0.28
                sim.rpy[1] = -0.22
            t0 = time.monotonic()
            recovered = False
            while time.monotonic() - t0 < 5.0:
                with sim_lock:
                    r_abs, p_abs = abs(sim.rpy[0]), abs(sim.rpy[1])
                if r_abs < 0.08 and p_abs < 0.08:
                    recovered = True
                    break
                commander.set_manual_control(0, 0, 500, 0, lease_s=1.0)
                time.sleep(0.05)
            assert recovered, f"STABILIZE did not damp attitude disturbance: rpy={sim.rpy}"

            # 4. Depth dive + stabilization in ALT_HOLD
            res = commander.set_mode("ALT_HOLD")
            assert res.get("ok"), f"ALT_HOLD failed: {res}"
            # Command downward throttle pulse (z=250 < 500) to submerge deeper
            t_dive = time.monotonic()
            while time.monotonic() - t_dive < 3.2:
                commander.set_manual_control(0, 0, 250, 0, lease_s=0.5)
                time.sleep(0.05)
            # Release throttle to neutral (z=500) and verify ALT_HOLD captures and holds submerged depth
            commander.set_manual_control(0, 0, 500, 0, lease_s=2.0)
            time.sleep(0.6)
            with sim_lock:
                hold_z = sim.pos_ned[2]
            assert hold_z > 0.55, f"Vehicle did not submerge under ALT_HOLD dive command: z={hold_z:.3f}"
            # Apply physical downward heave velocity impulse (keeping Baro/IMU/DVL physically consistent)
            with sim_lock:
                sim.vel_body[2] += 1.2
            t0 = time.monotonic()
            restored = False
            while time.monotonic() - t0 < 5.0:
                commander.set_manual_control(0, 0, 500, 0, lease_s=1.0)
                with sim_lock:
                    err = abs(sim.pos_ned[2] - hold_z)
                if time.monotonic() - t0 > 1.0 and err < 0.25:
                    restored = True
                    break
                time.sleep(0.05)
            assert restored, (
                f"ALT_HOLD did not restore depth after heave disturbance: "
                f"z={sim.pos_ned[2]:.3f} vs hold={hold_z:.3f}"
            )
            commander.release_manual_control()

            # 5. Guided waypoint navigation & arrival
            wp_res = runner.start([{
                "north": 1.1, "east": 0.4, "depth": 0.6,
                "yaw": 15.0, "max_vel": 0.6, "tolerance": 0.45, "hold_sec": 0.3,
            }])
            assert wp_res.get("ok"), f"GuidedMissionRunner start failed: {wp_res}"
            t0 = time.monotonic()
            completed = False
            while time.monotonic() - t0 < 16.0:
                st = runner.status()
                if st["status"] == "completed":
                    completed = True
                    break
                time.sleep(0.1)
            assert completed, f"GUIDED waypoint mission did not complete: {runner.status()} pos={sim.pos_ned}"

            # 6. Disarm and confirm neutral PWMs
            disarm = commander.arm(False, force=True)
            assert disarm.get("ok"), f"Disarm failed: {disarm}"
            neutral_ok = False
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0:
                if not latest_rc["armed"] and latest_rc["channels"] == (1500,) * 8:
                    neutral_ok = True
                    break
                time.sleep(0.05)
            assert neutral_ok, f"Motors did not return to disarmed neutral: {latest_rc}"

            print(
                "Closed-loop native ArduSub 6-DoF dynamics test passed: "
                f"STABILIZE/ALT_HOLD/GUIDED waypoint completed at NED=({sim.pos_ned[0]:.2f}, "
                f"{sim.pos_ned[1]:.2f}, {sim.pos_ned[2]:.2f})"
            )
        finally:
            stop_ev.set()
            runner.close()
            commander.close()
            t_mav.join(timeout=1.0)
            t_plant.join(timeout=1.0)
            link.close()
            for proc in (ardusub, dvl_bridge):
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
            dvl_sock.close()
            tcp.close()
            rc_sock.close()
            for fd in (xm, xs, dm, ds):
                os.close(fd)


if __name__ == "__main__":
    main()
