#!/usr/bin/env python3
"""Observe real ArduSub RCOutput cycles using virtual sensors and localhost UDP."""

import os
import pty
import json
import select
import sys
import signal
import socket
import struct
import subprocess
import tempfile
import time
import tty
from pathlib import Path

from test_xsens_native_startup import BINARY, HWDEF, ROOT, data_item, mavutil, packet
from rcoutput_mapping import load_motor_order, motors_in_esp_order, parse_observation
from actuator_adapter import ActuatorAdapter


def check_guard_sim(mapped, directory, arm_frame, pwm_frame, disarm_frame):
    """Feed observed native mixer values to the isolated C++ guard."""
    executable = Path(directory) / "actuator_guard_sim"
    subprocess.run(
        ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "integration"),
         str(ROOT / "integration/actuator_guard_sim.cpp"), "-o", str(executable)],
        check=True, capture_output=True, text=True,
    )
    commands = [(0, arm_frame), (10, pwm_frame), (20, disarm_frame)]
    run = subprocess.run(
        [str(executable)], input="".join(f"{when} {frame}\n" for when, frame in commands),
        check=True, capture_output=True, text=True,
    )
    rows = [line.split() for line in run.stdout.splitlines()]
    expected_states = ["armed", "armed", "disarmed"]
    if (len(rows) != len(commands) or [row[1] for row in rows] != expected_states or
            tuple(map(int, rows[1][2:])) != mapped or
            tuple(map(int, rows[2][2:])) != (1500,) * 8):
        raise SystemExit(f"Native output failed guard simulation: {run.stdout}")


def main():
    active = "--active" in sys.argv[1:]
    guard_sim = "--guard-sim" in sys.argv[1:]
    serial_pty = "--serial-pty" in sys.argv[1:]
    if guard_sim and not active:
        raise SystemExit("--guard-sim requires --active")
    if serial_pty and not active:
        raise SystemExit("--serial-pty requires --active")
    motor_order = load_motor_order()
    adapter = ActuatorAdapter(motor_order, 0x1234ABCD)
    if not BINARY.is_file() or "#define HAL_ATAL_RCOUT_LOOPBACK_ENABLED 1" not in HWDEF.read_text():
        raise SystemExit("Build with integration/atal_loopback_output.hwdef first")
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 14660))
    receiver.settimeout(0.2)
    with tempfile.TemporaryDirectory(prefix="atal-rcoutput-") as directory:
        bridge = None
        bridge_master = bridge_slave = None
        bridge_receiver = None
        bridge_output = b""
        if serial_pty:
            bridge_master, bridge_slave = pty.openpty()
            tty.setraw(bridge_master)
            bridge_config = Path(directory) / "bridge.json"
            bridge_config.write_text(json.dumps({"actuators": {"enabled": True}}))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(("127.0.0.1", 0))
                bridge_port = probe.getsockname()[1]
            bridge = subprocess.Popen([
                sys.executable, str(ROOT / "integration/actuator_serial_bridge.py"),
                "--device", os.ttyname(bridge_slave), "--port", str(bridge_port),
                "--config", str(bridge_config), "--enable-hardware",
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            bridge_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            request = b""
            handshake_deadline = time.monotonic() + 3
            while b"\n" not in request and time.monotonic() < handshake_deadline:
                if select.select([bridge_master], [], [], 0.1)[0]:
                    request += os.read(bridge_master, 4096)
            if not request.startswith(b"A1,HELLO,"):
                bridge.terminate()
                bridge.wait(timeout=2)
                bridge_receiver.close()
                os.close(bridge_master)
                os.close(bridge_slave)
                receiver.close()
                raise SystemExit(f"No ESP32 HELLO challenge: {request!r}")
            challenge = request.split(b",", 2)[2].split(b"\n", 1)[0]
            os.write(bridge_master, b"A1,SESSION,1234ABCD," + challenge + b"\n")
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
        forward_seen_ms = None
        observed_mapped = None
        arm_frame = None
        target_pwm_frame = None
        disarm_frame = None
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
                if serial_pty:
                    bridge_receiver.sendto(frame, ("127.0.0.1", bridge_port))
                    for _ in range(64):
                        if not select.select([bridge_master], [], [], 0)[0]:
                            break
                        bridge_output += os.read(bridge_master, 4096)
                adapter_frames = adapter.ingest(frame, time.monotonic_ns() // 1_000_000)
                for command in adapter_frames:
                    if ",ARM," in command and arm_frame is None:
                        arm_frame = command
                    elif ",DISARM," in command:
                        disarm_frame = command
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
                        observed_mapped = mapped
                        target_pwm_frame = next((command for command in adapter_frames if ",PWM," in command), None)
                        if forward_seen_ms is None:
                            forward_seen_ms = time.monotonic()
                        if serial_pty and time.monotonic() - forward_seen_ms < 0.1:
                            continue
                        connection.mav.command_long_send(
                            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 0, 0, 0, 0, 0, 0, 0,
                        )
                        disarm_requested = True
                if active and disarm_requested and not armed and values[:8] == (1500,) * 8:
                    if serial_pty:
                        serial_deadline = time.monotonic() + 1
                        while (not any(b",DISARM," in line for line in bridge_output.split(b"\n")[:-1])
                               and time.monotonic() < serial_deadline):
                            if select.select([bridge_master], [], [], 0.05)[0]:
                                bridge_output += os.read(bridge_master, 4096)
                        commands = [line.decode("ascii") for line in bridge_output.split(b"\n")[:-1]]
                        arm = next((line for line in commands if ",ARM," in line), None)
                        pwm = next((line for line in commands if ",PWM," in line and
                                    len(line.split(",")) == 13 and
                                    int(line.split(",")[9]) > 1500 and
                                    int(line.split(",")[10]) > 1500 and
                                    int(line.split(",")[4]) < 1500 and
                                    int(line.split(",")[6]) < 1500), None)
                        disarm = next((line for line in commands if ",DISARM," in line), None)
                        if not (arm and pwm and disarm):
                            raise SystemExit(f"Native serial bridge missed ARM/PWM/DISARM: {commands!r}")
                        if guard_sim:
                            check_guard_sim(tuple(map(int, pwm.split(",")[4:12])), directory,
                                            arm, pwm, disarm)
                    if guard_sim:
                        if not (arm_frame and target_pwm_frame and disarm_frame):
                            raise SystemExit("Native output did not traverse adapter ARM/PWM/DISARM states")
                        check_guard_sim(observed_mapped, directory, arm_frame, target_pwm_frame, disarm_frame)
                    print("Observed native mixer response and neutral output after disarm on localhost")
                    if guard_sim:
                        print("Observed mixer output passed isolated guard simulation")
                    if serial_pty:
                        print("Observed native mixer ARM/PWM/DISARM through guarded serial bridge PTY")
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
            if bridge is not None:
                bridge.terminate()
                try:
                    bridge.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    bridge.kill()
                    bridge.wait()
                bridge_receiver.close()
                os.close(bridge_master)
                os.close(bridge_slave)
            connection.close()
            for fd in (xsens_master, xsens_slave, depth_master, depth_slave):
                os.close(fd)


if __name__ == "__main__":
    main()
