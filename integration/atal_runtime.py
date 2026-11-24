"""Reproducible, single-owner ArduSub runtime supervisor and port ownership validator.

Replaces the ROS 2 control launch (`control_system.launch.py`) without starting
competing attitude controllers, motor mixers, or legacy unguarded ESP32 bridges.
"""

import argparse
import glob
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]
NATIVE_BINARY = ROOT / "build/linux/bin/ardusub"
COMPETING_ROS_NODES = (
    "attitude_controller_node",
    "motor_mixer_node",
    "esp32_bridge_node",
    "position_controller_node",
    "xsens_mti_driver_node",
)


def list_serial_ports():
    ports = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*"):
        for item in sorted(glob.glob(pattern)):
            ports.append({"device": item, "realpath": os.path.realpath(item)})
    return ports


def find_competing_ros_processes():
    """Return active legacy ROS 2 low-level controller processes that conflict with ArduSub."""
    found = []
    try:
        out = subprocess.check_output(["ps", "-eo", "pid,args"], text=True, timeout=2)
    except Exception:
        return found
    for line in out.splitlines()[1:]:
        parts = line.strip().split(None, 1)
        if len(parts) != 2:
            continue
        pid, cmd = parts
        if any(node in cmd for node in COMPETING_ROS_NODES):
            found.append({"pid": int(pid), "cmd": cmd})
    return found


def validate_serial_ownership(devices):
    """Ensure every configured real serial device resolves to a distinct tty."""
    seen = {}
    for role, dev in devices.items():
        if not dev or dev in ("auto", "virtual", "disabled"):
            continue
        real = os.path.realpath(dev)
        if real in seen:
            raise ValueError(
                f"Serial port ownership conflict: {role} ({dev}) and "
                f"{seen[real]} both resolve to {real}"
            )
        seen[real] = role
    return seen


def build_runtime_parm(include_dvl=True, include_motor=True,
                       serial3_udp_rate=10, log_disarmed=False):
    parts = [
        (ROOT / "integration/atal_sensor_defaults.parm").read_text(encoding="utf-8"),
        f"SERIAL3_PROTOCOL 2\nSERIAL3_BAUD 115\n"
        f"SR3_EXTRA1 {int(serial3_udp_rate)}\nSR3_EXTRA2 5\n"
        f"SR3_RAW_SENS 5\nSR3_EXT_STAT 2\n"
        f"SR3_POSITION 10\n",
    ]
    if include_dvl:
        parts.append(
            "VISO_TYPE 1\n"
            "EK3_SRC1_POSXY 0\nEK3_SRC1_VELXY 6\n"
            "EK3_SRC1_POSZ 1\nEK3_SRC1_VELZ 0\nEK3_SRC1_YAW 0\n"
        )
    if include_motor:
        parts.append((ROOT / "integration/atal_motor_defaults.parm").read_text(encoding="utf-8"))
    if log_disarmed:
        parts.append("LOG_DISARMED 1\n")
    return "\n".join(parts)


class RuntimeSupervisor:
    """Manages native ArduSub, DVL bridge, and dashboard processes safely."""

    def __init__(self, config_path=None):
        self.config_path = Path(config_path) if config_path else (ROOT / "integration/vehicle_defaults.json")
        self.processes = {}
        self.mode = "stopped"
        self.last_error = ""
        self._temp_dir = None

    def status(self):
        running = {}
        for name, proc in list(self.processes.items()):
            alive = proc is not None and proc.poll() is None
            running[name] = {
                "running": alive,
                "pid": proc.pid if proc else None,
                "returncode": proc.poll() if proc else None,
            }
        return {
            "mode": self.mode,
            "binary_present": NATIVE_BINARY.is_file(),
            "competing_ros_processes": find_competing_ros_processes(),
            "processes": running,
            "last_error": self.last_error,
        }

    def start_native_sensors(self, xsens_dev, depth_dev, mavlink_gcs="udp:127.0.0.1:14551",
                             dashboard_udp="udp:127.0.0.1:14650", connect_a50=False,
                             enable_hardware_actuators=False, actuator_dev=""):
        if not NATIVE_BINARY.is_file():
            raise FileNotFoundError(f"Missing native ArduSub binary: {NATIVE_BINARY}")
        competing = find_competing_ros_processes()
        if competing:
            raise RuntimeError(f"Competing legacy ROS 2 controllers active: {competing}")
        validate_serial_ownership({
            "xsens": xsens_dev, "depth": depth_dev, "actuator": actuator_dev,
        })
        cfg = json.loads(self.config_path.read_text(encoding="utf-8"))
        if enable_hardware_actuators and not cfg["actuators"].get("enabled"):
            raise PermissionError("actuators.enabled is false in vehicle config; refusing hardware output")

        self.stop()
        self._temp_dir = tempfile.TemporaryDirectory(prefix="atal-runtime-")
        work = Path(self._temp_dir.name)
        parm = work / "runtime.parm"
        parm.write_text(build_runtime_parm(include_dvl=True, include_motor=True), encoding="utf-8")

        dvl_status_udp = "127.0.0.1:14655"
        (ROOT / "integration/logs/ardusub").mkdir(parents=True, exist_ok=True)
        (ROOT / "integration/logs/storage").mkdir(parents=True, exist_ok=True)
        cmd = [
            str(NATIVE_BINARY),
            "--serial0", mavlink_gcs,
            "--serial1", xsens_dev,
            "--serial2", depth_dev,
            "--serial3", dashboard_udp,
            "--defaults", str(parm),
            "--log-directory", str(ROOT / "integration/logs/ardusub"),
            "--storage-directory", str(ROOT / "integration/logs/storage"),
            "--terrain-directory", str(work / "terrain"),
        ]
        self.processes["ardusub"] = subprocess.Popen(
            cmd, cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        if connect_a50:
            self.processes["dvl_bridge"] = subprocess.Popen(
                [sys.executable, str(ROOT / "integration/dvl_bridge.py"),
                 "--config", str(self.config_path), "--connect-a50",
                 "--status-udp", dvl_status_udp],
                cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        if enable_hardware_actuators and actuator_dev:
            self.processes["actuator_serial_bridge"] = subprocess.Popen(
                [sys.executable, str(ROOT / "integration/actuator_serial_bridge.py"),
                 "--config", str(self.config_path), "--device", actuator_dev,
                 "--port", "14660", "--enable-hardware"],
                cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        self.mode = "native_sensors"
        return self.status()

    def stop(self):
        for name in ("actuator_serial_bridge", "dvl_bridge", "ardusub"):
            proc = self.processes.pop(name, None)
            if proc is not None and proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                except ProcessLookupError:
                    pass
        if self._temp_dir is not None:
            self._temp_dir.cleanup()
            self._temp_dir = None
        self.mode = "stopped"
        return self.status()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--status", action="store_true", help="print runtime ownership & binary check")
    args = parser.parse_args()
    sup = RuntimeSupervisor()
    if args.status or True:
        print(json.dumps(sup.status(), indent=2))


if __name__ == "__main__":
    main()
