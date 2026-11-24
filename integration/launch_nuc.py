#!/usr/bin/env python3
"""NUC runtime launcher for native ArduSub + A50 DVL + interactive dashboard.

Replaces the ROS 2 `control_system.launch.py` low-level control stack on the
vehicle NUC while preserving single-owner serial port validation and the
guarded actuator opt-in boundary.
"""

import argparse
from pathlib import Path
import signal
import subprocess
import sys
import time

from atal_runtime import ROOT, RuntimeSupervisor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=ROOT / "integration/vehicle_defaults.json",
                        help="vehicle config JSON path")
    parser.add_argument("--xsens-dev", required=True, help="Xsens MTi-300 serial device")
    parser.add_argument("--depth-dev", required=True, help="Bar30/ESP32 depth serial device")
    parser.add_argument("--esp32-dev", default="", help="guarded ESP32 thruster serial device")
    parser.add_argument("--connect-a50", action="store_true",
                        help="connect dvl_bridge.py to Water Linked A50 TCP")
    parser.add_argument("--enable-hardware", action="store_true",
                        help="enable guarded actuator_serial_bridge (requires config actuators.enabled=true)")
    parser.add_argument("--http-port", type=int, default=8088, help="dashboard HTTP port")
    parser.add_argument("--dashboard-udp-port", type=int, default=14650)
    parser.add_argument("--dvl-status-udp", default="127.0.0.1:14655")
    args = parser.parse_args()

    sup = RuntimeSupervisor(config_path=args.config)
    dash_proc = subprocess.Popen(
        [
            sys.executable, str(ROOT / "integration/dashboard_api.py"),
            "--mavlink", f"udpin:127.0.0.1:{args.dashboard_udp_port}",
            "--http-port", str(args.http_port),
            "--dvl-status-udp", args.dvl_status_udp,
            "--enable-control",
        ],
        cwd=ROOT,
        start_new_session=True,
    )
    stopping = False

    def _on_sig(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    try:
        sup.start_native_sensors(
            xsens_dev=args.xsens_dev,
            depth_dev=args.depth_dev,
            dashboard_udp=f"udp:127.0.0.1:{args.dashboard_udp_port}",
            connect_a50=args.connect_a50,
            enable_hardware_actuators=args.enable_hardware,
            actuator_dev=args.esp32_dev,
        )
        print(f"ATAL NUC stack running; dashboard at http://127.0.0.1:{args.http_port}/", flush=True)
        while not stopping and dash_proc.poll() is None:
            st = sup.status()
            if not st["processes"].get("ardusub", {}).get("running"):
                raise RuntimeError("Native ardusub process exited unexpectedly")
            time.sleep(0.5)
    finally:
        sup.stop()
        if dash_proc.poll() is None:
            dash_proc.terminate()
            try:
                dash_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                dash_proc.kill()
                dash_proc.wait()


if __name__ == "__main__":
    main()
