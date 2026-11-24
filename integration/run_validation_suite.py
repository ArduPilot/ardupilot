#!/usr/bin/env python3
"""Reproducible software validation runner for ATAL_ArduSub.

Runs static checks, C++ host tests, Python unit tests, ordinary native Linux
ArduSub sensor/EKF/dashboard/E2E replays, and (with --with-loopback) closed-loop
6-DoF dynamics & RCOutput guard verification before restoring the ordinary build.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parent.parent


def run_step(name, cmd):
    print(f"\n=== {name} ===", flush=True)
    subprocess.run(cmd, cwd=ROOT, check=True)


def run_cpp_host_tests():
    with tempfile.TemporaryDirectory(prefix="atal-cpp-") as tmp:
        d = Path(tmp)
        for src_name, exe_name in (
            ("test_actuator_guard.cpp", "test_actuator_guard"),
            ("test_esp32_guarded.cpp", "test_esp32_guarded"),
        ):
            exe = d / exe_name
            run_step(
                f"Compile & run {src_name}",
                ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                 "-I", str(ROOT / "integration"),
                 str(ROOT / "integration" / src_name), "-o", str(exe)],
            )
            subprocess.run([str(exe)], cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-loopback", action="store_true",
        help="also build atal_loopback_output.hwdef, run RCOutput + 6-DoF dynamics, and restore ordinary build",
    )
    args = parser.parse_args()

    # 1. Static & parity checks
    run_step("Layout & motor allocation check", [sys.executable, "integration/check_layout.py"])
    run_step("Allocation simulation", [sys.executable, "integration/simulate_allocation.py"])
    run_step("Dashboard parity check", [sys.executable, "integration/check_dashboard_parity.py"])

    if shutil.which("node"):
        run_step("Classic JS adapter syntax check", ["node", "--check", "integration/static/classic_adapter.js"])

    if shutil.which("pycodestyle"):
        py_files = sorted(str(p.relative_to(ROOT)) for p in (ROOT / "integration").glob("*.py"))
        run_step("pycodestyle (max-line-length=127)",
                 ["pycodestyle", "--max-line-length=127", "--ignore=E127,E128,E402,E501,W503,W504", *py_files])

    # 2. C++ host & parser unit tests
    run_cpp_host_tests()
    parser_test = ROOT / "build/linux/tests/test_xsens_parser"
    if not parser_test.is_file():
        run_step("Build test_xsens_parser",
                 ["./waf", "tests", "--targets=tests/test_xsens_parser", "--no-submodule-update", "-j4"])
    run_step("Run test_xsens_parser", [str(parser_test)])

    # 3. Python unit suite
    run_step("Python unittest suite",
             [sys.executable, "-m", "unittest", "discover", "-s", "integration", "-p", "test_*.py", "-v"])

    # 4. Native ordinary Linux ArduSub replay & E2E suite
    native_binary = ROOT / "build/linux/bin/ardusub"
    hwdef_h = ROOT / "build/linux/hwdef.h"
    need_ordinary = (not native_binary.is_file() or
                     (hwdef_h.is_file() and "HAL_ATAL_RCOUT_LOOPBACK_ENABLED" in hwdef_h.read_text()))
    if need_ordinary:
        run_step("Configure ordinary Linux build", ["./waf", "configure", "--board=linux", "--no-submodule-update"])
        run_step("Build ordinary Linux ArduSub", ["./waf", "sub", "--no-submodule-update", "-j4"])

    for script in (
        "integration/test_depth_native_replay.py",
        "integration/test_dvl_mavlink_replay.py",
        "integration/test_dvl_native_ingest.py",
        "integration/test_dvl_native_chain.py",
        "integration/test_extnav_position_native.py",
        "integration/test_dashboard_native_replay.py",
        "integration/test_dashboard_api_loopback.py",
        "integration/test_native_end_to_end.py",
    ):
        run_step(f"Run {script}", [sys.executable, script])

    # 5. Optional loopback RCOutput & 6-DoF closed-loop dynamics suite
    if args.with_loopback:
        try:
            run_step("Configure loopback hwdef build",
                     ["./waf", "configure", "--board=linux",
                      "--extra-hwdef", "integration/atal_loopback_output.hwdef", "--no-submodule-update"])
            run_step("Build loopback Linux ArduSub", ["./waf", "sub", "--no-submodule-update", "-j4"])
            run_step("RCOutput native loopback + guard sim + PTY",
                     [sys.executable, "integration/test_rcoutput_native_loopback.py",
                      "--active", "--guard-sim", "--serial-pty"])
            run_step("Closed-loop 6-DoF native dynamics", [sys.executable, "integration/test_closed_loop_dynamics.py"])
        finally:
            run_step("Restore ordinary Linux build", ["./waf", "configure", "--board=linux", "--no-submodule-update"])
            run_step("Rebuild ordinary Linux ArduSub", ["./waf", "sub", "--no-submodule-update", "-j4"])

    print("\nALL REQUESTED VALIDATION SUITES PASSED", flush=True)


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    main()
