"""AutoTest entry for Antoine through the MAVLink-to-NATS bridge.

The native Antoine/NATS/bridge stack must already be running. This suite uses
ArduPilot's AutoTest result handling without starting ArduCopter SITL.
"""

import os
import pathlib
import subprocess
import time

from vehicle_test_suite import Result
from vehicle_test_suite import Test
from vehicle_test_suite import TestSuite


class AutoTestAntoineBridge(TestSuite):
    """Run Antoine's focused MAVLink bridge acceptance scenario."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.frame = "antoine-native"

    def default_frame(self):
        return "antoine-native"

    def vehicleinfo_key(self):
        return "AntoineBridge"

    def tests(self):
        return [Test(self.MavlinkNatsFlight)]

    def MavlinkNatsFlight(self):
        """Arm, take off, observe altitude, land and disarm Antoine via NATS."""

    def run_tests(self, tests):
        """Invoke the Antoine acceptance client and report through AutoTest."""
        results = []
        antoine_root = pathlib.Path(
            os.environ.get(
                "ANTOINE_ROOT",
                pathlib.Path(__file__).resolve().parents[3] / "antoine",
            )
        ).resolve()
        script = antoine_root / "tools" / "antoine_mavlink_acceptance.py"
        if not script.is_file():
            raise FileNotFoundError(
                f"Antoine acceptance runner not found: {script}; set ANTOINE_ROOT"
            )

        for test in tests:
            result = Result(test)
            result.passed = False
            started = time.monotonic()
            try:
                command = [
                    "uv",
                    "run",
                    "--no-project",
                    "--with",
                    "pymavlink",
                    "python",
                    str(script),
                    "--host",
                    os.environ.get("ANTOINE_MAVLINK_HOST", "127.0.0.1"),
                    "--port",
                    os.environ.get("ANTOINE_MAVLINK_PORT", "14751"),
                    "--timeout",
                    os.environ.get("ANTOINE_MAVLINK_TIMEOUT", "60"),
                ]
                subprocess.run(command, cwd=antoine_root, check=True)
                result.passed = True
            except (OSError, subprocess.CalledProcessError) as error:
                result.exception = error
                result.reason = "Antoine bridge acceptance failed"
            result.time_elapsed = time.monotonic() - started
            results.append(result)
        return results
