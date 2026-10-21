"""AutoTest entry for Antoine through the MAVLink-to-NATS bridge.

The native Antoine/NATS/bridge stack must already be running. This suite uses
ArduPilot's AutoTest result handling without starting ArduCopter SITL.
"""

import json
import os
import pathlib
import shutil
import socket
import subprocess
import tempfile
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
        """Exercise arm, guided position, mission, landing and disarm via NATS."""

    def run_tests(self, tests):
        """Start an isolated Antoine stack and report its acceptance result."""
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

        if shutil.which("nats-server") is None:
            raise FileNotFoundError("nats-server is required to run test.AntoineBridge")
        cargo_target = self._cargo_target_dir(antoine_root)
        model_manifest = antoine_root / "sitl" / "antoine-sitl" / "Cargo.toml"
        bridge_manifest = (
            antoine_root
            / "integrations"
            / "antoine-mavlink-jetstream-bridge"
            / "Cargo.toml"
        )
        self._build(antoine_root, model_manifest, bridge_manifest)
        model_binary = cargo_target / "debug" / "antoine_sitl_nats"
        bridge_binary = cargo_target / "debug" / "antoine-mavlink-jetstream-bridge"

        for test in tests:
            result = Result(test)
            result.passed = False
            started = time.monotonic()
            run_dir = pathlib.Path(tempfile.mkdtemp(prefix="antoine-autotest-"))
            children = []
            log_handles = []
            try:
                nats_port = self._free_port(socket.SOCK_STREAM)
                mavlink_port = self._free_port(socket.SOCK_DGRAM)
                fallback_port = self._free_port(socket.SOCK_DGRAM)
                while fallback_port == mavlink_port:
                    fallback_port = self._free_port(socket.SOCK_DGRAM)

                nats_store = run_dir / "nats-store"
                nats_store.mkdir()
                nats_log = (run_dir / "nats.log").open("wb")
                log_handles.append(nats_log)
                children.append(
                    subprocess.Popen(
                        [
                            "nats-server",
                            "-js",
                            "-a",
                            "127.0.0.1",
                            "-p",
                            str(nats_port),
                            "-sd",
                            str(nats_store),
                        ],
                        cwd=antoine_root,
                        stdout=nats_log,
                        stderr=subprocess.STDOUT,
                    )
                )
                self._wait_tcp(children[0], nats_port, 10)

                model_log = (run_dir / "antoine.log").open("wb")
                log_handles.append(model_log)
                children.append(
                    subprocess.Popen(
                        [
                            str(model_binary),
                            "--callsign",
                            "antoine-sitl",
                            "--nats-url",
                            f"nats://127.0.0.1:{nats_port}",
                            "--plant",
                            "antoine",
                            "--thrust",
                            "0.0",
                            "--hz",
                            "30",
                            "--enable-mavlink-control",
                        ],
                        cwd=antoine_root,
                        stdout=model_log,
                        stderr=subprocess.STDOUT,
                    )
                )

                bridge_log = (run_dir / "bridge.log").open("wb")
                log_handles.append(bridge_log)
                children.append(
                    subprocess.Popen(
                        [
                            str(bridge_binary),
                            "--mode",
                            "antoine-nats",
                            "--callsign",
                            "antoine-sitl",
                            "--nats-url",
                            f"nats://127.0.0.1:{nats_port}",
                            "--qgc-bind",
                            f"127.0.0.1:{mavlink_port}",
                            "--qgc-peer",
                            f"127.0.0.1:{fallback_port}",
                            "--system-id",
                            "1",
                            "--home-alt-msl",
                            "82",
                        ],
                        cwd=antoine_root,
                        stdout=bridge_log,
                        stderr=subprocess.STDOUT,
                    )
                )
                self._wait_udp(children[2], mavlink_port, 10)
                command = [
                    "uv",
                    "run",
                    "--no-project",
                    "--with",
                    "pymavlink",
                    "python",
                    str(script),
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(mavlink_port),
                    "--timeout",
                    os.environ.get("ANTOINE_MAVLINK_TIMEOUT", "60"),
                ]
                subprocess.run(command, cwd=antoine_root, check=True)
                result.passed = True
            except (OSError, subprocess.CalledProcessError, RuntimeError, TimeoutError) as error:
                result.exception = error
                result.reason = "Antoine bridge acceptance failed"
            finally:
                for process in reversed(children):
                    if process.poll() is None:
                        try:
                            process.terminate()
                            process.wait(timeout=5)
                        except ProcessLookupError:
                            continue
                        except subprocess.TimeoutExpired:
                            try:
                                process.kill()
                                process.wait(timeout=5)
                            except ProcessLookupError:
                                pass
                for handle in log_handles:
                    handle.close()
            if result.passed:
                shutil.rmtree(run_dir)
            else:
                self.progress(f"Antoine bridge run artifacts retained at {run_dir}")
            result.time_elapsed = time.monotonic() - started
            results.append(result)
        return results

    @staticmethod
    def _free_port(kind):
        with socket.socket(socket.AF_INET, kind) as probe:
            probe.bind(("127.0.0.1", 0))
            return probe.getsockname()[1]

    @staticmethod
    def _wait_tcp(process, port, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("NATS exited before becoming ready")
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                    return
            except OSError:
                time.sleep(0.05)
        raise TimeoutError("NATS did not become ready")

    @staticmethod
    def _wait_udp(process, port, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("MAVLink bridge exited before binding UDP")
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    probe.bind(("127.0.0.1", port))
            except OSError:
                return
            time.sleep(0.05)
        raise TimeoutError("MAVLink bridge did not bind its UDP port")

    @staticmethod
    def _cargo_target_dir(antoine_root):
        manifest = antoine_root / "Cargo.toml"
        metadata = subprocess.run(
            [
                "cargo",
                "metadata",
                "--format-version",
                "1",
                "--no-deps",
                "--manifest-path",
                str(manifest),
            ],
            cwd=antoine_root,
            check=True,
            capture_output=True,
            text=True,
        )
        return pathlib.Path(json.loads(metadata.stdout)["target_directory"])

    @staticmethod
    def _build(antoine_root, model_manifest, bridge_manifest):
        subprocess.run(
            [
                "cargo",
                "build",
                "--locked",
                "--manifest-path",
                str(model_manifest),
                "--features",
                "nats",
                "--bin",
                "antoine_sitl_nats",
            ],
            cwd=antoine_root,
            check=True,
        )
        subprocess.run(
            [
                "cargo",
                "build",
                "--locked",
                "--manifest-path",
                str(bridge_manifest),
                "--bin",
                "antoine-mavlink-jetstream-bridge",
            ],
            cwd=antoine_root,
            check=True,
        )
