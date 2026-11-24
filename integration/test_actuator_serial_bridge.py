import json
import os
import pty
import select
import socket
import subprocess
import sys
import tempfile
import time
import tty
import unittest
from pathlib import Path
from unittest.mock import patch

from actuator_serial_bridge import StatusMonitor, read_session, run
from test_actuator_adapter import observation


class SessionTests(unittest.TestCase):
    def test_session_announcement(self):
        class FakeSerial:
            lines = iter((b"A1,SESSION,87654321,00000000\n",
                          b"booting\n", b"A1,SESSION,1234ABCD,DEADBEEF\r\n"))

            def write(self, data):
                self.sent = data

            def readline(self):
                return next(self.lines)

        fake = FakeSerial()
        with patch("actuator_serial_bridge.secrets.randbits", return_value=0xDEADBEEF):
            self.assertEqual(read_session(fake), 0x1234ABCD)
        self.assertEqual(fake.sent, b"A1,HELLO,DEADBEEF\n")

    def test_guard_fault_stops_bridge(self):
        class FakeSerial:
            in_waiting = 9

            def read(self, _size):
                return b"A1,FAULT\n"

            def flush(self):
                pass

        with self.assertRaisesRegex(RuntimeError, "ESP32 guard fault"):
            run(FakeSerial(), 0, 0x1234ABCD, lambda: False)

    def test_status_monitor_bounds_partial_and_unexpected_input(self):
        monitor = StatusMonitor()
        monitor.ingest(b"A1,FA")
        with self.assertRaisesRegex(RuntimeError, "ESP32 guard fault"):
            monitor.ingest(b"ULT\n")
        with self.assertRaisesRegex(RuntimeError, "status stream overflow"):
            StatusMonitor().ingest(b"X" * 257)
        with self.assertRaisesRegex(RuntimeError, "Unexpected ESP32 status"):
            StatusMonitor().ingest(b"garbage\n")
        with self.assertRaisesRegex(RuntimeError, "session changed"):
            StatusMonitor().ingest(b"A1,SESSION,00000001,ABCDEF12\n")

    def test_bridge_requires_both_enable_switches(self):
        script = str(Path(__file__).with_name("actuator_serial_bridge.py"))
        result = subprocess.run([sys.executable, script, "--device", "/nonexistent"],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("serial bridge disabled", result.stderr)

    def test_pty_handoff_from_native_observation(self):
        master, slave = pty.openpty()
        tty.setraw(master)
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "vehicle.json"
            config.write_text(json.dumps({"actuators": {"enabled": True}}))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            process = subprocess.Popen([
                sys.executable, str(Path(__file__).with_name("actuator_serial_bridge.py")),
                "--device", os.ttyname(slave), "--port", str(port),
                "--config", str(config), "--enable-hardware",
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                # PTY models the ESP32 boot announcement; no real serial port is opened.
                time.sleep(0.15)
                request = b""
                deadline = time.monotonic() + 2
                while b"\n" not in request and time.monotonic() < deadline:
                    if select.select([master], [], [], 0.1)[0]:
                        request += os.read(master, 4096)
                self.assertTrue(request.startswith(b"A1,HELLO,"))
                challenge = request.split(b",", 2)[2].strip()
                os.write(master, b"A1,SESSION,1234ABCD," + challenge + b"\n")
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    time.sleep(0.15)
                    sender.sendto(observation(False), ("127.0.0.1", port))
                    time.sleep(0.05)
                    sender.sendto(observation(True), ("127.0.0.1", port))
                    last_observation_at = time.monotonic()
                output = b""
                deadline = time.monotonic() + 2
                while b",PWM," not in output and time.monotonic() < deadline:
                    if select.select([master], [], [], 0.1)[0]:
                        output += os.read(master, 4096)
                self.assertIn(b"A1,ARM,1234ABCD,1,", output)
                self.assertIn(b"A1,PWM,1234ABCD,2,", output)
                deadline = time.monotonic() + 2
                while b",DISARM," not in output and time.monotonic() < deadline:
                    if select.select([master], [], [], 0.1)[0]:
                        output += os.read(master, 4096)
                self.assertIn(b",DISARM,", output)
                self.assertLess(time.monotonic() - last_observation_at, 0.8)
                self.assertIsNone(process.poll(), "bridge should stay alive after source timeout")
                process.terminate()
                self.assertEqual(process.wait(timeout=2), 0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    unittest.main()
