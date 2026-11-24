import unittest
import os
import select
import socket
import subprocess
import sys
import time
from pathlib import Path

from actuator_adapter import ActuatorAdapter
from rcoutput_mapping import load_motor_order


def observation(armed, motors=(1500,) * 8):
    channels = (*motors, *(0 for _ in range(8)))
    return ("ATAL_RCOUT,1," + str(int(armed)) + "," + ",".join(map(str, channels))).encode()


class ActuatorAdapterTests(unittest.TestCase):
    def setUp(self):
        self.adapter = ActuatorAdapter(load_motor_order(), 0x1234ABCD)

    def test_requires_disarmed_observation_and_maps_native_output(self):
        motors = (1550, 1550, 1450, 1450, 1500, 1500, 1500, 1500)
        self.assertEqual(self.adapter.ingest(observation(True, motors), 0), ())
        self.assertEqual(self.adapter.ingest(observation(False), 10), ())
        frames = self.adapter.ingest(observation(True, motors), 20)
        self.assertEqual([frame.split(",")[1] for frame in frames], ["ARM", "PWM"])
        self.assertEqual(tuple(map(int, frames[1].split(",")[4:12])),
                         (1450, 1500, 1450, 1500, 1500, 1550, 1550, 1500))
        self.assertEqual([frame.split(",")[1] for frame in self.adapter.ingest(observation(False), 30)],
                         ["DISARM"])
        self.assertEqual(self.adapter.tick(300), ())

    def test_invalid_and_stale_output_fail_closed_until_disarmed(self):
        self.adapter.ingest(observation(False), 0)
        self.adapter.ingest(observation(True), 10)
        self.assertEqual([frame.split(",")[1] for frame in self.adapter.ingest(b"bad", 20)], ["DISARM"])
        self.assertEqual(self.adapter.ingest(observation(True), 30), ())
        self.adapter.ingest(observation(False), 40)
        self.assertEqual([frame.split(",")[1] for frame in self.adapter.ingest(observation(True), 50)],
                         ["ARM", "PWM"])
        self.assertEqual([frame.split(",")[1] for frame in self.adapter.tick(251)], ["DISARM"])
        self.assertEqual(self.adapter.ingest(observation(True), 260), ())

    def test_serial_rate_limit_keeps_latest_output_and_disarm(self):
        adapter = ActuatorAdapter(load_motor_order(), 0x1234ABCD, pwm_interval_ms=20)
        adapter.ingest(observation(False), 0)
        self.assertEqual([frame.split(",")[1] for frame in adapter.ingest(observation(True), 1)],
                         ["ARM", "PWM"])
        self.assertEqual(adapter.ingest(observation(True, (1600,) * 8), 10), ())
        frames = adapter.ingest(observation(True, (1600,) * 8), 21)
        self.assertEqual(len(frames), 1)
        self.assertEqual(tuple(map(int, frames[0].split(",")[4:12])), (1600,) * 8)
        self.assertEqual([frame.split(",")[1] for frame in adapter.ingest(observation(False), 22)],
                         ["DISARM"])

    def test_udp_process_emits_frames_and_disarms_on_shutdown(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        process = subprocess.Popen(
            [sys.executable, str(Path(__file__).with_name("actuator_adapter.py")), "--port", str(port)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        try:
            self.assertTrue(select.select([process.stderr], [], [], 2)[0])
            self.assertIn(b"Waiting for disarmed", process.stderr.readline())
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(observation(False), ("127.0.0.1", port))
                time.sleep(0.1)
                sender.sendto(observation(True), ("127.0.0.1", port))
            output = b""
            deadline = time.monotonic() + 2
            while b",PWM," not in output and time.monotonic() < deadline:
                if select.select([process.stdout], [], [], 0.1)[0]:
                    output += os.read(process.stdout.fileno(), 4096)
            self.assertIn(b",ARM,", output)
            self.assertIn(b",PWM,", output)
            process.terminate()
            tail, _ = process.communicate(timeout=2)
            self.assertIn(b",DISARM,", output + tail)
            self.assertEqual(process.returncode, 0)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


if __name__ == "__main__":
    unittest.main()
