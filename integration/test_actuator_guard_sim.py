#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Exercise native-output mapping through the isolated ESP32 guard simulator."""

import subprocess
import tempfile
import unittest
from pathlib import Path

from actuator_frames import encode_frame
from rcoutput_mapping import load_motor_order, motors_in_esp_order, parse_observation


HERE = Path(__file__).resolve().parent
SESSION = 0x1234ABCD


def observation(armed, motors):
    values = list(motors) + [0] * 8
    return ("ATAL_RCOUT,1," + str(int(armed)) + "," + ",".join(map(str, values))).encode()


class ActuatorGuardSimulatorTests(unittest.TestCase):
    def test_mapped_output_disarm_and_stale_fault(self):
        with tempfile.TemporaryDirectory(prefix="atal-guard-sim-") as directory:
            executable = Path(directory) / "actuator_guard_sim"
            subprocess.run(
                ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I", str(HERE),
                 str(HERE / "actuator_guard_sim.cpp"), "-o", str(executable)],
                check=True, capture_output=True, text=True,
            )
            ap_values = (1550, 1550, 1450, 1450, 1500, 1500, 1500, 1500)
            armed, channels = parse_observation(observation(True, ap_values))
            self.assertTrue(armed)
            mapped = motors_in_esp_order(channels, load_motor_order())
            commands = [
                (0, encode_frame("ARM", SESSION, 1)),
                (10, encode_frame("PWM", SESSION, 2, mapped)),
                (20, encode_frame("DISARM", SESSION, 3)),
                (30, encode_frame("ARM", SESSION, 4)),
                (40, encode_frame("PWM", SESSION, 5, mapped)),
                (291, "TICK"),
                (300, encode_frame("RESET", SESSION, 6)),
                (310, encode_frame("ARM", SESSION, 7)),
                (320, encode_frame("PWM", SESSION, 8, mapped)),
            ]
            run = subprocess.run(
                [str(executable)], input="".join(f"{when} {frame}\n" for when, frame in commands),
                check=True, capture_output=True, text=True,
            )
            rows = [line.split() for line in run.stdout.splitlines()]
            self.assertEqual(len(rows), len(commands))
            self.assertEqual([row[1] for row in rows],
                             ["armed", "armed", "disarmed", "armed", "armed",
                              "fault", "disarmed", "armed", "armed"])
            self.assertEqual(tuple(map(int, rows[1][2:])), mapped)
            self.assertEqual(tuple(map(int, rows[2][2:])), (1500,) * 8)
            self.assertEqual(tuple(map(int, rows[5][2:])), (1500,) * 8)
            self.assertEqual(tuple(map(int, rows[-1][2:])), mapped)

    def test_invalid_frame_latches_neutral_until_reset(self):
        with tempfile.TemporaryDirectory(prefix="atal-guard-sim-") as directory:
            executable = Path(directory) / "actuator_guard_sim"
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I", str(HERE),
                            str(HERE / "actuator_guard_sim.cpp"), "-o", str(executable)],
                           check=True, capture_output=True, text=True)
            commands = [encode_frame("ARM", SESSION, 1),
                        encode_frame("PWM", SESSION, 2, (1500,) * 8)[:-1] + "0",
                        encode_frame("ARM", SESSION, 3),
                        encode_frame("RESET", SESSION, 4)]
            run = subprocess.run([str(executable)], input="".join(f"{i} {frame}\n" for i, frame in enumerate(commands)),
                                 check=True, capture_output=True, text=True)
            rows = [line.split() for line in run.stdout.splitlines()]
            self.assertEqual([row[:2] for row in rows],
                             [["1", "armed"], ["0", "fault"], ["0", "fault"], ["1", "disarmed"]])
            self.assertEqual(tuple(map(int, rows[1][2:])), (1500,) * 8)


if __name__ == "__main__":
    unittest.main()
