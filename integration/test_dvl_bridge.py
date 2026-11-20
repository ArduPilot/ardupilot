import json
import unittest

from dvl_bridge import measurement


CONFIG = {"fom_max_mps": 0.25, "mount_quaternion_wxyz": [1, 0, 0, 0],
          "offset_m_frd": [0, 0, 0], "velocity_timeout_s": 0.75}
REPORT = {"type": "velocity", "velocity_valid": True, "status": 0,
          "altitude": 2.0, "fom": 0.1, "vx": 1.0, "vy": 2.0,
          "vz": 3.0, "time_of_validity": 123456789}


class BridgeTests(unittest.TestCase):
    def test_valid_velocity_requires_fresh_attitude(self):
        line = json.dumps(REPORT)
        self.assertIsNone(measurement(line, 10.0, None, CONFIG))
        self.assertIsNone(measurement(line, 10.0, (9.4, (1, 0, 0, 0), (0, 0, 0)), CONFIG))
        result = measurement(line, 10.0, (9.9, (1, 0, 0, 0), (0, 0, 0)), CONFIG)
        self.assertEqual(result[0], (1, 2, 3))
        self.assertEqual(result[2], 123456789)

    def test_invalid_bottom_lock_never_publishes(self):
        report = dict(REPORT, velocity_valid=False)
        attitude = (9.9, (1, 0, 0, 0), (0, 0, 0))
        self.assertIsNone(measurement(json.dumps(report), 10.0, attitude, CONFIG))

    def test_invalid_attitude_never_publishes(self):
        attitude = (9.9, (0, 0, 0, 0), (0, 0, 0))
        self.assertIsNone(measurement(json.dumps(REPORT), 10.0, attitude, CONFIG))


if __name__ == "__main__":
    unittest.main()
