"""Protocol and frame checks for the non-actuating A50 adapter."""

import json
import math
import unittest

from dvl_velocity import parse_velocity, rotate, to_ned, mavlink_covariance


class DvlVelocityTests(unittest.TestCase):
    def setUp(self):
        self.report = dict(type="velocity", vx=1.0, vy=0.0, vz=0.0,
                           velocity_valid=True, status=0, altitude=1.2,
                           fom=0.1, time=100.0, time_of_validity=1700000000000000,
                           covariance=[[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]])

    def sample(self):
        return parse_velocity(json.dumps(self.report), 10.0)

    def test_bottom_track_and_frame_rotation(self):
        sample = self.sample()
        self.assertEqual(sample.validity_unix_us, 1700000000000000)
        self.assertAlmostEqual(sample.velocity_error_mps, math.sqrt(0.03))
        # Body forward becomes East at +90 deg NED yaw.
        yaw_90 = (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
        north, east, down = to_ned(sample, yaw_90, now_monotonic_s=10.1)
        self.assertAlmostEqual(north, 0, places=6)
        self.assertAlmostEqual(east, 1, places=6)
        self.assertAlmostEqual(down, 0, places=6)
        self.assertAlmostEqual(sum(mavlink_covariance(sample.velocity_error_mps)[::4]), 0.03)

    def test_reject_invalid_and_water_track(self):
        for change in ({"velocity_valid": False}, {"type": "velocity_water"},
                       {"type": "position_local"}, {"status": 1}, {"fom": float("nan")},
                       {"vx": float("inf")}, {"altitude": -1},
                       {"time_of_validity": -5}, {"covariance": [[-1, 0, 0]] * 3},
                       {"covariance": [[0.01, float("nan"), 0], [0, 0.01, 0], [0, 0, 0.01]]}):
            report = self.report | change
            with self.subTest(change=change):
                self.assertIsNone(parse_velocity(json.dumps(report), 10.0))
        self.assertIsNone(parse_velocity('{bad json', 10.0))

    def test_stale_and_lever_arm(self):
        sample = self.sample()
        with self.assertRaisesRegex(ValueError, "stale"):
            to_ned(sample, (1, 0, 0, 0), now_monotonic_s=11)
        # +1 rad/s yaw at a sensor 1m forward means sensor velocity is +1m/s right.
        corrected = to_ned(sample, (1, 0, 0, 0),
                           offset_m_frd=(1, 0, 0), body_rates_rad_s=(0, 0, 1),
                           now_monotonic_s=10.1)
        self.assertEqual(corrected, (1, -1, 0))
        self.assertEqual(rotate((1, 0, 0, 0), (1, 2, 3)), (1, 2, 3))


if __name__ == "__main__":
    unittest.main()
