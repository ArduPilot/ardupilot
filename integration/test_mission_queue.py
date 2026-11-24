# AP_FLAKE8_CLEAN
"""Mission queue trust-boundary checks; no MAVLink or vehicle commands."""

import unittest

from mission_queue import validate_ros_waypoints


WAYPOINT = {"north": 3.0, "east": -2.0, "depth": 0.8, "yaw": 45,
            "max_vel": 0.4, "tolerance": 0.2, "hold_sec": 2.0,
            "status": "pending"}


class MissionQueueTests(unittest.TestCase):
    def test_preserves_every_behavior_field(self):
        parsed = validate_ros_waypoints([WAYPOINT])
        self.assertEqual(parsed, [{key: float(value) for key, value in WAYPOINT.items() if key != "status"}])

    def test_rejects_incomplete_unknown_and_unsafe_values(self):
        changes = ({"north": float("nan")}, {"yaw": True}, {"depth": -1},
                   {"max_vel": 0}, {"tolerance": 0}, {"hold_sec": -1},
                   {"status": "running"}, {"frame": "global"})
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_ros_waypoints([WAYPOINT | change])
        with self.assertRaises(ValueError):
            validate_ros_waypoints([{key: value for key, value in WAYPOINT.items() if key != "yaw"}])
        with self.assertRaises(ValueError):
            validate_ros_waypoints([])


if __name__ == "__main__":
    unittest.main()
