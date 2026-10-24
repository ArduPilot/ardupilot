#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Mission upload conversion must preserve NAV_SCRIPT_TIME integer arguments."""

import sys
import unittest

from pathlib import Path
from unittest.mock import patch

from pymavlink.dialects.v20 import ardupilotmega

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vehicle_test_suite import TestSuite as VehicleTestSuite  # noqa: E402
from vehicle_test_suite import mavutil  # noqa: E402


class MissionConversionTest(unittest.TestCase):
    def setUp(self):
        dialect = patch('vehicle_test_suite.mavutil.mavlink', ardupilotmega)
        dialect.start()
        self.addCleanup(dialect.stop)

    def test_script_arguments_are_not_coordinates(self):
        wp = mavutil.mavlink.MAVLink_mission_item_message(
            1, 1, 4, 0, mavutil.mavlink.MAV_CMD_NAV_SCRIPT_TIME, 0, 1,
            72, 0, 300, 1, 13, -17, 0)
        converted = VehicleTestSuite.wp_to_mission_item_int(None, wp, 0)
        self.assertEqual((converted.x, converted.y), (13, -17))
        self.assertEqual((converted.param1, converted.param3, converted.param4), (72, 300, 1))
        self.assertIs(VehicleTestSuite.wp_to_mission_item_int(None, converted, 0), converted)

    def test_waypoint_coordinates_still_scaled(self):
        wp = mavutil.mavlink.MAVLink_mission_item_message(
            1, 1, 3, 3, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
            0, 0, 0, 0, -35.36, 149.16, 40)
        converted = VehicleTestSuite.wp_to_mission_item_int(None, wp, 0)
        self.assertEqual((converted.x, converted.y), (int(wp.x * 1e7), int(wp.y * 1e7)))


if __name__ == '__main__':
    unittest.main()
