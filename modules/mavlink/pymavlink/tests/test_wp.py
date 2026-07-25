#!/usr/bin/env python3

"""
regression tests for mavwp.py
"""

import unittest
import os
try:
    from importlib.resources import files as importlib_files
except ImportError:
    # importlib.resources.files() requires Python 3.9+; use backport for older versions
    from importlib_resources import files as importlib_files
import sys

os.environ["MAVLINK20"] = "1"

from pymavlink import mavwp
from pymavlink import mavutil

class MAVWPTest(unittest.TestCase):

    """
    Class to test mavwp
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(MAVWPTest, self).__init__(*args, **kwargs)

    def makewp(self, seq):
        return mavutil.mavlink.MAVLink_mission_item_int_message(
            1, # wp.target_system,
            1, # wp.target_component,
            0, # wp.seq,
            0, # wp.frame,
            0, # wp.command,
            0, # wp.current,
            1, # wp.autocontinue,
            0, # wp.param1,
            0, # wp.param2,
            0, # wp.param3,
            0, # wp.param4,
            0, # int(wp.x*1.0e7),
            0, # int(wp.y*1.0e7),
            seq, # wp.z
        )

    def make_wps(self):
        # create waypoints
        count = 4
        waypoints = []
        for i in range(count):
            waypoints.append(self.makewp(i))

        return waypoints

    def test_add_remove(self):
        """Test we can add/remove waypoints to/from mavwp"""
        loader = mavwp.MAVWPLoader()
        self.assertEqual(loader.mav_mission_type(),
                         mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        waypoints = self.make_wps()

        # make sure basic addition works
        for i in range(len(waypoints)):
            self.assertEqual(loader.count(), i)
            loader.add(waypoints[i])
            self.assertEqual(loader.count(), i+1)

        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(2).seq, 2)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 1)
        self.assertEqual(loader.wp(2).z, 2)
        # short test for the new item() method:
        self.assertEqual(loader.item(0).z, 0)
        self.assertEqual(loader.item(1).z, 1)
        self.assertEqual(loader.item(2).z, 2)


        # remove a middle one, make sure things get renumbered
        loader.remove(waypoints[0])
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(2).seq, 2)
        self.assertEqual(loader.wp(0).z, 1)
        self.assertEqual(loader.wp(1).z, 2)
        self.assertEqual(loader.wp(2).z, 3)

        loader.clear()
        self.assertEqual(loader.count(), 0)

    def test_insert(self):
        """Test we can insert waypoints into mavwp"""
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()

        loader.add(waypoints[0])
        loader.add(waypoints[2])
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 2)
        loader.insert(1, waypoints[3])
        self.assertEqual(loader.count(), 3)
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(2).seq, 2)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 3)
        self.assertEqual(loader.wp(2).z, 2)

    def test_set(self):
        """Test we set waypoints in mavwp"""
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()

        loader.add(waypoints[0])
        loader.add(waypoints[2])
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 2)
        loader.set(waypoints[3], 1)
        self.assertEqual(loader.count(), 2)
        self.assertEqual(loader.wp(0).seq, 0)
        self.assertEqual(loader.wp(1).seq, 1)
        self.assertEqual(loader.wp(0).z, 0)
        self.assertEqual(loader.wp(1).z, 3)

        # setting at the end of the list extends the list:
        loader.set(waypoints[1], 2)

    def test_add_latlonalt(self):
        '''test add_latlonalt method'''
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()

        loader.add(waypoints[0])
        loader.add(waypoints[2])

        loader.add_latlonalt(5, 6, 7, terrain_alt=True)
        loader.add_latlonalt(10, 11, 12, terrain_alt=False)

        wp1 = loader.wp(2)
        self.assertEqual(wp1.x, 5)
        self.assertEqual(wp1.y, 6)
        self.assertEqual(wp1.z, 7)
        self.assertEqual(wp1.frame, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)

        wp1 = loader.wp(3)
        self.assertEqual(wp1.x, 10)
        self.assertEqual(wp1.y, 11)
        self.assertEqual(wp1.z, 12)
        self.assertEqual(wp1.frame, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_save(self):
        '''test load/save in default format'''
        loader = mavwp.MAVWPLoader()
        waypoints = self.make_wps()
        for wp in waypoints:
            loader.add(wp)

        outfile = "test_wp-wp.txt"
        loader.save(outfile)

        loader2 = mavwp.MAVWPLoader()
        loader2.load(outfile)

        outfile2 = "test_wp-wp2.txt"
        loader.save(outfile2)

        self.assertEqual(loader.count(), loader2.count())

        for i in range(loader.count()):
            self.assertEqual(loader.wp(i).z, loader2.wp(i).z)

    def test_wp_is_loiter(self):
        '''test is_loiter method'''
        loader = mavwp.MAVWPLoader()

        wp = self.makewp(1)
        wp.command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
        loader.add(wp)

        wp = self.makewp(2)
        wp.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        loader.add(wp)

        wp = self.makewp(2)
        wp.command = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        loader.add(wp)

        self.assertTrue(loader.wp_is_loiter(0))
        self.assertFalse(loader.wp_is_loiter(1))
        self.assertFalse(loader.wp_is_loiter(2))

        assert True

    def test_is_location_command(self):
        loader = mavwp.MAVWPLoader()
        self.assertFalse(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH))
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT))
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS))

class RallyTest(unittest.TestCase):
    '''tests functions related to loading waypoints and transferring them
    via the mission-item-protocol'''
    def test_rally_load(self):
        '''test loading rally points from old RALLY style file'''
        loader = mavwp.MissionItemProtocol_Rally()
        self.assertEqual(loader.mav_mission_type(),
                         mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT))

        # test loading a QGC WPL 110 file:
        rally_filepath = importlib_files(__spec__.parent).joinpath("rally-110.txt")
        loader.load(rally_filepath)
        self.assertEqual(loader.count(), 2)
        self.assertEqual(loader.wp(0).command, mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT)

        # test loading tradition "RALLY" style format:
        rally_filepath = importlib_files(__spec__.parent).joinpath("rally.txt")
        loader.load(rally_filepath)
        self.assertEqual(loader.count(), 2)
        self.assertEqual(loader.wp(0).command, mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT)

class FenceTest(unittest.TestCase):
    def test_fence_load(self):
        '''test loading rally points from old RALLY style file'''
        loader = mavwp.MissionItemProtocol_Fence()
        self.assertEqual(loader.mav_mission_type(),
                         mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.assertTrue(loader.is_location_command(mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION))

        # test loading a QGC WPL 110 file:
        fence_filepath = importlib_files(__spec__.parent).joinpath("fence-110.txt")
        loader.load(fence_filepath)
        self.assertEqual(loader.count(), 10)
        self.assertEqual(loader.wp(3).command, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

        # test loading tradition lat/lng-pair style format:
        fence_filepath = importlib_files(__spec__.parent).joinpath("fence.txt")
        loader.load(fence_filepath)
        # there are 6 lines in the file - one return point, four fence
        # points and a "fence closing point".  We don't store the
        # fence closing point.
        self.assertEqual(loader.count(), 5)
        self.assertEqual(loader.wp(0).command, mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT)
        self.assertEqual(loader.wp(3).command, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

if __name__ == '__main__':
    unittest.main()
