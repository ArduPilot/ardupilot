#!/usr/bin/env python

from __future__ import print_function
import os
from pymavlink import mavutil

from common import AutoTest

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-27.274439, 151.290064, 343, 8.7)

class AutoTestTracker(AutoTest):

    def log_name(self):
        return "AntennaTracker"

    def test_filepath(self):
         return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_mode(self):
        return "AUTO"

    def is_tracker(self):
        return True

    def default_frame(self):
        return "tracker"

    def apply_defaultfile_parameters(self):
        # tracker doesn't have a default parameters file
        pass

    def sysid_thismav(self):
        return 2

    def disabled_tests(self):
        return {
            "ArmFeatures": "See https://github.com/ArduPilot/ardupilot/issues/10652",
        }

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestTracker, self).tests()
        ret.extend([
        ])
        return ret
