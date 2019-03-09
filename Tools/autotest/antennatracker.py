#!/usr/bin/env python

from __future__ import print_function
import os
import pexpect
from pymavlink import mavutil

from common import AutoTest
from pysim import util
from pysim import vehicleinfo
import operator

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

    def start_stream_systemtime(self):
        '''AntennaTracker doesn't stream this by default but we need it for get_sim_time'''
        try:
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 10)
        except Exception:
            pass
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 10)

    def set_rc_default(self):
        '''tracker does not send RC_CHANNELS, so can't set_rc_default'''
        '''... however, dodgily hook in here to get system time:'''
        self.start_stream_systemtime()

    def initialise_after_reboot_sitl(self):
        self.start_stream_systemtime()

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

    def reboot_sitl(self):
        """Reboot SITL instance and wait it to reconnect."""
        self.mavproxy.send("reboot\n")
        self.mavproxy.expect("Initialising APM")
        # empty mav to avoid getting old timestamps:
        while self.mav.recv_match(blocking=False):
            pass
        self.initialise_after_reboot_sitl()

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
