#!/usr/bin/env python

'''
Drive a Sailboat in SITL

AP_FLAKE8_CLEAN

'''

from __future__ import print_function

import os

from rover import AutoTestRover

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


def log_name(self):
    return "Sailboat"


class AutoTestSailboat(AutoTestRover):

    def vehicleinfo_key(self):
        return "Rover"

    def init(self):
        if self.frame is None:
            self.frame = 'sailboat'
        super(AutoTestSailboat, self).init()

    def tests(self):
        '''return list of all tests'''
        ret = ([])

        ret.extend([
            ("DriveRTL",
             "Drive an RTL Mission",
             self.drive_rtl_mission),

            ("DriveMission",
             "Drive Mission %s" % "balancebot1.txt",
             lambda: self.drive_mission("balancebot1.txt", strict=False)),

        ])
        return ret

    def default_mode(self):
        return 'MANUAL'
