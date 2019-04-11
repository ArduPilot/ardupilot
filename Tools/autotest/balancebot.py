#!/usr/bin/env python

# Drive balancebot in SITL
from __future__ import print_function

import os

from apmrover2 import AutoTestRover
from common import AutoTest

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

def log_name(self):
    return "BalanceBot"

class AutoTestBalanceBot(AutoTestRover):

    def vehicleinfo_key(self):
        return "APMrover2"

    def init(self):
        if self.frame is None:
            self.frame = 'balancebot'
        super(AutoTestBalanceBot, self).init()

    def test_do_set_mode_via_command_long(self):
        self.do_set_mode_via_command_long("HOLD")
        self.do_set_mode_via_command_long("MANUAL")

    def rc_defaults(self):
        ret = super(AutoTestBalanceBot, self).rc_defaults()
        ret[3] = 1500
        return ret

    def is_balancebot(self):
        return True

    def drive_rtl_mission_max_distance_from_home(self):
        '''maximum distance allowed from home at end'''
        '''balancebot tends to wander backwards, away from the target'''
        return 8

    def drive_rtl_mission(self):
        # if we Hold then the balancebot continues to wander
        # indefinitely at ~1m/s
        self.set_parameter("MIS_DONE_BEHAVE", 1)
        super(AutoTestBalanceBot, self).drive_rtl_mission()

    def tests(self):
        '''return list of all tests'''

        '''note that while AutoTestBalanceBot inherits from Rover we don't
inherit Rover's tests!'''
        ret = AutoTest.tests(self)

        ret.extend([

            ("DriveRTL",
             "Drive an RTL Mission",
             self.drive_rtl_mission),

            ("DriveMission",
             "Drive Mission %s" % "balancebot1.txt",
             lambda: self.drive_mission("balancebot1.txt")),

            ("GetBanner", "Get Banner", self.do_get_banner),

            ("GetCapabilities",
             "Get Capabilities",
             self.do_get_autopilot_capabilities),

            ("DO_SET_MODE",
             "Set mode via MAV_COMMAND_DO_SET_MODE",
             self.test_do_set_mode_via_command_long),

            ("ServoRelayEvents",
             "Test ServoRelayEvents",
             self.test_servorelayevents),

            ("DownLoadLogs", "Download logs", lambda:
             self.log_download(
                 self.buildlogs_path("APMrover2-log.bin"),
                 upload_logs=len(self.fail_list) > 0)),
            ])
        return ret

    def default_mode(self):
        return 'MANUAL'

