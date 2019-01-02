#!/usr/bin/env python

# Drive balancebot in SITL
from __future__ import print_function

import os
import pexpect

from apmrover2 import AutoTestRover
from common import AutoTest

from pymavlink import mavutil

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

# HOME = mavutil.location(-35.362938, 149.165085, 584, 270)
HOME = mavutil.location(40.071374969556928,
                        -105.22978898137808,
                        1583.702759,
                        246)


class AutoTestBalanceBot(AutoTestRover):
    def __init__(self,
                 binary,
                 valgrind=False,
                 gdb=False,
                 speedup=10,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 **kwargs):
        super(AutoTestBalanceBot, self).__init__(binary,
                                                 valgrind,
                                                 gdb,
                                                 speedup,
                                                 frame,
                                                 params,
                                                 gdbserver,
                                                 **kwargs)
        self.log_name = "BalanceBot"

    def vehicleinfo_key(self):
        return "APMrover2"

    def init(self):
        if self.frame is None:
            self.frame = 'balancebot'
        super(AutoTestBalanceBot, self).init()

    def test_do_set_mode_via_command_long(self):
        self.do_set_mode_via_command_long("HOLD")
        self.do_set_mode_via_command_long("MANUAL")

    def set_rc_default(self):
        super(AutoTestBalanceBot, self).set_rc_default()
        self.set_rc(3, 1500)

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

