#!/usr/bin/env python

# Drive balancebot in SITL
from __future__ import print_function

import os
import pexpect

from apmrover2 import AutoTestRover

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

    def init(self):
        if self.frame is None:
            self.frame = 'balancebot'
        super(AutoTestBalanceBot, self).init()

    def drive_mission_balancebot1(self):
        self.drive_mission(os.path.join(testdir, "balancebot1.txt"))

    def autotest(self):
        """Autotest APMrover2 in SITL."""
        self.check_test_syntax(test_file=os.path.realpath(__file__))
        if not self.hasInit:
            self.init()
        self.progress("Started simulator")

        self.fail_list = []
        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s" %
                          self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(8, 1800)
            self.progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)
            self.mavproxy.send('switch 6\n')  # Manual mode
            self.wait_mode('MANUAL')

            self.progress("Waiting reading for arm")
            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.run_test("Drive an RTL Mission", self.drive_rtl_mission)

            self.run_test("Drive Mission %s" % "balancebot1.txt",
                          self.drive_mission_balancebot1)

            self.run_test("Disarm Vehicle", self.disarm_vehicle)

            self.run_test("Get Banner", self.do_get_banner)

            self.run_test("Get Capabilities",
                          self.do_get_autopilot_capabilities)

            self.run_test("Set mode via MAV_COMMAND_DO_SET_MODE",
                          lambda: self.do_set_mode_via_command_long("HOLD"))

            self.run_test("Test ServoRelayEvents",
                          self.test_servorelayevents)

            self.run_test("Download logs", lambda:
                          self.log_download(
                              self.buildlogs_path("APMrover2-log.bin"),
                              upload_logs=len(self.fail_list)>0))
    #        if not drive_left_circuit(self):
    #            self.progress("Failed left circuit")
    #            failed = True
    #        if not drive_RTL(self):
    #            self.progress("Failed RTL")
    #            failed = True

        except pexpect.TIMEOUT:
            self.progress("Failed with timeout")
            self.fail_list.append(("*timeout*", None))

        self.close()

        if len(self.fail_list):
            self.progress("FAILED STEPS: %s" % self.fail_list)
            return False
        return True
