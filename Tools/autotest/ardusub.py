#!/usr/bin/env python

# Dive ArduSub in SITL
from __future__ import print_function
import os

import pexpect
from pymavlink import mavutil

from pysim import util

from common import AutoTest
from common import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

SITL_START_LOCATION = mavutil.location(33.810313, -118.393867, 0, 185)


class Joystick():
    Pitch = 1
    Roll = 2
    Throttle = 3
    Yaw = 4
    Forward = 5
    Lateral = 6


class AutoTestSub(AutoTest):
    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return []

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return ["AUTO", "GUIDED", "CIRCLE", "POSHOLD"]

    @staticmethod
    def get_position_armable_modes_list():
        return []

    @staticmethod
    def get_normal_armable_modes_list():
        return ["ACRO", "ALT_HOLD", "MANUAL", "STABILIZE", "SURFACE"]

    def log_name(self):
        return "ArduSub"

    def test_filepath(self):
         return os.path.realpath(__file__)

    def default_mode(self):
        return 'MANUAL'

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_frame(self):
        return 'vectored'

    def init(self):
        super(AutoTestSub, self).init()

        # FIXME:
        self.set_parameter("FS_GCS_ENABLE", 0)

    def is_sub(self):
        return True

    def arming_test_mission(self):
        return os.path.join(testdir, "ArduSub-Missions", "test_arming.txt")

    def dive_manual(self):
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.set_rc(Joystick.Throttle, 1600)
        self.set_rc(Joystick.Forward, 1600)
        self.set_rc(Joystick.Lateral, 1550)

        self.wait_distance(50, accuracy=7, timeout=200)
        self.set_rc(Joystick.Yaw, 1550)

        self.wait_heading(0)
        self.set_rc(Joystick.Yaw, 1500)

        self.wait_distance(50, accuracy=7, timeout=100)
        self.set_rc(Joystick.Yaw, 1550)

        self.wait_heading(0)
        self.set_rc(Joystick.Yaw, 1500)
        self.set_rc(Joystick.Forward, 1500)
        self.set_rc(Joystick.Lateral, 1100)

        self.wait_distance(75, accuracy=7, timeout=100)
        self.set_rc_default()

        self.disarm_vehicle()
        self.progress("Manual dive OK")

    def dive_mission(self, filename):
        self.progress("Executing mission %s" % filename)
        self.load_mission(filename)
        self.set_rc_default()

        self.arm_vehicle()

        self.mavproxy.send('mode auto\n')
        self.wait_mode('AUTO')

        self.wait_waypoint(1, 5, max_dist=5)

        self.disarm_vehicle()

        self.progress("Mission OK")

    def test_gripper_mission(self):
        self.context_push()
        ex = None
        try:
            try:
                self.get_parameter("GRIP_ENABLE", timeout=5)
            except NotAchievedException as e:
                self.progress("Skipping; Gripper not enabled in config?")
                return

            self.load_mission("sub-gripper-mission.txt")
            self.mavproxy.send('mode loiter\n')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.mavproxy.send('mode auto\n')
            self.wait_mode('AUTO')
            self.mavproxy.expect("Gripper Grabbed")
            self.mavproxy.expect("Gripper Released")
        except Exception as e:
            self.progress("Exception caught")
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def dive_set_position_target(self):
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        startpos = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                       blocking=True)

        lat = 5
        lon = 5
        alt = 10

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not move far enough")
            # send a position-control command
            self.mav.mav.set_position_target_global_int_send(
                0, # timestamp
                1, # target system_id
                1, # target component id
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b1111111111111000, # mask specifying use-only-lat-lon-alt
                lat, # lat
                lon, # lon
                alt, # alt
                0, # vx
                0, # vy
                0, # vz
                0, # afx
                0, # afy
                0, # afz
                0, # yaw
                0, # yawrate
            )
            pos = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                      blocking=True)
            delta = self.get_distance_int(startpos, pos)
            self.progress("delta=%f (want >10)" % delta)
            if delta > 10:
                break
        self.change_mode('MANUAL')
        self.disarm_vehicle()

    def reboot_sitl(self):
        """Reboot SITL instance and wait it to reconnect."""
        self.mavproxy.send("reboot\n")
        self.mavproxy.expect("Init ArduSub")
        # empty mav to avoid getting old timestamps:
        while self.mav.recv_match(blocking=False):
            pass
        self.initialise_after_reboot_sitl()

    def disabled_tests(self):
        ret = super(AutoTestSub, self).disabled_tests()
        ret.update({
            "SensorConfigErrorLoop": "Sub does not instantiate AP_Stats.  Also see https://github.com/ArduPilot/ardupilot/issues/10247",
        })
        return ret

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestSub, self).tests()

        ret.extend([
            ("DiveManual", "Dive manual", self.dive_manual),

            ("DiveMission",
             "Dive mission",
             lambda: self.dive_mission("sub_mission.txt")),

            ("GripperMission",
             "Test gripper mission items",
             self.test_gripper_mission),

            ("SET_POSITION_TARGET_GLOBAL_INT",
             "Move vehicle using SET_POSITION_TARGET_GLOBAL_INT",
             self.dive_set_position_target),

            ("DownLoadLogs", "Download logs", lambda:
             self.log_download(
                 self.buildlogs_path("ArduSub-log.bin"),
                 upload_logs=len(self.fail_list) > 0)),
        ])

        return ret
