#!/usr/bin/env python

from __future__ import print_function

import math
import operator
import os

from pymavlink import mavextra
from pymavlink import mavutil

from common import AutoTest
from common import NotAchievedException

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

    def achieve_attitude(self, desyaw, despitch, tolerance=1, target_system=2, target_component=1):
        '''use set_attitude_target to achieve desyaw / despitch'''
        tstart = self.get_sim_time()
        last_attitude_target_sent = 0
        last_debug = 0
        self.progress("Using set_attitude_target to achieve attitude")
        while True:
            now = self.get_sim_time()
            if now - tstart > 60:
                raise NotAchievedException("Did not achieve attitude")
            if now - last_attitude_target_sent > 0.5:
                last_attitude_target_sent = now
                type_mask = (
                    1 << 0 | # ignore roll rate
                    1 << 6 # ignore throttle
                )
                self.mav.mav.set_attitude_target_send(
                    0, # time_boot_ms
                    target_system, # target sysid
                    target_component, # target compid
                    type_mask, # bitmask of things to ignore
                    mavextra.euler_to_quat([0,
                                            math.radians(despitch),
                                            math.radians(desyaw)]), # att
                    0, # yaw rate (rad/s)
                    0, # pitch rate
                    0, # yaw rate
                    0) # thrust, 0 to 1, translated to a climb/descent rate
            m = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=2)
            if m is None:
                raise NotAchievedException("Did not get ATTITUDE")
            if now - last_debug > 1:
                last_debug = now
                self.progress("yaw=%f desyaw=%f pitch=%f despitch=%f" %
                              (math.degrees(m.yaw), desyaw,
                               math.degrees(m.pitch), despitch))
            yaw_ok = abs(math.degrees(m.yaw) - desyaw) < tolerance
            pitch_ok = abs(math.degrees(m.pitch) - despitch) < tolerance
            if yaw_ok and pitch_ok:
                self.progress("Achieved attitude")
                break

    def GUIDED(self):
        self.change_mode(4) # "GUIDED"
        self.achieve_attitude(desyaw=10, despitch=30)
        self.achieve_attitude(desyaw=0, despitch=0)
        self.achieve_attitude(desyaw=45, despitch=10)

    def MANUAL(self):
        self.change_mode(0) # "MANUAL"
        for chan in 1, 2:
            for pwm in 1200, 1600, 1367:
                self.set_rc(chan, pwm);
                self.wait_servo_channel_value(chan, pwm)

    def SERVOTEST(self):
        self.change_mode(0) # "MANUAL"
        # magically changes to SERVOTEST (3)
        for value in 1900, 1200:
            channel = 1
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                         channel,
                         value,
                         0,
                         0,
                         0,
                         0,
                         0,
                         timeout=1)
            self.wait_servo_channel_value(channel, value)
        for value in 1300, 1670:
            channel = 2
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                         channel,
                         value,
                         0,
                         0,
                         0,
                         0,
                         0,
                         timeout=1)
            self.wait_servo_channel_value(channel, value)

    def SCAN(self):
        self.change_mode(2) # "SCAN"
        self.set_parameter("SCAN_SPEED_YAW", 20)
        for channel in 1, 2:
            self.wait_servo_channel_value(channel,
                                          1900,
                                          timeout=90,
                                          comparator=operator.ge)
        for channel in 1, 2:
            self.wait_servo_channel_value(channel,
                                          1200,
                                          timeout=90,
                                          comparator=operator.le)

    def disabled_tests(self):
        return {
            "ArmFeatures": "See https://github.com/ArduPilot/ardupilot/issues/10652",
            "Parameters": "reboot does not work",
        }

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestTracker, self).tests()
        ret.extend([
            ("GUIDED",
             "Test GUIDED mode",
             self.GUIDED),

            ("MANUAL",
             "Test MANUAL mode",
             self.MANUAL),

            ("SERVOTEST",
             "Test SERVOTEST mode",
             self.SERVOTEST),

            ("NMEAOutput",
             "Test AHRS NMEA Output can be read by out NMEA GPS",
             self.nmea_output),

            ("SCAN",
             "Test SCAN mode",
             self.SCAN),
        ])
        return ret
