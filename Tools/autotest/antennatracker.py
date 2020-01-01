#!/usr/bin/env python

'''
Test AntennaTracker vehicle in SITL

AP_FLAKE8_CLEAN

'''

from __future__ import print_function

import math
import operator
import os

from pymavlink import mavextra
from pymavlink import mavutil

from common import AutoTest
from common import NotAchievedException
from common import AutoTestTimeoutException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-27.274439, 151.290064, 343, 8.7)


class AutoTestTracker(AutoTest):

    def log_name(self):
        return "AntennaTracker"

    def default_speedup(self):
        '''Tracker seems to be race-free'''
        return 100

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

    def set_current_test_name(self, name):
        self.current_test_name_directory = "AntennaTracker_Tests/" + name + "/"

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

    def reboot_sitl(self, *args, **kwargs):
        self.disarm_vehicle()
        super(AutoTestTracker, self).reboot_sitl(*args, **kwargs)

    def GUIDED(self):
        self.reboot_sitl() # temporary hack around control issues
        self.change_mode(4) # "GUIDED"
        self.achieve_attitude(desyaw=10, despitch=30)
        self.achieve_attitude(desyaw=0, despitch=0)
        self.achieve_attitude(desyaw=45, despitch=10)

    def MANUAL(self):
        self.change_mode(0) # "MANUAL"
        for chan in 1, 2:
            for pwm in 1200, 1600, 1367:
                self.set_rc(chan, pwm)
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

    def send_vehicle_position(self, loc, vehicle_pressure):
        # send global-position-int and scaled-pressure messages
        self.mav.mav.global_position_int_send(
            0, # time_boot_ms
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            int(loc.alt * 1000),
            0, # relative alt....
            0, # Ground X Speed
            0, # Ground Y Speed
            0, # Ground Z Speed
            65535 # HDG
        )
        self.mav.mav.scaled_pressure_send(
            0, # time_boot_ms
            vehicle_pressure["pressure"], # press-abs
            0, # press-diff
            vehicle_pressure["temperature"] # temperature
        )

    def tracking(self):
        self.progress("SERVO1_MIN: %f" % self.get_parameter("SERVO1_MIN"))
        self.progress("YAW_RANGE: %f" % self.get_parameter("YAW_RANGE"))
        self.set_parameter("SYSID_TARGET", 0)
        self.delay_sim_time(100)
        self.change_mode("AUTO")
        self.wait_ready_to_arm()

        vehicle_gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True)
        # remember that the simulated vehicle's zero is at heading 270
        vehicle_locs = [
            # ((n, e, d), (p, hdg))
            ((0, -20, 0), (0, -90)),
            ((20, -20, 0), (0, -45)),
            ((-20, -20, 0), (0, -135)),
            ((30, -40, 0), (0, -53)),
            ((-30, -40, 0), (0, -126)),
            # ((20, 0, 0), (0, 0)),
            # ((20, 20, -20), (45, 45)),
            # ((20, 30, -20), (61, 43)),
            # ((20, 90, -20), (-11, 85)),
        ]
        for vehicle_loc in vehicle_locs:
            ((vehicle_loc_offset_n,
              vehicle_loc_offset_e,
              vehicle_loc_offset_d),
             (despitch, desyaw)) = vehicle_loc
            self.start_subtest("New vehicle pos: %f,%f alt=%f" %
                               (vehicle_gpi.lat, vehicle_gpi.lon, vehicle_gpi.alt))
            vehicle_loc = mavutil.location(
                vehicle_gpi.lat * 1.0e-7,
                vehicle_gpi.lon * 1.0e-7,
                vehicle_gpi.alt * 1e-3 - vehicle_loc_offset_d
            )
            if self.mavproxy is not None:
                self.mavproxy.send("map icon %f %f hoop\n" %
                                   (vehicle_loc.lat, vehicle_loc.lng))
            self.location_offset_ne(vehicle_loc, vehicle_loc_offset_n, vehicle_loc_offset_e)
            vehicle_sp = self.mav.recv_match(type='SCALED_PRESSURE',
                                             blocking=True)
            vehicle_pressure = {
                "pressure": vehicle_sp.press_abs,
                "temperature": vehicle_sp.temperature,
            }
            tstart = self.get_sim_time()
            last_vehicle_position_message_sent = tstart
            while True:
                now = self.get_sim_time_cached()
                if now - last_vehicle_position_message_sent > 0.1:
                    self.send_vehicle_position(vehicle_loc, vehicle_pressure)
                if now - tstart > 3000:
                    raise NotAchievedException("Did not point at vehicle")
                ex = None
                try:
                    self.wait_attitude(desyaw=desyaw,
                                       despitch=despitch,
                                       timeout=2,
                                       tolerance=2)
                except AutoTestTimeoutException as e:
                    ex = e
                if ex is None:
                    # success
                    break

    def disabled_tests(self):
        return {
            "ArmFeatures": "See https://github.com/ArduPilot/ardupilot/issues/10652",
            "CPUFailsafe": " tracker doesn't have a CPU failsafe",
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

            ("Tracking",
             "Test antenna tracking vehicle",
             self.tracking),

            ("NMEAOutput",
             "Test AHRS NMEA Output can be read by out NMEA GPS",
             self.nmea_output),

            ("SCAN",
             "Test SCAN mode",
             self.SCAN),
        ])
        return ret
