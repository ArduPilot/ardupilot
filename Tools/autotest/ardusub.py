#!/usr/bin/env python

'''
Dive ArduSub in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
import os
import sys
import time

from pymavlink import mavutil

from common import AutoTest
from common import NotAchievedException
from common import AutoTestTimeoutException

if sys.version_info[0] < 3:
    ConnectionResetError = AutoTestTimeoutException

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

    def default_speedup(self):
        '''Sub seems to be race-free'''
        return 100

    def test_filepath(self):
        return os.path.realpath(__file__)

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduSub_Tests/" + name + "/"

    def default_mode(self):
        return 'MANUAL'

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_frame(self):
        return 'vectored'

    def is_sub(self):
        return True

    def watch_altitude_maintained(self, delta=1, timeout=5.0):
        """Watch and wait for the actual altitude to be maintained

        Keyword Arguments:
            delta {float} -- Maximum altitude range to be allowed from actual point (default: {0.5})
            timeout {float} -- Timeout time in simulation seconds (default: {5.0})

        Raises:
            NotAchievedException: Exception when altitude fails to hold inside the time and
                altitude range
        """
        tstart = self.get_sim_time_cached()
        previous_altitude = self.mav.recv_match(type='VFR_HUD', blocking=True).alt
        self.progress('Altitude to be watched: %f' % (previous_altitude))
        while True:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if self.get_sim_time_cached() - tstart > timeout:
                self.progress('Altitude hold done: %f' % (previous_altitude))
                return
            if abs(m.alt - previous_altitude) > delta:
                raise NotAchievedException(
                    "Altitude not maintained: want %.2f (+/- %.2f) got=%.2f" %
                    (previous_altitude, delta, m.alt))

    def test_alt_hold(self):
        """Test ALT_HOLD mode
        """
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('ALT_HOLD')

        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            raise NotAchievedException("Did not get GLOBAL_POSITION_INT")
        pwm = 1000
        if msg.relative_alt/1000.0 < -5.5:
            # need to g`o up, not down!
            pwm = 2000
        self.set_rc(Joystick.Throttle, pwm)
        self.wait_altitude(altitude_min=-6, altitude_max=-5)
        self.set_rc(Joystick.Throttle, 1500)

        # let the vehicle settle (momentum / stopping point shenanigans....)
        self.delay_sim_time(1)

        self.watch_altitude_maintained()

        self.set_rc(Joystick.Throttle, 1000)
        self.wait_altitude(altitude_min=-20, altitude_max=-19)
        self.set_rc(Joystick.Throttle, 1500)

        # let the vehicle settle (momentum / stopping point shenanigans....)
        self.delay_sim_time(1)

        self.watch_altitude_maintained()

        self.set_rc(Joystick.Throttle, 1900)
        self.wait_altitude(altitude_min=-14, altitude_max=-13)
        self.set_rc(Joystick.Throttle, 1500)

        # let the vehicle settle (momentum / stopping point shenanigans....)
        self.delay_sim_time(1)

        self.watch_altitude_maintained()

        self.set_rc(Joystick.Throttle, 1900)
        self.wait_altitude(altitude_min=-5, altitude_max=-4)
        self.set_rc(Joystick.Throttle, 1500)

        # let the vehicle settle (momentum / stopping point shenanigans....)
        self.delay_sim_time(1)

        self.watch_altitude_maintained()

        self.disarm_vehicle()

    def test_pos_hold(self):
        """Test POSHOLD mode"""
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # point North
        self.reach_heading_manual(0)
        self.change_mode('POSHOLD')

        # dive a little
        self.set_rc(Joystick.Throttle, 1300)
        self.delay_sim_time(3)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(2)

        # Save starting point
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            raise NotAchievedException("Did not get GLOBAL_POSITION_INT")
        start_pos = self.mav.location()
        # Hold in perfect conditions
        self.progress("Testing position hold in perfect conditions")
        self.delay_sim_time(10)
        distance_m = self.get_distance(start_pos, self.mav.location())
        if distance_m > 1:
            raise NotAchievedException("Position Hold was unable to keep position in calm waters within 1 meter after 10 seconds, drifted {} meters".format(distance_m))  # noqa

        # Hold in 1 m/s current
        self.progress("Testing position hold in current")
        self.set_parameter("SIM_WIND_SPD", 1)
        self.set_parameter("SIM_WIND_T", 1)
        self.delay_sim_time(10)
        distance_m = self.get_distance(start_pos, self.mav.location())
        if distance_m > 1:
            raise NotAchievedException("Position Hold was unable to keep position in 1m/s current within 1 meter after 10 seconds, drifted {} meters".format(distance_m))  # noqa

        # Move forward slowly in 1 m/s current
        start_pos = self.mav.location()
        self.progress("Testing moving forward in position hold in 1m/s current")
        self.set_rc(Joystick.Forward, 1600)
        self.delay_sim_time(10)
        distance_m = self.get_distance(start_pos, self.mav.location())
        bearing = self.get_bearing(start_pos, self.mav.location())
        if distance_m < 2 or (bearing > 30 and bearing < 330):
            raise NotAchievedException("Position Hold was unable to move north 2 meters, moved {} at {} degrees instead".format(distance_m, bearing))  # noqa
        self.disarm_vehicle()

    def test_mot_thst_hover_ignore(self):
        """Test if we are ignoring MOT_THST_HOVER parameter
        """

        # Test default parameter value
        mot_thst_hover_value = self.get_parameter("MOT_THST_HOVER")
        if mot_thst_hover_value != 0.5:
            raise NotAchievedException("Unexpected default MOT_THST_HOVER parameter value {}".format(mot_thst_hover_value))

        # Test if parameter is being ignored
        for value in [0.25, 0.75]:
            self.set_parameter("MOT_THST_HOVER", value)
            self.test_alt_hold()

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

        m = self.mav.recv_match(type='SCALED_PRESSURE3', blocking=True)
        if m is None:
            raise NotAchievedException("Did not get SCALED_PRESSURE3")
        if m.temperature != 2650:
            raise NotAchievedException("Did not get correct TSYS01 temperature")

    def dive_mission(self, filename):
        self.progress("Executing mission %s" % filename)
        self.load_mission(filename)
        self.set_rc_default()

        self.arm_vehicle()

        self.change_mode('AUTO')

        self.wait_waypoint(1, 5, max_dist=5)

        self.disarm_vehicle()

        self.progress("Mission OK")

    def test_gripper_mission(self):
        try:
            self.get_parameter("GRIP_ENABLE", timeout=5)
        except NotAchievedException:
            self.progress("Skipping; Gripper not enabled in config?")
            return

        self.load_mission("sub-gripper-mission.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_statustext("Gripper Grabbed", timeout=60)
        self.wait_statustext("Gripper Released", timeout=60)

    def dive_set_position_target(self):
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        startpos = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                       blocking=True)

        lat = 5
        lon = 5
        alt = -10

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

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not move far enough")
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
        # out battery is reset to full on reboot.  So reduce it to 10%
        # and wait for it to go above 50.
        self.run_cmd(mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                     255,  # battery mask
                     10,  # percentage
                     0,
                     0,
                     0,
                     0,
                     0,
                     0)
        self.run_cmd_reboot()
        tstart = time.time()
        while True:
            if time.time() - tstart > 30:
                raise NotAchievedException("Did not detect reboot")
            # ask for the message:
            batt = None
            try:
                self.send_cmd(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                              mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0)
                batt = self.mav.recv_match(type='BATTERY_STATUS',
                                           blocking=True,
                                           timeout=1)
            except ConnectionResetError:
                pass
            self.progress("Battery: %s" % str(batt))
            if batt is None:
                continue
            if batt.battery_remaining > 50:
                break
        self.initialise_after_reboot_sitl()

    def default_parameter_list(self):
        ret = super(AutoTestSub, self).default_parameter_list()
        ret["FS_GCS_ENABLE"] = 0  # FIXME
        return ret

    def disabled_tests(self):
        ret = super(AutoTestSub, self).disabled_tests()
        ret.update({
            "ConfigErrorLoop": "Sub does not instantiate AP_Stats.  Also see https://github.com/ArduPilot/ardupilot/issues/10247",  # noqa
        })
        return ret

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestSub, self).tests()

        ret.extend([
            ("DiveManual", "Dive manual", self.dive_manual),

            ("AltitudeHold", "Test altitude holde mode", self.test_alt_hold),
            ("PositionHold", "Test position hold mode", self.test_pos_hold),

            ("DiveMission",
             "Dive mission",
             lambda: self.dive_mission("sub_mission.txt")),

            ("GripperMission",
             "Test gripper mission items",
             self.test_gripper_mission),

            ("MotorThrustHoverParameterIgnore", "Test if we are ignoring MOT_THST_HOVER", self.test_mot_thst_hover_ignore),

            ("SET_POSITION_TARGET_GLOBAL_INT",
             "Move vehicle using SET_POSITION_TARGET_GLOBAL_INT",
             self.dive_set_position_target),

            ("TestLogDownloadMAVProxy",
             "Test Onboard Log Download using MAVProxy",
             self.test_log_download_mavproxy),

            ("LogUpload",
             "Upload logs",
             self.log_upload),
        ])

        return ret
