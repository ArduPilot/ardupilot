'''
Dive ArduSub in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
import os
import sys
import time

from pymavlink import mavutil

import vehicle_test_suite
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import AutoTestTimeoutException

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


class AutoTestSub(vehicle_test_suite.TestSuite):
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

    def watch_altitude_maintained(self, delta=0.3, timeout=5.0):
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

    def AltitudeHold(self):
        """Test ALT_HOLD mode"""
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('ALT_HOLD')

        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            raise NotAchievedException("Did not get GLOBAL_POSITION_INT")
        pwm = 1300
        if msg.relative_alt/1000.0 < -6.0:
            # need to g`o up, not down!
            pwm = 1700
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

        # Make sure the code can handle buoyancy changes
        self.set_parameter("SIM_BUOYANCY", 10)
        self.watch_altitude_maintained()
        self.set_parameter("SIM_BUOYANCY", -10)
        self.watch_altitude_maintained()

        # Make sure that the ROV will dive with a small input down even if there is a 10N buoyancy force upwards
        self.set_parameter("SIM_BUOYANCY", 10)
        self.set_rc(Joystick.Throttle, 1350)
        self.wait_altitude(altitude_min=-6, altitude_max=-5.5)

        self.set_rc(Joystick.Throttle, 1500)
        self.watch_altitude_maintained()
        self.disarm_vehicle()

    def RngfndQuality(self):
        """Check lua Range Finder quality information flow"""
        self.context_push()
        self.context_collect('STATUSTEXT')

        ex = None
        try:
            self.set_parameters({
                "SCR_ENABLE": 1,
                "RNGFND1_TYPE": 36,
                "RNGFND1_ORIENT": 25,
                "RNGFND1_MIN_CM": 10,
                "RNGFND1_MAX_CM": 5000,
            })

            self.install_example_script_context("rangefinder_quality_test.lua")

            # These string must match those sent by the lua test script.
            complete_str = "#complete#"
            failure_str = "!!failure!!"

            self.reboot_sitl()

            self.wait_statustext(complete_str, timeout=20, check_context=True)
            found_failure = self.statustext_in_collections(failure_str)

            if found_failure is not None:
                raise NotAchievedException("RngfndQuality test failed: " + found_failure.text)

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()

        if ex:
            raise ex

    def ModeChanges(self, delta=0.2):
        """Check if alternating between ALTHOLD, STABILIZE and POSHOLD affects altitude"""
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # zero buoyancy and no baro noise
        self.set_parameter("SIM_BUOYANCY", 0)
        self.set_parameter("SIM_BARO_RND", 0)
        # dive a bit to make sure we are not surfaced
        self.change_mode('STABILIZE')
        self.set_rc(Joystick.Throttle, 1350)
        self.delay_sim_time(10)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(3)
        # start the test itself, go through some modes and check if anything changes
        previous_altitude = self.mav.recv_match(type='VFR_HUD', blocking=True).alt
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2)
        self.change_mode('POSHOLD')
        self.delay_sim_time(2)
        self.change_mode('STABILIZE')
        self.delay_sim_time(2)
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2)
        self.change_mode('STABILIZE')
        self.delay_sim_time(2)
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2)
        self.change_mode('MANUAL')
        self.disarm_vehicle()
        final_altitude = self.mav.recv_match(type='VFR_HUD', blocking=True).alt
        if abs(previous_altitude - final_altitude) > delta:
            raise NotAchievedException(
                "Changing modes affected depth with no throttle input!, started at {}, ended at {}"
                .format(previous_altitude, final_altitude)
            )

    def PositionHold(self):
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

    def MotorThrustHoverParameterIgnore(self):
        """Test if we are ignoring MOT_THST_HOVER parameter"""

        # Test default parameter value
        mot_thst_hover_value = self.get_parameter("MOT_THST_HOVER")
        if mot_thst_hover_value != 0.5:
            raise NotAchievedException("Unexpected default MOT_THST_HOVER parameter value {}".format(mot_thst_hover_value))

        # Test if parameter is being ignored
        for value in [0.25, 0.75]:
            self.set_parameter("MOT_THST_HOVER", value)
            self.AltitudeHold()

    def DiveManual(self):
        '''Dive manual'''
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

        m = self.assert_receive_message('SCALED_PRESSURE3')

        # Note this temperature matches the output of the Atmospheric Model for Air currently
        # And should be within 1 deg C of 40 degC
        if (m.temperature < 3900) or (4100 < m.temperature):
            raise NotAchievedException("Did not get correct TSYS01 temperature: Got %f" % m.temperature)

    def DiveMission(self):
        '''Dive mission'''
        filename = "sub_mission.txt"
        self.progress("Executing mission %s" % filename)
        self.load_mission(filename)
        self.set_rc_default()

        self.arm_vehicle()

        self.change_mode('AUTO')

        self.wait_waypoint(1, 5, max_dist=5)

        self.disarm_vehicle()

        self.progress("Mission OK")

    def GripperMission(self):
        '''Test gripper mission items'''
        self.load_mission("sub-gripper-mission.txt")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, 2, max_dist=5)
        self.wait_statustext("Gripper Grabbed", timeout=60)
        self.wait_waypoint(1, 4, max_dist=5)
        self.wait_statustext("Gripper Released", timeout=60)
        self.wait_waypoint(1, 6, max_dist=5)
        self.disarm_vehicle()

    def SET_POSITION_TARGET_GLOBAL_INT(self):
        '''Move vehicle using SET_POSITION_TARGET_GLOBAL_INT'''
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
        # our battery is reset to full on reboot.  So reduce it to 10%
        # and wait for it to go above 50.
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_BATTERY_RESET,
            p1=65535,   # battery mask
            p2=10,      # percentage
        )
        self.run_cmd_reboot()
        tstart = time.time()
        while True:
            if time.time() - tstart > 30:
                raise NotAchievedException("Did not detect reboot")
            # ask for the message:
            batt = None
            try:
                self.send_cmd(
                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                    p1=mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
                )
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

    def DoubleCircle(self):
        '''Test entering circle twice'''
        self.change_mode('CIRCLE')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('STABILIZE')
        self.change_mode('CIRCLE')
        self.disarm_vehicle()

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

    def MAV_CMD_NAV_LOITER_UNLIM(self):
        '''test handling of MAV_CMD_NAV_LOITER_UNLIM received via mavlink'''
        for cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('CIRCLE')
            cmd(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
            self.assert_mode('POSHOLD')

    def MAV_CMD_NAV_LAND(self):
        '''test handling of MAV_CMD_NAV_LAND received via mavlink'''
        for cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('CIRCLE')
            cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
            self.assert_mode('SURFACE')

    def MAV_CMD_MISSION_START(self):
        '''test handling of MAV_CMD_NAV_LAND received via mavlink'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        self.wait_ready_to_arm()
        self.arm_vehicle()
        for cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('CIRCLE')
            cmd(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.assert_mode('AUTO')
        self.disarm_vehicle()

    def MAV_CMD_DO_CHANGE_SPEED(self):
        '''ensure vehicle changes speeds when DO_CHANGE_SPEED received'''
        items = [
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, -3),  # Dive so we have constrat drag
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, -1),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ]
        self.upload_simple_relhome_mission(items)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_MISSION_START)
        self.progress("SENT MISSION START")
        self.wait_mode('AUTO')
        self.wait_current_waypoint(2)  # wait after we finish diving to 3m
        for run_cmd in self.run_cmd, self.run_cmd_int:
            for speed in [1, 1.5, 0.5]:
                run_cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, p2=speed)
                self.wait_groundspeed(speed-0.2, speed+0.2, minimum_duration=2, timeout=60)
        self.disarm_vehicle()

    def _MAV_CMD_CONDITION_YAW(self, run_cmd):
        self.arm_vehicle()
        self.change_mode('GUIDED')
        for angle in 5, 30, 60, 10:
            angular_rate = 10
            direction = 1
            relative_or_absolute = 0
            run_cmd(
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                p1=angle,
                p2=angular_rate,
                p3=direction,
                p4=relative_or_absolute,  # 1 for relative, 0 for absolute
            )
            self.wait_heading(angle, minimum_duration=2)

        self.start_subtest('Relative angle')
        run_cmd(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=0,
            p2=10,
            p3=1,
            p4=0,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(0, minimum_duration=2)
        run_cmd(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=20,
            p2=10,
            p3=1,
            p4=1,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(20, minimum_duration=2)

        self.disarm_vehicle()

    def MAV_CMD_CONDITION_YAW(self):
        '''ensure vehicle yaws according to GCS command'''
        self._MAV_CMD_CONDITION_YAW(self.run_cmd)
        self._MAV_CMD_CONDITION_YAW(self.run_cmd_int)

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestSub, self).tests()

        ret.extend([
            self.DiveManual,
            self.AltitudeHold,
            self.RngfndQuality,
            self.PositionHold,
            self.ModeChanges,
            self.DiveMission,
            self.GripperMission,
            self.DoubleCircle,
            self.MotorThrustHoverParameterIgnore,
            self.SET_POSITION_TARGET_GLOBAL_INT,
            self.TestLogDownloadMAVProxy,
            self.TestLogDownloadMAVProxyNetwork,
            self.MAV_CMD_NAV_LOITER_UNLIM,
            self.MAV_CMD_NAV_LAND,
            self.MAV_CMD_MISSION_START,
            self.MAV_CMD_DO_CHANGE_SPEED,
            self.MAV_CMD_CONDITION_YAW,
        ])

        return ret
