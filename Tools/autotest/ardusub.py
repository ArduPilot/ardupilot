'''
Dive ArduSub in SITL

Depth of water is 50m, the ground is flat
Parameters are in-code defaults plus default_params/sub.parm

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
import os
import sys

from pymavlink import mavutil

import vehicle_test_suite
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import PreconditionFailedException

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


# Values for EK3_MAG_CAL
class MagCal():
    WHEN_FLYING = 0
    WHEN_MANOEUVRING = 1
    NEVER = 2
    AFTER_FIRST_CLIMB = 3
    ALWAYS = 4


# Values for XKFS.MAG_FUSION
class MagFuseSel():
    NOT_FUSING = 0
    FUSE_YAW = 1
    FUSE_MAG = 2


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
            # need to go up, not down!
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

        # restart SITL RF driver
        self.reboot_sitl()

        if ex:
            raise ex

    def watch_distance_maintained(self, delta=0.3, timeout=5.0):
        """Watch and wait for the rangefinder reading to be maintained"""
        tstart = self.get_sim_time_cached()
        previous_distance = self.mav.recv_match(type='RANGEFINDER', blocking=True).distance
        self.progress('Distance to be watched: %.2f' % previous_distance)
        while True:
            m = self.mav.recv_match(type='RANGEFINDER', blocking=True)
            if self.get_sim_time_cached() - tstart > timeout:
                self.progress('Distance hold done: %f' % previous_distance)
                return
            if abs(m.distance - previous_distance) > delta:
                raise NotAchievedException(
                    "Distance not maintained: want %.2f (+/- %.2f) got=%.2f" %
                    (previous_distance, delta, m.distance))

    def Surftrak(self):
        """Test SURFTRAK mode"""

        if self.get_parameter('RNGFND1_MAX_CM') != 3000.0:
            raise PreconditionFailedException("RNGFND1_MAX_CM is not %g" % 3000.0)

        # Something closer to Bar30 noise
        self.context_push()
        self.set_parameter("SIM_BARO_RND", 0.01)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('MANUAL')

        # Dive to -5m, outside of rangefinder range, will act like ALT_HOLD
        pwm = 1300 if self.get_altitude(relative=True) > -6 else 1700
        self.set_rc(Joystick.Throttle, pwm)
        self.wait_altitude(altitude_min=-6, altitude_max=-5, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(1)
        self.context_collect('STATUSTEXT')
        self.change_mode(21)
        self.wait_statustext('waiting for a rangefinder reading', check_context=True)
        self.context_clear_collection('STATUSTEXT')
        self.watch_altitude_maintained()

        # Move into range, should set a rangefinder target and maintain it
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=-26, altitude_max=-25, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(4)
        self.wait_statustext('rangefinder target is', check_context=True)
        self.context_clear_collection('STATUSTEXT')
        self.watch_distance_maintained()

        # Move a few meters, should apply a delta and maintain the new rangefinder target
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=-31, altitude_max=-30, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(4)
        self.wait_statustext('rangefinder target is', check_context=True)
        self.watch_distance_maintained()

        self.disarm_vehicle()
        self.context_pop()

    def prepare_synthetic_seafloor_test(self, sea_floor_depth, rf_target):
        self.set_parameters({
            "SCR_ENABLE": 1,
            "RNGFND1_TYPE": 36,
            "RNGFND1_ORIENT": 25,
            "RNGFND1_MIN_CM": 10,
            "RNGFND1_MAX_CM": 3000,
            "SCR_USER1": 2,                 # Configuration bundle
            "SCR_USER2": sea_floor_depth,   # Depth in meters
            "SCR_USER3": 101,               # Output log records
            "SCR_USER4": rf_target,         # Rangefinder target in meters
        })

        self.install_example_script_context("sub_test_synthetic_seafloor.lua")

        # Reboot to enable scripting.
        self.reboot_sitl()
        self.set_rc_default()
        self.wait_ready_to_arm()

    def watch_true_distance_maintained(self, match_distance, delta=0.3, timeout=5.0, final_waypoint=0):
        """Watch and wait for the rangefinder reading to be maintained"""

        def get_true_distance():
            """Return the True distance from the simulated range finder"""
            m_true = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=3.0)
            if m_true is None:
                return m_true
            idx_tr = m_true.text.find('#TR#')
            if idx_tr < 0:
                return None
            return float(m_true.text[(idx_tr+4):(idx_tr+12)])

        tstart = self.get_sim_time_cached()
        self.progress('Distance to be watched: %.2f (+/- %.2f)' % (match_distance, delta))
        max_delta = 0.0

        while True:
            timed_out = self.get_sim_time_cached() - tstart > timeout
            # If final_waypoint>0 then timeout is failure, otherwise success
            if timed_out and final_waypoint > 0:
                raise NotAchievedException(
                    "Mission not complete: want waypoint %i, only made it to waypoint %i" %
                    (final_waypoint, self.mav.waypoint_current()))
            if timed_out:
                self.progress('Distance hold done. Max delta:%.2fm' % max_delta)
                return

            true_distance = get_true_distance()
            if true_distance is None:
                continue
            match_delta = abs(true_distance - match_distance)
            if match_delta > max_delta:
                max_delta = match_delta
            if match_delta > delta:
                raise NotAchievedException(
                    "Distance not maintained: want %.2f (+/- %.2f) got=%.2f (%.2f)" %
                    (match_distance, delta, true_distance, match_delta))
            if final_waypoint > 0:
                if self.mav.waypoint_current() >= final_waypoint:
                    self.progress('Distance hold during mission done. Max delta:%.2fm' % max_delta)
                    return

    def SimTerrainSurftrak(self):
        """Move at a constant height above synthetic sea floor"""

        sea_floor_depth = 50    # Depth of sea floor at location of test
        match_distance = 15     # Desired sub distance from sea floor
        start_altitude = -sea_floor_depth+match_distance
        end_altitude = start_altitude - 10
        validation_delta = 1.5  # Largest allowed distance between sub height and desired height

        self.context_push()
        self.prepare_synthetic_seafloor_test(sea_floor_depth, match_distance)
        self.change_mode('MANUAL')
        self.arm_vehicle()

        # Dive to match_distance off the bottom in preparation for the mission
        pwm = 1300 if self.get_altitude(relative=True) > start_altitude else 1700
        self.set_rc(Joystick.Throttle, pwm)
        self.wait_altitude(altitude_min=start_altitude-1, altitude_max=start_altitude, relative=False, timeout=120)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(1)

        # Turn on surftrak and move around
        self.change_mode(21)

        # Go south over the ridge.
        self.reach_heading_manual(180)
        self.set_rc(Joystick.Forward, 1650)
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=60)
        self.set_rc(Joystick.Forward, 1500)

        # Shift west a bit
        self.reach_heading_manual(270)
        self.set_rc(Joystick.Forward, 1650)
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=5)
        self.set_rc(Joystick.Forward, 1500)

        # Go south over the plateau
        self.reach_heading_manual(180)
        self.set_rc(Joystick.Forward, 1650)
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=60)

        # The mission ends at end_altitude. Do a check to ensure that the sub is at this altitude
        self.wait_altitude(altitude_min=end_altitude-validation_delta/2, altitude_max=end_altitude+validation_delta/2,
                           relative=False, timeout=1)

        self.set_rc(Joystick.Forward, 1500)

        self.disarm_vehicle()
        self.context_pop()
        self.reboot_sitl()  # e.g. revert rangefinder configuration

    def SimTerrainMission(self):
        """Mission at a constant height above synthetic sea floor"""

        sea_floor_depth = 50    # Depth of sea floor at location of test
        match_distance = 15     # Desired sub distance from sea floor
        start_altitude = -sea_floor_depth+match_distance
        end_altitude = start_altitude - 10
        validation_delta = 1.5  # Largest allowed distance between sub height and desired height

        self.context_push()
        self.prepare_synthetic_seafloor_test(sea_floor_depth, match_distance)

        # The synthetic seafloor has an east-west ridge south of the sub.
        # The mission contained in terrain_mission.txt instructs the sub
        # to remain at 15m above the seafloor and travel south over the
        # ridge. Then the sub moves west and travels north over the ridge.
        filename = "terrain_mission.txt"
        self.load_mission(filename)

        self.change_mode('MANUAL')
        self.arm_vehicle()

        # Dive to match_distance off the bottom in preparation for the mission
        pwm = 1300 if self.get_altitude(relative=True) > start_altitude else 1700
        self.set_rc(Joystick.Throttle, pwm)
        self.wait_altitude(altitude_min=start_altitude-1, altitude_max=start_altitude, relative=False, timeout=120)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(1)

        self.change_mode('AUTO')
        self.watch_true_distance_maintained(match_distance, delta=validation_delta, timeout=500.0, final_waypoint=4)

        # The mission ends at end_altitude. Do a check to ensure that the sub is at this altitude.
        self.wait_altitude(altitude_min=end_altitude-validation_delta/2, altitude_max=end_altitude+validation_delta/2,
                           relative=False, timeout=1)

        self.disarm_vehicle()
        self.context_pop()
        self.reboot_sitl()  # e.g. revert rangefinder configuration

    def ModeChanges(self, delta=0.2):
        """Check if alternating between ALTHOLD, STABILIZE, POSHOLD and SURFTRAK (mode 21) affects altitude"""
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
        self.change_mode(21)
        self.delay_sim_time(2)
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2)
        self.change_mode('STABILIZE')
        self.delay_sim_time(2)
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2)
        self.change_mode(21)
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
        '''test handling of MAV_CMD_MISSION_START received via mavlink'''
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
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, -3),  # Dive so we have constant drag
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

    def MAV_CMD_DO_REPOSITION(self):
        """Move vehicle using MAV_CMD_DO_REPOSITION"""
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Dive so that rangefinder is in range, required for MAV_FRAME_GLOBAL_TERRAIN_ALT
        start_altitude = -25
        pwm = 1300 if self.get_altitude(relative=True) > start_altitude else 1700
        self.set_rc(Joystick.Throttle, pwm)
        self.wait_altitude(altitude_min=start_altitude-1, altitude_max=start_altitude, relative=False, timeout=120)
        self.set_rc(Joystick.Throttle, 1500)
        self.change_mode('GUIDED')

        loc = self.mav.location()

        # Reposition, alt relative to surface
        loc = self.offset_location_ne(loc, 10, 10)
        loc.alt = start_altitude
        self.send_do_reposition(loc, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.wait_location(loc, timeout=120)

        # Reposition, alt relative to seafloor
        loc = self.offset_location_ne(loc, 10, 10)
        loc.alt = -start_altitude
        self.send_do_reposition(loc, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.wait_location(loc, timeout=120)

        self.disarm_vehicle()

    def TerrainMission(self):
        """Mission using surface tracking"""

        if self.get_parameter('RNGFND1_MAX_CM') != 3000.0:
            raise PreconditionFailedException("RNGFND1_MAX_CM is not %g" % 3000.0)

        filename = "terrain_mission.txt"
        self.progress("Executing mission %s" % filename)
        self.load_mission(filename)
        self.set_rc_default()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, 4, max_dist=5)
        self.delay_sim_time(3)

        # Expect sub to hover at final altitude
        self.assert_altitude(-36.0)

        self.disarm_vehicle()
        self.progress("Mission OK")

    def wait_ekf_happy_const_pos(self, timeout=45):
        # All of these must be set for arming to happen in constant position mode:
        required_value = (mavutil.mavlink.EKF_ATTITUDE |
                          mavutil.mavlink.EKF_VELOCITY_HORIZ |
                          mavutil.mavlink.EKF_VELOCITY_VERT |
                          mavutil.mavlink.EKF_POS_VERT_ABS |
                          mavutil.mavlink.EKF_CONST_POS_MODE)

        # None of these bits must be set for arming to happen:
        error_bits = mavutil.mavlink.EKF_UNINITIALIZED

        self.wait_ekf_flags(required_value, error_bits, timeout=timeout)

    def wait_ready_to_arm_const_pos(self, timeout=120):
        self.progress("Waiting for ready to arm (constant position mode)")
        start = self.get_sim_time()
        self.wait_ekf_happy_const_pos(timeout=timeout)
        armable_time = self.get_sim_time() - start
        self.progress("Took %u seconds to become armable" % armable_time)
        self.total_waiting_to_arm_time += armable_time
        self.waiting_to_arm_count += 1

    def collected_msgs(self, msg_type):
        c = self.context_get()
        if msg_type not in c.collections:
            raise NotAchievedException("Not collecting (%s)" % str(msg_type))
        return c.collections[msg_type]

    def SetGlobalOrigin(self):
        """Test SET_GPS_GLOBAL_ORIGIN mav msg"""
        self.context_push()
        self.set_parameters({
            'GPS1_TYPE': 0,             # Disable the GPS
            'EK3_SRC1_POSXY': 0,        # Make sure EK3_SRC parameters do not refer to a GPS
        })
        self.reboot_sitl()

        # Wait for the EKF to be happy in constant position mode
        self.wait_ready_to_arm_const_pos()

        if self.current_onboard_log_contains_message('ORGN'):
            raise NotAchievedException("Found unexpected ORGN message")

        self.context_collect('GPS_GLOBAL_ORIGIN')

        # This should set the EKF origin, write an ORGN msg to df and a GPS_GLOBAL_ORIGIN msg to MAVLink
        self.mav.mav.set_gps_global_origin_send(1, int(47.607584 * 1e7), int(-122.343911 * 1e7), 0)
        self.delay_sim_time(1)

        if not self.current_onboard_log_contains_message('ORGN'):
            raise NotAchievedException("Did not find expected ORGN message")

        num_mavlink_origin_msgs = len(self.collected_msgs('GPS_GLOBAL_ORIGIN'))
        if num_mavlink_origin_msgs != 1:
            raise NotAchievedException("Expected 1 GPS_GLOBAL_ORIGIN message, found %d" % num_mavlink_origin_msgs)

        self.context_pop()

        # restart GPS driver
        self.reboot_sitl()

    def BackupOrigin(self):
        """Test ORIGIN_LAT and ORIGIN_LON parameters"""

        self.context_push()
        self.set_parameters({
            'GPS1_TYPE': 0,              # Disable GPS
            'EK3_SRC1_POSXY': 0,        # Make sure EK3_SRC parameters do not refer to GPS
            'EK3_SRC1_VELXY': 0,        # Make sure EK3_SRC parameters do not refer to GPS
            'ORIGIN_LAT': 47.607584,
            'ORIGIN_LON': -122.343911,
        })
        self.reboot_sitl()
        self.context_collect('STATUSTEXT')

        # Wait for the EKF to be happy in constant position mode
        self.wait_ready_to_arm_const_pos()

        if self.current_onboard_log_contains_message('ORGN'):
            raise NotAchievedException("Found unexpected ORGN message")

        # This should set the origin and write a record to ORGN
        self.arm_vehicle()

        self.wait_statustext('Using backup location', check_context=True)

        if not self.current_onboard_log_contains_message('ORGN'):
            raise NotAchievedException("Did not find expected ORGN message")

        self.disarm_vehicle()
        self.context_pop()

    def assert_mag_fusion_selection(self, expect_sel):
        """Get the most recent XKFS message and check the MAG_FUSION value"""
        self.progress("Expect mag fusion selection %d" % expect_sel)
        mlog = self.dfreader_for_current_onboard_log()
        found_sel = MagFuseSel.NOT_FUSING
        while True:
            m = mlog.recv_match(type='XKFS')
            if m is None:
                break
            found_sel = m.MAG_FUSION
        if found_sel != expect_sel:
            raise NotAchievedException("Expected mag fusion selection %d, found %d" % (expect_sel, found_sel))

    def FuseMag(self):
        """Test EK3_MAG_CAL values"""

        # WHEN_FLYING: switch to FUSE_MAG after sub is armed for 5 seconds; switch to FUSE_YAW on disarm
        self.set_parameters({'EK3_MAG_CAL': MagCal.WHEN_FLYING})
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)
        self.arm_vehicle()
        self.delay_sim_time(10)
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG)
        self.disarm_vehicle()
        self.delay_sim_time(1)
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)

        # AFTER_FIRST_CLIMB: switch to FUSE_MAG after sub is armed and descends 0.5m; switch to FUSE_YAW on disarm
        self.set_parameters({'EK3_MAG_CAL': MagCal.AFTER_FIRST_CLIMB})
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)
        altitude = self.get_altitude(relative=True)
        self.arm_vehicle()
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=altitude-4, altitude_max=altitude-3, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG)
        self.disarm_vehicle()
        self.delay_sim_time(1)
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)

        # ALWAYS
        self.set_parameters({'EK3_MAG_CAL': MagCal.ALWAYS})
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG)

    def wait_for_stop(self):
        """Watch the sub slow down and stop"""
        tstart = self.get_sim_time_cached()
        lstart = self.mav.location()

        dmax = 0
        dprev = 0

        while True:
            self.delay_sim_time(1)

            dcurr = self.get_distance(lstart, self.mav.location())

            if dcurr - dmax < -0.2:
                raise NotAchievedException("Bounced back from %.2fm to %.2fm" % (dmax, dcurr))
            if dcurr > dmax:
                dmax = dcurr

            if abs(dcurr - dprev) < 0.1:
                self.progress("Stopping distance %.2fm, less than %.2fs" % (dcurr, self.get_sim_time_cached() - tstart))
                return

            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Took to long to stop")

            dprev = dcurr

    def PosHoldBounceBack(self):
        """Test for bounce back in POSHOLD mode"""
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # dive a little
        self.set_rc(Joystick.Throttle, 1300)
        self.delay_sim_time(3)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(2)

        # hold position
        self.change_mode('POSHOLD')

        for pilot_speed in range(50, 251, 100):
            # set max speed
            self.set_parameter('PILOT_SPEED', pilot_speed)

            # try different stick values, resulting speed is ~ max_speed * effort * gain
            for pwm in range(1700, 1901, 100):
                self.progress('PILOT_SPEED %d, forward pwm %d' % (pilot_speed, pwm))
                self.set_rc(Joystick.Forward, pwm)
                self.delay_sim_time(3)
                self.set_rc(Joystick.Forward, 1500)
                self.wait_for_stop()

        self.disarm_vehicle()

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestSub, self).tests()

        ret.extend([
            self.DiveManual,
            self.AltitudeHold,
            self.Surftrak,
            self.SimTerrainSurftrak,
            self.SimTerrainMission,
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
            self.TestLogDownloadLogRestart,
            self.MAV_CMD_NAV_LOITER_UNLIM,
            self.MAV_CMD_NAV_LAND,
            self.MAV_CMD_MISSION_START,
            self.MAV_CMD_DO_CHANGE_SPEED,
            self.MAV_CMD_CONDITION_YAW,
            self.MAV_CMD_DO_REPOSITION,
            self.TerrainMission,
            self.SetGlobalOrigin,
            self.BackupOrigin,
            self.FuseMag,
            self.PosHoldBounceBack,
        ])

        return ret
