'''
Fly Copter in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import annotations

import copy
import math
import os
import shutil
import tempfile
import time
import numpy
import pathlib
import re

from pymavlink import quaternion
from pymavlink import mavutil
from pymavlink import mavextra
from pymavlink import rotmat

from pysim import util
from pysim import vehicleinfo

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException, AutoTestTimeoutException, PreconditionFailedException
from vehicle_test_suite import Test
from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import WaitAndMaintainArmed
from vehicle_test_suite import WaitModeTimeout

from pymavlink.rotmat import Vector3, Matrix3

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(
    -35.362938,
    149.165085,
    584.0805053710938,
    270
)

# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize


class AutoTestCopter(vehicle_test_suite.TestSuite):
    @staticmethod
    def get_not_armable_mode_list():
        return ["AUTO", "AUTOTUNE", "BRAKE", "CIRCLE", "FLIP", "LAND", "RTL", "SMART_RTL", "AVOID_ADSB", "FOLLOW"]

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return ["FLIP", "AUTOTUNE"]

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return ["DRIFT", "GUIDED", "LOITER", "POSHOLD", "THROW"]

    @staticmethod
    def get_normal_armable_modes_list():
        return ["ACRO", "ALT_HOLD", "STABILIZE", "GUIDED_NOGPS"]

    def log_name(self):
        return "ArduCopter"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def default_speedup(self):
        return 100

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduCopter_Tests/" + name + "/"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def mavproxy_options(self):
        ret = super(AutoTestCopter, self).mavproxy_options()
        if self.frame != 'heli':
            ret.append('--quadcopter')
        return ret

    def sitl_streamrate(self):
        return 5

    def vehicleinfo_key(self):
        return 'ArduCopter'

    def default_frame(self):
        return "+"

    def apply_defaultfile_parameters(self):
        # Copter passes in a defaults_filepath in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        return self.model_defaults_filepath(self.frame)

    def wait_disarmed_default_wait_time(self):
        return 120

    def close(self):
        super(AutoTestCopter, self).close()

        # [2014/05/07] FC Because I'm doing a cross machine build
        # (source is on host, build is on guest VM) I cannot hard link
        # This flag tells me that I need to copy the data out
        if self.copy_tlog:
            shutil.copy(self.logfile, self.buildlog)

    def is_copter(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("DISARM_DELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("DISARM_DELAY", delay)

    def takeoff(self,
                alt_min=30,
                takeoff_throttle=1700,
                require_absolute=True,
                mode="STABILIZE",
                timeout=120,
                max_err=5,
                alt_minimum_duration=0,
                ):
        """Takeoff get to 30m altitude."""
        self.progress("TAKEOFF")
        self.change_mode(mode)
        if not self.armed():
            self.wait_ready_to_arm(require_absolute=require_absolute, timeout=timeout)
            self.zero_throttle()
            self.arm_vehicle()
        if mode == 'GUIDED':
            self.user_takeoff(alt_min=alt_min, timeout=timeout, max_err=max_err)
        else:
            self.set_rc(3, takeoff_throttle)
        self.wait_altitude(alt_min-1, alt_min+max_err, relative=True, timeout=timeout, minimum_duration=alt_minimum_duration)
        self.hover()
        self.progress("TAKEOFF COMPLETE")

    def land_and_disarm(self, timeout=60):
        """Land the quad."""
        self.progress("STARTING LANDING")
        self.change_mode("LAND")
        self.wait_landed_and_disarmed(timeout=timeout)

    def wait_landed_and_disarmed(self, min_alt=6, timeout=60):
        """Wait to be landed and disarmed"""
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        alt = m.relative_alt / 1000.0 # mm -> m
        if alt > min_alt:
            self.wait_altitude(min_alt-1, min_alt+5, relative=True, timeout=timeout)
#        self.wait_statustext("SIM Hit ground", timeout=timeout)
        self.wait_disarmed()

    def hover(self, hover_throttle=1500):
        self.set_rc(3, hover_throttle)

    # Climb/descend to a given altitude
    def setAlt(self, desiredAlt=50):
        pos = self.mav.location(relative_alt=True)
        if pos.alt > desiredAlt:
            self.set_rc(3, 1300)
            self.wait_altitude((desiredAlt-5), desiredAlt, relative=True)
        if pos.alt < (desiredAlt-5):
            self.set_rc(3, 1800)
            self.wait_altitude((desiredAlt-5), desiredAlt, relative=True)
        self.hover()

    # Takeoff, climb to given altitude, and fly east for 10 seconds
    def takeoffAndMoveAway(self, dAlt=50, dDist=50):
        self.progress("Centering sticks")
        self.set_rc_from_map({
            1: 1500,
            2: 1500,
            3: 1000,
            4: 1500,
        })
        self.takeoff(alt_min=dAlt, mode='GUIDED')
        self.change_mode("ALT_HOLD")

        self.progress("Yaw to east")
        self.set_rc(4, 1580)
        self.wait_heading(90)
        self.set_rc(4, 1500)

        self.progress("Fly eastbound away from home")
        self.set_rc(2, 1800)
        self.delay_sim_time(10)
        self.set_rc(2, 1500)
        self.hover()
        self.progress("Copter staging 50 meters east of home at 50 meters altitude In mode Alt Hold")

    # loiter - fly south west, then loiter within 5m position and altitude
    def ModeLoiter(self, holdtime=10, maxaltchange=5, maxdistchange=5):
        """Hold loiter position."""
        self.takeoff(10, mode="LOITER")

        # first aim south east
        self.progress("turn south east")
        self.set_rc(4, 1580)
        self.wait_heading(170)
        self.set_rc(4, 1500)

        # fly south east 50m
        self.set_rc(2, 1100)
        self.wait_distance(50)
        self.set_rc(2, 1500)

        # wait for copter to slow moving
        self.wait_groundspeed(0, 2)

        m = self.assert_receive_message('VFR_HUD')
        start_altitude = m.alt
        start = self.mav.location()
        tstart = self.get_sim_time()
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))
        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.assert_receive_message('VFR_HUD')
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            self.progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                raise NotAchievedException(
                    "Loiter alt shifted %u meters (> limit %u)" %
                    (alt_delta, maxaltchange))
            if delta > maxdistchange:
                raise NotAchievedException(
                    "Loiter shifted %u meters (> limit of %u)" %
                    (delta, maxdistchange))
        self.progress("Loiter OK for %u seconds" % holdtime)

        self.progress("Climb to 30m")
        self.change_alt(30)

        self.progress("Descend to 20m")
        self.change_alt(20)

        self.do_RTL()

    def ModeAltHold(self):
        '''Test AltHold Mode'''
        self.takeoff(10, mode="ALT_HOLD")
        self.watch_altitude_maintained(altitude_min=9, altitude_max=11)
        # feed in full elevator and aileron input and make sure we
        # retain altitude:
        self.set_rc_from_map({
            1: 1000,
            2: 1000,
        })
        self.watch_altitude_maintained(altitude_min=9, altitude_max=11)
        self.set_rc_from_map({
            1: 1500,
            2: 1500,
        })
        self.do_RTL()

    def fly_to_origin(self, final_alt=10):
        origin = self.poll_message("GPS_GLOBAL_ORIGIN")
        self.change_mode("GUIDED")
        self.guided_move_global_relative_alt(origin.latitude,
                                             origin.longitude,
                                             final_alt)

    def change_alt(self, alt_min, climb_throttle=1920, descend_throttle=1080):
        """Change altitude."""
        def adjust_altitude(current_alt, target_alt, accuracy):
            if math.fabs(current_alt - target_alt) <= accuracy:
                self.hover()
            elif current_alt < target_alt:
                self.set_rc(3, climb_throttle)
            else:
                self.set_rc(3, descend_throttle)
        self.wait_altitude(
            (alt_min - 5),
            alt_min,
            relative=True,
            called_function=lambda current_alt, target_alt: adjust_altitude(current_alt, target_alt, 1)
        )
        self.hover()

    def RecordThenPlayMission(self, side=50, timeout=300):
        '''Use switches to toggle in mission, then fly it'''
        self.takeoff(20, mode="ALT_HOLD")

        """Fly a square, flying N then E ."""
        tstart = self.get_sim_time()

        # ensure all sticks in the middle
        self.set_rc_from_map({
            1: 1500,
            2: 1500,
            3: 1500,
            4: 1500,
        })

        # switch to loiter mode temporarily to stop us from rising
        self.change_mode('LOITER')

        # first aim north
        self.progress("turn right towards north")
        self.set_rc(4, 1580)
        self.wait_heading(10)
        self.set_rc(4, 1500)

        # save bottom left corner of box as waypoint
        self.progress("Save WP 1 & 2")
        self.save_wp()

        # switch back to ALT_HOLD mode
        self.change_mode('ALT_HOLD')

        # pitch forward to fly north
        self.progress("Going north %u meters" % side)
        self.set_rc(2, 1300)
        self.wait_distance(side)
        self.set_rc(2, 1500)

        # save top left corner of square as waypoint
        self.progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        self.progress("Going east %u meters" % side)
        self.set_rc(1, 1700)
        self.wait_distance(side)
        self.set_rc(1, 1500)

        # save top right corner of square as waypoint
        self.progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        self.progress("Going south %u meters" % side)
        self.set_rc(2, 1700)
        self.wait_distance(side)
        self.set_rc(2, 1500)

        # save bottom right corner of square as waypoint
        self.progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        self.progress("Going west %u meters" % side)
        self.set_rc(1, 1300)
        self.wait_distance(side)
        self.set_rc(1, 1500)

        # save bottom left corner of square (should be near home) as waypoint
        self.progress("Save WP 6")
        self.save_wp()

        # reduce throttle again
        self.set_rc(3, 1500)

        # descend to 10m
        self.progress("Descend to 10m in Loiter")
        self.change_mode('LOITER')
        self.set_rc(3, 1200)
        time_left = timeout - (self.get_sim_time() - tstart)
        self.progress("timeleft = %u" % time_left)
        if time_left < 20:
            time_left = 20
        self.wait_altitude(-10, 10, timeout=time_left, relative=True)
        self.set_rc(3, 1500)
        self.save_wp()

        # Disarm and reset before testing mission
        self.land_and_disarm()
        self.set_rc(3, 1000)
        self.change_mode('LOITER')

        # save the stored mission to file
        mavproxy = self.start_mavproxy()
        num_wp = self.save_mission_to_file_using_mavproxy(
            mavproxy,
            os.path.join(testdir, "ch7_mission.txt"))
        self.stop_mavproxy(mavproxy)
        if not num_wp:
            raise NotAchievedException("save_mission_to_file failed")

        self.arm_vehicle()
        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.change_mode('AUTO')
        # Raise throttle in auto to trigger takeoff
        self.set_rc(3, 1500)
        self.wait_waypoint(0, num_wp-1, timeout=500)
        self.progress("test: MISSION COMPLETE: passed!")
        self.land_and_disarm()

    def WPArcs(self):
        '''Test WP Arc functionality'''
        _ = self.load_and_start_mission("ArcWPMissionTest.txt")

        self.wait_waypoint(0, 15, max_dist=10, timeout=400)
        self.progress("Check that vehicle flys an arc between WP 15 and 16")
        # A crude check, if the arc waypoint doesn't work then the copter will fly directly south between wp 15 and 16
        # If the arc waypoint works then we expect the copter to arc 180 deg in a CW direction meaning it will travel east
        # of WP 15 before attaining WP 16
        tstart = self.get_sim_time()
        timeout = 200
        last_print_time = 0
        while True:
            now = self.get_sim_time_cached()
            delta = now - tstart
            pos = self.assert_receive_message('GLOBAL_POSITION_INT')
            current_long = pos.lon * 1e-7
            desired_long = 149.16522
            current_wp = self.mav.waypoint_current()
            if delta > timeout:
                raise AutoTestTimeoutException(
                    "Failed to achieve position within %fs" % (timeout,)
                )
            if now - last_print_time > 1:
                self.progress(
                    "Waiting for current long (%f) > desired long (%f) "
                    "(%.2fs remaining before timeout)"
                    % (current_long, desired_long, timeout - delta)
                )
                last_print_time = now

            if current_wp > 19:
                raise NotAchievedException(
                    "mission progressed beyond WP 19 before current longitude > "
                    "desired long (%f)" % desired_long
                )

            if (current_long > desired_long):
                self.progress(
                    "Success: Copter traveled far enough east whilst traveling to "
                    "WP 19 that we are likely flying a WP Arc"
                )
                break

        self.wait_disarmed()

    def circle_from_arc(self, p1, p2, theta_deg):
        """
        Compute a single circle center and radius given two points on a circle
        and the signed central angle they subtend.

        p1, p2: (x, y)
        theta: signed central angle in radians
               - the absolute value is used for geometry
               - the sign determines which side of the chord the center is on

        ... with thanks to ChatGPT

        """

        theta = math.radians(theta_deg)

        (x1, y1), (x2, y2) = p1, p2

        # chord vector and length
        dx = x2 - x1
        dy = y2 - y1
        d = math.hypot(dx, dy)

        # radius from absolute angle
        a = abs(theta)
        R = d / (2 * math.sin(a / 2))

        # midpoint of chord
        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2

        # distance from midpoint to center
        h = math.sqrt(R**2 - (d/2)**2)

        # unit perpendicular
        ux = -dy / d
        uy = dx / d

        # choose side based on sign of theta
        sign = 1 if theta >= 0 else -1

        cx = mx + sign * h * ux
        cy = my + sign * h * uy

        return (cx, cy), R

    def WPArcs2(self):
        '''more tests for waypoint arcs'''
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })

        def assert_no_movement(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            if abs(m.climb) > 0.2 or abs(m.groundspeed) > 0.2:
                raise NotAchievedException(f"Moved when I shouldn't {m=}")

        def assert_no_lateral_movement(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            print(f"{m.groundspeed=}")
            if abs(m.groundspeed) > 0.2:
                raise NotAchievedException(f"Moved when I shouldn't {m=}")

        self.start_subtest("Shouldn't move if no takeoff")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
        ])

        self.context_pop()

        self.start_subtest("Shouldn't move if no takeoff")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 20, 20, 20, {"p1": 90}),
        ])
        self.context_pop()

        self.context_push()
        self.start_subtest("Shouldn't move if no takeoff, even if we repeat ourselves")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.install_message_hook_context(assert_no_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
        ])
        self.context_pop()

        self.start_subtest("Shouldn't move if no start waypoint")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_lateral_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.context_pop()

        self.start_subtest("Shouldn't move if endpoint is startpoint")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_lateral_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.context_pop()

        self.progress("Creating a log per-flight")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.set_parameter("LOG_FILE_DSRMROT", 1)
        self.arm_vehicle()
        self.disarm_vehicle()

        # this will move but won't arc (it should...):
        self.start_subtest("Should move if no start waypoint")
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed()
        self.context_pop()

        self.start_subtest("Degenerate arc - should be a straight-line segment")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 0}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed()

        self.start_subtest("Degenerate arc - no length")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 10, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 10, 0, 20, {"p1": 180}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(5, 20)
        self.wait_disarmed()

        self.start_subtest("Should arc if we start with a waypoint")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        pos, radius = self.circle_from_arc((0, 0), (90, 0), 90)
        loc = self.offset_location_ne(self.home_position_as_mav_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=10,
            timeout=30,
            track_angle=False,
        )
        self.wait_distance_to_home(80, 100)
        self.wait_disarmed()

        self.start_subtest("arc - should end up with a mirrored D")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        pos, radius = self.circle_from_arc((0, 0), (90, 0), -90)
        loc = self.offset_location_ne(self.home_position_as_mav_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 270}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=10,
            timeout=30,
            track_angle=False,
        )
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - should end up with a D")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        pos, radius = self.circle_from_arc((0, 0), (90, 0), 90)
        loc = self.offset_location_ne(self.home_position_as_mav_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": -270}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=10,
            timeout=30,
            track_angle=False,
        )
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - should end up with helix up")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        pos = (45, 0)
        radius = 45
        loc = self.offset_location_ne(self.home_position_as_mav_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 100, {"p1": -2700}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=360,
            timeout=400,
            track_angle=False,
        )
        self.wait_altitude(99, 101, relative=True, timeout=240)
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - should end up with helix down")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 2700}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(90, 100, relative=True, timeout=120)
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - chained arcs")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 20, {"p1": -90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 20, {"p1": -90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(15, 25, relative=True, timeout=120)
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - chained arcs - up/down")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 180}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 40, {"p1": -180}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 40, {"p1": 180}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 120, 0, 20, {"p1": -180}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(15, 25, relative=True, timeout=120)
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed(timeout=600)
        self.context_pop()

    # enter RTL mode and wait for the vehicle to disarm
    def do_RTL(self, distance_min=None, check_alt=True, distance_max=10, timeout=250, quiet=False):
        """Enter RTL mode and wait for the vehicle to disarm at Home."""
        self.change_mode("RTL")
        self.hover()
        self.wait_rtl_complete(check_alt=check_alt, distance_max=distance_max, timeout=timeout, quiet=True)

    def wait_rtl_complete(self, check_alt=True, distance_max=10, timeout=250, quiet=False):
        """Wait for RTL to reach home and disarm"""
        self.progress("Waiting RTL to reach Home and disarm")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            home = ""
            alt_valid = alt <= 1
            distance_valid = home_distance < distance_max
            if check_alt:
                if alt_valid and distance_valid:
                    home = "HOME"
            else:
                if distance_valid:
                    home = "HOME"
            if not quiet:
                self.progress("Alt: %.02f  HomeDist: %.02f %s" %
                              (alt, home_distance, home))

            # our post-condition is that we are disarmed:
            if not self.armed():
                if home == "":
                    raise NotAchievedException("Did not get home")
                # success!
                return

        raise AutoTestTimeoutException("Did not get home and disarm")

    def LoiterToAlt(self):
        """Loiter-To-Alt"""

        self.context_push()

        self.set_parameters({
            "PLND_ENABLED": 1,
            "PLND_TYPE": 4,
        })

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        num_wp = self.load_mission("copter_loiter_to_alt.txt")

        self.change_mode('LOITER')

        self.install_terrain_handlers_context()
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.change_mode('AUTO')

        self.set_rc(3, 1550)

        self.wait_current_waypoint(2)

        self.set_rc(3, 1500)

        self.wait_waypoint(0, num_wp-1, timeout=500)

        self.wait_disarmed()

        self.context_pop()
        self.reboot_sitl()

    # Tests all actions and logic behind the radio failsafe
    def ThrottleFailsafe(self, side=60, timeout=360):
        '''Test Throttle Failsafe'''
        self.start_subtest("If you haven't taken off yet RC failure should be instant disarm")
        self.change_mode("STABILIZE")
        self.set_parameter("DISARM_DELAY", 0)
        self.arm_vehicle()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.disarm_wait(timeout=1)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter("DISARM_DELAY", 10)

        # Trigger an RC failure with the failsafe disabled. Verify no action taken.
        self.start_subtest("Radio failsafe disabled test: FS_THR_ENABLE=0 should take no failsafe action")
        self.set_parameter('FS_THR_ENABLE', 0)
        self.set_parameter('FS_OPTIONS', 0)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.end_subtest("Completed Radio failsafe disabled test")

        # Trigger an RC failure, verify radio failsafe triggers,
        # restore radio, verify RC function by changing modes to circle
        # and stabilize.
        self.start_subtest("Radio failsafe recovery test")
        self.set_parameter('FS_THR_ENABLE', 1)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5)
        self.set_rc(5, 1050)
        self.wait_mode("CIRCLE")
        self.set_rc(5, 1950)
        self.wait_mode("STABILIZE")
        self.end_subtest("Completed Radio failsafe recovery test")

        # Trigger and RC failure, verify failsafe triggers and RTL completes
        self.start_subtest("Radio failsafe RTL with no options test: FS_THR_ENABLE=1 & FS_OPTIONS=0")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe RTL with no options test")

        # Trigger and RC failure, verify failsafe triggers and land completes
        self.start_subtest("Radio failsafe LAND with no options test: FS_THR_ENABLE=3 & FS_OPTIONS=0")
        self.set_parameter('FS_THR_ENABLE', 3)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe LAND with no options test")

        # Trigger and RC failure, verify failsafe triggers and SmartRTL completes
        self.start_subtest("Radio failsafe SmartRTL->RTL with no options test: FS_THR_ENABLE=4 & FS_OPTIONS=0")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL->RTL with no options test")

        # Trigger and RC failure, verify failsafe triggers and SmartRTL completes
        self.start_subtest("Radio failsafe SmartRTL->Land with no options test: FS_THR_ENABLE=5 & FS_OPTIONS=0")
        self.set_parameter('FS_THR_ENABLE', 5)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL_Land with no options test")

        # Trigger a GPS failure and RC failure, verify RTL fails into
        # land mode and completes
        self.start_subtest("Radio failsafe RTL fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 1)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe RTL fails into land mode due to bad position.")

        # Trigger a GPS failure and RC failure, verify SmartRTL fails
        # into land mode and completes
        self.start_subtest("Radio failsafe SmartRTL->RTL fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe SmartRTL->RTL fails into land mode due to bad position.")

        # Trigger a GPS failure and RC failure, verify SmartRTL fails
        # into land mode and completes
        self.start_subtest("Radio failsafe SmartRTL->LAND fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 5)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe SmartRTL->LAND fails into land mode due to bad position.")

        # Trigger a GPS failure, then restore the GPS. Trigger an RC
        # failure, verify SmartRTL fails into RTL and completes
        self.start_subtest("Radio failsafe SmartRTL->RTL fails into RTL mode due to no path.")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.wait_statustext("SmartRTL deactivated: bad position", timeout=60)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL->RTL fails into RTL mode due to no path.")

        # Trigger a GPS failure, then restore the GPS. Trigger an RC
        # failure, verify SmartRTL fails into Land and completes
        self.start_subtest("Radio failsafe SmartRTL->LAND fails into land mode due to no path.")
        self.set_parameter('FS_THR_ENABLE', 5)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.wait_statustext("SmartRTL deactivated: bad position", timeout=60)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL->LAND fails into land mode due to no path.")

        # Trigger an RC failure in guided mode with the option enabled
        # to continue in guided. Verify no failsafe action takes place
        self.start_subtest("Radio failsafe with option to continue in guided mode: FS_THR_ENABLE=1 & FS_OPTIONS=4")
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.setGCSfailsafe(1)
        self.set_parameter('FS_THR_ENABLE', 1)
        self.set_parameter('FS_OPTIONS', 4)
        self.takeoffAndMoveAway()
        self.change_mode("GUIDED")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(5)
        self.wait_mode("GUIDED")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5)
        self.change_mode("ALT_HOLD")
        self.setGCSfailsafe(0)
        # self.change_mode("RTL")
        # self.wait_disarmed()
        self.end_subtest("Completed Radio failsafe with option to continue in guided mode")

        # Trigger an RC failure in AUTO mode with the option enabled
        # to continue the mission. Verify no failsafe action takes
        # place
        self.start_subtest("Radio failsafe RTL with option to continue mission: FS_THR_ENABLE=1 & FS_OPTIONS=1")
        self.set_parameter('FS_OPTIONS', 1)
        self.progress("# Load copter_mission")
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")
#        self.takeoffAndMoveAway()
        self.change_mode("AUTO")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(5)
        self.wait_mode("AUTO")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5)
        self.wait_mode("AUTO")
        # self.change_mode("RTL")
        # self.wait_disarmed()
        self.end_subtest("Completed Radio failsafe RTL with option to continue mission")

        # Trigger an RC failure in AUTO mode without the option
        # enabled to continue. Verify failsafe triggers and RTL
        # completes
        self.start_subtest("Radio failsafe RTL in mission without "
                           "option to continue should RTL: FS_THR_ENABLE=1 & FS_OPTIONS=0")
        self.set_parameter('FS_OPTIONS', 0)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe RTL in mission without option to continue")

        self.progress("All radio failsafe tests complete")
        self.set_parameter('FS_THR_ENABLE', 0)
        self.reboot_sitl()

    def ThrottleFailsafePassthrough(self):
        '''check servo passthrough on RC failsafe.  Make sure it doesn't glitch to the bad RC input value'''
        channel = 7
        trim_value = 1450
        self.set_parameters({
            'RC%u_MIN' % channel: 1000,
            'RC%u_MAX' % channel: 2000,
            'SERVO%u_MIN' % channel: 1000,
            'SERVO%u_MAX' % channel: 2000,
            'SERVO%u_TRIM' % channel: trim_value,
            'SERVO%u_FUNCTION' % channel: 146,  # scaled passthrough for channel 7
            'FS_THR_ENABLE': 1,
            'RC_FS_TIMEOUT': 10,
            'SERVO_RC_FS_MSK': 1 << (channel-1),
        })

        self.reboot_sitl()

        self.context_set_message_rate_hz('SERVO_OUTPUT_RAW', 200)

        self.set_rc(channel, 1799)
        expected_servo_output_value = 1778  # 1778 because of weird trim
        self.wait_servo_channel_value(channel, expected_servo_output_value)
        # receiver goes into failsafe with wild override values:

        def ensure_SERVO_values_never_input(mav, m):
            if m.get_type() != "SERVO_OUTPUT_RAW":
                return
            value = getattr(m, "servo%u_raw" % channel)
            if value != expected_servo_output_value and value != trim_value:
                raise NotAchievedException("Bad servo value %u received" % value)

        self.install_message_hook_context(ensure_SERVO_values_never_input)
        self.progress("Forcing receiver into failsafe")
        self.set_rc_from_map({
            3: 800,
            channel: 1300,
        })
        self.wait_servo_channel_value(channel, trim_value)
        self.delay_sim_time(10)

    # Tests all actions and logic behind the GCS failsafe
    def GCSFailsafe(self, side=60, timeout=360):
        '''Test GCS Failsafe'''
        # Test double-SmartRTL; ensure we do SmarRTL twice rather than
        # landing (tests fix for actual bug)
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.context_push()
        self.start_subtest("GCS failsafe SmartRTL twice")
        self.setGCSfailsafe(3)
        self.set_parameter('FS_OPTIONS', 8)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe")

        def ensure_smartrtl(mav, m):
            if m.get_type() != "HEARTBEAT":
                return
            # can't use mode_is here because we're in the message hook
            print("Mode: %s" % self.mav.flightmode)
            if self.mav.flightmode != "SMART_RTL":
                raise NotAchievedException("Not in SMART_RTL")
        self.install_message_hook_context(ensure_smartrtl)

        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe")

        self.wait_disarmed()

        self.end_subtest("GCS failsafe SmartRTL twice")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.context_pop()

        # Trigger telemetry loss with failsafe disabled. Verify no action taken.
        self.start_subtest("GCS failsafe disabled test: FS_GCS_ENABLE=0 should take no failsafe action")
        self.setGCSfailsafe(0)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.set_heartbeat_rate(self.speedup)
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.end_subtest("Completed GCS failsafe disabled test")

        # Trigger telemetry loss with failsafe enabled. Verify
        # failsafe triggers to RTL. Restore telemetry, verify failsafe
        # clears, and change modes.
        self.start_subtest("GCS failsafe recovery test: FS_GCS_ENABLE=1 & FS_OPTIONS=0")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 0)
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.change_mode("LOITER")
        self.end_subtest("Completed GCS failsafe recovery test")

        # Trigger telemetry loss with failsafe enabled. Verify
        # failsafe triggers to RTL. Restore telemetry, verify failsafe
        # clears, and change modes.
        self.start_subtest("GCS failsafe recovery test: FS_GCS_ENABLE=1 & FS_OPTIONS=0 & FS_GCS_TIMEOUT=10")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 0)
        old_gcs_timeout = self.get_parameter("FS_GCS_TIMEOUT")
        new_gcs_timeout = old_gcs_timeout * 2
        self.set_parameter("FS_GCS_TIMEOUT", new_gcs_timeout)
        self.set_heartbeat_rate(0)
        self.delay_sim_time(old_gcs_timeout + (new_gcs_timeout - old_gcs_timeout) / 2)
        self.assert_mode("LOITER")
        self.wait_mode("RTL")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.change_mode("LOITER")
        self.set_parameter('FS_GCS_TIMEOUT', old_gcs_timeout)
        self.end_subtest("Completed GCS failsafe recovery test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe RTL with no options test: FS_GCS_ENABLE=1 & FS_OPTIONS=0")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 0)
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe RTL with no options test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and land completes
        self.start_subtest("GCS failsafe LAND with no options test: FS_GCS_ENABLE=5 & FS_OPTIONS=0")
        self.setGCSfailsafe(5)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe land with no options test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and SmartRTL completes
        self.start_subtest("GCS failsafe SmartRTL->RTL with no options test: FS_GCS_ENABLE=3 & FS_OPTIONS=0")
        self.setGCSfailsafe(3)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe SmartRTL->RTL with no options test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and SmartRTL completes
        self.start_subtest("GCS failsafe SmartRTL->Land with no options test: FS_GCS_ENABLE=4 & FS_OPTIONS=0")
        self.setGCSfailsafe(4)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe SmartRTL->Land with no options test")

        # Trigger telemetry loss with an invalid failsafe value. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe invalid value with no options test: FS_GCS_ENABLE=99 & FS_OPTIONS=0")
        self.setGCSfailsafe(99)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe invalid value with no options test")

        # Trigger telemetry loss with failsafe enabled to test FS_OPTIONS settings
        self.start_subtest("GCS failsafe with option bit tests: FS_GCS_ENABLE=1 & FS_OPTIONS=64/2/16")
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 16)
        self.takeoffAndMoveAway()
        self.progress("Testing continue in pilot controlled modes")
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe - Continuing Pilot Control", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.progress("Testing continue in auto mission")
        self.set_parameter('FS_OPTIONS', 2)
        self.change_mode("AUTO")
        self.delay_sim_time(5)
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe - Continuing Auto Mode", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("AUTO")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.progress("Testing continue landing in land mode")
        self.set_parameter('FS_OPTIONS', 8)
        self.change_mode("LAND")
        self.delay_sim_time(5)
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe - Continuing Landing", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe with option bits")

        self.setGCSfailsafe(0)
        self.set_parameter('FS_OPTIONS', 0)
        self.progress("All GCS failsafe tests complete")

    def CustomController(self, timeout=300):
        '''Test Custom Controller'''
        self.progress("Configure custom controller parameters")
        self.set_parameters({
            'CC_TYPE': 2,
            'CC_AXIS_MASK': 7,
            'RC6_OPTION': 109,
        })
        self.set_rc(6, 1000)
        self.reboot_sitl()

        if self.get_parameter("CC_TYPE") != 2 :
            raise NotAchievedException("Custom controller is not switched to PID backend.")

        # check if we can retrieve any param inside PID backend
        self.get_parameter("CC2_RAT_YAW_P")

        # takeoff in GPS mode and switch to CIRCLE
        self.takeoff(10, mode="LOITER", takeoff_throttle=2000)
        self.change_mode("CIRCLE")

        self.context_push()
        self.context_collect('STATUSTEXT')

        # switch custom controller on
        self.set_rc(6, 2000)
        self.wait_statustext("Custom controller is ON", check_context=True)

        # wait 20 second to see if the custom controller destabilize the aircraft
        if self.wait_altitude(7, 13, relative=True, minimum_duration=20) :
            raise NotAchievedException("Custom controller is not stable.")

        # switch custom controller off
        self.set_rc(6, 1000)
        self.wait_statustext("Custom controller is OFF", check_context=True)

        self.context_pop()
        self.do_RTL()
        self.progress("Custom controller test complete")

    # Tests all actions and logic behind the battery failsafe
    def BatteryFailsafe(self, timeout=300):
        '''Fly Battery Failsafe'''
        self.progress("Configure battery failsafe parameters")
        self.set_parameters({
            'SIM_SPEEDUP': 4,
            'BATT_LOW_VOLT': 11.5,
            'BATT_CRT_VOLT': 10.1,
            'BATT_FS_LOW_ACT': 0,
            'BATT_FS_CRT_ACT': 0,
            'FS_OPTIONS': 0,
            'SIM_BATT_VOLTAGE': 12.5,
        })

        # Trigger low battery condition with failsafe disabled. Verify
        # no action taken.
        self.start_subtest("Batt failsafe disabled test")
        self.takeoffAndMoveAway()
        m = self.assert_receive_message('BATTERY_STATUS')
        if m.charge_state != mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK:
            raise NotAchievedException("Expected state ok")
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        m = self.assert_receive_message('BATTERY_STATUS')
        if m.charge_state != mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_LOW:
            raise NotAchievedException("Expected state low")
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        m = self.assert_receive_message('BATTERY_STATUS')
        if m.charge_state != mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_CRITICAL:
            raise NotAchievedException("Expected state critical")
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.change_mode("RTL")
        self.wait_rtl_complete()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.reboot_sitl()
        self.end_subtest("Completed Batt failsafe disabled test")

        # TWO STAGE BATTERY FAILSAFE: Trigger low battery condition,
        # then critical battery condition. Verify RTL and Land actions
        # complete.
        self.start_subtest("Two stage battery failsafe test with RTL and Land")
        self.takeoffAndMoveAway()
        self.delay_sim_time(3)
        self.set_parameters({
            'BATT_FS_LOW_ACT': 2,
            'BATT_FS_CRT_ACT': 1,
            'SIM_BATT_VOLTAGE': 11.4,
        })
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("RTL")
        self.delay_sim_time(10)
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.reboot_sitl()
        self.end_subtest("Completed two stage battery failsafe test with RTL and Land")

        # TWO STAGE BATTERY FAILSAFE: Trigger low battery condition,
        # then critical battery condition. Verify both SmartRTL
        # actions complete
        self.start_subtest("Two stage battery failsafe test with SmartRTL")
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 3)
        self.set_parameter('BATT_FS_CRT_ACT', 4)
        self.delay_sim_time(10)
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("SMART_RTL")
        self.change_mode("LOITER")
        self.delay_sim_time(10)
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.reboot_sitl()
        self.end_subtest("Completed two stage battery failsafe test with SmartRTL")

        # Trigger low battery condition in land mode with FS_OPTIONS
        # set to allow land mode to continue. Verify landing completes
        # uninterrupted.
        self.start_subtest("Battery failsafe with FS_OPTIONS set to continue landing")
        self.takeoffAndMoveAway()
        self.set_parameter('FS_OPTIONS', 8)
        self.change_mode("LAND")
        self.delay_sim_time(5)
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.reboot_sitl()
        self.end_subtest("Completed battery failsafe with FS_OPTIONS set to continue landing")

        # Trigger a critical battery condition, which triggers a land
        # mode failsafe. Trigger an RC failure. Verify the RC failsafe
        # is prevented from stopping the low battery landing.
        self.start_subtest("Battery failsafe critical landing")
        self.takeoffAndMoveAway(100, 50)
        self.set_parameters({
            'FS_OPTIONS': 0,
            'BATT_FS_LOW_ACT': 1,
            'BATT_FS_CRT_ACT': 1,
            'FS_THR_ENABLE': 1,
        })
        self.delay_sim_time(5)
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.wait_mode("LAND")
        self.delay_sim_time(10)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(10)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter('SIM_BATT_VOLTAGE', 12.5)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.reboot_sitl()
        self.end_subtest("Completed battery failsafe critical landing")

        # Trigger low battery condition with failsafe set to brake/land
        self.start_subtest("Battery failsafe brake/land")
        self.context_push()
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 7)
        self.delay_sim_time(10)
        self.change_mode('LOITER')
        self.set_rc(1, 2000)
        self.wait_groundspeed(8, 10)
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode('BRAKE')
        self.set_rc(1, 1500)
        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()
        self.end_subtest("Completed brake/land failsafe test")

        self.start_subtest("Battery failsafe brake/land - land")
        self.context_push()
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 7)
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(10)
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode('LAND')
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.context_pop()
        self.end_subtest("Completed brake/land failsafe test")

        # Trigger low battery condition with failsafe set to terminate. Copter will disarm and crash.
        self.start_subtest("Battery failsafe terminate")
        self.takeoffAndMoveAway()
        self.set_parameter('BATT_FS_LOW_ACT', 5)
        self.delay_sim_time(10)
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_disarmed()
        self.end_subtest("Completed terminate failsafe test")

        self.progress("All Battery failsafe tests complete")

    def BatteryMissing(self):
        ''' Test battery health pre-arm and missing failsafe'''
        self.context_push()

        # Should be good to arm with no changes
        self.wait_ready_to_arm()

        # Make monitor unhealthy, this should result in unhealthy prearm
        self.set_parameters({
            'BATT_VOLT_PIN': -1,
        })

        self.drain_mav()

        # Battery should go unhealthy immediately
        self.assert_prearm_failure("Battery 1 unhealthy", other_prearm_failures_fatal=False)

        # Return monitor to health
        self.context_pop()
        self.context_push()

        self.wait_ready_to_arm()

        # take off and then trigger in flight
        self.takeoff(10, mode="LOITER")
        self.set_parameters({
            'BATT_VOLT_PIN': -1,
        })

        # Should trigger missing failsafe
        self.wait_statustext("Battery 1 is missing")

        # Done, reset params and reboot to clear failsafe
        self.land_and_disarm()
        self.context_pop()
        self.reboot_sitl()

    def VibrationFailsafe(self):
        '''Test Vibration Failsafe'''
        self.context_push()

        # takeoff in Loiter to 20m
        self.takeoff(20, mode="LOITER")

        # simulate accel bias caused by high vibration
        self.set_parameters({
            'SIM_ACC1_BIAS_Z': 2,
            'SIM_ACC2_BIAS_Z': 2,
            'SIM_ACC3_BIAS_Z': 2,
        })

        # wait for Vibration compensation warning and change to LAND mode
        self.wait_statustext("Vibration compensation ON", timeout=30)
        self.change_mode("LAND")

        # check vehicle descends to 2m or less within 40 seconds
        self.wait_altitude(-5, 2, timeout=50, relative=True)

        # force disarm of vehicle (it will likely not automatically disarm)
        self.disarm_vehicle(force=True)

        # revert simulated accel bias and reboot to restore EKF health
        self.context_pop()
        self.reboot_sitl()

    # Tests the motor failsafe
    def TakeoffCheck(self):
        '''Test takeoff check'''
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            'SIM_ESC_TELEM': 1,
            'SIM_ESC_ARM_RPM': 500,
            'TKOFF_RPM_MIN': 1000,
        })

        self.test_takeoff_check_mode("STABILIZE")
        self.test_takeoff_check_mode("ACRO")
        self.test_takeoff_check_mode("LOITER")
        self.test_takeoff_check_mode("ALT_HOLD")
        # self.test_takeoff_check_mode("FLOWHOLD")
        self.test_takeoff_check_mode("GUIDED", True)
        self.test_takeoff_check_mode("POSHOLD")
        # self.test_takeoff_check_mode("SPORT")

        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            'SIM_ESC_TELEM': 1,
            'TKOFF_RPM_MIN': 1,
            'TKOFF_RPM_MAX': 3,
        })
        self.test_takeoff_check_mode("STABILIZE")
        self.test_takeoff_check_mode("ACRO")
        self.test_takeoff_check_mode("LOITER")
        self.test_takeoff_check_mode("ALT_HOLD")
        # self.test_takeoff_check_mode("FLOWHOLD")
        self.test_takeoff_check_mode("GUIDED", True)
        self.test_takeoff_check_mode("POSHOLD")
        # self.test_takeoff_check_mode("SPORT")

    def assert_dataflash_message_field_level_at(self,
                                                mtype,
                                                field,
                                                level,
                                                maintain=1,
                                                tolerance=0.05,
                                                timeout=30,
                                                condition=None,
                                                dfreader_start_timestamp=None,
                                                verbose=False):
        '''wait for EKF's accel bias to reach a level and maintain it'''

        if verbose:
            self.progress("current onboard log filepath: %s" % self.current_onboard_log_filepath())
        dfreader = self.dfreader_for_current_onboard_log()

        achieve_start = None
        current_value = None
        while True:
            m = dfreader.recv_match(type=mtype, condition=condition)
            if m is None:
                raise NotAchievedException("%s.%s did not maintain %f" %
                                           (mtype, field, level))
            if dfreader_start_timestamp is not None:
                if m.TimeUS < dfreader_start_timestamp:
                    continue
            if verbose:
                print("m=%s" % str(m))
            current_value = getattr(m, field)

            if abs(current_value - level) > tolerance:
                if achieve_start is not None:
                    self.progress("Achieve stop at %u" % m.TimeUS)
                    achieve_start = None
                continue

            dfreader_now = m.TimeUS
            if achieve_start is None:
                self.progress("Achieve start at %u (got=%f want=%f)" %
                              (dfreader_now, current_value, level))
                if maintain is None:
                    return
                achieve_start = m.TimeUS
                continue

            # we're achieving....
            if dfreader_now - achieve_start > maintain*1e6:
                return dfreader_now

    # Tests any EK3 accel bias is subtracted from the correct IMU data
    def EK3AccelBias(self):
        '''Test EK3 Accel Bias data'''
        self.context_push()

        self.start_test("Test zero bias")
        self.delay_sim_time(2)
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2",
            "AZ",
            0.0,
            condition="XKF2.C==1",
            maintain=1,
        )

        # Add 2m/s/s bias to the second IMU
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })

        self.start_subtest("Ensuring second core has bias")
        self.delay_sim_time(30)
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==1",
        )

        self.start_subtest("Ensuring earth frame is compensated")
        self.assert_dataflash_message_field_level_at(
            "RATE", "A", 0,
            maintain=1,
            tolerance=2,    # RATE.A is in cm/s/s
            dfreader_start_timestamp=dfreader_tstart,
        )

        # now switch the EKF to only using the second core:
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.0,
            "EK3_IMU_MASK": 0b10,
        })
        self.reboot_sitl()

        self.delay_sim_time(30)
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.0,
            condition="XKF2.C==0",
        )

        # Add 2m/s/s bias to the second IMU
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })

        self.start_subtest("Ensuring first core now has bias")
        self.delay_sim_time(30)
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==0",
        )

        self.start_subtest("Ensuring earth frame is compensated")
        self.assert_dataflash_message_field_level_at(
            "RATE", "A", 0,
            maintain=1,
            tolerance=2,  # RATE.A is in cm/s/s
            dfreader_start_timestamp=dfreader_tstart,
            verbose=True,
        )

        # revert simulated accel bias and reboot to restore EKF health
        self.context_pop()
        self.reboot_sitl()

    # StabilityPatch - fly south, then hold loiter within 5m
    # position and altitude and reduce 1 motor to 60% efficiency
    def StabilityPatch(self,
                       holdtime=30,
                       maxaltchange=5,
                       maxdistchange=10):
        '''Fly stability patch'''
        self.takeoff(10, mode="LOITER")

        # first south
        self.progress("turn south")
        self.set_rc(4, 1580)
        self.wait_heading(180)
        self.set_rc(4, 1500)

        # fly west 80m
        self.set_rc(2, 1100)
        self.wait_distance(80)
        self.set_rc(2, 1500)

        # wait for copter to slow moving
        self.wait_groundspeed(0, 2)

        m = self.assert_receive_message('VFR_HUD')
        start_altitude = m.alt
        start = self.mav.location()
        tstart = self.get_sim_time()
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))

        # cut motor 1's to efficiency
        self.progress("Cutting motor 1 to 65% efficiency")
        self.set_parameters({
            "SIM_ENGINE_MUL": 0.65,
            "SIM_ENGINE_FAIL": 1 << 0, # motor 1
        })

        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.assert_receive_message('VFR_HUD')
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            self.progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                raise NotAchievedException(
                    "Loiter alt shifted %u meters (> limit %u)" %
                    (alt_delta, maxaltchange))
            if delta > maxdistchange:
                raise NotAchievedException(
                    ("Loiter shifted %u meters (> limit of %u)" %
                     (delta, maxdistchange)))

        # restore motor 1 to 100% efficiency
        self.set_parameter("SIM_ENGINE_MUL", 1.0)

        self.progress("Stability patch and Loiter OK for %us" % holdtime)

        self.progress("RTL after stab patch")
        self.do_RTL()

    def debug_arming_issue(self):
        while True:
            self.send_mavlink_arm_command()
            m = self.mav.recv_match(blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() in ["STATUSTEXT", "COMMAND_ACK"]:
                print("Got: %s" % str(m))
            if self.mav.motors_armed():
                self.progress("Armed")
                return

    # fly_fence_test - fly east until you hit the horizontal circular fence
    avoid_behave_slide = 0

    def fly_fence_avoid_test_radius_check(self, timeout=180, avoid_behave=avoid_behave_slide):
        using_mode = "LOITER" # must be something which adjusts velocity!
        self.change_mode(using_mode)
        fence_radius = 15
        fence_margin = 3
        self.set_parameters({
            "FENCE_ENABLE": 1, # fence
            "FENCE_TYPE": 2, # circle
            "FENCE_RADIUS": fence_radius,
            "FENCE_MARGIN": fence_margin,
            "AVOID_ENABLE": 1,
            "AVOID_BEHAVE": avoid_behave,
            "RC10_OPTION": 40, # avoid-enable
        })
        self.wait_ready_to_arm()
        self.set_rc(10, 2000)
        home_distance = self.distance_to_home(use_cached_home=True)
        if home_distance > 5:
            raise PreconditionFailedException("Expected to be within 5m of home")
        self.zero_throttle()
        self.arm_vehicle()
        self.set_rc(3, 1700)
        self.wait_altitude(10, 100, relative=True)
        self.set_rc(3, 1500)
        self.set_rc(2, 1400)
        self.wait_distance_to_home(12, 20, timeout=30)
        tstart = self.get_sim_time()
        push_time = 70 # push against barrier for 60 seconds
        failed_max = False
        failed_min = False
        while True:
            if self.get_sim_time() - tstart > push_time:
                self.progress("Push time up")
                break
            # make sure we don't RTL:
            if not self.mode_is(using_mode):
                raise NotAchievedException("Changed mode away from %s" % using_mode)
            distance = self.distance_to_home(use_cached_home=True)
            inner_radius = fence_radius - fence_margin
            want_min = inner_radius - 1 # allow 1m either way
            want_max = inner_radius + 1 # allow 1m either way
            self.progress("Push: distance=%f %f<want<%f" %
                          (distance, want_min, want_max))
            if distance < want_min:
                if failed_min is False:
                    self.progress("Failed min")
                    failed_min = True
            if distance > want_max:
                if failed_max is False:
                    self.progress("Failed max")
                    failed_max = True
        if failed_min and failed_max:
            raise NotAchievedException("Failed both min and max checks.  Clever")
        if failed_min:
            raise NotAchievedException("Failed min")
        if failed_max:
            raise NotAchievedException("Failed max")
        self.set_rc(2, 1500)
        self.do_RTL()

    def HorizontalAvoidFence(self, timeout=180):
        '''Test horizontal Avoidance fence'''
        self.fly_fence_avoid_test_radius_check(avoid_behave=1, timeout=timeout)
        self.fly_fence_avoid_test_radius_check(avoid_behave=0, timeout=timeout)

    # fly_fence_test - fly east until you hit the horizontal circular fence
    def HorizontalFence(self, timeout=180):
        '''Test horizontal fence'''
        # enable fence, disable avoidance
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 0,
        })

        self.change_mode("LOITER")
        self.wait_ready_to_arm()

        # fence requires home to be set:
        m = self.poll_home_position(quiet=False)

        self.start_subtest("ensure we can't arm if outside fence")
        self.load_fence("fence-in-middle-of-nowhere.txt")

        self.delay_sim_time(5) # let fence check run so it loads-from-eeprom
        self.assert_prearm_failure("Vehicle breaching Polygon fence")
        self.progress("Failed to arm outside fence (good!)")
        self.clear_fence()
        self.delay_sim_time(5) # let fence breach clear
        self.drain_mav()
        self.end_subtest("ensure we can't arm if outside fence")

        self.start_subtest("ensure we can't arm with bad radius")
        self.context_push()
        self.set_parameter("FENCE_RADIUS", -1)
        self.assert_prearm_failure("Invalid Circle FENCE_RADIUS value")
        self.context_pop()
        self.progress("Failed to arm with bad radius")
        self.drain_mav()
        self.end_subtest("ensure we can't arm with bad radius")

        self.start_subtest("ensure we can't arm with bad alt")
        self.context_push()
        self.set_parameter("FENCE_ALT_MAX", -1)
        self.assert_prearm_failure("Invalid FENCE_ALT_MAX value")
        self.context_pop()
        self.progress("Failed to arm with bad altitude")
        self.end_subtest("ensure we can't arm with bad radius")

        self.start_subtest("Check breach-fence behaviour")
        self.set_parameter("FENCE_TYPE", 2)
        self.takeoff(10, mode="LOITER")

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160, timeout=60)
        self.set_rc(4, 1500)

        fence_radius = self.get_parameter("FENCE_RADIUS")

        self.progress("flying forward (east) until we hit fence")
        pitching_forward = True
        self.set_rc(2, 1100)

        self.progress("Waiting for fence breach")
        tstart = self.get_sim_time()
        while not self.mode_is("RTL"):
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("Did not breach fence")

            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            self.progress("Alt: %.02f  HomeDistance: %.02f (fence radius=%f)" %
                          (alt, home_distance, fence_radius))

        self.progress("Waiting until we get home and disarm")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            self.progress("Alt: %.02f  HomeDistance: %.02f" %
                          (alt, home_distance))
            # recenter pitch sticks once we're home so we don't fly off again
            if pitching_forward and home_distance < 50:
                pitching_forward = False
                self.set_rc(2, 1475)
                # disable fence
                self.set_parameter("FENCE_ENABLE", 0)
            if (alt <= 1 and home_distance < 10) or (not self.armed() and home_distance < 10):
                # reduce throttle
                self.zero_throttle()
                self.change_mode("LAND")
                self.wait_landed_and_disarmed()
                self.progress("Reached home OK")
                self.zero_throttle()
                return

        # give we're testing RTL, doing one here probably doesn't make sense
        home_distance = self.distance_to_home(use_cached_home=True)
        raise AutoTestTimeoutException(
            "Fence test failed to reach home (%fm distance) - "
            "timed out after %u seconds" % (home_distance, timeout,))

    # MaxAltFence - fly up until you hit the fence ceiling
    def MaxAltFence(self):
        '''Test Max Alt Fence'''
        self.takeoff(10, mode="LOITER")
        """Hold loiter position."""

        # enable fence, disable avoidance
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 0,
            "FENCE_TYPE": 1,
            "FENCE_ENABLE" : 1,
        })

        self.change_alt(10)

        # first east
        self.progress("turning east")
        self.set_rc(4, 1580)
        self.wait_heading(160, timeout=60)
        self.set_rc(4, 1500)

        self.progress("flying east 20m")
        self.set_rc(2, 1100)
        self.wait_distance(20)

        self.progress("flying up")
        self.set_rc_from_map({
            2: 1500,
            3: 1800,
        })

        # wait for fence to trigger
        self.wait_mode('RTL', timeout=120)

        self.wait_rtl_complete()

        self.zero_throttle()

    # MaxAltFence - fly up and make sure fence action does not trigger
    # Also check that the vehicle will not try and descend too fast when trying to backup from a max alt fence due to avoidance
    def MaxAltFenceAvoid(self):
        '''Test Max Alt Fence Avoidance'''
        self.takeoff(10, mode="LOITER")
        """Hold loiter position."""

        # enable fence, only max altitude, default is 100m
        # No action, rely on avoidance to prevent the breach
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 1,
            "FENCE_ACTION": 0,
        })

        # Try and fly past the fence
        self.set_rc(3, 1920)

        # Avoid should prevent the vehicle flying past the fence, so the altitude wait should timeouts
        try:
            self.wait_altitude(140, 150, timeout=90, relative=True)
            raise NotAchievedException("Avoid should prevent reaching altitude")
        except AutoTestTimeoutException:
            pass
        except Exception as e:
            raise e

        # Check descent is not too fast, allow 10% above the configured backup speed
        max_descent_rate = -self.get_parameter("AVOID_BACKUP_SPD") * 1.1

        def get_climb_rate(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            if m.climb < max_descent_rate:
                raise NotAchievedException("Descending too fast want %f got %f" % (max_descent_rate, m.climb))

        self.context_push()
        self.install_message_hook_context(get_climb_rate)

        # Reduce fence alt, this will result in a fence breach, but there is no action.
        # Avoid should then backup the vehicle to be under the new fence alt.
        self.set_parameters({
            "FENCE_ALT_MAX": 50,
        })
        self.wait_altitude(40, 50, timeout=90, relative=True)

        self.context_pop()

        self.set_rc(3, 1500)
        self.do_RTL()

    # fly_alt_min_fence_test - fly down until you hit the fence floor
    def MinAltFence(self):
        '''Test Min Alt Fence'''
        self.takeoff(30, mode="LOITER", timeout=60)

        # enable fence, disable avoidance
        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_ENABLE" : 1,
            "FENCE_TYPE": 8,
            "FENCE_ALT_MIN": 20,
        })

        self.change_alt(30)

        # Activate the floor fence
        # TODO this test should run without requiring this
        self.do_fence_enable()

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160, timeout=60)
        self.set_rc(4, 1500)

        # fly forward (east) at least 20m
        self.set_rc(2, 1100)
        self.wait_distance(20)

        # stop flying forward and start flying down:
        self.set_rc_from_map({
            2: 1500,
            3: 1200,
        })

        # wait for fence to trigger
        self.wait_mode('RTL', timeout=120)

        self.wait_rtl_complete()

        # Disable the fence using mavlink command to ensure cleaned up SITL state
        self.do_fence_disable()

        self.zero_throttle()

    # MinAltFenceAvoid - fly down and make sure fence action does not trigger
    # Also check that the vehicle will not try and ascend too fast when trying to backup from a min alt fence due to avoidance
    def MinAltFenceAvoid(self):
        '''Test Min Alt Fence Avoidance'''

        # enable fence, only min altitude
        # No action, rely on avoidance to prevent the breach
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 8,
            "FENCE_ALT_MIN": 20,
            "FENCE_ACTION": 0,
        })
        self.reboot_sitl()

        self.takeoff(30, mode="LOITER")
        """Hold loiter position."""

        # Try and fly past the fence
        self.set_rc(3, 1120)

        # Avoid should prevent the vehicle flying past the fence, so the altitude wait should timeouts
        try:
            self.wait_altitude(10, 15, timeout=90, relative=True)
            raise NotAchievedException("Avoid should prevent reaching altitude")
        except AutoTestTimeoutException:
            pass
        except Exception as e:
            raise e

        # Check ascent is not too fast, allow 10% above the configured backup speed
        max_ascent_rate = self.get_parameter("AVOID_BACKUP_SPD") * 1.1

        def get_climb_rate(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            if m.climb > max_ascent_rate:
                raise NotAchievedException("Ascending too fast want %f got %f" % (max_ascent_rate, m.climb))

        self.context_push()
        self.install_message_hook_context(get_climb_rate)

        # Reduce fence alt, this will result in a fence breach, but there is no action.
        # Avoid should then backup the vehicle to be over the new fence alt.
        self.set_parameters({
            "FENCE_ALT_MIN": 30,
        })
        self.wait_altitude(30, 40, timeout=90, relative=True)

        self.context_pop()

        self.set_rc(3, 1500)
        self.do_RTL()

    def FenceFloorEnabledLanding(self):
        """Ensures we can initiate and complete an RTL while the fence is
        enabled.
        """
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.progress("Test Landing while fence floor enabled")
        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_ENABLE" : 1,
            "FENCE_TYPE": 15,
            "FENCE_ALT_MIN": 20,
            "FENCE_ALT_MAX": 30,
        })

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.user_takeoff(alt_min=25)

        # Check fence is enabled
        self.assert_fence_enabled()

        # Change to RC controlled mode
        self.change_mode('LOITER')

        self.set_rc(3, 1800)

        self.wait_mode('RTL', timeout=120)
        # center throttle
        self.set_rc(3, 1500)

        # wait until we are below the fence floor and re-enter loiter
        self.wait_altitude(5, 15, relative=True)
        self.change_mode('LOITER')
        # wait for manual recovery to expire
        self.delay_sim_time(15)

        # lower throttle and try and land
        self.set_rc(3, 1300)
        self.wait_altitude(0, 2, relative=True)
        self.zero_throttle()
        self.wait_landed_and_disarmed()
        self.assert_fence_enabled()
        # must not be in RTL
        self.assert_mode("LOITER")

        # Assert fence is healthy since it was enabled automatically
        self.assert_sensor_state(fence_bit, healthy=True)

        # Disable the fence using mavlink command to ensure cleaned up SITL state
        self.do_fence_disable()
        self.assert_fence_disabled()

    def FenceFloorAutoDisableLanding(self):
        """Ensures we can initiate and complete an RTL while the fence is enabled"""

        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.progress("Test Landing while fence floor enabled")
        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_TYPE": 11,
            "FENCE_ALT_MIN": 10,
            "FENCE_ALT_MAX": 20,
            "FENCE_AUTOENABLE" : 1,
        })

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff(alt_min=15, mode="GUIDED")

        # Check fence is enabled
        self.assert_fence_enabled()

        # Change to RC controlled mode
        self.change_mode('LOITER')

        self.set_rc(3, 1800)

        self.wait_mode('RTL', timeout=120)

        self.wait_landed_and_disarmed(0)
        # the breach should have cleared since we auto-disable the
        # fence on landing
        self.assert_fence_disabled()

        # Assert fences have gone now that we have landed and disarmed
        self.assert_sensor_state(fence_bit, present=True, enabled=False)

    def FenceFloorAutoEnableOnArming(self):
        """Ensures we can auto-enable fences on arming and still takeoff and land"""

        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_TYPE": 11,
            "FENCE_ALT_MIN": 10,
            "FENCE_ALT_MAX": 20,
            "FENCE_AUTOENABLE" : 3,
        })

        self.change_mode("GUIDED")
        # Check fence is not enabled
        self.assert_fence_disabled()

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff(alt_min=15, mode="GUIDED")

        # Check fence is enabled
        self.assert_fence_enabled()

        # Change to RC controlled mode
        self.change_mode('LOITER')

        self.set_rc(3, 1800)

        self.wait_mode('RTL', timeout=120)
        # Assert fence is not healthy now that we are in RTL
        self.assert_sensor_state(fence_bit, healthy=False)

        self.wait_landed_and_disarmed(0)
        # the breach should have cleared since we auto-disable the
        # fence on landing
        self.assert_fence_disabled()

        # Assert fences have gone now that we have landed and disarmed
        self.assert_sensor_state(fence_bit, present=True, enabled=False)

        # Disable the fence using mavlink command to ensure cleaned up SITL state
        self.assert_fence_disabled()

    def FenceMargin(self, timeout=180):
        '''Test warning on crossing fence margin'''
        # enable fence, disable avoidance
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 6,    # polygon and circle fences
            "FENCE_MARGIN" : 30,
            "FENCE_RADIUS" : 150,
            "AVOID_ENABLE": 0,
            "FENCE_OPTIONS": 4
        })

        self.change_mode("LOITER")
        self.wait_ready_to_arm()

        # fence requires home to be set:
        m = self.poll_home_position(quiet=False)

        # 110m polyfence
        home_loc = self.mav.location()
        radius = self.get_parameter("FENCE_RADIUS")
        if self.use_map and self.mavproxy is not None:
            self.mavproxy.send("map circle %f %f %f green\n" % (home_loc.lat, home_loc.lng, radius))

        locs = [
            self.offset_location_ne(home_loc, -110, -110),
            self.offset_location_ne(home_loc, 110, -110),
            self.offset_location_ne(home_loc, 110, 110),
            self.offset_location_ne(home_loc, -110, 110),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, locs),
        ])

        self.takeoff(10, mode="LOITER")

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160, timeout=60)
        self.set_rc(4, 1500)

        fence_radius = self.get_parameter("FENCE_RADIUS")

        self.progress("flying forward (east) until we hit fence")
        pitching_forward = True
        self.set_rc(2, 1100)
        self.wait_statustext("Polygon fence in ([0-9]+[.])?[0-9]?m", regex=True)

        self.wait_statustext("Circle and Polygon fences in ([0-9]+[.])?[0-9]?m", regex=True)
        self.progress("Waiting for fence breach")
        tstart = self.get_sim_time()
        while not self.mode_is("RTL"):
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("Did not breach fence")

            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            self.progress("Alt: %.02f  HomeDistance: %.02f (fence radius=%f)" %
                          (alt, home_distance, fence_radius))

        self.wait_statustext("Circle fence cleared margin breach")
        self.progress("Waiting until we get home and disarm")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            self.progress("Alt: %.02f  HomeDistance: %.02f" %
                          (alt, home_distance))
            # recenter pitch sticks once we're home so we don't fly off again
            if pitching_forward and home_distance < 50:
                pitching_forward = False
                self.set_rc(2, 1475)
                # disable fence
                self.set_parameter("FENCE_ENABLE", 0)
            if (alt <= 1 and home_distance < 10) or (not self.armed() and home_distance < 10):
                # reduce throttle
                self.zero_throttle()
                self.change_mode("LAND")
                self.wait_landed_and_disarmed()
                self.progress("Reached home OK")
                self.zero_throttle()
                return

        # give we're testing RTL, doing one here probably doesn't make sense
        home_distance = self.distance_to_home(use_cached_home=True)
        raise AutoTestTimeoutException(
            "Fence test failed to reach home (%fm distance) - "
            "timed out after %u seconds" % (home_distance, timeout,))

    def FenceUpload_MissionItem(self, timeout=180):
        '''Test MISSION_ITEM fence upload/download'''
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 6,  # polygon and circle fences
        })

        self.poll_home_position(quiet=False)

        home_loc = self.mav.location()

        fence_loc = [
            self.offset_location_ne(home_loc, -110, -110),
            self.offset_location_ne(home_loc, 110, -110),
            self.offset_location_ne(home_loc, 110, 110),
            self.offset_location_ne(home_loc, -110, 110),
        ]

        seq = 0
        items = []
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        count = len(fence_loc)
        for loc in fence_loc:
            item = self.mav.mav.mission_item_encode(
                1,
                1,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, 0,
                count, 0, 0, 0,
                loc.lat, loc.lng, 33.0,
                mission_type
            )
            items.append(item)
            seq += 1

        self.upload_using_mission_protocol(mission_type, items)
        downloaded_items = self.download_using_mission_protocol(mission_type)

        if len(downloaded_items) != len(items):
            raise NotAchievedException(f"Mismatch in number of items: sent={len(items)} received={len(downloaded_items)}")

        for i, (sent, received) in enumerate(zip(items, downloaded_items)):
            mismatches = []

            # Normalize lat/lon to float before comparison
            sent_lat = sent.x
            sent_lng = sent.y
            recv_lat = received.x / 1e7 if isinstance(received.x, int) else received.x
            recv_lng = received.y / 1e7 if isinstance(received.y, int) else received.y

            if sent.command != received.command:
                mismatches.append(f"command: {sent.command} != {received.command}")
            if not math.isclose(sent_lat, recv_lat, abs_tol=1e-2):
                mismatches.append(f"lat: {sent_lat} != {recv_lat}")
            if not math.isclose(sent_lng, recv_lng, abs_tol=1e-2):
                mismatches.append(f"lng: {sent_lng} != {recv_lng}")
            if not math.isclose(sent.param1, received.param1, abs_tol=1e-3):
                mismatches.append(f"param1: {sent.param1} != {received.param1}")

            if mismatches:
                raise NotAchievedException(f"Mismatch in item {i}: " + "; ".join(mismatches))

        print("Fence upload/download verification passed.")

    def assert_fence_not_breached(self):
        m = self.assert_receive_message('FENCE_STATUS', timeout=2)
        if m.breach_status != 0:
            raise NotAchievedException("Fence is breached")

    def assert_fence_breached(self):
        m = self.assert_receive_message('FENCE_STATUS', timeout=2)
        if m.breach_status == 0:
            raise NotAchievedException("Fence is not breached")

    def BackupFence(self):
        '''ensure the lateral backup fence functionality works'''
        self.set_parameters({
            "FENCE_RADIUS": 10,
            "FENCE_TYPE": 2,  # circle only
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 0,
        })
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
                               present=True,
                               enabled=True,
                               healthy=True,
                               timeout=30)
        self.assert_fence_not_breached()
        self.takeoff(10, mode='LOITER')
        self.set_rc(2, 1300)
        self.wait_mode('RTL', timeout=30)
        self.assert_fence_breached()
        self.change_mode('LOITER')
        # continue to suck as a pilot
        self.wait_mode('RTL', timeout=30)
        self.assert_fence_breached()
        self.set_rc(2, 1500)  # stop sucking
        self.wait_disarmed()
        self.assert_at_home()

    def GPSGlitchLoiter(self, timeout=30, max_distance=20):
        """fly_gps_glitch_loiter_test. Fly south east in loiter and test
        reaction to gps glitch."""
        self.takeoff(10, mode="LOITER")

        # turn on simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(True)

        # set-up gps glitch array
        glitch_lat = [0.0002996,
                      0.0006958,
                      0.0009431,
                      0.0009991,
                      0.0009444,
                      0.0007716,
                      0.0006221]
        glitch_lon = [0.0000717,
                      0.0000912,
                      0.0002761,
                      0.0002626,
                      0.0002807,
                      0.0002049,
                      0.0001304]
        glitch_num = len(glitch_lat)
        self.progress("GPS Glitches:")
        for i in range(1, glitch_num):
            self.progress("glitch %d %.7f %.7f" %
                          (i, glitch_lat[i], glitch_lon[i]))

        # turn south east
        self.progress("turn south east")
        self.set_rc(4, 1580)
        try:
            self.wait_heading(150)
            self.set_rc(4, 1500)
            # fly forward (south east) at least 60m
            self.set_rc(2, 1100)
            self.wait_distance(60)
            self.set_rc(2, 1500)
            # wait for copter to slow down
        except Exception as e:
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            raise e

        # record time and position
        tstart = self.get_sim_time()
        tnow = tstart
        start_pos = self.sim_location()

        # initialise current glitch
        glitch_current = 0
        self.progress("Apply first glitch")
        self.set_parameters({
            "SIM_GPS1_GLTCH_X": glitch_lat[glitch_current],
            "SIM_GPS1_GLTCH_Y": glitch_lon[glitch_current],
        })

        # record position for 30 seconds
        while tnow < tstart + timeout:
            tnow = self.get_sim_time_cached()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # turn off glitching if we've reached the end of glitch list
                if glitch_current >= glitch_num:
                    glitch_current = -1
                    self.progress("Completed Glitches")
                    self.set_parameters({
                        "SIM_GPS1_GLTCH_X": 0,
                        "SIM_GPS1_GLTCH_Y": 0,
                    })
                else:
                    self.progress("Applying glitch %u" % glitch_current)
                    # move onto the next glitch
                    self.set_parameters({
                        "SIM_GPS1_GLTCH_X": glitch_lat[glitch_current],
                        "SIM_GPS1_GLTCH_Y": glitch_lon[glitch_current],
                    })

            # start displaying distance moved after all glitches applied
            if glitch_current == -1:
                m = self.assert_receive_message(type='GLOBAL_POSITION_INT')
                alt = m.alt/1000.0 # mm -> m
                curr_pos = self.sim_location()
                moved_distance = self.get_distance(curr_pos, start_pos)
                self.progress("Alt: %.02f  Moved: %.0f" %
                              (alt, moved_distance))
                if moved_distance > max_distance:
                    raise NotAchievedException(
                        "Moved over %u meters, Failed!" % max_distance)
            else:
                self.drain_mav()

        # disable gps glitch
        if glitch_current != -1:
            self.set_parameters({
                "SIM_GPS1_GLTCH_X": 0,
                "SIM_GPS1_GLTCH_Y": 0,
            })
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        self.progress("GPS glitch test passed!"
                      "  stayed within %u meters for %u seconds" %
                      (max_distance, timeout))
        self.do_RTL()
        # re-arming is problematic because the GPS is glitching!
        self.reboot_sitl()

    def GPSGlitchLoiter2(self):
        """test vehicle handles GPS glitch (aka EKF Reset) without twitching"""
        self.context_push()
        self.takeoff(10, mode="LOITER")

        # wait for vehicle to level
        self.wait_attitude(desroll=0, despitch=0, timeout=10, tolerance=1)

        # apply glitch
        self.set_parameter("SIM_GPS1_GLTCH_X", 0.001)

        # check lean angles remain stable for 20 seconds
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 20:
            m = self.assert_receive_message('ATTITUDE')
            roll_deg = math.degrees(m.roll)
            pitch_deg = math.degrees(m.pitch)
            self.progress("checking att: roll=%f pitch=%f " % (roll_deg, pitch_deg))
            if abs(roll_deg) > 2 or abs(pitch_deg) > 2:
                raise NotAchievedException("fly_gps_glitch_loiter_test2 failed, roll or pitch moved during GPS glitch")

        # RTL, remove glitch and reboot sitl
        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

    def GPSGlitchAuto(self, timeout=180):
        '''fly mission and test reaction to gps glitch'''
        # set-up gps glitch array
        glitch_lat = [0.0002996,
                      0.0006958,
                      0.0009431,
                      0.0009991,
                      0.0009444,
                      0.0007716,
                      0.0006221]
        glitch_lon = [0.0000717,
                      0.0000912,
                      0.0002761,
                      0.0002626,
                      0.0002807,
                      0.0002049,
                      0.0001304]
        glitch_num = len(glitch_lat)
        self.progress("GPS Glitches:")
        for i in range(1, glitch_num):
            self.progress("glitch %d %.7f %.7f" %
                          (i, glitch_lat[i], glitch_lon[i]))

        # Fly mission #1
        self.progress("# Load copter_glitch_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_glitch_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_glitch_mission failed")

        # turn on simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(True)

        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.set_current_waypoint(1)

        self.change_mode("STABILIZE")
        self.wait_ready_to_arm()
        self.zero_throttle()
        self.arm_vehicle()

        # switch into AUTO mode and raise throttle
        self.change_mode('AUTO')
        self.set_rc(3, 1500)

        # wait until 100m from home
        try:
            self.wait_distance(100, 5, 90)
        except Exception as e:
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            raise e

        # stop and test loss of GPS for a short time - it should resume GPS use without falling back into a non aiding mode
        self.change_mode("LOITER")
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
        })
        self.delay_sim_time(2)
        self.set_parameters({
            "SIM_GPS1_ENABLE": 1,
        })
        # regaining GPS should not result in it falling back to a non-navigation mode
        self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=1)
        # It should still be navigating after enougnh time has passed for any pending timeouts to activate.
        self.delay_sim_time(10)
        self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=1)
        self.change_mode("AUTO")

        # record time and position
        tstart = self.get_sim_time()

        # initialise current glitch
        glitch_current = 0
        self.progress("Apply first glitch")
        self.set_parameters({
            "SIM_GPS1_GLTCH_X": glitch_lat[glitch_current],
            "SIM_GPS1_GLTCH_Y": glitch_lon[glitch_current],
        })

        # record position for 30 seconds
        while glitch_current < glitch_num:
            tnow = self.get_sim_time()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # apply next glitch
                if glitch_current < glitch_num:
                    self.progress("Applying glitch %u" % glitch_current)
                    self.set_parameters({
                        "SIM_GPS1_GLTCH_X": glitch_lat[glitch_current],
                        "SIM_GPS1_GLTCH_Y": glitch_lon[glitch_current],
                    })

        # turn off glitching
        self.progress("Completed Glitches")
        self.set_parameters({
            "SIM_GPS1_GLTCH_X": 0,
            "SIM_GPS1_GLTCH_Y": 0,
        })

        # continue with the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # wait for arrival back home
        self.wait_distance_to_home(0, 10, timeout=timeout)

        # turn off simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        self.progress("GPS Glitch test Auto completed: passed!")
        self.wait_disarmed()
        # re-arming is problematic because the GPS is glitching!
        self.reboot_sitl()

    #   fly_simple - assumes the simple bearing is initialised to be
    #   directly north flies a box with 100m west, 15 seconds north,
    #   50 seconds east, 15 seconds south
    def SimpleMode(self, side=50):
        '''Fly in SIMPLE mode'''
        self.takeoff(10, mode="LOITER")

        # set SIMPLE mode for all flight modes
        self.set_parameter("SIMPLE", 63)

        # switch to stabilize mode
        self.change_mode('STABILIZE')
        self.set_rc(3, 1545)

        # fly south 50m
        self.progress("# Flying south %u meters" % side)
        self.set_rc(1, 1300)
        self.wait_distance(side, 5, 60)
        self.set_rc(1, 1500)

        # fly west 8 seconds
        self.progress("# Flying west for 8 seconds")
        self.set_rc(2, 1300)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < (tstart + 8):
            self.assert_receive_message('VFR_HUD')
        self.set_rc(2, 1500)

        # fly north 25 meters
        self.progress("# Flying north %u meters" % (side/2.0))
        self.set_rc(1, 1700)
        self.wait_distance(side/2, 5, 60)
        self.set_rc(1, 1500)

        # fly east 8 seconds
        self.progress("# Flying east for 8 seconds")
        self.set_rc(2, 1700)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < (tstart + 8):
            self.assert_receive_message('VFR_HUD')
        self.set_rc(2, 1500)

        # hover in place
        self.hover()

        self.do_RTL(timeout=500)

    # fly_super_simple - flies a circle around home for 45 seconds
    def SuperSimpleCircle(self, timeout=45):
        '''Fly a circle in SUPER SIMPLE mode'''
        self.takeoff(10, mode="LOITER")

        # fly forward 20m
        self.progress("# Flying forward 20 meters")
        self.set_rc(2, 1300)
        self.wait_distance(20, 5, 60)
        self.set_rc(2, 1500)

        # set SUPER SIMPLE mode for all flight modes
        self.set_parameter("SUPER_SIMPLE", 63)

        # switch to stabilize mode
        self.change_mode("ALT_HOLD")
        self.set_rc(3, 1500)

        # start copter yawing slowly
        self.set_rc(4, 1550)

        # roll left for timeout seconds
        self.progress("# rolling left from pilot's POV for %u seconds"
                      % timeout)
        self.set_rc(1, 1300)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < (tstart + timeout):
            self.assert_receive_message('VFR_HUD')

        # stop rolling and yawing
        self.set_rc(1, 1500)
        self.set_rc(4, 1500)

        # restore simple mode parameters to default
        self.set_parameter("SUPER_SIMPLE", 0)

        # hover in place
        self.hover()

        self.do_RTL()

    # fly_circle - flies a circle with 20m radius
    def ModeCircle(self, holdtime=36):
        '''Fly CIRCLE mode'''
        # the following should not be required.  But there appears to
        # be a physics failure in the simulation which is causing CI
        # to fall over a lot.  -pb 202007021209
        self.reboot_sitl()

        self.takeoff(10, mode="LOITER")

        # face west
        self.progress("turn west")
        self.set_rc(4, 1580)
        self.wait_heading(270)
        self.set_rc(4, 1500)

        # set CIRCLE radius
        self.set_parameter("CIRCLE_RADIUS", 3000)

        # fly forward (east) at least 100m
        self.set_rc(2, 1100)
        self.wait_distance(100)
        # return pitch stick back to middle
        self.set_rc(2, 1500)

        # set CIRCLE mode
        self.change_mode('CIRCLE')

        # wait
        m = self.assert_receive_message('VFR_HUD')
        start_altitude = m.alt
        tstart = self.get_sim_time()
        self.progress("Circle at %u meters for %u seconds" %
                      (start_altitude, holdtime))
        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.assert_receive_message('VFR_HUD')
            self.progress("heading %d" % m.heading)

        self.progress("CIRCLE OK for %u seconds" % holdtime)

        self.do_RTL()

    def CompassMot(self):
        '''test code that adjust mag field for motor interference'''
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0,  # p1
            0,  # p2
            0,  # p3
            0,  # p4
            0,  # p5
            1,  # p6
            0  # p7
        )
        self.context_collect("STATUSTEXT")
        self.wait_statustext("Starting calibration", check_context=True)
        self.wait_statustext("Current", check_context=True)
        rc3_min = self.get_parameter('RC3_MIN')
        rc3_max = self.get_parameter('RC3_MAX')
        rc3_dz = self.get_parameter('RC3_DZ')

        def set_rc3_for_throttle_pct(thr_pct):
            value = int((rc3_min+rc3_dz) + (thr_pct/100.0) * (rc3_max-(rc3_min+rc3_dz)))
            self.progress("Setting rc3 to %u" % value)
            self.set_rc(3, value)

        throttle_in_pct = 0
        set_rc3_for_throttle_pct(throttle_in_pct)
        self.assert_received_message_field_values("COMPASSMOT_STATUS", {
            "interference": 0,
            "throttle": throttle_in_pct
        }, verbose=True, very_verbose=True)
        tstart = self.get_sim_time()
        delta = 5
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("did not run through entire range")
            throttle_in_pct += delta
            self.progress("Using throttle %f%%" % throttle_in_pct)
            set_rc3_for_throttle_pct(throttle_in_pct)
            self.wait_message_field_values("COMPASSMOT_STATUS", {
                "throttle": throttle_in_pct * 10.0,
            }, verbose=True, very_verbose=True, epsilon=1)
            if throttle_in_pct == 0:
                # finished counting down
                break
            if throttle_in_pct == 100:
                # start counting down
                delta = -delta

        m = self.wait_message_field_values("COMPASSMOT_STATUS", {
            "throttle": 0,
        }, verbose=True)
        for axis in "X", "Y", "Z":
            fieldname = "Compensation" + axis
            if getattr(m, fieldname) <= 0:
                raise NotAchievedException("Expected non-zero %s" % fieldname)

        # it's kind of crap - but any command-ack will stop the
        # calibration
        self.mav.mav.command_ack_send(0, 1)
        self.wait_statustext("Calibration successful")

    def MagFail(self):
        '''test failover of compass in EKF'''
        # we want both EK2 and EK3
        self.set_parameters({
            "EK2_ENABLE": 1,
            "EK3_ENABLE": 1,
        })

        self.takeoff(10, mode="LOITER")

        self.change_mode('CIRCLE')

        self.delay_sim_time(20)

        self.context_collect("STATUSTEXT")

        self.progress("Failing first compass")
        self.set_parameter("SIM_MAG1_FAIL", 1)

        # we want for the message twice, one for EK2 and again for EK3
        self.wait_statustext("EKF2 IMU0 switching to compass 1", check_context=True)
        self.wait_statustext("EKF3 IMU0 switching to compass 1", check_context=True)
        self.progress("compass switch 1 OK")

        self.delay_sim_time(2)

        self.context_clear_collection("STATUSTEXT")

        self.progress("Failing 2nd compass")
        self.set_parameter("SIM_MAG2_FAIL", 1)

        self.wait_statustext("EKF2 IMU0 switching to compass 2", check_context=True)
        self.wait_statustext("EKF3 IMU0 switching to compass 2", check_context=True)

        self.progress("compass switch 2 OK")

        self.delay_sim_time(2)

        self.context_clear_collection("STATUSTEXT")

        self.progress("Failing 3rd compass")
        self.set_parameter("SIM_MAG3_FAIL", 1)
        self.delay_sim_time(2)
        self.set_parameter("SIM_MAG1_FAIL", 0)

        self.wait_statustext("EKF2 IMU0 switching to compass 0", check_context=True)
        self.wait_statustext("EKF3 IMU0 switching to compass 0", check_context=True)
        self.progress("compass switch 0 OK")

        self.do_RTL()

    def TestEKF3CompassFailover(self):
        '''Test if changed compasses goes back to primary in EKF3 on disarm'''
        # Enable only EKF3
        self.set_parameters({
            "EK3_ENABLE": 1,
        })

        self.takeoff(10, mode="LOITER")

        self.context_collect("STATUSTEXT")

        self.progress("Disabling compass")
        self.set_parameter("SIM_MAG1_FAIL", 1)

        self.wait_statustext("EKF3 IMU0 switching to compass 1", check_context=True)
        self.wait_statustext("EKF3 IMU1 switching to compass 1", check_context=True)
        self.progress("Compass switch detected")

        self.context_clear_collection("STATUSTEXT")
        self.delay_sim_time(2)

        self.progress("Re-enabling compass")
        self.set_parameter("SIM_MAG1_FAIL", 0)

        self.context_clear_collection("STATUSTEXT")
        self.progress("Landing and disarming")
        self.land_and_disarm()

        self.wait_statustext("EKF3 IMU0 switching to compass 0", check_context=True)
        self.wait_statustext("EKF3 IMU1 switching to compass 0", check_context=True)
        self.progress("Compass restored")

        self.progress("Landing and disarming")
        self.land_and_disarm()

    def ModeFlip(self):
        '''Fly Flip Mode'''
        self.context_set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100)

        self.takeoff(20)

        self.progress("Flipping in roll")
        self.set_rc(1, 1700)
        self.send_cmd_do_set_mode('FLIP') # don't wait for success
        self.wait_attitude(despitch=0, desroll=45, tolerance=30)
        self.wait_attitude(despitch=0, desroll=90, tolerance=30)
        self.wait_attitude(despitch=0, desroll=-45, tolerance=30)
        self.progress("Waiting for level")
        self.set_rc(1, 1500) # can't change quickly enough!
        self.wait_attitude(despitch=0, desroll=0, tolerance=5)

        self.progress("Regaining altitude")
        self.change_mode('ALT_HOLD')
        self.wait_altitude(19, 60, relative=True)

        self.progress("Flipping in pitch")
        self.set_rc(2, 1700)
        self.send_cmd_do_set_mode('FLIP') # don't wait for success
        self.wait_attitude(despitch=45, desroll=0, tolerance=30)
        # can't check roll here as it flips from 0 to -180..
        self.wait_attitude(despitch=90, tolerance=30)
        self.wait_attitude(despitch=-45, tolerance=30)
        self.progress("Waiting for level")
        self.set_rc(2, 1500) # can't change quickly enough!
        self.wait_attitude(despitch=0, desroll=0, tolerance=5)

        self.do_RTL()

    def configure_EKFs_to_use_optical_flow_instead_of_GPS(self):
        '''configure EKF to use optical flow instead of GPS'''
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        if ahrs_ekf_type == 2:
            self.set_parameter("EK2_GPS_TYPE", 3)
        if ahrs_ekf_type == 3:
            self.set_parameters({
                "EK3_SRC1_POSXY": 0,
                "EK3_SRC1_VELXY": 5,
                "EK3_SRC1_VELZ": 0,
            })

    def OpticalFlowLocation(self):
        '''test optical flow doesn't supply location'''

        self.context_push()

        self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, False, False, False, verbose=True)

        self.start_subtest("Make sure no crash if no rangefinder")
        self.set_parameter("SIM_FLOW_ENABLE", 1)
        self.set_parameter("FLOW_TYPE", 10)

        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()

        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, True, True, True, verbose=True)

        self.change_mode('LOITER')
        self.delay_sim_time(5)
        self.wait_statustext("Need Position Estimate", timeout=300)

        self.context_pop()

        self.reboot_sitl()

    def OpticalFlow(self):
        '''test OpticalFlow in flight'''
        self.start_subtest("Make sure no crash if no rangefinder")

        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
        })

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        self.change_mode('LOITER')

        class CheckOpticalFlow(vehicle_test_suite.TestSuite.MessageHook):
            '''ensures OPTICAL_FLOW data matches other data'''
            def __init__(self, suite):
                super().__init__(suite)

                self.flow_rate_rads = 0
                self.rangefinder_distance = 0
                self.gps_speed = 0
                self.last_debug_time = 0
                self.distance_sensor_distance = 0

            def progress_prefix(self):
                return "COF: "

            def process(self, mav, m):
                m_type = m.get_type()
                if m_type == "OPTICAL_FLOW":
                    self.flow_rate_rads = math.sqrt(m.flow_comp_m_x**2+m.flow_comp_m_y**2)
                elif m_type == "RANGEFINDER":
                    self.rangefinder_distance = m.distance
                elif m_type == "DISTANCE_SENSOR":
                    self.distance_sensor_distance = m.current_distance * 0.01
                elif m_type == "GPS_RAW_INT":
                    self.gps_speed = m.vel/100.0  # cm/s -> m/s

                of_speed = self.flow_rate_rads * self.rangefinder_distance
                if abs(of_speed - self.gps_speed) > 3:
                    raise NotAchievedException(f"gps={self.gps_speed} vs of={of_speed} mismatch")
                of_speed = self.flow_rate_rads * self.distance_sensor_distance
                if abs(of_speed - self.gps_speed) > 3:
                    raise NotAchievedException(f"gps={self.gps_speed} vs of={of_speed} mismatch")

                now = self.suite.get_sim_time_cached()
                if now - self.last_debug_time > 5:
                    self.last_debug_time = now
                    self.progress(f"gps={self.gps_speed} of={of_speed}")

        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())

        check_optical_flow = CheckOpticalFlow(self)
        self.install_message_hook_context(check_optical_flow)

        self.fly_generic_mission("CMAC-copter-navtest.txt")

    def OpticalFlowLimits(self):
        '''test EKF navigation limiting'''
        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            "SIM_GPS1_ENABLE": 0,
            "SIM_TERRAIN": 0,
        })

        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        # we can't takeoff in loiter as we need flow healthy
        self.takeoff(alt_min=5, mode='ALT_HOLD', require_absolute=False, takeoff_throttle=1800)
        self.change_mode('LOITER')

        # speed should be limited to <10m/s
        self.set_rc(2, 1000)

        tstart = self.get_sim_time()
        timeout = 60
        started_climb = False
        while self.get_sim_time_cached() - tstart < timeout:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            spd = math.sqrt(m.vx**2 + m.vy**2) * 0.01
            alt = m.relative_alt*0.001

            # calculate max speed from altitude above the ground
            margin = 2.0
            max_speed = alt * 1.5 + margin
            self.progress("%0.1f: Low Speed: %f (want <= %u) alt=%.1f" %
                          (self.get_sim_time_cached() - tstart,
                           spd,
                           max_speed, alt))
            if spd > max_speed:
                raise NotAchievedException(("Speed should be limited by"
                                            "EKF optical flow limits"))

            # after 30 seconds start climbing
            if not started_climb and self.get_sim_time_cached() - tstart > 30:
                started_climb = True
                self.set_rc(3, 1900)
                self.progress("Moving higher")

            # check altitude is not climbing above 35m
            if alt > 35:
                raise NotAchievedException("Alt should be limited by EKF optical flow limits")
        self.reboot_sitl(force=True)

    def OpticalFlowCalibration(self):
        '''test optical flow calibration'''
        ex = None
        self.context_push()
        try:

            self.set_parameter("SIM_FLOW_ENABLE", 1)
            self.set_parameter("FLOW_TYPE", 10)
            self.set_analog_rangefinder_parameters()

            # RC9 starts/stops calibration
            self.set_parameter("RC9_OPTION", 158)

            # initialise flow scaling parameters to incorrect values
            self.set_parameter("FLOW_FXSCALER", -200)
            self.set_parameter("FLOW_FYSCALER", 200)

            self.reboot_sitl()

            # ensure calibration is off
            self.set_rc(9, 1000)

            # takeoff to 10m in loiter
            self.takeoff(10, mode="LOITER", require_absolute=True, timeout=720)

            # start calibration
            self.set_rc(9, 2000)

            tstart = self.get_sim_time()
            timeout = 90
            veh_dir_tstart = self.get_sim_time()
            veh_dir = 0
            while self.get_sim_time_cached() - tstart < timeout:
                # roll and pitch vehicle until samples collected
                # change direction of movement every 2 seconds
                if self.get_sim_time_cached() - veh_dir_tstart > 2:
                    veh_dir_tstart = self.get_sim_time()
                    veh_dir = veh_dir + 1
                    if veh_dir > 3:
                        veh_dir = 0
                if veh_dir == 0:
                    # move right
                    self.set_rc(1, 1800)
                    self.set_rc(2, 1500)
                if veh_dir == 1:
                    # move left
                    self.set_rc(1, 1200)
                    self.set_rc(2, 1500)
                if veh_dir == 2:
                    # move forward
                    self.set_rc(1, 1500)
                    self.set_rc(2, 1200)
                if veh_dir == 3:
                    # move back
                    self.set_rc(1, 1500)
                    self.set_rc(2, 1800)

            # return sticks to center
            self.set_rc(1, 1500)
            self.set_rc(2, 1500)

            # stop calibration (not actually necessary)
            self.set_rc(9, 1000)

            # check scaling parameters have been restored to values near zero
            flow_scalar_x = self.get_parameter("FLOW_FXSCALER")
            flow_scalar_y = self.get_parameter("FLOW_FYSCALER")
            if ((flow_scalar_x > 30) or (flow_scalar_x < -30)):
                raise NotAchievedException("FlowCal failed to set FLOW_FXSCALER correctly")
            if ((flow_scalar_y > 30) or (flow_scalar_y < -30)):
                raise NotAchievedException("FlowCal failed to set FLOW_FYSCALER correctly")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def AutoTune(self):
        """Test autotune mode"""

        # autotune changes a set of parameters on the vehicle which
        # are not in our context.  That changes the flight
        # characteristics, which we can't afford between runs.  So
        # completely reset the simulated vehicle after the run is
        # complete by "customising" the commandline here:
        self.customise_SITL_commandline([])

        self.set_parameters({
            "AUTOTUNE_AGGR": 0.1,
            "AUTOTUNE_MIN_D": 0.0004,
        })

        gain_names = [
            "ATC_RAT_RLL_D",
            "ATC_RAT_RLL_I",
            "ATC_RAT_RLL_P",
        ]

        ogains = self.get_parameters(gain_names)
        # set these parameters so they get reverted at the end of the test:
        self.set_parameters(ogains)

        self.takeoff(10)

        tstart = self.get_sim_time()

        self.change_mode('AUTOTUNE')

        self.wait_statustext("AutoTune: Success", timeout=5000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))

        # near enough for now:
        self.change_mode('LAND')
        self.wait_landed_and_disarmed()

        self.progress("checking the original gains have been re-instated")
        ngains = self.get_parameters(gain_names)
        for g in ngains.keys():
            if ogains[g] != ngains[g]:
                raise NotAchievedException(f"AUTOTUNE gains not discarded {ogains=} {ngains=}")

    def AutoTuneYawD(self):
        """Test autotune mode"""

        # autotune changes a set of parameters on the vehicle which
        # are not in our context.  That changes the flight
        # characteristics, which we can't afford between runs.  So
        # completely reset the simulated vehicle after the run is
        # complete by "customising" the commandline here:
        self.customise_SITL_commandline([])

        self.set_parameters({
            "AUTOTUNE_AGGR": 0.1,
            "AUTOTUNE_AXES": 12,
        })

        gain_names = [
            "ATC_RAT_RLL_D",
            "ATC_RAT_RLL_I",
            "ATC_RAT_RLL_P",
            "ATC_RAT_YAW_D",
        ]
        ogains = self.get_parameters(gain_names)
        # set these parameters so they get reverted at the end of the test:
        self.set_parameters(ogains)

        self.takeoff(10)

        tstart = self.get_sim_time()

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        # fail-fast if any determination fails:
        self.install_message_hook_context(vehicle_test_suite.TestSuite.FailFastStatusText(
            self, "determination failed"
        ))
        self.wait_statustext("AutoTune: Success", timeout=5000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))

        # near enough for now:
        self.change_mode('LAND')
        self.wait_landed_and_disarmed()

        self.progress("checking the original gains have been re-instated")
        ngains = self.get_parameters(gain_names)
        for g in ngains.keys():
            if ogains[g] != ngains[g]:
                raise NotAchievedException(f"AUTOTUNE gains not discarded {ogains=} {ngains=}")

    def EKF3SRCPerCore(self):
        '''Check SP/SH values in XKF4 for GPS (core 0) vs VICON (core 1) with targeted glitches.'''
        self.set_parameters({
            "EK3_ENABLE": 1,
            "AHRS_EKF_TYPE": 3,
            "VISO_TYPE": 2,
            "SERIAL5_PROTOCOL": 2,
            "EK3_SRC2_POSXY": 6,
            "EK3_SRC2_VELXY": 6,
            "EK3_SRC2_POSZ": 6,
            "EK3_SRC2_VELZ": 6,
            "EK3_SRC2_YAW": 6,
            "EK3_SRC_OPTIONS": 8  # bitmask: enable multiple EKF cores
        })

        self.customise_SITL_commandline(["--serial5=sim:vicon"])
        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.takeoff(3)
        self.delay_sim_time(5)

        self.progress("Injecting VICON glitch")
        self.set_parameter("SIM_VICON_GLIT_X", 100)
        self.set_parameter("SIM_VICON_GLIT_Y", 100)
        self.delay_sim_time(5)
        self.set_parameter("SIM_VICON_GLIT_X", 0)
        self.set_parameter("SIM_VICON_GLIT_Y", 0)

        self.progress("Injecting BARO glitch")
        self.set_parameter("SIM_BARO_GLITCH", 10)
        self.delay_sim_time(5)
        self.set_parameter("SIM_BARO_GLITCH", 0)

        self.do_RTL()
        self.wait_disarmed(timeout=100)

        dfreader = self.dfreader_for_current_onboard_log()

        max_SP_core0 = 0
        max_SP_core1 = 0
        max_SH_core0 = 0
        max_SH_core1 = 0

        while True:
            m = dfreader.recv_match(type="XKF4")
            if m is None:
                break
            if not hasattr(m, "C"):
                continue
            if m.C == 0:
                max_SP_core0 = max(max_SP_core0, m.SP)
                max_SH_core0 = max(max_SH_core0, m.SH)
            elif m.C == 1:
                max_SP_core1 = max(max_SP_core1, m.SP)
                max_SH_core1 = max(max_SH_core1, m.SH)

        self.progress(f"Core 0 (GPS): SP={max_SP_core0:.2f}, SH={max_SH_core0:.2f}")
        self.progress(f"Core 1 (VICON): SP={max_SP_core1:.2f}, SH={max_SH_core1:.2f}")

        # Validate: horizontal glitch only on core 1, vertical glitch only on core 0
        if max_SP_core0 > 0.1:
            raise NotAchievedException("Unexpected high SP in core 0 (GPS) during VICON glitch")
        if max_SP_core1 < 0.9:
            raise NotAchievedException("VICON glitch did not raise SP in core 1")

        if max_SH_core1 > 0.5:
            raise NotAchievedException("Unexpected high SH in core 1 (VICON) during BARO glitch")
        if max_SH_core0 < 0.9:
            raise NotAchievedException("BARO glitch did not raise SH in core 0")

    def AutoTuneSwitch(self):
        """Test autotune on a switch with gains being saved"""

        # autotune changes a set of parameters on the vehicle which
        # are not in our context.  That changes the flight
        # characteristics, which we can't afford between runs.  So
        # completely reset the simulated vehicle after the run is
        # complete by "customising" the commandline here:
        self.customise_SITL_commandline([])

        self.set_parameters({
            "RC8_OPTION": 17,
            "AUTOTUNE_AGGR": 0.1,
            "AUTOTUNE_MIN_D": 0.0004,
        })

        self.takeoff(10, mode='LOITER')

        def print_gains(name, gains):
            self.progress(f"AUTOTUNE {name} gains are P:%f I:%f D:%f" % (
                gains["ATC_RAT_RLL_P"],
                gains["ATC_RAT_RLL_I"],
                gains["ATC_RAT_RLL_D"]
            ))

        def get_roll_gains(name):
            ret = self.get_parameters([
                "ATC_RAT_RLL_D",
                "ATC_RAT_RLL_I",
                "ATC_RAT_RLL_P",
            ], verbose=False)
            print_gains(name, ret)
            return ret

        def gains_same(gains1, gains2):
            for c in 'P', 'I', 'D':
                p_name = f"ATC_RAT_RLL_{c}"
                if abs(gains1[p_name] - gains2[p_name]) > 0.00001:
                    return False
            return True

        self.progress("Take a copy of original gains")
        original_gains = get_roll_gains("pre-tuning")
        scaled_original_gains = copy.copy(original_gains)
        scaled_original_gains["ATC_RAT_RLL_I"] *= 0.1

        pre_rllt = self.get_parameter("ATC_RAT_RLL_FLTT")

        # hold position in loiter and run autotune
        self.set_rc(8, 1850)
        self.wait_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        sim_time_expected = 5000
        deadline = tstart + sim_time_expected
        while self.get_sim_time_cached() < deadline:
            now = self.get_sim_time_cached()
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("STATUSTEXT (%u<%u): %s" % (now, deadline, m.text))
            if "Determination Failed" in m.text:
                break
            if "AutoTune: Success" in m.text:
                self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
                post_gains = get_roll_gains("post")
                self.progress("Check original gains are used after tuning finished")
                if not gains_same(original_gains, post_gains):
                    raise NotAchievedException("AUTOTUNE original gains not restored")

                self.progress("Check original gains are re-instated by switch")
                self.set_rc(8, 1100)
                self.delay_sim_time(1)
                current_gains = get_roll_gains("set-original")
                if not gains_same(original_gains, current_gains):
                    raise NotAchievedException("AUTOTUNE original gains not restored")

                self.progress("Use autotuned gains")
                self.set_rc(8, 1850)
                self.delay_sim_time(1)
                tuned_gains = get_roll_gains("tuned")
                if gains_same(tuned_gains, original_gains):
                    raise NotAchievedException("AUTOTUNE tuned gains same as pre gains")
                if gains_same(tuned_gains, scaled_original_gains):
                    raise NotAchievedException("AUTOTUNE tuned gains same as scaled pre gains")

                self.progress("land without changing mode")
                self.set_rc(3, 1000)
                self.wait_altitude(-1, 5, relative=True)
                self.wait_disarmed()
                self.progress("Check gains are still there after disarm")
                disarmed_gains = get_roll_gains("post-disarm")
                if not gains_same(tuned_gains, disarmed_gains):
                    raise NotAchievedException("AUTOTUNE gains not present on disarm")

                self.reboot_sitl()
                self.progress("Check gains are still there after reboot")
                reboot_gains = get_roll_gains("post-reboot")
                if not gains_same(tuned_gains, reboot_gains):
                    raise NotAchievedException("AUTOTUNE gains not present on reboot")
                self.progress("Check FLTT is unchanged")
                if pre_rllt != self.get_parameter("ATC_RAT_RLL_FLTT"):
                    raise NotAchievedException("AUTOTUNE FLTT was modified")
                return

        raise NotAchievedException("AUTOTUNE failed (%u seconds)" %
                                   (self.get_sim_time() - tstart))

    def AutoTuneAux(self):
        """Test autotune with gains being tested using the Aux function"""

        # autotune changes a set of parameters on the vehicle which
        # are not in our context.  That changes the flight
        # characteristics, which we can't afford between runs.  So
        # completely reset the simulated vehicle after the run is
        # complete by "customising" the commandline here:
        self.customise_SITL_commandline([])

        self.set_parameters({
            "RC8_OPTION": 180,
            "AUTOTUNE_AGGR": 0.1,
            "AUTOTUNE_MIN_D": 0.0004,
        })

        self.takeoff(10, mode='LOITER')

        def print_gains(name, gains):
            self.progress(f"AUTOTUNE {name} gains are P:%f I:%f D:%f" % (
                gains["ATC_RAT_RLL_P"],
                gains["ATC_RAT_RLL_I"],
                gains["ATC_RAT_RLL_D"]
            ))

        def get_roll_gains(name):
            ret = self.get_parameters([
                "ATC_RAT_RLL_D",
                "ATC_RAT_RLL_I",
                "ATC_RAT_RLL_P",
            ], verbose=False)
            print_gains(name, ret)
            return ret

        def gains_same(gains1, gains2):
            for c in 'P', 'I', 'D':
                p_name = f"ATC_RAT_RLL_{c}"
                if abs(gains1[p_name] - gains2[p_name]) > 0.00001:
                    return False
            return True

        self.progress("Take a copy of original gains")
        original_gains = get_roll_gains("pre-tuning")
        scaled_original_gains = copy.copy(original_gains)
        scaled_original_gains["ATC_RAT_RLL_I"] *= 0.1

        pre_rllt = self.get_parameter("ATC_RAT_RLL_FLTT")

        # hold position in loiter and run autotune
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        sim_time_expected = 5000
        deadline = tstart + sim_time_expected
        while self.get_sim_time_cached() < deadline:
            now = self.get_sim_time_cached()
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("STATUSTEXT (%u<%u): %s" % (now, deadline, m.text))
            if "Determination Failed" in m.text:
                break
            if "AutoTune: Success" in m.text:
                self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
                post_gains = get_roll_gains("post")
                self.progress("Check original gains are used after tuning finished")
                if not gains_same(original_gains, post_gains):
                    raise NotAchievedException("AUTOTUNE original gains not restored")

                self.change_mode('LOITER')
                self.wait_mode('LOITER')

                self.progress("Use autotuned gains")
                self.set_rc(8, 1850)
                self.delay_sim_time(1)
                tuned_gains = get_roll_gains("tuned")
                if gains_same(tuned_gains, original_gains):
                    raise NotAchievedException("AUTOTUNE tuned gains same as pre gains")
                if gains_same(tuned_gains, scaled_original_gains):
                    raise NotAchievedException("AUTOTUNE tuned gains same as scaled pre gains")

                self.progress("Check original gains are re-instated by switch")
                self.set_rc(8, 1100)
                self.delay_sim_time(1)
                current_gains = get_roll_gains("set-original")
                if not gains_same(original_gains, current_gains):
                    raise NotAchievedException("AUTOTUNE original gains not restored")

                self.progress("land using Autotune Gains")
                self.set_rc(8, 1850)
                self.delay_sim_time(1)
                self.set_rc(3, 1000)
                self.wait_altitude(-1, 5, relative=True)
                self.wait_disarmed()
                self.progress("Check gains are still there after disarm")
                disarmed_gains = get_roll_gains("post-disarm")
                if not gains_same(tuned_gains, disarmed_gains):
                    raise NotAchievedException("AUTOTUNE gains not present on disarm")

                self.reboot_sitl()
                self.progress("Check gains are still there after reboot")
                reboot_gains = get_roll_gains("post-reboot")
                if not gains_same(tuned_gains, reboot_gains):
                    raise NotAchievedException("AUTOTUNE gains not present on reboot")
                self.progress("Check FLTT is unchanged")
                if pre_rllt != self.get_parameter("ATC_RAT_RLL_FLTT"):
                    raise NotAchievedException("AUTOTUNE FLTT was modified")
                return

        raise NotAchievedException("AUTOTUNE failed (%u seconds)" %
                                   (self.get_sim_time() - tstart))

    def EK3_RNG_USE_HGT(self):
        '''basic tests for using rangefinder when speed and height below thresholds'''
        # this takes advantage of some code in send_status_report
        # which only reports terrain variance when using switch-height
        # and using the rangefinder
        self.context_push()

        self.set_analog_rangefinder_parameters()
        # set use-height to 20m (the parameter is a percentage of max range)
        self.set_parameters({
            'EK3_RNG_USE_HGT': (20 / self.get_parameter('RNGFND1_MAX')) * 100,
        })
        self.reboot_sitl()

        # add a listener that verifies rangefinder innovations look good
        global alt
        alt = None

        def verify_innov(mav, m):
            global alt
            if m.get_type() == 'GLOBAL_POSITION_INT':
                alt = m.relative_alt * 0.001  # mm -> m
                return
            if m.get_type() != 'EKF_STATUS_REPORT':
                return
            if alt is None:
                return
            if alt > 1 and alt < 8:  # 8 is very low, but it takes a long time to start to use the rangefinder again
                zero_variance_wanted = False
            elif alt > 20:
                zero_variance_wanted = True
            else:
                return
            variance = m.terrain_alt_variance
            if zero_variance_wanted and variance > 0.00001:
                raise NotAchievedException("Wanted zero variance at height %f, got %f" % (alt, variance))
            elif not zero_variance_wanted and variance == 0:
                raise NotAchievedException("Wanted non-zero variance at alt=%f, got zero" % alt)

        self.install_message_hook_context(verify_innov)

        self.takeoff(50, mode='GUIDED')
        current_alt = self.mav.location().alt
        target_position = mavutil.location(
            -35.362938,
            149.165185,
            current_alt,
            0
        )

        self.fly_guided_move_to(target_position, timeout=300)

        self.change_mode('LAND')
        self.wait_disarmed()

        self.context_pop()

        self.reboot_sitl()

    def TerrainDBPreArm(self):
        '''test that pre-arm checks are working correctly for terrain database'''
        self.context_push()

        self.progress("# Load msission with terrain alt")
        # load the waypoint
        num_wp = self.load_mission("terrain_wp.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load terrain_wp failed")

        self.set_analog_rangefinder_parameters()
        self.set_parameters({
            "WPNAV_RFND_USE": 1,
            "TERRAIN_ENABLE": 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # make sure we can still arm with valid rangefinder and terrain db disabled
        self.set_parameter("TERRAIN_ENABLE", 0)
        self.wait_ready_to_arm()
        self.progress("# Vehicle armed with terrain db disabled")

        # make sure we can't arm with terrain db enabled and no rangefinder in us
        self.set_parameter("WPNAV_RFND_USE", 0)
        self.assert_prearm_failure("terrain disabled")

        self.context_pop()

        self.reboot_sitl()

    def CopterMission(self):
        '''fly mission which tests a significant number of commands'''
        # Fly mission #1
        self.progress("# Load copter_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")

        self.fly_loaded_mission(num_wp)

        self.progress("Auto mission completed: passed!")

    def set_origin(self, loc, timeout=60):
        '''set the GPS global origin to loc'''
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get non-zero lat")
            target_system = 1
            self.mav.mav.set_gps_global_origin_send(
                target_system,
                int(loc.lat * 1e7),
                int(loc.lng * 1e7),
                int(loc.alt * 1e3)
            )
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            self.progress("gpi=%s" % str(gpi))
            if gpi.lat != 0:
                break

    def MAV_CMD_DO_SET_GLOBAL_ORIGIN(self):
        '''test MAV_CMD_DO_SET_GLOBAL_ORIGIN command'''
        # don't run this test if command not available:
        try:
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN
        except AttributeError:
            return

        # disable the GPS so we don't get origin that way:
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
            "GPS1_TYPE": 0,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        # we must wait until we are using EKF3 as DCM does not allow
        # set_origin:
        self.wait_statustext("EKF3 active", check_context=True)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            p5=0,
            p6=0,
            p7=0,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            p5=0,
            p6=int(214*1e7),
            p7=0,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED
        )
        origin_lat = -23.322332
        origin_lng = 123.45678
        origin_alt = 23.67  # metres
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            p5=int(origin_lat*1e7),
            p6=int(origin_lng*1e7),
            p7=origin_alt
        )
        ggo = self.poll_message('GPS_GLOBAL_ORIGIN')
        self.assert_message_field_values(ggo, {
            "latitude": int(origin_lat*1e7),
            "longitude": int(origin_lng*1e7),
            "altitude": int(origin_alt*1000),  # m -> mm
        })

    def FarOrigin(self):
        '''fly a mission far from the vehicle origin'''
        # Fly mission #1
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
        })
        self.reboot_sitl()
        nz = mavutil.location(-43.730171, 169.983118, 1466.3, 270)
        self.set_origin(nz)
        self.set_parameters({
            "SIM_GPS1_ENABLE": 1,
        })
        self.progress("# Load copter_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")

        self.fly_loaded_mission(num_wp)

        self.progress("Auto mission completed: passed!")

    def fly_loaded_mission(self, num_wp):
        '''fly mission loaded on vehicle.  FIXME: get num_wp from vehicle'''
        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.set_current_waypoint(1)

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # switch into AUTO mode and raise throttle
        self.change_mode("AUTO")
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # set throttle to minimum
        self.zero_throttle()

        # wait for disarm
        self.wait_disarmed()
        self.progress("MOTORS DISARMED OK")

    def CANGPSCopterMission(self):
        '''fly mission which tests normal operation alongside CAN GPS'''
        self.set_parameters({
            "CAN_P1_DRIVER": 1,
            "GPS1_TYPE": 9,
            "GPS2_TYPE": 9,
            # disable simulated GPS, so only via DroneCAN
            "SIM_GPS1_ENABLE": 0,
            "SIM_GPS2_ENABLE": 0,
            # this ensures we use DroneCAN baro and compass
            "SIM_BARO_COUNT" : 0,
            "SIM_MAG1_DEVID" : 0,
            "SIM_MAG2_DEVID" : 0,
            "SIM_MAG3_DEVID" : 0,
            "COMPASS_USE2"   : 0,
            "COMPASS_USE3"   : 0,
            # use DroneCAN rangefinder
            "RNGFND1_TYPE" : 24,
            "RNGFND1_MAX" : 110.00,
            # use DroneCAN battery monitoring, and enforce with a arming voltage
            "BATT_MONITOR" : 8,
            "BATT_ARM_VOLT" : 12.0,
            "SIM_SPEEDUP": 2,
        })

        self.context_push()
        # enable only GPS arming check during ordering test
        self.set_parameter("ARMING_SKIPCHK", ~(1 << 3))
        self.context_collect('STATUSTEXT')

        self.reboot_sitl()
        # Test UAVCAN GPS ordering working
        gps1_det_text = self.wait_text("GPS 1: specified as DroneCAN.*", regex=True, check_context=True)
        gps2_det_text = self.wait_text("GPS 2: specified as DroneCAN.*", regex=True, check_context=True)
        gps1_nodeid = int(gps1_det_text.split('-')[1])
        gps2_nodeid = int(gps2_det_text.split('-')[1])
        if gps1_nodeid is None or gps2_nodeid is None:
            raise NotAchievedException("GPS not ordered per the order of Node IDs")

        self.context_stop_collecting('STATUSTEXT')

        GPS_Order_Tests = [[gps2_nodeid, gps2_nodeid, gps2_nodeid, 0,
                            "PreArm: Same Node Id {} set for multiple GPS".format(gps2_nodeid)],
                           [gps1_nodeid, int(gps2_nodeid/2), gps1_nodeid, 0,
                            "Selected GPS Node {} not set as instance {}".format(int(gps2_nodeid/2), 2)],
                           [int(gps1_nodeid/2), gps2_nodeid, 0, gps2_nodeid,
                            "Selected GPS Node {} not set as instance {}".format(int(gps1_nodeid/2), 1)],
                           [gps1_nodeid, gps2_nodeid, gps1_nodeid, gps2_nodeid, ""],
                           [gps2_nodeid, gps1_nodeid, gps2_nodeid, gps1_nodeid, ""],
                           [gps1_nodeid, 0, gps1_nodeid, gps2_nodeid, ""],
                           [0, gps2_nodeid, gps1_nodeid, gps2_nodeid, ""]]
        for case in GPS_Order_Tests:
            self.progress("############################### Trying Case: " + str(case))
            self.set_parameters({
                "GPS1_CAN_OVRIDE": case[0],
                "GPS2_CAN_OVRIDE": case[1],
            })
            self.drain_mav()
            self.context_collect('STATUSTEXT')
            self.reboot_sitl()
            gps1_det_text = None
            gps2_det_text = None
            try:
                gps1_det_text = self.wait_text("GPS 1: specified as DroneCAN.*", regex=True, check_context=True)
            except AutoTestTimeoutException:
                pass
            try:
                gps2_det_text = self.wait_text("GPS 2: specified as DroneCAN.*", regex=True, check_context=True)
            except AutoTestTimeoutException:
                pass

            self.context_stop_collecting('STATUSTEXT')
            self.change_mode('LOITER')
            if case[2] == 0 and case[3] == 0:
                if gps1_det_text or gps2_det_text:
                    raise NotAchievedException("Failed ordering for requested CASE:", case)

            if case[2] == 0 or case[3] == 0:
                if bool(gps1_det_text is not None) == bool(gps2_det_text is not None):
                    print(gps1_det_text)
                    print(gps2_det_text)
                    raise NotAchievedException("Failed ordering for requested CASE:", case)

            if gps1_det_text:
                if case[2] != int(gps1_det_text.split('-')[1]):
                    raise NotAchievedException("Failed ordering for requested CASE:", case)
            if gps2_det_text:
                if case[3] != int(gps2_det_text.split('-')[1]):
                    raise NotAchievedException("Failed ordering for requested CASE:", case)
            if len(case[4]):
                self.context_collect('STATUSTEXT')
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    p1=1,  # ARM
                    timeout=10,
                    want_result=mavutil.mavlink.MAV_RESULT_FAILED,
                )
                self.wait_statustext(case[4], check_context=True)
                self.context_stop_collecting('STATUSTEXT')
        self.progress("############################### All GPS Order Cases Tests Passed")
        self.progress("############################### Test Healthy Prearm check")
        self.set_parameter("ARMING_SKIPCHK", 0)
        self.stop_sup_program(instance=0)
        self.start_sup_program(instance=0, args="-M")
        self.stop_sup_program(instance=1)
        self.start_sup_program(instance=1, args="-M")
        self.delay_sim_time(2)
        self.context_collect('STATUSTEXT')
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=1,  # ARM
            timeout=10,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.wait_statustext(".*Node .* unhealthy", check_context=True, regex=True)
        self.stop_sup_program(instance=0)
        self.start_sup_program(instance=0)
        self.stop_sup_program(instance=1)
        self.start_sup_program(instance=1)
        self.context_stop_collecting('STATUSTEXT')
        self.context_pop()

        self.set_parameters({
            # use DroneCAN ESCs for flight
            "CAN_D1_UC_ESC_BM" : 0x0f,
            # this stops us using local servo output, guaranteeing we are
            # flying on DroneCAN ESCs
            "SIM_CAN_SRV_MSK" : 0xFF,
            # we can do the flight faster
            "SIM_SPEEDUP" : 5,
        })

        self.CopterMission()

    def TakeoffAlt(self):
        '''Test Takeoff command altitude'''
        # Test case #1 (set target altitude to relative -10m from the ground, -10m is invalid, so it is set to 1m)
        self.progress("Testing relative alt from the ground")
        self.do_takeoff_alt("copter_takeoff.txt", 1, False)
        # Test case #2 (set target altitude to relative -10m during flight, -10m is invalid, so keeps current altitude)
        self.progress("Testing relative alt during flight")
        self.do_takeoff_alt("copter_takeoff.txt", 10, True)

        self.progress("Takeoff mission completed: passed!")

    def do_takeoff_alt(self, mission_file, target_alt, during_flight=False):
        self.progress("# Load %s" % mission_file)
        # load the waypoint count
        num_wp = self.load_mission(mission_file, strict=False)
        if not num_wp:
            raise NotAchievedException("load %s failed" % mission_file)

        self.set_current_waypoint(1)

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        if during_flight:
            self.user_takeoff(alt_min=target_alt)

        # switch into AUTO mode and raise throttle
        self.change_mode("AUTO")
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # altitude check
        self.wait_altitude(target_alt - 1 , target_alt + 1, relative=True)

        self.change_mode('LAND')

        # set throttle to minimum
        self.zero_throttle()

        # wait for disarm
        self.wait_disarmed()
        self.progress("MOTORS DISARMED OK")

    def GuidedEKFLaneChange(self):
        '''test lane change with GPS diff on startup'''
        self.set_parameters({
            "EK3_SRC1_POSZ": 3,
            "EK3_AFFINITY" : 1,
            "GPS2_TYPE" : 1,
            "SIM_GPS2_ENABLE" : 1,
            "SIM_GPS2_GLTCH_Z" : -30
            })
        self.reboot_sitl()

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()

        self.delay_sim_time(10, reason='"both EKF lanes to init"')

        self.set_parameters({
            "SIM_GPS2_GLTCH_Z" : 0
            })

        self.delay_sim_time(20, reason="EKF to do a position Z reset")

        self.arm_vehicle()
        self.user_takeoff(alt_min=20)
        gps_alt = self.get_altitude(altitude_source='GPS_RAW_INT.alt')
        self.progress("Initial guided alt=%.1fm" % gps_alt)

        self.context_collect('STATUSTEXT')
        self.progress("force a lane change")
        self.set_parameters({
            "INS_ACCOFFS_X" : 5
            })
        self.wait_statustext("EKF3 lane switch 1", timeout=10, check_context=True)

        self.watch_altitude_maintained(
            altitude_min=gps_alt-2,
            altitude_max=gps_alt+2,
            altitude_source='GPS_RAW_INT.alt',
            minimum_duration=10,
        )

        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def test_EKF3_option_disable_lane_switch(self):
        '''Test that EK3_OPTION disables lane switching, and EK3_PRIMARY forces switch when re-enabled'''

        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "EK3_IMU_MASK": 3,       # Use IMU0 and IMU1
            "EK3_OPTIONS": 2,         # Disable lane switching
            "EK3_PRIMARY": 0,        # Start with lane 0
        })

        self.reboot_sitl()

        self.lane_switches = []

        # Hook to track STATUSTEXT messages for EKF lane switches
        def statustext_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            if message.text.startswith("EKF primary changed:"):
                try:
                    lane = int(message.text.strip().split(":")[-1])
                    self.lane_switches.append(lane)
                except ValueError:
                    pass  # ignore malformed messages

        self.install_message_hook_context(statustext_hook)

        self.takeoff(50, mode='ALT_HOLD')

        self.delay_sim_time(5)

        ####################################################################################
        self.start_subtest("Ensure no lane switch occurs with EK3_OPTIONS = 2")
        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "INS_ACCOFFS_X" : 5
        })

        self.delay_sim_time(10)  # Wait to confirm no switch
        if self.lane_switches:
            raise NotAchievedException(f"Unexpected lane switch occurred: {self.lane_switches}")
        self.progress("Success: No lane switch occurred with EK3_OPTIONS = 2")
        self.context_clear_collection("STATUSTEXT")
        self.set_parameters({
            "INS_ACCOFFS_X" : 0.01,
        })

        ####################################################################################
        self.start_subtest("EK3_PRIMARY = 1 (expect switch)")

        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "EK3_PRIMARY": 1,
        })

        # Wait for automatic lane switch to occur
        self.wait_statustext(
            text="EKF primary changed:1",
            timeout=30,
            check_context=True
        )

        self.context_clear_collection("STATUSTEXT")
        self.disarm_vehicle(force=True)

    def MotorFail(self, ):
        """Test flight with reduced motor efficiency"""
        # we only expect an octocopter to survive ATM:
        self.MotorFail_test_frame('octa', 8, frame_class=3)
        # self.MotorFail_test_frame('hexa', 6, frame_class=2)
        # self.MotorFail_test_frame('y6', 6, frame_class=5)

    def MotorFail_test_frame(self, model, servo_count, frame_class, fail_servo=0, fail_mul=0.0, holdtime=30):
        self.set_parameters({
            'FRAME_CLASS': frame_class,
        })
        self.customise_SITL_commandline([], model=model)

        self.takeoff(25, mode="LOITER")

        # Get initial values
        start_hud = self.assert_receive_message('VFR_HUD')
        start_attitude = self.assert_receive_message('ATTITUDE')

        hover_time = 5
        tstart = self.get_sim_time()
        int_error_alt = 0
        int_error_yaw_rate = 0
        int_error_yaw = 0
        self.progress("Hovering for %u seconds" % hover_time)
        failed = False
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > holdtime + hover_time:
                break

            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            hud = self.assert_receive_message('VFR_HUD')
            attitude = self.assert_receive_message('ATTITUDE')

            if not failed and now - tstart > hover_time:
                self.progress("Killing motor %u (%u%%)" %
                              (fail_servo+1, fail_mul))
                self.set_parameters({
                    "SIM_ENGINE_MUL": fail_mul,
                    "SIM_ENGINE_FAIL": 1 << fail_servo,
                })
                failed = True

            if failed:
                self.progress("Hold Time: %f/%f" % (now-tstart, holdtime))

            servo_pwm = [
                servo.servo1_raw,
                servo.servo2_raw,
                servo.servo3_raw,
                servo.servo4_raw,
                servo.servo5_raw,
                servo.servo6_raw,
                servo.servo7_raw,
                servo.servo8_raw,
            ]

            self.progress("PWM output per motor")
            for i, pwm in enumerate(servo_pwm[0:servo_count]):
                if pwm > 1900:
                    state = "oversaturated"
                elif pwm < 1200:
                    state = "undersaturated"
                else:
                    state = "OK"

                if failed and i == fail_servo:
                    state += " (failed)"

                self.progress("servo %u [pwm=%u] [%s]" % (i+1, pwm, state))

            alt_delta = hud.alt - start_hud.alt
            yawrate_delta = attitude.yawspeed - start_attitude.yawspeed
            yaw_delta = attitude.yaw - start_attitude.yaw

            self.progress("Alt=%fm (delta=%fm)" % (hud.alt, alt_delta))
            self.progress("Yaw rate=%f (delta=%f) (rad/s)" %
                          (attitude.yawspeed, yawrate_delta))
            self.progress("Yaw=%f (delta=%f) (deg)" %
                          (attitude.yaw, yaw_delta))

            dt = self.get_sim_time() - now
            int_error_alt += abs(alt_delta/dt)
            int_error_yaw_rate += abs(yawrate_delta/dt)
            int_error_yaw += abs(yaw_delta/dt)
            self.progress("## Error Integration ##")
            self.progress("  Altitude: %fm" % int_error_alt)
            self.progress("  Yaw rate: %f rad/s" % int_error_yaw_rate)
            self.progress("  Yaw: %f deg" % int_error_yaw)
            self.progress("----")

            if int_error_yaw > 5:
                raise NotAchievedException("Vehicle is spinning")

            if alt_delta < -20:
                raise NotAchievedException("Vehicle is descending")

        self.progress("Fixing motors")
        self.set_parameter("SIM_ENGINE_FAIL", 0)

        self.do_RTL()

    def hover_for_interval(self, hover_time):
        '''hovers for an interval of hover_time seconds.  Returns the bookend
        times for that interval (in time-since-boot frame), and the
        output throttle level at the end of the period.
        '''
        self.progress("Hovering for %u seconds" % hover_time)
        tstart = self.get_sim_time()
        self.delay_sim_time(hover_time, reason='data collection')
        vfr_hud = self.poll_message('VFR_HUD')
        tend = self.get_sim_time()
        return tstart, tend, vfr_hud.throttle

    def MotorVibration(self):
        """Test flight with motor vibration"""
        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            # these are real values taken from a 180mm Quad:
            "SIM_GYR1_RND": 20,
            "SIM_ACC1_RND": 5,
            "SIM_ACC2_RND": 5,
            "SIM_INS_THR_MIN": 0.1,
        })
        self.reboot_sitl()

        # do a simple up-and-down flight to gather data:
        self.takeoff(15, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        # if we don't reduce vibes here then the landing detector
        # may not trigger
        self.set_parameter("SIM_VIB_MOT_MAX", 0)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
        # ignore the first 20Hz and look for a peak at -15dB or more
        # it should be at about 190Hz, each bin is 1000/1024Hz wide
        ignore_bins = int(100 * 1.024)  # start at 100Hz to be safe
        freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
        if numpy.amax(psd["X"][ignore_bins:]) < -15 or freq < 100 or freq > 300:
            raise NotAchievedException(
                "Did not detect a motor peak, found %f at %f dB" %
                (freq, numpy.amax(psd["X"][ignore_bins:])))
        else:
            self.progress("Detected motor peak at %fHz" % freq)

        # now add a notch and check that post-filter the peak is squashed below 40dB
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTC2_ENABLE": 1,
            "INS_HNTC2_FREQ": freq,
            "INS_HNTC2_ATT": 50,
            "INS_HNTC2_BW": freq/2,
            "INS_HNTC2_MODE": 0,
            "SIM_VIB_MOT_MAX": 350,
        })
        self.reboot_sitl()

        # do a simple up-and-down flight to gather data:
        self.takeoff(15, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        self.set_parameter("SIM_VIB_MOT_MAX", 0)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
        freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
        peakdB = numpy.amax(psd["X"][ignore_bins:])
        if peakdB < -23:
            self.progress("Did not detect a motor peak, found %f at %f dB" % (freq, peakdB))
        else:
            raise NotAchievedException("Detected peak %.1f Hz %.2f dB" % (freq, peakdB))

    def VisionPosition(self):
        """Disable GPS navigation, enable Vicon input."""
        # scribble down a location we can set origin to:

        self.customise_SITL_commandline(["--serial5=sim:vicon:"])
        self.progress("Waiting for location")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        old_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        print("old_pos=%s" % str(old_pos))

        # configure EKF to use external nav instead of GPS
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        if ahrs_ekf_type == 2:
            self.set_parameter("EK2_GPS_TYPE", 3)
        if ahrs_ekf_type == 3:
            self.set_parameters({
                "EK3_SRC1_POSXY": 6,
                "EK3_SRC1_VELXY": 6,
                "EK3_SRC1_POSZ": 6,
                "EK3_SRC1_VELZ": 6,
            })
        self.set_parameters({
            "GPS1_TYPE": 0,
            "VISO_TYPE": 1,
            "SERIAL5_PROTOCOL": 1,
        })
        self.reboot_sitl()
        # without a GPS or some sort of external prompting, AP
        # doesn't send system_time messages.  So prompt it:
        self.mav.mav.system_time_send(int(time.time() * 1000000), 0)
        self.progress("Waiting for non-zero-lat")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise AutoTestTimeoutException("Did not get non-zero lat")
            self.mav.mav.set_gps_global_origin_send(1,
                                                    old_pos.lat,
                                                    old_pos.lon,
                                                    old_pos.alt)
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            self.progress("gpi=%s" % str(gpi))
            if gpi.lat != 0:
                break

        self.takeoff()
        self.set_rc(1, 1600)
        tstart = self.get_sim_time()
        while True:
            vicon_pos = self.assert_receive_message('VISION_POSITION_ESTIMATE')
            # print("vpe=%s" % str(vicon_pos))
            # gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            # self.progress("gpi=%s" % str(gpi))
            if vicon_pos.x > 40:
                break

            if self.get_sim_time_cached() - tstart > 100:
                raise AutoTestTimeoutException("Vicon showed no movement")

        # recenter controls:
        self.set_rc(1, 1500)
        self.progress("# Enter RTL")
        self.change_mode('RTL')
        self.set_rc(3, 1500)
        tstart = self.get_sim_time()
        # self.install_messageprinter_handlers_context(['SIMSTATE', 'GLOBAL_POSITION_INT'])
        self.wait_disarmed(timeout=200)

    def BodyFrameOdom(self):
        """Disable GPS navigation, enable input of VISION_POSITION_DELTA."""

        if self.get_parameter("AHRS_EKF_TYPE") != 3:
            # only tested on this EKF
            return

        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        if self.current_onboard_log_contains_message("XKFD"):
            raise NotAchievedException("Found unexpected XKFD message")

        # scribble down a location we can set origin to:
        self.progress("Waiting for location")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        old_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        print("old_pos=%s" % str(old_pos))

        # configure EKF to use external nav instead of GPS
        self.set_parameters({
            "EK3_SRC1_POSXY": 6,
            "EK3_SRC1_VELXY": 6,
            "EK3_SRC1_POSZ": 6,
            "EK3_SRC1_VELZ": 6,
            "GPS1_TYPE": 0,
            "VISO_TYPE": 1,
            "SERIAL5_PROTOCOL": 1,
            "SIM_VICON_TMASK": 8,  # send VISION_POSITION_DELTA
        })
        self.reboot_sitl()
        # without a GPS or some sort of external prompting, AP
        # doesn't send system_time messages.  So prompt it:
        self.mav.mav.system_time_send(int(time.time() * 1000000), 0)
        self.progress("Waiting for non-zero-lat")
        tstart = self.get_sim_time()
        while True:
            self.mav.mav.set_gps_global_origin_send(1,
                                                    old_pos.lat,
                                                    old_pos.lon,
                                                    old_pos.alt)
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            self.progress("gpi=%s" % str(gpi))
            if gpi.lat != 0:
                break

            if self.get_sim_time_cached() - tstart > 60:
                raise AutoTestTimeoutException("Did not get non-zero lat")

        self.takeoff(alt_min=5, mode='ALT_HOLD', require_absolute=False, takeoff_throttle=1800)
        self.change_mode('LAND')
        # TODO: something more elaborate here - EKF will only aid
        # relative position
        self.wait_disarmed()
        if not self.current_onboard_log_contains_message("XKFD"):
            raise NotAchievedException("Did not find expected XKFD message")

    def FlyMissionTwice(self):
        '''fly a mission twice in a row without changing modes in between.
        Seeks to show bugs in mission state machine'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        num_wp = self.get_mission_count()
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        for i in 1, 2:
            self.progress("run %u" % i)
            self.arm_vehicle()
            self.wait_waypoint(num_wp-1, num_wp-1)
            self.wait_disarmed()
            self.delay_sim_time(20)

    def FlyMissionTwiceWithReset(self):
        '''Fly a mission twice in a row without changing modes in between.
        Allow the mission to complete, then reset the mission state machine and restart the mission.'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        num_wp = self.get_mission_count()
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        for i in 1, 2:
            self.progress("run %u" % i)
            # Use the "Reset Mission" param of DO_SET_MISSION_CURRENT to reset mission state machine
            self.set_current_waypoint_using_mav_cmd_do_set_mission_current(seq=0, reset=1)
            self.arm_vehicle()
            self.wait_waypoint(num_wp-1, num_wp-1)
            self.wait_disarmed()
            self.delay_sim_time(20)

    def MissionIndexValidity(self):
        '''Confirm that attempting to select an invalid mission item is rejected.'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        num_wp = self.get_mission_count()
        accepted_indices = [0, 1, num_wp-1]
        denied_indices = [-1, num_wp]

        for seq in accepted_indices:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                         p1=seq,
                         timeout=1,
                         want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)

        for seq in denied_indices:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                         p1=seq,
                         timeout=1,
                         want_result=mavutil.mavlink.MAV_RESULT_DENIED)

    def InvalidJumpTags(self):
        '''Verify the behaviour when selecting invalid jump tags.'''

        MAX_TAG_NUM = 65535
        # Jump tag is not present, so expect FAILED
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
                     p1=MAX_TAG_NUM,
                     timeout=1,
                     want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # Jump tag is too big, so expect DENIED
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
                     p1=MAX_TAG_NUM+1,
                     timeout=1,
                     want_result=mavutil.mavlink.MAV_RESULT_DENIED)

    def GPSViconSwitching(self):
        """Fly GPS and Vicon switching test"""
        """Setup parameters including switching to EKF3"""
        self.set_parameters({
            "VISO_TYPE": 2,      # enable vicon
            "SERIAL5_PROTOCOL": 2,
            "EK3_ENABLE": 1,
            "EK3_SRC2_POSXY": 6, # External Nav
            "EK3_SRC2_POSZ": 6,  # External Nav
            "EK3_SRC2_VELXY": 6, # External Nav
            "EK3_SRC2_VELZ": 6,  # External Nav
            "EK3_SRC2_YAW": 6,   # External Nav
            "RC7_OPTION": 80,    # RC aux switch 7 set to Viso Align
            "RC8_OPTION": 90,    # RC aux switch 8 set to EKF source selector
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
        })
        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        # switch to use GPS
        self.set_rc(8, 1000)

        # ensure we can get a global position:
        self.poll_home_position(timeout=120)

        # record starting position
        old_pos = self.get_global_position_int()
        print("old_pos=%s" % str(old_pos))

        # align vicon yaw with ahrs heading
        self.set_rc(7, 2000)

        # takeoff to 10m in Loiter
        self.progress("Moving to ensure location is tracked")
        self.takeoff(10, mode="LOITER", require_absolute=True, timeout=720)

        # fly forward in Loiter
        self.set_rc(2, 1300)

        # disable vicon
        self.set_parameter("SIM_VICON_FAIL", 1)

        # ensure vehicle remain in Loiter for 15 seconds
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 15:
            if not self.mode_is('LOITER'):
                raise NotAchievedException("Expected to stay in loiter for >15 seconds")

        # re-enable vicon
        self.set_parameter("SIM_VICON_FAIL", 0)

        # switch to vicon, disable GPS and wait 10sec to ensure vehicle remains in Loiter
        self.set_rc(8, 1500)
        self.set_parameter("GPS1_TYPE", 0)

        # ensure vehicle remain in Loiter for 15 seconds
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 15:
            if not self.mode_is('LOITER'):
                raise NotAchievedException("Expected to stay in loiter for >15 seconds")

        # RTL and check vehicle arrives within 10m of home
        self.set_rc(2, 1500)
        self.do_RTL()

    def RTLSpeed(self):
        """Test RTL Speed parameters"""
        rtl_speed_ms = 7
        wpnav_speed_ms = 4
        wpnav_accel_mss = 3
        tolerance = 0.5
        self.load_mission("copter_rtl_speed.txt")
        self.set_parameters({
            'WPNAV_ACCEL': wpnav_accel_mss * 100,
            'RTL_SPEED': rtl_speed_ms * 100,
            'WPNAV_SPEED': wpnav_speed_ms * 100,
        })
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1600)
        self.wait_altitude(19, 25, relative=True)
        self.wait_groundspeed(wpnav_speed_ms-tolerance, wpnav_speed_ms+tolerance)
        self.monitor_groundspeed(wpnav_speed_ms, timeout=20)
        self.change_mode('RTL')
        self.wait_groundspeed(rtl_speed_ms-tolerance, rtl_speed_ms+tolerance)
        self.monitor_groundspeed(rtl_speed_ms, timeout=5)
        self.change_mode('AUTO')
        self.wait_groundspeed(0-tolerance, 0+tolerance)
        self.wait_groundspeed(wpnav_speed_ms-tolerance, wpnav_speed_ms+tolerance)
        self.monitor_groundspeed(wpnav_speed_ms, tolerance=0.6, timeout=5)
        self.do_RTL()

    def NavDelay(self):
        """Fly a simple mission that has a delay in it."""

        self.load_mission("copter_nav_delay.txt")

        self.set_parameter("DISARM_DELAY", 0)

        self.change_mode("LOITER")
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.change_mode("AUTO")
        self.set_rc(3, 1600)
        count_start = -1
        count_stop = -1
        tstart = self.get_sim_time()
        last_mission_current_msg = 0
        last_seq = None
        while self.armed(): # we RTL at end of mission
            now = self.get_sim_time_cached()
            if now - tstart > 200:
                raise AutoTestTimeoutException("Did not disarm as expected")
            m = self.assert_receive_message('MISSION_CURRENT')
            at_delay_item = ""
            if m.seq == 3:
                at_delay_item = "(At delay item)"
                if count_start == -1:
                    count_start = now
            if ((now - last_mission_current_msg) > 1 or m.seq != last_seq):
                dist = None
                x = self.mav.messages.get("NAV_CONTROLLER_OUTPUT", None)
                if x is not None:
                    dist = x.wp_dist
                    self.progress("MISSION_CURRENT.seq=%u dist=%s %s" %
                                  (m.seq, dist, at_delay_item))
                last_mission_current_msg = self.get_sim_time_cached()
                last_seq = m.seq
            if m.seq > 3:
                if count_stop == -1:
                    count_stop = now
        calculated_delay = count_stop - count_start
        want_delay = 59 # should reflect what's in the mission file
        self.progress("Stopped for %u seconds (want >=%u seconds)" %
                      (calculated_delay, want_delay))
        if calculated_delay < want_delay:
            raise NotAchievedException("Did not delay for long enough")

    def RangeFinder(self):
        '''Test RangeFinder Basic Functionality'''
        self.assert_not_receiving_message('RANGEFINDER', timeout=5)
        self.assert_not_receiving_message('DISTANCE_SENSOR', timeout=5)

        # may need to force a rotation if some other test has used the
        # rangefinder...
        self.progress("Ensure no RFND messages in log")
        self.set_parameter("LOG_DISARMED", 1)
        if self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("Found unexpected RFND message")

        self.set_analog_rangefinder_parameters()
        self.set_parameter("RC9_OPTION", 10) # rangefinder
        self.set_rc(9, 2000)

        self.reboot_sitl()

        self.progress("Making sure we now get RANGEFINDER messages")
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())
        m = self.assert_receive_message('RANGEFINDER', timeout=10)

        self.progress("Making sure we now get DISTANCE_SENSOR messages")
        self.assert_receive_message('DISTANCE_SENSOR', timeout=10)

        self.progress("Checking RangeFinder is marked as enabled in mavlink")
        m = self.assert_receive_message('SYS_STATUS', timeout=10)
        flags = m.onboard_control_sensors_enabled
        if not flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
            raise NotAchievedException("Laser not enabled in SYS_STATUS")
        self.progress("Disabling laser using switch")
        self.set_rc(9, 1000)
        self.delay_sim_time(1)
        self.progress("Checking RangeFinder is marked as disabled in mavlink")
        m = self.assert_receive_message('SYS_STATUS', timeout=10)
        flags = m.onboard_control_sensors_enabled
        if flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
            raise NotAchievedException("Laser enabled in SYS_STATUS")

        self.progress("Re-enabling rangefinder")
        self.set_rc(9, 2000)
        self.delay_sim_time(1)
        m = self.assert_receive_message('SYS_STATUS', timeout=10)
        flags = m.onboard_control_sensors_enabled
        if not flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
            raise NotAchievedException("Laser not enabled in SYS_STATUS")

        self.takeoff(10, mode="LOITER")

        m_p = self.assert_receive_message('GLOBAL_POSITION_INT')

        m_r = self.assert_receive_message('RANGEFINDER')
        if abs(m_r.distance - m_p.relative_alt/1000) > 1:
            raise NotAchievedException(
                "RANGEFINDER/GLOBAL_POSITION_INT mismatch %0.2f vs %0.2f" %
                (m_r.distance, m_p.relative_alt/1000))

        m_ds = self.assert_receive_message('DISTANCE_SENSOR')
        if abs(m_ds.current_distance * 0.01 - m_p.relative_alt/1000) > 1:
            raise NotAchievedException(
                "DISTANCE_SENSOR/GLOBAL_POSITION_INT mismatch %0.2f vs %0.2f" %
                (m_ds.current_distance*0.01, m_p.relative_alt/1000))

        self.land_and_disarm()

        if not self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("Did not see expected RFND message")

    def SplineTerrain(self):
        '''Test Splines and Terrain'''
        self.set_parameter("TERRAIN_ENABLE", 0)
        self.fly_mission("wp.txt")

    def WPNAV_SPEED(self):
        '''ensure resetting WPNAV_SPEED during a mission works'''

        loc = self.poll_home_position()
        alt = 20
        loc.alt = alt
        items = []

        # 100 waypoints in a line, 10m apart in a northerly direction
        #        for i in range(1, 100):
        #            items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, i*10, 0, alt))

        # 1 waypoint a long way away
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, alt),)

        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

        self.upload_simple_relhome_mission(items)

        start_speed_ms = self.get_parameter('WPNAV_SPEED') / 100.0

        self.takeoff(20)
        self.change_mode('AUTO')
        self.wait_groundspeed(start_speed_ms-1, start_speed_ms+1, minimum_duration=10)

        for speed_ms in 7, 8, 7, 8, 9, 10, 11, 7:
            self.set_parameter('WPNAV_SPEED', speed_ms*100)
            self.wait_groundspeed(speed_ms-1, speed_ms+1, minimum_duration=10)
        self.do_RTL()

    def WPNAV_SPEED_UP(self):
        '''Change speed (up) during mission'''

        items = []

        # 1 waypoint a long way up
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20000),)

        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

        self.upload_simple_relhome_mission(items)

        start_speed_ms = self.get_parameter('WPNAV_SPEED_UP') / 100.0

        minimum_duration = 5

        self.takeoff(20)
        self.change_mode('AUTO')
        self.wait_climbrate(start_speed_ms-1, start_speed_ms+1, minimum_duration=minimum_duration)

        for speed_ms in 7, 8, 7, 8, 6, 2:
            self.set_parameter('WPNAV_SPEED_UP', speed_ms*100)
            self.wait_climbrate(speed_ms-1, speed_ms+1, minimum_duration=minimum_duration)
        self.do_RTL(timeout=240)

    def WPNAV_SPEED_DN(self):
        '''Change speed (down) during mission'''

        items = []

        # 1 waypoint a long way back down
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10),)

        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

        self.upload_simple_relhome_mission(items)

        minimum_duration = 5

        self.takeoff(500, timeout=70)
        self.change_mode('AUTO')

        start_speed_ms = self.get_parameter('WPNAV_SPEED_DN') / 100.0
        self.wait_climbrate(-start_speed_ms-1, -start_speed_ms+1, minimum_duration=minimum_duration)

        for speed_ms in 7, 8, 7, 8, 6, 2:
            self.set_parameter('WPNAV_SPEED_DN', speed_ms*100)
            self.wait_climbrate(-speed_ms-1, -speed_ms+1, minimum_duration=minimum_duration)
        self.do_RTL()

    def load_and_start_mission(self, filename, strict=True):
        num_wp = self.load_mission(filename, strict=strict)
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        return num_wp

    def fly_mission(self, filename, strict=True):
        num_wp = self.load_and_start_mission(filename, strict)
        self.wait_waypoint(num_wp-1, num_wp-1)
        self.wait_disarmed()

    def fly_generic_mission(self, filename, strict=True):
        num_wp = self.load_generic_mission(filename, strict=strict)
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(num_wp-1, num_wp-1)
        self.wait_disarmed()

    def SurfaceTracking(self):
        '''Test Surface Tracking'''
        ex = None
        self.context_push()

        self.install_terrain_handlers_context()

        try:
            self.set_analog_rangefinder_parameters()
            self.set_parameter("RC9_OPTION", 10) # rangefinder
            self.set_rc(9, 2000)

            self.reboot_sitl() # needed for both rangefinder and initial position
            self.assert_vehicle_location_is_at_startup_location()

            self.takeoff(10, mode="LOITER")
            lower_surface_pos = mavutil.location(-35.362421, 149.164534, 584, 270)
            here = self.mav.location()
            bearing = self.get_bearing(here, lower_surface_pos)

            self.change_mode("GUIDED")
            self.guided_achieve_heading(bearing)
            self.change_mode("LOITER")
            self.delay_sim_time(2)
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            orig_absolute_alt_mm = m.alt

            self.progress("Original alt: absolute=%f" % orig_absolute_alt_mm)

            self.progress("Flying somewhere which surface is known lower compared to takeoff point")
            self.set_rc(2, 1450)
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 200:
                    raise NotAchievedException("Did not reach lower point")
                m = self.assert_receive_message('GLOBAL_POSITION_INT')
                x = mavutil.location(m.lat/1e7, m.lon/1e7, m.alt/1e3, 0)
                dist = self.get_distance(x, lower_surface_pos)
                delta = (orig_absolute_alt_mm - m.alt)/1000.0

                self.progress("Distance: %fm abs-alt-delta: %fm" %
                              (dist, delta))
                if dist < 15:
                    if delta < 0.8:
                        raise NotAchievedException("Did not dip in altitude as expected")
                    break

            self.set_rc(2, 1500)
            self.do_RTL()

        except Exception as e:
            self.print_exception_caught(e)
            self.disarm_vehicle(force=True)
            ex = e

        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_rangefinder_switchover(self):
        """test that the EKF correctly handles the switchover between baro and rangefinder"""
        self.set_analog_rangefinder_parameters()

        self.set_parameters({
            "RNGFND1_MAX": 15.00
        })

        # configure EKF to use rangefinder for altitude at low altitudes
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        if ahrs_ekf_type == 2:
            self.set_parameter("EK2_RNG_USE_HGT", 70)
        if ahrs_ekf_type == 3:
            self.set_parameter("EK3_RNG_USE_HGT", 70)

        self.reboot_sitl() # needed for both rangefinder and initial position
        self.assert_vehicle_location_is_at_startup_location()

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1800)
        self.set_rc(2, 1200)
        # wait till we get to 50m
        self.wait_altitude(50, 52, True, 60)

        self.change_mode("RTL")
        # wait till we get to 25m
        self.wait_altitude(25, 27, True, 120)

        # level up
        self.set_rc(2, 1500)
        self.wait_altitude(14, 15, relative=True)

        self.wait_rtl_complete()

    def _Parachute(self, command):
        '''Test Parachute Functionality using specific mavlink command'''
        self.set_rc(9, 1000)
        self.set_parameters({
            "CHUTE_ENABLED": 1,
            "CHUTE_TYPE": 10,
            "SERVO9_FUNCTION": 27,
            "SIM_PARA_ENABLE": 1,
            "SIM_PARA_PIN": 9,
        })

        self.progress("Test triggering parachute in mission")
        self.load_mission("copter_parachute_mission.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1600)
        self.wait_statustext('BANG', timeout=60)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.progress("Test triggering with mavlink message")
        self.takeoff(20)
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=2, # release
        )
        self.wait_statustext('BANG', timeout=60)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.progress("Testing three-position switch")
        self.set_parameter("RC9_OPTION", 23) # parachute 3pos

        self.progress("Test manual triggering")
        self.takeoff(20)
        self.set_rc(9, 2000)
        self.wait_statustext('BANG', timeout=60)
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.progress("Test mavlink triggering")
        self.takeoff(20)
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_DISABLE,
        )
        ok = False
        try:
            self.wait_statustext('BANG', timeout=2)
        except AutoTestTimeoutException:
            ok = True
        if not ok:
            raise NotAchievedException("Disabled parachute fired")
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_ENABLE,
        )
        ok = False
        try:
            self.wait_statustext('BANG', timeout=2)
        except AutoTestTimeoutException:
            ok = True
        if not ok:
            raise NotAchievedException("Enabled parachute fired")

        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        # parachute should not fire if you go from disabled to release:
        self.takeoff(20)
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_RELEASE,
        )
        ok = False
        try:
            self.wait_statustext('BANG', timeout=2)
        except AutoTestTimeoutException:
            ok = True
        if not ok:
            raise NotAchievedException("Parachute fired when going straight from disabled to release")

        # now enable then release parachute:
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_ENABLE,
        )
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_RELEASE,
        )
        self.wait_statustext('BANG! Parachute deployed', timeout=2)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.context_push()
        self.progress("Crashing with 3pos switch in enable position")
        self.takeoff(40)
        self.set_rc(9, 1500)
        self.set_parameters({
            "SIM_ENGINE_FAIL": 1 << 1, # motor 2
        })
        self.wait_statustext('BANG! Parachute deployed', timeout=60)
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.context_pop()

        self.progress("Crashing with 3pos switch in disable position")
        loiter_alt = 10
        self.takeoff(loiter_alt, mode='LOITER')
        self.set_rc(9, 1100)
        self.set_parameters({
            "SIM_ENGINE_FAIL": 1 << 1, # motor 2
        })
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + 5:
            m = self.assert_receive_message('STATUSTEXT')
            if m is None:
                continue
            if "BANG" in m.text:
                self.set_rc(9, 1000)
                self.reboot_sitl()
                raise NotAchievedException("Parachute deployed when disabled")
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def Parachute(self):
        '''Test Parachute Functionality'''
        self._Parachute(self.run_cmd)
        self._Parachute(self.run_cmd_int)

    def PrecisionLanding(self):
        """Use PrecLand backends precision messages to land aircraft."""

        self.context_push()

        for backend in [4, 2]:  # SITL, SITL-IRLOCK
            ex = None
            try:
                self.set_parameters({
                    "PLND_ENABLED": 1,
                    "PLND_TYPE": backend,
                })

                self.set_analog_rangefinder_parameters()
                self.set_parameter("SIM_SONAR_SCALE", 12)

                start = self.mav.location()
                target = start
                (target.lat, target.lng) = mavextra.gps_offset(start.lat, start.lng, 4, -4)
                self.progress("Setting target to %f %f" % (target.lat, target.lng))

                self.set_parameters({
                    "SIM_PLD_ENABLE": 1,
                    "SIM_PLD_LAT": target.lat,
                    "SIM_PLD_LON": target.lng,
                    "SIM_PLD_HEIGHT": 0,
                    "SIM_PLD_ALT_LMT": 15,
                    "SIM_PLD_DIST_LMT": 10,
                })

                self.reboot_sitl()

                self.progress("Waiting for location")
                self.zero_throttle()
                self.takeoff(10, 1800, mode="LOITER")
                self.change_mode("LAND")
                self.zero_throttle()
                self.wait_landed_and_disarmed()
                self.assert_receive_message('GLOBAL_POSITION_INT')
                new_pos = self.mav.location()
                delta = self.get_distance(target, new_pos)
                self.progress("Landed %f metres from target position" % delta)
                max_delta = 1.5
                if delta > max_delta:
                    raise NotAchievedException("Did not land close enough to target position (%fm > %fm" % (delta, max_delta))

                if not self.current_onboard_log_contains_message("PL"):
                    raise NotAchievedException("Did not see expected PL message")

            except Exception as e:
                self.print_exception_caught(e)
                ex = e
            self.reboot_sitl()
        self.zero_throttle()
        self.context_pop()
        self.reboot_sitl()
        self.progress("All done")

        if ex is not None:
            raise ex

    def Landing(self):
        """Test landing the aircraft."""

        def check_landing_speeds(land_speed_high, land_speed_low, land_alt_low, land_speed_high_accuracy=0.1):
            self.progress("Checking landing speeds (speed_high=%f speed_low=%f alt_low=%f" %
                          (land_speed_high, land_speed_low, land_alt_low))
            land_high_maintain = 5
            land_low_maintain = land_alt_low / land_speed_low / 2

            takeoff_alt = (land_high_maintain * land_speed_high + land_alt_low) + 20
            # this is pretty rough, but takes *so much longer* in LOITER
            self.takeoff(takeoff_alt, mode='STABILIZE', timeout=200, takeoff_throttle=2000)
            # check default landing speeds:
            self.change_mode('LAND')
            # ensure higher-alt descent rate:
            self.wait_descent_rate(land_speed_high,
                                   minimum_duration=land_high_maintain,
                                   accuracy=land_speed_high_accuracy)
            self.wait_descent_rate(land_speed_low)
            # ensure we transition to low descent rate at correct height:
            self.assert_altitude(land_alt_low, relative=True)
            # now make sure we maintain that descent rate:
            self.wait_descent_rate(land_speed_low, minimum_duration=land_low_maintain)
            self.wait_disarmed()

        # test the defaults.  By default LAND_SPEED_HIGH is 0 so
        # WPNAV_SPEED_DN is used
        check_landing_speeds(
            self.get_parameter("WPNAV_SPEED_DN") / 100,  # cm/s -> m/s
            self.get_parameter("LAND_SPEED") / 100,  # cm/s -> m/s
            self.get_parameter("LAND_ALT_LOW") / 100 # cm -> m
        )

        def test_landing_speeds(land_speed_high, land_speed_low, land_alt_low, **kwargs):
            self.set_parameters({
                "LAND_SPEED_HIGH": land_speed_high * 100,  # m/s -> cm/s
                "LAND_SPEED": land_speed_low * 100,  # m/s -> cm/s
                "LAND_ALT_LOW": land_alt_low * 100,  # m -> cm
            })
            check_landing_speeds(land_speed_high, land_speed_low, land_alt_low, **kwargs)

        test_landing_speeds(
            5,  # descent speed high
            1,  # descent speed low
            30,  # transition altitude
            land_speed_high_accuracy=0.5
        )

    def get_system_clock_utc(self, time_seconds):
        # this is a copy of ArduPilot's AP_RTC function!
        # separate time into ms, sec, min, hour and days but all expressed
        # in milliseconds
        time_ms = time_seconds * 1000
        ms = time_ms % 1000
        sec_ms = (time_ms % (60 * 1000)) - ms
        min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms
        hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms

        # convert times as milliseconds into appropriate units
        secs = sec_ms / 1000
        mins = min_ms / (60 * 1000)
        hours = hour_ms / (60 * 60 * 1000)
        return (hours, mins, secs, 0)

    def calc_delay(self, seconds, delay_for_seconds):
        # delay-for-seconds has to be long enough that we're at the
        # waypoint before that time.  Otherwise we'll try to wait a
        # day....
        if delay_for_seconds >= 3600:
            raise ValueError("Won't handle large delays")
        (hours,
         mins,
         secs,
         ms) = self.get_system_clock_utc(seconds)
        self.progress("Now is %uh %um %us" % (hours, mins, secs))
        secs += delay_for_seconds # add seventeen seconds
        mins += int(secs/60)
        secs %= 60

        hours += int(mins / 60)
        mins %= 60

        if hours > 24:
            raise ValueError("Way too big a delay")
        self.progress("Delay until %uh %um %us" %
                      (hours, mins, secs))
        return (hours, mins, secs, 0)

    def reset_delay_item(self, seq, seconds_in_future):
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command = mavutil.mavlink.MAV_CMD_NAV_DELAY
        # retrieve mission item and check it:
        tried_set = False
        hours = None
        mins = None
        secs = None
        while True:
            self.progress("Requesting item")
            self.mav.mav.mission_request_send(1,
                                              1,
                                              seq)
            st = self.mav.recv_match(type='MISSION_ITEM',
                                     blocking=True,
                                     timeout=1)
            if st is None:
                continue

            print("Item: %s" % str(st))
            have_match = (tried_set and
                          st.seq == seq and
                          st.command == command and
                          st.param2 == hours and
                          st.param3 == mins and
                          st.param4 == secs)
            if have_match:
                return

            self.progress("Mission mismatch")

            m = None
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > 3:
                    raise NotAchievedException(
                        "Did not receive MISSION_REQUEST")
                self.mav.mav.mission_write_partial_list_send(1,
                                                             1,
                                                             seq,
                                                             seq)
                m = self.mav.recv_match(type='MISSION_REQUEST',
                                        blocking=True,
                                        timeout=1)
                if m is None:
                    continue
                if m.seq != st.seq:
                    continue
                break

            self.progress("Sending absolute-time mission item")

            # we have to change out the delay time...
            now = self.mav.messages["SYSTEM_TIME"]
            if now is None:
                raise PreconditionFailedException("Never got SYSTEM_TIME")
            if now.time_unix_usec == 0:
                raise PreconditionFailedException("system time is zero")
            (hours, mins, secs, ms) = self.calc_delay(now.time_unix_usec/1000000, seconds_in_future)

            self.mav.mav.mission_item_send(
                1, # target system
                1, # target component
                seq, # seq
                frame, # frame
                command, # command
                0, # current
                1, # autocontinue
                0, # p1 (relative seconds)
                hours, # p2
                mins, # p3
                secs, # p4
                0, # p5
                0, # p6
                0) # p7
            tried_set = True
            ack = self.mav.recv_match(type='MISSION_ACK',
                                      blocking=True,
                                      timeout=1)
            self.progress("Received ack: %s" % str(ack))

    def NavDelayAbsTime(self):
        """fly a simple mission that has a delay in it"""
        self.fly_nav_delay_abstime_x(87)

    def fly_nav_delay_abstime_x(self, delay_for, expected_delay=None):
        """fly a simple mission that has a delay in it, expect a delay"""

        if expected_delay is None:
            expected_delay = delay_for

        self.load_mission("copter_nav_delay.txt")

        self.change_mode("LOITER")

        self.wait_ready_to_arm()

        delay_item_seq = 3
        self.reset_delay_item(delay_item_seq, delay_for)
        delay_for_seconds = delay_for
        reset_at_m = self.assert_receive_message('SYSTEM_TIME')
        reset_at = reset_at_m.time_unix_usec/1000000

        self.arm_vehicle()
        self.change_mode("AUTO")
        self.set_rc(3, 1600)
        count_stop = -1
        tstart = self.get_sim_time()
        while self.armed(): # we RTL at end of mission
            now = self.get_sim_time_cached()
            if now - tstart > 240:
                raise AutoTestTimeoutException("Did not disarm as expected")
            m = self.assert_receive_message('MISSION_CURRENT')
            at_delay_item = ""
            if m.seq == delay_item_seq:
                at_delay_item = "(delay item)"
            self.progress("MISSION_CURRENT.seq=%u %s" % (m.seq, at_delay_item))
            if m.seq > delay_item_seq:
                if count_stop == -1:
                    count_stop_m = self.assert_receive_message('SYSTEM_TIME')
                    count_stop = count_stop_m.time_unix_usec/1000000
        calculated_delay = count_stop - reset_at
        error = abs(calculated_delay - expected_delay)
        self.progress("Stopped for %u seconds (want >=%u seconds)" %
                      (calculated_delay, delay_for_seconds))
        if error > 2:
            raise NotAchievedException("delay outside expectations")

    def NavDelayTakeoffAbsTime(self):
        """make sure taking off at a specific time works"""
        self.load_mission("copter_nav_delay_takeoff.txt")

        self.change_mode("LOITER")
        self.wait_ready_to_arm()

        delay_item_seq = 2
        delay_for_seconds = 77
        self.reset_delay_item(delay_item_seq, delay_for_seconds)
        reset_at = self.get_sim_time_cached()

        self.arm_vehicle()
        self.change_mode("AUTO")

        self.set_rc(3, 1600)

        # should not take off for about least 77 seconds
        tstart = self.get_sim_time()
        took_off = False
        while self.armed():
            now = self.get_sim_time_cached()
            if now - tstart > 200:
                # timeout
                break
            m = self.assert_receive_message('MISSION_CURRENT')
            now = self.get_sim_time_cached()
            self.progress("%s" % str(m))
            if m.seq > delay_item_seq:
                if not took_off:
                    took_off = True
                    delta_time = now - reset_at
                    if abs(delta_time - delay_for_seconds) > 2:
                        raise NotAchievedException((
                            "Did not take off on time "
                            "measured=%f want=%f" %
                            (delta_time, delay_for_seconds)))

        if not took_off:
            raise NotAchievedException("Did not take off")

    def ModeZigZag(self):
        '''test zigzag mode'''
        # set channel 8 for zigzag savewp and recentre it
        self.set_parameter("RC8_OPTION", 61)
        self.set_parameter("LOIT_OPTIONS", 0)

        self.takeoff(alt_min=5, mode='LOITER')

        ZIGZAG = 24
        j = 0
        slowdown_speed = 0.3 # because Copter takes a long time to actually stop
        self.start_subtest("Conduct ZigZag test for all 4 directions")
        while j < 4:
            self.progress("## Align heading with the run-way (j=%d)##" % j)
            self.set_rc(8, 1500)
            self.set_rc(4, 1420)
            self.wait_heading(352-j*90)
            self.set_rc(4, 1500)
            self.change_mode(ZIGZAG)
            self.progress("## Record Point A ##")
            self.set_rc(8, 1100)  # record point A
            self.set_rc(1, 1700)  # fly side-way for 20m
            self.wait_distance(20)
            self.set_rc(1, 1500)
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.progress("## Record Point A ##")
            self.set_rc(8, 1500)    # pilot always have to cross mid position when changing for low to high position
            self.set_rc(8, 1900)    # record point B

            i = 1
            while i < 2:
                self.start_subtest("Run zigzag A->B and B->A (i=%d)" % i)
                self.progress("## fly forward for 10 meter ##")
                self.set_rc(2, 1300)
                self.wait_distance(10)
                self.set_rc(2, 1500)    # re-centre pitch rc control
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
                self.set_rc(8, 1500)    # switch to mid position
                self.progress("## auto execute vector BA ##")
                self.set_rc(8, 1100)
                self.wait_distance(17)  # wait for it to finish
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down

                self.progress("## fly forward for 10 meter ##")
                self.set_rc(2, 1300)    # fly forward for 10 meter
                self.wait_distance(10)
                self.set_rc(2, 1500)    # re-centre pitch rc control
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
                self.set_rc(8, 1500)    # switch to mid position
                self.progress("## auto execute vector AB ##")
                self.set_rc(8, 1900)
                self.wait_distance(17)  # wait for it to finish
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
                i = i + 1
            # test the case when pilot switch to manual control during the auto flight
            self.start_subtest("test the case when pilot switch to manual control during the auto flight")
            self.progress("## fly forward for 10 meter ##")
            self.set_rc(2, 1300)    # fly forward for 10 meter
            self.wait_distance(10)
            self.set_rc(2, 1500)    # re-centre pitch rc control
            self.wait_groundspeed(0, 0.3)   # wait until the copter slows down
            self.set_rc(8, 1500)    # switch to mid position
            self.progress("## auto execute vector BA ##")
            self.set_rc(8, 1100)    # switch to low position, auto execute vector BA
            self.wait_distance(8)   # purposely switch to manual halfway
            self.set_rc(8, 1500)
            self.wait_groundspeed(0, slowdown_speed)   # copter should slow down here
            self.progress("## Manual control to fly forward ##")
            self.set_rc(2, 1300)    # manual control to fly forward
            self.wait_distance(8)
            self.set_rc(2, 1500)    # re-centre pitch rc control
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.progress("## continue vector BA ##")
            self.set_rc(8, 1100)    # copter should continue mission here
            self.wait_distance(8)   # wait for it to finish rest of BA
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.set_rc(8, 1500)    # switch to mid position
            self.progress("## auto execute vector AB ##")
            self.set_rc(8, 1900)    # switch to execute AB again
            self.wait_distance(17)  # wait for it to finish
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.change_mode('LOITER')
            j = j + 1

        self.do_RTL()

    def SetModesViaModeSwitch(self):
        '''Set modes via modeswitch'''
        fltmode_ch = 5
        self.set_parameter("FLTMODE_CH", fltmode_ch)
        self.set_rc(fltmode_ch, 1000) # PWM for mode1
        testmodes = [("FLTMODE1", 4, "GUIDED", 1165),
                     ("FLTMODE2", 2, "ALT_HOLD", 1295),
                     ("FLTMODE3", 6, "RTL", 1425),
                     ("FLTMODE4", 7, "CIRCLE", 1555),
                     ("FLTMODE5", 1, "ACRO", 1685),
                     ("FLTMODE6", 17, "BRAKE", 1815),
                     ]
        for mode in testmodes:
            (parm, parm_value, name, pwm) = mode
            self.set_parameter(parm, parm_value)

        for mode in reversed(testmodes):
            (parm, parm_value, name, pwm) = mode
            self.set_rc(fltmode_ch, pwm)
            self.wait_mode(name)

        for mode in testmodes:
            (parm, parm_value, name, pwm) = mode
            self.set_rc(fltmode_ch, pwm)
            self.wait_mode(name)

        for mode in reversed(testmodes):
            (parm, parm_value, name, pwm) = mode
            self.set_rc(fltmode_ch, pwm)
            self.wait_mode(name)

    def SetModesViaAuxSwitch(self):
        '''"Set modes via auxswitch"'''
        fltmode_ch = int(self.get_parameter("FLTMODE_CH"))
        self.set_rc(fltmode_ch, 1000)
        self.wait_mode("CIRCLE")
        self.set_rc(9, 1000)
        self.set_rc(10, 1000)
        self.set_parameters({
            "RC9_OPTION": 18, # land
            "RC10_OPTION": 55, # guided
        })
        self.set_rc(9, 1900)
        self.wait_mode("LAND")
        self.set_rc(10, 1900)
        self.wait_mode("GUIDED")
        self.set_rc(10, 1000) # this re-polls the mode switch
        self.wait_mode("CIRCLE")

    def fly_guided_stop(self,
                        timeout=20,
                        groundspeed_tolerance=0.05,
                        climb_tolerance=0.01):
        """stop the vehicle moving in guided mode"""
        self.progress("Stopping vehicle")
        tstart = self.get_sim_time()
        # send a position-control command
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-x-y-z
            0, # x
            0, # y
            0, # z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Vehicle did not stop")
            m = self.assert_receive_message('VFR_HUD')
            print("%s" % str(m))
            if (m.groundspeed < groundspeed_tolerance and
                    m.climb < climb_tolerance):
                break

    def send_set_position_target_global_int(self, lat, lon, alt):
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY, # mask specifying use-only-lat-lon-alt
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

    def fly_guided_move_global_relative_alt(self, lat, lon, alt):
        startpos = self.assert_receive_message('GLOBAL_POSITION_INT')

        self.send_set_position_target_global_int(lat, lon, alt)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not move far enough")
            # send a position-control command
            pos = self.assert_receive_message('GLOBAL_POSITION_INT')
            delta = self.get_distance_int(startpos, pos)
            self.progress("delta=%f (want >10)" % delta)
            if delta > 10:
                break

    def fly_guided_move_local(self, x, y, z_up, timeout=100):
        """move the vehicle using MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED"""
        startpos = self.assert_receive_message('LOCAL_POSITION_NED')
        self.progress("startpos=%s" % str(startpos))

        tstart = self.get_sim_time()
        # send a position-control command
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-x-y-z
            x, # x
            y, # y
            -z_up, # z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not reach destination")
            dist = self.distance_to_local_position((x, y, -z_up))
            if dist < 1:
                self.progress(f"Reach distance ({dist})")
                self.progress("endpos=%s" % str(startpos))
                break

    def test_guided_local_position_target(self, x, y, z_up):
        """ Check target position being received by vehicle """
        # set POSITION_TARGET_LOCAL_NED message rate using SET_MESSAGE_INTERVAL
        self.progress("Setting local target in NED: (%f, %f, %f)" % (x, y, -z_up))
        self.progress("Setting rate to 1 Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 1)

        # mask specifying use only xyz
        target_typemask = MAV_POS_TARGET_TYPE_MASK.POS_ONLY

        # set position target
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
            x, # x
            y, # y
            -z_up, # z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        m = self.assert_receive_message('POSITION_TARGET_LOCAL_NED', timeout=2)
        self.progress("Received local target: %s" % str(m))

        if not (m.type_mask == (target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE) or m.type_mask == target_typemask):
            raise NotAchievedException("Did not receive proper mask: expected=%u or %u, got=%u" %
                                       ((target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE), target_typemask, m.type_mask))

        if x - m.x > 0.1:
            raise NotAchievedException("Did not receive proper target position x: wanted=%f got=%f" % (x, m.x))

        if y - m.y > 0.1:
            raise NotAchievedException("Did not receive proper target position y: wanted=%f got=%f" % (y, m.y))

        if z_up - (-m.z) > 0.1:
            raise NotAchievedException("Did not receive proper target position z: wanted=%f got=%f" % (z_up, -m.z))

    def test_guided_local_velocity_target(self, vx, vy, vz_up, timeout=3):
        " Check local target velocity being received by vehicle "
        self.progress("Setting local NED velocity target: (%f, %f, %f)" % (vx, vy, -vz_up))
        self.progress("Setting POSITION_TARGET_LOCAL_NED message rate to 10Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10)

        # mask specifying use only vx,vy,vz & accel. Even though we don't test acceltargets below currently
        #  a velocity only mask returns a velocity & accel mask
        target_typemask = (MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)

        # Drain old messages and ignore the ramp-up to the required target velocity
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            # send velocity-control command
            self.mav.mav.set_position_target_local_ned_send(
                0, # timestamp
                1, # target system_id
                1, # target component id
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
                0, # x
                0, # y
                0, # z
                vx, # vx
                vy, # vy
                -vz_up, # vz
                0, # afx
                0, # afy
                0, # afz
                0, # yaw
                0, # yawrate
            )
            m = self.assert_receive_message('POSITION_TARGET_LOCAL_NED')

            self.progress("Received local target: %s" % str(m))

        # Check the last received message
        if not (m.type_mask == (target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE) or m.type_mask == target_typemask):
            raise NotAchievedException("Did not receive proper mask: expected=%u or %u, got=%u" %
                                       ((target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE), target_typemask, m.type_mask))

        if vx - m.vx > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vx: wanted=%f got=%f" % (vx, m.vx))

        if vy - m.vy > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vy: wanted=%f got=%f" % (vy, m.vy))

        if vz_up - (-m.vz) > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vz: wanted=%f got=%f" % (vz_up, -m.vz))

        self.progress("Received proper target velocity commands")

    def wait_for_local_velocity(self, vx, vy, vz_up, timeout=10):
        """ Wait for local target velocity"""

        # debug messages
        self.progress("Waiting for local NED velocity target: (%f, %f, %f)" % (vx, vy, -vz_up))
        self.progress("Setting LOCAL_POSITION_NED message rate to 10Hz")

        # set position local ned message stream rate
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)

        # wait for position local ned message
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:

            # get position target local ned message
            m = self.mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)

            # could not be able to get a valid target local ned message within given time
            if m is None:

                # raise an error that did not receive a valid target local ned message within given time
                raise NotAchievedException("Did not receive any position local ned message for 1 second!")

            # got a valid target local ned message within given time
            else:

                # debug message
                self.progress("Received local position ned message: %s" % str(m))

                # check if velocity values are in range
                if vx - m.vx <= 0.1 and vy - m.vy <= 0.1 and vz_up - (-m.vz) <= 0.1:

                    # get out of function
                    self.progress("Vehicle successfully reached to target velocity!")
                    return

        # raise an exception
        error_message = "Did not receive target velocities vx, vy, vz_up, wanted=(%f, %f, %f) got=(%f, %f, %f)"
        error_message = error_message % (vx, vy, vz_up, m.vx, m.vy, -m.vz)
        raise NotAchievedException(error_message)

    def test_position_target_message_mode(self):
        " Ensure that POSITION_TARGET_LOCAL_NED messages are sent in Guided Mode only "
        self.hover()
        self.change_mode('LOITER')
        self.progress("Setting POSITION_TARGET_LOCAL_NED message rate to 10Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10)

        self.assert_not_receiving_message('POSITION_TARGET_LOCAL_NED')

        self.progress("Did not receive any POSITION_TARGET_LOCAL_NED message in LOITER mode. Success")

    def earth_to_body(self, vector):
        r = mavextra.rotation(self.mav.messages["ATTITUDE"]).invert()
        #        print("r=%s" % str(r))
        return r * vector

    def precision_loiter_to_pos(self, x, y, z, send_bf=True, timeout=40):
        '''send landing_target messages at vehicle until it arrives at
        location to x, y, z from origin (in metres), z is *up*
        if send_bf is True then targets sent in body-frame, if False then sent in local FRD frame'''
        dest_ned = rotmat.Vector3(x, y, -z)
        tstart = self.get_sim_time()
        success_start = -1
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not loiter to position!")
            m_pos = self.assert_receive_message('LOCAL_POSITION_NED')
            pos_ned = rotmat.Vector3(m_pos.x, m_pos.y, m_pos.z)
            #            print("dest_ned=%s" % str(dest_ned))
            #            print("pos_ned=%s" % str(pos_ned))
            delta_ef = dest_ned - pos_ned
            #            print("delta_ef=%s" % str(delta_ef))

            # determine if we've successfully navigated to close to
            # where we should be:
            dist = math.sqrt(delta_ef.x * delta_ef.x + delta_ef.y * delta_ef.y)
            dist_max = 1
            self.progress("dist=%f want <%f" % (dist, dist_max))
            if dist < dist_max:
                # success!  We've gotten within our target distance
                if success_start == -1:
                    success_start = now
                elif now - success_start > 10:
                    self.progress("Yay!")
                    break
            else:
                success_start = -1

            mav_frame = None
            if send_bf:
                # rotate delta_ef from NED to body frame FRD
                delta_bf = self.earth_to_body(delta_ef)
                print("delta_bf=%s" % str(delta_bf))
                angle_x = math.atan2(delta_bf.y, delta_bf.z)
                angle_y = -math.atan2(delta_bf.x, delta_bf.z)
                distance = math.sqrt(delta_bf.x * delta_bf.x +
                                     delta_bf.y * delta_bf.y +
                                     delta_bf.z * delta_bf.z)
                mav_frame = mavutil.mavlink.MAV_FRAME_BODY_FRD
            else:
                # rotate delta_ef from NED to Local FRD
                rotmat_yaw = Matrix3()
                rotmat_yaw.rotate_yaw(self.mav.messages["ATTITUDE"].yaw)
                delta_local_frd = rotmat_yaw * delta_ef
                angle_x = math.atan2(delta_local_frd.y, delta_local_frd.z)
                angle_y = -math.atan2(delta_local_frd.x, delta_local_frd.z)
                distance = math.sqrt(delta_local_frd.x * delta_local_frd.x +
                                     delta_local_frd.y * delta_local_frd.y +
                                     delta_local_frd.z * delta_local_frd.z)
                mav_frame = mavutil.mavlink.MAV_FRAME_LOCAL_FRD

            # att = self.mav.messages["ATTITUDE"]
            # print("r=%f p=%f y=%f" % (math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw)))
            # print("angle_x=%s angle_y=%s" % (str(math.degrees(angle_x)), str(math.degrees(angle_y))))
            # print("distance=%s" % str(distance))

            self.mav.mav.landing_target_send(
                0, # time_usec
                1, # target_num
                mav_frame, # frame
                angle_x, # angle x (radians)
                angle_y, # angle y (radians)
                distance, # distance to target
                0.01, # size of target in radians, X-axis
                0.01 # size of target in radians, Y-axis
            )

    def set_servo_gripper_parameters(self):
        self.set_parameters({
            "GRIP_ENABLE": 1,
            "GRIP_TYPE": 1,
            "SIM_GRPS_ENABLE": 1,
            "SIM_GRPS_PIN": 8,
            "SERVO8_FUNCTION": 28,
        })

    def PayloadPlaceMission(self):
        """Test payload placing in auto."""
        self.set_analog_rangefinder_parameters()
        self.set_servo_gripper_parameters()
        self.reboot_sitl()

        num_wp = self.load_and_start_mission("copter_payload_place.txt")

        self.wait_text("Gripper load releas(ed|ing)", timeout=90, regex=True)
        dist_limit = 1
        # this is a copy of the point in the mission file:
        target_loc = mavutil.location(-35.363106,
                                      149.165436,
                                      0,
                                      0)
        dist = self.get_distance(target_loc, self.mav.location())
        self.progress("dist=%f" % (dist,))
        if dist > dist_limit:
            raise NotAchievedException("Did not honour target lat/lng (dist=%f want <%f" %
                                       (dist, dist_limit))

        self.wait_waypoint(num_wp-1, num_wp-1)
        self.do_RTL()

    def PayloadPlaceMissionOpenGripper(self):
        '''test running the mission when the gripper is open'''
        self.set_analog_rangefinder_parameters()
        self.set_servo_gripper_parameters()
        self.reboot_sitl()

        self.load_mission("copter_payload_place.txt")

        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.progress("Opening the gripper before time")
        self.run_auxfunc(19, 0)

        self.wait_text("Abort: Gripper Open", timeout=90)

        self.wait_disarmed()

    def Weathervane(self):
        '''Test copter weathervaning'''
        # We test nose into wind code paths and yaw direction here and test side into wind
        # yaw direction in QuadPlane tests to reduce repetition.
        self.set_parameters({
            "SIM_WIND_SPD": 10,
            "SIM_WIND_DIR": 100,
            "GUID_OPTIONS": 129, # allow weathervaning and arming from tx in guided
            "AUTO_OPTIONS": 131, # allow arming in auto, take off without raising the stick, and weathervaning
            "WVANE_ENABLE": 1,
            "WVANE_GAIN": 3,
            "WVANE_VELZ_MAX": 1,
            "WVANE_SPD_MAX": 2
        })

        self.progress("Test weathervaning in auto")
        self.load_mission("weathervane_mission.txt", strict=False)

        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_statustext("Weathervane Active", timeout=60)
        self.do_RTL()
        self.wait_disarmed()
        self.change_mode("GUIDED")

        # After take off command in guided we enter the velaccl sub mode
        self.progress("Test weathervaning in guided vel-accel")
        self.set_rc(3, 1000)
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.user_takeoff(alt_min=15)
        # Wait for heading to match wind direction.
        self.wait_heading(100, accuracy=8, timeout=100)

        self.progress("Test weathervaning in guided pos only")
        # Travel directly north to align heading north and build some airspeed.
        self.fly_guided_move_local(x=40, y=0, z_up=15)
        # Wait for heading to match wind direction.
        self.wait_heading(100, accuracy=8, timeout=100)
        self.do_RTL()

    def _DO_WINCH(self, command):
        self.context_push()
        self.load_default_params_file("copter-winch.parm")
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.start_subtest("starts relaxed")
        self.wait_servo_channel_value(9, 0)

        self.start_subtest("rate control")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RATE_CONTROL,  # command
            p3=0,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 1900)

        self.start_subtest("relax")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RELAXED,  # command
            p3=0,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 0)

        self.start_subtest("hold but zero output")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RATE_CONTROL,  # command
            p3=0,  # length to release
            p4=0,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 1500)

        self.start_subtest("relax")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RELAXED,  # command
            p3=0,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 0)

        self.start_subtest("position")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RELATIVE_LENGTH_CONTROL,  # command
            p3=2,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 1900)
        self.wait_servo_channel_value(9, 1500, timeout=60)

        self.context_pop()
        self.reboot_sitl()

    def DO_WINCH(self):
        '''test mavlink DO_WINCH command'''
        self._DO_WINCH(self.run_cmd_int)
        self._DO_WINCH(self.run_cmd)

    def GuidedSubModeChange(self):
        """"Ensure we can move around in guided after a takeoff command."""

        '''start by disabling GCS failsafe, otherwise we immediately disarm
        due to (apparently) not receiving traffic from the GCS for
        too long.  This is probably a function of --speedup'''
        self.set_parameters({
            "FS_GCS_ENABLE": 0,
            "DISARM_DELAY": 0, # until traffic problems are fixed
        })
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.user_takeoff(alt_min=10)

        self.start_subtest("yaw through absolute angles using MAV_CMD_CONDITION_YAW")
        self.guided_achieve_heading(45)
        self.guided_achieve_heading(135)

        self.start_subtest("move the vehicle using set_position_target_global_int")
        # the following numbers are 5-degree-latitude and 5-degrees
        # longitude - just so that we start to really move a lot.
        self.fly_guided_move_global_relative_alt(5, 5, 10)

        self.start_subtest("move the vehicle using MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED")
        self.fly_guided_stop(groundspeed_tolerance=0.1)
        self.fly_guided_move_local(5, 5, 10)

        self.start_subtest("Checking that WP_YAW_BEHAVIOUR 0 works")
        self.set_parameter('WP_YAW_BEHAVIOR', 0)
        self.delay_sim_time(2)
        orig_heading = self.get_heading()
        self.fly_guided_move_local(5, 0, 10)
        # ensure our heading hasn't changed:
        self.assert_heading(orig_heading)
        self.fly_guided_move_local(0, 5, 10)
        # ensure our heading hasn't changed:
        self.assert_heading(orig_heading)

        self.start_subtest("Check target position received by vehicle using SET_MESSAGE_INTERVAL")
        self.test_guided_local_position_target(5, 5, 10)
        self.test_guided_local_velocity_target(2, 2, 1)
        self.test_position_target_message_mode()

        self.do_RTL()

    def WPYawBehaviour1RTL(self):
        '''ensure behaviour 1 (face home) works in RTL'''
        self.start_subtest("moving off in guided mode and checking return yaw")
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.wait_heading(272, timeout=1)  # verify initial heading"
        self.takeoff(2, mode='GUIDED')
        self.set_parameter('WP_YAW_BEHAVIOR', 1)  # 1 is face next waypoint
        self.fly_guided_move_local(100, 100, z_up=20)
        self.wait_heading(45, timeout=1)
        self.change_mode('RTL')
        self.wait_heading(225, minimum_duration=10, timeout=20)
        self.wait_disarmed()

        self.reboot_sitl()
        self.start_subtest("now the same but in auto mode")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 2),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 100, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameter('AUTO_OPTIONS', 3)
        self.change_mode('AUTO')
        self.set_rc(3, 1000)
        self.wait_ready_to_arm()
        self.wait_heading(273, timeout=1)  # verify initial heading"
        self.arm_vehicle()
        self.wait_heading(45, minimum_duration=10, timeout=20)
        self.wait_current_waypoint(3)
        self.wait_heading(225, minimum_duration=10, timeout=20)
        self.wait_disarmed()

    def TestGripperMission(self):
        '''Test Gripper mission items'''
        num_wp = self.load_mission("copter-gripper-mission.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.assert_vehicle_location_is_at_startup_location()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1500)
        self.wait_statustext("Gripper Grabbed", timeout=60)
        self.wait_statustext("Gripper Released", timeout=60)
        self.wait_waypoint(num_wp-1, num_wp-1)
        self.wait_disarmed()

    def SplineLastWaypoint(self):
        '''Test Spline as last waypoint'''
        self.load_mission("copter-spline-last-waypoint.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1500)
        self.wait_altitude(10, 3000, relative=True)
        self.do_RTL()

    def ManualThrottleModeChange(self):
        '''Check manual throttle mode changes denied on high throttle'''
        self.set_parameter("FS_GCS_ENABLE", 0) # avoid GUIDED instant disarm
        self.change_mode("STABILIZE")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("ACRO")
        self.change_mode("STABILIZE")
        self.change_mode("GUIDED")
        self.set_rc(3, 1700)
        self.watch_altitude_maintained(altitude_min=-1, altitude_max=0.2) # should not take off in guided
        self.run_cmd_do_set_mode(
            "ACRO",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.run_cmd_do_set_mode(
            "STABILIZE",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.run_cmd_do_set_mode(
            "DRIFT",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.progress("Check setting an invalid mode")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=126,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            timeout=1,
        )
        self.set_rc(3, 1000)
        self.run_cmd_do_set_mode("ACRO")
        self.wait_disarmed()

    def constrained_mount_pitch(self, pitch_angle_deg, mount_instance=1):
        PITCH_MIN = self.get_parameter("MNT%u_PITCH_MIN" % mount_instance)
        PITCH_MAX = self.get_parameter("MNT%u_PITCH_MAX" % mount_instance)
        return min(max(pitch_angle_deg, PITCH_MIN), PITCH_MAX)

    def test_mount_pitch(self, despitch, despitch_tolerance, mount_mode, timeout=10, hold=0, constrained=True):
        self.set_mount_mode(mount_mode)
        tstart = self.get_sim_time()
        success_start = 0

        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Mount pitch not achieved")

            # We expect to achieve the desired pitch angle unless constrained by mount limits
            if constrained:
                despitch = self.constrained_mount_pitch(despitch)

            '''retrieve latest angles from GIMBAL_DEVICE_ATTITUDE_STATUS'''
            mount_roll, mount_pitch, mount_yaw, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()

            # self.progress("despitch=%f roll=%f pitch=%f yaw=%f" % (despitch, mount_roll, mount_pitch, mount_yaw))
            if abs(despitch - mount_pitch) > despitch_tolerance:
                self.progress("Mount pitch incorrect: got=%f want=%f (+/- %f)" %
                              (mount_pitch, despitch, despitch_tolerance))
                success_start = 0
                continue
            self.progress("Mount pitch correct: %f degrees == %f" %
                          (mount_pitch, despitch))
            if success_start == 0:
                success_start = now
            if now - success_start >= hold:
                self.progress("Mount pitch achieved")
                return

    def do_pitch(self, pitch):
        '''pitch aircraft in guided/angle mode'''
        self.mav.mav.set_attitude_target_send(
            0, # time_boot_ms
            1, # target sysid
            1, # target compid
            0, # bitmask of things to ignore
            mavextra.euler_to_quat([0, math.radians(pitch), 0]), # att
            0, # roll rate  (rad/s)
            0, # pitch rate (rad/s)
            0, # yaw rate   (rad/s)
            0.5) # thrust, 0 to 1, translated to a climb/descent rate

    def do_yaw_rate(self, yaw_rate):
        '''yaw aircraft in guided/rate mode'''
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=60,  # target angle
            p2=0,  # degrees/second
            p3=1,  # -1 is counter-clockwise, 1 clockwise
            p4=1,  # 1 for relative, 0 for absolute
            quiet=True,
        )

    def get_mount_roll_pitch_yaw_deg(self):
        '''return mount (aka gimbal) roll, pitch and yaw angles in degrees'''
        # wait for gimbal attitude message
        m = self.assert_receive_message('GIMBAL_DEVICE_ATTITUDE_STATUS', timeout=5)

        yaw_is_absolute = m.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK
        # convert quaternion to euler angles and return
        q = quaternion.Quaternion(m.q)
        euler = q.euler
        return math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2]), yaw_is_absolute

    def set_mount_mode(self, mount_mode):
        '''set mount mode'''
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
            p1=mount_mode,
            p2=0, # stabilize roll (unsupported)
            p3=0, # stabilize pitch (unsupported)
        )
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
            p1=mount_mode,
            p2=0, # stabilize roll (unsupported)
            p3=0, # stabilize pitch (unsupported)
        )

    def test_mount_rc_targetting(self, pitch_rc_neutral=1500, do_rate_tests=True):
        '''called in multipleplaces to make sure that mount RC targeting works'''
        if True:
            self.context_push()
            self.set_parameters({
                'RC6_OPTION': 0,
                'RC11_OPTION': 212,    # MOUNT1_ROLL
                'RC12_OPTION': 213,    # MOUNT1_PITCH
                'RC13_OPTION': 214,    # MOUNT1_YAW
                'RC12_MIN': 1100,
                'RC12_MAX': 1900,
                'RC12_TRIM': 1500,
                'MNT1_PITCH_MIN': -45,
                'MNT1_PITCH_MAX': 45,
            })
            self.progress("Testing RC angular control")
            # default RC min=1100 max=1900
            self.set_rc_from_map({
                11: 1500,
                12: 1500,
                13: 1500,
            })
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.progress("Testing RC input down 1/4 of its range in the output, should be down 1/4 range in output")
            rc12_in = 1400
            rc12_min = 1100 # default
            rc12_max = 1900 # default
            mpitch_min = -45.0
            mpitch_max = 45.0
            expected_pitch = (float(rc12_in-rc12_min)/float(rc12_max-rc12_min) * (mpitch_max-mpitch_min)) + mpitch_min
            self.progress("expected mount pitch: %f" % expected_pitch)
            if expected_pitch != -11.25:
                raise NotAchievedException("Calculation wrong - defaults changed?!")
            self.set_rc(12, rc12_in)
            self.test_mount_pitch(-11.25, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.set_rc(12, 1800)
            self.test_mount_pitch(33.75, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.set_rc_from_map({
                11: 1500,
                12: 1500,
                13: 1500,
            })

            try:
                self.context_push()
                self.set_parameters({
                    "RC12_MIN": 1000,
                    "RC12_MAX": 2000,
                    "MNT1_PITCH_MIN": -90,
                    "MNT1_PITCH_MAX": 10,
                })
                self.set_rc(12, 1000)
                self.test_mount_pitch(-90.00, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
                self.set_rc(12, 2000)
                self.test_mount_pitch(10.00, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
                self.set_rc(12, 1500)
                self.test_mount_pitch(-40.00, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            finally:
                self.context_pop()

            self.set_rc(12, 1500)

            if do_rate_tests:
                self.test_mount_rc_targetting_rate_control()

            self.context_pop()

    def test_mount_rc_targetting_rate_control(self, pitch_rc_neutral=1500):
        if True:
            self.progress("Testing RC rate control")
            self.set_parameter('MNT1_RC_RATE', 10)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            # Note that we don't constrain the desired angle in the following so that we don't
            # timeout due to fetching Mount pitch limit params.
            self.set_rc(12, 1300)
            self.test_mount_pitch(-5, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-10, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-15, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-20, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.set_rc(12, 1700)
            self.test_mount_pitch(-15, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-10, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(-5, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)
            self.test_mount_pitch(5, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, constrained=False)

            self.progress("Reverting to angle mode")
            self.set_parameter('MNT1_RC_RATE', 0)
            self.set_rc(12, 1500)
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

    def mount_test_body(self, pitch_rc_neutral=1500, do_rate_tests=True, constrain_sysid_target=True):
        '''Test Camera/Antenna Mount - assumes a camera is set up and ready to go'''
        if True:
            # make sure we're getting gimbal device attitude status
            self.assert_receive_message('GIMBAL_DEVICE_ATTITUDE_STATUS', timeout=5, very_verbose=True)

            # change mount to neutral mode (point forward, not stabilising)
            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

            # test pitch is not neutral to start with
            mount_roll_deg, mount_pitch_deg, mount_yaw_deg, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
            if mount_roll_deg != 0 or mount_pitch_deg != 0 or mount_yaw_deg != 0:
                raise NotAchievedException("Mount not neutral")

            self.takeoff(30, mode='GUIDED')

            # pitch vehicle back and confirm gimbal is still not stabilising
            despitch = 10
            despitch_tolerance = 3

            self.progress("Pitching vehicle")
            self.do_pitch(despitch) # will time out!

            self.wait_pitch(despitch, despitch_tolerance)

            # check gimbal is still not stabilising
            mount_roll_deg, mount_pitch_deg, mount_yaw_deg, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
            if mount_roll_deg != 0 or mount_pitch_deg != 0 or mount_yaw_deg != 0:
                raise NotAchievedException("Mount stabilising when not requested")

            # center RC tilt control and change mount to RC_TARGETING mode
            self.progress("Gimbal to RC Targeting mode")
            self.set_rc(6, pitch_rc_neutral)
            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

            # pitch vehicle back and confirm gimbal is stabilising
            self.progress("Pitching vehicle")
            self.do_pitch(despitch)
            self.wait_pitch(despitch, despitch_tolerance)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

            # point gimbal at specified angle
            self.progress("Point gimbal using GIMBAL_MANAGER_PITCHYAW (ANGLE)")
            self.do_pitch(0)    # level vehicle
            self.wait_pitch(0, despitch_tolerance)
            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
            for (method, angle) in (self.run_cmd, -20), (self.run_cmd_int, -30):
                method(
                    mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                    p1=angle,   # pitch angle in degrees
                    p2=0,     # yaw angle in degrees
                    p3=0,     # pitch rate in degrees (NaN to ignore)
                    p4=0,     # yaw rate in degrees (NaN to ignore)
                    p5=0,     # flags (0=Body-frame, 16/GIMBAL_MANAGER_FLAGS_YAW_LOCK=Earth Frame)
                    p6=0,     # unused
                    p7=0,     # gimbal id
                )
                self.test_mount_pitch(angle, 1, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

            # this is a one-off; ArduCopter *will* time out this directive!
            self.progress("Levelling aircraft")
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                1, # target sysid
                1, # target compid
                0, # bitmask of things to ignore
                mavextra.euler_to_quat([0, 0, 0]), # att
                0, # roll rate  (rad/s)
                0, # pitch rate (rad/s)
                0, # yaw rate   (rad/s)
                0.5) # thrust, 0 to 1, translated to a climb/descent rate

            self.wait_groundspeed(0, 1)

            # now test RC targeting
            self.progress("Testing mount RC targeting")

            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.test_mount_rc_targetting(
                pitch_rc_neutral=pitch_rc_neutral,
                do_rate_tests=do_rate_tests,
            )

            self.progress("Testing mount ROI behaviour")
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            start = self.mav.location()
            self.progress("start=%s" % str(start))
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     10,
                                                     20)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI_LOCATION")
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                p5=roi_lat,
                p6=roi_lon,
                p7=roi_alt,
            )
            self.test_mount_pitch(-52, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)
            self.progress("Using MAV_CMD_DO_SET_ROI_LOCATION")
            # start by pointing the gimbal elsewhere with a
            # known-working command:
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                p5=roi_lat + 1,
                p6=roi_lon + 1,
                p7=roi_alt,
            )
            # now point it with command_int:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                p5=int(roi_lat * 1e7),
                p6=int(roi_lon * 1e7),
                p7=roi_alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.test_mount_pitch(-52, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)

            self.progress("Using MAV_CMD_DO_SET_ROI_NONE")
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
            self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

            start = self.mav.location()
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     -100,
                                                     -200)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI")
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                p5=roi_lat,
                p6=roi_lon,
                p7=roi_alt,
            )
            self.test_mount_pitch(-7.5, 1, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)

            start = self.mav.location()
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     -100,
                                                     -200)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI (COMMAND_INT)")
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                0,
                0,
                0,
                0,
                int(roi_lat*1e7),
                int(roi_lon*1e7),
                roi_alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            )
            self.test_mount_pitch(-7.5, 1, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)
            self.progress("Using MAV_CMD_DO_SET_ROI (COMMAND_INT), absolute-alt-frame")
            # this is pointing essentially straight down
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                0,
                0,
                0,
                0,
                int(roi_lat*1e7),
                int(roi_lon*1e7),
                roi_alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            )
            self.test_mount_pitch(-70, 1, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT, hold=2)

            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

            self.progress("Testing mount roi-sysid behaviour")
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)
            start = self.mav.location()
            self.progress("start=%s" % str(start))
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     10,
                                                     20)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI_SYSID")
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
                p1=self.mav.source_system,
            )
            self.mav.mav.global_position_int_send(
                0, # time boot ms
                int(roi_lat * 1e7),
                int(roi_lon * 1e7),
                0 * 1000, # mm alt amsl
                0 * 1000, # relalt mm UP!
                0, # vx
                0, # vy
                0, # vz
                0 # heading
            )
            self.test_mount_pitch(-89, 5, mavutil.mavlink.MAV_MOUNT_MODE_SYSID_TARGET, hold=2)

            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
                p1=self.mav.source_system,
            )
            self.mav.mav.global_position_int_send(
                0, # time boot ms
                int(roi_lat * 1e7),
                int(roi_lon * 1e7),
                670 * 1000, # mm alt amsl
                100 * 1000, # mm UP!
                0, # vx
                0, # vy
                0, # vz
                0 # heading
            )
            self.test_mount_pitch(
                68,
                5,
                mavutil.mavlink.MAV_MOUNT_MODE_SYSID_TARGET,
                hold=2,
                constrained=constrain_sysid_target,
            )

            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

            self.disarm_vehicle(force=True)

            self.test_mount_body_yaw()

    def test_mount_body_yaw(self):
        '''check reporting of yaw'''
        # change mount to neutral mode (point forward, not stabilising)
        self.takeoff(10, mode='GUIDED')

        self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

        for heading in 30, 45, 150:
            self.guided_achieve_heading(heading)

            r, p , y, yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()

            if yaw_is_absolute:
                raise NotAchievedException("Expected a relative yaw")

            if y > 1:
                raise NotAchievedException("Bad yaw (y=%f)")

        self.do_RTL()

    def Mount(self):
        '''test servo mount'''
        self.setup_servo_mount()
        self.reboot_sitl() # to handle MNT_TYPE changing
        self.mount_test_body()

    def MountSolo(self):
        '''test type=2, a "Solo" mount'''
        self.set_parameters({
            "MNT1_TYPE": 2,
            "RC6_OPTION": 213,  # MOUNT1_PITCH
        })
        self.customise_SITL_commandline([
            "--gimbal" # connects on port 5762
        ])
        self.mount_test_body(
            pitch_rc_neutral=1818,
            do_rate_tests=False,  # solo can't do rate control (yet?)
            constrain_sysid_target=False,  # not everything constrains all angles
        )

    def assert_mount_rpy(self, r, p, y, tolerance=1):
        '''assert mount atttiude in degrees'''
        got_r, got_p, got_y, yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
        for (want, got, name) in (r, got_r, "roll"), (p, got_p, "pitch"), (y, got_y, "yaw"):
            if abs(want - got) > tolerance:
                raise NotAchievedException("%s incorrect; want=%f got=%f" %
                                           (name, want, got))

    def neutralise_gimbal(self):
        '''put mount into neutralise mode, assert it is at zero angles'''
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL,
        )
        self.test_mount_pitch(0, 0, mavutil.mavlink.MAV_MOUNT_MODE_RETRACT)

    def MAV_CMD_DO_MOUNT_CONTROL(self):
        '''test MAV_CMD_DO_MOUNT_CONTROL mavlink command'''

        # setup mount parameters
        self.context_push()
        self.setup_servo_mount()
        self.reboot_sitl() # to handle MNT_TYPE changing

        takeoff_loc = self.mav.location()

        self.takeoff(20, mode='GUIDED')
        self.guided_achieve_heading(315)

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
        )

        for method in self.run_cmd, self.run_cmd_int:
            self.start_subtest("MAV_MOUNT_MODE_GPS_POINT")

            self.progress("start=%s" % str(takeoff_loc))
            t = self.offset_location_ne(takeoff_loc, 20, 0)
            self.progress("targeting=%s" % str(t))

            # this command is *weird* as the lat/lng is *always* 1e7,
            # even when transported via COMMAND_LONG!
            x = int(t.lat * 1e7)
            y = int(t.lng * 1e7)
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p4=0,  # this is a relative altitude!
                p5=x,
                p6=y,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT,
            )
            self.test_mount_pitch(-45, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)
            self.neutralise_gimbal()

            self.start_subtest("MAV_MOUNT_MODE_HOME_LOCATION")
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION,
            )
            self.test_mount_pitch(-90, 5, mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION)
            self.neutralise_gimbal()

            # try an invalid mount mode.  Note that this is asserting we
            # are receiving a result code which is actually incorrect;
            # this should be MAV_RESULT_DENIED
            self.start_subtest("Invalid mode")
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=87,
                want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            )

            self.start_subtest("MAV_MOUNT_MODE_MAVLINK_TARGETING")
            r = 15
            p = 20
            y = 30
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p1=p,
                p2=r,
                p3=y,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            )
            self.delay_sim_time(2)
            self.assert_mount_rpy(r, p, y)
            self.neutralise_gimbal()

            self.start_subtest("MAV_MOUNT_MODE_RC_TARGETING")
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
            )
            self.test_mount_rc_targetting()

            self.start_subtest("MAV_MOUNT_MODE_RETRACT")
            self.context_push()
            retract_r = 13
            retract_p = 23
            retract_y = 33
            self.set_parameters({
                "MNT1_RETRACT_X": retract_r,
                "MNT1_RETRACT_Y": retract_p,
                "MNT1_RETRACT_Z": retract_y,
            })
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
            )
            self.delay_sim_time(3)
            self.assert_mount_rpy(retract_r, retract_p, retract_y)
            self.context_pop()

        self.do_RTL()

        self.context_pop()
        self.reboot_sitl()

    def AutoYawDO_MOUNT_CONTROL(self):
        '''test AutoYaw behaviour when MAV_CMD_DO_MOUNT_CONTROL sent to Mount without Yaw control'''

        # setup mount parameters
        self.context_push()

        yaw_servo = 7
        self.setup_servo_mount(roll_servo=5, pitch_servo=6, yaw_servo=yaw_servo)
        # Disable Mount Yaw servo
        self.set_parameters({
            "SERVO%u_FUNCTION" % yaw_servo: 0,
        })
        self.reboot_sitl() # to handle MNT_TYPE changing

        self.takeoff(20, mode='GUIDED')

        for mount_yaw in [-45, 0, 45]:
            heading = 330
            self.guided_achieve_heading(heading)
            self.assert_heading(heading)

            self.neutralise_gimbal()

            r = 15
            p = 20
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p1=p,
                p2=r,
                p3=mount_yaw,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            )
            self.delay_sim_time(5)
            # We have disabled yaw servo, so expect mount yaw to be zero
            self.assert_mount_rpy(r, p, 0)
            # But we expect the copter to yaw instead
            self.assert_heading(heading + mount_yaw)

        self.do_RTL()

        self.context_pop()
        self.reboot_sitl()

    def MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE(self):
        '''test MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE mavlink command'''
        # setup mount parameters
        self.context_push()
        self.setup_servo_mount()
        self.reboot_sitl() # to handle MNT_TYPE changing

        self.context_set_message_rate_hz('GIMBAL_MANAGER_STATUS', 10)
        self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
            "gimbal_device_id": 1,
            "primary_control_sysid": 0,
            "primary_control_compid": 0,
        })

        for method in self.run_cmd, self.run_cmd_int:
            self.start_subtest("set_sysid-compid")
            method(
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                p1=37,
                p2=38,
            )
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": 37,
                "primary_control_compid": 38,
            })

            self.start_subtest("leave unchanged")
            method(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, p1=-1)
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": 37,
                "primary_control_compid": 38,
            })

            # ardupilot currently handles this incorrectly:
            # self.start_subtest("self-controlled")
            # method(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, p1=-2)
            # self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
            #     "gimbal_device_id": 1,
            #     "primary_control_sysid": 1,
            #     "primary_control_compid": 1,
            # })

            self.start_subtest("release control")
            method(
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                p1=self.mav.source_system,
                p2=self.mav.source_component,
            )
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": self.mav.source_system,
                "primary_control_compid": self.mav.source_component,
            })
            method(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, p1=-3)
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": 0,
                "primary_control_compid": 0,
            })

        self.context_pop()
        self.reboot_sitl()

    def MountYawVehicleForMountROI(self):
        '''Test Camera/Antenna Mount vehicle yawing for ROI'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        yaw_servo = 7
        self.setup_servo_mount(yaw_servo=yaw_servo)
        self.reboot_sitl() # to handle MNT1_TYPE changing

        self.progress("checking ArduCopter yaw-aircraft-for-roi")
        self.takeoff(20, mode='GUIDED')

        m = self.assert_receive_message('VFR_HUD')
        self.progress("current heading %u" % m.heading)
        self.set_parameter("SERVO%u_FUNCTION" % yaw_servo, 0) # yaw
        self.progress("Waiting for check_servo_map to do its job")
        self.delay_sim_time(5)
        self.progress("Pointing North")
        self.guided_achieve_heading(0)
        self.delay_sim_time(5)
        start = self.mav.location()
        (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                 start.lng,
                                                 -100,
                                                 -100)
        roi_alt = 0
        self.progress("Using MAV_CMD_DO_SET_ROI")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            p5=roi_lat,
            p6=roi_lon,
            p7=roi_alt,
        )

        self.progress("Waiting for vehicle to point towards ROI")
        self.wait_heading(225, timeout=600, minimum_duration=2)

        # the following numbers are 1-degree-latitude and
        # 0-degrees longitude - just so that we start to
        # really move a lot.
        there = mavutil.location(1, 0, 0, 0)

        self.progress("Starting to move")
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-lat-lon-alt
            there.lat, # lat
            there.lng, # lon
            there.alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )

        self.progress("Starting to move changes the target")
        bearing = self.bearing_to(there)
        self.wait_heading(bearing, timeout=600, minimum_duration=2)

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            p5=roi_lat,
            p6=roi_lon,
            p7=roi_alt,
        )

        self.progress("Wait for vehicle to point sssse due to moving")
        self.wait_heading(170, timeout=600, minimum_duration=1)

        self.do_RTL()

    def ThrowMode(self):
        '''Fly Throw Mode'''
        # test boomerang mode:
        self.progress("Throwing vehicle away")
        self.set_parameters({
            "THROW_NEXTMODE": 6,
            "SIM_SHOVE_Z": -30,
            "SIM_SHOVE_X": -20,
        })
        self.change_mode('THROW')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        try:
            self.set_parameter("SIM_SHOVE_TIME", 500)
        except ValueError:
            # the shove resets this to zero
            pass

        tstart = self.get_sim_time()
        self.wait_mode('RTL')
        max_good_tdelta = 15
        tdelta = self.get_sim_time() - tstart
        self.progress("Vehicle in RTL")
        self.wait_rtl_complete()
        self.progress("Vehicle disarmed")
        if tdelta > max_good_tdelta:
            raise NotAchievedException("Took too long to enter RTL: %fs > %fs" %
                                       (tdelta, max_good_tdelta))
        self.progress("Vehicle returned")

    def hover_and_check_matched_frequency_with_fft_and_psd(self, dblevel=-15, minhz=200, maxhz=300, peakhz=None,
                                                           reverse=None, takeoff=True, instance=0):
        '''Takeoff and hover, checking the noise against the provided db level and returning psd'''
        # find a motor peak
        if takeoff:
            self.takeoff(10, mode="ALT_HOLD")

        tstart, tend, hover_throttle = self.hover_for_interval(15)
        self.do_RTL()

        psd = self.mavfft_fttd(1, instance, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        freq = psd["F"][numpy.argmax(psd["X"][minhz:maxhz]) + minhz] * (1000. / 1024.)
        peakdb = numpy.amax(psd["X"][minhz:maxhz])
        if peakdb < dblevel or (peakhz is not None and abs(freq - peakhz) / peakhz > 0.05):
            if reverse is not None:
                self.progress("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
            else:
                raise NotAchievedException("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            if reverse is not None:
                raise NotAchievedException(
                    "Detected motor peak at %fHz, throttle %f%%, %fdB" %
                    (freq, hover_throttle, peakdb))
            else:
                self.progress("Detected motor peak at %fHz, throttle %f%%, %fdB" %
                              (freq, hover_throttle, peakdb))

        return freq, hover_throttle, peakdb, psd

    def hover_and_check_matched_frequency_with_fft(self, dblevel=-15, minhz=200, maxhz=300, peakhz=None,
                                                   reverse=None, takeoff=True, instance=0):
        '''Takeoff and hover, checking the noise against the provided db level and returning peak db'''
        freq, hover_throttle, peakdb, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(dblevel, minhz,
                                                                    maxhz, peakhz, reverse, takeoff, instance)
        return freq, hover_throttle, peakdb

    def get_average_esc_frequency(self):
        mlog = self.dfreader_for_current_onboard_log()
        rpm_total = 0
        rpm_count = 0
        tho = 0
        while True:
            m = mlog.recv_match()
            if m is None:
                break
            msg_type = m.get_type()
            if msg_type == "CTUN":
                tho = m.ThO
            elif msg_type == "ESC" and tho > 0.1:
                rpm_total += m.RPM
                rpm_count += 1

        esc_hz = rpm_total / (rpm_count * 60)
        return esc_hz

    def DynamicNotches(self):
        """Use dynamic harmonic notch to control motor noise."""
        self.progress("Flying with dynamic notches")

        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100, # set the gyro filter high so we can observe behaviour
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            "SIM_GYR1_RND": 20,
        })
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        # find a motor peak
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-15, 200, 300)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": freq,
            "INS_HNTCH_REF": hover_throttle/100.,
            "INS_HNTCH_HMNCS": 5, # first and third harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": freq/2,
        })
        self.reboot_sitl()

        freq, hover_throttle, peakdb1 = \
            self.hover_and_check_matched_frequency_with_fft(-10, 20, 350, reverse=True)

        # now add double dynamic notches and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 1)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2 = \
            self.hover_and_check_matched_frequency_with_fft(-15, 20, 350, reverse=True)

        # double-notch should do better, but check for within 5%
        if peakdb2 * 1.05 > peakdb1:
            raise NotAchievedException(
                "Double-notch peak was higher than single-notch peak %fdB > %fdB" %
                (peakdb2, peakdb1))

        # now add triple dynamic notches and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 16)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2 = \
            self.hover_and_check_matched_frequency_with_fft(-15, 20, 350, reverse=True)

        # triple-notch should do better, but check for within 5%
        if peakdb2 * 1.05 > peakdb1:
            raise NotAchievedException(
                "Triple-notch peak was higher than single-notch peak %fdB > %fdB" %
                (peakdb2, peakdb1))

        # now add quintuple dynamic notches and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 64)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2 = \
            self.hover_and_check_matched_frequency_with_fft(-15, 20, 350, reverse=True)

        # triple-notch should do better, but check for within 5%
        if peakdb2 * 1.05 > peakdb1:
            raise NotAchievedException(
                "Quintuple-notch peak was higher than single-notch peak %fdB > %fdB" %
                (peakdb2, peakdb1))

    def DynamicRpmNotches(self):
        """Use dynamic harmonic notch to control motor noise via ESC telemetry."""
        self.progress("Flying with ESC telemetry driven dynamic notches")

        self.set_rc_default()
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 300, # set gyro filter high so we can observe behaviour
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            "SIM_GYR1_RND": 20,
            "SIM_ESC_TELEM": 1
        })
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        # find a motor peak, the peak is at about 190Hz, so checking between 50 and 320Hz should be safe.
        # there is a second harmonic at 380Hz which should be avoided to make the test reliable
        # detect at -5dB so we don't pick some random noise as the peak. The actual peak is about +15dB
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-5, 50, 320)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 4,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 5, # first and third harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_MODE": 3,
        })
        self.reboot_sitl()

        # -10dB is pretty conservative - actual is about -25dB
        freq, hover_throttle, peakdb1, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb1 = psd["X"][int(esc_hz)]

        # now add notch-per motor and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 2)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb2 = psd["X"][int(esc_hz)]

        # notch-per-motor will be better at the average ESC frequency
        if esc_peakdb2 > esc_peakdb1:
            raise NotAchievedException(
                "Notch-per-motor peak was higher than single-notch peak %fdB > %fdB" %
                (esc_peakdb2, esc_peakdb1))

        # check that the noise is being squashed at all. this needs to be an aggressive check so that failure happens easily
        # testing shows this to be -58dB on average
        if esc_peakdb2 > -25:
            raise NotAchievedException(
                "Notch-per-motor had a peak of %fdB there should be none" % esc_peakdb2)

        # Now do it again for an octacopter
        self.progress("Flying Octacopter with ESC telemetry driven dynamic notches")
        self.set_parameter("INS_HNTCH_OPTS", 0)
        self.customise_SITL_commandline(
            [],
            defaults_filepath=','.join(self.model_defaults_filepath("octa")),
            model="octa"
        )
        freq, hover_throttle, peakdb1, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb1 = psd["X"][int(esc_hz)]

        # now add notch-per motor and check that the peak is squashed
        self.set_parameter("INS_HNTCH_HMNCS", 1)
        self.set_parameter("INS_HNTCH_OPTS", 2)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-15, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb2 = psd["X"][int(esc_hz)]

        # notch-per-motor will be better at the average ESC frequency
        if esc_peakdb2 > esc_peakdb1:
            raise NotAchievedException(
                "Notch-per-motor peak was higher than single-notch peak %fdB > %fdB" %
                (esc_peakdb2, esc_peakdb1))

    def DynamicRpmNotchesRateThread(self):
        """Use dynamic harmonic notch to control motor noise via ESC telemetry."""
        self.progress("Flying with ESC telemetry driven dynamic notches")
        self.context_push()
        self.set_rc_default()
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 300, # set gyro filter high so we can observe behaviour
            "LOG_BITMASK": 959,
            "LOG_DISARMED": 0,
            "SIM_VIB_MOT_MAX": 350,
            "SIM_GYR1_RND": 20,
            "SIM_ESC_TELEM": 1,
            "FSTRATE_ENABLE": 1
        })
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        # find a motor peak, the peak is at about 190Hz, so checking between 50 and 320Hz should be safe.
        # there is a second harmonic at 380Hz which should be avoided to make the test reliable
        # detect at -5dB so we don't pick some random noise as the peak. The actual peak is about +15dB
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-5, 50, 320)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 4,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 5, # first and third harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_MODE": 3,
            "FSTRATE_ENABLE": 1
        })
        self.reboot_sitl()

        # -10dB is pretty conservative - actual is about -25dB
        freq, hover_throttle, peakdb1, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb1 = psd["X"][int(esc_hz)]

        # now add notch-per motor and check that the peak is squashed
        self.set_parameter("INS_HNTCH_OPTS", 2)
        self.reboot_sitl()

        freq, hover_throttle, peakdb2, psd = \
            self.hover_and_check_matched_frequency_with_fft_and_psd(-10, 50, 320, reverse=True, instance=2)
        # find the noise at the motor frequency
        esc_hz = self.get_average_esc_frequency()
        esc_peakdb2 = psd["X"][int(esc_hz)]

        # notch-per-motor will be better at the average ESC frequency
        if esc_peakdb2 > esc_peakdb1:
            raise NotAchievedException(
                "Notch-per-motor peak was higher than single-notch peak %fdB > %fdB" %
                (esc_peakdb2, esc_peakdb1))

        # check that the noise is being squashed at all. this needs to be an aggressive check so that failure happens easily
        # testing shows this to be -58dB on average
        if esc_peakdb2 > -25:
            raise NotAchievedException(
                "Notch-per-motor had a peak of %fdB there should be none" % esc_peakdb2)
        self.context_pop()
        self.reboot_sitl()

    def hover_and_check_matched_frequency(self, dblevel=-15, minhz=200, maxhz=300, fftLength=32, peakhz=None):
        '''do a simple up-and-down test flight with current vehicle state.
        Check that the onboard filter comes up with the same peak-frequency that
        post-processing does.'''
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(minhz * scale)
        smaxhz = int(maxhz * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])

        self.progress("Post-processing FFT detected motor peak at %fHz/%fdB, throttle %f%%" %
                      (freq, peakdb, hover_throttle))

        if peakdb < dblevel:
            raise NotAchievedException(
                "Detected motor peak not strong enough; want=%fdB got=%fdB" %
                (peakdb, dblevel))

        # caller can supply an expected frequency:
        if peakhz is not None and abs(freq - peakhz) / peakhz > 0.05:
            raise NotAchievedException(
                "Post-processing detected motor peak at wrong frequency; want=%fHz got=%fHz" %
                (peakhz, freq))

        # we have a peak make sure that the onboard filter detected
        # something close logging is at 10Hz

        # peak within resolution of FFT length
        pkAvg, nmessages = self.extract_median_FTN1_PkAvg_from_current_onboard_log(tstart, tend)
        self.progress("Onboard-FFT detected motor peak at %fHz (processed %d FTN1 messages)" % (pkAvg, nmessages))

        # accuracy is determined by sample rate and fft length, given
        # our use of quinn we could probably use half of this
        freqDelta = 1000. / fftLength
        if abs(pkAvg - freq) > freqDelta:
            raise NotAchievedException(
                "post-processed FFT does not  agree with onboard filter on peak frequency; onboard=%fHz post-processed=%fHz/%fdB" %  # noqa
                (pkAvg, freq, dblevel)
            )
        return freq

    def extract_median_FTN1_PkAvg_from_current_onboard_log(self, tstart, tend):
        '''extracts FTN1 messages from log, returns median of pkAvg values and
        the number of samples'''
        mlog = self.dfreader_for_current_onboard_log()
        freqs = []
        while True:
            m = mlog.recv_match(
                type='FTN1',
                blocking=False,
                condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6))
            if m is None:
                break
            freqs.append(m.PkAvg)
        return numpy.median(numpy.asarray(freqs)), len(freqs)

    def PIDNotches(self):
        """Use dynamic harmonic notch to control motor noise."""
        self.progress("Flying with PID notches")
        self.set_parameters({
            "FILT1_TYPE": 1,
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100, # set the gyro filter high so we can observe behaviour
            "LOG_BITMASK": 65535,
            "LOG_DISARMED": 0,
            "SIM_VIB_FREQ_X": 120,  # roll
            "SIM_VIB_FREQ_Y": 120,  # pitch
            "SIM_VIB_FREQ_Z": 180,  # yaw
            "FILT1_NOTCH_FREQ": 120,
            "ATC_RAT_RLL_NEF": 1,
            "ATC_RAT_PIT_NEF": 1,
            "ATC_RAT_YAW_NEF": 1,
            "SIM_GYR1_RND": 5,
        })
        self.reboot_sitl()

        self.hover_and_check_matched_frequency_with_fft(dblevel=5, minhz=20, maxhz=350, reverse=True)

    def StaticNotches(self):
        """Use static harmonic notch to control motor noise."""
        self.progress("Flying with Static notches")
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 4,
            "INS_GYRO_FILTER": 100, # set the gyro filter high so we can observe behaviour
            "LOG_BITMASK": 65535,
            "LOG_DISARMED": 0,
            "SIM_VIB_FREQ_X": 120,  # roll
            "SIM_VIB_FREQ_Y": 120,  # pitch
            "SIM_VIB_FREQ_Z": 120,  # yaw
            "SIM_VIB_MOT_MULT": 0,
            "SIM_GYR1_RND": 5,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 120,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 3, # first and second harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_MODE": 0, # static notch
        })
        self.reboot_sitl()

        self.hover_and_check_matched_frequency_with_fft(dblevel=-15, minhz=20, maxhz=350, reverse=True, instance=2)

    def ThrottleGainBoost(self):
        """Use PD and Angle P boost for anti-gravity."""
        # basic gyro sample rate test
        self.progress("Flying with Throttle-Gain Boost")

        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 959,
            "LOG_DISARMED": 0,
            "ATC_THR_G_BOOST": 5.0,
        })

        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")
        hover_time = 15
        self.progress("Hovering for %u seconds" % hover_time)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + hover_time:
            self.assert_receive_message('ATTITUDE')

        # fly fast forrest!
        self.set_rc(3, 1900)
        self.set_rc(2, 1200)
        self.wait_groundspeed(5, 1000)
        self.set_rc(3, 1500)
        self.set_rc(2, 1500)

        self.do_RTL()

    def test_gyro_fft_harmonic(self, averaging):
        """Use dynamic harmonic notch to control motor noise with harmonic matching of the first harmonic."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT harmonic - Gyro sample rate")
        self.start_subtest("Hover to calculate approximate hover frequency")
        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "FFT_THR_REF": self.get_parameter("MOT_THST_HOVER"),
            "SIM_GYR1_RND": 20,  # enable a noisy gyro
        })

        # motor peak enabling FFT will also enable the arming
        # check, self-testing the functionality
        self.set_parameters({
            "FFT_ENABLE": 1,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "FFT_SNR_REF": 10,
        })
        if averaging:
            self.set_parameter("FFT_NUM_FRAMES", 8)

        # Step 1: inject actual motor noise and use the FFT to track it
        self.set_parameters({
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 175Hz
            "FFT_WINDOW_SIZE": 64,
            "FFT_WINDOW_OLAP": 0.75,
        })

        self.reboot_sitl()
        freq = self.hover_and_check_matched_frequency(-15, 100, 250, 64)

        # Step 2: add a second harmonic and check the first is still tracked
        self.start_subtest("Add a fixed frequency harmonic at twice the hover frequency "
                           "and check the right harmonic is found")
        self.set_parameters({
            "SIM_VIB_FREQ_X": freq * 2,
            "SIM_VIB_FREQ_Y": freq * 2,
            "SIM_VIB_FREQ_Z": freq * 2,
            "SIM_VIB_MOT_MULT": 0.25,  # halve the motor noise so that the higher harmonic dominates
        })
        self.reboot_sitl()

        self.hover_and_check_matched_frequency(-15, 100, 250, 64, None)

        # Step 3: switch harmonics mid flight and check for tracking
        self.start_subtest("Switch harmonics mid flight and check the right harmonic is found")
        self.set_parameter("FFT_HMNC_PEAK", 0)
        self.reboot_sitl()

        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 10
        tstart, tend_unused, hover_throttle = self.hover_for_interval(hover_time)

        self.progress("Switching motor vibration multiplier")
        self.set_parameter("SIM_VIB_MOT_MULT", 5.0)

        tstart_unused, tend, hover_throttle = self.hover_for_interval(hover_time)

        self.do_RTL()

        # peak within resolution of FFT length, the highest energy peak switched but our detection should not
        pkAvg, nmessages = self.extract_median_FTN1_PkAvg_from_current_onboard_log(tstart, tend)

        freqDelta = 1000. / self.get_parameter("FFT_WINDOW_SIZE")

        if abs(pkAvg - freq) > freqDelta:
            raise NotAchievedException("FFT did not detect a harmonic motor peak, found %f, wanted %f" % (pkAvg, freq))

        # Step 4: dynamic harmonic
        self.start_subtest("Enable dynamic harmonics and make sure both frequency peaks are attenuated")
        # find a motor peak
        freq, hover_throttle, peakdb = self.hover_and_check_matched_frequency_with_fft(-15, 100, 350)

        # now add a dynamic notch and check that the peak is squashed
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_HMNCS": 1,
            "INS_HNTCH_MODE": 4,
            "INS_HNTCH_FREQ": freq,
            "INS_HNTCH_REF": hover_throttle/100.0,
            "INS_HNTCH_ATT": 100,
            "INS_HNTCH_BW": freq/2,
            "INS_HNTCH_OPTS": 3,
        })
        self.reboot_sitl()

        # 5db is far in excess of the attenuation that the double dynamic-harmonic notch is able
        # to provide (-7dB on average), but without the notch the peak is around 20dB so still a safe test
        self.hover_and_check_matched_frequency_with_fft(5, 100, 350, reverse=True)

        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
        })
        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameter("FFT_ENABLE", 0)

    def GyroFFTHarmonic(self):
        """Use dynamic harmonic notch to control motor noise with harmonic matching of the first harmonic."""
        self.test_gyro_fft_harmonic(False)

    def GyroFFTContinuousAveraging(self):
        """Use dynamic harmonic notch with FFT averaging to control motor noise
           with harmonic matching of the first harmonic."""
        self.test_gyro_fft_harmonic(True)

    def GyroFFT(self):
        """Use dynamic harmonic notch to control motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT - Gyro sample rate")

        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 4,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "SIM_GYR1_RND": 20,  # enable a noisy motor peak
        })
        # enabling FFT will also enable the arming check,
        # self-testing the functionality
        self.set_parameters({
            "FFT_ENABLE": 1,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "FFT_SNR_REF": 10,
            "FFT_WINDOW_SIZE": 128,
            "FFT_WINDOW_OLAP": 0.75,
            "FFT_SAMPLE_MODE": 0,
        })

        # Step 1: inject a very precise noise peak at 250hz and make sure the in-flight fft
        # can detect it really accurately. For a 128 FFT the frequency resolution is 8Hz so
        # a 250Hz peak should be detectable within 5%
        self.start_subtest("Inject noise at 250Hz and check the FFT can find the noise")
        self.set_parameters({
            "SIM_VIB_FREQ_X": 250,
            "SIM_VIB_FREQ_Y": 250,
            "SIM_VIB_FREQ_Z": 250,
        })

        self.reboot_sitl()

        # find a motor peak
        self.hover_and_check_matched_frequency(-15, 100, 350, 128, 250)

        # Step 1b: run the same test with an FFT length of 256 which is needed to flush out a
        # whole host of bugs related to uint8_t. This also tests very accurately the frequency resolution
        self.set_parameter("FFT_WINDOW_SIZE", 256)
        self.start_subtest("Inject noise at 250Hz and check the FFT can find the noise")

        self.reboot_sitl()

        # find a motor peak
        self.hover_and_check_matched_frequency(-15, 100, 350, 256, 250)
        self.set_parameter("FFT_WINDOW_SIZE", 128)

        # Step 2: inject actual motor noise and use the standard length FFT to track it
        self.start_subtest("Hover and check that the FFT can find the motor noise")
        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MAX": 250,  # gives a motor peak at about 175Hz
            "FFT_WINDOW_SIZE": 32,
            "FFT_WINDOW_OLAP": 0.5,
        })

        self.reboot_sitl()
        freq = self.hover_and_check_matched_frequency(-15, 100, 250, 32)

        self.set_parameter("SIM_VIB_MOT_MULT", 1.)

        # Step 3: add a FFT dynamic notch and check that the peak is squashed
        self.start_subtest("Add a dynamic notch, hover and check that the noise peak is now gone")
        self.set_parameters({
            "INS_LOG_BAT_OPT": 2,
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": freq,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": freq/2,
            "INS_HNTCH_MODE": 4,
        })
        self.reboot_sitl()

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        # fly fast forrest!
        self.set_rc(3, 1900)
        self.set_rc(2, 1200)
        self.wait_groundspeed(5, 1000)
        self.set_rc(3, 1500)
        self.set_rc(2, 1500)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(100 * scale)
        smaxhz = int(350 * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])
        if peakdb < 0:
            self.progress("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            raise NotAchievedException("Detected %fHz motor peak at %fdB" % (freq, peakdb))

        # Step 4: loop sample rate test with larger window
        self.start_subtest("Hover and check that the FFT can find the motor noise when running at fast loop rate")
        # we are limited to half the loop rate for frequency detection
        self.set_parameters({
            "FFT_MAXHZ": 185,
            "INS_LOG_BAT_OPT": 4,
            "SIM_VIB_MOT_MAX": 220,
            "FFT_WINDOW_SIZE": 64,
            "FFT_WINDOW_OLAP": 0.75,
            "FFT_SAMPLE_MODE": 1,
        })
        self.reboot_sitl()

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(15)
        self.do_RTL()

        # why are we not checking the results from that flight? -pb20220613

        # prevent update parameters from messing with the settings
        # when we pop the context
        self.set_parameter("FFT_ENABLE", 0)

    def GyroFFTAverage(self):
        """Use dynamic harmonic notch to control motor noise setup via FFT averaging."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT harmonic - Gyro sample rate")
        # Step 1
        self.start_subtest("Hover to calculate approximate hover frequency and see that it is tracked")
        # magic tridge EKF type that dramatically speeds up the test
        self.set_parameters({
            "INS_HNTCH_ATT": 100,
            "AHRS_EKF_TYPE": 10,
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 2,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 0,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "SIM_GYR1_RND": 20,  # enable a noisy gyro
        })
        # motor peak enabling FFT will also enable the arming
        # check, self-testing the functionality
        self.set_parameters({
            "FFT_ENABLE": 1,
            "FFT_WINDOW_SIZE": 64,  # not the default, but makes the test more reliable
            "FFT_SNR_REF": 10,
            "FFT_MINHZ": 80,
            "FFT_MAXHZ": 450,
        })

        # Step 1: inject actual motor noise and use the FFT to track it
        self.set_parameters({
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 175Hz
            "RC7_OPTION" : 162,   # FFT tune
        })

        self.reboot_sitl()

        # hover and engage FFT tracker
        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 60

        # start the tune
        self.set_rc(7, 2000)

        tstart, tend, hover_throttle = self.hover_for_interval(hover_time)

        # finish the tune
        self.set_rc(7, 1000)

        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        freq = psd["F"][numpy.argmax(psd["X"][50:450]) + 50] * (1000. / 1024.)

        detected_ref = self.get_parameter("INS_HNTCH_REF")
        detected_freq = self.get_parameter("INS_HNTCH_FREQ")
        self.progress("FFT detected parameters were %fHz, ref %f" % (detected_freq, detected_ref))

        # approximate the scaled frequency
        scaled_freq_at_hover = math.sqrt((hover_throttle / 100.) / detected_ref) * detected_freq

        # Check we matched
        if abs(scaled_freq_at_hover - freq) / scaled_freq_at_hover > 0.05:
            raise NotAchievedException("Detected frequency %fHz did not match required %fHz" %
                                       (scaled_freq_at_hover, freq))

        if self.get_parameter("INS_HNTCH_ENABLE") != 1:
            raise NotAchievedException("Harmonic notch was not enabled")

        # Step 2: now rerun the test and check that the peak is squashed
        self.start_subtest("Verify that noise is suppressed by the harmonic notch")
        self.hover_and_check_matched_frequency_with_fft(0, 100, 350, reverse=True, takeoff=False)

        # reset notch to defaults
        self.set_parameters({
            "INS_HNTCH_HMNCS": 3.0,
            "INS_HNTCH_ENABLE": 0.0,
            "INS_HNTCH_REF": 0.0,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_FM_RAT": 1.0
        })

        # Step 3: add a second harmonic and check the first is still tracked
        self.start_subtest("Add a fixed frequency harmonic at twice the hover frequency "
                           "and check the right harmonic is found")
        self.set_parameters({
            "SIM_VIB_FREQ_X": detected_freq * 2,
            "SIM_VIB_FREQ_Y": detected_freq * 2,
            "SIM_VIB_FREQ_Z": detected_freq * 2,
            "SIM_VIB_MOT_MULT": 0.25,  # halve the motor noise so that the higher harmonic dominates
        })
        self.reboot_sitl()

        # hover and engage FFT tracker
        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 60

        # start the tune
        self.set_rc(7, 2000)

        tstart, tend, hover_throttle = self.hover_for_interval(hover_time)

        # finish the tune
        self.set_rc(7, 1000)

        self.do_RTL()

        detected_ref = self.get_parameter("INS_HNTCH_REF")
        detected_freq = self.get_parameter("INS_HNTCH_FREQ")
        self.progress("FFT detected parameters were %fHz, ref %f" % (detected_freq, detected_ref))

        # approximate the scaled frequency
        scaled_freq_at_hover = math.sqrt((hover_throttle / 100.) / detected_ref) * detected_freq

        # Check we matched
        if abs(scaled_freq_at_hover - freq) / scaled_freq_at_hover > 0.05:
            raise NotAchievedException("Detected frequency %fHz did not match required %fHz" %
                                       (scaled_freq_at_hover, freq))

        if self.get_parameter("INS_HNTCH_ENABLE") != 1:
            raise NotAchievedException("Harmonic notch was not enabled")

        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
            "INS_HNTCH_HMNCS": 3.0,
            "INS_HNTCH_ENABLE": 0.0,
            "INS_HNTCH_REF": 0.0,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_BW": 40,
            "INS_HNTCH_FM_RAT": 1.0
        })
        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameter("FFT_ENABLE", 0)

    def GyroFFTPostFilter(self):
        """Use FFT-driven dynamic harmonic notch to control post-RPM filter motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT post-filter suppression - Gyro sample rate")
        # This set of parameters creates two noise peaks one at the motor frequency and one at 250Hz
        # we then use ESC telemetry to drive the notch to clean up the motor noise and a post-filter
        # FFT notch to clean up the remaining 250Hz. If either notch fails then the test will be failed
        # due to too much noise being present
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,    # magic tridge EKF type that dramatically speeds up the test
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 4,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 3,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "SIM_GYR1_RND": 20,     # enable a noisy gyro
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 1,   # first harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 30,
            "INS_HNTCH_MODE": 3,    # ESC telemetry
            "INS_HNTCH_OPTS": 2,    # notch-per-motor
            "INS_HNTC2_ENABLE": 1,
            "INS_HNTC2_FREQ": 80,
            "INS_HNTC2_REF": 1.0,
            "INS_HNTC2_HMNCS": 1,
            "INS_HNTC2_ATT": 50,
            "INS_HNTC2_BW": 40,
            "INS_HNTC2_MODE": 4,    # in-flight FFT
            "INS_HNTC2_OPTS": 18,   # triple-notch, notch-per-FFT peak
            "FFT_ENABLE": 1,
            "FFT_WINDOW_SIZE": 64,  # not the default, but makes the test more reliable
            "FFT_OPTIONS": 1,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 145Hz
            "SIM_VIB_FREQ_X": 250,  # create another peak at 250hz
            "SIM_VIB_FREQ_Y": 250,
            "SIM_VIB_FREQ_Z": 250,
            "SIM_GYR_FILE_RW": 2,   # write data to a file
        })
        self.reboot_sitl()

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(60)
        # fly fast forrest!
        self.set_rc(3, 1900)
        self.set_rc(2, 1200)
        self.wait_groundspeed(5, 1000)
        self.set_rc(3, 1500)
        self.set_rc(2, 1500)
        self.do_RTL()

        psd = self.mavfft_fttd(1, 2, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(100 * scale)
        smaxhz = int(350 * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])
        if peakdb < -5:
            self.progress("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            raise NotAchievedException("Detected %fHz motor peak at %fdB" % (freq, peakdb))

        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
            "SIM_GYR_FILE_RW": 0,  # stop writing data
            "FFT_ENABLE": 0,
        })

    def GyroFFTMotorNoiseCheck(self):
        """Use FFT to detect post-filter motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with FFT motor-noise detection - Gyro sample rate")
        # This set of parameters creates two noise peaks one at the motor frequency and one at 250Hz
        # we then use ESC telemetry to drive the notch to clean up the motor noise and a post-filter
        # FFT notch to clean up the remaining 250Hz. If either notch fails then the test will be failed
        # due to too much noise being present
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,    # magic tridge EKF type that dramatically speeds up the test
            "EK2_ENABLE": 0,
            "EK3_ENABLE": 0,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 4,
            "INS_GYRO_FILTER": 100,
            "INS_FAST_SAMPLE": 3,
            "LOG_BITMASK": 958,
            "LOG_DISARMED": 0,
            "SIM_DRIFT_SPEED": 0,
            "SIM_DRIFT_TIME": 0,
            "SIM_GYR1_RND": 200,     # enable a noisy gyro
            "INS_HNTCH_ENABLE": 1,
            "INS_HNTCH_FREQ": 80,
            "INS_HNTCH_REF": 1.0,
            "INS_HNTCH_HMNCS": 1,   # first harmonic
            "INS_HNTCH_ATT": 50,
            "INS_HNTCH_BW": 30,
            "INS_HNTCH_MODE": 3,    # ESC telemetry
            "INS_HNTCH_OPTS": 2,    # notch-per-motor
            "INS_HNTC2_ENABLE": 1,
            "INS_HNTC2_FREQ": 80,
            "INS_HNTC2_REF": 1.0,
            "INS_HNTC2_HMNCS": 1,
            "INS_HNTC2_ATT": 50,
            "INS_HNTC2_BW": 40,
            "INS_HNTC2_MODE": 0,    # istatic notch
            "INS_HNTC2_OPTS": 16,   # triple-notch
            "FFT_ENABLE": 1,
            "FFT_WINDOW_SIZE": 64,  # not the default, but makes the test more reliable
            "FFT_OPTIONS": 3,
            "FFT_MINHZ": 50,
            "FFT_MAXHZ": 450,
            "SIM_VIB_MOT_MAX": 250, # gives a motor peak at about 145Hz
            "SIM_VIB_FREQ_X": 250,  # create another peak at 250hz
            "SIM_VIB_FREQ_Y": 250,
            "SIM_VIB_FREQ_Z": 250,
            "SIM_GYR_FILE_RW": 2,   # write data to a file
        })
        self.reboot_sitl()

        # do test flight:
        self.takeoff(10, mode="ALT_HOLD")
        tstart, tend, hover_throttle = self.hover_for_interval(10)
        self.wait_statustext("Noise ", timeout=20)
        self.set_parameter("SIM_GYR1_RND", 0) # stop noise so that we can get home
        self.do_RTL()

        # prevent update parameters from messing with the settings when we pop the context
        self.set_parameters({
            "SIM_VIB_FREQ_X": 0,
            "SIM_VIB_FREQ_Y": 0,
            "SIM_VIB_FREQ_Z": 0,
            "SIM_VIB_MOT_MULT": 1.0,
            "SIM_GYR_FILE_RW": 0,  # stop writing data
            "FFT_ENABLE": 0,
        })

    def BrakeMode(self):
        '''Fly Brake Mode'''
        # test brake mode
        self.progress("Testing brake mode")
        self.takeoff(10, mode="LOITER")

        self.progress("Ensuring RC inputs have no effect in brake mode")
        self.change_mode("STABILIZE")
        self.set_rc(3, 1500)
        self.set_rc(2, 1200)
        self.wait_groundspeed(5, 1000)

        self.change_mode("BRAKE")
        self.wait_groundspeed(0, 1)

        self.set_rc(2, 1500)

        self.do_RTL()
        self.progress("Ran brake  mode")

    def fly_guided_move_to(self, destination, timeout=30):
        '''move to mavutil.location location; absolute altitude'''
        tstart = self.get_sim_time()
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-lat-lon-alt
            int(destination.lat * 1e7), # lat
            int(destination.lng * 1e7), # lon
            destination.alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise NotAchievedException()
            delta = self.get_distance(self.mav.location(), destination)
            self.progress("delta=%f (want <1)" % delta)
            if delta < 1:
                break

    def AltTypes(self):
        '''Test Different Altitude Types'''
        '''start by disabling GCS failsafe, otherwise we immediately disarm
        due to (apparently) not receiving traffic from the GCS for
        too long.  This is probably a function of --speedup'''

        '''this test flies the vehicle somewhere lower than were it started.
        It then disarms.  It then arms, which should reset home to the
        new, lower altitude.  This delta should be outside 1m but
        within a few metres of the old one.

        '''

        self.install_terrain_handlers_context()

        self.set_parameter("FS_GCS_ENABLE", 0)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        max_initial_home_alt_m = 500
        if m.relative_alt > max_initial_home_alt_m:
            raise NotAchievedException("Initial home alt too high (%fm > %fm)" %
                                       (m.relative_alt*1000, max_initial_home_alt_m*1000))
        orig_home_offset_mm = m.alt - m.relative_alt
        self.user_takeoff(5)

        self.progress("Flying to low position")
        current_alt = self.mav.location().alt
# 10m delta        low_position = mavutil.location(-35.358273, 149.169165, current_alt, 0)
        low_position = mavutil.location(-35.36200016, 149.16415599, current_alt, 0)
        self.fly_guided_move_to(low_position, timeout=240)
        self.change_mode('LAND')
        # expecting home to change when disarmed
        self.wait_landed_and_disarmed()
        # wait a while for home to move (it shouldn't):
        self.delay_sim_time(10)
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        new_home_offset_mm = m.alt - m.relative_alt
        home_offset_delta_mm = orig_home_offset_mm - new_home_offset_mm
        self.progress("new home offset: %f delta=%f" %
                      (new_home_offset_mm, home_offset_delta_mm))
        self.progress("gpi=%s" % str(m))
        max_home_offset_delta_mm = 10
        if home_offset_delta_mm > max_home_offset_delta_mm:
            raise NotAchievedException("Large home offset delta: want<%f got=%f" %
                                       (max_home_offset_delta_mm, home_offset_delta_mm))
        self.progress("Ensuring home moves when we arm")
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        post_arming_home_offset_mm = m.alt - m.relative_alt
        self.progress("post-arming home offset: %f" % (post_arming_home_offset_mm))
        self.progress("gpi=%s" % str(m))
        min_post_arming_home_offset_delta_mm = -2500
        max_post_arming_home_offset_delta_mm = -4000
        delta_between_original_home_alt_offset_and_new_home_alt_offset_mm = post_arming_home_offset_mm - orig_home_offset_mm
        self.progress("delta=%f-%f=%f" % (
            post_arming_home_offset_mm,
            orig_home_offset_mm,
            delta_between_original_home_alt_offset_and_new_home_alt_offset_mm))
        self.progress("Home moved %fm vertically" % (delta_between_original_home_alt_offset_and_new_home_alt_offset_mm/1000.0))
        if delta_between_original_home_alt_offset_and_new_home_alt_offset_mm > min_post_arming_home_offset_delta_mm:
            raise NotAchievedException(
                "Home did not move vertically on arming: want<=%f got=%f" %
                (min_post_arming_home_offset_delta_mm, delta_between_original_home_alt_offset_and_new_home_alt_offset_mm))
        if delta_between_original_home_alt_offset_and_new_home_alt_offset_mm < max_post_arming_home_offset_delta_mm:
            raise NotAchievedException(
                "Home moved too far vertically on arming: want>=%f got=%f" %
                (max_post_arming_home_offset_delta_mm, delta_between_original_home_alt_offset_and_new_home_alt_offset_mm))

        self.wait_disarmed()

    def PrecisionLoiterCompanion(self):
        """Use Companion PrecLand backend precision messages to loiter."""

        self.set_parameters({
            "PLND_ENABLED": 1,
            "PLND_TYPE": 1,  # enable companion backend:
            "RC7_OPTION": 39,  # set up a channel switch to enable precision loiter:
        })
        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()

        self.progress("Waiting for location")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        # we should be doing precision loiter at this point
        start = self.assert_receive_message('LOCAL_POSITION_NED')

        self.takeoff(20, mode='ALT_HOLD')

        # move away a little
        self.set_rc(2, 1550)
        self.wait_distance(5, accuracy=1)
        self.set_rc(2, 1500)
        self.change_mode('LOITER')

        # turn precision loiter on:
        self.context_collect('STATUSTEXT')
        self.set_rc(7, 2000)

        # try to drag aircraft to a position 5 metres north-east-east:
        self.precision_loiter_to_pos(start.x + 5, start.y + 10, start.z + 10, True)
        self.wait_statustext("PrecLand: Target Found", check_context=True, timeout=10)
        self.wait_statustext("PrecLand: Init Complete", check_context=True, timeout=10)
        # .... then northwest
        self.precision_loiter_to_pos(start.x + 5, start.y - 10, start.z + 10, True)

        # repeat using earth-frame targets
        self.precision_loiter_to_pos(start.x + 5, start.y - 10, start.z + 10, False)

        self.disarm_vehicle(force=True)

    def loiter_requires_position(self):
        # ensure we can't switch to LOITER without position
        self.progress("Ensure we can't enter LOITER without position")
        self.context_push()
        self.set_parameters({
            "GPS1_TYPE": 2,
            "SIM_GPS1_ENABLE": 0,
        })
        # if there is no GPS at all then we must direct EK3 to not use
        # it at all.  Otherwise it will never initialise, as it wants
        # to calculate the lag and size its delay buffers accordingly.
        self.set_parameters({
            "EK3_SRC1_POSXY": 0,
            "EK3_SRC1_VELZ": 0,
            "EK3_SRC1_VELXY": 0,
        })
        self.reboot_sitl()
        self.delay_sim_time(30)  # wait for accels/gyros to settle

        # check for expected EKF flags
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        expected_ekf_flags = (mavutil.mavlink.ESTIMATOR_ATTITUDE |
                              mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                              mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                              mavutil.mavlink.ESTIMATOR_CONST_POS_MODE)
        if ahrs_ekf_type == 2:
            expected_ekf_flags = expected_ekf_flags | mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL
        self.wait_ekf_flags(expected_ekf_flags, 0, timeout=120)

        # arm in Stabilize and attempt to switch to Loiter
        self.change_mode('STABILIZE')
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        self.run_cmd_do_set_mode(
            "LOITER",
            want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.wait_statustext("requires position", check_context=True)
        self.disarm_vehicle()
        self.context_pop()
        self.reboot_sitl()

    def ArmFeatures(self):
        '''Arm features'''
        self.loiter_requires_position()

        super(AutoTestCopter, self).ArmFeatures()

    def ParameterChecks(self):
        '''Test Arming Parameter Checks'''
        self.test_parameter_checks_poscontrol("PSC")

    def PosHoldTakeOff(self):
        """ensure vehicle stays put until it is ready to fly"""
        self.context_push()

        self.set_parameter("PILOT_TKOFF_ALT", 700)
        self.change_mode('POSHOLD')
        self.set_rc(3, 1000)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.delay_sim_time(2)
        # check we are still on the ground...
        relative_alt = self.get_altitude(relative=True)
        if relative_alt > 0.1:
            raise NotAchievedException("Took off prematurely")

        self.progress("Pushing throttle up")
        self.set_rc(3, 1710)
        self.delay_sim_time(0.5)
        self.progress("Bringing back to hover throttle")
        self.set_rc(3, 1500)

        # make sure we haven't already reached alt:
        relative_alt = self.get_altitude(relative=True)
        max_initial_alt = 2.0
        if abs(relative_alt) > max_initial_alt:
            raise NotAchievedException("Took off too fast (%f > %f" %
                                       (relative_alt, max_initial_alt))

        self.progress("Monitoring takeoff-to-alt")
        self.wait_altitude(6.9, 8, relative=True, minimum_duration=10)
        self.progress("takeoff OK")

        self.land_and_disarm()
        self.set_rc(8, 1000)

        self.context_pop()

    def initial_mode(self):
        return "STABILIZE"

    def initial_mode_switch_mode(self):
        return "STABILIZE"

    def default_mode(self):
        return "STABILIZE"

    def rc_defaults(self):
        ret = super(AutoTestCopter, self).rc_defaults()
        ret[3] = 1000
        ret[5] = 1800 # mode switch
        return ret

    def MANUAL_CONTROL(self):
        '''test MANUAL_CONTROL mavlink message'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)

        self.change_mode('STABILIZE')
        self.takeoff(10)

        tstart = self.get_sim_time_cached()
        want_pitch_degrees = -12
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach pitch")
            self.progress("Sending pitch-forward")
            self.mav.mav.manual_control_send(
                1, # target system
                500, # x (pitch)
                32767, # y (roll)
                32767, # z (thrust)
                32767, # r (yaw)
                0) # button mask
            m = self.assert_receive_message('ATTITUDE', verbose=True)
            p = math.degrees(m.pitch)
            self.progress("pitch=%f want<=%f" % (p, want_pitch_degrees))
            if p <= want_pitch_degrees:
                break
        self.mav.mav.manual_control_send(
            1, # target system
            32767, # x (pitch)
            32767, # y (roll)
            32767, # z (thrust)
            32767, # r (yaw)
            0) # button mask
        self.do_RTL()

    def check_avoidance_corners(self):
        self.takeoff(10, mode="LOITER")
        here = self.mav.location()
        self.set_rc(2, 1400)
        west_loc = mavutil.location(-35.363007,
                                    149.164911,
                                    here.alt,
                                    0)
        self.wait_location(west_loc, accuracy=6)
        north_loc = mavutil.location(-35.362908,
                                     149.165051,
                                     here.alt,
                                     0)
        self.reach_heading_manual(0)
        self.wait_location(north_loc, accuracy=6, timeout=200)
        self.reach_heading_manual(90)
        east_loc = mavutil.location(-35.363013,
                                    149.165194,
                                    here.alt,
                                    0)
        self.wait_location(east_loc, accuracy=6)
        self.reach_heading_manual(225)
        self.wait_location(west_loc, accuracy=6, timeout=200)
        self.set_rc(2, 1500)
        self.do_RTL()

    def OBSTACLE_DISTANCE_3D_test_angle(self, angle):
        now = self.get_sim_time_cached()

        distance = 15
        right = distance * math.sin(math.radians(angle))
        front = distance * math.cos(math.radians(angle))
        down = 0

        expected_distance_cm = distance * 100
        # expected orientation
        expected_orientation = int((angle+22.5)/45) % 8
        self.progress("Angle %f expected orient %u" %
                      (angle, expected_orientation))

        tstart = self.get_sim_time()
        last_send = 0
        m = None
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 100:
                raise NotAchievedException("Did not get correct angle back (last-message=%s)" % str(m))

            if now - last_send > 0.1:
                self.progress("ang=%f sending front=%f right=%f" %
                              (angle, front, right))
                self.mav.mav.obstacle_distance_3d_send(
                    int(now*1000),  # time_boot_ms
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    65535,
                    front,  # x (m)
                    right,  # y (m)
                    down,  # z (m)
                    0,  # min_distance (m)
                    20  # max_distance (m)
                )
                last_send = now
            m = self.mav.recv_match(type="DISTANCE_SENSOR",
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            # self.progress("Got (%s)" % str(m))
            if m.orientation != expected_orientation:
                # self.progress("Wrong orientation (want=%u got=%u)" %
                # (expected_orientation, m.orientation))
                continue
            if abs(m.current_distance - expected_distance_cm) > 1:
                # self.progress("Wrong distance (want=%f got=%f)" %
                # (expected_distance_cm, m.current_distance))
                continue
            self.progress("distance-at-angle good")
            break

    def OBSTACLE_DISTANCE_3D(self):
        '''Check round-trip behaviour of distance sensors'''
        self.context_push()
        self.set_parameters({
            "SERIAL5_PROTOCOL": 1,
            "PRX1_TYPE": 2,
            "SIM_SPEEDUP": 8,  # much GCS interaction
        })
        self.reboot_sitl()
        # need yaw estimate to stabilise:
        self.wait_ekf_happy(require_absolute=True)

        for angle in range(0, 360):
            self.OBSTACLE_DISTANCE_3D_test_angle(angle)

        self.context_pop()
        self.reboot_sitl()

    def AC_Avoidance_Proximity(self):
        '''Test proximity avoidance slide behaviour'''

        self.context_push()

        self.load_fence("copter-avoidance-fence.txt")
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "PRX1_TYPE": 10,
            "PRX_LOG_RAW": 1,
            "RC10_OPTION": 40, # proximity-enable
        })
        self.reboot_sitl()
        self.progress("Enabling proximity")
        self.set_rc(10, 2000)
        self.check_avoidance_corners()

        self.assert_current_onboard_log_contains_message("PRX")
        self.assert_current_onboard_log_contains_message("PRXR")

        self.disarm_vehicle(force=True)

        self.context_pop()
        self.reboot_sitl()

    def ProximitySensors(self):
        '''ensure proximity sensors return appropriate data'''

        self.set_parameters({
            "SERIAL5_PROTOCOL": 11,
            "OA_DB_OUTPUT": 3,
            "OA_TYPE": 2,
        })
        sensors = [  # tuples of name, prx_type
            ('ld06', 16, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 275,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1130,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 1200,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 625,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 967,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 760,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 765,
            }),
            ('sf45b', 8, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 270,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 258,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1146,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 632,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 629,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 972,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 774,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 774,
            }),
            ('rplidara2', 5, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 277,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1130,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 1288,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 626,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 970,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 762,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 790,
            }),
            ('terarangertower', 3, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 282,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 450,
            }),
        ]

        # the following is a "magic" location SITL understands which
        # has some posts near it:
        home_string = "%s,%s,%s,%s" % (51.8752066, 14.6487840, 54.15, 0)
        for (name, prx_type, expected_distances) in sensors:
            self.start_subtest("Testing %s" % name)
            self.set_parameter("PRX1_TYPE", prx_type)
            self.customise_SITL_commandline([
                "--serial5=sim:%s:" % name,
                "--home", home_string,
            ])
            self.wait_ready_to_arm()
            expected_distances_copy = copy.copy(expected_distances)
            tstart = self.get_sim_time()
            failed = False
            wants = []
            gots = []
            epsilon = 20
            while True:
                if self.get_sim_time_cached() - tstart > 30:
                    raise AutoTestTimeoutException("Failed to get distances")
                if len(expected_distances_copy.keys()) == 0:
                    break
                m = self.assert_receive_message("DISTANCE_SENSOR")
                if m.orientation not in expected_distances_copy:
                    continue
                got = m.current_distance
                want = expected_distances_copy[m.orientation]
                wants.append(want)
                gots.append(got)
                if abs(want - got) > epsilon:
                    failed = True
                del expected_distances_copy[m.orientation]
            if failed:
                raise NotAchievedException(
                    "Distance too great (%s) (want=%s != got=%s)" %
                    (name, wants, gots))

    def AC_Avoidance_Proximity_AVOID_ALT_MIN(self):
        '''Test proximity avoidance with AVOID_ALT_MIN'''
        self.set_parameters({
            "PRX1_TYPE": 2,
            "AVOID_ALT_MIN": 10,
        })
        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()

        self.change_mode('LOITER')
        self.wait_ekf_happy()

        self.set_parameter("SIM_SPEEDUP", 20)

        tstart = self.get_sim_time()
        while True:
            if self.armed(cached=True):
                break
            if self.get_sim_time_cached() - tstart > 60:
                raise AutoTestTimeoutException("Did not arm")
            self.mav.mav.distance_sensor_send(
                0, # time_boot_ms
                10, # min_distance cm
                500, # max_distance cm
                400, # current_distance cm
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                26, # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, # orientation
                255  # covariance
            )
            self.send_mavlink_arm_command()
            # make a minimal effort to not do ridiculous amounts of
            # mavlink to the autopilot:
            self.do_timesync_roundtrip()

        self.takeoff(15, mode='LOITER')
        self.progress("Poking vehicle; should avoid")

        def shove(a, b):
            self.mav.mav.distance_sensor_send(
                0,  # time_boot_ms
                10, # min_distance cm
                500, # max_distance cm
                20, # current_distance cm
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                21, # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, # orientation
                255  # covariance
            )
        self.wait_speed_vector_bf(
            Vector3(-0.4, 0.0, 0.0),
            timeout=10,
            called_function=shove,
        )

        self.change_alt(5)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                break
            vel = self.get_body_frame_velocity()
            if vel.length() > 0.5:
                raise NotAchievedException("Moved too much (%s)" %
                                           (str(vel),))
            shove(None, None)

        self.disarm_vehicle(force=True)

    def AC_Avoidance_Fence(self):
        '''Test fence avoidance slide behaviour'''
        self.load_fence("copter-avoidance-fence.txt")
        self.set_parameter("FENCE_ENABLE", 1)
        self.check_avoidance_corners()

    def AvoidanceAltFence(self):
        '''Test fence avoidance at minimum and maximum altitude'''
        self.context_push()

        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 9,   # min and max alt fence
            "FENCE_ALT_MIN": 10,
            "FENCE_ALT_MAX": 30,
        })

        self.change_mode('LOITER')
        self.wait_ekf_happy()

        tstart = self.get_sim_time()
        self.takeoff(15, mode='LOITER')
        self.progress("Increasing throttle, vehicle should stay below 30m")
        self.set_rc(3, 1920)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                break
            alt = self.get_altitude(relative=True)
            self.progress("Altitude %s" % alt)
            if alt > 30:
                raise NotAchievedException("Breached maximum altitude (%s)" % (str(alt),))

        self.progress("Decreasing, vehicle should stay above 10m")
        self.set_rc(3, 1080)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                break
            alt = self.get_altitude(relative=True)
            self.progress("Altitude %s" % alt)
            if alt < 10:
                raise NotAchievedException("Breached minimum altitude (%s)" % (str(alt),))
        self.context_pop()

        self.do_RTL()

    def mav_location_from_message_cache(self) -> mavutil.location:
        return mavutil.location(
            self.mav.messages['GPS_RAW_INT'].lat*1.0e-7,
            self.mav.messages['GPS_RAW_INT'].lon*1.0e-7,
            self.mav.messages['VFR_HUD'].alt,  # relative alt only
            self.mav.messages['VFR_HUD'].heading
        )

    def ModeFollow(self):
        '''Fly follow mode'''
        foll_ofs_x = 30 # metres
        self.set_parameters({
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": self.mav.source_system,
            "FOLL_OFS_X": -foll_ofs_x,
            "FOLL_OFS_TYPE": 1, # relative to other vehicle heading
        })
        self.takeoff(10, mode="LOITER")
        self.context_push()
        self.set_parameter("SIM_SPEEDUP", 1)
        self.change_mode("FOLLOW")
        new_loc = self.mav.location()
        new_loc_offset_n = 20
        new_loc_offset_e = 30
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
        self.progress("new_loc: %s" % str(new_loc))
        heading = 0
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f greenplane %f\n" %
                               (new_loc.lat, new_loc.lng, heading))

        expected_loc = copy.copy(new_loc)
        self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f hoop\n" %
                               (expected_loc.lat, expected_loc.lng))
        self.progress("expected_loc: %s" % str(expected_loc))

        origin = self.poll_message('GPS_GLOBAL_ORIGIN')

        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                raise NotAchievedException("Did not FOLLOW")
            if now - last_sent > 0.5:
                gpi = self.mav.mav.global_position_int_encode(
                    int(now * 1000), # time_boot_ms
                    int(new_loc.lat * 1e7),
                    int(new_loc.lng * 1e7),
                    int(new_loc.alt * 1000), # alt in mm
                    int(new_loc.alt * 1000 - origin.altitude), # relative alt - urp.
                    vx=0,
                    vy=0,
                    vz=0,
                    hdg=heading
                )
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
            self.assert_receive_message('GLOBAL_POSITION_INT')
            pos = self.mav_location_from_message_cache()
            delta = self.get_distance(expected_loc, pos)
            max_delta = 3
            self.progress("position delta=%f (want <%f)" % (delta, max_delta))
            if delta < max_delta:
                break

        self.start_subtest("Trying relative-follow mode")
        self.change_mode('LOITER')
        self.set_parameter('FOLL_ALT_TYPE', 1)  # use relative-home data
        new_loc = self.mav.location()
        new_loc_offset_n = -40
        new_loc_offset_e = 60
        new_loc.alt += 1
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
        expected_loc = copy.copy(new_loc)
        self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f hoop\n" %
                               (expected_loc.lat, expected_loc.lng))
        self.progress("expected_loc: %s" % str(expected_loc))
        self.progress("new_loc: %s" % str(new_loc))
        self.change_mode('FOLLOW')
        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                raise NotAchievedException("Did not FOLLOW")
            if now - last_sent > 0.5:
                gpi = self.mav.mav.global_position_int_encode(
                    int(now * 1000), # time_boot_ms
                    int(new_loc.lat * 1e7),
                    int(new_loc.lng * 1e7),
                    666, # alt in mm - note incorrect data!
                    int(new_loc.alt * 1000 - origin.altitude), # relative alt - urp.
                    vx=0,
                    vy=0,
                    vz=0,
                    hdg=heading
                )
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
            self.assert_receive_message('GLOBAL_POSITION_INT')
            pos = self.mav_location_from_message_cache()
            delta = self.get_distance(expected_loc, pos)
            max_delta = 3
            self.progress("position delta=%f (want <%f)" % (delta, max_delta))
            if delta < max_delta:
                break

        self.start_subtest("Trying follow-with-velocity mode")
        self.change_mode('LOITER')
        self.set_parameter('FOLL_ALT_TYPE', 1)  # use relative-home data
        new_loc = self.mav.location()
        vel_n = 3  # m/s
        vel_e = 2
        vel_d = -1
        tstart = self.get_sim_time()
        first_loop = True
        last_sent = 0
        last_loop_s = tstart
        heading = math.atan2(vel_n, vel_e)
        while True:
            now = self.get_sim_time_cached()
            dt = now - last_loop_s
            last_loop_s = now
            new_loc_offset_n = vel_n * dt
            new_loc_offset_e = vel_e * dt
            new_loc_offset_u = vel_d * dt * -1

            self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
            new_loc.alt += new_loc_offset_u
            expected_loc = copy.copy(new_loc)
            self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
            self.progress("expected_loc: %s" % str(expected_loc))
            self.progress("new_loc: %s" % str(new_loc))
            if first_loop:
                self.change_mode('FOLLOW')
                first_loop = False

            if now - tstart > 60:
                raise NotAchievedException("Did not FOLLOW")
            if now - last_sent > 0.5:
                gpi = self.mav.mav.global_position_int_encode(
                    int(now * 1000), # time_boot_ms
                    int(new_loc.lat * 1e7),
                    int(new_loc.lng * 1e7),
                    666, # alt in mm - note incorrect data!
                    int(new_loc.alt * 1000 - origin.altitude), # relative alt - urp.
                    vx=int(vel_n*100),
                    vy=int(vel_e*100),
                    vz=int(vel_d*100),
                    hdg=int(math.degrees(heading)*100)  # rad->cdeg
                )
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
            self.assert_receive_message('GLOBAL_POSITION_INT')
            pos = self.mav.location()
            delta = self.get_distance(expected_loc, pos)
            want_delta = foll_ofs_x
            self.progress("position delta=%f (want <%f)" % (delta, max_delta))
            if abs(want_delta - delta) < 3:
                break

        self.context_pop()
        self.do_RTL()

    def ModeFollow_with_FOLLOW_TARGET(self):
        '''test ModeFollow passing in FOLLOW_TARGET messages'''
        foll_ofs_x = 30 # metres
        self.set_parameters({
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": self.mav.source_system,
            "FOLL_OFS_X": -foll_ofs_x,
            "FOLL_OFS_TYPE": 1, # relative to other vehicle heading
        })
        self.takeoff(10, mode="LOITER")
        self.context_push()
        self.set_parameter("SIM_SPEEDUP", 1)
        self.change_mode("FOLLOW")
        new_loc = self.mav.location()
        new_loc_offset_n = 40
        new_loc_offset_e = 60
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
        self.progress("new_loc: %s" % str(new_loc))
        heading = 0
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f greenplane %f\n" %
                               (new_loc.lat, new_loc.lng, heading))

        expected_loc = copy.copy(new_loc)
        self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f hoop\n" %
                               (expected_loc.lat, expected_loc.lng))
        self.progress("expected_loc: %s" % str(expected_loc))

        def send_target():
            vel_n = 3  # m/s
            vel_e = 2
            # vel_d = -1
            heading = math.atan2(vel_n, vel_e)
            attitude = quaternion.Quaternion([
                math.radians(0),
                math.radians(0),
                math.radians(heading)
            ])
            now = self.get_sim_time_cached()
            self.mav.mav.follow_target_send(
                int(now * 1000), # time_boot_ms
                1 << 0 | 1 << 1 | 1 << 3,  # pos, vel, att+rates
                int(new_loc.lat * 1e7),
                int(new_loc.lng * 1e7),
                new_loc.alt, # alt in m
                [0, 0, 0],  # velocity m/s
                [0, 0, 0],  # acceleration m/s/s
                attitude,   # attitude quaternion
                [0, 0, 0],  # rates
                [0, 0, 0],  # covariances
                0  # custom state
            )
        self.wait_location(
            expected_loc,
            fn=send_target,
            fn_interval=0.5,
            accuracy=3,
            timeout=60,
        )

        self.start_subtest("Trying follow-with-velocity mode")
        self.change_mode('LOITER')
        self.set_parameter('FOLL_ALT_TYPE', 1)  # use relative-home data
        new_loc = self.mav.location()
        self.change_mode('FOLLOW')
        last_loop_s = self.get_sim_time_cached()

        vel_n = 3  # m/s
        vel_e = 2
        vel_d = 0

        mul = 40
        self.location_offset_ne(expected_loc, mul*vel_n, mul*vel_e)

        def send_target_vel():
            nonlocal last_loop_s

            now = self.get_sim_time_cached()
            dt = now - last_loop_s
            last_loop_s = now

            new_loc_offset_n = vel_n * dt
            new_loc_offset_e = vel_e * dt
            new_loc_offset_u = vel_d * dt * -1
            self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
            new_loc.alt += new_loc_offset_u

            # update heading
            heading = math.atan2(vel_n, vel_e)
            attitude = quaternion.Quaternion([
                math.radians(0),
                math.radians(0),
                math.radians(heading)
            ])
            now = self.get_sim_time_cached()
            self.mav.mav.follow_target_send(
                int(now * 1000), # time_boot_ms
                1 << 0 | 1 << 1 | 1 << 3,  # pos, vel, att+rates
                int(new_loc.lat * 1e7),
                int(new_loc.lng * 1e7),
                new_loc.alt, # alt in m
                [vel_n, vel_e, vel_d],  # velocity m/s
                [0, 0, 0],  # acceleration m/s/s
                attitude,   # attitude quaternion
                [0, 0, 0],  # rates
                [0, 0, 0],  # covariances
                0  # custom state
            )
        self.wait_speed_vector(
            Vector3(vel_n, vel_e, 0),
            timeout=100,
            called_function=lambda plop, empty: send_target_vel(),
            minimum_duration=2,
        )

        self.context_pop()
        self.do_RTL()

    def get_global_position_int(self, timeout=30):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get good global_position_int")
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            self.progress("GPI: %s" % str(m))
            if m is None:
                continue
            if m.lat != 0 or m.lon != 0:
                return m

    def BeaconPosition(self):
        '''Fly Beacon Position'''
        self.reboot_sitl()

        self.wait_ready_to_arm(require_absolute=True)

        old_pos = self.get_global_position_int()
        print("old_pos=%s" % str(old_pos))

        self.set_parameters({
            "BCN_TYPE": 10,
            "BCN_LATITUDE": SITL_START_LOCATION.lat,
            "BCN_LONGITUDE": SITL_START_LOCATION.lng,
            "BCN_ALT": SITL_START_LOCATION.alt,
            "BCN_ORIENT_YAW": 0,
            "AVOID_ENABLE": 4,
            "GPS1_TYPE": 0,
            "EK3_ENABLE": 1,
            "EK3_SRC1_POSXY": 4, # Beacon
            "EK3_SRC1_POSZ": 1,  # Baro
            "EK3_SRC1_VELXY": 0, # None
            "EK3_SRC1_VELZ": 0,  # None
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
        })
        self.reboot_sitl()

        # require_absolute=True infers a GPS is present
        self.wait_ready_to_arm(require_absolute=False)

        tstart = self.get_sim_time()
        timeout = 20
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get new position like old position")
            self.progress("Fetching location")
            new_pos = self.get_global_position_int()
            pos_delta = self.get_distance_int(old_pos, new_pos)
            max_delta = 1
            self.progress("delta=%u want <= %u" % (pos_delta, max_delta))
            if pos_delta <= max_delta:
                break

        self.progress("Moving to ensure location is tracked")
        self.takeoff(10, mode="STABILIZE")
        self.change_mode("CIRCLE")

        self.context_push()
        validator = vehicle_test_suite.TestSuite.ValidateGlobalPositionIntAgainstSimState(self, max_allowed_divergence=10)
        self.install_message_hook_context(validator)

        self.delay_sim_time(20)
        self.progress("Tracked location just fine")
        self.context_pop()

        self.change_mode("LOITER")
        self.wait_groundspeed(0, 0.3, timeout=120)
        self.land_and_disarm()

        self.assert_current_onboard_log_contains_message("BCN")

        self.disarm_vehicle(force=True)

    def AC_Avoidance_Beacon(self):
        '''Test beacon avoidance slide behaviour'''
        self.context_push()
        ex = None
        try:
            self.set_parameters({
                "BCN_TYPE": 10,
                "BCN_LATITUDE": int(SITL_START_LOCATION.lat),
                "BCN_LONGITUDE": int(SITL_START_LOCATION.lng),
                "BCN_ORIENT_YAW": 45,
                "AVOID_ENABLE": 4,
            })
            self.reboot_sitl()

            self.takeoff(10, mode="LOITER")
            self.set_rc(2, 1400)
            here = self.mav.location()
            west_loc = mavutil.location(-35.362919, 149.165055, here.alt, 0)
            self.wait_location(west_loc, accuracy=1)
            self.reach_heading_manual(0)
            north_loc = mavutil.location(-35.362881, 149.165103, here.alt, 0)
            self.wait_location(north_loc, accuracy=1)
            self.set_rc(2, 1500)
            self.set_rc(1, 1600)
            east_loc = mavutil.location(-35.362986, 149.165227, here.alt, 0)
            self.wait_location(east_loc, accuracy=1)
            self.set_rc(1, 1500)
            self.set_rc(2, 1600)
            south_loc = mavutil.location(-35.363025, 149.165182, here.alt, 0)
            self.wait_location(south_loc, accuracy=1)
            self.set_rc(2, 1500)
            self.do_RTL()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.clear_fence()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def BaroWindCorrection(self):
        '''Test wind estimation and baro position error compensation'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('Callisto'),
            model="octa-quad:@ROMFS/models/Callisto.json",
            wipe=True,
        )
        wind_spd_truth = 8.0
        wind_dir_truth = 90.0
        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "BARO1_WCF_ENABLE": 1.000000,
        })
        self.reboot_sitl()
        self.set_parameters({
            "BARO1_WCF_FWD": -0.300000,
            "BARO1_WCF_BCK": -0.300000,
            "BARO1_WCF_RGT": 0.300000,
            "BARO1_WCF_LFT": 0.300000,
            "BARO1_WCF_UP": 0.300000,
            "BARO1_WCF_DN": 0.300000,
            "SIM_BARO_WCF_FWD": -0.300000,
            "SIM_BARO_WCF_BAK": -0.300000,
            "SIM_BARO_WCF_RGT": 0.300000,
            "SIM_BARO_WCF_LFT": 0.300000,
            "SIM_BARO_WCF_UP": 0.300000,
            "SIM_BARO_WCF_DN": 0.300000,
            "SIM_WIND_DIR": wind_dir_truth,
            "SIM_WIND_SPD": wind_spd_truth,
            "SIM_WIND_T": 1.000000,
        })
        self.reboot_sitl()

        # require_absolute=True infers a GPS is present
        self.wait_ready_to_arm(require_absolute=False)

        self.progress("Climb to 20m in LOITER and yaw spin for 30 seconds")
        self.takeoff(10, mode="LOITER")
        self.set_rc(4, 1400)
        self.delay_sim_time(30)

        # check wind estimates
        m = self.assert_receive_message('WIND')
        speed_error = abs(m.speed - wind_spd_truth)
        angle_error = abs(m.direction - wind_dir_truth)
        if (speed_error > 1.0):
            raise NotAchievedException("Wind speed incorrect - want %f +-1 got %f m/s" % (wind_spd_truth, m.speed))
        if (angle_error > 15.0):
            raise NotAchievedException(
                "Wind direction incorrect - want %f +-15 got %f deg" %
                (wind_dir_truth, m.direction))
        self.progress("Wind estimate is good, now check height variation for 30 seconds")

        # check height stability over another 30 seconds
        z_min = 1E6
        z_max = -1E6
        tstart = self.get_sim_time()
        while (self.get_sim_time() < tstart + 30):
            m = self.assert_receive_message('LOCAL_POSITION_NED')
            if (m.z > z_max):
                z_max = m.z
            if (m.z < z_min):
                z_min = m.z
        if (z_max-z_min > 0.5):
            raise NotAchievedException("Height variation is excessive")
        self.progress("Height variation is good")

        self.set_rc(4, 1500)
        self.land_and_disarm()

    def wait_generator_speed_and_state(self, rpm_min, rpm_max, want_state, timeout=240):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not move to state/speed")

            m = self.assert_receive_message("GENERATOR_STATUS", timeout=10)

            if m.generator_speed < rpm_min:
                self.progress("Too slow (%u<%u)" % (m.generator_speed, rpm_min))
                continue
            if m.generator_speed > rpm_max:
                self.progress("Too fast (%u>%u)" % (m.generator_speed, rpm_max))
                continue
            if m.status != want_state:
                self.progress("Wrong state (got=%u want=%u)" % (m.status, want_state))
            break
        self.progress("Got generator speed and state")

    def RichenPower(self):
        '''Test RichenPower generator'''
        self.set_parameters({
            "SERIAL5_PROTOCOL": 30,
            "SIM_RICH_ENABLE": 1,
            "SERVO8_FUNCTION": 42,
            "SIM_RICH_CTRL": 8,
            "RC9_OPTION": 85,
            "LOG_DISARMED": 1,
            "BATT2_MONITOR": 17,
            "GEN_TYPE": 3,
        })
        self.reboot_sitl()
        self.set_rc(9, 1000) # remember this is a switch position - stop
        self.customise_SITL_commandline(["--serial5=sim:richenpower"])
        self.wait_statustext("requested state is not RUN", timeout=60)

        self.set_message_rate_hz("GENERATOR_STATUS", 10)

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        self.context_collect('STATUSTEXT')
        self.set_rc(9, 2000) # remember this is a switch position - run
        self.wait_statustext("Generator HIGH", check_context=True)
        self.set_rc(9, 1000) # remember this is a switch position - stop
        self.wait_statustext("requested state is not RUN", timeout=200)

        self.set_rc(9, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 8000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.set_rc(9, 2000) # remember this is a switch position - run
#        self.wait_generator_speed_and_state(3000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_WARMING_UP)

        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==1",  # id is zero-indexed
        )
        self.progress("Received battery status: %s" % str(bs))
        want_bs_volt = 50000
        if bs.voltages[0] != want_bs_volt:
            raise NotAchievedException("Battery voltage not as expected (want=%f) got=(%f)" % (want_bs_volt, bs.voltages[0],))

        self.progress("Moving *back* to idle")
        self.set_rc(9, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 10000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.progress("Moving *back* to run")
        self.set_rc(9, 2000) # remember this is a switch position - run
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        self.set_message_rate_hz("GENERATOR_STATUS", -1)
        self.set_parameter("LOG_DISARMED", 0)
        if not self.current_onboard_log_contains_message("RICH"):
            raise NotAchievedException("Did not find expected RICH message")

    def IE24(self):
        '''Test IntelligentEnergy 2.4kWh generator with V1 and V2 telemetry protocols'''
        protocol_ver = (1, 2)
        for ver in protocol_ver:
            self.start_subtest(f"IE24 general tests for protocol {ver}")
            self.context_push()
            self.run_IE24(ver)
            self.context_pop()
            self.reboot_sitl()

            self.start_subtest(f"IE24 low capacity failsafe test for protocol {ver}")
            self.context_push()
            self.test_IE24_low_capacity_failsafe(ver)
            self.context_pop()
            self.reboot_sitl()

    def run_IE24(self, proto_ver):
        '''Test IntelligentEnergy 2.4kWh generator'''
        elec_battery_instance = 2
        fuel_battery_instance = 1
        self.set_parameters({
            "SERIAL5_PROTOCOL": 30,
            "SERIAL5_BAUD": 115200,
            "GEN_TYPE": 2,
            "BATT%u_MONITOR" % (fuel_battery_instance + 1): 18,  # fuel-based generator
            "BATT%u_MONITOR" % (elec_battery_instance + 1): 17,
            "SIM_IE24_ENABLE": proto_ver,
            "LOG_DISARMED": 1,
        })

        self.customise_SITL_commandline(["--serial5=sim:ie24"])

        self.start_subtest("Protocol %i: ensure that BATTERY_STATUS for electrical generator message looks right" % proto_ver)
        self.start_subsubtest("Protocol %i: Checking original voltage (electrical)" % proto_ver)
        # ArduPilot spits out essentially uninitialised battery
        # messages until we read things from the battery:
        self.delay_sim_time(30)
        original_elec_m = self.wait_message_field_values('BATTERY_STATUS', {
            "charge_state": mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK
        }, instance=elec_battery_instance)
        original_fuel_m = self.wait_message_field_values('BATTERY_STATUS', {
            "charge_state": mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK
        }, instance=fuel_battery_instance)

        if original_elec_m.battery_remaining < 90:
            raise NotAchievedException("Bad original percentage")
        self.start_subsubtest("Ensure percentage is counting down")
        self.wait_message_field_values('BATTERY_STATUS', {
            "battery_remaining": original_elec_m.battery_remaining - 1,
        }, instance=elec_battery_instance)

        self.start_subtest("Protocol %i: ensure that BATTERY_STATUS for fuel generator message looks right" % proto_ver)
        self.start_subsubtest("Protocol %i: Checking original voltage (fuel)" % proto_ver)
        # ArduPilot spits out essentially uninitialised battery
        # messages until we read things from the battery:
        if original_fuel_m.battery_remaining <= 90:
            raise NotAchievedException("Bad original percentage (want=>%f got %f" % (90, original_fuel_m.battery_remaining))
        self.start_subsubtest("Protocol %i: Ensure percentage is counting down" % proto_ver)
        self.wait_message_field_values('BATTERY_STATUS', {
            "battery_remaining": original_fuel_m.battery_remaining - 1,
        }, instance=fuel_battery_instance)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        # Test for pre-arm check fail when state is not running
        self.start_subtest("Protocol %i: Without takeoff generator error should cause failsafe and disarm" % proto_ver)
        self.set_parameter("SIM_IE24_STATE", 8)
        self.wait_statustext("Status not running", timeout=40)
        self.try_arm(result=False,
                     expect_msg="Status not running")
        self.set_parameter("SIM_IE24_STATE", 2) # Explicitly set state to running

        # Test that error code does result in failsafe
        self.start_subtest("Protocol %i: Without taken off generator error should cause failsafe and disarm" % proto_ver)
        self.change_mode("STABILIZE")
        self.set_parameter("DISARM_DELAY", 0)
        self.arm_vehicle()
        self.set_parameter("SIM_IE24_ERROR", 30)
        self.disarm_wait(timeout=1)
        self.set_parameter("SIM_IE24_ERROR", 0)
        self.set_parameter("DISARM_DELAY", 10)

    def test_IE24_low_capacity_failsafe(self, proto_ver):
        elec_battery_instance = 2
        fuel_battery_instance = 1
        self.set_parameters({
            "SERIAL5_PROTOCOL": 30,
            "SERIAL5_BAUD": 115200,
            "GEN_TYPE": 2,
            "BATT%u_MONITOR" % (fuel_battery_instance + 1): 18,  # fuel-based generator
            "BATT%u_MONITOR" % (elec_battery_instance + 1): 17,
            "SIM_IE24_ENABLE": proto_ver,
            "LOG_DISARMED": 1,
        })

        self.customise_SITL_commandline(["--serial5=sim:ie24"])

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        original_fuel_m = self.wait_message_field_values('BATTERY_STATUS', {
            "charge_state": mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK
        }, instance=fuel_battery_instance)

        # set failsafe parameters
        self.set_parameters({
            "BATT%u_CAPACITY" % (fuel_battery_instance + 1): 100,
            "BATT%u_LOW_MAH" % (fuel_battery_instance + 1): original_fuel_m.battery_remaining - 5,
            "BATT%u_FS_LOW_ACT" % (fuel_battery_instance + 1): 1,  # LAND
        })

        self.takeoff(mode='GUIDED')
        self.wait_mode('LAND')
        self.wait_disarmed()
        self.set_rc(3, 1000)  # Restore the throttle stick since takeoff raised it.

    def LoweheiserAuto(self):
        '''Ensure the Loweheiser generator works as expected in auto-starter mode.'''

        gen_ctrl_ch = 9

        self.set_parameters({
            "SERIAL5_PROTOCOL": 2,    # mavlink
            "MAV3_OPTIONS": 2,        # private
            "GEN_TYPE": 4,            # loweheiser
            "EFI_TYPE": 4,            # loweheiser
            "SIM_EFI_TYPE": 2,        # loweheiser sim
            "BATT2_MONITOR": 17,  # generator (elec)
            "BATT3_MONITOR": 18,  # generator (fuel-level)
            "BATT3_CAPACITY": 10000,  # generator (fuel) in mL
            f"RC{gen_ctrl_ch}_OPTION": 85,  # generator control
            "LOG_DISARMED": 1
        })

        self.set_rc_from_map({
            gen_ctrl_ch: 1000,   # remember this is a switch position - stop
        })

        self.customise_SITL_commandline(["--serial5=sim:loweheiser"])

        self.set_parameters({
            "GEN_L_IDLE_TH": 25,
            "GEN_L_IDLE_TH_H": 40,
            "GEN_L_RUN_TEMP": 60,
            "GEN_L_IDLE_TEMP": 80,
        })

        self.reboot_sitl()

        self.assert_parameter_value("GEN_L_IDLE_TH", 25)

        self.delay_sim_time(10)  # so we can actually receive messages...

        #######################################################################
        # Generator OFF subtest.
        #######################################################################
        self.start_subtest("Checking GENERATOR_STATUS while OFF.")
        self.set_message_rate_hz("GENERATOR_STATUS", 10)
        self.delay_sim_time(1)

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        self.start_subtest("Checking EFI_STATUS while OFF.")
        self.set_message_rate_hz("EFI_STATUS", 10)
        self.delay_sim_time(1)
        self.assert_received_message_field_values('EFI_STATUS', {
            "health": 1,
            "ecu_index": 1.0,
            "rpm": 0.0,
            "fuel_consumed": 0,
            "fuel_flow": float("nan"),
            "engine_load": 0.0,
            "throttle_position": 0.0,
            "spark_dwell_time": 0.0,
            "barometric_pressure": 0.0,
            "intake_manifold_pressure": float("nan"),
            "intake_manifold_temperature": float("nan"),
            "cylinder_head_temperature": float("nan"),
            "ignition_timing": 0.0,
            "injection_time": float("nan"),
            "exhaust_gas_temperature": float("nan"),
            "throttle_out": 0.0,
            "pt_compensation": 0.0,
            "ignition_voltage": 0,  # As per the spec, 0 means "Unknown".
        }, epsilon=1.0)
        self.assert_received_message_field_values('GENERATOR_STATUS', {
            "status": 1,
            "generator_speed": 0,
            "battery_current": -0.30000001192092896,
            "load_current": 10.119999885559082,
            "power_generated": 521.0,
            "bus_voltage": 50,
            "rectifier_temperature": 32767,
            "bat_current_setpoint": float("nan"),
            "generator_temperature": 32767,
            "runtime": 0,
            "time_until_maintenance": 300*60*60,
        })

        self.context_collect('STATUSTEXT')

        # check prearms - should bounce due to generator not in correct state
        self.try_arm(result=False, expect_msg="requested state is not RUN")

        #######################################################################
        # Generator IDLE subtest.
        #######################################################################
        self.start_subtest("Setting generator to IDLE state.")
        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_statustext("Generator MIDDLE", check_context=True)
        self.delay_sim_time(2)
        self.drain_mav()
        self.assert_received_message_field_values('EFI_STATUS', {
            "intake_manifold_pressure": 94,
            "exhaust_gas_temperature": float("nan"),
            "ignition_voltage": 12,
            "throttle_position": 40,
        }, epsilon=1.0)

        self.wait_message_field_values('EFI_STATUS', {
            "cylinder_head_temperature": 20,
        }, epsilon=5.0, timeout=10)

        self.assert_received_message_field_values('GENERATOR_STATUS', {
            # Commenting out "status" argument.
            # For some reason the test suite can't accept an integer value. Says it's not part of the FLAG.
            # "status": mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE,
            "battery_current": -0.30000001192092896,
            "load_current": 10.119999885559082,
            "power_generated": 521.0,
            "bus_voltage": 50,
            "rectifier_temperature": 32767,
            "bat_current_setpoint": float("nan"),
            "generator_temperature": 32767,
            "runtime": 2,
            "time_until_maintenance": 300*60*60 - 2,
        })
        self.wait_generator_speed_and_state(2000, 3000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        #######################################################################
        # Generator RUN subtest.
        #######################################################################
        self.start_subtest("Setting generator to RUN state.")
        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run
        self.wait_statustext("Generator HIGH", check_context=True)

        # check prearms - should bounce due to generator too cold
        self.try_arm(result=False, expect_msg="Generator warming up")

        self.set_rc(gen_ctrl_ch, 1000) # remember this is a switch position - stop
        self.wait_statustext("requested state is not RUN", timeout=200)

        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(2000, 3000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run

        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==1",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        if bs is None:
            raise NotAchievedException("Did not receive BATTERY_STATUS")
        self.progress("Received battery status: %s" % str(bs))
        want_bs_volt = 50000
        if bs.voltages[0] != want_bs_volt:
            raise NotAchievedException("Battery voltage not as expected (want=%f) got=(%f)" % (want_bs_volt, bs.voltages[0],))

        self.progress("Checking battery remaining")
        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==2",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        self.progress("Waiting for some fuel to be consumed...")
        self.wait_message_field_values("BATTERY_STATUS", {
            "id": 2,
            "battery_remaining": bs.battery_remaining-1,
        }, timeout=100)

        bs2 = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==2",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        if bs2.battery_remaining >= bs.battery_remaining:
            raise NotAchievedException("Expected battery remaining to drop")
        if bs2.current_consumed <= bs.current_consumed:
            raise NotAchievedException("Expected energy consumed to rise")

        self.progress("Checking battery reset")
        batt_reset_values = [(25, 24),
                             (50, 49),
                             (63, 62),
                             (87, 86),
                             (100, 99)]

        for (reset_val, return_val) in batt_reset_values:
            self.run_cmd(mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                         (1 << 2), # param1 - bitmask of batteries to reset
                         reset_val, # level to reset to
                         0, # param3
                         0, # param4
                         0, # param5
                         0, # param6
                         0 # param7
                         )
            self.wait_message_field_values("BATTERY_STATUS", {
                "id": 2,
                "battery_remaining": return_val,
            }, timeout=5)

        self.progress("Moving *back* to idle")
        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 10000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.progress("Moving *back* to run")
        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        self.set_parameter("LOG_DISARMED", 0)
        if not self.current_onboard_log_contains_message("LOEG"):
            raise NotAchievedException("Did not find expected LOEG message in .bin log")
        if not self.current_onboard_log_contains_message("LOEC"):
            raise NotAchievedException("Did not find expected LOEC message in .bin log")

        self.set_parameter("LOG_DISARMED", 1)  # Re-start logging, to help with debugging.
        self.progress("Stopping generator")
        self.set_rc(gen_ctrl_ch, 1000)

        self.wait_statustext("Cooling down")

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        #######################################################################
        # E-stop subtest.
        #######################################################################
        self.start_subtest("Checking safety switch estop")
        self.set_rc(gen_ctrl_ch, 2000)
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        self.set_safetyswitch_on()
        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 0,
            "rpm": 0,
        })

        self.set_safetyswitch_off()

        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 80,  # Throttle set by governor.
            "rpm": 8000,  # RPM for 50% throttle
        }, timeout=20)

        # stop generator:
        self.set_rc(gen_ctrl_ch, 1000)
        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        #######################################################################
        # Battery failsafe subtest.
        #######################################################################
        self.start_subtest("Battery Failsafes")
        self.context_push()
        batt3_capacity = 500
        batt3_low_mah = 100
        batt3_low_consumed_mah = batt3_capacity - batt3_low_mah
        self.set_parameters({
            "BATT3_CAPACITY": batt3_capacity,
            "BATT3_LOW_MAH": batt3_low_mah,
            "BATT3_CRT_MAH": 50,
            "BATT3_FS_LOW_ACT": 2, # RTL
            "BATT3_FS_CRT_ACT": 1, # LAND
            "BATT3_LOW_VOLT": 0,
        })
        self.reboot_sitl()
        self.set_rc(gen_ctrl_ch, 2000)
        self.takeoff(10, mode='GUIDED')

        first_efi_status = self.assert_receive_message('EFI_STATUS', verbose=True)
        if first_efi_status.fuel_consumed < 100:  # takes about this much to get going
            raise NotAchievedException("Unexpected fuel consumed value after takeoff (%f)" % first_efi_status.fuel_consumed)

        self.fly_guided_move_local(100, 100, 20)

        self.wait_mode('RTL', timeout=300)

        second_efi_status = self.assert_receive_message('EFI_STATUS', verbose=True)

        if second_efi_status.fuel_consumed < batt3_low_consumed_mah:
            raise NotAchievedException("Unexpected fuel consumed value after failsafe (%f)" % second_efi_status.fuel_consumed)

        self.wait_mode('LAND', timeout=300)
        self.wait_disarmed()

        self.context_pop()
        self.reboot_sitl()

        self.set_message_rate_hz("EFI_STATUS", -1)
        self.set_message_rate_hz("GENERATOR_STATUS", -1)

    def LoweheiserManual(self):
        '''Ensure the Loweheiser generator works as expected in manual starter mode.'''

        gen_ctrl_ch = 9
        loweheiser_man_throt_ch = 10
        loweheiser_man_start_ch = 11

        self.set_parameters({
            "SERIAL5_PROTOCOL": 2,    # mavlink
            "MAV3_OPTIONS": 2,        # private
            "GEN_TYPE": 4,            # loweheiser
            "EFI_TYPE": 4,            # loweheiser
            "SIM_EFI_TYPE": 2,        # loweheiser sim
            "BATT2_MONITOR": 17,  # generator (elec)
            "BATT3_MONITOR": 18,  # generator (fuel-level)
            "BATT3_CAPACITY": 10000,  # generator (fuel) in mL
            f"RC{gen_ctrl_ch}_OPTION": 85,  # generator control
            "LOG_DISARMED": 1,
            "RC%u_OPTION" % loweheiser_man_throt_ch: 218,  # loweheiser manual throttle control
            "RC%u_DZ" % loweheiser_man_throt_ch: 20,
            "RC%u_TRIM" % loweheiser_man_throt_ch: 1000,
            "RC%u_MIN" % loweheiser_man_throt_ch: 1000,
            "RC%u_MAX" % loweheiser_man_throt_ch: 2000,
            "RC%u_OPTION" % loweheiser_man_start_ch: 111, # loweheiser starter channel
        })

        self.set_rc_from_map({
            gen_ctrl_ch: 1000,   # remember this is a switch position - stop
            loweheiser_man_throt_ch: 1000,  # zero throttle
            loweheiser_man_start_ch: 1000,  # the starter is off
        })

        self.customise_SITL_commandline(["--serial5=sim:loweheiser"])

        self.set_parameters({
            "GEN_L_IDLE_TH": 25,
            "GEN_L_IDLE_TH_H": 40,
            "GEN_L_RUN_TEMP": 60,
            "GEN_L_IDLE_TEMP": 80,
        })

        self.reboot_sitl()

        self.assert_parameter_value("GEN_L_IDLE_TH", 25)

        self.delay_sim_time(10)  # so we can actually receive messages...

        #######################################################################
        # Generator OFF subtest.
        #######################################################################
        self.start_subtest("Checking GENERATOR_STATUS while OFF.")
        self.set_message_rate_hz("GENERATOR_STATUS", 10)
        self.delay_sim_time(1)

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        self.start_subtest("Checking EFI_STATUS while OFF.")
        self.set_message_rate_hz("EFI_STATUS", 10)
        self.delay_sim_time(1)
        self.assert_received_message_field_values('EFI_STATUS', {
            "health": 1,
            "ecu_index": 1.0,
            "rpm": 0.0,
            "fuel_consumed": 0,
            "fuel_flow": float("nan"),
            "engine_load": 0.0,
            "throttle_position": 0.0,
            "spark_dwell_time": 0.0,
            "barometric_pressure": 0.0,
            "intake_manifold_pressure": float("nan"),
            "intake_manifold_temperature": float("nan"),
            "cylinder_head_temperature": float("nan"),
            "ignition_timing": 0.0,
            "injection_time": float("nan"),
            "exhaust_gas_temperature": float("nan"),
            "throttle_out": 0.0,
            "pt_compensation": 0.0,
            "ignition_voltage": 0,  # As per the spec, 0 means "Unknown".
        }, epsilon=1.0)
        self.assert_received_message_field_values('GENERATOR_STATUS', {
            "status": 1,
            "generator_speed": 0,
            "battery_current": -0.30000001192092896,
            "load_current": 10.119999885559082,
            "power_generated": 521.0,
            "bus_voltage": 50,
            "rectifier_temperature": 32767,
            "bat_current_setpoint": float("nan"),
            "generator_temperature": 32767,
            "runtime": 0,
            "time_until_maintenance": 300*60*60,
        })

        rc_dz = self.get_parameter('RC%u_DZ' % loweheiser_man_throt_ch)
        rc_trim = int(self.get_parameter('RC%u_TRIM' % loweheiser_man_throt_ch))
        rc_min = self.get_parameter('RC%u_MIN' % loweheiser_man_throt_ch)
        rc_max = int(self.get_parameter('RC%u_MAX' % loweheiser_man_throt_ch))

        self.progress("Ensuring 1000 and rc_trim all leave throttle out 0")
        for i in 1000, rc_trim:
            self.progress("Checking %u pwm" % i)
            self.set_rc(loweheiser_man_throt_ch, i)
            self.delay_sim_time(1)
            self.assert_received_message_field_values('EFI_STATUS', {
                "throttle_position": 0,
                "rpm": 0,
            })

        self.context_collect('STATUSTEXT')
        # check prearms - should bounce due to generator not in correct state
        self.try_arm(result=False, expect_msg="requested state is not RUN")

        #######################################################################
        # Generator IDLE subtest.
        #######################################################################
        self.start_subtest("Setting generator to IDLE state.")
        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_statustext("Generator MIDDLE", check_context=True)
        self.delay_sim_time(5)

        # Ensure the generator has not auto-started.
        self.assert_received_message_field_values('EFI_STATUS', {
            "throttle_out": 0,
            "rpm": 0,
        })

        # at 50 percent throttle should not start until user triggers starter
        pwm_for_fifty_percent_throttle = int(rc_min + rc_dz + int((rc_max-rc_min-rc_dz)/2))
        self.progress("Using PWM of %u for 50 percent throttle" % pwm_for_fifty_percent_throttle)
        self.set_rc(loweheiser_man_throt_ch, pwm_for_fifty_percent_throttle)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 51,  # magic fixed throttle value from AP_Generator_Loweheiser.cpp
            "rpm": 0,
        }, timeout=20)

        self.set_rc(loweheiser_man_start_ch, 2000)  # Run the starter for a few seconds.

        self.drain_mav()
        self.assert_received_message_field_values('EFI_STATUS', {
            "intake_manifold_pressure": 94,
            "exhaust_gas_temperature": float("nan"),
            "ignition_voltage": 12,
        }, epsilon=1.0)

        self.wait_message_field_values('EFI_STATUS', {
            "cylinder_head_temperature": 20,
        }, epsilon=5.0, timeout=10)

        self.set_rc(loweheiser_man_start_ch, 1000)

        self.assert_received_message_field_values('GENERATOR_STATUS', {
            # Commenting out "status" argument.
            # For some reason the test suite can't accept an integer value. Says it's not part of the FLAG.
            # "status": mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE,
            "battery_current": -0.30000001192092896,
            "load_current": 10.119999885559082,
            "power_generated": 521.0,
            "bus_voltage": 50,
            "rectifier_temperature": 32767,
            "bat_current_setpoint": float("nan"),
            "generator_temperature": 32767,
            "runtime": 2,
            "time_until_maintenance": 300*60*60 - 2,
        })

        self.progress("Generator at idle should not run governor and use throttle input")
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 51,  # magic fixed throttle value from AP_Generator_Loweheiser.cpp
            "rpm": 4080,  # RPM for 50% throttle
        }, timeout=20)

        #######################################################################
        # Generator RUN subtest.
        #######################################################################
        self.start_subtest("Setting generator to RUN state.")
        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run
        self.wait_statustext("Generator HIGH", check_context=True)

        # check prearms - should bounce due to generator too cold
        self.try_arm(result=False, expect_msg="Generator warming up")

        self.set_rc(gen_ctrl_ch, 1000) # remember this is a switch position - stop
        self.wait_statustext("requested state is not RUN", timeout=200)

        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.set_rc(loweheiser_man_start_ch, 2000)
        self.wait_generator_speed_and_state(2000, 3000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)
        self.set_rc(loweheiser_man_start_ch, 1000)

        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run

        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==1",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        if bs is None:
            raise NotAchievedException("Did not receive BATTERY_STATUS")
        self.progress("Received battery status: %s" % str(bs))
        want_bs_volt = 50000
        if bs.voltages[0] != want_bs_volt:
            raise NotAchievedException("Battery voltage not as expected (want=%f) got=(%f)" % (want_bs_volt, bs.voltages[0],))

        self.progress("Checking battery remaining")
        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==2",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        self.progress("Waiting for some fuel to be consumed...")
        self.wait_message_field_values("BATTERY_STATUS", {
            "id": 2,
            "battery_remaining": bs.battery_remaining-1,
        }, timeout=100)

        bs2 = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==2",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        if bs2.battery_remaining >= bs.battery_remaining:
            raise NotAchievedException("Expected battery remaining to drop")
        if bs2.current_consumed <= bs.current_consumed:
            raise NotAchievedException("Expected energy consumed to rise")

        self.progress("Checking battery reset")
        batt_reset_values = [(25, 24),
                             (50, 49),
                             (63, 62),
                             (87, 86),
                             (100, 99)]

        for (reset_val, return_val) in batt_reset_values:
            self.run_cmd(mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                         (1 << 2), # param1 - bitmask of batteries to reset
                         reset_val, # level to reset to
                         0, # param3
                         0, # param4
                         0, # param5
                         0, # param6
                         0 # param7
                         )
            self.wait_message_field_values("BATTERY_STATUS", {
                "id": 2,
                "battery_remaining": return_val,
            }, timeout=5)

        self.progress("Moving *back* to idle")
        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 10000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.progress("Moving *back* to run")
        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        self.set_parameter("LOG_DISARMED", 0)
        if not self.current_onboard_log_contains_message("LOEG"):
            raise NotAchievedException("Did not find expected LOEG message in .bin log")
        if not self.current_onboard_log_contains_message("LOEC"):
            raise NotAchievedException("Did not find expected LOEC message in .bin log")

        self.set_parameter("LOG_DISARMED", 1)  # Re-start logging, to help with debugging.
        self.progress("Stopping generator")
        self.set_rc(gen_ctrl_ch, 1000)

        self.wait_statustext("Cooling down")

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        #######################################################################
        # E-stop subtest.
        #######################################################################
        self.start_subtest("Checking safety switch estop")
        # Start the engine.
        self.set_rc(gen_ctrl_ch, 1500)
        self.set_rc(loweheiser_man_start_ch, 2000)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "rpm": 4000,  # RPM for 50% throttle
        }, timeout=20)
        self.set_rc(loweheiser_man_start_ch, 1000)

        self.set_safetyswitch_on()
        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 0,
            "rpm": 0,
        })

        self.set_safetyswitch_off()

        self.set_rc(gen_ctrl_ch, 1500)
        self.set_rc(loweheiser_man_start_ch, 2000)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 51,  # magic fixed throttle value from AP_Generator_Loweheiser.cpp
            "rpm": 4000,  # RPM for 50% throttle
        }, timeout=20)
        self.set_rc(loweheiser_man_start_ch, 1000)

        # stop generator:
        self.set_rc(gen_ctrl_ch, 1000)
        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        #######################################################################
        # Battery failsafe subtest.
        #######################################################################
        self.start_subtest("Battery Failsafes")
        self.context_push()
        batt3_capacity = 500
        batt3_low_mah = 100
        batt3_low_consumed_mah = batt3_capacity - batt3_low_mah
        self.set_parameters({
            "BATT3_CAPACITY": batt3_capacity,
            "BATT3_LOW_MAH": batt3_low_mah,
            "BATT3_CRT_MAH": 50,
            "BATT3_FS_LOW_ACT": 2, # RTL
            "BATT3_FS_CRT_ACT": 1, # LAND
            "BATT3_LOW_VOLT": 0,
        })
        self.reboot_sitl()

        # Start the generator anew.
        self.set_rc(gen_ctrl_ch, 1500)
        self.set_rc(loweheiser_man_start_ch, 2000)
        self.delay_sim_time(5)
        self.set_rc(loweheiser_man_start_ch, 1000)
        self.set_rc(gen_ctrl_ch, 2000)
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)
        self.takeoff(10, mode='GUIDED')

        first_efi_status = self.assert_receive_message('EFI_STATUS', verbose=True)
        if first_efi_status.fuel_consumed < 100:  # takes about this much to get going
            raise NotAchievedException("Unexpected fuel consumed value after takeoff (%f)" % first_efi_status.fuel_consumed)

        self.fly_guided_move_local(100, 100, 20)

        self.wait_mode('RTL', timeout=300)

        second_efi_status = self.assert_receive_message('EFI_STATUS', verbose=True)

        if second_efi_status.fuel_consumed < batt3_low_consumed_mah:
            raise NotAchievedException("Unexpected fuel consumed value after failsafe (%f)" % second_efi_status.fuel_consumed)

        self.wait_mode('LAND', timeout=300)
        self.wait_disarmed()

        self.context_pop()
        self.reboot_sitl()

        self.set_message_rate_hz("EFI_STATUS", -1)
        self.set_message_rate_hz("GENERATOR_STATUS", -1)

    def AuxSwitchOptions(self):
        '''Test random aux mode options'''
        self.set_parameter("RC7_OPTION", 58) # clear waypoints
        self.load_mission("copter_loiter_to_alt.txt")
        self.set_rc(7, 1000)
        self.assert_mission_count(5)
        self.progress("Clear mission")
        self.set_rc(7, 2000)
        self.delay_sim_time(1) # allow switch to debounce
        self.assert_mission_count(0)
        self.set_rc(7, 1000)
        self.set_parameter("RC7_OPTION", 24) # reset mission
        self.delay_sim_time(2)
        self.load_mission("copter_loiter_to_alt.txt")
        set_wp = 4
        self.set_current_waypoint(set_wp)
        self.wait_current_waypoint(set_wp, timeout=10)
        self.progress("Reset mission")
        self.set_rc(7, 2000)
        self.delay_sim_time(1)
        self.wait_current_waypoint(0, timeout=10)
        self.set_rc(7, 1000)

    def AuxFunctionsInMission(self):
        '''Test use of auxiliary functions in missions'''
        self.load_mission("aux_functions.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1500)
        self.wait_mode('ALT_HOLD')
        self.change_mode('AUTO')
        self.wait_rtl_complete()

    def MAV_CMD_AIRFRAME_CONFIGURATION(self):
        '''deploy/retract landing gear using mavlink command'''
        self.context_push()
        self.set_parameters({
            "LGR_ENABLE": 1,
            "SERVO10_FUNCTION": 29,
            "SERVO10_MIN": 1001,
            "SERVO10_MAX": 1999,
        })
        self.reboot_sitl()

        # starts loose:
        self.wait_servo_channel_value(10, 0)

        # 0 is down:
        self.start_subtest("Put gear down")
        self.run_cmd(mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, p2=0)
        self.wait_servo_channel_value(10, 1999)

        # 1 is up:
        self.start_subtest("Put gear up")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, p2=1)
        self.wait_servo_channel_value(10, 1001)

        # 0 is down:
        self.start_subtest("Put gear down")
        self.run_cmd(mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, p2=0)
        self.wait_servo_channel_value(10, 1999)

        self.context_pop()
        self.reboot_sitl()

    def WatchAlts(self):
        '''Ensure we can monitor different altitudes'''
        self.takeoff(30, mode='GUIDED')
        self.delay_sim_time(5, reason='let altitude settle')

        self.progress("Testing absolute altitudes")
        absolute_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
        self.progress("absolute_alt=%f" % absolute_alt)
        epsilon = 4  # SIM_STATE and vehicle state can be off by a bit...
        for source in ['GLOBAL_POSITION_INT.alt', 'SIM_STATE.alt', 'GPS_RAW_INT.alt']:
            self.watch_altitude_maintained(
                absolute_alt-epsilon,
                absolute_alt+epsilon,
                altitude_source=source
            )

        self.progress("Testing absolute altitudes")
        relative_alt = self.get_altitude(relative=True)
        for source in ['GLOBAL_POSITION_INT.relative_alt']:
            self.watch_altitude_maintained(
                relative_alt-epsilon,
                relative_alt+epsilon,
                altitude_source=source
            )

        self.do_RTL()

    def TestTetherStuck(self):
        """Test tethered vehicle stuck because of tether"""
        # Enable tether simulation
        self.set_parameters({
            "SIM_TETH_ENABLE": 1,
        })
        self.delay_sim_time(2)
        self.reboot_sitl()

        # Set tether line length
        self.set_parameters({
            "SIM_TETH_LINELEN": 10,
        })
        self.delay_sim_time(2)

        # Prepare and take off
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff(10, mode='LOITER')

        # Simulate vehicle getting stuck by increasing RC throttle
        self.set_rc(3, 1900)
        self.delay_sim_time(10, reason='let tether get stuck')

        # Monitor behavior for 10 seconds
        tstart = self.get_sim_time()
        initial_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
        self.progress(f"initial_alt={initial_alt}")
        stuck = True  # Assume it's stuck unless proven otherwise

        while self.get_sim_time() - tstart < 10:
            # Fetch current altitude
            current_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
            self.progress(f"current_alt={current_alt}")

            # Fetch and log battery status
            battery_status = self.assert_receive_message('BATTERY_STATUS')
            if battery_status:
                self.progress(f"Battery: {battery_status}")

            # Check if the vehicle is stuck.
            # We assume the vehicle is stuck if the current is high and the altitude is not changing
            if battery_status and (battery_status.current_battery < 6500 or abs(current_alt - initial_alt) > 3):
                stuck = False  # Vehicle moved or current is abnormal
                break

        if not stuck:
            raise NotAchievedException("Vehicle did not get stuck as expected")

        # Land and disarm the vehicle to ensure we can go down
        self.land_and_disarm()

    def fly_rangefinder_drivers_fly(self, rangefinders):
        '''ensure rangefinder gives height-above-ground'''
        expected_alt = 5
        self.takeoff(expected_alt, mode='GUIDED', alt_minimum_duration=5)
        ds = self.assert_receive_message("DISTANCE_SENSOR")
        gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
        delta = abs(ds.current_distance*0.01 - gpi.relative_alt/1000.0)
        alt_msg = f"{ds.current_distance*0.01=} disagrees with global-position-int.relative_alt ({gpi.relative_alt/1000.0}) by {delta}m"  # NOQA:E501
        self.progress(alt_msg)
        self.progress(f"Terrain report: {self.mav.messages['TERRAIN_REPORT']}")
        if delta > 1:
            raise NotAchievedException(alt_msg)

        for i in range(0, len(rangefinders)):
            name = rangefinders[i]
            self.progress("i=%u (%s)" % (i, name))
            ds = self.assert_receive_message(
                "DISTANCE_SENSOR",
                timeout=2,
                condition=f"DISTANCE_SENSOR.id=={i}",
                verbose=True,
            )
            if abs(ds.current_distance/100.0 - gpi.relative_alt/1000.0) > 1:
                raise NotAchievedException(
                    "distance sensor.current_distance (%f) (%s) disagrees with global-position-int.relative_alt (%s)" %
                    (ds.current_distance/100.0, name, gpi.relative_alt/1000.0))

        self.land_and_disarm()

        self.progress("Ensure RFND messages in log")
        if not self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("No RFND messages in log")

    def MAVProximity(self):
        '''Test MAVLink proximity driver'''
        self.start_subtest("Test mavlink proximity sensor using DISTANCE_SENSOR messages")  # noqa
        self.set_parameter("SERIAL5_PROTOCOL", 1)
        self.set_parameter("PRX1_TYPE", 2)  # mavlink
        self.reboot_sitl()

        self.progress("Should be unhealthy while we don't send messages")
        self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY, True, True, False)

        self.progress("Should be healthy while we're sending good messages")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise NotAchievedException("Sensor did not come good")
            self.mav.mav.distance_sensor_send(
                0,  # time_boot_ms
                10, # min_distance cm
                50, # max_distance cm
                20, # current_distance cm
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                21, # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, # orientation
                255  # covariance
            )
            if self.sensor_has_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY, True, True, True):
                self.progress("Sensor has good state")
                break
            self.delay_sim_time(0.1)

        self.progress("Should be unhealthy again if we stop sending messages")
        self.delay_sim_time(1)
        self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY, True, True, False)

        # now make sure we get echoed back the same sorts of things we send:
        # distances are in cm
        distance_map = {
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 30,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 35,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 20,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 15,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 70,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 80,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 10,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 90,
        }

        wanted_distances = copy.copy(distance_map)
        sensor_enum = mavutil.mavlink.enums["MAV_SENSOR_ORIENTATION"]

        def my_message_hook(mav, m):
            if m.get_type() != 'DISTANCE_SENSOR':
                return
            self.progress("Got (%s)" % str(m))
            want = distance_map[m.orientation]
            got = m.current_distance
            # ArduPilot's floating point conversions make it imprecise:
            delta = abs(want-got)
            if delta > 1:
                self.progress(
                    "Wrong distance (%s): want=%f got=%f" %
                    (sensor_enum[m.orientation].name, want, got))
                return
            if m.orientation not in wanted_distances:
                return
            self.progress(
                "Correct distance (%s): want=%f got=%f" %
                (sensor_enum[m.orientation].name, want, got))
            del wanted_distances[m.orientation]

        self.install_message_hook_context(my_message_hook)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise NotAchievedException("Sensor did not give right distances")  # noqa
            for (orient, dist) in distance_map.items():
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    10, # min_distance cm
                    90, # max_distance cm
                    dist, # current_distance cm
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    orient, # orientation
                    255  # covariance
                )
            self.wait_heartbeat()
            if len(wanted_distances.keys()) == 0:
                break

    def fly_rangefinder_mavlink_distance_sensor(self):
        self.start_subtest("Test mavlink rangefinder using DISTANCE_SENSOR messages")
        self.context_push()
        self.set_parameters({
            "RTL_ALT_TYPE": 0,
            "LGR_ENABLE": 1,
            "LGR_DEPLOY_ALT": 1,
            "LGR_RETRACT_ALT": 10, # metres
            "SERVO10_FUNCTION": 29
        })
        ex = None
        try:
            self.set_parameter("SERIAL5_PROTOCOL", 1)
            self.set_parameter("RNGFND1_TYPE", 10)
            self.reboot_sitl()
            self.set_parameter("RNGFND1_MAX", 327.67)

            self.progress("Should be unhealthy while we don't send messages")
            self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, True, False)

            self.progress("Should be healthy while we're sending good messages")
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 5:
                    raise NotAchievedException("Sensor did not come good")
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    10, # min_distance
                    50, # max_distance
                    20, # current_distance
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255 # covariance
                )
                if self.sensor_has_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, True, True):
                    self.progress("Sensor has good state")
                    break
                self.delay_sim_time(0.1)

            self.progress("Should be unhealthy again if we stop sending messages")
            self.delay_sim_time(1)
            self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, True, False)

            self.progress("Landing gear should deploy with current_distance below min_distance")
            self.change_mode('STABILIZE')
            timeout = 60
            tstart = self.get_sim_time()
            while not self.armed():
                if self.get_sim_time() - tstart > timeout:
                    raise NotAchievedException("Failed to become armable after %f seconds" % timeout)
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    100, # min_distance (cm)
                    2500, # max_distance (cm)
                    200, # current_distance (cm)
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255  # covariance
                )
                try:
                    self.arm_vehicle()
                except Exception:
                    pass
            self.delay_sim_time(1)  # servo function maps only periodically updated
#            self.send_debug_trap()

            self.run_cmd(
                mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION,
                p2=0,  # deploy
            )

            self.context_collect("STATUSTEXT")
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > 5:
                    raise NotAchievedException("Retraction did not happen")
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    100, # min_distance (cm)
                    6000, # max_distance (cm)
                    1500, # current_distance (cm)
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255  # covariance
                )
                self.delay_sim_time(0.1)
                try:
                    self.wait_text("LandingGear: RETRACT", check_context=True, timeout=0.1)
                except Exception:
                    continue
                self.progress("Retracted")
                break
#            self.send_debug_trap()
            while True:
                if self.get_sim_time_cached() - tstart > 5:
                    raise NotAchievedException("Deployment did not happen")
                self.progress("Sending distance-sensor message")
                self.mav.mav.distance_sensor_send(
                    0, # time_boot_ms
                    300, # min_distance
                    500, # max_distance
                    250, # current_distance
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255 # covariance
                )
                try:
                    self.wait_text("LandingGear: DEPLOY", check_context=True, timeout=0.1)
                except Exception:
                    continue
                self.progress("Deployed")
                break
            self.disarm_vehicle()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def GSF(self):
        '''test the Gaussian Sum filter'''
        self.context_push()
        self.set_parameter("EK2_ENABLE", 1)
        self.reboot_sitl()
        self.takeoff(20, mode='LOITER')
        self.set_rc(2, 1400)
        self.delay_sim_time(5)
        self.set_rc(2, 1500)
        self.progress("Path: %s" % self.current_onboard_log_filepath())
        dfreader = self.dfreader_for_current_onboard_log()
        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

        # ensure log messages present
        want = set(["XKY0", "XKY1", "NKY0", "NKY1"])
        still_want = want
        while len(still_want):
            m = dfreader.recv_match(type=want)
            if m is None:
                raise NotAchievedException("Did not get %s" % want)
            still_want.remove(m.get_type())

    def GSF_reset(self):
        '''test the Gaussian Sum filter based Emergency reset'''
        self.context_push()
        self.set_parameters({
            "COMPASS_ORIENT": 4,    # yaw 180
            "COMPASS_USE2": 0,      # disable backup compasses to avoid pre-arm failures
            "COMPASS_USE3": 0,
        })
        self.reboot_sitl()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        # record starting position
        startpos = self.assert_receive_message('LOCAL_POSITION_NED')
        self.progress("startpos=%s" % str(startpos))

        # arm vehicle and takeoff to at least 5m
        self.arm_vehicle()
        expected_alt = 5
        self.user_takeoff(alt_min=expected_alt)

        # watch for emergency yaw reset
        self.wait_text("EKF3 IMU. emergency yaw reset", timeout=5, regex=True)

        # record how far vehicle flew off
        endpos = self.assert_receive_message('LOCAL_POSITION_NED')
        delta_x = endpos.x - startpos.x
        delta_y = endpos.y - startpos.y
        dist_m = math.sqrt(delta_x*delta_x + delta_y*delta_y)
        self.progress("GSF yaw reset triggered at %f meters" % dist_m)

        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

        # ensure vehicle did not fly too far
        dist_m_max = 8
        if dist_m > dist_m_max:
            raise NotAchievedException("GSF reset failed, vehicle flew too far (%f > %f)" % (dist_m, dist_m_max))

    def FlyRangeFinderMAVlink(self):
        '''fly mavlink-connected rangefinder'''
        self.fly_rangefinder_mavlink_distance_sensor()

        # explicit test for the mavlink driver as it doesn't play so nice:
        self.set_parameters({
            "SERIAL5_PROTOCOL": 1,
            "RNGFND1_TYPE": 10,
            "RTL_ALT": 500,
            "RTL_ALT_TYPE": 1,
            "SIM_TERRAIN": 0,
        })
        self.customise_SITL_commandline(['--serial5=sim:rf_mavlink'])

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        expected_alt = 5
        self.user_takeoff(alt_min=expected_alt)

        self.context_push()
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise NotAchievedException("Mavlink rangefinder not working")
            rf = self.assert_receive_message("RANGEFINDER")
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            if abs(rf.distance - gpi.relative_alt/1000.0) > 1:
                self.progress("rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                              (rf.distance, gpi.relative_alt/1000.0))
                continue

            ds = self.assert_receive_message("DISTANCE_SENSOR", timeout=2, verbose=True)
            if abs(ds.current_distance/100.0 - gpi.relative_alt/1000.0) > 1:
                self.progress(
                    "distance sensor.current_distance (%f) disagrees with global-position-int.relative_alt (%s)" %
                    (ds.current_distance/100.0, gpi.relative_alt/1000.0))
                continue
            break

        self.context_pop()

        self.progress("mavlink rangefinder OK")
        self.land_and_disarm()

    def MaxBotixI2CXL(self):
        '''Test maxbotix rangefinder drivers'''
        ex = None
        try:
            self.context_push()

            self.start_subtest("No messages")
            self.assert_not_receiving_message("DISTANCE_SENSOR", timeout=5)

            self.start_subtest("Default address")
            self.set_parameter("RNGFND1_TYPE", 2)  # maxbotix
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            self.assert_receive_message("DISTANCE_SENSOR", timeout=5, verbose=True)

            self.start_subtest("Explicitly set to default address")
            self.set_parameters({
                "RNGFND1_TYPE": 2,  # maxbotix
                "RNGFND1_ADDR": 0x70,
            })
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            rf = self.assert_receive_message("DISTANCE_SENSOR", timeout=5, verbose=True)

            self.start_subtest("Explicitly set to non-default address")
            self.set_parameter("RNGFND1_ADDR", 0x71)
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            rf = self.assert_receive_message("DISTANCE_SENSOR", timeout=5, verbose=True)

            self.start_subtest("Two MaxBotix RangeFinders")
            self.set_parameters({
                "RNGFND1_TYPE": 2,  # maxbotix
                "RNGFND1_ADDR": 0x70,
                "RNGFND1_MIN": 1.50,
                "RNGFND2_TYPE": 2,  # maxbotix
                "RNGFND2_ADDR": 0x71,
                "RNGFND2_MIN": 2.50,
            })
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            for i in [0, 1]:
                rf = self.assert_receive_message(
                    "DISTANCE_SENSOR",
                    timeout=5,
                    condition=f"DISTANCE_SENSOR.id=={i}",
                    verbose=True,
                )
                expected_dist = 150
                if i == 1:
                    expected_dist = 250
                if rf.min_distance != expected_dist:
                    raise NotAchievedException("Unexpected min_cm (want=%u got=%u)" %
                                               (expected_dist, rf.min_distance))

            self.context_pop()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.reboot_sitl()
        if ex is not None:
            raise ex

    def FlyRangeFinderSITL(self):
        '''fly the type=100 perfect rangefinder'''
        self.set_parameters({
            "RNGFND1_TYPE": 100,
            "RTL_ALT": 500,
            "RTL_ALT_TYPE": 1,
            "SIM_TERRAIN": 0,
        })
        self.reboot_sitl()
        self.fly_rangefinder_drivers_fly([("unused", "sitl")])
        self.wait_disarmed()

    def RangeFinderDrivers(self):
        '''Test rangefinder drivers'''
        self.set_parameters({
            "RTL_ALT": 500,
            "RTL_ALT_TYPE": 1,
            "SIM_TERRAIN": 0,
        })
        drivers = [
            ("lightwareserial", 8),  # autodetected between this and -binary
            ("lightwareserial-binary", 8),
            ("USD1_v0", 11),
            ("USD1_v1", 11),
            ("leddarone", 12),
            ("maxsonarseriallv", 13),
            ("nmea", 17, {"baud": 9600}),
            ("wasp", 18),
            ("benewake_tf02", 19),
            ("blping", 23),
            ("benewake_tfmini", 20),
            ("lanbao", 26),
            ("benewake_tf03", 27),
            ("gyus42v2", 31),
            ("teraranger_serial", 35),
            ("nooploop_tofsense", 37),
            ("ainsteinlrd1", 42),
            ("rds02uf", 43),
            ("lightware_grf", 45),
        ]
        # you can use terrain - if you don't the vehicle just uses a
        # plane based on home.
        # self.install_terrain_handlers_context()
        while len(drivers):
            do_drivers = drivers[0:3]
            drivers = drivers[3:]
            command_line_args = []
            self.context_push()
            for offs in range(3):
                serial_num = offs + 4
                if len(do_drivers) > offs:
                    if len(do_drivers[offs]) > 2:
                        (sim_name, rngfnd_param_value, kwargs) = do_drivers[offs]
                    else:
                        (sim_name, rngfnd_param_value) = do_drivers[offs]
                        kwargs = {}
                    command_line_args.append("--serial%s=sim:%s" %
                                             (serial_num, sim_name))
                    sets = {
                        "SERIAL%u_PROTOCOL" % serial_num: 9, # rangefinder
                        "RNGFND%u_TYPE" % (offs+1): rngfnd_param_value,
                    }
                    if "baud" in kwargs:
                        sets["SERIAL%u_BAUD" % serial_num] = kwargs["baud"]
                    self.set_parameters(sets)
            self.customise_SITL_commandline(command_line_args)
            self.fly_rangefinder_drivers_fly([x[0] for x in do_drivers])
            self.context_pop()

        class I2CDriverToTest:
            def __init__(self, name, rngfnd_type, rngfnd_addr=None):
                self.name = name
                self.rngfnd_type = rngfnd_type
                self.rngfnd_addr = rngfnd_addr

        i2c_drivers = [
            I2CDriverToTest("maxbotixi2cxl", 2),
            I2CDriverToTest("terarangeri2c", 14, rngfnd_addr=0x31),
            I2CDriverToTest("lightware_legacy16bit", 7, rngfnd_addr=0x66),
            I2CDriverToTest("tfs20l", 46, rngfnd_addr=0x10),
            I2CDriverToTest("TFMiniPlus", 25, rngfnd_addr=0x09),
        ]
        while len(i2c_drivers):
            do_drivers = i2c_drivers[0:9]
            i2c_drivers = i2c_drivers[9:]
            count = 1
            p = {}
            for d in do_drivers:
                p[f"RNGFND{count}_TYPE"] = d.rngfnd_type
                if d.rngfnd_addr is not None:
                    p[f"RNGFND{count}_ADDR"] = d.rngfnd_addr
                count += 1

            self.set_parameters(p)

            self.reboot_sitl()
            self.fly_rangefinder_drivers_fly([x.name for x in do_drivers])

    def RangeFinderDriversMaxAlt_FlyDriver(self,
                                           name : str,
                                           rngfnd_type : int,
                                           simname : str,
                                           maxalt : float,
                                           sqalt : float | None = None,
                                           sq_at_sqalt : float | None = None,
                                           expected_statustext : str | None = None,
                                           ):
        '''test max-height behaviour'''
        # lightwareserial goes to 130m when out of range
        self.progress(f"Flying {name}")
        self.set_parameters({
            "SERIAL4_PROTOCOL": 9,
            "RNGFND1_TYPE": rngfnd_type,
            "WPNAV_SPEED_UP": 1000,  # cm/s
        })
        self.customise_SITL_commandline([
            f"--serial4=sim:{simname}",
        ])
        takeoff_alt = sqalt
        if sqalt is None:
            takeoff_alt = maxalt
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())
        self.takeoff(takeoff_alt, mode='GUIDED', timeout=240, max_err=0.5)
        self.assert_rangefinder_distance_between(takeoff_alt-5, takeoff_alt+5)

        self.wait_rangefinder_distance(takeoff_alt-5, takeoff_alt+5)

        rf_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION

        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        if sq_at_sqalt is not None:
            self.assert_distance_sensor_quality(sq_at_sqalt)

        self.progress("Moving higher to be out of max rangefinder range")
        self.fly_guided_move_local(0, 0, takeoff_alt + 50)

        # sensor remains healthy even out-of-range
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)

        self.assert_distance_sensor_quality(1)

        self.progress("Test behaviour above 655.35 metres (UINT16_MAX cm)")
        self.context_push()
        self.context_collect('STATUSTEXT')
#        self.send_debug_trap()
        self.fly_guided_move_local(0, 0, 700)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        self.assert_distance_sensor_quality(1)
        if expected_statustext is not None:
            self.wait_statustext(expected_statustext, check_context=True)
        self.context_pop()

        # life's too short for soft landings:
        self.disarm_vehicle(force=True)

    def RangeFinderDriversMaxAlt_LightwareSerial(self) -> None:
        '''test max-height behaviour - LightwareSerial'''
        self.RangeFinderDriversMaxAlt_FlyDriver(
            name="LightwareSerial",
            rngfnd_type=8,
            simname='lightwareserial',
            maxalt=100,
            sqalt=95,
            sq_at_sqalt=100,
            expected_statustext=None,
        )

    def RangeFinderDriversMaxAlt_AinsteinLRD1(self) -> None:
        '''test max-height behaviour - Ainstein LRD1 - pre v19.0.0 firmware'''
        self.context_collect('STATUSTEXT')
        self.RangeFinderDriversMaxAlt_FlyDriver(
            name="Ainstein-LR-D1",
            rngfnd_type=42,
            simname='ainsteinlrd1',
            maxalt=500,
            sqalt=495,
            sq_at_sqalt=39,
            expected_statustext='Altitude reading overflow',
        )
        self.wait_text("Rangefinder: Temperature alert", check_context=True)

    def RangeFinderDriversMaxAlt_AinsteinLRD1_v19(self) -> None:
        '''test max-height behaviour - Ainstein LRD1 - v19.0.0 firmware'''
        self.context_collect('STATUSTEXT')
        self.RangeFinderDriversMaxAlt_FlyDriver(
            name="Ainstein-LR-D1-v19",
            rngfnd_type=42,
            simname='ainsteinlrd1_v19',
            maxalt=500,
            sqalt=495,
            sq_at_sqalt=39,
        )
        self.wait_text("Rangefinder: MCU Temperature alert", check_context=True)

    def RangeFinderDriversLongRange(self):
        '''test rangefinder above 327m'''
        # FIXME: when we get a driver+simulator for a rangefinder with
        # >327m range change this to use that driver
        self.set_parameters({
            "SERIAL4_PROTOCOL": 9,
            "RNGFND1_TYPE": 19,  # BenewakeTF02
            "WPNAV_SPEED_UP": 1000,  # cm/s
        })
        self.customise_SITL_commandline([
            "--serial4=sim:benewake_tf02",
        ])

        text_good = "GOOD"
        text_out_of_range_high = "OUT_OF_RANGE_HIGH"

        rf_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION

        alt = 3
        self.takeoff(alt, mode='GUIDED')
        self.assert_parameter_value("RNGFND1_MAX", 7.00)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range got range={m.current_distance * 0.01}")
        self.assert_parameter_value("RNGFND1_MAX", m.max_distance * 0.01, epsilon=0.00001)
        self.assert_parameter_value("RNGFND1_MIN", m.min_distance * 0.01, epsilon=0.00001)

        self.send_statustext(text_good)

        alt = 10
        self.fly_guided_move_local(0, 0, alt)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range")
        self.assert_parameter_value("RNGFND1_MAX", m.max_distance * 0.01, epsilon=0.00001)
        self.assert_parameter_value("RNGFND1_MIN", m.min_distance * 0.01, epsilon=0.00001)
        self.send_statustext(text_out_of_range_high)

        self.progress("Moving the goalposts")
        self.set_parameter("RNGFND1_MAX", 20)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range")
        self.assert_parameter_value("RNGFND1_MAX", m.max_distance * 0.01, epsilon=0.00001)
        self.assert_parameter_value("RNGFND1_MIN", m.min_distance * 0.01, epsilon=0.00001)
        self.send_statustext(text_good)
        self.delay_sim_time(2)

        dfreader = self.dfreader_for_current_onboard_log()

        self.do_RTL()

        self.progress("Checking in/out of range markers in log")
        required_range = None
        count = 0
        while True:
            m = dfreader.recv_match(type=["MSG", "RFND"])
            if m is None:
                break
            m_type = m.get_type()
            if m_type == "MSG":
                for t in [text_good, text_out_of_range_high]:
                    if t in m.Message:
                        required_range = t
                continue
            if m_type == "RFND":
                if required_range is None:
                    continue
                if required_range == text_good:
                    required_state = 4
                elif required_range == text_out_of_range_high:
                    required_state = 3
                else:
                    raise ValueError(f"Unexpected text {required_range}")
                if m.Stat != required_state:
                    raise NotAchievedException(f"Not in expected state want={required_state} got={m.Stat} dist={m.Dist}")
                self.progress(f"In expected state {required_range}")
                required_range = None
                count += 1

        if count < 3:
            raise NotAchievedException("Did not see range markers")

    def RangeFinderSITLLongRange(self):
        '''test rangefinder above 327m'''
        # FIXME: when we get a driver+simulator for a rangefinder with
        # >327m range change this to use that driver
        self.set_parameters({
            "RNGFND1_TYPE": 100,  # SITL
            "WPNAV_SPEED_UP": 1000,  # cm/s
            "RNGFND1_MAX": 7000,  # metres
        })
        self.reboot_sitl()

        text_good = "GOOD"
        text_high = "HIGH"

        rf_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION

        alt = 3
        self.takeoff(alt, mode='GUIDED')
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        got = m.current_distance * 0.01
        if abs(got - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range {got=}")

        self.send_statustext(text_good)

        alt = 635
        self.fly_guided_move_local(0, 0, alt)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range")

        self.send_statustext(text_high)

        dfreader = self.dfreader_for_current_onboard_log()

        # fast descent!
        def hook(msg, m):
            if m.get_type() != 'HEARTBEAT':
                return
            # tell vehicle to only pay attention to the attitude:
            bitmask = 0
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE

            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                1, # target sysid
                1, # target compid
                0, # bitmask of things to ignore
                mavextra.euler_to_quat([
                    math.radians(-180),
                    math.radians(0),
                    math.radians(0)]), # att
                0, # roll rate  (rad/s)
                0, # pitch rate (rad/s)
                0, # yaw rate   (rad/s)
                1   # thrust, 0 to 1, translated to a climb/descent rate
            )

        self.install_message_hook_context(hook)

        self.wait_altitude(0, 30, timeout=200, relative=True)
        self.do_RTL()

        self.progress("Checking in/out of range markers in log")
        good = False
        while True:
            m = dfreader.recv_match(type=["MSG", "RFND"])
            if m is None:
                break
            m_type = m.get_type()
            if m_type == "RFND":
                if m.Dist < 600:
                    continue
                good = True
                break

        if not good:
            raise NotAchievedException("Did not see good alt")

    def ShipTakeoff(self):
        '''Fly Simulated Ship Takeoff'''
        # test ship takeoff
        self.wait_groundspeed(0, 2)
        self.set_parameters({
            "SIM_SHIP_ENABLE": 1,
            "SIM_SHIP_SPEED": 10,
            "SIM_SHIP_DSIZE": 2,
        })
        self.wait_ready_to_arm()
        # we should be moving with the ship
        self.wait_groundspeed(9, 11)
        self.takeoff(10)
        # above ship our speed drops to 0
        self.wait_groundspeed(0, 2)
        self.land_and_disarm()
        # ship will have moved on, so we land on the water which isn't moving
        self.wait_groundspeed(0, 2)

    def ParameterValidation(self):
        '''Test parameters are checked for validity'''
        # wait 10 seconds for initialisation
        self.delay_sim_time(10)
        self.progress("invalid; min must be less than max:")
        self.set_parameters({
            "MOT_PWM_MIN": 100,
            "MOT_PWM_MAX": 50,
        })
        self.drain_mav()
        self.assert_prearm_failure("Motors: Check MOT_PWM_MIN and MOT_PWM_MAX")
        self.progress("invalid; min must be less than max (equal case):")
        self.set_parameters({
            "MOT_PWM_MIN": 100,
            "MOT_PWM_MAX": 100,
        })
        self.drain_mav()
        self.assert_prearm_failure("Motors: Check MOT_PWM_MIN and MOT_PWM_MAX")
        self.progress("Spin min more than 0.3")
        self.set_parameters({
            "MOT_PWM_MIN": 1000,
            "MOT_PWM_MAX": 2000,
            "MOT_SPIN_MIN": 0.5,
        })
        self.drain_mav()
        self.assert_prearm_failure("PreArm: Motors: MOT_SPIN_MIN too high 0.50 > 0.3")
        self.progress("Spin arm more than spin min")
        self.set_parameters({
            "MOT_SPIN_MIN": 0.1,
            "MOT_SPIN_ARM": 0.2,
        })
        self.drain_mav()
        self.assert_prearm_failure("PreArm: Motors: MOT_SPIN_ARM > MOT_SPIN_MIN")

    def SensorErrorFlags(self):
        '''Test we get ERR messages when sensors have issues'''
        self.reboot_sitl()

        for (param_names, param_value, expected_subsys, expected_ecode, desc) in [
                (['SIM_BARO_DISABLE', 'SIM_BAR2_DISABLE'], 1, 18, 4, 'unhealthy'),
                (['SIM_BARO_DISABLE', 'SIM_BAR2_DISABLE'], 0, 18, 0, 'healthy'),
                (['SIM_MAG1_FAIL', 'SIM_MAG2_FAIL', 'SIM_MAG3_FAIL'], 1, 3, 4, 'unhealthy'),
                (['SIM_MAG1_FAIL', 'SIM_MAG2_FAIL', 'SIM_MAG3_FAIL'], 0, 3, 0, 'healthy'),
        ]:
            sp = dict()
            for name in param_names:
                sp[name] = param_value
            self.set_parameters(sp)
            self.delay_sim_time(1)
            mlog = self.dfreader_for_current_onboard_log()
            success = False
            while True:
                m = mlog.recv_match(type='ERR')
                print("Got (%s)" % str(m))
                if m is None:
                    break
                if m.Subsys == expected_subsys and m.ECode == expected_ecode:  # baro / ecode
                    success = True
                    break
            if not success:
                raise NotAchievedException("Did not find %s log message" % desc)

    def AltEstimation(self):
        '''Test that Alt Estimation is mandatory for ALT_HOLD'''
        # disable barometer so there is no altitude source
        self.set_parameters({
            "SIM_BARO_DISABLE": 1,
            "SIM_BAR2_DISABLE": 1,
        })

        self.wait_gps_disable(position_vertical=True)

        # turn off arming checks (mandatory arming checks will still be run)
        self.set_parameter("ARMING_SKIPCHK", -1)

        # delay 12 sec to allow EKF to lose altitude estimate
        self.delay_sim_time(12)

        self.change_mode("ALT_HOLD")
        self.assert_prearm_failure("Need Alt Estimate")

        # arm vehicle in stabilize to bypass barometer pre-arm checks
        self.change_mode("STABILIZE")
        self.arm_vehicle()
        self.set_rc(3, 1700)
        try:
            self.change_mode("ALT_HOLD", timeout=10)
        except AutoTestTimeoutException:
            self.progress("PASS not able to set mode without Position : %s" % "ALT_HOLD")

        # check that mode change to ALT_HOLD has failed (it should)
        if self.mode_is("ALT_HOLD"):
            raise NotAchievedException("Changed to ALT_HOLD with no altitude estimate")
        self.disarm_vehicle(force=True)

    def EKFSource(self):
        '''Check EKF Source Prearms work'''
        self.wait_ready_to_arm()

        self.start_subtest("bad yaw source")
        self.set_parameter("EK3_SRC3_YAW", 17)
        self.assert_prearm_failure("Check EK3_SRC3_YAW")

        self.context_push()
        self.start_subtest("missing required yaw source")
        self.set_parameters({
            "EK3_SRC3_YAW": 3, # External Yaw with Compass Fallback
            "COMPASS_USE": 0,
            "COMPASS_USE2": 0,
            "COMPASS_USE3": 0,
        })
        self.assert_prearm_failure("EK3 sources require Compass")
        self.context_pop()

    def EK3_EXT_NAV_vel_without_vert(self):
        '''Test that EK3 External Navigation velocity works without vertical velocity.'''

        # Setup Vicon system and EKF3 parameters
        self.set_parameters({
            "VISO_TYPE": 2,      # Enable Vicon system
            "SERIAL5_PROTOCOL": 2,
            "EK3_ENABLE": 1,      # Enable EKF3
            "EK3_SRC2_POSXY": 6,  # External Navigation for horizontal position
            "EK3_SRC2_POSZ": 1,   # Use Barometer for altitude
            "EK3_SRC2_VELXY": 6,  # External Navigation for horizontal velocity
            "EK3_SRC2_VELZ": 0,   # No vertical velocity input
            "EK3_SRC2_YAW": 1,    # Use Compass for yaw
            "EK3_SRC_OPTIONS": 0,  # Dont fuse all velocity sources
            "RC8_OPTION": 90,     # RC aux switch 8 set to EKF source selector
            "AHRS_EKF_TYPE": 3,   # Enable EKF3
        })

        # Customize SITL command line for Vicon simulation
        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        # Switch to use Vicon
        self.set_rc(8, 1500)

        # Take off
        self.progress("Moving")
        self.takeoffAndMoveAway()

        # Increase Vicon vertical velocity error
        self.set_parameter("SIM_VICON_VGLI_Z", 1)

        # Set innovation limit
        innov_limit = 0.1  # Threshold for velocity variance

        max_variance = 0.0
        tstart = self.get_sim_time()
        while True:
            msg = self.assert_receive_message(type="EKF_STATUS_REPORT")
            variance = msg.velocity_variance
            max_variance = max(max_variance, variance)
            if variance > innov_limit:
                raise NotAchievedException(
                    f"Velocity variance too high {variance:.2f} > {innov_limit:.2f}"
                )
            if self.get_sim_time_cached() - tstart > 10:
                break
        print(f"Max variance before enabling vertical velocity fusion: {max_variance:.2f}")

        # Now enable vertical velocity fusion and check for innovation spike
        self.set_parameter("EK3_SRC2_VELZ", 6)

        spike_detected = False
        max_variance = 0.0
        tstart = self.get_sim_time()
        while True:
            msg = self.assert_receive_message(type="EKF_STATUS_REPORT")
            variance = msg.velocity_variance
            max_variance = max(max_variance, variance)
            if variance > innov_limit:
                spike_detected = True
            if self.get_sim_time_cached() - tstart > 10:
                break
        print(f"Max variance after enabling vertical velocity fusion: {max_variance:.2f}")

        if not spike_detected:
            raise NotAchievedException("Velocity variance did not spike as expected after enabling vertical velocity fusion.")

        self.disarm_vehicle(force=True)

    def test_replay_gps_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 1,
            "AHRS_TRIM_X": 0.01,
            "AHRS_TRIM_Y": -0.03,
            "GPS2_TYPE": 1,
            "GPS1_POS_X": 0.1,
            "GPS1_POS_Y": 0.2,
            "GPS1_POS_Z": 0.3,
            "GPS2_POS_X": -0.1,
            "GPS2_POS_Y": -0.02,
            "GPS2_POS_Z": -0.31,
            "INS_POS1_X": 0.12,
            "INS_POS1_Y": 0.14,
            "INS_POS1_Z": -0.02,
            "INS_POS2_X": 0.07,
            "INS_POS2_Y": 0.012,
            "INS_POS2_Z": -0.06,
            "RNGFND1_TYPE": 1,
            "RNGFND1_PIN": 0,
            "RNGFND1_SCALING": 30,
            "RNGFND1_POS_X": 0.17,
            "RNGFND1_POS_Y": -0.07,
            "RNGFND1_POS_Z": -0.005,
            "SIM_SONAR_SCALE": 30,
            "SIM_GPS2_ENABLE": 1,
        })
        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)

        current_log_filepath = self.current_onboard_log_filepath()
        self.progress("Current log path: %s" % str(current_log_filepath))

        self.change_mode("LOITER")
        self.wait_ready_to_arm(require_absolute=True)
        self.arm_vehicle()
        self.takeoffAndMoveAway()
        self.do_RTL()

        self.reboot_sitl()

        return current_log_filepath

    def test_replay_beacon_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
        })

        old_onboard_logs = sorted(self.log_list())
        self.BeaconPosition()
        new_onboard_logs = sorted(self.log_list())

        self.reboot_sitl()

        log_difference = [x for x in new_onboard_logs if x not in old_onboard_logs]
        return log_difference[1] # index depends on the reboots and ordering thereof in BeaconPosition!

    def test_replay_optical_flow_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
        })

        old_onboard_logs = sorted(self.log_list())
        self.OpticalFlowLimits()
        new_onboard_logs = sorted(self.log_list())

        log_difference = [x for x in new_onboard_logs if x not in old_onboard_logs]
        print("log difference: %s" % str(log_difference))
        return log_difference[0]

    def GPSBlendingLog(self):
        '''Test GPS Blending'''
        '''ensure we get dataflash log messages for blended instance'''
        # configure:
        self.set_parameters({
            "GPS2_TYPE": 1,
            "SIM_GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "GPS_AUTO_SWITCH": 2,
        })
        self.reboot_sitl()

        # ensure we're seeing the second GPS:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("Did not get good GPS2_RAW message")
            m = self.assert_receive_message('GPS2_RAW', verbose=True)
            if m.lat == 0:
                continue
            break

        # create a log we can expect blended data to appear in:
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.delay_sim_time(5)
        self.disarm_vehicle()

        # inspect generated log for messages:
        dfreader = self.dfreader_for_current_onboard_log()
        wanted = set([0, 1, 2])
        seen_primary_change = False
        while True:
            m = dfreader.recv_match(type=["GPS", "EV"]) # disarmed
            if m is None:
                break
            mtype = m.get_type()
            if mtype == 'GPS':
                try:
                    wanted.remove(m.I)
                except KeyError:
                    continue
            elif mtype == 'EV':
                if m.Id == 67:  # GPS_PRIMARY_CHANGED
                    seen_primary_change = True
            if len(wanted) == 0 and seen_primary_change:
                break

        if len(wanted):
            raise NotAchievedException("Did not get all three GPS types")
        if not seen_primary_change:
            raise NotAchievedException("Did not see primary change")

    def GPSBlending(self):
        '''Test GPS Blending'''
        '''ensure we get dataflash log messages for blended instance'''

        # configure:
        self.set_parameters({
            "WP_YAW_BEHAVIOR": 0,  # do not yaw
            "GPS2_TYPE": 1,
            "SIM_GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "SIM_GPS1_POS_X": 1.0,
            "SIM_GPS1_POS_Y": -1.0,
            "SIM_GPS2_POS_X": -1.0,
            "SIM_GPS2_POS_Y": 1.0,
            "GPS_AUTO_SWITCH": 2,
        })
        self.reboot_sitl()

        alt = 10
        self.takeoff(alt, mode='GUIDED')
        self.fly_guided_move_local(30, 0, alt)
        self.fly_guided_move_local(30, 30, alt)
        self.fly_guided_move_local(0, 30, alt)
        self.fly_guided_move_local(0, 0, alt)
        self.change_mode('LAND')

        current_log_file = self.dfreader_for_current_onboard_log()

        self.wait_disarmed()

        # ensure that the blended solution is always about half-way
        # between the two GPSs:
        passes = 0
        current_ts = None
        while True:
            m = current_log_file.recv_match(type='GPS')
            if m is None:
                break
            if current_ts is None:
                if m.I != 0:  # noqa
                    continue
                current_ts = m.TimeUS
                measurements = {}
            if m.TimeUS != current_ts:
                current_ts = None
                continue
            measurements[m.I] = (m.Lat, m.Lng)
            if len(measurements) == 3:
                # check lat:
                for n in 0, 1:
                    expected_blended = (measurements[0][n] + measurements[1][n])/2
                    epsilon = 0.0000002
                    error = abs(measurements[2][n] - expected_blended)
                    if error > epsilon:
                        raise NotAchievedException("Blended diverged")
                current_ts = None
                passes += 1

        if passes == 0:
            raise NotAchievedException("Did not see three GPS measurements!")

    def GPSWeightedBlending(self):
        '''Test GPS Weighted Blending'''

        self.context_push()

        # configure:
        self.set_parameters({
            "WP_YAW_BEHAVIOR": 0,  # do not yaw
            "GPS2_TYPE": 1,
            "SIM_GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "SIM_GPS1_POS_X": 1.0,
            "SIM_GPS1_POS_Y": -1.0,
            "SIM_GPS2_POS_X": -1.0,
            "SIM_GPS2_POS_Y": 1.0,
            "GPS_AUTO_SWITCH": 2,
        })
        # configure velocity errors such that the 1st GPS should be
        # 4/5, second GPS 1/5 of result (0.5**2)/((0.5**2)+(1.0**2))
        self.set_parameters({
            "SIM_GPS1_VERR_X": 0.3,  # m/s
            "SIM_GPS1_VERR_Y": 0.4,
            "SIM_GPS2_VERR_X": 0.6,  # m/s
            "SIM_GPS2_VERR_Y": 0.8,
            "GPS_BLEND_MASK": 4,  # use only speed for blend calculations
        })
        self.reboot_sitl()

        alt = 10
        self.takeoff(alt, mode='GUIDED')
        self.fly_guided_move_local(30, 0, alt)
        self.fly_guided_move_local(30, 30, alt)
        self.fly_guided_move_local(0, 30, alt)
        self.fly_guided_move_local(0, 0, alt)
        self.change_mode('LAND')

        current_log_file = self.dfreader_for_current_onboard_log()

        self.wait_disarmed()

        # ensure that the blended solution is always about half-way
        # between the two GPSs:
        current_ts = None
        while True:
            m = current_log_file.recv_match(type='GPS')
            if m is None:
                break
            if current_ts is None:
                if m.I != 0:  # noqa
                    continue
                current_ts = m.TimeUS
                measurements = {}
            if m.TimeUS != current_ts:
                current_ts = None
                continue
            measurements[m.I] = (m.Lat, m.Lng, m.Alt)
            if len(measurements) == 3:
                # check lat:
                for n in 0, 1, 2:
                    expected_blended = 0.8*measurements[0][n] + 0.2*measurements[1][n]
                    axis_epsilons = [0.0000002, 0.0000002, 0.2]
                    epsilon = axis_epsilons[n]
                    error = abs(measurements[2][n] - expected_blended)
                    if error > epsilon:
                        raise NotAchievedException(f"Blended diverged {n=} {measurements[0][n]=} {measurements[1][n]=}")
                current_ts = None

        self.context_pop()
        self.reboot_sitl()

    def GPSBlendingAffinity(self):
        '''test blending when affinity in use'''
        # configure:
        self.set_parameters({
            "WP_YAW_BEHAVIOR": 0,  # do not yaw
            "GPS2_TYPE": 1,
            "SIM_GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "SIM_GPS1_POS_X": 1.0,
            "SIM_GPS1_POS_Y": -1.0,
            "SIM_GPS2_POS_X": -1.0,
            "SIM_GPS2_POS_Y": 1.0,
            "GPS_AUTO_SWITCH": 2,

            "EK3_AFFINITY" : 1,
            "EK3_IMU_MASK": 7,
            "SIM_IMU_COUNT": 3,
            "INS_ACC3OFFS_X": 0.001,
            "INS_ACC3OFFS_Y": 0.001,
            "INS_ACC3OFFS_Z": 0.001,
        })
        # force-calibration of accel:
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, p5=76)
        self.reboot_sitl()

        alt = 10
        self.takeoff(alt, mode='GUIDED')
        self.fly_guided_move_local(30, 0, alt)
        self.fly_guided_move_local(30, 30, alt)
        self.fly_guided_move_local(0, 30, alt)
        self.fly_guided_move_local(0, 0, alt)
        self.change_mode('LAND')

        current_log_file = self.dfreader_for_current_onboard_log()

        self.wait_disarmed()

        # ensure that the blended solution is always about half-way
        # between the two GPSs:
        current_ts = None
        max_errors = [0, 0, 0]
        while True:
            m = current_log_file.recv_match(type='XKF1')
            if m is None:
                break
            if current_ts is None:
                if m.C != 0:  # noqa
                    continue
                current_ts = m.TimeUS
                measurements = {}
            if m.TimeUS != current_ts:
                current_ts = None
                continue
            measurements[m.C] = (m.PN, m.PE, m.PD)
            if len(measurements) == 3:
                # check lat:
                axis_epsilons = [0.05, 0.05, 0.06]
                for n in 0, 1, 2:
                    expected_blended = 0.5*measurements[0][n] + 0.5*measurements[1][n]
                    epsilon = axis_epsilons[n]
                    error = abs(measurements[2][n] - expected_blended)
                    # self.progress(f"{n=} {error=}")
                    if error > max_errors[n]:
                        max_errors[n] = error
                    if error > epsilon:
                        raise NotAchievedException(f"Blended diverged {n=} {measurements[0][n]=} {measurements[1][n]=} {measurements[2][n]=} {error=}")  # noqa:E501
                current_ts = None
        self.progress(f"{max_errors=}")

    def Callisto(self):
        '''Test Callisto'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('Callisto'),
            model="octa-quad:@ROMFS/models/Callisto.json",
            wipe=True,
        )
        self.takeoff(10)
        self.do_RTL()

    def FlyEachFrame(self):
        '''Fly each supported internal frame'''
        vinfo = vehicleinfo.VehicleInfo()
        copter_vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
            'heli-compound': "wrong binary, different takeoff regime",
            'heli-dual': "wrong binary, different takeoff regime",
            'heli': "wrong binary, different takeoff regime",
            'heli-gas': "wrong binary, different takeoff regime",
            'heli-blade360': "wrong binary, different takeoff regime",
            "quad-can" : "needs CAN periph",
        }
        for frame in sorted(copter_vinfo_options["frames"].keys()):
            self.start_subtest("Testing frame (%s)" % str(frame))
            if frame in known_broken_frames:
                self.progress("Actually, no I'm not - it is known-broken (%s)" %
                              (known_broken_frames[frame]))
                continue
            frame_bits = copter_vinfo_options["frames"][frame]
            print("frame_bits: %s" % str(frame_bits))
            if frame_bits.get("external", False):
                self.progress("Actually, no I'm not - it is an external simulation")
                continue
            model = frame_bits.get("model", frame)
            defaults = self.model_defaults_filepath(frame)
            if not isinstance(defaults, list):
                defaults = [defaults]
            self.context_push()
            frame_script = frame_bits.get('frame_example_script', None)
            if frame_script is not None:
                self.install_example_script_context(frame_script)
            self.customise_SITL_commandline(
                [],
                defaults_filepath=defaults,
                model=model,
                wipe=True,
            )
            if frame_script is not None:
                self.set_parameters({
                    "SCR_ENABLE": 1,
                    "LOG_BITMASK": 65535,
                })
                self.reboot_sitl()

            # add a listener that verifies yaw looks good:
            def verify_yaw(mav, m):
                if m.get_type() != 'ATTITUDE':
                    return
                yawspeed_thresh_rads = math.radians(20)
                if m.yawspeed > yawspeed_thresh_rads:
                    raise NotAchievedException("Excessive yaw on takeoff: %f deg/s > %f deg/s (frame=%s)" %
                                               (math.degrees(m.yawspeed), math.degrees(yawspeed_thresh_rads), frame))
            self.context_push()
            self.install_message_hook_context(verify_yaw)
            self.takeoff(10)
            self.context_pop()
            self.hover()
            self.change_mode('ALT_HOLD')
            self.delay_sim_time(1)

            def verify_rollpitch(mav, m):
                if m.get_type() != 'ATTITUDE':
                    return
                pitch_thresh_rad = math.radians(2)
                if m.pitch > pitch_thresh_rad:
                    raise NotAchievedException("Excessive pitch %f deg > %f deg" %
                                               (math.degrees(m.pitch), math.degrees(pitch_thresh_rad)))
                roll_thresh_rad = math.radians(2)
                if m.roll > roll_thresh_rad:
                    raise NotAchievedException("Excessive roll %f deg > %f deg" %
                                               (math.degrees(m.roll), math.degrees(roll_thresh_rad)))
            self.context_push()
            self.install_message_hook_context(verify_rollpitch)
            for i in range(5):
                self.set_rc(4, 2000)
                self.delay_sim_time(0.5)
                self.set_rc(4, 1500)
                self.delay_sim_time(5)
            self.context_pop()

            self.do_RTL()

            self.context_pop()

    def Replay(self):
        '''test replay correctness'''
        self.progress("Building Replay")
        util.build_SITL('tool/Replay', clean=False, configure=False)
        self.set_parameters({
            "LOG_DARM_RATEMAX": 0,
            "LOG_FILE_RATEMAX": 0,
        })

        bits = [
            ('GPS', self.test_replay_gps_bit),
            ('Beacon', self.test_replay_beacon_bit),
            ('OpticalFlow', self.test_replay_optical_flow_bit),
        ]
        for (name, func) in bits:
            self.start_subtest("%s" % name)
            self.test_replay_bit(func)

    def test_replay_bit(self, bit):

        self.context_push()
        current_log_filepath = bit()

        self.progress("Running replay on (%s) (%u bytes)" % (
            (current_log_filepath, os.path.getsize(current_log_filepath))
        ))

        self.run_replay(current_log_filepath)

        replay_log_filepath = self.current_onboard_log_filepath()

        self.context_pop()

        self.progress("Replay log path: %s" % str(replay_log_filepath))

        check_replay = util.load_local_module("Tools/Replay/check_replay.py")

        ok = check_replay.check_log(replay_log_filepath, self.progress, verbose=True)
        if not ok:
            raise NotAchievedException("check_replay (%s) failed" % current_log_filepath)

    def DefaultIntervalsFromFiles(self):
        '''Test setting default mavlink message intervals from files'''
        ex = None
        intervals_filepath = util.reltopdir("message-intervals-chan0.txt")
        self.progress("Using filepath (%s)" % intervals_filepath)
        try:
            with open(intervals_filepath, "w") as f:
                f.write("""30 50
28 100
29 200
""")
                f.close()

            # other tests may have explicitly set rates, so wipe parameters:
            def custom_stream_rate_setter():
                for stream in mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS:
                    self.set_streamrate(5, stream=stream)

            self.customise_SITL_commandline(
                [],
                wipe=True,
                set_streamrate_callback=custom_stream_rate_setter,
            )

            self.assert_message_rate_hz("ATTITUDE", 20)
            self.assert_message_rate_hz("SCALED_PRESSURE", 5)

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        os.unlink(intervals_filepath)

        self.reboot_sitl()

        if ex is not None:
            raise ex

    def BaroDrivers(self):
        '''Test Baro Drivers'''
        sensors = [
            ("MS5611", 2),
        ]
        for (name, bus) in sensors:
            self.context_push()
            if bus is not None:
                self.set_parameter("BARO_EXT_BUS", bus)
            self.set_parameter("BARO_PROBE_EXT", 1 << 2)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()

            # insert listener to compare airspeeds:
            messages = [None, None, None]

            global count
            count = 0

            def check_pressure(mav, m):
                global count
                m_type = m.get_type()
                count += 1
                # if count > 500:
                #     if press_abs[0] is None or press_abs[1] is None:
                #         raise NotAchievedException("Not receiving messages")
                if m_type == 'SCALED_PRESSURE3':
                    off = 2
                elif m_type == 'SCALED_PRESSURE2':
                    off = 1
                elif m_type == 'SCALED_PRESSURE':
                    off = 0
                else:
                    return

                messages[off] = m

                if None in messages:
                    return
                first = messages[0]
                for msg in messages[1:]:
                    delta_press_abs = abs(first.press_abs - msg.press_abs)
                    if delta_press_abs > 0.5: # 50 Pa leeway
                        raise NotAchievedException("Press_Abs mismatch (press1=%s press2=%s)" % (first, msg))
                    delta_temperature = abs(first.temperature - msg.temperature)
                    if delta_temperature > 300:  # that's 3-degrees leeway
                        raise NotAchievedException("Temperature mismatch (t1=%s t2=%s)" % (first, msg))
            self.install_message_hook_context(check_pressure)
            self.fly_mission("copter_mission.txt", strict=False)
            if None in messages:
                raise NotAchievedException("Missing a message")

            self.context_pop()
        self.reboot_sitl()

    def PositionWhenGPSIsZero(self):
        '''Ensure position doesn't zero when GPS lost'''
        # https://github.com/ArduPilot/ardupilot/issues/14236
        self.progress("arm the vehicle and takeoff in Guided")
        self.takeoff(20, mode='GUIDED')
        self.progress("fly 50m North (or whatever)")
        old_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        self.fly_guided_move_global_relative_alt(50, 0, 20)
        self.set_parameter('GPS1_TYPE', 0)
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 30 and self.mode_is('LAND'):
                self.progress("Bug not reproduced")
                break
            m = self.assert_receive_message('GLOBAL_POSITION_INT', verbose=True)
            pos_delta = self.get_distance_int(old_pos, m)
            self.progress("Distance: %f" % pos_delta)
            if pos_delta < 5:
                raise NotAchievedException("Bug reproduced - returned to near origin")
        self.wait_disarmed()
        self.reboot_sitl()

    def RTL_ALT_FINAL(self):
        '''Test RTL with RTL_ALT_FINAL'''
        self.progress("arm the vehicle and takeoff in Guided")
        self.takeoff(20, mode='GUIDED')

        # Set home current location, this gives a large home vs origin difference
        self.set_home(self.mav.location())

        self.progress("fly 50m North (or whatever)")
        self.fly_guided_move_local(50, 0, 50)
        target_alt = 10
        self.set_parameter('RTL_ALT_FINAL', target_alt * 100)

        self.progress("Waiting RTL to reach Home and hold")
        self.change_mode('RTL')

        # Expecting to return and hold 10m above home
        tstart = self.get_sim_time()
        reachedHome = False
        while self.get_sim_time_cached() < tstart + 250:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            home = math.sqrt((alt-target_alt)**2 + home_distance**2) < 2
            if home and not reachedHome:
                reachedHome = True
                self.progress("Reached home - holding")
                self.delay_sim_time(20)
                continue

            if reachedHome:
                if not home:
                    raise NotAchievedException("Should still be at home")
                if not self.armed():
                    raise NotAchievedException("Should still be armed")
                break

        self.progress("Hold at home successful - landing")
        self.change_mode("LAND")
        self.wait_landed_and_disarmed()

    def SMART_RTL(self):
        '''Check SMART_RTL'''
        self.progress("arm the vehicle and takeoff in Guided")
        self.takeoff(20, mode='GUIDED')
        self.progress("fly around a bit (or whatever)")
        locs = [
            (50, 0, 20),
            (-50, 50, 20),
            (-50, 0, 20),
        ]
        for (lat, lng, alt) in locs:
            self.fly_guided_move_local(lat, lng, alt)

        self.change_mode('SMART_RTL')
        for (lat, lng, alt) in reversed(locs):
            self.wait_distance_to_local_position(
                (lat, lng, -alt),
                0,
                10,
                timeout=60
            )
        self.wait_disarmed()

    def get_ground_effect_duration_from_current_onboard_log(self, bit, ignore_multi=False):
        '''returns a duration in seconds we were expecting to interact with
        the ground.  Will die if there's more than one such block of
        time and ignore_multi is not set (will return first duration
        otherwise)
        '''
        ret = []
        dfreader = self.dfreader_for_current_onboard_log()
        seen_expected_start_TimeUS = None
        first = None
        last = None
        while True:
            m = dfreader.recv_match(type="XKF4")
            if m is None:
                break
            last = m
            if first is None:
                first = m
            # self.progress("%s" % str(m))
            expected = m.SS & (1 << bit)
            if expected:
                if seen_expected_start_TimeUS is None:
                    seen_expected_start_TimeUS = m.TimeUS
                    continue
            else:
                if seen_expected_start_TimeUS is not None:
                    duration = (m.TimeUS - seen_expected_start_TimeUS)/1000000.0
                    ret.append(duration)
                    seen_expected_start_TimeUS = None
        if seen_expected_start_TimeUS is not None:
            duration = (last.TimeUS - seen_expected_start_TimeUS)/1000000.0
            ret.append(duration)
        return ret

    def get_takeoffexpected_durations_from_current_onboard_log(self, ignore_multi=False):
        return self.get_ground_effect_duration_from_current_onboard_log(11, ignore_multi=ignore_multi)

    def get_touchdownexpected_durations_from_current_onboard_log(self, ignore_multi=False):
        return self.get_ground_effect_duration_from_current_onboard_log(12, ignore_multi=ignore_multi)

    def ThrowDoubleDrop(self):
        '''Test a more complicated drop-mode scenario'''
        self.progress("Getting a lift to altitude")
        self.set_parameters({
            "SIM_SHOVE_Z": -11,
            "THROW_TYPE": 1,   # drop
            "MOT_SPOOL_TIME": 2,
        })
        self.change_mode('THROW')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        try:
            self.set_parameter("SIM_SHOVE_TIME", 30000)
        except ValueError:
            # the shove resets this to zero
            pass

        self.wait_altitude(100, 1000, timeout=100, relative=True)
        self.context_collect('STATUSTEXT')
        self.wait_statustext("throw detected - spooling motors", check_context=True, timeout=10)
        self.wait_statustext("throttle is unlimited - uprighting", check_context=True)
        self.wait_statustext("uprighted - controlling height", check_context=True)
        self.wait_statustext("height achieved - controlling position", check_context=True)
        self.progress("Waiting for still")
        self.wait_speed_vector(Vector3(0, 0, 0))
        self.change_mode('ALT_HOLD')
        self.set_rc(3, 1000)
        self.wait_disarmed(timeout=90)
        self.zero_throttle()

        self.progress("second flight")
        self.upload_square_mission_items_around_location(self.poll_home_position())

        self.set_parameters({
            "THROW_NEXTMODE": 3,  # auto
        })

        self.change_mode('THROW')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        try:
            self.set_parameter("SIM_SHOVE_TIME", 30000)
        except ValueError:
            # the shove resets this to zero
            pass

        self.wait_altitude(100, 1000, timeout=100, relative=True)
        self.wait_statustext("throw detected - spooling motors", check_context=True, timeout=10)
        self.wait_statustext("throttle is unlimited - uprighting", check_context=True)
        self.wait_statustext("uprighted - controlling height", check_context=True)
        self.wait_statustext("height achieved - controlling position", check_context=True)
        self.wait_mode('AUTO')
        self.wait_disarmed(timeout=240)

    def GroundEffectCompensation_takeOffExpected(self):
        '''Test EKF's handling of takeoff-expected'''
        self.change_mode('ALT_HOLD')
        self.set_parameter("LOG_FILE_DSRMROT", 1)
        self.progress("Making sure we'll have a short log to look at")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        # arm the vehicle and let it disarm normally.  This should
        # yield a log where the EKF considers a takeoff imminent until
        # disarm
        self.start_subtest("Check ground effect compensation remains set in EKF while we're at idle on the ground")
        self.arm_vehicle()
        self.wait_disarmed()

        durations = self.get_takeoffexpected_durations_from_current_onboard_log()
        duration = durations[0]
        want = 9
        self.progress("takeoff-expected duration: %fs" % (duration,))
        if duration < want:  # assumes default 10-second DISARM_DELAY
            raise NotAchievedException("Should have been expecting takeoff for longer than %fs (want>%f)" %
                                       (duration, want))

        self.start_subtest("takeoffExpected should be false very soon after we launch into the air")
        self.takeoff(mode='ALT_HOLD', alt_min=5)
        self.change_mode('LAND')
        self.wait_disarmed()
        durations = self.get_takeoffexpected_durations_from_current_onboard_log(ignore_multi=True)
        self.progress("touchdown-durations: %s" % str(durations))
        duration = durations[0]
        self.progress("takeoff-expected-duration %f" % (duration,))
        want_lt = 5
        if duration >= want_lt:
            raise NotAchievedException("Was expecting takeoff for longer than expected; got=%f want<=%f" %
                                       (duration, want_lt))

    def _MAV_CMD_CONDITION_YAW(self, command):
        self.start_subtest("absolute")
        self.takeoff(20, mode='GUIDED')

        m = self.assert_receive_message('VFR_HUD')
        initial_heading = m.heading

        self.progress("Ensuring initial heading is steady")
        target = initial_heading
        command(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=target,  # target angle
            p2=10,  # degrees/second
            p3=1,  # -1 is counter-clockwise, 1 clockwise
            p4=0,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(target, minimum_duration=2, timeout=50)
        self.wait_yaw_speed(0)

        degsecond = 2

        def rate_watcher(mav, m):
            if m.get_type() != 'ATTITUDE':
                return
            if abs(math.degrees(m.yawspeed)) > 5*degsecond:
                raise NotAchievedException("Moved too fast (%f>%f)" %
                                           (math.degrees(m.yawspeed), 5*degsecond))
        self.install_message_hook_context(rate_watcher)
        self.progress("Yaw CW 60 degrees")
        target = initial_heading + 60
        part_way_target = initial_heading + 10
        command(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=target,     # target angle
            p2=degsecond,  # degrees/second
            p3=1,          # -1 is counter-clockwise, 1 clockwise
            p4=0,          # 1 for relative, 0 for absolute
        )
        self.wait_heading(part_way_target)
        self.wait_heading(target, minimum_duration=2)

        self.progress("Yaw CCW 60 degrees")
        target = initial_heading
        part_way_target = initial_heading + 30
        command(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=target,  # target angle
            p2=degsecond,  # degrees/second
            p3=-1,  # -1 is counter-clockwise, 1 clockwise
            p4=0,  # 1 for relative, 0 for absolute
        )
        self.wait_heading(part_way_target)
        self.wait_heading(target, minimum_duration=2)

        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def MAV_CMD_CONDITION_YAW(self):
        '''Test response to MAV_CMD_CONDITION_YAW via mavlink'''
        self.context_push()
        self._MAV_CMD_CONDITION_YAW(self.run_cmd_int)
        self.context_pop()
        self.context_push()
        self._MAV_CMD_CONDITION_YAW(self.run_cmd)
        self.context_pop()

    def GroundEffectCompensation_touchDownExpected(self):
        '''Test EKF's handling of touchdown-expected'''
        self.zero_throttle()
        self.change_mode('ALT_HOLD')
        self.set_parameter("LOG_FILE_DSRMROT", 1)
        self.progress("Making sure we'll have a short log to look at")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        self.start_subtest("Make sure touchdown-expected duration is about right")
        self.takeoff(20, mode='ALT_HOLD')
        self.change_mode('LAND')
        self.wait_disarmed()

        durations = self.get_touchdownexpected_durations_from_current_onboard_log(ignore_multi=True)
        self.progress("touchdown-durations: %s" % str(durations))
        duration = durations[-1]
        expected = 23  # this is the time in the final descent phase of LAND
        if abs(duration - expected) > 5:
            raise NotAchievedException("Was expecting roughly %fs of touchdown expected, got %f" % (expected, duration))

    def upload_square_mission_items_around_location(self, loc):
        alt = 20
        loc.alt = alt
        items = [
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt)
        ]

        for (ofs_n, ofs_e) in (20, 20), (20, -20), (-20, -20), (-20, 20), (20, 20):
            items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, ofs_n, ofs_e, alt))

        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

        self.upload_simple_relhome_mission(items)

    def RefindGPS(self):
        '''Refind the GPS and attempt to RTL rather than continue to land'''
        # https://github.com/ArduPilot/ardupilot/issues/14236
        self.progress("arm the vehicle and takeoff in Guided")
        self.takeoff(50, mode='GUIDED')
        self.progress("fly 50m North (or whatever)")
        old_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        self.fly_guided_move_global_relative_alt(50, 0, 50)
        self.set_parameter('GPS1_TYPE', 0)
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 30 and self.mode_is('LAND'):
                self.progress("Bug not reproduced")
                break
            m = self.assert_receive_message('GLOBAL_POSITION_INT', verbose=True)
            pos_delta = self.get_distance_int(old_pos, m)
            self.progress("Distance: %f" % pos_delta)
            if pos_delta < 5:
                raise NotAchievedException("Bug reproduced - returned to near origin")
        self.set_parameter('GPS1_TYPE', 1)
        self.do_RTL()

    def GPSForYaw(self):
        '''Moving baseline GPS yaw'''
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)
        m = self.assert_receive_message("GPS2_RAW", very_verbose=True)
        want = 27000
        if abs(m.yaw - want) > 500:
            raise NotAchievedException("Expected to get GPS-from-yaw (want %f got %f)" % (want, m.yaw))
        self.wait_ready_to_arm()

    def SMART_RTL_EnterLeave(self):
        '''check SmartRTL behaviour when entering/leaving'''
        # we had a bug where we would consume points when re-entering smartrtl

        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameter('AUTO_OPTIONS', 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.change_mode('ALT_HOLD')
        self.change_mode('SMART_RTL')
        self.change_mode('ALT_HOLD')
        self.change_mode('SMART_RTL')

    def SMART_RTL_Repeat(self):
        '''Test whether Smart RTL catches the repeat'''
        self.takeoff(alt_min=10, mode='GUIDED')
        self.set_rc(3, 1500)
        self.change_mode("CIRCLE")
        self.delay_sim_time(1300)
        self.change_mode("SMART_RTL")
        self.wait_disarmed()

    def GPSForYawCompassLearn(self):
        '''Moving baseline GPS yaw - with compass learning'''
        self.context_push()
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.set_parameter("EK3_SRC1_YAW", 3)  # GPS with compass fallback
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)

        self.wait_ready_to_arm()

        self.takeoff(10, mode='GUIDED')
        tstart = self.get_sim_time()
        compass_learn_set = False
        while True:
            delta_t = self.get_sim_time_cached() - tstart
            if delta_t > 30:
                break
            if not compass_learn_set and delta_t > 10:
                self.set_parameter("COMPASS_LEARN", 3)
                compass_learn_set = True

            self.check_attitudes_match()
            self.delay_sim_time(1)

        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

    def AP_Avoidance(self):
        '''ADSB-based avoidance'''
        self.set_parameters({
            "AVD_ENABLE": 1,
            "ADSB_TYPE": 1,  # mavlink
            "AVD_F_ACTION": 2,  # climb or descend
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()

        here = self.mav.location()

        self.context_push()

        self.start_subtest("F_ALT_MIN zero - disabled, can't arm in face of threat")
        self.set_parameters({
            "AVD_F_ALT_MIN": 0,
        })
        self.wait_ready_to_arm()
        self.test_adsb_send_threatening_adsb_message(here)
        self.delay_sim_time(1)
        self.try_arm(result=False,
                     expect_msg="ADSB threat detected")

        self.wait_ready_to_arm(timeout=60)

        self.context_pop()

        self.start_subtest("F_ALT_MIN 16m relative - arm in face of threat")
        self.context_push()
        self.set_parameters({
            "AVD_F_ALT_MIN": int(16 + here.alt),
        })
        self.wait_ready_to_arm()
        self.test_adsb_send_threatening_adsb_message(here)
#        self.delay_sim_time(1)
        self.arm_vehicle()
        self.disarm_vehicle()
        self.context_pop()

    def PAUSE_CONTINUE(self):
        '''Test MAV_CMD_DO_PAUSE_CONTINUE in AUTO mode'''
        self.load_mission(filename="copter_mission.txt", strict=False)
        self.set_parameter(name="AUTO_OPTIONS", value=3)
        self.change_mode(mode="AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_current_waypoint(wpnum=3, timeout=500)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()

        self.wait_current_waypoint(wpnum=4, timeout=500)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()

        # sending a pause, or resume, to the aircraft twice, doesn't result in reporting a failure
        self.wait_current_waypoint(wpnum=5, timeout=500)
        self.send_pause_command()
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()
        self.send_resume_command()

        self.wait_disarmed(timeout=500)

    def PAUSE_CONTINUE_GUIDED(self):
        '''Test MAV_CMD_DO_PAUSE_CONTINUE in GUIDED mode'''
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with LOCATION!")
        self.change_mode(mode="GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter(name="GUID_TIMEOUT", value=120)
        self.user_takeoff(alt_min=30)

        # send vehicle to global position target
        location = self.home_relative_loc_ne(n=300, e=0)
        target_typemask = MAV_POS_TARGET_TYPE_MASK.POS_ONLY
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # relative altitude frame
            target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # target typemask as pos only
            int(location.lat * 1e7), # lat
            int(location.lng * 1e7), # lon
            30, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0) # yawrate

        self.wait_distance_to_home(distance_min=100, distance_max=150, timeout=120)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()
        self.wait_location(loc=location, timeout=120)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with LOCATION!")
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with DESTINATION!")
        self.guided_achieve_heading(heading=270)

        # move vehicle on x direction
        location = self.offset_location_ne(location=self.mav.location(), metres_north=0, metres_east=-300)
        self.mav.mav.set_position_target_global_int_send(
            0, # system time in milliseconds
            1, # target system
            1, # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # coordinate frame MAV_FRAME_BODY_NED
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY, # type mask (pos only)
            int(location.lat*1e7), # position x
            int(location.lng*1e7), # position y
            30, # position z
            0, # velocity x
            0, # velocity y
            0, # velocity z
            0, # accel x
            0, # accel y
            0, # accel z
            0, # yaw
            0) # yaw rate

        self.wait_location(loc=location, accuracy=200, timeout=120)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()
        self.wait_location(loc=location, timeout=120)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with DESTINATION!")
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with VELOCITY!")

        # give velocity command
        vx, vy, vz_up = (5, 5, 0)
        self.test_guided_local_velocity_target(vx=vx, vy=vy, vz_up=vz_up, timeout=10)

        self.wait_for_local_velocity(vx=vx, vy=vy, vz_up=vz_up, timeout=10)
        self.send_pause_command()
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.send_resume_command()
        self.wait_for_local_velocity(vx=vx, vy=vy, vz_up=vz_up, timeout=10)
        self.test_guided_local_velocity_target(vx=0, vy=0, vz_up=0, timeout=10)
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with VELOCITY!")
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with ACCELERATION!")

        # give acceleration command
        ax, ay, az_up = (1, 1, 0)
        target_typemask = (MAV_POS_TARGET_TYPE_MASK.POS_IGNORE | MAV_POS_TARGET_TYPE_MASK.VEL_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
            0, # x
            0, # y
            0, # z
            0, # vx
            0, # vy
            0, # vz
            ax, # afx
            ay, # afy
            -az_up, # afz
            0, # yaw
            0, # yawrate
        )

        self.wait_for_local_velocity(vx=5, vy=5, vz_up=0, timeout=10)
        self.send_pause_command()
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.send_resume_command()
        self.wait_for_local_velocity(vx=5, vy=5, vz_up=0, timeout=10)
        self.test_guided_local_velocity_target(vx=0, vy=0, vz_up=0, timeout=10)
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with ACCELERATION!")

        # start pause/continue subtest with posvelaccel
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with POSITION and VELOCITY and ACCELERATION!")
        self.guided_achieve_heading(heading=0)

        # give posvelaccel command
        x, y, z_up = (-300, 0, 30)
        target_typemask = (MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
            x, # x
            y, # y
            -z_up, # z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )

        self.wait_distance_to_local_position(local_position=(x, y, -z_up), distance_min=400, distance_max=450, timeout=120)
        self.send_pause_command()
        self.wait_for_local_velocity(0, 0, 0, timeout=10)
        self.send_resume_command()
        self.wait_distance_to_local_position(local_position=(x, y, -z_up), distance_min=0, distance_max=10, timeout=120)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with POSITION and VELOCITY and ACCELERATION!")
        self.do_RTL(timeout=120)

    def DO_CHANGE_SPEED(self):
        '''Change speed during mission using waypoint items'''
        self.load_mission("mission.txt", strict=False)

        self.set_parameters({
            "AUTO_OPTIONS": 3,
            "ANGLE_MAX": 4500,
        })

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_current_waypoint(1)
        self.wait_groundspeed(
            3.5, 4.5,
            minimum_duration=5,
            timeout=60,
        )

        self.wait_current_waypoint(3)
        self.wait_groundspeed(
            14.5, 15.5,
            minimum_duration=10,
            timeout=60,
        )

        self.wait_current_waypoint(5)
        self.wait_groundspeed(
            9.5, 11.5,
            minimum_duration=10,
            timeout=60,
        )

        self.set_parameter("ANGLE_MAX", 6000)
        self.wait_current_waypoint(7)
        self.wait_groundspeed(
            15.5, 16.5,
            minimum_duration=10,
            timeout=60,
        )

        self.wait_disarmed()

    def AUTO_LAND_TO_BRAKE(self):
        '''ensure terrain altitude is taken into account when braking'''
        self.set_parameters({
            "PLND_ACC_P_NSE": 2.500000,
            "PLND_ALT_MAX": 8.000000,
            "PLND_ALT_MIN": 0.750000,
            "PLND_BUS": -1,
            "PLND_CAM_POS_X": 0.000000,
            "PLND_CAM_POS_Y": 0.000000,
            "PLND_CAM_POS_Z": 0.000000,
            "PLND_ENABLED": 1,
            "PLND_EST_TYPE": 1,
            "PLND_LAG": 0.020000,
            "PLND_LAND_OFS_X": 0.000000,
            "PLND_LAND_OFS_Y": 0.000000,
            "PLND_OPTIONS": 0,
            "PLND_RET_BEHAVE": 0,
            "PLND_RET_MAX": 4,
            "PLND_STRICT": 1,
            "PLND_TIMEOUT": 4.000000,
            "PLND_TYPE": 4,
            "PLND_XY_DIST_MAX": 2.500000,
            "PLND_YAW_ALIGN": 0.000000,

            "SIM_PLD_ALT_LMT": 15.000000,
            "SIM_PLD_DIST_LMT": 10.000000,
            "SIM_PLD_ENABLE": 1,
            "SIM_PLD_HEIGHT": 0,
            "SIM_PLD_LAT": -20.558929,
            "SIM_PLD_LON": -47.415035,
            "SIM_PLD_RATE": 100,
            "SIM_PLD_TYPE": 1,
            "SIM_PLD_YAW": 87,

            "SIM_SONAR_SCALE": 12,
        })

        self.set_analog_rangefinder_parameters()

        self.load_mission('mission.txt')
        self.customise_SITL_commandline([
            "--home", self.sitl_home_string_from_mission("mission.txt"),
        ])

        self.set_parameter('AUTO_OPTIONS', 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_current_waypoint(7)
        self.wait_altitude(10, 15, relative=True, timeout=60)
        self.change_mode('BRAKE')
        # monitor altitude here
        self.wait_altitude(10, 15, relative=True, minimum_duration=20)
        self.change_mode('AUTO')
        self.wait_disarmed()

    def MAVLandedStateTakeoff(self):
        '''check EXTENDED_SYS_STATE message'''
        ex = None
        try:
            self.set_message_rate_hz(id=mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, rate_hz=1)
            self.wait_extended_sys_state(vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                                         landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, timeout=10)
            self.load_mission(filename="copter_mission.txt")
            self.set_parameter(name="AUTO_OPTIONS", value=3)
            self.change_mode(mode="AUTO")
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.wait_extended_sys_state(vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                                         landed_state=mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF, timeout=30)
            self.wait_extended_sys_state(vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                                         landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, timeout=60)
            self.land_and_disarm()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, -1)
        if ex is not None:
            raise ex

    def ATTITUDE_FAST(self):
        '''ensure that when ATTITUDE_FAST is set we get many messages'''

        # enable fast logging (bit 1)
        # disable PID logging (bit 12) to avoid slowdowns in attitude logging in SITL
        log_bitmask_old = int(self.get_parameter('LOG_BITMASK'))
        log_bitmask_new = (log_bitmask_old | (1 << 0)) & ~(1 << 12)

        path = self.generate_rate_sample_log(log_bitmask=log_bitmask_new)

        self.check_dflog_message_rates(path, {
            'ANG': 400,
        })

    def BaseLoggingRates(self):
        '''ensure messages come out at specific rates'''
        path = self.generate_rate_sample_log()
        self.check_dflog_message_rates(path, {
            "ATT": 10,
            "IMU": 25,
        })

    def FETtecESC_flight(self):
        '''fly with servo outputs from FETtec ESC'''
        self.start_subtest("FETtec ESC flight")
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        self.fly_loaded_mission(num_wp)

    def FETtecESC_esc_power_checks(self):
        '''Make sure state machine copes with ESCs rebooting'''
        self.start_subtest("FETtec ESC reboot")
        self.wait_ready_to_arm()
        self.context_collect('STATUSTEXT')
        self.progress("Turning off an ESC off ")
        mask = int(self.get_parameter("SIM_FTOWESC_POW"))

        for mot_id_to_kill in 1, 2:
            self.progress("Turning ESC=%u off" % mot_id_to_kill)
            self.set_parameter("SIM_FTOWESC_POW", mask & ~(1 << mot_id_to_kill))
            self.delay_sim_time(1)
            self.assert_prearm_failure("are not running")
            self.progress("Turning it back on")
            self.set_parameter("SIM_FTOWESC_POW", mask)
            self.wait_ready_to_arm()

            self.progress("Turning ESC=%u off (again)" % mot_id_to_kill)
            self.set_parameter("SIM_FTOWESC_POW", mask & ~(1 << mot_id_to_kill))
            self.delay_sim_time(1)
            self.assert_prearm_failure("are not running")
            self.progress("Turning it back on")
            self.set_parameter("SIM_FTOWESC_POW", mask)
            self.wait_ready_to_arm()

        self.progress("Turning all ESCs off")
        self.set_parameter("SIM_FTOWESC_POW", 0)
        self.delay_sim_time(1)
        self.assert_prearm_failure("are not running")
        self.progress("Turning them back on")
        self.set_parameter("SIM_FTOWESC_POW", mask)
        self.wait_ready_to_arm()

    def fettec_assert_bad_mask(self, mask):
        '''assert the mask is bad for fettec driver'''
        self.start_subsubtest("Checking mask (%s) is bad" % (mask,))
        self.context_push()
        self.set_parameter("SERVO_FTW_MASK", mask)
        self.reboot_sitl()
        self.delay_sim_time(12)  # allow accels/gyros to be happy
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                raise NotAchievedException("Expected mask to be only problem within 20 seconds")
            try:
                self.assert_prearm_failure("Invalid motor mask")
                break
            except NotAchievedException:
                self.delay_sim_time(1)
        self.context_pop()
        self.reboot_sitl()

    def fettec_assert_good_mask(self, mask):
        '''assert the mask is bad for fettec driver'''
        self.start_subsubtest("Checking mask (%s) is good" % (mask,))
        self.context_push()
        self.set_parameter("SERVO_FTW_MASK", mask)
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.context_pop()
        self.reboot_sitl()

    def FETtecESC_safety_switch(self):
        mot = self.find_first_set_bit(int(self.get_parameter("SERVO_FTW_MASK"))) + 1
        self.wait_esc_telem_rpm(mot, 0, 0)
        self.wait_ready_to_arm()
        self.context_push()
        self.set_parameter("DISARM_DELAY", 0)
        self.arm_vehicle()
        # we have to wait for a while for the arming tone to go out
        # before the motors will spin:
        self.wait_esc_telem_rpm(
            esc=mot,
            rpm_min=17640,
            rpm_max=17640,
            minimum_duration=2,
            timeout=5,
        )
        self.set_safetyswitch_on()
        self.wait_esc_telem_rpm(mot, 0, 0)
        self.set_safetyswitch_off()
        self.wait_esc_telem_rpm(
            esc=mot,
            rpm_min=17640,
            rpm_max=17640,
            minimum_duration=2,
            timeout=5,
        )
        self.context_pop()
        self.wait_disarmed()

    def FETtecESC_btw_mask_checks(self):
        '''ensure prearm checks work as expected'''
        for bad_mask in [0b1000000000000000, 0b10100000000000000]:
            self.fettec_assert_bad_mask(bad_mask)
        for good_mask in [0b00001, 0b00101, 0b110000000000]:
            self.fettec_assert_good_mask(good_mask)

    def FETtecESC(self):
        '''Test FETtecESC'''
        self.set_parameters({
            "SERIAL5_PROTOCOL": 38,
            "SERVO_FTW_MASK": 0b11101000,
            "SIM_FTOWESC_ENA": 1,
            "SERVO1_FUNCTION": 0,
            "SERVO2_FUNCTION": 0,
            "SERVO3_FUNCTION": 0,
            "SERVO4_FUNCTION": 33,
            "SERVO5_FUNCTION": 0,
            "SERVO6_FUNCTION": 34,
            "SERVO7_FUNCTION": 35,
            "SERVO8_FUNCTION": 36,
            "SIM_ESC_TELEM": 0,
        })
        self.customise_SITL_commandline(["--serial5=sim:fetteconewireesc"])
        self.FETtecESC_safety_switch()
        self.FETtecESC_esc_power_checks()
        self.FETtecESC_btw_mask_checks()
        self.FETtecESC_flight()

    def PerfInfo(self):
        '''Test Scheduler PerfInfo output'''
        self.set_parameter('SCHED_OPTIONS', 1)  # enable gathering
        # sometimes we need to trigger collection....
        content = self.fetch_file_via_ftp("@SYS/tasks.txt")
        self.delay_sim_time(5)
        content = self.fetch_file_via_ftp("@SYS/tasks.txt")
        self.progress("Got content (%s)" % str(content))

        lines = content.split("\n")

        if not lines[0].startswith("TasksV2"):
            raise NotAchievedException("Expected TasksV2 as first line first not (%s)" % lines[0])
        # last line is empty, so -2 here
        if not lines[-2].startswith("AP_Vehicle::update_arming"):
            raise NotAchievedException("Expected EFI last not (%s)" % lines[-2])

    def RTL_TO_RALLY(self, target_system=1, target_component=1):
        '''Check RTL to rally point'''
        self.wait_ready_to_arm()
        rally_loc = self.home_relative_loc_ne(50, -25)
        rally_alt = 37
        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(rally_loc.lat * 1e7), # latitude
                int(rally_loc.lng * 1e7), # longitude
                rally_alt, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
        ]
        self.upload_using_mission_protocol(
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
            items
        )
        self.set_parameters({
            'RALLY_INCL_HOME': 0,
        })
        self.takeoff(10)
        self.change_mode('RTL')
        self.wait_location(rally_loc)
        self.assert_altitude(rally_alt, relative=True)
        self.progress("Ensuring we're descending")
        self.wait_altitude(20, 25, relative=True)
        self.change_mode('LOITER')
        self.progress("Flying home")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.change_mode('RTL')
        self.wait_disarmed()
        self.assert_at_home()

    def NoRCOnBootPreArmFailure(self):
        '''Ensure we can't arm with no RC on boot if THR_FS_VALUE set'''
        self.context_push()
        for rc_failure_mode in 1, 2:
            self.set_parameters({
                "SIM_RC_FAIL": rc_failure_mode,
            })
            self.reboot_sitl()
            if rc_failure_mode == 1:
                self.assert_prearm_failure("RC not found",
                                           other_prearm_failures_fatal=False)
            elif rc_failure_mode == 2:
                self.assert_prearm_failure("Throttle below failsafe",
                                           other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()

    def IMUConsistency(self):
        '''test IMUs must be consistent with one another'''
        self.wait_ready_to_arm()

        self.start_subsubtest("prearm checks for accel inconsistency")
        self.context_push()
        self.set_parameters({
            "SIM_ACC1_BIAS_X": 5,
        })
        self.assert_prearm_failure("Accels inconsistent")
        self.context_pop()
        tstart = self.get_sim_time()
        self.wait_ready_to_arm()
        if self.get_sim_time() - tstart < 8:
            raise NotAchievedException("Should take 10 seconds to become armableafter IMU upset")

        self.start_subsubtest("prearm checks for gyro inconsistency")
        self.context_push()
        self.set_parameters({
            "SIM_GYR1_BIAS_X": math.radians(10),
        })
        self.assert_prearm_failure("Gyros inconsistent")
        self.context_pop()
        tstart = self.get_sim_time()
        self.wait_ready_to_arm()
        if self.get_sim_time() - tstart < 8:
            raise NotAchievedException("Should take 10 seconds to become armableafter IMU upset")

    def Sprayer(self):
        """Test sprayer functionality."""
        self.context_push()

        rc_ch = 9
        pump_ch = 5
        spinner_ch = 6
        pump_ch_min = 1050
        pump_ch_trim = 1520
        pump_ch_max = 1950
        spinner_ch_min = 975
        spinner_ch_trim = 1510
        spinner_ch_max = 1975

        self.set_parameters({
            "SPRAY_ENABLE": 1,

            "SERVO%u_FUNCTION" % pump_ch: 22,
            "SERVO%u_MIN" % pump_ch: pump_ch_min,
            "SERVO%u_TRIM" % pump_ch: pump_ch_trim,
            "SERVO%u_MAX" % pump_ch: pump_ch_max,

            "SERVO%u_FUNCTION" % spinner_ch: 23,
            "SERVO%u_MIN" % spinner_ch: spinner_ch_min,
            "SERVO%u_TRIM" % spinner_ch: spinner_ch_trim,
            "SERVO%u_MAX" % spinner_ch: spinner_ch_max,

            "SIM_SPR_ENABLE": 1,
            "SIM_SPR_PUMP": pump_ch,
            "SIM_SPR_SPIN": spinner_ch,

            "RC%u_OPTION" % rc_ch: 15,
            "LOG_DISARMED": 1,
        })

        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.progress("test boot-up state - it's zero-output!")
        self.wait_servo_channel_value(spinner_ch, 0)
        self.wait_servo_channel_value(pump_ch, 0)

        self.progress("Enable sprayer")
        self.set_rc(rc_ch, 2000)

        self.progress("Testing zero-speed state")
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing turning it off")
        self.set_rc(rc_ch, 1000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing turning it back on")
        self.set_rc(rc_ch, 2000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.takeoff(30, mode='LOITER')

        self.progress("Testing speed-ramping")
        self.set_rc(1, 1700) # start driving forward

        # this is somewhat empirical...
        self.wait_servo_channel_value(
            pump_ch,
            1458,
            timeout=60,
            comparator=lambda x, y : abs(x-y) < 5
        )

        self.progress("Turning it off again")
        self.set_rc(rc_ch, 1000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.start_subtest("Checking mavlink commands")
        self.progress("Starting Sprayer")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SPRAYER, p1=1)

        self.progress("Testing speed-ramping")
        self.wait_servo_channel_value(
            pump_ch,
            1458,
            timeout=60,
            comparator=lambda x, y : abs(x-y) < 5
        )

        self.start_subtest("Stopping Sprayer")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SPRAYER, p1=0)

        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.disarm_vehicle(force=True)

        self.context_pop()

        self.reboot_sitl()

        self.progress("Sprayer OK")

    def tests1a(self):
        '''return list of all tests'''
        ret = super(AutoTestCopter, self).tests()  # about 5 mins and ~20 initial tests from autotest/vehicle_test_suite.py
        ret.extend([
             self.NavDelayTakeoffAbsTime,
             self.NavDelayAbsTime,
             self.NavDelay,
             self.GuidedSubModeChange,
             self.MAV_CMD_CONDITION_YAW,
             self.LoiterToAlt,
             self.PayloadPlaceMission,
             self.PayloadPlaceMissionOpenGripper,
             self.PrecisionLoiterCompanion,
             self.Landing,
             self.PrecisionLanding,
             self.SetModesViaModeSwitch,
             self.BackupFence,
             self.SetModesViaAuxSwitch,
             self.AuxSwitchOptions,
             self.AuxFunctionsInMission,
             self.AutoTune,
             self.AutoTuneYawD,
             self.NoRCOnBootPreArmFailure,
        ])
        return ret

    def tests1b(self):
        '''return list of all tests'''
        ret = ([
             self.ThrowMode,
             self.BrakeMode,
             self.RecordThenPlayMission,
             self.ThrottleFailsafe,
             self.ThrottleFailsafePassthrough,
             self.GCSFailsafe,
             self.CustomController,
             self.WPArcs,
             self.WPArcs2,
        ])
        return ret

    def tests1c(self):
        '''return list of all tests'''
        ret = ([
             self.BatteryFailsafe,
             self.BatteryMissing,
             self.VibrationFailsafe,
             self.EK3AccelBias,
             self.StabilityPatch,
             self.OBSTACLE_DISTANCE_3D,
             self.AC_Avoidance_Proximity,
             self.AC_Avoidance_Proximity_AVOID_ALT_MIN,
             self.AC_Avoidance_Fence,
             self.AC_Avoidance_Beacon,
             self.AvoidanceAltFence,
             self.BaroWindCorrection,
             self.SetpointGlobalPos,
             self.ThrowDoubleDrop,
             self.SetpointGlobalVel,
             self.SetpointBadVel,
             self.SplineTerrain,
             self.TakeoffCheck,
             self.GainBackoffTakeoff,
        ])
        return ret

    def tests1d(self):
        '''return list of all tests'''
        ret = ([
             self.HorizontalFence,
             self.HorizontalAvoidFence,
             self.MaxAltFence,
             self.MaxAltFenceAvoid,
             self.MinAltFence,
             self.MinAltFenceAvoid,
             self.FenceFloorEnabledLanding,
             self.FenceFloorAutoDisableLanding,
             self.FenceFloorAutoEnableOnArming,
             self.FenceMargin,
             self.FenceUpload_MissionItem,
             self.AutoTuneSwitch,
             self.AutoTuneAux,
             self.GPSGlitchLoiter,
             self.GPSGlitchLoiter2,
             self.GPSGlitchAuto,
             self.ModeAltHold,
             self.ModeLoiter,
             self.SimpleMode,
             self.SuperSimpleCircle,
             self.ModeCircle,
             self.MagFail,
             self.OpticalFlow,
             self.OpticalFlowLocation,
             self.OpticalFlowLimits,
             self.OpticalFlowCalibration,
             self.MotorFail,
             self.ModeFlip,
             self.CopterMission,
             self.TakeoffAlt,
             self.SplineLastWaypoint,
             self.Gripper,
             self.TestLocalHomePosition,
             self.TestGripperMission,
             self.VisionPosition,
             self.ATTITUDE_FAST,
             self.BaseLoggingRates,
             self.BodyFrameOdom,
             self.GPSViconSwitching,
        ])
        return ret

    def tests1e(self):
        '''return list of all tests'''
        ret = ([
             self.BeaconPosition,
             self.RTLSpeed,
             self.Mount,
             self.MountYawVehicleForMountROI,
             self.MAV_CMD_DO_MOUNT_CONTROL,
             self.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
             self.AutoYawDO_MOUNT_CONTROL,
             self.Button,
             self.ShipTakeoff,
             self.RangeFinder,
             self.BaroDrivers,
             self.SurfaceTracking,
             self.Parachute,
             self.ParameterChecks,
             self.ManualThrottleModeChange,
             self.MANUAL_CONTROL,
             self.ModeZigZag,
             self.PosHoldTakeOff,
             self.ModeFollow,
             self.ModeFollow_with_FOLLOW_TARGET,
             self.RangeFinderDrivers,
             self.FlyRangeFinderMAVlink,
             self.FlyRangeFinderSITL,
             self.RangeFinderDriversMaxAlt_LightwareSerial,
             self.RangeFinderDriversMaxAlt_AinsteinLRD1,
             self.RangeFinderDriversMaxAlt_AinsteinLRD1_v19,
             self.RangeFinderDriversLongRange,
             self.RangeFinderSITLLongRange,
             self.MaxBotixI2CXL,
             self.MAVProximity,
             self.ParameterValidation,
             self.AltTypes,
             self.PAUSE_CONTINUE,
             self.PAUSE_CONTINUE_GUIDED,
             self.RichenPower,
             self.IE24,
             self.LoweheiserAuto,
             self.LoweheiserManual,
             self.MAVLandedStateTakeoff,
             self.Weathervane,
             self.MAV_CMD_AIRFRAME_CONFIGURATION,
             self.MAV_CMD_NAV_LOITER_UNLIM,
             self.MAV_CMD_NAV_RETURN_TO_LAUNCH,
             self.MAV_CMD_NAV_VTOL_LAND,
             self.clear_roi,
             self.ReadOnlyDefaults,
        ])
        return ret

    def tests2a(self):
        '''return list of all tests'''
        ret = ([
            # something about SITLCompassCalibration appears to fail
            # this one, so we put it first:
            self.FixedYawCalibration,

            # we run this single 8min-and-40s test on its own, apart
            #   from requiring FixedYawCalibration right before it
            #   because without it, it fails to calibrate this
            #   autotest appears to interfere with
            #   FixedYawCalibration, no idea why.
            self.SITLCompassCalibration,
        ])
        return ret

    def ScriptMountPOI(self):
        '''test the MountPOI example script'''
        self.context_push()

        self.install_terrain_handlers_context()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "RC12_OPTION": 300,
        })
        self.setup_servo_mount()
        self.reboot_sitl()
        self.set_rc(6, 1300)
        self.install_applet_script_context('mount-poi.lua')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.context_collect('STATUSTEXT')
        self.set_rc(12, 2000)
        self.wait_statustext('POI.*-35.*149', check_context=True, regex=True)
        self.set_rc(12, 1000)

        self.context_pop()
        self.reboot_sitl()

    def ScriptMountAllModes(self):
        '''test location from all mount modes using the scripting backend'''

        # enable scripting and set mount type to scripting
        self.set_parameters({
            "SCR_ENABLE": 1,
            "MNT1_TYPE": 9,
        })
        self.reboot_sitl()

        # install get-target-location-mount-backend.lua script
        self.install_example_script_context('get-target-location-mount-backend.lua')
        self.reboot_sitl()
        self.takeoff(50)

        self.context_collect("STATUSTEXT")

        self.start_subtest("Test RETRACT (mode 0)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
        )
        self.wait_statustext("Mode 0: No target", check_context=True)

        self.start_subtest("Test NEUTRAL (mode 1)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL,
        )
        self.wait_statustext("Mode 1: No target", check_context=True)

        self.start_subtest("Test MAVLINK_TARGETING (mode 2)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
        )
        self.wait_statustext("Mode 2: No target", check_context=True)

        self.start_subtest("Test RC_TARGETING (mode 3)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
        )
        self.wait_statustext("Mode 3: No target", check_context=True)

        self.start_subtest("Test GPS_POINT (mode 4)")
        roi_lat = -35.363150
        roi_lng = 149.165320
        roi_alt = 580.0

        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            p5=int(roi_lat * 1e7),
            p6=int(roi_lng * 1e7),
            p7=roi_alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )

        expected_roi_lat = f"{roi_lat:.3f}"
        expected_roi_lng = f"{roi_lng:.3f}"

        self.wait_statustext(
            rf"Mode 4:.*{expected_roi_lat}.*{expected_roi_lng}",
            check_context=True,
            regex=True,
        )

        self.start_subtest("Test SYSID_TARGET (mode 5)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
            p1=self.mav.source_system,
        )
        # Single-vehicle SITL, SYSID_TARGET has no target to track
        self.wait_statustext("Mode 5: No target", check_context=True)

        self.start_subtest("Test HOME_LOCATION (mode 6)")
        loc_home = self.poll_home_position()

        expected_lat = f"{loc_home.latitude/1e7:.6f}"
        expected_lng = f"{loc_home.longitude/1e7:.6f}"

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION,
        )

        self.wait_statustext(
            rf"Mode 6:.*{expected_lat}.*{expected_lng}",
            check_context=True,
            regex=True,
        )

        self.do_RTL()

    def ScriptCopterPosOffsets(self):
        '''test the copter-posoffset.lua example script'''
        self.context_push()

        # enable scripting and arming/takingoff in Auto mode
        self.set_parameters({
            "SCR_ENABLE": 1,
            "AUTO_OPTIONS": 3,
            "RC12_OPTION": 300
        })
        self.reboot_sitl()

        # install copter-posoffset script
        self.install_example_script_context('copter-posoffset.lua')
        self.reboot_sitl()

        # create simple mission with a single takeoff command
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20)
        ])

        # switch to loiter to wait for position estimate (aka GPS lock)
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        # arm and takeoff in Auto mode
        self.change_mode('AUTO')
        self.arm_vehicle()

        # wait for vehicle to climb to at least 10m
        self.wait_altitude(8, 12, relative=True)

        # add position offset to East and confirm vehicle moves
        self.set_parameter("PSC_OFS_POS_E", 20)
        self.set_rc(12, 2000)
        self.wait_distance(18)

        # remove position offset and wait for vehicle to return home
        self.set_parameter("PSC_OFS_POS_E", 0)
        self.wait_distance_to_home(distance_min=0, distance_max=4, timeout=20)

        # add velocity offset and confirm vehicle moves
        self.set_parameter("PSC_OFS_VEL_N", 5)
        self.wait_groundspeed(4.8, 5.2, minimum_duration=5, timeout=20)

        # remove velocity offset and switch to RTL
        self.set_parameter("PSC_OFS_VEL_N", 0)
        self.set_rc(12, 1000)
        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

    def AHRSTrimLand(self):
        '''test land detector with significant AHRS trim'''
        self.context_push()
        self.set_parameters({
            "SIM_ACC_TRIM_X": 0.12,
            "AHRS_TRIM_X": 0.12,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.takeoff(alt_min=20, mode='LOITER')
        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

    def GainBackoffTakeoff(self):
        '''test gain backoff on takeoff'''
        self.context_push()
        self.progress("Test gains are fully backed off")
        self.set_parameters({
            "ATC_LAND_R_MULT": 0.0,
            "ATC_LAND_P_MULT": 0.0,
            "ATC_LAND_Y_MULT": 0.0,
            "GCS_PID_MASK" : 7,
            "LOG_BITMASK": 180222,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.change_mode('ALT_HOLD')

        class ValidatePDZero(vehicle_test_suite.TestSuite.MessageHook):
            '''asserts correct values in PID_TUNING'''

            def __init__(self, suite, axis):
                super(ValidatePDZero, self).__init__(suite)
                self.pid_tuning_count = 0
                self.p_sum = 0
                self.d_sum = 0
                self.i_sum = 0
                self.axis = axis

            def hook_removed(self):
                if self.pid_tuning_count == 0:
                    raise NotAchievedException("Did not get PID_TUNING")
                if self.i_sum == 0:
                    raise ValueError("I sum is zero")
                print(f"ValidatePDZero: PID_TUNING count: {self.pid_tuning_count}")

            def process(self, mav, m):
                if m.get_type() != 'PID_TUNING' or m.axis != self.axis:
                    return
                self.pid_tuning_count += 1
                self.p_sum += m.P
                self.d_sum += m.D
                self.i_sum += m.I
                if self.p_sum > 0:
                    raise ValueError("P sum is not zero")
                if self.d_sum > 0:
                    raise ValueError("D sum is not zero")

        self.progress("Check that PD values are zero")
        self.install_message_hook_context(ValidatePDZero(self, mavutil.mavlink.PID_TUNING_ROLL))
        self.install_message_hook_context(ValidatePDZero(self, mavutil.mavlink.PID_TUNING_PITCH))
        self.install_message_hook_context(ValidatePDZero(self, mavutil.mavlink.PID_TUNING_YAW))
        # until the context pop happens, all received PID_TUNINGS will be verified as good
        self.arm_vehicle()
        self.set_rc(3, 1500)
        self.delay_sim_time(2)
        self.set_rc(2, 1250)
        self.delay_sim_time(5)
        self.assert_receive_message('PID_TUNING', timeout=5)
        self.set_rc_default()
        self.zero_throttle()
        self.disarm_vehicle()
        self.context_pop()

        self.context_push()
        self.progress("Test gains are not backed off")
        self.set_parameters({
            "ATC_LAND_R_MULT": 1.0,
            "ATC_LAND_P_MULT": 1.0,
            "ATC_LAND_Y_MULT": 1.0,
            "GCS_PID_MASK" : 7,
            "LOG_BITMASK": 180222,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.change_mode('ALT_HOLD')

        class ValidatePDNonZero(vehicle_test_suite.TestSuite.MessageHook):
            '''asserts correct values in PID_TUNING'''

            def __init__(self, suite, axis):
                super(ValidatePDNonZero, self).__init__(suite)
                self.pid_tuning_count = 0
                self.p_sum = 0
                self.d_sum = 0
                self.i_sum = 0
                self.axis = axis

            def hook_removed(self):
                if self.pid_tuning_count == 0:
                    raise NotAchievedException("Did not get PID_TUNING")
                if self.p_sum == 0:
                    raise ValueError("P sum is zero")
                if self.i_sum == 0:
                    raise ValueError("I sum is zero")
                print(f"ValidatePDNonZero: PID_TUNING count: {self.pid_tuning_count}")

            def process(self, mav, m):
                if m.get_type() != 'PID_TUNING' or m.axis != self.axis:
                    return
                self.pid_tuning_count += 1
                self.p_sum += m.P
                self.d_sum += m.D
                self.i_sum += m.I

        self.progress("Check that PD values are non-zero")
        self.install_message_hook_context(ValidatePDNonZero(self, mavutil.mavlink.PID_TUNING_ROLL))
        self.install_message_hook_context(ValidatePDNonZero(self, mavutil.mavlink.PID_TUNING_PITCH))
        self.install_message_hook_context(ValidatePDNonZero(self, mavutil.mavlink.PID_TUNING_YAW))
        # until the context pop happens, all received PID_TUNINGS will be verified as good
        self.arm_vehicle()
        self.set_rc(3, 1500)
        self.delay_sim_time(2)
        self.set_rc(2, 1250)
        self.delay_sim_time(5)
        self.assert_receive_message('PID_TUNING', timeout=5)
        self.set_rc_default()
        self.zero_throttle()
        self.disarm_vehicle()

        self.context_pop()
        self.reboot_sitl()

    def SafetySwitch(self):
        '''test safety switch behaviour'''
        self.wait_ready_to_arm()

        self.set_safetyswitch_on()
        self.assert_prearm_failure("safety switch")

        self.set_safetyswitch_off()
        self.wait_ready_to_arm()

        self.takeoff(2, mode='LOITER')
        self.set_safetyswitch_on()

        self.wait_servo_channel_value(1, 0)
        self.set_safetyswitch_off()

        self.change_mode('LAND')
        self.wait_disarmed()

        # test turning safety on/off using explicit MAVLink command:
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, mavutil.mavlink.SAFETY_SWITCH_STATE_SAFE)
        self.assert_prearm_failure("safety switch")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, mavutil.mavlink.SAFETY_SWITCH_STATE_DANGEROUS)
        self.wait_ready_to_arm()

    def ArmSwitchAfterReboot(self):
        '''test that the arming switch does not trigger after a reboot'''
        self.wait_ready_to_arm()
        self.set_parameters({
            "RC8_OPTION": 153,
        })
        self.set_rc(8, 2000)
        self.wait_armed()
        self.disarm_vehicle()
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                break
            if self.armed():
                raise NotAchievedException("Armed after reboot with switch high")
            armmsg = self.statustext_in_collections('Arm: ')
            if armmsg is not None:
                raise NotAchievedException("statustext(%s) means we tried to arm" % armmsg.text)
        self.progress("Did not arm via arming switfch after a reboot")

    def GuidedYawRate(self):
        '''ensuer guided yaw rate is not affected by rate of sewt-attitude messages'''
        self.takeoff(30, mode='GUIDED')
        rates = {}
        for rate in 1, 10:
            # command huge yaw rate for a while
            tstart = self.get_sim_time()
            interval = 1/rate
            yawspeed_rads_sum = 0
            yawspeed_rads_count = 0
            last_sent = 0
            while True:
                self.drain_mav()
                tnow = self.get_sim_time_cached()
                if tnow - last_sent > interval:
                    self.do_yaw_rate(60)  # this is... unlikely
                    last_sent = tnow
                if tnow - tstart < 5:  # let it spin up to speed first
                    continue
                yawspeed_rads_sum += self.mav.messages['ATTITUDE'].yawspeed
                yawspeed_rads_count += 1
                if tnow - tstart > 15:  # 10 seconds of measurements
                    break
            yawspeed_degs = math.degrees(yawspeed_rads_sum / yawspeed_rads_count)
            rates[rate] = yawspeed_degs
            self.progress("Input rate %u hz: average yaw rate %f deg/s" % (rate, yawspeed_degs))

        if rates[10] < rates[1] * 0.95:
            raise NotAchievedException("Guided yaw rate slower for higher rate updates")

        self.do_RTL()

    def test_rplidar(self, sim_device_name, expected_distances):
        '''plonks a Copter with a RPLidarA2 in the middle of a simulated field
        of posts and checks that the measurements are what we expect.'''
        self.set_parameters({
            "SERIAL5_PROTOCOL": 11,
            "PRX1_TYPE": 5,
        })
        self.customise_SITL_commandline([
            "--serial5=sim:%s:" % sim_device_name,
            "--home", "51.8752066,14.6487840,0,0",  # SITL has "posts" here
        ])

        self.wait_ready_to_arm()

        wanting_distances = copy.copy(expected_distances)
        tstart = self.get_sim_time()
        timeout = 60
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not get all distances")
            m = self.mav.recv_match(type="DISTANCE_SENSOR",
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("Got (%s)" % str(m))
            if m.orientation not in wanting_distances:
                continue
            if abs(m.current_distance - wanting_distances[m.orientation]) > 5:
                self.progress("Wrong distance orient=%u want=%u got=%u" %
                              (m.orientation,
                               wanting_distances[m.orientation],
                               m.current_distance))
                continue
            self.progress("Correct distance for orient %u (want=%u got=%u)" %
                          (m.orientation,
                           wanting_distances[m.orientation],
                           m.current_distance))
            del wanting_distances[m.orientation]
            if len(wanting_distances.items()) == 0:
                break

    def RPLidarA2(self):
        '''test Raspberry Pi Lidar A2'''
        expected_distances = {
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 276,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1130,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 1286,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 626,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 971,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 762,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 792,
        }

        self.test_rplidar("rplidara2", expected_distances)

    def RPLidarA1(self):
        '''test Raspberry Pi Lidar A1'''
        return  # we don't send distances when too long?
        expected_distances = {
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 276,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 800,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 800,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 626,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 800,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 762,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 792,
        }

        self.test_rplidar("rplidara1", expected_distances)

    def BrakeZ(self):
        '''check jerk limit correct in Brake mode'''
        self.set_parameter('PSC_JERK_D', 3)
        self.takeoff(50, mode='GUIDED')
        vx, vy, vz_up = (0, 0, -1)
        self.test_guided_local_velocity_target(vx=vx, vy=vy, vz_up=vz_up, timeout=10)

        self.wait_for_local_velocity(vx=vx, vy=vy, vz_up=vz_up, timeout=10)
        self.change_mode('BRAKE')
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.land_and_disarm()

    def MISSION_START(self):
        '''test mavlink command MAV_CMD_MISSION_START'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 200),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('LOITER')
            self.set_current_waypoint(1)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.change_mode('AUTO')
            command(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_altitude(20, 1000, relative=True)
            self.change_mode('RTL')
            self.wait_disarmed()

    def DO_CHANGE_SPEED_in_guided(self):
        '''test Copter DO_CHANGE_SPEED handling in guided mode'''
        self.takeoff(20, mode='GUIDED')

        new_loc = self.mav.location()
        new_loc_offset_n = 2000
        new_loc_offset_e = 0
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)

        second_loc_offset_n = -1000
        second_loc_offset_e = 0
        second_loc = self.mav.location()
        self.location_offset_ne(second_loc, second_loc_offset_n, second_loc_offset_e)

        # for run_cmd we fly away from home
        for (tloc, command) in (new_loc, self.run_cmd), (second_loc, self.run_cmd_int):
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p1=-1,  # "default"
                p2=0,   # flags; none supplied here
                p3=0,   # loiter radius for planes, zero ignored
                p4=float("nan"),  # nan means do whatever you want to do
                p5=int(tloc.lat * 1e7),
                p6=int(tloc.lng * 1e7),
                p7=tloc.alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            )
            for speed in [2, 10, 4]:
                command(
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    p1=1,  # groundspeed,
                    p2=speed,
                    p3=-1,  # throttle, -1 is no-change
                    p4=0,   # absolute value, not relative
                )
                self.wait_groundspeed(speed-0.2, speed+0.2, minimum_duration=10, timeout=20)

        # we've made random changes to vehicle guided speeds above;
        # reboot vehicle to reset those:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def _MAV_CMD_DO_FLIGHTTERMINATION(self, command):
        self.set_parameters({
            "MAV_GCS_SYSID": self.mav.source_system,
            "DISARM_DELAY": 0,
        })
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        command(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, p1=1)
        self.wait_disarmed()
        self.reboot_sitl()

    def MAV_CMD_DO_FLIGHTTERMINATION(self):
        '''test MAV_CMD_DO_FLIGHTTERMINATION works on Copter'''
        self._MAV_CMD_DO_FLIGHTTERMINATION(self.run_cmd)
        self._MAV_CMD_DO_FLIGHTTERMINATION(self.run_cmd_int)

    def MAV_CMD_NAV_LOITER_UNLIM(self):
        '''ensure MAV_CMD_NAV_LOITER_UNLIM via mavlink works'''
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
            self.wait_mode('LOITER')

    def MAV_CMD_NAV_RETURN_TO_LAUNCH(self):
        '''ensure MAV_CMD_NAV_RETURN_TO_LAUNCH via mavlink works'''
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            self.wait_mode('RTL')

    def MAV_CMD_NAV_VTOL_LAND(self):
        '''ensure MAV_CMD_NAV_LAND via mavlink works'''
        for command in self.run_cmd, self.run_cmd_int:
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND)
            self.wait_mode('LAND')
            self.change_mode('STABILIZE')
            command(mavutil.mavlink.MAV_CMD_NAV_LAND)
            self.wait_mode('LAND')

    def clear_roi(self):
        '''ensure three commands that clear ROI are equivalent'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,    0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, 20), # directly North, i.e. 0 degrees
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 400, 0, 20), # directly North, i.e. 0 degrees
        ])

        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        home_loc = self.mav.location()

        cmd_ids = [
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE,
        ]
        for command in self.run_cmd, self.run_cmd_int:
            for cmd_id in cmd_ids:
                self.wait_waypoint(2, 2)

                # Set an ROI at the Home location, expect to point at Home
                self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION, p5=home_loc.lat, p6=home_loc.lng, p7=home_loc.alt)
                self.wait_heading(180)

                # Clear the ROI, expect to point at the next Waypoint
                self.progress("Clear ROI using %s(%d)" % (command.__name__, cmd_id))
                command(cmd_id)
                self.wait_heading(0)

                self.wait_waypoint(4, 4)
                self.set_current_waypoint_using_mav_cmd_do_set_mission_current(seq=2)

        self.land_and_disarm()

    def start_flying_simple_relhome_mission(self, items):
        '''uploads items, changes mode to auto, waits ready to arm and arms
        vehicle.  If the first item it a takeoff you can expect the
        vehicle to fly after this method returns
        '''

        self.upload_simple_relhome_mission(items)

        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

    def _MAV_CMD_DO_LAND_START(self, run_cmd):
        alt = 5
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0),
            (mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 2000, alt),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0),
        ])

        self.wait_current_waypoint(2)
        run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START)
        self.wait_current_waypoint(5)
        # we pretend to be in RTL mode while doing this:
        self.wait_mode("AUTO_RTL")
        self.do_RTL()

    def MAV_CMD_DO_LAND_START(self):
        '''test handling of mavlink-received MAV_CMD_DO_LAND_START command'''
        self._MAV_CMD_DO_LAND_START(self.run_cmd)
        self.zero_throttle()
        self._MAV_CMD_DO_LAND_START(self.run_cmd_int)

    def _MAV_CMD_SET_EKF_SOURCE_SET(self, run_cmd):
        run_cmd(
            mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET,
            17,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        run_cmd(mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, 2)

        self.assert_prearm_failure('Need Position Estimate')
        run_cmd(mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, 1)

        self.wait_ready_to_arm()

    def MAV_CMD_SET_EKF_SOURCE_SET(self):
        '''test setting of source sets using mavlink command'''
        self._MAV_CMD_SET_EKF_SOURCE_SET(self.run_cmd)
        self._MAV_CMD_SET_EKF_SOURCE_SET(self.run_cmd_int)

    def MAV_CMD_NAV_TAKEOFF(self):
        '''test issuing takeoff command via mavlink'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=5)
        self.wait_altitude(4.5, 5.5, minimum_duration=5, relative=True)
        self.change_mode('LAND')
        self.wait_disarmed()

        self.start_subtest("Check NAV_TAKEOFF is above home location, not current location")
        # reset home 20 metres above current location
        current_alt_abs = self.get_altitude(relative=False)

        loc = self.mav.location()

        home_z_ofs = 20
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            p5=loc.lat,
            p6=loc.lng,
            p7=current_alt_abs + home_z_ofs,
        )

        self.change_mode('GUIDED')
        self.arm_vehicle()
        takeoff_alt = 5
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=takeoff_alt)
        self.wait_altitude(
            current_alt_abs + home_z_ofs + takeoff_alt - 0.5,
            current_alt_abs + home_z_ofs + takeoff_alt + 0.5,
            minimum_duration=5,
            relative=False,
        )
        self.change_mode('LAND')
        self.wait_disarmed()

        self.reboot_sitl()  # unlock home position

    def MAV_CMD_NAV_TAKEOFF_command_int(self):
        '''test issuing takeoff command via mavlink and command_int'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        self.start_subtest("Check NAV_TAKEOFF is above home location, not current location")
        # reset home 20 metres above current location
        current_alt_abs = self.get_altitude(relative=False)

        loc = self.mav.location()

        home_z_ofs = 20
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            p5=loc.lat,
            p6=loc.lng,
            p7=current_alt_abs + home_z_ofs,
        )

        self.change_mode('GUIDED')
        self.arm_vehicle()
        takeoff_alt = 5
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=takeoff_alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            current_alt_abs + home_z_ofs + takeoff_alt - 0.5,
            current_alt_abs + home_z_ofs + takeoff_alt + 0.5,
            minimum_duration=5,
            relative=False,
        )
        self.change_mode('LAND')
        self.wait_disarmed()

        self.reboot_sitl()  # unlock home position

    def Ch6TuningWPSpeed(self):
        '''test waypoint speed can be changed via Ch6 tuning knob'''
        self.set_parameters({
            "RC6_OPTION": 219,  # RC6 used for tuning
            "TUNE": 10,  # 10 is waypoint speed
            "TUNE_MIN": 0.02,  # 20cm/s
            "TUNE_MAX": 1000,  # 10m/s
            "AUTO_OPTIONS": 3,
        })
        self.set_rc(6, 2000)
        self.reboot_sitl()

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')

        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1500)
        self.wait_groundspeed(4.5, 5.5, minimum_duration=5)

        self.set_rc(6, 2000)
        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1300)
        self.wait_groundspeed(2.5, 3.5, minimum_duration=5)

        self.do_RTL()

    def Ch6TuningLoitMaxXYSpeed(self):
        '''test loiter can be changed via Ch6 tuning knob'''
        self.set_parameters({
            "RC6_OPTION": 219,  # RC6 used for tuning
            "TUNE": 60,  # 60 is x/y loiter speed
            "TUNE_MIN": 0.02,  # 20cm/s
            "TUNE_MAX": 1000,  # 10m/s
            "AUTO_OPTIONS": 3,
        })
        self.set_rc(6, 2000)
        self.reboot_sitl()

        self.takeoff(mode='LOITER')

        self.set_rc(2, 1000)

        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1500)
        self.wait_groundspeed(4.5, 5.5, minimum_duration=5)

        self.set_rc(6, 2000)
        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1300)
        self.wait_groundspeed(2.5, 3.5, minimum_duration=5)

        self.set_rc(2, 1500)

        self.do_RTL()

    def DualTuningChannels(self):
        '''check tuning channel options work independently'''
        RC6_MIN = 1100    # yes, this is strange!
        RC6_MAX = 1200    # yes, this is strange!
        TUNE_MIN = 25      # cm/s
        TUNE_MAX = 1000   # cm/s

        RC7_MIN = 1000
        RC7_MAX = 2000
        TUNE2_MIN = 100  # cm/s
        TUNE2_MAX = 500  # cm/s
        self.set_parameters({
            "RC6_OPTION": 219,  # RC6 used for tuning
            "RC6_MIN": RC6_MIN,
            "RC6_MAX": RC6_MAX,
            "TUNE": 60,         # 60 is x/y loiter speed
            "TUNE_MIN": TUNE_MIN,
            "TUNE_MAX": TUNE_MAX,

            "RC7_OPTION": 220,  # RC7 used for second param tuning
            "RC7_MIN": RC7_MIN,
            "RC7_MAX": RC7_MAX,
            "TUNE2": 28,        # PSC_NE_VEL_I
            "TUNE2_MIN":  TUNE2_MIN,
            "TUNE2_MAX":  TUNE2_MAX,
        })

        self.reboot_sitl()

        self.set_rc(6, RC6_MIN)
        self.delay_sim_time(1)
        self.assert_parameter_value("LOIT_SPEED", TUNE_MIN)

        self.set_rc(6, RC6_MAX)
        self.delay_sim_time(1)
        self.assert_parameter_value("LOIT_SPEED", TUNE_MAX)

        self.set_rc(6, RC6_MIN)
        self.delay_sim_time(1)
        self.assert_parameter_value("LOIT_SPEED", TUNE_MIN)

        self.set_rc(6, int((RC6_MIN+RC6_MAX)/2))
        self.delay_sim_time(1)
        # note that this check is also used below ("RC6 is unaffected")
        self.assert_parameter_value("LOIT_SPEED", int((TUNE_MIN+TUNE_MAX)/2), epsilon=1)

        self.set_rc(7, RC7_MIN)
        self.delay_sim_time(1)
        self.assert_parameter_value("PSC_NE_VEL_I", TUNE2_MIN)

        self.set_rc(7, RC7_MAX)
        self.delay_sim_time(1)
        self.assert_parameter_value("PSC_NE_VEL_I", TUNE2_MAX)

        # make sure RC6 is unaffected:
        self.assert_parameter_value("LOIT_SPEED", int((TUNE_MIN+TUNE_MAX)/2), epsilon=1)

        self.set_rc(7, int((RC7_MIN+RC7_MAX)/2))
        self.delay_sim_time(1)
        self.assert_parameter_value("PSC_NE_VEL_I", int((TUNE2_MIN+TUNE2_MAX)/2), epsilon=1)

    def PILOT_THR_BHV(self):
        '''test the PILOT_THR_BHV parameter'''
        self.start_subtest("Test default behaviour, no disarm on land")
        self.set_parameters({
            "DISARM_DELAY": 0,
        })
        self.takeoff(2, mode='GUIDED')
        self.set_rc(3, 1500)
        self.change_mode('LOITER')
        self.set_rc(3, 1300)

        maintain_armed = WaitAndMaintainArmed(self, minimum_duration=20)
        maintain_armed.run()

        self.start_subtest("Test THR_BEHAVE_DISARM_ON_LAND_DETECT")
        self.set_parameters({
            "PILOT_THR_BHV": 4,  # Disarm on land detection
        })
        self.zero_throttle()
        self.takeoff(2, mode='GUIDED')
        self.set_rc(3, 1500)
        self.change_mode('LOITER')
        self.set_rc(3, 1300)

        self.wait_disarmed()

    def CameraLogMessages(self):
        '''ensure Camera log messages are good'''
        self.set_parameter("RC12_OPTION", 9) # CameraTrigger
        self.set_parameter("CAM1_TYPE", 1)   # Camera with servo trigger
        self.reboot_sitl() # needed for RC12_OPTION to take effect

        gpis = []
        gps_raws = []
        attitudes = []

        self.takeoff(10, mode='GUIDED')
        self.set_rc(12, 2000)
        gpis.append(self.assert_receive_message('GLOBAL_POSITION_INT'))
        gps_raws.append(self.assert_receive_message('GPS_RAW_INT'))
        attitudes.append(self.assert_receive_message('ATTITUDE'))
        self.set_rc(12, 1000)

        self.fly_guided_move_local(0, 0, 20)

        self.set_rc(12, 2000)
        gpis.append(self.assert_receive_message('GLOBAL_POSITION_INT'))
        gps_raws.append(self.assert_receive_message('GPS_RAW_INT'))
        attitudes.append(self.assert_receive_message('ATTITUDE'))
        self.set_rc(12, 1000)

        self.change_mode('LOITER')
        self.set_rc(1, 1000)
        self.set_rc(2, 1000)
        self.delay_sim_time(5)
        self.set_rc(12, 2000)
        gpis.append(self.assert_receive_message('GLOBAL_POSITION_INT'))
        gps_raws.append(self.assert_receive_message('GPS_RAW_INT'))
        attitudes.append(self.assert_receive_message('ATTITUDE'))
        self.set_rc(12, 1000)
        self.set_rc(1, 1500)
        self.set_rc(2, 1500)

        dfreader = self.dfreader_for_current_onboard_log()
        self.do_RTL()

        for i in range(len(gpis)):
            gpi = gpis[i]
            gps_raw = gps_raws[i]
            attitude = attitudes[i]
            m = dfreader.recv_match(type="CAM")

            things = [
                ["absalt", gpi.alt*0.001, m.Alt],
                ["relalt", gpi.relative_alt*0.001, m.RelAlt],
                ["gpsalt", gps_raw.alt*0.001, m.GPSAlt],  # use GPS_RAW here?
                ["R", math.degrees(attitude.roll), m.R],
                ["P", math.degrees(attitude.pitch), m.P],
                ["Y", mavextra.wrap_360(math.degrees(attitude.yaw)), m.Y],
            ]
            for (name, want, got) in things:
                if abs(got - want) > 1:
                    raise NotAchievedException(f"Incorrect {name} {want=} {got=}")
                self.progress(f"{name} {want=} {got=}")

            want = gpi.relative_alt*0.001
            got = m.RelAlt
            if abs(got - want) > 1:
                raise NotAchievedException(f"Incorrect relalt {want=} {got=}")

    def LoiterToGuidedHomeVSOrigin(self):
        '''test moving from guided to loiter mode when home is a different alt
        to origin'''
        self.set_parameters({
            "TERRAIN_ENABLE": 1,
            "SIM_TERRAIN": 1,
        })
        self.takeoff(10, mode='GUIDED')
        here = self.mav.location()
        self.set_home(here)
        self.change_mode('LOITER')
        self.wait_altitude(here.alt-1, here.alt+1, minimum_duration=10)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()  # to "unstick" home

    def GuidedModeThrust(self):
        '''test handling of option-bit-3, where mavlink commands are
        intrepreted as thrust not climb rate'''
        self.set_parameter('GUID_OPTIONS', 8)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mav.mav.set_attitude_target_send(
            0, # time_boot_ms
            1, # target sysid
            1, # target compid
            0, # bitmask of things to ignore
            mavextra.euler_to_quat([0, 0, 0]), # att
            0, # roll rate  (rad/s)
            0, # pitch rate (rad/s)
            0, # yaw rate   (rad/s)
            0.5
        ) # thrust, 0 to 1
        self.wait_altitude(0.5, 100, relative=True, timeout=10)
        self.do_RTL()

    def AutoRTL(self):
        '''Test Auto RTL mode using do land start and return path start mission items'''
        alt = 50
        guided_loc = self.home_relative_loc_ne(1000, 0)
        guided_loc.alt += alt

        # Arm, take off and fly to guided location
        self.takeoff(mode='GUIDED')
        self.fly_guided_move_to(guided_loc, timeout=240)

        # Try auto RTL mode, should fail with no mission
        try:
            self.change_mode('AUTO_RTL', timeout=10)
            raise NotAchievedException("Should not change mode with no mission")
        except WaitModeTimeout:
            pass
        except Exception as e:
            raise e

        # pymavlink does not understand the new return path command yet, at some point it will
        cmd_return_path_start = 188 # mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START

        # Do land start and do return path should both fail as commands too
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # Load mission with do land start
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, alt), # 1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  750, 0, alt), # 2
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START), # 3
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, alt),  # 4
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 250, 0, alt),  # 5
        ])

        # Return path should still fail
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # Do land start should jump to the waypoint following the item
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(4)

        # Back to guided location
        self.change_mode('GUIDED')
        self.fly_guided_move_to(guided_loc)

        # mode change to Auto RTL should do the same
        self.change_mode('AUTO_RTL')
        self.drain_mav()
        self.assert_current_waypoint(4)

        # Back to guided location
        self.change_mode('GUIDED')
        self.fly_guided_move_to(guided_loc)

        # Add a return path item
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1250, 0, alt), # 1
            self.create_MISSION_ITEM_INT(cmd_return_path_start),  # 2
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 900, 0, alt),  # 3
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 750, 0, alt),  # 4
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 550, 0, alt),  # 5
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, alt),  # 6
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START), # 7
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  250, 0, alt), # 8
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -250, 0, alt), # 9
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -500, 0, alt), # 10
        ])

        # Return path should now work
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(3)

        # Back to guided location
        self.change_mode('GUIDED')
        self.fly_guided_move_to(guided_loc)

        # mode change to Auto RTL should join the return path
        self.change_mode('AUTO_RTL')
        self.drain_mav()
        self.assert_current_waypoint(3)

        # do land start should still work
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(8)

        # Move a bit closer in guided
        return_path_test = self.home_relative_loc_ne(600, 0)
        return_path_test.alt += alt
        self.change_mode('GUIDED')
        self.fly_guided_move_to(return_path_test, timeout=100)

        # check the mission is joined further along
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(5)

        # fly over home
        home = self.home_relative_loc_ne(0, 0)
        home.alt += alt
        self.change_mode('GUIDED')
        self.fly_guided_move_to(home, timeout=140)

        # Should never join return path after do land start
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(6)

        # Done
        self.land_and_disarm()

    def EK3_OGN_HGT_MASK(self):
        '''test baraometer-alt-compensation based on long-term GPS readings'''
        self.context_push()
        self.set_parameters({
            'EK3_OGN_HGT_MASK': 1,  # compensate baro drift using GPS
        })
        self.reboot_sitl()

        expected_alt = 10

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        current_alt = self.get_altitude()

        expected_alt_abs = current_alt + expected_alt

        self.takeoff(expected_alt, mode='GUIDED')
        self.delay_sim_time(5)

        self.set_parameter("SIM_BARO_DRIFT", 0.01)  # 1cm/second

        def check_altitude(mav, m):
            m_type = m.get_type()
            epsilon = 10  # in metres
            if m_type == 'GPS_RAW_INT':
                got_gps_alt = m.alt * 0.001
                if abs(expected_alt_abs - got_gps_alt) > epsilon:
                    raise NotAchievedException(f"Bad GPS altitude (got={got_gps_alt} want={expected_alt_abs})")
            elif m_type == 'GLOBAL_POSITION_INT':
                got_canonical_alt = m.relative_alt * 0.001
                if abs(expected_alt - got_canonical_alt) > epsilon:
                    raise NotAchievedException(f"Bad canonical altitude (got={got_canonical_alt} want={expected_alt})")

        self.install_message_hook_context(check_altitude)

        self.delay_sim_time(1500)

        self.disarm_vehicle(force=True)

        self.context_pop()

        self.reboot_sitl(force=True)

    def GuidedForceArm(self):
        '''ensure Guided acts appropriately when force-armed'''
        self.set_parameters({
            "EK3_SRC2_VELXY": 5,
            "SIM_GPS1_ENABLE": 0,
        })
        self.load_default_params_file("copter-optflow.parm")
        self.reboot_sitl()
        self.delay_sim_time(30)
        self.change_mode('GUIDED')
        self.arm_vehicle(force=True)
        self.takeoff(20, mode='GUIDED')
        location = self.offset_location_ne(self.sim_location(), metres_north=0, metres_east=-300)
        self.progress("Ensure we don't move for 10 seconds")
        tstart = self.get_sim_time()
        startpos = self.sim_location_int()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                break
            self.send_set_position_target_global_int(int(location.lat*1e7), int(location.lng*1e7), 10)
            dist = self.get_distance_int(startpos, self.sim_location_int())
            if dist > 10:
                raise NotAchievedException("Wandered too far from start position")
            self.delay_sim_time(1)

        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def EK3_OGN_HGT_MASK_climbing(self):
        '''check combination of height bits doesn't cause climb'''
        self.context_push()
        self.set_parameters({
            'EK3_OGN_HGT_MASK': 5,
        })
        self.reboot_sitl()

        expected_alt = 10

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        current_alt = self.get_altitude()

        expected_alt_abs = current_alt + expected_alt

        self.takeoff(expected_alt, mode='GUIDED')
        self.delay_sim_time(5)

        def check_altitude(mav, m):
            m_type = m.get_type()
            epsilon = 10  # in metres
            if m_type == 'GPS_RAW_INT':
                got_gps_alt = m.alt * 0.001
                if abs(expected_alt_abs - got_gps_alt) > epsilon:
                    raise NotAchievedException(f"Bad GPS altitude (got={got_gps_alt} want={expected_alt_abs})")
            elif m_type == 'GLOBAL_POSITION_INT':
                if abs(expected_alt - m.relative_alt * 0.001) > epsilon:
                    raise NotAchievedException("Bad canonical altitude")

        self.install_message_hook_context(check_altitude)

        self.delay_sim_time(1500)

        self.disarm_vehicle(force=True)

        self.context_pop()
        self.reboot_sitl(force=True)

    def GuidedWeatherVane(self):
        '''check Copter Guided mode weathervane option'''
        self.set_parameters({
            'SIM_WIND_SPD': 10,
            'SIM_WIND_DIR': 90,
            'WVANE_ENABLE': 1,
        })
        self.takeoff(20, mode='GUIDED')
        self.guided_achieve_heading(0)

        self.set_parameter("GUID_OPTIONS", 128)
        self.wait_heading(90, timeout=60, minimum_duration=10)
        self.do_RTL()

    def Clamp(self):
        '''test Copter docking clamp'''
        clamp_ch = 11
        self.set_parameters({
            "SIM_CLAMP_CH": clamp_ch,
        })

        self.takeoff(1, mode='LOITER')

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.progress("Ensure can't take off with clamp in place")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: grabbed vehicle", check_context=True)
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(0, 5, minimum_duration=5, relative=True)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=1000)
        self.wait_statustext("SITL: Clamp: released vehicle", check_context=True)
        self.wait_altitude(5, 5000, minimum_duration=1, relative=True)
        self.do_RTL()
        self.set_rc(3, 1000)
        self.change_mode('LOITER')
        self.context_pop()

        self.progress("Same again for repeatability")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: grabbed vehicle", check_context=True)
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(0, 1, minimum_duration=5, relative=True)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=1000)
        self.wait_statustext("SITL: Clamp: released vehicle", check_context=True)
        self.wait_altitude(5, 5000, minimum_duration=1, relative=True)
        self.do_RTL()
        self.set_rc(3, 1000)
        self.change_mode('LOITER')
        self.context_pop()

        here = self.mav.location()
        loc = self.offset_location_ne(here, 10, 0)
        self.takeoff(5, mode='GUIDED')
        self.send_do_reposition(loc, frame=mavutil.mavlink.MAV_FRAME_GLOBAL)
        self.wait_location(loc, timeout=120)
        self.land_and_disarm()

        # explicitly set home so we RTL to the right spot
        self.set_home(here)

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: missed vehicle", check_context=True)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=1000)
        self.context_pop()

        self.takeoff(5, mode='GUIDED')
        self.do_RTL()

        self.takeoff(5, mode='GUIDED')
        self.land_and_disarm()

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: grabbed vehicle", check_context=True)
        self.context_pop()

        self.reboot_sitl()  # because we set home

    def GripperReleaseOnThrustLoss(self):
        '''tests that gripper is released on thrust loss if option set'''

        self.context_push()
        self.set_servo_gripper_parameters()
        self.reboot_sitl()

        self.takeoff(30, mode='LOITER')
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SIM_ENGINE_MUL": 0.5,
            "SIM_ENGINE_FAIL": 1 << 1, # motor 2
            "FLIGHT_OPTIONS": 4,
        })

        self.wait_statustext("Gripper load released", regex=True, timeout=60)
        self.context_pop()

        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()

    def GripperInitialPosition(self):
        '''test gripper initial position'''

        self.set_servo_gripper_parameters()
        self.set_parameters({
            "RC8_OPTION": 19,  # gripper
        })
        grip_neutral = 1423
        self.set_parameter('GRIP_NEUTRAL', grip_neutral)

        # install a message hook which ensures that the output stays at neutral
        global seen_grip_neutral
        seen_grip_neutral = False

        def hook(mav, m):
            global seen_grip_neutral
            if m.get_type() != 'SERVO_OUTPUT_RAW':
                return
            if m.servo8_raw == 0:
                # initial value
                if seen_grip_neutral:
                    raise NotAchievedException("Saw 0 after seeing grip neutral")
                return
            if m.servo8_raw == grip_neutral:
                # expected value after autopilot boot
                seen_grip_neutral = True
                return
            raise NotAchievedException(f"Received bad value {m.servo8_raw=} for servo gripper")
        self.install_message_hook_context(hook)

        self.reboot_sitl()
        self.wait_ready_to_arm()

        if not seen_grip_neutral:
            raise NotAchievedException("Never saw grip neutral")

    def REQUIRE_LOCATION_FOR_ARMING(self):
        '''check AP_Arming::Option::REQUIRE_LOCATION_FOR_ARMING works'''
        self.context_push()
        self.set_parameters({
            "SIM_GPS1_NUMSATS": 3,  # EKF does not like < 6
        })
        self.reboot_sitl()
        self.change_mode('STABILIZE')
        self.wait_prearm_sys_status_healthy()
        self.assert_home_position_not_set()
        self.arm_vehicle()
        self.disarm_vehicle()
        self.change_mode('LOITER')
        self.assert_prearm_failure("waiting for home", other_prearm_failures_fatal=False)

        self.change_mode('STABILIZE')
        self.set_parameters({
            "ARMING_NEED_LOC": 1,
        })
        self.assert_prearm_failure("Need Position Estimate", other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()

    def AutoContinueOnRCFailsafe(self):
        '''check LOITER when entered after RC failsafe is ignored in auto'''
        self.set_parameters({
            "FS_OPTIONS": 1,  # 1 is "RC continue if in auto"
        })

        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 40, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 120, 0, 10),
        ])

        self.takeoff(mode='LOITER')
        self.set_rc(1, 1200)
        self.delay_sim_time(1)  # build up some pilot desired stuff
        self.change_mode('AUTO')
        self.wait_waypoint(2, 2, max_dist=3)
        self.set_parameters({
            'SIM_RC_FAIL': 1,
        })
#        self.set_rc(1, 1500)  # note we are still in RC fail!
        self.wait_waypoint(3, 3, max_dist=3)
        self.assert_mode_is('AUTO')
        self.change_mode('LOITER')
        self.wait_groundspeed(0, 0.1, minimum_duration=30, timeout=450)
        self.do_RTL()

    def MissionRTLYawBehaviour(self):
        '''check end-of-mission yaw behaviour'''
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })

        self.start_subtest("behaviour with WP_YAW_BEHAVE set to next-waypoint-except-RTL")
        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        original_heading = self.get_heading()
        if abs(original_heading) < 5:
            raise NotAchievedException(f"Bad original heading {original_heading}")
        self.arm_vehicle()
        self.wait_current_waypoint(3)
        self.wait_rtl_complete()
        self.wait_disarmed()
        if abs(self.get_heading()) > 5:
            raise NotAchievedException("Should have yaw zero without option")

        # must change out of auto and back in again to reset state machine:
        self.change_mode('LOITER')
        self.change_mode('AUTO')

        self.start_subtest("behaviour with WP_YAW_BEHAVE set to next-waypoint")
        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  0, 20, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameters({
            "WP_YAW_BEHAVIOR": 1,  # look at next waypoint (including in RTL)
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        original_heading = self.get_heading()
        if abs(original_heading) > 1:
            raise NotAchievedException("Bad original heading")
        self.arm_vehicle()
        self.wait_current_waypoint(3)
        self.wait_rtl_complete()
        self.wait_disarmed()
        new_heading = self.get_heading()
        if abs(new_heading - original_heading) > 5:
            raise NotAchievedException(f"Should return to original heading want={original_heading} got={new_heading}")

    def BatteryInternalUseOnly(self):
        '''batteries marked as internal use only should not appear over mavlink'''
        self.set_parameters({
            "BATT_MONITOR": 4,  # 4 is analog volt+curr
            "BATT2_MONITOR": 4,
        })
        self.reboot_sitl()
        self.wait_message_field_values('BATTERY_STATUS', {
            "id": 0,
        })
        self.wait_message_field_values('BATTERY_STATUS', {
            "id": 1,
        })
        self.progress("Making battery private")
        self.set_parameters({
            "BATT_OPTIONS": 256,
        })
        self.wait_message_field_values('BATTERY_STATUS', {
            "id": 1,
        })
        for i in range(10):
            self.assert_received_message_field_values('BATTERY_STATUS', {
                "id": 1
            })

    def MAV_CMD_MISSION_START_p1_p2(self):
        '''make sure we deny MAV_CMD_MISSION_START if either p1 or p2 non-zero'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
        ])
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            p1=1,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            p2=1,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            p1=1,
            p2=1,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

    def ScriptingAHRSSource(self):
        '''test ahrs-source.lua script'''
        self.install_example_script_context("ahrs-source.lua")
        self.set_parameters({
            "RC10_OPTION": 90,
            "SCR_ENABLE": 1,
            "SCR_USER1": 10,    # something else
            "SCR_USER2": 10,    # GPS something
            "SCR_USER3": 0.2,   # ExtNav innovation
        })
        self.set_rc(10, 2000)
        self.reboot_sitl()
        self.context_collect('STATUSTEXT')
        self.set_rc(10, 1000)
        self.wait_statustext('Using EKF Source Set 1', check_context=True)
        self.set_rc(10, 1500)
        self.wait_statustext('Using EKF Source Set 2', check_context=True)
        self.set_rc(10, 2000)
        self.wait_statustext('Using EKF Source Set 3', check_context=True)

    def CommonOrigin(self):
        """Test common origin between EKF2 and EKF3"""
        self.context_push()

        # start on EKF2
        self.set_parameters({
            'AHRS_EKF_TYPE': 2,
            'EK2_ENABLE': 1,
            'EK3_CHECK_SCALE': 1, # make EK3 slow to get origin
        })
        self.reboot_sitl()

        self.context_collect('STATUSTEXT')

        self.wait_statustext("EKF2 IMU0 origin set", timeout=60, check_context=True)
        self.wait_statustext("EKF2 IMU0 is using GPS", timeout=60, check_context=True)
        self.wait_statustext("EKF2 active", timeout=60, check_context=True)

        self.context_collect('GPS_GLOBAL_ORIGIN')

        # get EKF2 origin
        self.run_cmd(mavutil.mavlink.MAV_CMD_GET_HOME_POSITION)
        ek2_origin = self.assert_receive_message('GPS_GLOBAL_ORIGIN', check_context=True)

        # switch to EKF3
        self.set_parameters({
            'SIM_GPS1_GLTCH_X' : 0.001, # about 100m
            'EK3_CHECK_SCALE' : 100,
            'AHRS_EKF_TYPE'   : 3})

        self.wait_statustext("EKF3 IMU0 is using GPS", timeout=60, check_context=True)
        self.wait_statustext("EKF3 active", timeout=60, check_context=True)

        self.run_cmd(mavutil.mavlink.MAV_CMD_GET_HOME_POSITION)
        ek3_origin = self.assert_receive_message('GPS_GLOBAL_ORIGIN', check_context=True)

        self.progress("Checking origins")
        if ek2_origin.time_usec == ek3_origin.time_usec:
            raise NotAchievedException("Did not get a new GPS_GLOBAL_ORIGIN message")

        print(ek2_origin, ek3_origin)

        if (ek2_origin.latitude != ek3_origin.latitude or
                ek2_origin.longitude != ek3_origin.longitude or
                ek2_origin.altitude != ek3_origin.altitude):
            raise NotAchievedException("Did not get matching EK2 and EK3 origins")

        self.context_pop()

        # restart GPS driver
        self.reboot_sitl()

    def ReadOnlyDefaults(self):
        '''test that defaults marked "readonly" can't be set'''
        defaults_filepath = tempfile.NamedTemporaryFile(mode='w', delete=False)
        defaults_filepath.write("""
DISARM_DELAY 77 @READONLY
RTL_ALT 123
RTL_ALT_FINAL 129
""")
        defaults_filepath.close()
        self.customise_SITL_commandline([
        ], defaults_filepath=defaults_filepath.name)

        self.context_collect('STATUSTEXT')
        self.send_set_parameter_direct("DISARM_DELAY", 88)

        self.wait_statustext("Param write denied (DISARM_DELAY)")
        self.assert_parameter_value("DISARM_DELAY", 77)
        self.assert_parameter_value("RTL_ALT", 123)

        self.start_subtest('Ensure something is writable....')
        self.set_parameter('RTL_ALT_FINAL', 101)

        new_values_filepath = tempfile.NamedTemporaryFile(mode='w', delete=False)
        new_values_filepath.write("""
DISARM_DELAY 99
RTL_ALT 111
""")
        new_values_filepath.close()

        self.start_subtest("Ensure parameters can't be set via FTP either")
        mavproxy = self.start_mavproxy()
        # can't do two FTP things at once, so wait until parameters are received
        mavproxy.expect("Received .* parameters")
        self.mavproxy_load_module(mavproxy, 'ftp')
        mavproxy.send(f"param ftpload {new_values_filepath.name}\n")
        mavproxy.expect("Loaded")
        self.delay_sim_time(1)
        self.stop_mavproxy(mavproxy)

        self.assert_parameter_value("DISARM_DELAY", 77)
        self.assert_parameter_value("RTL_ALT", 111)
        self.assert_parameter_value('RTL_ALT_FINAL', 101)

    def ScriptingFlipMode(self):
        '''test adding custom mode from scripting'''
        # Really it would be nice to check for the AVAILABLE_MODES message, but pymavlink does not understand them yet.

        # enable scripting and install flip script
        self.set_parameters({
            "SCR_ENABLE": 1,
        })
        self.install_example_script_context('Flip_Mode.lua')
        self.reboot_sitl()

        # Takeoff in loiter
        self.takeoff(10, mode="LOITER")

        # Try and switch to flip, should not be possible from loiter
        try:
            self.change_mode(100, timeout=10)
        except AutoTestTimeoutException:
            self.progress("PASS not able to enter from loiter")

        # Should be alowd to enter from alt hold
        self.change_mode("ALT_HOLD")
        self.change_mode(100)

        # Should return to previous mode after flipping
        self.wait_mode("ALT_HOLD")

        # Test done
        self.land_and_disarm()

    def ScriptingFlyVelocity(self):
        '''test flying a velocity vector using scripting'''

        # enable scripting and install set-target-velocity script
        self.set_parameters({
            "SCR_ENABLE": 1,
        })
        self.install_example_script_context('set-target-velocity.lua')
        self.reboot_sitl()

        # Should be allowed to enter from alt hold
        self.change_mode("ALT_HOLD")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(6, 2000)

        # Should return to previous mode after flipping
        self.wait_mode("RTL", timeout=120)

        # Test done
        self.land_and_disarm()

    def RTLYaw(self):
        '''test that vehicle yaws to original heading on RTL'''
        # 0 is WP_YAW_BEHAVIOR_NONE
        # 1 is WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP
        # 2 is WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
        # 3 is WP_YAW_BEHAVIOR_LOOK_AHEAD
        for behaviour in 1, 3:
            self.set_parameters({
                'WP_YAW_BEHAVIOR': behaviour,
            })
            self.change_mode('GUIDED')
            original_heading = self.get_heading()
            target_heading = 100
            if original_heading - target_heading < 90:
                raise NotAchievedException("Bad initial heading")
            self.takeoff(10, mode='GUIDED')
            self.guided_achieve_heading(target_heading)
            self.change_mode('RTL')
            self.wait_heading(original_heading)
            self.wait_disarmed()

    def CompassLearnCopyFromEKF(self):
        '''test compass learning whereby we copy learnt offsets from the EKF'''
        self.reboot_sitl()
        self.context_push()
        self.set_parameters({
            "SIM_MAG1_OFS_X": 1100,
        })
        self.assert_prearm_failure("Check mag field", other_prearm_failures_fatal=False)
        self.context_pop()
        self.wait_ready_to_arm()
        self.takeoff(30, mode='ALT_HOLD')
        # prevent EKF switching to good compass:
        self.set_parameters({
            'COMPASS_USE2': 0,
            'COMPASS_USE3': 0,
        })
        self.assert_parameter_value("COMPASS_OFS_X", 20, epsilon=30)
        # set the parameter so it gets reset at context pop time:
        self.set_parameter("COMPASS_OFS_X", 20)
        new_compass_ofs_x = 200
        self.set_parameters({
            "SIM_MAG1_OFS_X": new_compass_ofs_x,
        })
        self.set_parameter("COMPASS_LEARN", 2)  # 2 is Copy-from-EKF

        # commence silly flying to try to give the EKF as much
        # information as possible for it to converge its estimation;
        # there's a 5e-6 check before we consider the offsets good!
        self.set_rc(4, 1450)
        self.set_rc(1, 1450)
        for i in range(0, 5):  # we descend through all of this:
            self.change_mode('LOITER')
            self.delay_sim_time(10)
            self.change_mode('ALT_HOLD')
            self.change_mode('FLIP')

        self.set_parameter('ANGLE_MAX', 7000)
        self.change_mode('ALT_HOLD')
        for j in 1000, 2000:
            for i in 1, 2, 4:
                self.set_rc(i, j)
                self.delay_sim_time(10)
        self.set_rc(1, 1500)
        self.set_rc(2, 1500)
        self.set_rc(4, 1500)

        self.do_RTL()
        self.assert_parameter_value("COMPASS_OFS_X", new_compass_ofs_x, epsilon=30)
        self.reboot_sitl()
        self.assert_parameter_value("COMPASS_OFS_X", new_compass_ofs_x, epsilon=30)

    def RudderDisarmMidair(self):
        '''check disarm behaviour mid-air'''
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.change_mode('STABILIZE')

        # Set home current location, this gives a large home vs origin difference
        self.set_home(self.mav.location())

        self.set_rc(4, 2000)
        self.wait_armed()
        self.set_rc(4, 1500)
        self.set_rc(3, 2000)
        self.wait_altitude(300, 20000, relative=True, timeout=10000)
        self.hover()
        self.set_rc(4, 1000)
        self.delay_sim_time(11)
        self.progress("Checking we are still armed")
        self.assert_armed()

        self.set_rc(3, 2000)
        self.wait_altitude(300, 20000, relative=True)
        self.set_rc(3, 1000)
        self.set_rc(4, 1000)
        self.delay_sim_time(11)
        self.progress("Checking we disarm")
        self.wait_disarmed(timeout=5)
        self.set_rc(4, 1500)
        self.arm_vehicle()
        self.set_rc(3, 1600)
        self.delay_sim_time(10)

        self.do_RTL(timeout=600)

    def mission_NAV_LOITER_TURNS(self):
        '''test that loiter turns basically works'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=1,
                p3=30,
                z=30,  # circle is 10m higher than takeoff
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.set_parameter('AUTO_OPTIONS', 3)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_disarmed()

    def mission_NAV_LOITER_TURNS_off_center(self):
        '''test that loiter turns basically works - copter on edge of circle'''
        self.start_subtest("Start circle when on edge of circle")
        radius = 30
        self.wait_ready_to_arm()
        here = self.mav.location()
        circle_centre_loc = self.offset_location_ne(here, radius, 0)
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=1,
                p3=radius,
                x=int(circle_centre_loc.lat*1e7),
                y=int(circle_centre_loc.lng*1e7),
                z=30,  # circle is 10m higher than takeoff
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.set_parameter('AUTO_OPTIONS', 3)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_disarmed()

    def AHRSAutoTrim(self):
        '''calibrate AHRS trim using RC input'''
        self.progress("Making earth frame same as body frame")  # because I'm lazy
        self.takeoff(5, mode='GUIDED')
        self.guided_achieve_heading(0)
        self.do_land()

        self.set_parameters({
            'RC9_OPTION': 182,
        })

        param_last_check_time = 0
        for mode in ['STABILIZE', 'ALT_HOLD']:
            self.set_parameters({
                'AHRS_TRIM_X': 0.1,
                'AHRS_TRIM_Y': -0.1,
            })
            self.takeoff(mode=mode)
            self.set_rc(9, 2000)
            tstart = self.get_sim_time()
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > 30:
                    raise ValueError(f"Failed to reduce trims in {mode}!")
                lpn = self.assert_receive_message('LOCAL_POSITION_NED')
                delta = 40
                roll_input = 1500
                if lpn.vx > 0:
                    roll_input -= delta
                elif lpn.vx < 0:
                    roll_input += delta

                pitch_input = 1500
                if lpn.vy > 0:
                    pitch_input += delta
                elif lpn.vy < 0:
                    pitch_input -= delta
                self.set_rc_from_map({
                    1: roll_input,
                    2: pitch_input,
                }, quiet=True)

                # check parameters once/second:
                if now - param_last_check_time > 1:
                    param_last_check_time = now
                    trim_x = self.get_parameter('AHRS_TRIM_X', verbose=False)
                    trim_y = self.get_parameter('AHRS_TRIM_Y', verbose=False)
                    self.progress(f"trim_x={trim_x} trim_y={trim_y}")
                    if abs(trim_x) < 0.01 and abs(trim_y) < 0.01:
                        self.progress("Good AHRS trims")
                        self.progress(f"vx={lpn.vx} vy={lpn.vy}")
                        if abs(lpn.vx) > 1 or abs(lpn.vy) > 1:
                            raise NotAchievedException("Velocity after trimming?!")
                        break
            self.context_collect('STATUSTEXT')
            self.set_rc(9, 1000)
            self.wait_statustext('Trim saved', check_context=True)
            self.context_stop_collecting('STATUSTEXT')
            self.do_land()
            self.set_rc_default()

        self.progress("Landing should cancel the autotrim")
        self.takeoff(10, mode='STABILIZE')
        self.context_collect('STATUSTEXT')
        self.set_rc(9, 2000)
        self.wait_statustext('AutoTrim running', check_context=True)
        self.do_land()
        self.wait_statustext('AutoTrim cancelled', check_context=True)
        self.set_rc(9, 1000)

        self.progress("Changing mode to LOITER")
        self.takeoff(10, mode='STABILIZE')
        self.context_collect('STATUSTEXT')
        self.set_rc(9, 2000)
        self.wait_statustext('AutoTrim running', check_context=True)
        self.change_mode('LOITER')
        self.wait_statustext('AutoTrim cancelled', check_context=True)
        self.do_land()
        self.set_rc(9, 1000)

    def RTLStoppingDistanceSpeed(self):
        '''test stopping distance unaffected by RTL speed'''
        self.upload_simple_relhome_mission([
            #                                              N  E   U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,          0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,       200, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0,  0),
        ])
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })
        self.context_push()
        self.set_parameters({
            'RTL_SPEED': 100,  # cm/s
        })

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(2)

        self.progress("Waiting for vehicle to get up to speed")
        wpnav_speed = self.get_parameter('WPNAV_SPEED')
        self.wait_groundspeed(wpnav_speed/100-0.1, wpnav_speed/100+0.1)

        rtl_start_pos = self.get_local_position_NED()

        # we accelerate so hard we can miss the zero value!
        self.context_set_message_rate_hz('VFR_HUD', 50)

        # note that there's going to be a significant latency involved
        # between taking rtl_start_pos and entering RTL, but we're
        # really interested in deviation to the east/west, and we are
        # travelling due North, ...
        self.change_mode('RTL')

        self.progress("Waiting for vehicle to stop")
        self.wait_groundspeed(-0.3, 0.3)
        rtl_stopping_point_pos = self.get_local_position_NED()

        self.progress("Checking vehicle deviation from track")
        y_delta = abs(rtl_start_pos.y - rtl_stopping_point_pos.y)
        self.progress(f"deviated {y_delta}m from track")
        if y_delta > 0.2:
            raise NotAchievedException(f"RTL deviated from track {y_delta}m")
        self.context_pop()
        self.change_mode('LOITER')
        self.do_RTL()

    def ModeAllowsEntryWhenNoPilotInput(self):
        '''test we can't enter modes when no RC input'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameters({
            "AUTO_OPTIONS": 3,
            "FS_THR_ENABLE": 4,  # continue in auto
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(2)
        self.context_collect('STATUSTEXT')
        self.set_rc(3, 200)
        self.wait_statustext('Radio Failsafe', check_context=True)
        self.send_cmd_do_set_mode('STABILIZE')
        self.wait_statustext('in RC failsafe', check_context=True)
        self.hover()
        self.wait_statustext('Radio Failsafe Cleared', check_context=True)
        self.do_RTL()

    def RCOverridesNoRCReceiver(self):
        '''test RC override timeout with no RC receiver present'''
        self.set_parameters({
            "MAV_GCS_SYSID": 250,
            "SIM_RC_FAIL": 1,  # no-pulses
        })
        self.reboot_sitl()

        rc_send_enabled = True
        rc1_value = 1500
        rc2_value = 1500
        rc3_value = 1000
        rc4_value = 1500

        # register a hook which sends rcn_values as overrides
        def rc_override_sender(mav, message):
            if message.get_type() == 'ATTITUDE':
                if not rc_send_enabled:
                    return
                self.mav.mav.rc_channels_override_send(
                    mav.target_system, # target system
                    1, # targe component
                    rc1_value, # chan1_raw
                    rc2_value, # chan2_raw
                    rc3_value, # chan3_raw
                    rc4_value, # chan4_raw
                    65535, # chan5_raw
                    65535, # chan6_raw
                    65535, # chan7_raw
                    65535  # chan8_raw
                )
        self.install_message_hook_context(rc_override_sender)
        self.do_set_mode_via_command_int('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Takeoff throttle")
        rc3_value = 1800
        self.delay_sim_time(1)
        self.wait_altitude(20, 30, relative=True)
        self.progress("Neutral throttle")
        rc3_value = 1600
        self.progress("yaw ccw")
        rc4_value = 1450
        self.wait_yaw_speed(math.radians(-30), accuracy=5, minimum_duration=10)
        self.progress("Kill overrides and wait for speed to get neutralised")
        rc_send_enabled = False
        self.wait_yaw_speed(0, 1, minimum_duration=10)
        self.do_set_mode_via_command_int('RTL')
        self.wait_disarmed()

    def MISSION_OPTION_CLEAR_MISSION_AT_BOOT(self):
        '''check functionality of mission-clear-at-boot option'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        if len(self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)) != 4:
            raise NotAchievedException("No mission after upload?!")

        self.reboot_sitl()
        if len(self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)) != 4:
            raise NotAchievedException("No mission after reboot?!")

        self.set_parameters({
            "MIS_OPTIONS": 1,
        })
        self.reboot_sitl()

        if len(self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)) != 0:
            raise NotAchievedException("Mission did not clear after reboot with option set")

    def IgnorePilotYaw(self):
        '''test RTL_OPTIONS bit 2, "IgnorePilotYaw"'''
        self.takeoff(10, mode='GUIDED')
        self.fly_guided_move_local(10, 10, 10, timeout=100)

        orig_yaw = self.assert_receive_message('VFR_HUD').heading

        def ensure_heading_stays_constant(mav, m):
            print(m.get_type())
            if m.get_type() != "VFR_HUD":
                return
            new_yaw = m.heading
            if abs(orig_yaw - new_yaw) > 15:
                raise NotAchievedException(f"Vehicle yawed! ({orig_yaw=} {new_yaw=}")

        self.context_push()
        self.install_message_hook_context(ensure_heading_stays_constant)

        self.change_mode('RTL')

        self.set_parameter('RTL_OPTIONS', 1 << 2)
        self.set_rc(4, 2000)
        self.delay_sim_time(5)
        self.context_pop()

        self.wait_disarmed()

    def PLDNoParameters(self):
        ''''try enabling PLD with lat/lng/alt all zero'''
        self.set_parameter('SIM_PLD_LAT', 0)
        self.set_parameter('SIM_PLD_LON', 0)
        self.set_parameter('SIM_PLD_HEIGHT', 0)
        self.set_parameter('SIM_PLD_ENABLE', 1)
        self.wait_statustext('Set SIM_PLD_LAT, SIM_PLD_LAT and SIM_PLD_ALT')

    def NoRC(self):
        '''check what happens if we fly with no RC'''
        self.set_parameters({
            "SIM_RC_FAIL": 1,
            "FS_THR_ENABLE": 0,
            "AUTO_OPTIONS": 3,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.reboot_sitl()

        # add a listener that verifies yaw rate remains zero:
        def verify_yaw(mav, m):
            if m.get_type() != 'ATTITUDE':
                return
            thresh_degs = 10
            current_degs = math.degrees(abs(m.yawspeed))
            if current_degs > thresh_degs:
                raise NotAchievedException(
                    f"Excessive yaw: {current_degs} deg/s > {thresh_degs} deg/s"
                )
        self.install_message_hook_context(verify_yaw)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_disarmed()

    def LuaParamSet(self):
        '''test param-set.lua applet'''
        self.set_parameters({
            'SCR_ENABLE': 1,
        })

        self.context_push()

        self.install_applet_script_context("param-set.lua")
        self.reboot_sitl()

        # collect original parameter values for everything other than
        # SCR_ENABLE and PARAM_SET_ENABLE.  This resolves an issue at
        # the end of the test where the popped context wants to set
        # PARAM_SET_ENABLE back to its original value (enabled!),
        # which can cause problems resetting
        orig_parameter_values = self.get_parameters([
            'RTL_ALT',
            'DISARM_DELAY',
        ])

        self.wait_ready_to_arm()  # scripts will be ready by now!
        self.start_subtest("set RTL_ALT freely")
        self.context_collect("STATUSTEXT")
        self.set_parameter("RTL_ALT", 23)
        self.set_parameter("RTL_ALT", 28)
        # Ensure there are no scripting errors.
        error = self.statustext_in_collections("Internal Error")
        if error is not None:
            raise NotAchievedException("Script errored out in positive test.")
        self.context_stop_collecting("STATUSTEXT")

        self.start_subtest("Unable to set DISARM_DELAY freely")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.context_collect('PARAM_ERROR')
        old_disarm_delay_value = self.get_parameter('DISARM_DELAY')
        self.send_set_parameter_direct('DISARM_DELAY', 78)
        self.wait_statustext('param-set: param set denied (DISARM_DELAY)', check_context=True)
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": 'DISARM_DELAY',
            "param_index": -1,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_PERMISSION_DENIED,
        }, check_context=True, very_verbose=True)
        self.assert_parameter_value('DISARM_DELAY', old_disarm_delay_value)
        self.context_pop()

        self.start_subtest("Disabling applet via parameter should allow freely setting DISARM_DELAY")
        self.set_parameter("PARAM_SET_ENABLE", 0)
        self.set_parameter("DISARM_DELAY", 56)

        self.start_subtest("Re-enabling applet via parameter should stop freely setting DISARM_DELAY")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.context_collect('PARAM_ERROR')
        self.set_parameter("PARAM_SET_ENABLE", 1)
        old_disarm_delay_value = self.get_parameter('DISARM_DELAY')
        self.send_set_parameter_direct('DISARM_DELAY', 78)
        self.wait_statustext('param-set: param set denied (DISARM_DELAY)', check_context=True)
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": 'DISARM_DELAY',
            "param_index": -1,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_PERMISSION_DENIED,
        }, check_context=True, very_verbose=True)
        self.assert_parameter_value('DISARM_DELAY', old_disarm_delay_value)
        self.context_pop()

        self.start_subtest("Ensure that parameter values are persistent")
        self.set_parameter('DISARM_DELAY', 111)
        self.reboot_sitl()
        self.assert_parameter_value('DISARM_DELAY', 111)

        # set parameters to their original value so the following
        # context pop should only need to set PARAM_SET_ENABLE back to
        # its default value of 1.  That prevents permission errors
        # created by param-set.lua!
        self.set_parameters(orig_parameter_values)

        self.context_pop()

    def do_land(self):
        self.change_mode('LAND')
        self.wait_disarmed()

    def PeriphMultiUARTTunnel(self):
        '''test peripheral multi-uart tunneling'''

        speedup = 1

        self.progress("Building Periph")
        periph_board = 'sitl_periph_can_to_serial'
        periph_builddir = util.reltopdir('build-periph')
        util.build_SITL(
            'bin/AP_Periph',
            board=periph_board,
            clean=False,
            configure=True,
            debug=True,
            extra_configure_args=[
                '--out', periph_builddir,
            ],
            # extra_defines={
            # },
        )

        binary_path = ""

        self.progress("Starting Periph simulation")
        binary_path = pathlib.Path(periph_builddir, periph_board, 'bin', 'AP_Periph')
        periph_rundir = util.reltopdir('run-periph')
        if not os.path.exists(periph_rundir):
            os.mkdir(periph_rundir)
        periph_exp = util.start_SITL(
            binary_path,
            cwd=periph_rundir,
            stdout_prefix="periph",
            gdb=self.gdb,
            valgrind=self.valgrind,
            customisations=[
                '-I', str(1),
                '--serial0', 'mcast:',
                '--serial1', 'tcp:2',
                '--serial2', 'tcp:3',
            ],
            param_defaults={
                "SIM_SPEEDUP": speedup,
            },
        )
        self.expect_list_add(periph_exp)

        self.progress("Reconfiguring for multicast")
        self.customise_SITL_commandline([
            "--serial5=mcast:",
        ],
            model="octa-quad:@ROMFS/models/Callisto.json",
            defaults_filepath=self.model_defaults_filepath('Callisto'),
            wipe=True,
        )

        self.set_parameters({
            'SERIAL5_PROTOCOL': 2,
            "CAN_P1_DRIVER": 1,  # needed for multicast state!
        })
        self.reboot_sitl()
        self.set_parameters({
            "CAN_D1_UC_SER_EN": 1, # enable serial
            "CAN_D1_UC_S1_IDX": 1,  # serial port number on CAN device
            "CAN_D1_UC_S1_NOD": 125,  # FIXME: set this explicitly
            "CAN_D1_UC_S1_PRO": 2,  # protocol to set on remote node

            "CAN_D1_UC_S2_IDX": 2,  # serial port number on CAN device
            "CAN_D1_UC_S2_NOD": 125,  # FIXME: set this explicitly
            "CAN_D1_UC_S2_PRO": 2,  # protocol to set on remote node
        })

        self.reboot_sitl()

        # must be done after the reboot:
        self.set_parameters({
            'SIM_SPEEDUP': speedup,
        })

        # uncomment this if you just want the test scenario set up for you:
        # self.delay_sim_time(100000)

        self.progress("Connect to the serial port on the peripheral, which should be talking mavlink")
        self.drain_mav()
        mav2 = mavutil.mavlink_connection(
            "tcp:localhost:5772",
            robust_parsing=True,
            source_system=9,
            source_component=9,
        )
        self.assert_receive_message("HEARTBEAT", mav=mav2, very_verbose=True, timeout=5)
        self.drain_mav()

        self.progress("Connect to the other serial port on the peripheral, which should also be talking mavlink")
        self.drain_mav()
        mav3 = mavutil.mavlink_connection(
            "tcp:localhost:5773",
            robust_parsing=True,
            source_system=10,
            source_component=10,
        )
        self.assert_receive_message("HEARTBEAT", mav=mav3, very_verbose=True, timeout=5)
        self.drain_mav()

        # make sure we continue to get heartbeats:
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time()
            if now - tstart > 30:
                break
            self.assert_receive_message('HEARTBEAT', mav=mav2, verbose=2)
            self.drain_mav()
            self.assert_receive_message('HEARTBEAT', mav=mav3, verbose=2)
            self.drain_mav()

    def RCProtocolFailsafe(self):
        '''ensure we failsafe when the RC protocol failsafe is set'''
        self.takeoff(10, mode='LOITER')
        self.set_parameter('SIM_RC_FAIL', 3)  # protocol failsafe
        self.wait_mode('RTL')
        self.wait_disarmed()

    def LUAConfigProfile(self):
        '''test the config_profiles.lua example script'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('Callisto'),
            model="octa-quad:@ROMFS/models/Callisto.json",
            wipe=True,
        )
        self.install_example_script_context("config_profiles.lua")
        self.set_parameters({
            'SCR_ENABLE': 1,
        })
        self.context_push()

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.assert_prearm_failure('CFG: Reboot needed for config change', other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameter("CFG_PAY_SEL", 1)
        self.wait_parameter_values({
            "GRIP_ENABLE": 1,
            "RC8_OPTION": 19,
        })
        self.set_parameter("CFG_PAY_SEL", 3)  # avt gimbal
        self.wait_parameter_values({
            "GRIP_ENABLE": 0,
            "RC8_OPTION": 0,
        })
        self.set_parameter("CFG_PAY_SEL", 2)  # hook slung
        self.wait_parameter_values({
            "GRIP_ENABLE": 1,
            "RC8_OPTION": 19,
        })
        self.progress("Apply default parameters")
        self.set_parameter("CFG_PAY_SEL", 0)
        self.wait_parameter_values({
            "GRIP_ENABLE": 0,
            "RC8_OPTION": 0,
        })

        self.set_parameter("CFG_PAY_SEL", 22)
        self.wait_statustext("Invalid profile selected for PAY")

        self.start_subtest("Test the different modes (nothing / apply / defaults)")

        self.context_pop()

        self.start_subtest("Testing of different validations")

        def set_config_profiles_config_domains(new_domains):
            path = pathlib.Path(self.installed_script_path("config_profiles.lua"))
            content = path.read_text()

            start_marker = '-- This is a marker for the start of the config_domains; it is used to swap these out for CI testing'  # noqa: E501
            end_marker = '-- This is a marker for the end of the config_domains; it is used to swap these out for CI testing'  # noqa: E501
            pattern = re.compile(
                rf"({re.escape(start_marker)})(.*?){re.escape(end_marker)}",
                re.DOTALL
            )

            if not pattern.search(content):
                raise ValueError("Start or end marker not found in file")

            replaced = pattern.sub(rf"\1\n{new_domains}\n{end_marker}", content)
            path.write_text(replaced)

        def assert_prearm_failure_for_domains_string(domains_string, prearm_failure_reason):
            self.start_subtest(f"Testing domains for ({prearm_failure_reason})")
            set_config_profiles_config_domains(domains_string)
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.reboot_sitl()  # can't just restart scripting as we reuse indexes for different parameters
            # self.wait_statustext('config_profiles .* starting', regex=True, check_context=True)
            self.assert_prearm_failure(prearm_failure_reason, other_prearm_failures_fatal=False, timeout=120)
            self.context_pop()

        domain_param_not_in_defaults = """
-- a table of param indexes to help avoid annoying conflicts:
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 3,
      all_param_defaults = {  -- all parameters present in the params for each option
      },
      profiles = {
         [1] = {
            name = "Param Not in defaults",
            params = {
               ["ACRO_BAL_PITCH"] = 1,
            },
         },
      },
   }
}
"""
        assert_prearm_failure_for_domains_string(domain_param_not_in_defaults, "Not in defaults")

        domain_param_not_in_defaults = """
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
   ["DMN_B"]       = 11,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 5,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["ACRO_BAL_PITCH"] = 1,
      },
      profiles = {
         [1] = {
            name = "exists in two domains",
            params = {
               ["ACRO_BAL_PITCH"] = 1,
            },
         },
      },
   },
   DOMAIN_B = {
      param_name = "DMN_B",
      param_sel_index = 3,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["ACRO_BAL_PITCH"] = 1,
      },
      profiles = {
         [1] = {
            name = "param in two domains",
            params = {
               ["ACRO_BAL_PITCH"] = 1,
            },
         },
      },
   },
}

"""
        assert_prearm_failure_for_domains_string(domain_param_not_in_defaults, "CFG: ACRO_BAL_PITCH exists in two")

        domain_profile_param_same_as_domain_default_param_value = """
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 5,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["BATT_CAPACITY"] = 6,
      },
      profiles = {
         [1] = {
            name = "param value same as default",
            params = {
               ["BATT_CAPACITY"] = 6,
            },
         },
      },
   },
}

"""
        assert_prearm_failure_for_domains_string(
            domain_profile_param_same_as_domain_default_param_value,
            "has the same value as"
        )

        domain_profile_param_must_be_set_when_mode_is_apply_defaults = """
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 0,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["BATT_CAPACITY"] = must_be_set,
      },
      profiles = {
         [1] = {
            name = "param must be set",
            params = {
               ["BATT_CAPACITY"] = 6,
            },
         },
      },
   },
}

"""
        assert_prearm_failure_for_domains_string(
            domain_profile_param_must_be_set_when_mode_is_apply_defaults,
            "BATT_CAPACITY in DMN_A is must-be-set but al"
        )

    def ScriptParamRegistration(self):
        '''test parameter script registration'''
        self.set_parameters({
            'SCR_ENABLE': 1,
        })
        script_content = """
local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "TST_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local PARAM_A = bind_add_param("A", 5, 22)
local PARAM_B = bind_add_param("B", 1, 33)

function update()
  gcs:send_text(3, string.format("test script running"))
  return update, 1000
end

return update, 1000
"""  # noqa: E501
        self.install_script_content_context("test.lua", script_content)
        self.reboot_sitl()
        self.wait_statustext('test script running')
        self.assert_parameter_value('TST_A', 22)
        self.assert_parameter_value('TST_B', 33)

        all_params = self.fetch_all_parameters()
        for pname in "TST_A", "TST_B":
            if pname not in all_params:
                raise ValueError(f"{pname} not in fetched-all-parameters")

        self.start_subtest("Remove parameter at runtime")
        script_content = """
local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "TST_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local PARAM_B = bind_add_param("B", 1, 33)

function update()
  gcs:send_text(3, string.format("test script running"))
  return update, 1000
end

return update, 1000
"""  # noqa: E501
        self.install_script_content_context("test.lua", script_content)
        self.scripting_restart()
        self.wait_statustext('restart')
        self.wait_statustext('test script running')
        self.assert_parameter_value('TST_B', 33)

        all_params = self.fetch_all_parameters()
        for pname in ["TST_B"]:
            if pname not in all_params:
                raise ValueError(f"{pname} not in fetched-all-parameters")
        for pname in ["TST_A"]:
            if pname in all_params:
                raise ValueError(f"{pname} in fetched-all-parameters when it should have gone away")

    def tests2b(self):  # this block currently around 9.5mins here
        '''return list of all tests'''
        ret = ([
            self.MotorVibration,
            Test(self.DynamicNotches, attempts=4),
            self.PositionWhenGPSIsZero,
            self.DynamicRpmNotches, # Do not add attempts to this - failure is sign of a bug
            self.DynamicRpmNotchesRateThread,
            self.PIDNotches,
            self.mission_NAV_LOITER_TURNS,
            self.mission_NAV_LOITER_TURNS_off_center,
            self.StaticNotches,
            self.LuaParamSet,
            self.RefindGPS,
            Test(self.GyroFFT, attempts=1, speedup=8),
            Test(self.GyroFFTHarmonic, attempts=4, speedup=8),
            Test(self.GyroFFTAverage, attempts=1, speedup=8),
            Test(self.GyroFFTContinuousAveraging, attempts=4, speedup=8),
            self.WPYawBehaviour1RTL,
            self.GyroFFTPostFilter,
            self.GyroFFTMotorNoiseCheck,
            self.CompassReordering,
            self.SixCompassCalibrationAndReordering,
            self.CRSF,
            self.MotorTest,
            self.AltEstimation,
            self.EKFSource,
            self.GSF,
            self.GSF_reset,
            self.AP_Avoidance,
            self.RTL_ALT_FINAL,
            self.SMART_RTL,
            self.SMART_RTL_EnterLeave,
            self.SMART_RTL_Repeat,
            self.RTL_TO_RALLY,
            self.RTLYaw,
            self.FlyEachFrame,
            self.ScriptParamRegistration,
            self.GPSBlending,
            self.GPSWeightedBlending,
            self.GPSBlendingLog,
            self.GPSBlendingAffinity,
            self.DataFlash,
            Test(self.DataFlashErase, attempts=8),
            self.Callisto,
            self.PerfInfo,
            self.ModeAllowsEntryWhenNoPilotInput,
            self.Replay,
            self.FETtecESC,
            self.ProximitySensors,
            self.GroundEffectCompensation_touchDownExpected,
            self.GroundEffectCompensation_takeOffExpected,
            self.DO_CHANGE_SPEED,
            self.MISSION_START,
            self.AUTO_LAND_TO_BRAKE,
            self.WPNAV_SPEED,
            self.RTLStoppingDistanceSpeed,
            self.WPNAV_SPEED_UP,
            self.WPNAV_SPEED_DN,
            self.DO_WINCH,
            self.SensorErrorFlags,
            self.GPSForYaw,
            self.DefaultIntervalsFromFiles,
            self.GPSTypes,
            self.MultipleGPS,
            self.WatchAlts,
            self.GuidedEKFLaneChange,
            self.Sprayer,
            self.AutoContinueOnRCFailsafe,
            self.EK3_RNG_USE_HGT,
            self.NoRC,
            self.RCOverridesNoRCReceiver,
            self.TerrainDBPreArm,
            self.ThrottleGainBoost,
            self.ScriptMountPOI,
            self.ScriptMountAllModes,
            self.ScriptCopterPosOffsets,
            self.MountSolo,
            self.FlyMissionTwice,
            self.FlyMissionTwiceWithReset,
            self.MissionIndexValidity,
            self.InvalidJumpTags,
            self.IMUConsistency,
            self.AHRSTrimLand,
            self.IBus,
            self.GuidedYawRate,
            self.RudderDisarmMidair,
            self.NoArmWithoutMissionItems,
            self.DO_CHANGE_SPEED_in_guided,
            self.ArmSwitchAfterReboot,
            self.RPLidarA1,
            self.RPLidarA2,
            self.MISSION_OPTION_CLEAR_MISSION_AT_BOOT,
            self.SafetySwitch,
            self.RCProtocolFailsafe,
            self.BrakeZ,
            self.MAV_CMD_DO_FLIGHTTERMINATION,
            self.MAV_CMD_DO_LAND_START,
            self.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            self.MAV_CMD_SET_EKF_SOURCE_SET,
            self.MAV_CMD_NAV_TAKEOFF,
            self.MAV_CMD_NAV_TAKEOFF_command_int,
            self.Ch6TuningWPSpeed,
            self.DualTuningChannels,
            self.PILOT_THR_BHV,
            self.GPSForYawCompassLearn,
            self.CameraLogMessages,
            self.LoiterToGuidedHomeVSOrigin,
            self.GuidedModeThrust,
            self.CompassMot,
            self.AutoRTL,
            self.EK3_OGN_HGT_MASK_climbing,
            self.EK3_OGN_HGT_MASK,
            self.FarOrigin,
            self.GuidedForceArm,
            self.GuidedWeatherVane,
            self.LUAConfigProfile,
            self.Clamp,
            self.GripperReleaseOnThrustLoss,
            self.GripperInitialPosition,
            self.REQUIRE_LOCATION_FOR_ARMING,
            self.LoggingFormat,
            self.MissionRTLYawBehaviour,
            self.BatteryInternalUseOnly,
            self.MAV_CMD_MISSION_START_p1_p2,
            self.ScriptingAHRSSource,
            self.CommonOrigin,
            self.TestTetherStuck,
            self.ScriptingFlipMode,
            self.ScriptingFlyVelocity,
            self.EK3_EXT_NAV_vel_without_vert,
            self.CompassLearnCopyFromEKF,
            self.AHRSAutoTrim,
            self.Ch6TuningLoitMaxXYSpeed,
            self.IgnorePilotYaw,
            self.TestEKF3CompassFailover,
            self.test_EKF3_option_disable_lane_switch,
            self.PLDNoParameters,
            self.PeriphMultiUARTTunnel,
            self.EKF3SRCPerCore,
        ])
        return ret

    def testcan(self):
        ret = ([
            self.CANGPSCopterMission,
            self.TestLogDownloadMAVProxyCAN,
        ])
        return ret

    def BattCANSplitAuxInfo(self):
        '''test CAN battery periphs'''
        self.start_subtest("Swap UAVCAN backend at runtime")
        self.set_parameters({
            "CAN_P1_DRIVER": 1,
            "BATT_MONITOR": 4,  # 4 is analog volt+curr
            "BATT2_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
            "BATT_SERIAL_NUM": 0,
            "BATT2_SERIAL_NUM": 0,
            "BATT_OPTIONS": 128,  # allow split auxinfo
            "BATT2_OPTIONS": 128,  # allow split auxinfo
        })
        self.reboot_sitl()
        self.delay_sim_time(2)
        self.set_parameters({
            "BATT_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
            "BATT2_MONITOR": 4,  # 8 is UAVCAN_BatteryInfo
        })
        self.delay_sim_time(2)
        self.set_parameters({
            "BATT_MONITOR": 4,  # 8 is UAVCAN_BatteryInfo
            "BATT2_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
        })
        self.delay_sim_time(2)
        self.set_parameters({
            "BATT_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
            "BATT2_MONITOR": 4,  # 8 is UAVCAN_BatteryInfo
        })
        self.delay_sim_time(2)

    def BattCANReplaceRuntime(self):
        '''test CAN battery periphs'''
        self.start_subtest("Replace UAVCAN backend at runtime")
        self.set_parameters({
            "CAN_P1_DRIVER": 1,
            "BATT_MONITOR": 11,  # 4 is analog volt+curr
        })
        self.reboot_sitl()
        self.delay_sim_time(2)
        self.set_parameters({
            "BATT_MONITOR": 8,  # 4 is UAVCAN batterinfo
        })
        self.delay_sim_time(2)

    def testcanbatt(self):
        ret = ([
            self.BattCANReplaceRuntime,
            self.BattCANSplitAuxInfo,
        ])
        return ret

    def tests(self):
        ret = []
        ret.extend(self.tests1a())
        ret.extend(self.tests1b())
        ret.extend(self.tests1c())
        ret.extend(self.tests1d())
        ret.extend(self.tests1e())
        ret.extend(self.tests2a())
        ret.extend(self.tests2b())
        return ret

    def disabled_tests(self):
        return {
            "Parachute": "See https://github.com/ArduPilot/ardupilot/issues/4702",
            "GroundEffectCompensation_takeOffExpected": "Flapping",
            "GroundEffectCompensation_touchDownExpected": "Flapping",
            "FlyMissionTwice": "See https://github.com/ArduPilot/ardupilot/pull/18561",
            "CompassMot": "Causes an arithmetic exception in the EKF",
            "SMART_RTL_EnterLeave": "Causes a panic",
            "SMART_RTL_Repeat": "Currently fails due to issue with loop detection",
            "RTLStoppingDistanceSpeed": "Currently fails due to vehicle going off-course",
        }


class AutoTestCopterTests1a(AutoTestCopter):
    def tests(self):
        return self.tests1a()


class AutoTestCopterTests1b(AutoTestCopter):
    def tests(self):
        return self.tests1b()


class AutoTestCopterTests1c(AutoTestCopter):
    def tests(self):
        return self.tests1c()


class AutoTestCopterTests1d(AutoTestCopter):
    def tests(self):
        return self.tests1d()


class AutoTestCopterTests1e(AutoTestCopter):
    def tests(self):
        return self.tests1e()


class AutoTestCopterTests2a(AutoTestCopter):
    def tests(self):
        return self.tests2a()


class AutoTestCopterTests2b(AutoTestCopter):
    def tests(self):
        return self.tests2b()


class AutoTestCAN(AutoTestCopter):

    def tests(self):
        return self.testcan()


class AutoTestBattCAN(AutoTestCopter):

    def tests(self):
        return self.testcanbatt()
