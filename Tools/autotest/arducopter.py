#!/usr/bin/env python

'''
Fly Copter in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
import copy
import math
import os
import shutil
import time
import numpy

from pymavlink import mavutil
from pymavlink import mavextra

from pysim import util, rotmat

from common import AutoTest
from common import NotAchievedException, AutoTestTimeoutException, PreconditionFailedException
from common import Test

from pymavlink.rotmat import Vector3

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 584, 270)
SITL_START_LOCATION_AVC = mavutil.location(40.072842, -105.230575, 1586, 0)


# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize


class AutoTestCopter(AutoTest):
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
        return ["ACRO", "ALT_HOLD", "SPORT", "STABILIZE", "GUIDED_NOGPS"]

    def log_name(self):
        return "ArduCopter"

    def test_filepath(self):
        return os.path.realpath(__file__)

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

    def user_takeoff(self, alt_min=30):
        '''takeoff using mavlink takeoff command'''
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, # param1
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     alt_min # param7
                     )
        self.progress("Ran command")
        self.wait_for_alt(alt_min)

    def takeoff(self,
                alt_min=30,
                takeoff_throttle=1700,
                require_absolute=True,
                mode="STABILIZE",
                timeout=120):
        """Takeoff get to 30m altitude."""
        self.progress("TAKEOFF")
        self.change_mode(mode)
        if not self.armed():
            self.wait_ready_to_arm(require_absolute=require_absolute, timeout=timeout)
            self.zero_throttle()
            self.arm_vehicle()
        self.set_rc(3, takeoff_throttle)
        self.wait_for_alt(alt_min=alt_min, timeout=timeout)
        self.hover()
        self.progress("TAKEOFF COMPLETE")

    def wait_for_alt(self, alt_min=30, timeout=30, max_err=5):
        """Wait for minimum altitude to be reached."""
        self.wait_altitude(alt_min - 1,
                           (alt_min + max_err),
                           relative=True,
                           timeout=timeout)

    def land_and_disarm(self, timeout=60):
        """Land the quad."""
        self.progress("STARTING LANDING")
        self.change_mode("LAND")
        self.wait_landed_and_disarmed(timeout=timeout)

    def wait_landed_and_disarmed(self, min_alt=6, timeout=60):
        """Wait to be landed and disarmed"""
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = m.relative_alt / 1000.0 # mm -> m
        if alt > min_alt:
            self.wait_for_alt(min_alt, timeout=timeout)
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
        self.takeoff(alt_min=dAlt)
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
    def loiter(self, holdtime=10, maxaltchange=5, maxdistchange=5):
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

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_altitude = m.alt
        start = self.mav.location()
        tstart = self.get_sim_time()
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))
        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
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

    def watch_altitude_maintained(self, min_alt, max_alt, timeout=10):
        '''watch alt, relative alt must remain between min_alt and max_alt'''
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                return
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if m.alt <= min_alt:
                raise NotAchievedException("Altitude not maintained: want >%f got=%f" % (min_alt, m.alt))

    def test_mode_ALT_HOLD(self):
        self.takeoff(10, mode="ALT_HOLD")
        self.watch_altitude_maintained(9, 11, timeout=5)
        # feed in full elevator and aileron input and make sure we
        # retain altitude:
        self.set_rc_from_map({
            1: 1000,
            2: 1000,
        })
        self.watch_altitude_maintained(9, 11, timeout=5)
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

    def setGCSfailsafe(self, paramValue=0):
        # Slow down the sim rate if GCS Failsafe is in use
        if paramValue == 0:
            self.set_parameter("FS_GCS_ENABLE", paramValue)
            self.set_parameter("SIM_SPEEDUP", 10)
        else:
            self.set_parameter("SIM_SPEEDUP", 4)
            self.set_parameter("FS_GCS_ENABLE", paramValue)

    # fly a square in alt_hold mode
    def fly_square(self, side=50, timeout=300):

        self.takeoff(10, mode="ALT_HOLD")

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
        self.set_rc(3, 1300)
        time_left = timeout - (self.get_sim_time() - tstart)
        self.progress("timeleft = %u" % time_left)
        if time_left < 20:
            time_left = 20
        self.wait_altitude(-10, 10, timeout=time_left, relative=True)
        self.set_rc(3, 1500)
        self.save_wp()

        # save the stored mission to file
        num_wp = self.save_mission_to_file(os.path.join(testdir,
                                                        "ch7_mission.txt"))
        if not num_wp:
            self.fail_list.append("save_mission_to_file")
            self.progress("save_mission_to_file failed")

        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_waypoint(0, num_wp-1, timeout=500)
        self.progress("test: MISSION COMPLETE: passed!")
        self.land_and_disarm()

    # enter RTL mode and wait for the vehicle to disarm
    def do_RTL(self, distance_min=None, check_alt=True, distance_max=10, timeout=250):
        """Enter RTL mode and wait for the vehicle to disarm at Home."""
        self.change_mode("RTL")
        self.hover()
        self.wait_rtl_complete(check_alt=check_alt, distance_max=distance_max, timeout=timeout)

    def wait_rtl_complete(self, check_alt=True, distance_max=10, timeout=250):
        """Wait for RTL to reach home and disarm"""
        self.progress("Waiting RTL to reach Home and disarm")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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
            self.progress("Alt: %.02f  HomeDist: %.02f %s" %
                          (alt, home_distance, home))

            # our post-condition is that we are disarmed:
            if not self.armed():
                if home == "":
                    raise NotAchievedException("Did not get home")
                # success!
                return

        raise AutoTestTimeoutException("Did not get home and disarm")

    def fly_loiter_to_alt(self):
        """loiter to alt"""

        self.context_push()

        ex = None
        try:

            self.set_parameter("PLND_ENABLED", 1)
            self.set_parameter("PLND_TYPE", 4)

            self.set_analog_rangefinder_parameters()

            self.reboot_sitl()

            num_wp = self.load_mission("copter_loiter_to_alt.txt")

            self.change_mode('LOITER')

            self.wait_ready_to_arm()

            self.arm_vehicle()

            self.change_mode('AUTO')

            self.set_rc(3, 1550)

            self.wait_current_waypoint(2)

            self.set_rc(3, 1500)

            self.wait_waypoint(0, num_wp-1, timeout=500)

            self.wait_disarmed()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()
        self.reboot_sitl()

        if ex is not None:
            raise ex

    # Tests all actions and logic behind the radio failsafe
    def fly_throttle_failsafe(self, side=60, timeout=360):
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
        # restore radio, verify RC function by changing modes to cicle
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
        self.set_parameter('SIM_GPS_DISABLE', 1)
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS_DISABLE', 0)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe RTL fails into land mode due to bad position.")

        # Trigger a GPS failure and RC failure, verify SmartRTL fails
        # into land mode and completes
        self.start_subtest("Radio failsafe SmartRTL->RTL fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS_DISABLE', 1)
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS_DISABLE', 0)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe SmartRTL->RTL fails into land mode due to bad position.")

        # Trigger a GPS failure and RC failure, verify SmartRTL fails
        # into land mode and completes
        self.start_subtest("Radio failsafe SmartRTL->LAND fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 5)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS_DISABLE', 1)
        self.delay_sim_time(5)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS_DISABLE', 0)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe SmartRTL->LAND fails into land mode due to bad position.")

        # Trigger a GPS failure, then restore the GPS. Trigger an RC
        # failure, verify SmartRTL fails into RTL and completes
        self.start_subtest("Radio failsafe SmartRTL->RTL fails into RTL mode due to no path.")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS_DISABLE', 1)
        self.wait_statustext("SmartRTL deactivated: bad position", timeout=60)
        self.set_parameter('SIM_GPS_DISABLE', 0)
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
        self.set_parameter('SIM_GPS_DISABLE', 1)
        self.wait_statustext("SmartRTL deactivated: bad position", timeout=60)
        self.set_parameter('SIM_GPS_DISABLE', 0)
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

    # Tests all actions and logic behind the GCS failsafe
    def fly_gcs_failsafe(self, side=60, timeout=360):
        try:
            self.test_gcs_failsafe(side=side, timeout=timeout)
        except Exception as ex:
            self.setGCSfailsafe(0)
            self.set_parameter('FS_OPTIONS', 0)
            self.disarm_vehicle(force=True)
            self.reboot_sitl()
            raise ex

    def test_gcs_failsafe(self, side=60, timeout=360):
        # Test double-SmartRTL; ensure we do SmarRTL twice rather than
        # landing (tests fix for actual bug)
        self.set_parameter("SYSID_MYGCS", self.mav.source_system)
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
        self.reboot_sitl()

    # Tests all actions and logic behind the battery failsafe
    def fly_battery_failsafe(self, timeout=300):
        ex = None
        try:
            self.test_battery_failsafe(timeout=timeout)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.set_parameter('BATT_LOW_VOLT', 0)
        self.set_parameter('BATT_CRT_VOLT', 0)
        self.set_parameter('BATT_FS_LOW_ACT', 0)
        self.set_parameter('BATT_FS_CRT_ACT', 0)
        self.set_parameter('FS_OPTIONS', 0)
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def test_battery_failsafe(self, timeout=300):
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
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("ALT_HOLD")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
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
        self.set_parameter('BATT_FS_LOW_ACT', 2)
        self.set_parameter('BATT_FS_CRT_ACT', 1)
        self.set_parameter('SIM_BATT_VOLTAGE', 11.4)
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
        # uninterupted.
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
        self.set_parameter('FS_OPTIONS', 0)
        self.set_parameter('BATT_FS_LOW_ACT', 1)
        self.set_parameter('BATT_FS_CRT_ACT', 1)
        self.set_parameter('FS_THR_ENABLE', 1)
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

    # fly_stability_patch - fly south, then hold loiter within 5m
    # position and altitude and reduce 1 motor to 60% efficiency
    def fly_stability_patch(self,
                            holdtime=30,
                            maxaltchange=5,
                            maxdistchange=10):

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

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_altitude = m.alt
        start = self.mav.location()
        tstart = self.get_sim_time()
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))

        # cut motor 1's to efficiency
        self.progress("Cutting motor 1 to 65% efficiency")
        self.set_parameter("SIM_ENGINE_MUL", 0.65)

        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
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
        self.set_parameter("FENCE_ENABLE", 1) # fence
        self.set_parameter("FENCE_TYPE", 2) # circle
        fence_radius = 15
        self.set_parameter("FENCE_RADIUS", fence_radius)
        fence_margin = 3
        self.set_parameter("FENCE_MARGIN", fence_margin)
        self.set_parameter("AVOID_ENABLE", 1)
        self.set_parameter("AVOID_BEHAVE", avoid_behave)
        self.set_parameter("RC10_OPTION", 40) # avoid-enable
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
        self.wait_distance_to_home(12, 20)
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

    def fly_fence_avoid_test(self, timeout=180):
        self.fly_fence_avoid_test_radius_check(avoid_behave=1, timeout=timeout)
        self.fly_fence_avoid_test_radius_check(avoid_behave=0, timeout=timeout)

    def assert_prearm_failure(self, expected_statustext, timeout=5, ignore_prearm_failures=[]):
        seen_statustext = False
        seen_command_ack = False

        self.drain_mav()
        tstart = self.get_sim_time_cached()
        arm_last_send = 0
        while True:
            if seen_command_ack and seen_statustext:
                break
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException(
                    "Did not see failure-to-arm messages (statustext=%s command_ack=%s" %
                    (seen_statustext, seen_command_ack))
            if now - arm_last_send > 1:
                arm_last_send = now
                self.send_mavlink_arm_command()
            m = self.mav.recv_match(blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                if expected_statustext in m.text:
                    self.progress("Got: %s" % str(m))
                    seen_statustext = True
                elif "PreArm" in m.text and m.text[8:] not in ignore_prearm_failures:
                    self.progress("Got: %s" % str(m))
                    raise NotAchievedException("Unexpected prearm failure (%s)" % m.text)

            if m.get_type() == "COMMAND_ACK":
                print("Got: %s" % str(m))
                if m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if m.result != 4:
                        raise NotAchievedException("command-ack says we didn't fail to arm")
                    self.progress("Got: %s" % str(m))
                    seen_command_ack = True
            if self.mav.motors_armed():
                raise NotAchievedException("Armed when we shouldn't have")

    # fly_fence_test - fly east until you hit the horizontal circular fence
    def fly_fence_test(self, timeout=180):
        # enable fence, disable avoidance
        self.set_parameter("FENCE_ENABLE", 1)
        self.set_parameter("AVOID_ENABLE", 0)

        self.change_mode("LOITER")
        self.wait_ready_to_arm()

        # fence requires home to be set:
        m = self.poll_home_position()
        if m is None:
            raise NotAchievedException("Did not receive HOME_POSITION")
        self.progress("home: %s" % str(m))

        self.start_subtest("ensure we can't arm if outside fence")
        self.load_fence("fence-in-middle-of-nowhere.txt")

        self.delay_sim_time(5) # let fence check run so it loads-from-eeprom
        self.assert_prearm_failure("vehicle outside fence")
        self.progress("Failed to arm outside fence (good!)")
        self.clear_fence()
        self.delay_sim_time(5) # let fence breach clear
        self.drain_mav()
        self.end_subtest("ensure we can't arm if outside fence")

        self.start_subtest("ensure we can't arm with bad radius")
        self.context_push()
        self.set_parameter("FENCE_RADIUS", -1)
        self.assert_prearm_failure("Invalid FENCE_RADIUS value")
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
        self.wait_heading(160)
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

            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            self.progress("Alt: %.02f  HomeDistance: %.02f (fence radius=%f)" %
                          (alt, home_distance, fence_radius))

        self.progress("Waiting until we get home and disarm")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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

    # fly_alt_max_fence_test - fly up until you hit the fence
    def fly_alt_max_fence_test(self):
        self.takeoff(10, mode="LOITER")
        """Hold loiter position."""

        # enable fence, disable avoidance
        self.set_parameter("FENCE_ENABLE", 1)
        self.set_parameter("FENCE_AUTOENABLE", 1)
        self.set_parameter("AVOID_ENABLE", 0)
        self.set_parameter("FENCE_TYPE", 1)

        self.change_alt(10)

        # first east
        self.progress("turning east")
        self.set_rc(4, 1580)
        self.wait_heading(160)
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

    # fly_alt_min_fence_test - fly down until you hit the fence
    def fly_alt_min_fence_test(self):
        self.takeoff(50, mode="LOITER", timeout=120)
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # enable fence, disable avoidance
        self.set_parameter("FENCE_ENABLE", 1)
        self.set_parameter("FENCE_AUTOENABLE", 1)
        self.set_parameter("AVOID_ENABLE", 0)
        self.set_parameter("FENCE_TYPE", 8)
        self.set_parameter("FENCE_ALT_MIN", 20)

        self.change_alt(50)

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160)
        self.set_rc(4, 1500)

        # fly forward (east) at least 20m
        self.set_rc(2, 1100)
        self.wait_distance(20)

        # stop flying forward and start flying based on input:
        self.set_rc(2, 1500)
        self.set_rc(3, 1200)

        # wait for fence to trigger
        self.wait_mode('RTL', timeout=120)

        self.wait_rtl_complete()

        self.zero_throttle()


    def fly_gps_glitch_loiter_test(self, timeout=30, max_distance=20):
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
        self.set_parameter("SIM_GPS_GLITCH_X", glitch_lat[glitch_current])
        self.set_parameter("SIM_GPS_GLITCH_Y", glitch_lon[glitch_current])

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
                    self.set_parameter("SIM_GPS_GLITCH_X", 0)
                    self.set_parameter("SIM_GPS_GLITCH_Y", 0)
                else:
                    self.progress("Applying glitch %u" % glitch_current)
                    # move onto the next glitch
                    self.set_parameter("SIM_GPS_GLITCH_X", glitch_lat[glitch_current])
                    self.set_parameter("SIM_GPS_GLITCH_Y", glitch_lon[glitch_current])

            # start displaying distance moved after all glitches applied
            if glitch_current == -1:
                m = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                        blocking=True)
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
            self.set_parameter("SIM_GPS_GLITCH_X", 0)
            self.set_parameter("SIM_GPS_GLITCH_Y", 0)
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        self.progress("GPS glitch test passed!"
                      "  stayed within %u meters for %u seconds" %
                      (max_distance, timeout))
        self.do_RTL()
        # re-arming is problematic because the GPS is glitching!
        self.reboot_sitl()

    # fly_gps_glitch_auto_test - fly mission and test reaction to gps glitch
    def fly_gps_glitch_auto_test(self, timeout=180):
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

        # record time and position
        tstart = self.get_sim_time()

        # initialise current glitch
        glitch_current = 0
        self.progress("Apply first glitch")
        self.set_parameter("SIM_GPS_GLITCH_X", glitch_lat[glitch_current])
        self.set_parameter("SIM_GPS_GLITCH_Y", glitch_lon[glitch_current])

        # record position for 30 seconds
        while glitch_current < glitch_num:
            tnow = self.get_sim_time()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # apply next glitch
                if glitch_current < glitch_num:
                    self.progress("Applying glitch %u" % glitch_current)
                    self.set_parameter("SIM_GPS_GLITCH_X",
                                       glitch_lat[glitch_current])
                    self.set_parameter("SIM_GPS_GLITCH_Y",
                                       glitch_lon[glitch_current])

        # turn off glitching
        self.progress("Completed Glitches")
        self.set_parameter("SIM_GPS_GLITCH_X", 0)
        self.set_parameter("SIM_GPS_GLITCH_Y", 0)

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
    def fly_simple(self, side=50):
        self.takeoff(10, mode="LOITER")
        # hold position in loiter
        self.change_mode('LOITER')

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
            self.mav.recv_match(type='VFR_HUD', blocking=True)
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
            self.mav.recv_match(type='VFR_HUD', blocking=True)
        self.set_rc(2, 1500)

        # hover in place
        self.hover()

        self.do_RTL(timeout=500)

    # fly_super_simple - flies a circle around home for 45 seconds
    def fly_super_simple(self, timeout=45):
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
            self.mav.recv_match(type='VFR_HUD', blocking=True)

        # stop rolling and yawing
        self.set_rc(1, 1500)
        self.set_rc(4, 1500)

        # restore simple mode parameters to default
        self.set_parameter("SUPER_SIMPLE", 0)

        # hover in place
        self.hover()

        self.do_RTL()

    # fly_circle - flies a circle with 20m radius
    def fly_circle(self, holdtime=36):
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
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_altitude = m.alt
        tstart = self.get_sim_time()
        self.progress("Circle at %u meters for %u seconds" %
                      (start_altitude, holdtime))
        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("heading %d" % m.heading)

        self.progress("CIRCLE OK for %u seconds" % holdtime)

        self.do_RTL()

    # test_mag_fail - test failover of compass in EKF
    def test_mag_fail(self):
        # we want both EK2 and EK3
        self.set_parameter("EK2_ENABLE", 1)
        self.set_parameter("EK3_ENABLE", 1)

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

    def wait_attitude(self, desroll=None, despitch=None, timeout=2, tolerance=10):
        '''wait for an attitude (degrees)'''
        if desroll is None and despitch is None:
            raise ValueError("despitch or desroll must be supplied")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 2:
                raise AutoTestTimeoutException("Failed to achieve attitude")
            m = self.mav.recv_match(type='ATTITUDE', blocking=True)
            roll_deg = math.degrees(m.roll)
            pitch_deg = math.degrees(m.pitch)
            self.progress("wait_att: roll=%f desroll=%s pitch=%f despitch=%s" %
                          (roll_deg, desroll, pitch_deg, despitch))
            if desroll is not None and abs(roll_deg - desroll) > tolerance:
                continue
            if despitch is not None and abs(pitch_deg - despitch) > tolerance:
                continue
            return

    def fly_flip(self):
        ex = None
        try:
            self.mavproxy.send("set streamrate -1\n")
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100)

            self.takeoff(20)
            self.hover()
            old_speedup = self.get_parameter("SIM_SPEEDUP")
            self.set_parameter('SIM_SPEEDUP', 1)
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
            self.wait_for_alt(20, max_err=40)

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
            self.set_parameter('SIM_SPEEDUP', old_speedup)
            self.do_RTL()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 0)
        sr = self.sitl_streamrate()
        self.mavproxy.send("set streamrate %u\n" % sr)
        if ex is not None:
            raise ex

    # fly_optical_flow_limits - test EKF navigation limiting
    def fly_optical_flow_limits(self):
        ex = None
        self.context_push()
        try:

            self.set_parameter("SIM_FLOW_ENABLE", 1)
            self.set_parameter("FLOW_TYPE", 10)

            # configure EKF to use optical flow instead of GPS
            ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
            if ahrs_ekf_type == 2:
                self.set_parameter("EK2_GPS_TYPE", 3)
            if ahrs_ekf_type == 3:
                self.set_parameter("EK3_SRC1_POSXY", 0)
                self.set_parameter("EK3_SRC1_VELXY", 5)
                self.set_parameter("EK3_SRC1_VELZ", 0)

            self.set_analog_rangefinder_parameters()

            self.set_parameter("SIM_GPS_DISABLE", 1)
            self.set_parameter("SIM_TERRAIN", 0)

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
                m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.set_rc(2, 1500)
        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def fly_autotune(self):
        """Test autotune mode"""

        rlld = self.get_parameter("ATC_RAT_RLL_D")
        rlli = self.get_parameter("ATC_RAT_RLL_I")
        rllp = self.get_parameter("ATC_RAT_RLL_P")
        self.takeoff(10)

        # hold position in loiter
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
            if "AutoTune: Success" in m.text:
                self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
                # near enough for now:
                self.change_mode('LAND')
                self.wait_landed_and_disarmed()
                # check the original gains have been re-instated
                if (rlld != self.get_parameter("ATC_RAT_RLL_D") or
                        rlli != self.get_parameter("ATC_RAT_RLL_I") or
                        rllp != self.get_parameter("ATC_RAT_RLL_P")):
                    raise NotAchievedException("AUTOTUNE gains still present")
                return

        raise NotAchievedException("AUTOTUNE failed (%u seconds)" %
                                   (self.get_sim_time() - tstart))

    def fly_autotune_switch(self):
        """Test autotune on a switch with gains being saved"""

        # autotune changes a set of parameters on the vehicle which
        # are not in our context.  That changes the flight
        # characterstics, which we can't afford between runs.  So
        # completely reset the simulated vehicle after the run is
        # complete by "customising" the commandline here:
        self.customise_SITL_commandline([])

        self.context_push()

        ex = None

        try:
            self.fly_autotune_switch_body()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def fly_autotune_switch_body(self):
        self.set_parameter("RC8_OPTION", 17)
        self.set_parameter("ATC_RAT_RLL_FLTT", 20)
        rlld = self.get_parameter("ATC_RAT_RLL_D")
        rlli = self.get_parameter("ATC_RAT_RLL_I")
        rllp = self.get_parameter("ATC_RAT_RLL_P")
        rllt = self.get_parameter("ATC_RAT_RLL_FLTT")
        self.progress("AUTOTUNE pre-gains are P:%f I:%f D:%f" %
                      (self.get_parameter("ATC_RAT_RLL_P"),
                       self.get_parameter("ATC_RAT_RLL_I"),
                       self.get_parameter("ATC_RAT_RLL_D")))
        self.takeoff(10, mode='LOITER')

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
            if "AutoTune: Success" in m.text:
                self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
                # Check original gains are re-instated
                self.set_rc(8, 1100)
                self.delay_sim_time(1)
                self.progress("AUTOTUNE original gains are P:%f I:%f D:%f" %
                              (self.get_parameter("ATC_RAT_RLL_P"), self.get_parameter("ATC_RAT_RLL_I"),
                               self.get_parameter("ATC_RAT_RLL_D")))
                if (rlld != self.get_parameter("ATC_RAT_RLL_D") or
                        rlli != self.get_parameter("ATC_RAT_RLL_I") or
                        rllp != self.get_parameter("ATC_RAT_RLL_P")):
                    raise NotAchievedException("AUTOTUNE gains still present")
                # Use autotuned gains
                self.set_rc(8, 1850)
                self.delay_sim_time(1)
                self.progress("AUTOTUNE testing gains are P:%f I:%f D:%f" %
                              (self.get_parameter("ATC_RAT_RLL_P"), self.get_parameter("ATC_RAT_RLL_I"),
                               self.get_parameter("ATC_RAT_RLL_D")))
                if (rlld == self.get_parameter("ATC_RAT_RLL_D") or
                        rlli == self.get_parameter("ATC_RAT_RLL_I") or
                        rllp == self.get_parameter("ATC_RAT_RLL_P")):
                    raise NotAchievedException("AUTOTUNE gains not present in pilot testing")
                # land without changing mode
                self.set_rc(3, 1000)
                self.wait_for_alt(0)
                self.wait_disarmed()
                # Check gains are still there after disarm
                if (rlld == self.get_parameter("ATC_RAT_RLL_D") or
                        rlli == self.get_parameter("ATC_RAT_RLL_I") or
                        rllp == self.get_parameter("ATC_RAT_RLL_P")):
                    raise NotAchievedException("AUTOTUNE gains not present on disarm")

                self.reboot_sitl()
                # Check gains are still there after reboot
                if (rlld == self.get_parameter("ATC_RAT_RLL_D") or
                        rlli == self.get_parameter("ATC_RAT_RLL_I") or
                        rllp == self.get_parameter("ATC_RAT_RLL_P")):
                    raise NotAchievedException("AUTOTUNE gains not present on reboot")
                # Check FLTT is unchanged
                if rllt != self.get_parameter("ATC_RAT_RLL_FLTT"):
                    raise NotAchievedException("AUTOTUNE FLTT was modified")
                return

        raise NotAchievedException("AUTOTUNE failed (%u seconds)" %
                                   (self.get_sim_time() - tstart))

    # fly_auto_test - fly mission which tests a significant number of commands
    def fly_auto_test(self):
        # Fly mission #1
        self.progress("# Load copter_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")

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

        self.progress("Auto mission completed: passed!")

    # fly_auto_test using CAN GPS - fly mission which tests normal operation alongside CAN GPS
    def fly_auto_test_using_can_gps(self):
        self.set_parameter("CAN_P1_DRIVER", 1)
        self.set_parameter("GPS_TYPE", 9)
        self.reboot_sitl()
        self.fly_auto_test()

    def fly_motor_fail(self, fail_servo=0, fail_mul=0.0, holdtime=30):
        """Test flight with reduced motor efficiency"""

        # we only expect an octocopter to survive ATM:
        servo_counts = {
            # 2: 6, # hexa
            3: 8,  # octa
            # 5: 6, # Y6
        }
        frame_class = int(self.get_parameter("FRAME_CLASS"))
        if frame_class not in servo_counts:
            self.progress("Test not relevant for frame_class %u" % frame_class)
            return

        servo_count = servo_counts[frame_class]

        if fail_servo < 0 or fail_servo > servo_count:
            raise ValueError('fail_servo outside range for frame class')

        self.takeoff(10, mode="LOITER")

        self.change_alt(alt_min=50)

        # Get initial values
        start_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_attitude = self.mav.recv_match(type='ATTITUDE', blocking=True)

        hover_time = 5
        try:
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

                servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                            blocking=True)
                hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
                attitude = self.mav.recv_match(type='ATTITUDE', blocking=True)

                if not failed and now - tstart > hover_time:
                    self.progress("Killing motor %u (%u%%)" %
                                  (fail_servo+1, fail_mul))
                    self.set_parameter("SIM_ENGINE_FAIL", fail_servo)
                    self.set_parameter("SIM_ENGINE_MUL", fail_mul)
                    failed = True

                if failed:
                    self.progress("Hold Time: %f/%f" % (now-tstart, holdtime))

                servo_pwm = [servo.servo1_raw,
                             servo.servo2_raw,
                             servo.servo3_raw,
                             servo.servo4_raw,
                             servo.servo5_raw,
                             servo.servo6_raw,
                             servo.servo7_raw,
                             servo.servo8_raw]

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

                if int_error_yaw_rate > 0.1:
                    raise NotAchievedException("Vehicle is spinning")

                if alt_delta < -20:
                    raise NotAchievedException("Vehicle is descending")

            self.set_parameter("SIM_ENGINE_FAIL", 0)
            self.set_parameter("SIM_ENGINE_MUL", 1.0)
        except Exception as e:
            self.set_parameter("SIM_ENGINE_FAIL", 0)
            self.set_parameter("SIM_ENGINE_MUL", 1.0)
            raise e

        self.do_RTL()

    def fly_motor_vibration(self):
        """Test flight with motor vibration"""
        self.context_push()

        ex = None
        try:
            self.set_rc_default()
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

            self.takeoff(15, mode="ALT_HOLD")

            hover_time = 15
            tstart = self.get_sim_time()
            self.progress("Hovering for %u seconds" % hover_time)
            while self.get_sim_time_cached() < tstart + hover_time:
                self.mav.recv_match(type='ATTITUDE', blocking=True)
            tend = self.get_sim_time()

            # if we don't reduce vibes here then the landing detector
            # may not trigger
            self.set_parameter("SIM_VIB_MOT_MAX", 0)
            self.do_RTL()
            psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
            # ignore the first 20Hz and look for a peak at -15dB or more
            ignore_bins = 20
            freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
            if numpy.amax(psd["X"][ignore_bins:]) < -15 or freq < 180 or freq > 300:
                raise NotAchievedException(
                    "Did not detect a motor peak, found %f at %f dB" %
                    (freq, numpy.amax(psd["X"][ignore_bins:])))
            else:
                self.progress("Detected motor peak at %fHz" % freq)

            # now add a notch and check that post-filter the peak is squashed below 40dB
            self.set_parameters({
                "INS_LOG_BAT_OPT": 2,
                "INS_NOTCH_ENABLE": 1,
                "INS_NOTCH_FREQ": freq,
                "INS_NOTCH_ATT": 50,
                "INS_NOTCH_BW": freq/2,
                "SIM_VIB_MOT_MAX": 350,
            })
            self.reboot_sitl()

            self.takeoff(15, mode="ALT_HOLD")

            tstart = self.get_sim_time()
            self.progress("Hovering for %u seconds" % hover_time)
            while self.get_sim_time_cached() < tstart + hover_time:
                self.mav.recv_match(type='ATTITUDE', blocking=True)
            tend = self.get_sim_time()
            self.set_parameter("SIM_VIB_MOT_MAX", 0)
            self.do_RTL()
            psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
            freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
            peakdB = numpy.amax(psd["X"][ignore_bins:])
            if peakdB < -23:
                self.progress("Did not detect a motor peak, found %f at %f dB" % (freq, peakdB))
            else:
                raise NotAchievedException("Detected peak %.1f Hz %.2f dB" % (freq, peakdB))
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
            self.disarm_vehicle(force=True)

        self.context_pop()

        self.reboot_sitl()

        if ex is not None:
            raise ex

    def fly_vision_position(self):
        """Disable GPS navigation, enable Vicon input."""
        # scribble down a location we can set origin to:

        self.customise_SITL_commandline(["--uartF=sim:vicon:"])
        self.progress("Waiting for location")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        old_pos = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print("old_pos=%s" % str(old_pos))

        self.context_push()

        ex = None
        try:
            # configure EKF to use external nav instead of GPS
            ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
            if ahrs_ekf_type == 2:
                self.set_parameter("EK2_GPS_TYPE", 3)
            if ahrs_ekf_type == 3:
                self.set_parameter("EK3_SRC1_POSXY", 6)
                self.set_parameter("EK3_SRC1_VELXY", 6)
                self.set_parameter("EK3_SRC1_POSZ", 6)
                self.set_parameter("EK3_SRC1_VELZ", 6)
            self.set_parameter("GPS_TYPE", 0)
            self.set_parameter("VISO_TYPE", 1)
            self.set_parameter("SERIAL5_PROTOCOL", 1)
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
                gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True)
                self.progress("gpi=%s" % str(gpi))
                if gpi.lat != 0:
                    break

                if self.get_sim_time_cached() - tstart > 60:
                    raise AutoTestTimeoutException("Did not get non-zero lat")

            self.takeoff()
            self.set_rc(1, 1600)
            tstart = self.get_sim_time()
            while True:
                vicon_pos = self.mav.recv_match(type='VISION_POSITION_ESTIMATE',
                                                blocking=True)
                # print("vpe=%s" % str(vicon_pos))
                self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                    blocking=True)
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
            while True:
                if self.get_sim_time_cached() - tstart > 200:
                    raise NotAchievedException("Did not disarm")
                self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                    blocking=True)
                # print("gpi=%s" % str(gpi))
                self.mav.recv_match(type='SIMSTATE',
                                    blocking=True)
                # print("ss=%s" % str(ss))
                # wait for RTL disarm:
                if not self.armed():
                    break

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()
        self.zero_throttle()
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def fly_gps_vicon_switching(self):
        """Fly GPS and Vicon switching test"""
        self.customise_SITL_commandline(["--uartF=sim:vicon:"])

        """Setup parameters including switching to EKF3"""
        self.context_push()
        ex = None
        try:
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
            self.reboot_sitl()

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
            self.set_parameter("GPS_TYPE", 0)

            # ensure vehicle remain in Loiter for 15 seconds
            tstart = self.get_sim_time()
            while self.get_sim_time() - tstart < 15:
                if not self.mode_is('LOITER'):
                    raise NotAchievedException("Expected to stay in loiter for >15 seconds")

            # RTL and check vehicle arrives within 10m of home
            self.set_rc(2, 1500)
            self.do_RTL()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def fly_rtl_speed(self):
        """Test RTL Speed parameters"""
        rtl_speed_ms = 7
        wpnav_speed_ms = 4
        wpnav_accel_mss = 3
        tolerance = 0.5
        self.load_mission("copter_rtl_speed.txt")
        self.set_parameter('WPNAV_ACCEL', wpnav_accel_mss * 100)
        self.set_parameter('RTL_SPEED', rtl_speed_ms * 100)
        self.set_parameter('WPNAV_SPEED', wpnav_speed_ms * 100)
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
        self.monitor_groundspeed(wpnav_speed_ms, timeout=5)
        self.do_RTL()

    def fly_nav_delay(self):
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
            if now - tstart > 120:
                raise AutoTestTimeoutException("Did not disarm as expected")
            m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True)
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

    def test_rangefinder(self):
        ex = None
        self.context_push()

        self.progress("Making sure we don't ordinarily get RANGEFINDER")
        m = self.mav.recv_match(type='RANGEFINDER',
                                blocking=True,
                                timeout=5)

        if m is not None:
            raise NotAchievedException("Received unexpected RANGEFINDER msg")

        # may need to force a rotation if some other test has used the
        # rangefinder...
        self.progress("Ensure no RFND messages in log")
        self.set_parameter("LOG_DISARMED", 1)
        if self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("Found unexpected RFND message")

        try:
            self.set_analog_rangefinder_parameters()
            self.set_parameter("RC9_OPTION", 10) # rangefinder
            self.set_rc(9, 2000)

            self.reboot_sitl()

            self.progress("Making sure we now get RANGEFINDER messages")
            m = self.mav.recv_match(type='RANGEFINDER',
                                    blocking=True,
                                    timeout=10)
            if m is None:
                raise NotAchievedException("Did not get expected RANGEFINDER msg")

            self.progress("Checking RangeFinder is marked as enabled in mavlink")
            m = self.mav.recv_match(type='SYS_STATUS',
                                    blocking=True,
                                    timeout=10)
            flags = m.onboard_control_sensors_enabled
            if not flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                raise NotAchievedException("Laser not enabled in SYS_STATUS")
            self.progress("Disabling laser using switch")
            self.set_rc(9, 1000)
            self.delay_sim_time(1)
            self.progress("Checking RangeFinder is marked as disabled in mavlink")
            m = self.mav.recv_match(type='SYS_STATUS',
                                    blocking=True,
                                    timeout=10)
            flags = m.onboard_control_sensors_enabled
            if flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                raise NotAchievedException("Laser enabled in SYS_STATUS")

            self.progress("Re-enabling rangefinder")
            self.set_rc(9, 2000)
            self.delay_sim_time(1)
            m = self.mav.recv_match(type='SYS_STATUS',
                                    blocking=True,
                                    timeout=10)
            flags = m.onboard_control_sensors_enabled
            if not flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                raise NotAchievedException("Laser not enabled in SYS_STATUS")

            self.takeoff(10, mode="LOITER")

            m_r = self.mav.recv_match(type='RANGEFINDER',
                                      blocking=True)
            m_p = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                      blocking=True)

            if abs(m_r.distance - m_p.relative_alt/1000) > 1:
                raise NotAchievedException(
                    "rangefinder/global position int mismatch %0.2f vs %0.2f" %
                    (m_r.distance, m_p.relative_alt/1000))

            self.land_and_disarm()

            if not self.current_onboard_log_contains_message("RFND"):
                raise NotAchievedException("Did not see expected RFND message")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_surface_tracking(self):
        ex = None
        self.context_push()

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
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            orig_absolute_alt_mm = m.alt

            self.progress("Original alt: absolute=%f" % orig_absolute_alt_mm)

            self.progress("Flying somewhere which surface is known lower compared to takeoff point")
            self.set_rc(2, 1450)
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 200:
                    raise NotAchievedException("Did not reach lower point")
                m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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
        ex = None
        self.context_push()

        try:
            self.set_analog_rangefinder_parameters()

            self.set_parameters({
                "RNGFND1_MAX_CM": 1500
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

        except Exception as e:
            self.print_exception_caught(e)
            self.disarm_vehicle(force=True)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_parachute(self):

        self.set_rc(9, 1000)
        self.set_parameter("CHUTE_ENABLED", 1)
        self.set_parameter("CHUTE_TYPE", 10)
        self.set_parameter("SERVO9_FUNCTION", 27)
        self.set_parameter("SIM_PARA_ENABLE", 1)
        self.set_parameter("SIM_PARA_PIN", 9)

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
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
                     2, # release
                     0,
                     0,
                     0,
                     0,
                     0,
                     0)
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

        self.context_push()
        self.progress("Crashing with 3pos switch in enable position")
        self.takeoff(40)
        self.set_rc(9, 1500)
        self.set_parameter("SIM_ENGINE_MUL", 0)
        self.set_parameter("SIM_ENGINE_FAIL", 1)
        self.wait_statustext('BANG', timeout=60)
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.context_pop()

        self.progress("Crashing with 3pos switch in disable position")
        loiter_alt = 10
        self.takeoff(loiter_alt, mode='LOITER')
        self.set_rc(9, 1100)
        self.set_parameter("SIM_ENGINE_MUL", 0)
        self.set_parameter("SIM_ENGINE_FAIL", 1)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + 5:
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is None:
                continue
            if "BANG" in m.text:
                self.set_rc(9, 1000)
                self.reboot_sitl()
                raise NotAchievedException("Parachute deployed when disabled")
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def test_motortest(self, timeout=60):
        self.start_subtest("Testing PWM output")
        pwm_in = 1300
        # default frame is "+" - start motor of 2 is "B", which is
        # motor 1... see
        # https://ardupilot.org/copter/docs/connect-escs-and-motors.html
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                     2, # start motor
                     mavutil.mavlink.MOTOR_TEST_THROTTLE_PWM,
                     pwm_in, # pwm-to-output
                     2, # timeout in seconds
                     2, # number of motors to output
                     0, # compass learning
                     0,
                     timeout=timeout)
        # long timeouts here because there's a pause before we start motors
        self.wait_servo_channel_value(1, pwm_in, timeout=10)
        self.wait_servo_channel_value(4, pwm_in, timeout=10)
        self.wait_statustext("finished motor test")
        self.end_subtest("Testing PWM output")

        self.start_subtest("Testing percentage output")
        percentage = 90.1
        # since MOT_SPIN_MIN and MOT_SPIN_MAX are not set, the RC3
        # min/max are used.
        expected_pwm = 1000 + (self.get_parameter("RC3_MAX") - self.get_parameter("RC3_MIN")) * percentage/100.0
        self.progress("expected pwm=%f" % expected_pwm)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                     2, # start motor
                     mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
                     percentage, # pwm-to-output
                     2, # timeout in seconds
                     2, # number of motors to output
                     0, # compass learning
                     0,
                     timeout=timeout)
        self.wait_servo_channel_value(1, expected_pwm, timeout=10)
        self.wait_servo_channel_value(4, expected_pwm, timeout=10)
        self.wait_statustext("finished motor test")
        self.end_subtest("Testing percentage output")

    def fly_precision_sitl(self):
        """Use SITL PrecLand backend precision messages to land aircraft."""

        self.context_push()

        ex = None
        try:
            self.set_parameter("PLND_ENABLED", 1)
            self.set_parameter("PLND_TYPE", 4)

            self.set_analog_rangefinder_parameters()
            self.set_parameter("SIM_SONAR_SCALE", 12)

            start = self.mav.location()
            target = start
            (target.lat, target.lng) = mavextra.gps_offset(start.lat, start.lng, 4, -4)
            self.progress("Setting target to %f %f" % (target.lat, target.lng))

            self.set_parameter("SIM_PLD_ENABLE", 1)
            self.set_parameter("SIM_PLD_LAT", target.lat)
            self.set_parameter("SIM_PLD_LON", target.lng)
            self.set_parameter("SIM_PLD_HEIGHT", 0)
            self.set_parameter("SIM_PLD_ALT_LMT", 15)
            self.set_parameter("SIM_PLD_DIST_LMT", 10)

            self.reboot_sitl()

            self.progress("Waiting for location")
            self.zero_throttle()
            self.takeoff(10, 1800)
            self.change_mode("LAND")
            self.wait_landed_and_disarmed()
            self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            new_pos = self.mav.location()
            delta = self.get_distance(target, new_pos)
            self.progress("Landed %f metres from target position" % delta)
            max_delta = 1
            if delta > max_delta:
                raise NotAchievedException("Did not land close enough to target position (%fm > %fm" % (delta, max_delta))

            if not self.current_onboard_log_contains_message("PL"):
                raise NotAchievedException("Did not see expected PL message")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.zero_throttle()
        self.context_pop()
        self.reboot_sitl()
        self.progress("All done")

        if ex is not None:
            raise ex

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
        while True:
            self.progress("Requesting request for seq %u" % (seq,))
            self.mav.mav.mission_write_partial_list_send(1, # target system
                                                         1, # target component
                                                         seq, # start index
                                                         seq)
            req = self.mav.recv_match(type='MISSION_REQUEST',
                                      blocking=True,
                                      timeout=1)
            if req is not None and req.seq == seq:
                if req.get_srcSystem() == 255:
                    self.progress("Shutup MAVProxy")
                    continue
                # notionally this might be in the message cache before
                # we prompt for it... *shrug*
                break

        # we have received a request for the item.  Supply it:

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

    def fly_nav_delay_abstime(self):
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
        reset_at_m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True)
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
            m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True)
            at_delay_item = ""
            if m.seq == delay_item_seq:
                at_delay_item = "(delay item)"
            self.progress("MISSION_CURRENT.seq=%u %s" % (m.seq, at_delay_item))
            if m.seq > delay_item_seq:
                if count_stop == -1:
                    count_stop_m = self.mav.recv_match(type='SYSTEM_TIME',
                                                       blocking=True)
                    count_stop = count_stop_m.time_unix_usec/1000000
        calculated_delay = count_stop - reset_at
        error = abs(calculated_delay - expected_delay)
        self.progress("Stopped for %u seconds (want >=%u seconds)" %
                      (calculated_delay, delay_for_seconds))
        if error > 2:
            raise NotAchievedException("delay outside expectations")

    def fly_nav_takeoff_delay_abstime(self):
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
            m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True)
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

    def fly_zigzag_mode(self):
        '''test zigzag mode'''

        # set channel 8 for zigzag savewp and recentre it
        self.set_parameter("RC8_OPTION", 61)

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

    def test_setting_modes_via_modeswitch(self):
        self.context_push()
        ex = None
        try:
            fltmode_ch = 5
            self.set_parameter("FLTMODE_CH", fltmode_ch)
            self.set_rc(fltmode_ch, 1000) # PWM for mode1
            testmodes = [("FLTMODE1", 4, "GUIDED", 1165),
                         ("FLTMODE2", 13, "SPORT", 1295),
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

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def test_setting_modes_via_auxswitch(self):
        self.context_push()
        ex = None
        try:
            fltmode_ch = int(self.get_parameter("FLTMODE_CH"))
            self.set_rc(fltmode_ch, 1000)
            self.wait_mode("CIRCLE")
            self.set_rc(9, 1000)
            self.set_rc(10, 1000)
            self.set_parameter("RC9_OPTION", 18) # land
            self.set_parameter("RC10_OPTION", 55) # guided
            self.set_rc(9, 1900)
            self.wait_mode("LAND")
            self.set_rc(10, 1900)
            self.wait_mode("GUIDED")
            self.set_rc(10, 1000) # this re-polls the mode switch
            self.wait_mode("CIRCLE")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

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
            0b1111111111111000, # mask specifying use-only-x-y-z
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
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            print("%s" % str(m))
            if (m.groundspeed < groundspeed_tolerance and
                    m.climb < climb_tolerance):
                break

    def fly_guided_move_global_relative_alt(self, lat, lon, alt):
        startpos = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                       blocking=True)

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
            # send a position-control command
            pos = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                      blocking=True)
            delta = self.get_distance_int(startpos, pos)
            self.progress("delta=%f (want >10)" % delta)
            if delta > 10:
                break

    def fly_guided_move_local(self, x, y, z_up, timeout=100):
        """move the vehicle using MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED"""
        startpos = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        self.progress("startpos=%s" % str(startpos))

        tstart = self.get_sim_time()
        # send a position-control command
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b1111111111111000, # mask specifying use-only-x-y-z
            x, # x
            y, # y
            -z_up,# z
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
                raise NotAchievedException("Did not start to move")
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            print("%s" % m)
            if m.groundspeed > 0.5:
                break

        self.progress("Waiting for vehicle to stop...")
        self.wait_groundspeed(1, 100, timeout=timeout)

        stoppos = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        self.progress("stop_pos=%s" % str(stoppos))

        x_achieved = stoppos.x - startpos.x
        if x_achieved - x > 1:
            raise NotAchievedException("Did not achieve x position: want=%f got=%f" % (x, x_achieved))

        y_achieved = stoppos.y - startpos.y
        if y_achieved - y > 1:
            raise NotAchievedException("Did not achieve y position: want=%f got=%f" % (y, y_achieved))

        z_achieved = stoppos.z - startpos.z
        if z_achieved - z_up > 1:
            raise NotAchievedException("Did not achieve z position: want=%f got=%f" % (z_up, z_achieved))

    def test_guided_local_position_target(self, x, y, z_up):
        """ Check target position being received by vehicle """
        # set POSITION_TARGET_LOCAL_NED message rate using SET_MESSAGE_INTERVAL
        self.progress("Setting local target in NED: (%f, %f, %f)" % (x, y, -z_up))
        self.progress("Setting rate to 1 Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 1)
        # set position target
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b1111111111111000, # mask specifying use only xyz
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
        m = self.mav.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=True, timeout=2)
        self.progress("Received local target: %s" % str(m))

        if not (m.type_mask == 0xFFF8 or m.type_mask == 0x0FF8):
            raise NotAchievedException("Did not receive proper mask: expected=65528 or 4088, got=%u" % m.type_mask)

        if x - m.x > 0.1:
            raise NotAchievedException("Did not receive proper target position x: wanted=%f got=%f" % (x, m.x))

        if y - m.y > 0.1:
            raise NotAchievedException("Did not receive proper target position y: wanted=%f got=%f" % (y, m.y))

        if z_up - (-m.z) > 0.1:
            raise NotAchievedException("Did not receive proper target position z: wanted=%f got=%f" % (z_up, -m.z))

    def test_guided_local_velocity_target(self, vx, vy, vz_up, timeout=3):
        " Check local target velocity being recieved by vehicle "
        self.progress("Setting local NED velocity target: (%f, %f, %f)" % (vx, vy, -vz_up))
        self.progress("Setting POSITION_TARGET_LOCAL_NED message rate to 10Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10)

        # Drain old messages and ignore the ramp-up to the required target velocity
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            # send velocity-control command
            self.mav.mav.set_position_target_local_ned_send(
                0, # timestamp
                1, # target system_id
                1, # target component id
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b1111111111000111, # mask specifying use only vx,vy,vz
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
            m = self.mav.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=True, timeout=1)

            if m is None:
                raise NotAchievedException("Did not receive any message for 1 sec")

            self.progress("Received local target: %s" % str(m))

        # Check the last received message
        if not (m.type_mask == 0xFFC7 or m.type_mask == 0x0FC7):
            raise NotAchievedException("Did not receive proper mask: expected=65479 or 4039, got=%u" % m.type_mask)

        if vx - m.vx > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vx: wanted=%f got=%f" % (vx, m.vx))

        if vy - m.vy > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vy: wanted=%f got=%f" % (vy, m.vy))

        if vz_up - (-m.vz) > 0.1:
            raise NotAchievedException("Did not receive proper target velocity vz: wanted=%f got=%f" % (vz_up, -m.vz))

        self.progress("Received proper target velocity commands")

    def test_position_target_message_mode(self):
        " Ensure that POSITION_TARGET_LOCAL_NED messages are sent in Guided Mode only "
        self.hover()
        self.change_mode('LOITER')
        self.progress("Setting POSITION_TARGET_LOCAL_NED message rate to 10Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10)

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + 5:
            m = self.mav.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=True, timeout=1)
            if m is None:
                continue

            raise NotAchievedException("Received POSITION_TARGET message in LOITER mode: %s" % str(m))

        self.progress("Did not receive any POSITION_TARGET_LOCAL_NED message in LOITER mode. Success")

    def earth_to_body(self, vector):
        m = self.mav.messages["ATTITUDE"]
        x = rotmat.Vector3(m.roll, m.pitch, m.yaw)
#        print('r=%f p=%f y=%f' % (m.roll, m.pitch, m.yaw))
        return vector - x

    def loiter_to_ne(self, x, y, z, timeout=40):
        dest = rotmat.Vector3(x, y, z)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not loiter to ne!")
            m_pos = self.mav.recv_match(type='LOCAL_POSITION_NED',
                                        blocking=True)
            pos = rotmat.Vector3(m_pos.x, m_pos.y, m_pos.z)
            delta_ef = pos - dest
            dist = math.sqrt(delta_ef.x * delta_ef.x + delta_ef.y * delta_ef.y)
            dist_max = 2
            self.progress("dist=%f want <%f" % (dist, dist_max))
            if dist < dist_max:
                break
            delta_bf = self.earth_to_body(delta_ef)
            angle_x = math.atan2(delta_bf.x, delta_bf.z)
            angle_y = math.atan2(delta_bf.y, delta_bf.z)
            distance = math.sqrt(delta_bf.x * delta_bf.x +
                                 delta_bf.y * delta_bf.y +
                                 delta_bf.z * delta_bf.z)
            self.mav.mav.landing_target_send(
                0, # time_usec
                1, # target_num
                mavutil.mavlink.MAV_FRAME_GLOBAL, # frame; AP ignores
                angle_x, # angle x (radians)
                angle_y, # angle y (radians)
                distance, # distance to target
                0.01, # size of target in radians, X-axis
                0.01 # size of target in radians, Y-axis
            )

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            m_pos = self.mav.recv_match(type='LOCAL_POSITION_NED',
                                        blocking=True)
            pos = rotmat.Vector3(m_pos.x, m_pos.y, m_pos.z)
            delta_ef = pos - dest
            dist = math.sqrt(delta_ef.x * delta_ef.x + delta_ef.y * delta_ef.y)
            self.progress("dist=%f" % (dist,))

    def fly_payload_place_mission(self):
        """Test payload placing in auto."""
        self.context_push()

        ex = None
        try:
            self.set_analog_rangefinder_parameters()
            self.set_parameter("GRIP_ENABLE", 1)
            self.set_parameter("GRIP_TYPE", 1)
            self.set_parameter("SIM_GRPS_ENABLE", 1)
            self.set_parameter("SIM_GRPS_PIN", 8)
            self.set_parameter("SERVO8_FUNCTION", 28)
            self.set_parameter("RC9_OPTION", 19)
            self.reboot_sitl()
            self.set_rc(9, 2000)
            # load the mission:
            self.load_mission("copter_payload_place.txt")

            self.progress("Waiting for location")
            self.mav.location()
            self.zero_throttle()
            self.change_mode('STABILIZE')
            self.wait_ready_to_arm()

            self.arm_vehicle()

            self.change_mode('AUTO')

            self.set_rc(3, 1500)
            self.wait_text("Gripper load releas", timeout=90)

            self.wait_disarmed()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()
        self.reboot_sitl()
        self.progress("All done")

        if ex is not None:
            raise ex

    def fly_guided_change_submode(self):
        """"Ensure we can move around in guided after a takeoff command."""

        '''start by disabling GCS failsafe, otherwise we immediately disarm
        due to (apparently) not receiving traffic from the GCS for
        too long.  This is probably a function of --speedup'''
        self.set_parameter("FS_GCS_ENABLE", 0)
        self.set_parameter("DISARM_DELAY", 0) # until traffic problems are fixed
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

        self.start_subtest("Check target position received by vehicle using SET_MESSAGE_INTERVAL")
        self.test_guided_local_position_target(5, 5, 10)
        self.test_guided_local_velocity_target(2, 2, 1)
        self.test_position_target_message_mode()

        self.do_RTL()

    def test_gripper_mission(self):
        self.context_push()
        ex = None
        try:
            self.load_mission("copter-gripper-mission.txt")
            self.change_mode('LOITER')
            self.wait_ready_to_arm()
            self.assert_vehicle_location_is_at_startup_location()
            self.arm_vehicle()
            self.change_mode('AUTO')
            self.set_rc(3, 1500)
            self.wait_statustext("Gripper Grabbed", timeout=60)
            self.wait_statustext("Gripper Released", timeout=60)
        except Exception as e:
            self.print_exception_caught(e)
            self.change_mode('LAND')
            ex = e
        self.context_pop()
        self.wait_disarmed()
        if ex is not None:
            raise ex

    def test_spline_last_waypoint(self):
        self.context_push()
        ex = None
        try:
            self.load_mission("copter-spline-last-waypoint.txt")
            self.change_mode('LOITER')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.change_mode('AUTO')
            self.set_rc(3, 1500)
            self.wait_altitude(10, 3000, relative=True)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.do_RTL()
        self.wait_disarmed()
        if ex is not None:
            raise ex

    def fly_manual_throttle_mode_change(self):
        self.set_parameter("FS_GCS_ENABLE", 0) # avoid GUIDED instant disarm
        self.change_mode("STABILIZE")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("ACRO")
        self.change_mode("STABILIZE")
        self.change_mode("GUIDED")
        self.set_rc(3, 1700)
        self.watch_altitude_maintained(-1, 0.2) # should not take off in guided
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
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            126,
            0,
            0,
            0,
            0,
            0,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            timeout=1
        )
        self.set_rc(3, 1000)
        self.run_cmd_do_set_mode("ACRO")
        self.wait_disarmed()

    def test_mount_pitch(self, despitch, despitch_tolerance, timeout=10, hold=0):
        tstart = self.get_sim_time()
        success_start = 0
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Mount pitch not achieved")

            m = self.mav.recv_match(type='MOUNT_STATUS',
                                    blocking=True,
                                    timeout=5)
#            self.progress("pitch=%f roll=%f yaw=%f" %
#                          (m.pointing_a, m.pointing_b, m.pointing_c))
            mount_pitch = m.pointing_a/100.0 # centidegrees to degrees
            if abs(despitch - mount_pitch) > despitch_tolerance:
                self.progress("Mount pitch incorrect: got=%f want=%f (+/- %f)" %
                              (mount_pitch, despitch, despitch_tolerance))
                success_start = 0
                continue
            self.progress("Mount pitch correct: %f degrees == %f" %
                          (mount_pitch, despitch))
            if success_start == 0:
                success_start = now
                continue
            if now - success_start > hold:
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
            0, # roll rate (rad/s)
            1, # pitch rate
            0, # yaw rate
            0.5) # thrust, 0 to 1, translated to a climb/descent rate

    def test_mount(self):
        ex = None
        self.context_push()
        old_srcSystem = self.mav.mav.srcSystem
        self.mav.mav.srcSystem = 250
        self.set_parameter("DISARM_DELAY", 0)
        try:
            '''start by disabling GCS failsafe, otherwise we immediately disarm
            due to (apparently) not receiving traffic from the GCS for
            too long.  This is probably a function of --speedup'''
            self.set_parameter("FS_GCS_ENABLE", 0)

            self.progress("Setting up servo mount")
            roll_servo = 5
            pitch_servo = 6
            yaw_servo = 7
            self.set_parameter("MNT_TYPE", 1)
            self.set_parameter("SERVO%u_FUNCTION" % roll_servo, 8) # roll
            self.set_parameter("SERVO%u_FUNCTION" % pitch_servo, 7) # pitch
            self.set_parameter("SERVO%u_FUNCTION" % yaw_servo, 6) # yaw
            self.reboot_sitl() # to handle MNT_TYPE changing

            # make sure we're getting mount status and gimbal reports
            self.mav.recv_match(type='MOUNT_STATUS',
                                blocking=True,
                                timeout=5)
            self.mav.recv_match(type='GIMBAL_REPORT',
                                blocking=True,
                                timeout=5)

            # test pitch isn't stabilising:
            m = self.mav.recv_match(type='MOUNT_STATUS',
                                    blocking=True,
                                    timeout=5)
            if m.pointing_a != 0 or m.pointing_b != 0 or m.pointing_c != 0:
                raise NotAchievedException("Mount stabilising when not requested")

            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.user_takeoff()

            despitch = 10
            despitch_tolerance = 3

            self.progress("Pitching vehicle")
            self.do_pitch(despitch) # will time out!

            self.wait_pitch(despitch, despitch_tolerance)

            # check we haven't modified:
            m = self.mav.recv_match(type='MOUNT_STATUS',
                                    blocking=True,
                                    timeout=5)
            if m.pointing_a != 0 or m.pointing_b != 0 or m.pointing_c != 0:
                raise NotAchievedException("Mount stabilising when not requested")

            self.progress("Enable pitch stabilization using MOUNT_CONFIGURE")
            self.mav.mav.mount_configure_send(
                1, # target system
                1, # target component
                mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
                0, # stab-roll
                1, # stab-pitch
                0)

            self.do_pitch(despitch)
            self.test_mount_pitch(-despitch, 1)

            self.progress("Disable pitch using MAV_CMD_DO_MOUNT_CONFIGURE")
            self.do_pitch(despitch)
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
                         mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         )
            self.test_mount_pitch(0, 0)

            self.progress("Point somewhere using MOUNT_CONTROL (ANGLE)")
            self.do_pitch(despitch)
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
                         mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         )
            self.mav.mav.mount_control_send(
                1, # target system
                1, # target component
                20 * 100, # pitch
                20 * 100, # roll (centidegrees)
                0,  # yaw
                0   # save position
            )
            self.test_mount_pitch(20, 1)

            self.progress("Point somewhere using MOUNT_CONTROL (GPS)")
            self.do_pitch(despitch)
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
                         mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         )
            start = self.mav.location()
            self.progress("start=%s" % str(start))
            (t_lat, t_lon) = mavextra.gps_offset(start.lat, start.lng, 10, 20)
            t_alt = 0

            self.progress("loc %f %f %f" % (start.lat, start.lng, start.alt))
            self.progress("targetting %f %f %f" % (t_lat, t_lon, t_alt))
            self.do_pitch(despitch)
            self.mav.mav.mount_control_send(
                1, # target system
                1, # target component
                int(t_lat * 1e7), # lat
                int(t_lon * 1e7), # lon
                t_alt * 100, # alt
                0  # save position
            )
            self.test_mount_pitch(-52, 5)

            # now test RC targetting
            self.progress("Testing mount RC targetting")

            # this is a one-off; ArduCopter *will* time out this directive!
            self.progress("Levelling aircraft")
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                1, # target sysid
                1, # target compid
                0, # bitmask of things to ignore
                mavextra.euler_to_quat([0, 0, 0]), # att
                1, # roll rate (rad/s)
                1, # pitch rate
                1, # yaw rate
                0.5) # thrust, 0 to 1, translated to a climb/descent rate

            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
                         mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         )
            try:
                self.context_push()
                self.set_parameter('MNT_RC_IN_ROLL', 11)
                self.set_parameter('MNT_RC_IN_TILT', 12)
                self.set_parameter('MNT_RC_IN_PAN', 13)
                self.progress("Testing RC angular control")
                # default RC min=1100 max=1900
                self.set_rc_from_map({
                    11: 1500,
                    12: 1500,
                    13: 1500,
                })
                self.test_mount_pitch(0, 1)
                self.progress("Testing RC input down 1/4 of its range in the output, should be down 1/4 range in output")
                rc12_in = 1400
                rc12_min = 1100 # default
                rc12_max = 1900 # default
                angmin_tilt = -45.0 # default
                angmax_tilt = 45.0 # default
                expected_pitch = (float(rc12_in-rc12_min)/float(rc12_max-rc12_min) * (angmax_tilt-angmin_tilt)) + angmin_tilt
                self.progress("expected mount pitch: %f" % expected_pitch)
                if expected_pitch != -11.25:
                    raise NotAchievedException("Calculation wrong - defaults changed?!")
                self.set_rc(12, rc12_in)
                self.test_mount_pitch(-11.25, 0.01)
                self.set_rc(12, 1800)
                self.test_mount_pitch(33.75, 0.01)
                self.set_rc_from_map({
                    11: 1500,
                    12: 1500,
                    13: 1500,
                })

                try:
                    self.progress(
                        "Issue https://discuss.ardupilot.org/t/"
                        "gimbal-limits-with-storm32-backend-mavlink-not-applied-correctly/51438"
                    )
                    self.context_push()
                    self.set_parameter("RC12_MIN", 1000)
                    self.set_parameter("RC12_MAX", 2000)
                    self.set_parameter("MNT_ANGMIN_TIL", -9000)
                    self.set_parameter("MNT_ANGMAX_TIL", 1000)
                    self.set_rc(12, 1000)
                    self.test_mount_pitch(-90.00, 0.01)
                    self.set_rc(12, 2000)
                    self.test_mount_pitch(10.00, 0.01)
                    self.set_rc(12, 1500)
                    self.test_mount_pitch(-40.00, 0.01)
                finally:
                    self.context_pop()

                self.set_rc(12, 1500)

                self.progress("Testing RC rate control")
                self.set_parameter('MNT_JSTICK_SPD', 10)
                self.test_mount_pitch(0, 1)
                self.set_rc(12, 1300)
                self.test_mount_pitch(-5, 1)
                self.test_mount_pitch(-10, 1)
                self.test_mount_pitch(-15, 1)
                self.test_mount_pitch(-20, 1)
                self.set_rc(12, 1700)
                self.test_mount_pitch(-15, 1)
                self.test_mount_pitch(-10, 1)
                self.test_mount_pitch(-5, 1)
                self.test_mount_pitch(0, 1)
                self.test_mount_pitch(5, 1)

                self.progress("Reverting to angle mode")
                self.set_parameter('MNT_JSTICK_SPD', 0)
                self.set_rc(12, 1500)
                self.test_mount_pitch(0, 0.1)

                self.context_pop()

            except Exception as e:
                self.print_exception_caught(e)
                self.context_pop()
                raise e

            self.progress("Testing mount ROI behaviour")
            self.drain_mav_unparsed()
            self.test_mount_pitch(0, 0.1)
            start = self.mav.location()
            self.progress("start=%s" % str(start))
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     10,
                                                     20)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI_LOCATION")
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                         0,
                         0,
                         0,
                         0,
                         roi_lat,
                         roi_lon,
                         roi_alt,
                         )
            self.test_mount_pitch(-52, 5)

            start = self.mav.location()
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     -100,
                                                     -200)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI")
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                         0,
                         0,
                         0,
                         0,
                         roi_lat,
                         roi_lon,
                         roi_alt,
                         )
            self.test_mount_pitch(-7.5, 1)

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
            self.test_mount_pitch(-7.5, 1)
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
            self.test_mount_pitch(-70, 1, hold=2)

            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
                         mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         )
            self.test_mount_pitch(0, 0.1)

            self.progress("Testing mount roi-sysid behaviour")
            self.test_mount_pitch(0, 0.1)
            start = self.mav.location()
            self.progress("start=%s" % str(start))
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     10,
                                                     20)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI_SYSID")
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
                         250,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
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
            self.test_mount_pitch(-89, 5, hold=2)

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
            self.test_mount_pitch(68, 5, hold=2)

            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
                         mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         )
            self.test_mount_pitch(0, 0.1)

            self.progress("checking ArduCopter yaw-aircraft-for-roi")
            try:
                self.context_push()

                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                self.progress("current heading %u" % m.heading)
                self.set_parameter("SERVO%u_FUNCTION" % yaw_servo, 0) # yaw
                self.progress("Waiting for check_servo_map to do its job")
                self.delay_sim_time(5)
                start = self.mav.location()
                self.progress("Moving to guided/position controller")
                # the following numbers are 1-degree-latitude and
                # 0-degrees longitude - just so that we start to
                # really move a lot.
                self.fly_guided_move_global_relative_alt(1, 0, 0)
                self.guided_achieve_heading(0)
                (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                         start.lng,
                                                         -100,
                                                         -200)
                roi_alt = 0
                self.progress("Using MAV_CMD_DO_SET_ROI")
                self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                             0,
                             0,
                             0,
                             0,
                             roi_lat,
                             roi_lon,
                             roi_alt,
                             )

                self.wait_heading(110, timeout=600)

                self.context_pop()
            except Exception:
                self.context_pop()
                raise

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()

        self.mav.mav.srcSystem = old_srcSystem
        self.disarm_vehicle(force=True)
        self.reboot_sitl() # to handle MNT_TYPE changing

        if ex is not None:
            raise ex

    def fly_throw_mode(self):
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
            self.set_parameter("SIM_SHOVE_TIME", 500, retries=3)
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

    def hover_and_check_matched_frequency_with_fft(self, dblevel=-15, minhz=200, maxhz=300, peakhz=None, reverse=None):
        # find a motor peak
        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 15
        tstart = self.get_sim_time()
        self.progress("Hovering for %u seconds" % hover_time)
        while self.get_sim_time_cached() < tstart + hover_time:
            self.mav.recv_match(type='ATTITUDE', blocking=True)
        vfr_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
        tend = self.get_sim_time()

        self.do_RTL()
        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

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
                    (freq, vfr_hud.throttle, peakdb))
            else:
                self.progress("Detected motor peak at %fHz, throttle %f%%, %fdB" % (freq, vfr_hud.throttle, peakdb))

        return freq, vfr_hud, peakdb

    def fly_dynamic_notches(self):
        """Use dynamic harmonic notch to control motor noise."""
        self.progress("Flying with dynamic notches")
        self.context_push()

        ex = None
        # we are dealing with probabalistic scenarios involving threads, have two bites at the cherry
        for loop in ["first", "second"]:
            try:
                self.set_rc_default()
                self.set_parameter("AHRS_EKF_TYPE", 10)
                self.set_parameter("INS_LOG_BAT_MASK", 3)
                self.set_parameter("INS_LOG_BAT_OPT", 0)
                # set the gyro filter high so we can observe behaviour
                self.set_parameter("INS_GYRO_FILTER", 100)
                self.set_parameter("LOG_BITMASK", 958)
                self.set_parameter("LOG_DISARMED", 0)
                self.set_parameter("SIM_VIB_MOT_MAX", 350)
                self.set_parameter("SIM_GYR1_RND", 20)
                self.reboot_sitl()

                self.takeoff(10, mode="ALT_HOLD")

                # find a motor peak
                freq, vfr_hud, peakdb = self.hover_and_check_matched_frequency_with_fft(-15, 200, 300)

                # now add a dynamic notch and check that the peak is squashed
                self.set_parameter("INS_LOG_BAT_OPT", 2)
                self.set_parameter("INS_HNTCH_ENABLE", 1)
                self.set_parameter("INS_HNTCH_FREQ", freq)
                self.set_parameter("INS_HNTCH_REF", vfr_hud.throttle/100.)
                # first and third harmonic
                self.set_parameter("INS_HNTCH_HMNCS", 5)
                self.set_parameter("INS_HNTCH_ATT", 50)
                self.set_parameter("INS_HNTCH_BW", freq/2)
                self.reboot_sitl()

                freq, vfr_hud, peakdb1 = self.hover_and_check_matched_frequency_with_fft(-10, 20, 350, reverse=True)

                # now add double dynamic notches and check that the peak is squashed
                self.set_parameter("INS_HNTCH_OPTS", 1)
                self.reboot_sitl()

                freq, vfr_hud, peakdb2 = self.hover_and_check_matched_frequency_with_fft(-15, 20, 350, reverse=True)

                # double-notch should do better, but check for within 5%
                if peakdb2 * 1.05 > peakdb1:
                    raise NotAchievedException(
                        "Double-notch peak was higher than single-notch peak %fdB > %fdB" %
                        (peakdb2, peakdb1))

            except Exception as e:
                self.print_exception_caught(e)
                self.progress("Exception caught in %s loop" % (loop,))
                if loop != "second":
                    continue
                ex = e
            break

        self.context_pop()

        if ex is not None:
            raise ex

    def hover_and_check_matched_frequency(self, dblevel=-15, minhz=200, maxhz=300, fftLength=32, peakhz=None):
        # find a motor peak
        self.takeoff(10, mode="ALT_HOLD")

        hover_time = 15
        tstart = self.get_sim_time()
        self.progress("Hovering for %u seconds" % hover_time)
        while self.get_sim_time_cached() < tstart + hover_time:
            self.mav.recv_match(type='ATTITUDE', blocking=True)
        vfr_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
        tend = self.get_sim_time()

        self.do_RTL()
        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(minhz * scale)
        smaxhz = int(maxhz * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])
        if peakdb < dblevel:
            raise NotAchievedException("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        elif peakhz is not None and abs(freq - peakhz) / peakhz > 0.05:
            raise NotAchievedException("Did not detect a motor peak at %fHz, found %fHz at %fdB" % (peakhz, freq, peakdb))
        else:
            self.progress("Detected motor peak at %fHz, throttle %f%%, %fdB" % (freq, vfr_hud.throttle, peakdb))

        # we have a peak make sure that the FFT detected something close
        # logging is at 10Hz
        mlog = self.dfreader_for_current_onboard_log()
        # accuracy is determined by sample rate and fft length, given our use of quinn we could probably use half of this
        freqDelta = 1000. / fftLength
        pkAvg = freq
        nmessages = 1

        m = mlog.recv_match(
            type='FTN1',
            blocking=False,
            condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6)
        )
        freqs = []
        while m is not None:
            nmessages = nmessages + 1
            freqs.append(m.PkAvg)
            m = mlog.recv_match(
                type='FTN1',
                blocking=False,
                condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6)
            )

        # peak within resolution of FFT length
        pkAvg = numpy.median(numpy.asarray(freqs))
        self.progress("Detected motor peak at %fHz processing %d messages" % (pkAvg, nmessages))

        # peak within 5%
        if abs(pkAvg - freq) > freqDelta:
            raise NotAchievedException("FFT did not detect a motor peak at %f, found %f, wanted %f" % (dblevel, pkAvg, freq))

        return freq

    def fly_gyro_fft_harmonic(self):
        """Use dynamic harmonic notch to control motor noise with harmonic matching of the first harmonic."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT harmonic - Gyro sample rate")
        self.context_push()
        ex = None
        # we are dealing with probabalistic scenarios involving threads, have two bites at the cherry
        for loop in ["first", "second"]:
            try:
                self.start_subtest("Hover to calculate approximate hover frequency")
                self.set_rc_default()
                # magic tridge EKF type that dramatically speeds up the test
                self.set_parameter("AHRS_EKF_TYPE", 10)
                self.set_parameter("EK2_ENABLE", 0)
                self.set_parameter("EK3_ENABLE", 0)
                self.set_parameter("INS_LOG_BAT_MASK", 3)
                self.set_parameter("INS_LOG_BAT_OPT", 0)
                self.set_parameter("INS_GYRO_FILTER", 100)
                self.set_parameter("INS_FAST_SAMPLE", 0)
                self.set_parameter("LOG_BITMASK", 958)
                self.set_parameter("LOG_DISARMED", 0)
                self.set_parameter("SIM_DRIFT_SPEED", 0)
                self.set_parameter("SIM_DRIFT_TIME", 0)
                self.set_parameter("FFT_THR_REF", self.get_parameter("MOT_THST_HOVER"))
                # enable a noisy motor peak
                self.set_parameter("SIM_GYR1_RND", 20)
                # enabling FFT will also enable the arming check, self-testing the functionality
                self.set_parameter("FFT_ENABLE", 1)
                self.set_parameter("FFT_MINHZ", 50)
                self.set_parameter("FFT_MAXHZ", 450)
                self.set_parameter("FFT_SNR_REF", 10)

                # Step 1: inject actual motor noise and use the FFT to track it
                self.set_parameter("SIM_VIB_MOT_MAX", 250) # gives a motor peak at about 175Hz
                self.set_parameter("FFT_WINDOW_SIZE", 64)
                self.set_parameter("FFT_WINDOW_OLAP", 0.75)

                self.reboot_sitl()
                freq = self.hover_and_check_matched_frequency(-15, 100, 250, 64)

                # Step 2: add a second harmonic and check the first is still tracked
                self.start_subtest("Add a fixed frequency harmonic at twice the hover frequency "
                                   "and check the right harmonic is found")
                self.set_parameter("SIM_VIB_FREQ_X", freq * 2)
                self.set_parameter("SIM_VIB_FREQ_Y", freq * 2)
                self.set_parameter("SIM_VIB_FREQ_Z", freq * 2)
                self.set_parameter("SIM_VIB_MOT_MULT", 0.25) # halve the motor noise so that the higher harmonic dominates
                self.reboot_sitl()

                self.hover_and_check_matched_frequency(-15, 100, 250, 64, None)

                # Step 3: switch harmonics mid flight and check for tracking
                self.start_subtest("Switch harmonics mid flight and check the right harmonic is found")
                self.set_parameter("FFT_HMNC_PEAK", 0)
                self.reboot_sitl()

                self.takeoff(10, mode="ALT_HOLD")

                hover_time = 10
                tstart = self.get_sim_time()
                self.progress("Hovering for %u seconds" % hover_time)
                while self.get_sim_time_cached() < tstart + hover_time:
                    self.mav.recv_match(type='ATTITUDE', blocking=True)
                vfr_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)

                self.set_parameter("SIM_VIB_MOT_MULT", 5.0)

                self.progress("Hovering for %u seconds" % hover_time)
                while self.get_sim_time_cached() < tstart + hover_time:
                    self.mav.recv_match(type='ATTITUDE', blocking=True)
                vfr_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
                tend = self.get_sim_time()

                self.do_RTL()

                mlog = self.dfreader_for_current_onboard_log()
                m = mlog.recv_match(
                    type='FTN1',
                    blocking=False,
                    condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6))
                freqs = []
                while m is not None:
                    freqs.append(m.PkAvg)
                    m = mlog.recv_match(
                        type='FTN1',
                        blocking=False,
                        condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6))

                # peak within resolution of FFT length, the highest energy peak switched but our detection should not
                pkAvg = numpy.median(numpy.asarray(freqs))
                freqDelta = 1000. / self.get_parameter("FFT_WINDOW_SIZE")

                if abs(pkAvg - freq) > freqDelta:
                    raise NotAchievedException("FFT did not detect a harmonic motor peak, found %f, wanted %f" % (pkAvg, freq))

                # Step 4: dynamic harmonic
                self.start_subtest("Enable dynamic harmonics and make sure both frequency peaks are attenuated")
                # find a motor peak
                freq, vfr_hud, peakdb = self.hover_and_check_matched_frequency_with_fft(-15, 100, 350)

                # now add a dynamic notch and check that the peak is squashed
                self.set_parameter("INS_LOG_BAT_OPT", 2)
                self.set_parameter("INS_HNTCH_ENABLE", 1)
                self.set_parameter("INS_HNTCH_HMNCS", 3)
                self.set_parameter("INS_HNTCH_MODE", 4)
                self.set_parameter("INS_HNTCH_FREQ", freq)
                # self.set_parameter("INS_HNTCH_REF", 1.0)
                self.set_parameter("INS_HNTCH_REF", vfr_hud.throttle/100.)
                self.set_parameter("INS_HNTCH_ATT", 100)
                self.set_parameter("INS_HNTCH_BW", freq/2)
                self.set_parameter("INS_HNTCH_OPTS", 3)
                self.reboot_sitl()

                # 5db is far in excess of the attenuation that the double dynamic-harmonic notch is able
                # to provide (-7dB on average), but without the notch the peak is around 20dB so still a safe test
                self.hover_and_check_matched_frequency_with_fft(5, 100, 350, reverse=True)

                self.set_parameter("SIM_VIB_FREQ_X", 0)
                self.set_parameter("SIM_VIB_FREQ_Y", 0)
                self.set_parameter("SIM_VIB_FREQ_Z", 0)
                self.set_parameter("SIM_VIB_MOT_MULT", 1.)
                # prevent update parameters from messing with the settings when we pop the context
                self.set_parameter("FFT_ENABLE", 0)
                self.reboot_sitl()

            except Exception as e:
                self.print_exception_caught(e)
                self.progress("Exception caught in %s loop" % (loop, ))
                if loop != "second":
                    continue
                ex = e
            break

        self.context_pop()

        # need a final reboot because weird things happen to your
        # vehicle state when switching back from EKF type 10!
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def fly_gyro_fft(self):
        """Use dynamic harmonic notch to control motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT - Gyro sample rate")
        self.context_push()

        ex = None
        # we are dealing with probabalistic scenarios involving threads, have two bites at the cherry
        for loop in ["first" "second"]:
            try:
                self.set_rc_default()
                # magic tridge EKF type that dramatically speeds up the test
                self.set_parameter("AHRS_EKF_TYPE", 10)
                self.set_parameter("EK2_ENABLE", 0)
                self.set_parameter("EK3_ENABLE", 0)
                self.set_parameter("INS_LOG_BAT_MASK", 3)
                self.set_parameter("INS_LOG_BAT_OPT", 0)
                self.set_parameter("INS_GYRO_FILTER", 100)
                self.set_parameter("INS_FAST_SAMPLE", 0)
                self.set_parameter("LOG_BITMASK", 958)
                self.set_parameter("LOG_DISARMED", 0)
                self.set_parameter("SIM_DRIFT_SPEED", 0)
                self.set_parameter("SIM_DRIFT_TIME", 0)
                # enable a noisy motor peak
                self.set_parameter("SIM_GYR1_RND", 20)
                # enabling FFT will also enable the arming check, self-testing the functionality
                self.set_parameter("FFT_ENABLE", 1)
                self.set_parameter("FFT_MINHZ", 50)
                self.set_parameter("FFT_MAXHZ", 450)
                self.set_parameter("FFT_SNR_REF", 10)
                self.set_parameter("FFT_WINDOW_SIZE", 128)
                self.set_parameter("FFT_WINDOW_OLAP", 0.75)
                self.set_parameter("FFT_SAMPLE_MODE", 0)

                # Step 1: inject a very precise noise peak at 250hz and make sure the in-flight fft
                # can detect it really accurately. For a 128 FFT the frequency resolution is 8Hz so
                # a 250Hz peak should be detectable within 5%
                self.start_subtest("Inject noise at 250Hz and check the FFT can find the noise")
                self.set_parameter("SIM_VIB_FREQ_X", 250)
                self.set_parameter("SIM_VIB_FREQ_Y", 250)
                self.set_parameter("SIM_VIB_FREQ_Z", 250)

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
                self.set_parameter("SIM_VIB_FREQ_X", 0)
                self.set_parameter("SIM_VIB_FREQ_Y", 0)
                self.set_parameter("SIM_VIB_FREQ_Z", 0)
                self.set_parameter("SIM_VIB_MOT_MAX", 250) # gives a motor peak at about 175Hz
                self.set_parameter("FFT_WINDOW_SIZE", 32)
                self.set_parameter("FFT_WINDOW_OLAP", 0.5)

                self.reboot_sitl()
                freq = self.hover_and_check_matched_frequency(-15, 100, 250, 32)

                self.set_parameter("SIM_VIB_MOT_MULT", 1.)

                # Step 3: add a FFT dynamic notch and check that the peak is squashed
                self.start_subtest("Add a dynamic notch, hover and check that the noise peak is now gone")
                self.set_parameter("INS_LOG_BAT_OPT", 2)
                self.set_parameter("INS_HNTCH_ENABLE", 1)
                self.set_parameter("INS_HNTCH_FREQ", freq)
                self.set_parameter("INS_HNTCH_REF", 1.0)
                self.set_parameter("INS_HNTCH_ATT", 50)
                self.set_parameter("INS_HNTCH_BW", freq/2)
                self.set_parameter("INS_HNTCH_MODE", 4)
                self.reboot_sitl()

                self.takeoff(10, mode="ALT_HOLD")
                hover_time = 15
                self.progress("Hovering for %u seconds" % hover_time)
                tstart = self.get_sim_time()
                while self.get_sim_time_cached() < tstart + hover_time:
                    self.mav.recv_match(type='ATTITUDE', blocking=True)
                tend = self.get_sim_time()

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
                if peakdb < -9:
                    self.progress("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
                else:
                    raise NotAchievedException("Detected %fHz motor peak at %fdB" % (freq, peakdb))

                # Step 4: loop sample rate test with larger window
                self.start_subtest("Hover and check that the FFT can find the motor noise when running at fast loop rate")
                # we are limited to half the loop rate for frequency detection
                self.set_parameter("FFT_MAXHZ", 185)
                self.set_parameter("INS_LOG_BAT_OPT", 0)
                self.set_parameter("SIM_VIB_MOT_MAX", 220)
                self.set_parameter("FFT_WINDOW_SIZE", 64)
                self.set_parameter("FFT_WINDOW_OLAP", 0.75)
                self.set_parameter("FFT_SAMPLE_MODE", 1)
                self.reboot_sitl()

                self.takeoff(10, mode="ALT_HOLD")

                self.progress("Hovering for %u seconds" % hover_time)
                tstart = self.get_sim_time()
                while self.get_sim_time_cached() < tstart + hover_time:
                    self.mav.recv_match(type='ATTITUDE', blocking=True)
                tend = self.get_sim_time()

                self.do_RTL()
                # prevent update parameters from messing with the settings when we pop the context
                self.set_parameter("FFT_ENABLE", 0)
                self.reboot_sitl()

            except Exception as e:
                self.progress("Exception caught in %s loop" % (loop, ))
                if loop != "second":
                    continue
                ex = e
            break

        self.context_pop()

        # must reboot after we move away from EKF type 10 to EKF2 or EKF3
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def fly_brake_mode(self):
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
            0b1111111111111000, # mask specifying use-only-lat-lon-alt
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

    def test_altitude_types(self):
        '''start by disabling GCS failsafe, otherwise we immediately disarm
        due to (apparently) not receiving traffic from the GCS for
        too long.  This is probably a function of --speedup'''

        '''this test flies the vehicle somewhere lower than were it started.
        It then disarms.  It then arms, which should reset home to the
        new, lower altitude.  This delta should be outside 1m but
        within a few metres of the old one.

        '''

        self.set_parameter("FS_GCS_ENABLE", 0)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        post_arming_home_offset_mm = m.alt - m.relative_alt
        self.progress("post-arming home offset: %f" % (post_arming_home_offset_mm))
        self.progress("gpi=%s" % str(m))
        min_post_arming_home_offset_delta_mm = -3000
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

    def fly_precision_companion(self):
        """Use Companion PrecLand backend precision messages to loiter."""

        self.context_push()

        ex = None
        try:
            self.set_parameter("PLND_ENABLED", 1)
            # enable companion backend:
            self.set_parameter("PLND_TYPE", 1)

            self.set_analog_rangefinder_parameters()

            # set up a channel switch to enable precision loiter:
            self.set_parameter("RC7_OPTION", 39)

            self.reboot_sitl()

            self.progress("Waiting for location")
            self.mav.location()
            self.zero_throttle()
            self.change_mode('STABILIZE')
            self.wait_ready_to_arm()

            # we should be doing precision loiter at this point
            start = self.mav.recv_match(type='LOCAL_POSITION_NED',
                                        blocking=True)

            self.arm_vehicle()
            self.set_rc(3, 1800)
            alt_min = 10
            self.wait_altitude(alt_min,
                               (alt_min + 5),
                               relative=True)
            self.set_rc(3, 1500)
            # move away a little
            self.set_rc(2, 1550)
            self.wait_distance(5, accuracy=1)
            self.set_rc(2, 1500)
            self.change_mode('LOITER')

            # turn precision loiter on:
            self.set_rc(7, 2000)

            # try to drag aircraft to a position 5 metres north-east-east:
            self.loiter_to_ne(start.x + 5, start.y + 10, start.z + 10)
            self.loiter_to_ne(start.x + 5, start.y - 10, start.z + 10)

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()
        self.zero_throttle()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.progress("All done")

        if ex is not None:
            raise ex

    def loiter_requires_position(self):
        # ensure we can't switch to LOITER without position
        self.progress("Ensure we can't enter LOITER without position")
        self.context_push()
        self.set_parameter("GPS_TYPE", 2)
        self.set_parameter("SIM_GPS_DISABLE", 1)
        self.reboot_sitl()

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

    def test_arm_feature(self):
        self.loiter_requires_position()

        super(AutoTestCopter, self).test_arm_feature()

    def test_parameter_checks(self):
        self.test_parameter_checks_poscontrol("PSC")

    def fly_poshold_takeoff(self):
        """ensure vehicle stays put until it is ready to fly"""
        self.context_push()

        ex = None
        try:
            self.set_parameter("PILOT_TKOFF_ALT", 700)
            self.change_mode('POSHOLD')
            self.set_rc(3, 1000)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.delay_sim_time(2)
            # check we are still on the ground...
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if abs(m.relative_alt) > 100:
                raise NotAchievedException("Took off prematurely")

            self.progress("Pushing throttle up")
            self.set_rc(3, 1710)
            self.delay_sim_time(0.5)
            self.progress("Bringing back to hover throttle")
            self.set_rc(3, 1500)

            # make sure we haven't already reached alt:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            max_initial_alt = 500
            if abs(m.relative_alt) > max_initial_alt:
                raise NotAchievedException("Took off too fast (%f > %f" %
                                           (abs(m.relative_alt), max_initial_alt))

            self.progress("Monitoring takeoff-to-alt")
            self.wait_altitude(6.9, 8, relative=True)

            self.progress("Making sure we stop at our takeoff altitude")
            tstart = self.get_sim_time()
            while self.get_sim_time() - tstart < 5:
                m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                delta = abs(7000 - m.relative_alt)
                self.progress("alt=%f delta=%f" % (m.relative_alt/1000,
                                                   delta/1000))
                if delta > 1000:
                    raise NotAchievedException("Failed to maintain takeoff alt")
            self.progress("takeoff OK")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.land_and_disarm()
        self.set_rc(8, 1000)

        self.context_pop()

        if ex is not None:
            raise ex

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

    def test_manual_control(self):
        '''test manual_control mavlink message'''
        self.set_parameter("SYSID_MYGCS", self.mav.source_system)

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
            m = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            print("m=%s" % str(m))
            if m is None:
                continue
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
        self.set_rc(2, 1400)
        west_loc = mavutil.location(-35.363007,
                                    149.164911,
                                    0,
                                    0)
        self.wait_location(west_loc, accuracy=6)
        north_loc = mavutil.location(-35.362908,
                                     149.165051,
                                     0,
                                     0)
        self.reach_heading_manual(0)
        self.wait_location(north_loc, accuracy=6, timeout=200)
        self.reach_heading_manual(90)
        east_loc = mavutil.location(-35.363013,
                                    149.165194,
                                    0,
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
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                raise NotAchievedException("Did not get correct angle back")

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
        self.context_push()
        ex = None
        try:
            self.set_parameters({
                "SERIAL5_PROTOCOL": 1,
                "PRX_TYPE": 2,
            })
            self.reboot_sitl()

            for angle in range(0, 360):
                self.OBSTACLE_DISTANCE_3D_test_angle(angle)

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def fly_proximity_avoidance_test_corners(self):
        self.start_subtest("Corners")
        self.context_push()
        ex = None
        try:
            self.load_fence("copter-avoidance-fence.txt")
            self.set_parameter("FENCE_ENABLE", 1)
            self.set_parameter("PRX_TYPE", 10)
            self.set_parameter("RC10_OPTION", 40) # proximity-enable
            self.reboot_sitl()
            self.progress("Enabling proximity")
            self.set_rc(10, 2000)
            self.check_avoidance_corners()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.clear_fence()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def fly_proximity_avoidance_test_alt_no_avoid(self):
        self.start_subtest("Alt-no-avoid")
        self.context_push()
        ex = None
        try:
            self.set_parameter("PRX_TYPE", 2)
            self.set_parameter("AVOID_ALT_MIN", 10)
            self.set_analog_rangefinder_parameters()
            self.reboot_sitl()
            tstart = self.get_sim_time()
            self.change_mode('LOITER')
            while True:
                if self.armed():
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
                self.send_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              1,  # ARM
                              0,
                              0,
                              0,
                              0,
                              0,
                              0)
                self.wait_heartbeat()
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
                if vel.length() > 0.3:
                    raise NotAchievedException("Moved too much (%s)" %
                                               (str(vel),))
                shove(None, None)

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def fly_proximity_avoidance_test(self):
        self.fly_proximity_avoidance_test_alt_no_avoid()
        self.fly_proximity_avoidance_test_corners()

    def fly_fence_avoidance_test(self):
        self.context_push()
        ex = None
        try:
            self.load_fence("copter-avoidance-fence.txt")
            self.set_parameter("FENCE_ENABLE", 1)
            self.check_avoidance_corners()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.clear_fence()
        self.disarm_vehicle(force=True)
        if ex is not None:
            raise ex

    def global_position_int_for_location(self, loc, time_boot, heading=0):
        return self.mav.mav.global_position_int_encode(
            int(time_boot * 1000), # time_boot_ms
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            int(loc.alt * 1000), # alt in mm
            20, # relative alt - urp.
            vx=0,
            vy=0,
            vz=0,
            hdg=heading
        )

    def fly_follow_mode(self):
        self.set_parameter("FOLL_ENABLE", 1)
        self.set_parameter("FOLL_SYSID", self.mav.source_system)
        foll_ofs_x = 30 # metres
        self.set_parameter("FOLL_OFS_X", -foll_ofs_x)
        self.set_parameter("FOLL_OFS_TYPE", 1) # relative to other vehicle heading
        self.takeoff(10, mode="LOITER")
        self.set_parameter("SIM_SPEEDUP", 1)
        self.change_mode("FOLLOW")
        new_loc = self.mav.location()
        new_loc_offset_n = 20
        new_loc_offset_e = 30
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
        self.progress("new_loc: %s" % str(new_loc))
        heading = 0
        self.mavproxy.send("map icon %f %f greenplane %f\n" %
                           (new_loc.lat, new_loc.lng, heading))

        expected_loc = copy.copy(new_loc)
        self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
        self.mavproxy.send("map icon %f %f hoop\n" %
                           (expected_loc.lat, expected_loc.lng))
        self.progress("expected_loc: %s" % str(expected_loc))

        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                raise NotAchievedException("Did not FOLLOW")
            if now - last_sent > 0.5:
                gpi = self.global_position_int_for_location(new_loc,
                                                            now,
                                                            heading=heading)
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
            self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            pos = self.mav.location()
            delta = self.get_distance(expected_loc, pos)
            max_delta = 3
            self.progress("position delta=%f (want <%f)" % (delta, max_delta))
            if delta < max_delta:
                break
        self.do_RTL()

    def get_global_position_int(self, timeout=30):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get good global_position_int")
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            self.progress("GPI: %s" % str(m))
            if m is None:
                continue
            if m.lat != 0 or m.lon != 0:
                return m

    def fly_beacon_position(self):
        self.reboot_sitl()

        self.wait_ready_to_arm(require_absolute=True)

        old_pos = self.get_global_position_int()
        print("old_pos=%s" % str(old_pos))

        self.context_push()
        ex = None
        try:
            self.set_parameter("BCN_TYPE", 10)
            self.set_parameter("BCN_LATITUDE", SITL_START_LOCATION.lat)
            self.set_parameter("BCN_LONGITUDE", SITL_START_LOCATION.lng)
            self.set_parameter("BCN_ALT", SITL_START_LOCATION.alt)
            self.set_parameter("BCN_ORIENT_YAW", 0)
            self.set_parameter("AVOID_ENABLE", 4)
            self.set_parameter("GPS_TYPE", 0)
            self.set_parameter("EK3_ENABLE", 1)
            self.set_parameter("EK3_SRC1_POSXY", 4) # Beacon
            self.set_parameter("EK3_SRC1_POSZ", 1)  # Baro
            self.set_parameter("EK3_SRC1_VELXY", 0) # None
            self.set_parameter("EK3_SRC1_VELZ", 0)  # None
            self.set_parameter("EK2_ENABLE", 0)
            self.set_parameter("AHRS_EKF_TYPE", 3)
            self.reboot_sitl()

            # turn off GPS arming checks.  This may be considered a
            # bug that we need to do this.
            old_arming_check = int(self.get_parameter("ARMING_CHECK"))
            if old_arming_check == 1:
                old_arming_check = 1 ^ 25 - 1
            new_arming_check = int(old_arming_check) & ~(1 << 3)
            self.set_parameter("ARMING_CHECK", new_arming_check)

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

            tstart = self.get_sim_time()
            max_delta = 0
            max_allowed_delta = 10
            while True:
                if self.get_sim_time_cached() - tstart > timeout:
                    break

                pos_delta = self.get_distance_int(self.sim_location_int(), self.get_global_position_int())
                self.progress("pos_delta=%f max_delta=%f max_allowed_delta=%f" % (pos_delta, max_delta, max_allowed_delta))
                if pos_delta > max_delta:
                    max_delta = pos_delta
                if pos_delta > max_allowed_delta:
                    raise NotAchievedException("Vehicle location not tracking simulated location (%f > %f)" %
                                               (pos_delta, max_allowed_delta))
            self.progress("Tracked location just fine (max_delta=%f)" % max_delta)
            self.change_mode("LOITER")
            self.wait_groundspeed(0, 0.3, timeout=120)
            self.land_and_disarm()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def fly_beacon_avoidance_test(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("BCN_TYPE", 10)
            self.set_parameter("BCN_LATITUDE", int(SITL_START_LOCATION.lat))
            self.set_parameter("BCN_LONGITUDE", int(SITL_START_LOCATION.lng))
            self.set_parameter("BCN_ORIENT_YAW", 45)
            self.set_parameter("AVOID_ENABLE", 4)
            self.reboot_sitl()

            self.takeoff(10, mode="LOITER")
            self.set_rc(2, 1400)
            west_loc = mavutil.location(-35.362919, 149.165055, 0, 0)
            self.wait_location(west_loc, accuracy=7)
            self.reach_heading_manual(0)
            north_loc = mavutil.location(-35.362881, 149.165103, 0, 0)
            self.wait_location(north_loc, accuracy=7)
            self.set_rc(2, 1500)
            self.set_rc(1, 1600)
            east_loc = mavutil.location(-35.362986, 149.165227, 0, 0)
            self.wait_location(east_loc, accuracy=7)
            self.set_rc(1, 1500)
            self.set_rc(2, 1600)
            south_loc = mavutil.location(-35.363025, 149.165182, 0, 0)
            self.wait_location(south_loc, accuracy=7)
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

    def fly_wind_baro_compensation(self):
        self.context_push()
        ex = None
        try:
            self.customise_SITL_commandline(
                ["--defaults", ','.join(self.model_defaults_filepath('ArduCopter', 'Callisto'))],
                model="octa-quad:@ROMFS/models/Callisto.json",
                wipe=True,
            )
            wind_spd_truth = 8.0
            wind_dir_truth = 90.0
            self.set_parameter("EK3_ENABLE", 1)
            self.set_parameter("EK2_ENABLE", 0)
            self.set_parameter("AHRS_EKF_TYPE", 3)
            self.set_parameter("BARO1_WCF_ENABLE", 1.000000)
            self.reboot_sitl()
            self.set_parameter("EK3_DRAG_BCOEF_X", 361.000000)
            self.set_parameter("EK3_DRAG_BCOEF_Y", 361.000000)
            self.set_parameter("EK3_DRAG_MCOEF", 0.082000)
            self.set_parameter("BARO1_WCF_FWD", -0.300000)
            self.set_parameter("BARO1_WCF_BCK", -0.300000)
            self.set_parameter("BARO1_WCF_RGT", 0.300000)
            self.set_parameter("BARO1_WCF_LFT", 0.300000)
            self.set_parameter("SIM_BARO_WCF_FWD", -0.300000)
            self.set_parameter("SIM_BARO_WCF_BAK", -0.300000)
            self.set_parameter("SIM_BARO_WCF_RGT", 0.300000)
            self.set_parameter("SIM_BARO_WCF_LFT", 0.300000)
            self.set_parameter("SIM_WIND_DIR", wind_dir_truth)
            self.set_parameter("SIM_WIND_SPD", wind_spd_truth)
            self.set_parameter("SIM_WIND_T", 1.000000)
            self.reboot_sitl()

            # require_absolute=True infers a GPS is present
            self.wait_ready_to_arm(require_absolute=False)

            self.progress("Climb to 20m in LOITER and yaw spin for 30 seconds")
            self.takeoff(10, mode="LOITER")
            self.set_rc(4, 1400)
            self.delay_sim_time(30)

            # check wind esitmates
            m = self.mav.recv_match(type='WIND', blocking=True)
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
                m = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True)
                if (m.z > z_max):
                    z_max = m.z
                if (m.z < z_min):
                    z_min = m.z
            if (z_max-z_min > 0.5):
                raise NotAchievedException("Height variation is excessive")
            self.progress("Height variation is good")

            self.set_rc(4, 1500)
            self.land_and_disarm()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def wait_generator_speed_and_state(self, rpm_min, rpm_max, want_state, timeout=240):
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not move to state/speed")

            m = self.mav.recv_match(type="GENERATOR_STATUS", blocking=True, timeout=10)
            if m is None:
                raise NotAchievedException("Did not get GENERATOR_STATUS")

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

    def test_richenpower(self):
        self.set_parameter("SERIAL5_PROTOCOL", 30)
        self.set_parameter("SIM_RICH_ENABLE", 1)
        self.set_parameter("SERVO8_FUNCTION", 42)
        self.set_parameter("SIM_RICH_CTRL", 8)
        self.set_parameter("RC9_OPTION", 85)
        self.set_parameter("LOG_DISARMED", 1)
        self.set_parameter("BATT2_MONITOR", 17)
        self.set_parameter("GEN_TYPE", 3)
        self.reboot_sitl()
        self.set_rc(9, 1000) # remember this is a switch position - stop
        self.customise_SITL_commandline(["--uartF=sim:richenpower"])
        self.wait_statustext("requested state is not RUN", timeout=60)

        self.set_message_rate_hz("GENERATOR_STATUS", 10)
        self.drain_mav_unparsed()

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        messages = []

        def my_message_hook(mav, m):
            if m.get_type() != 'STATUSTEXT':
                return
            messages.append(m)
        self.install_message_hook(my_message_hook)
        try:
            self.set_rc(9, 2000) # remember this is a switch position - run
        finally:
            self.remove_message_hook(my_message_hook)
        if "Generator HIGH" not in [x.text for x in messages]:
            self.wait_statustext("Generator HIGH", timeout=60)
        self.set_rc(9, 1000) # remember this is a switch position - stop
        self.wait_statustext("requested state is not RUN", timeout=200)

        self.set_rc(9, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 8000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.set_rc(9, 2000) # remember this is a switch position - run
#        self.wait_generator_speed_and_state(3000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_WARMING_UP)

        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        bs = self.mav.recv_match(
            type="BATTERY_STATUS",
            condition="BATTERY_STATUS.id==1",  # id is zero-indexed
            timeout=1,
            blocking=True
        )
        if bs is None:
            raise NotAchievedException("Did not receive BATTERY_STATUS")
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
        if not self.current_onboard_log_contains_message("GEN"):
            raise NotAchievedException("Did not find expected GEN message")

    def test_ie24(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("SERIAL5_PROTOCOL", 30)
            self.set_parameter("SERIAL5_BAUD", 115200)
            self.set_parameter("GEN_TYPE", 2)
            self.set_parameter("BATT2_MONITOR", 17)
            self.set_parameter("SIM_IE24_ENABLE", 1)
            self.set_parameter("LOG_DISARMED", 1)

            self.customise_SITL_commandline(["--uartF=sim:ie24"])
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.disarm_vehicle()

            # Test for pre-arm check fail when state is not running
            self.start_subtest("If you haven't taken off generator error should cause instant failsafe and disarm")
            self.set_parameter("SIM_IE24_STATE", 8)
            self.wait_statustext("Status not running")
            self.try_arm(result=False,
                         expect_msg="Status not running")
            self.set_parameter("SIM_IE24_STATE", 2) # Explicitly set state to running

            # Test that error code does result in failsafe
            self.start_subtest("If you haven't taken off generator error should cause instant failsafe and disarm")
            self.change_mode("STABILIZE")
            self.set_parameter("DISARM_DELAY", 0)
            self.arm_vehicle()
            self.set_parameter("SIM_IE24_ERROR", 30)
            self.disarm_wait(timeout=1)
            self.set_parameter("SIM_IE24_ERROR", 0)
            self.set_parameter("DISARM_DELAY", 10)

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def get_mission_count(self):
        return self.get_parameter("MIS_TOTAL")

    def assert_mission_count(self, expected):
        count = self.get_mission_count()
        if count != expected:
            raise NotAchievedException("Unexpected count got=%u want=%u" %
                                       (count, expected))

    def test_aux_switch_options(self):
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
        self.drain_mav()
        self.wait_current_waypoint(0, timeout=10)
        self.set_rc(7, 1000)

    def fly_rangefinder_drivers_fly(self, rangefinders):
        '''ensure rangefinder gives height-above-ground'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        expected_alt = 5
        self.user_takeoff(alt_min=expected_alt)
        rf = self.mav.recv_match(type="RANGEFINDER", timeout=1, blocking=True)
        if rf is None:
            raise NotAchievedException("Did not receive rangefinder message")
        gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if gpi is None:
            raise NotAchievedException("Did not receive GLOBAL_POSITION_INT message")
        if abs(rf.distance - gpi.relative_alt/1000.0) > 1:
            raise NotAchievedException(
                "rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                (rf.distance, gpi.relative_alt/1000.0)
            )

        for i in range(0, len(rangefinders)):
            name = rangefinders[i]
            self.progress("i=%u (%s)" % (i, name))
            ds = self.mav.recv_match(
                type="DISTANCE_SENSOR",
                timeout=2,
                blocking=True,
                condition="DISTANCE_SENSOR.id==%u" % i
            )
            if ds is None:
                raise NotAchievedException("Did not receive DISTANCE_SENSOR message for id==%u (%s)" % (i, name))
            self.progress("Got: %s" % str(ds))
            if abs(ds.current_distance/100.0 - gpi.relative_alt/1000.0) > 1:
                raise NotAchievedException(
                    "distance sensor.current_distance (%f) (%s) disagrees with global-position-int.relative_alt (%s)" %
                    (ds.current_distance/100.0, name, gpi.relative_alt/1000.0))

        self.land_and_disarm()

        self.progress("Ensure RFND messages in log")
        if not self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("No RFND messages in log")

    def fly_proximity_mavlink_distance_sensor(self):
        self.start_subtest("Test mavlink proximity sensor using DISTANCE_SENSOR messages")  # noqa
        self.context_push()
        ex = None
        try:
            self.set_parameter("SERIAL5_PROTOCOL", 1)
            self.set_parameter("PRX_TYPE", 2)  # mavlink
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
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def fly_rangefinder_mavlink_distance_sensor(self):
        self.start_subtest("Test mavlink rangefinder using DISTANCE_SENSOR messages")
        self.context_push()
        self.set_parameter('RTL_ALT_TYPE', 0)
        ex = None
        try:
            self.set_parameter("SERIAL5_PROTOCOL", 1)
            self.set_parameter("RNGFND1_TYPE", 10)
            self.reboot_sitl()
            self.set_parameter("RNGFND1_MAX_CM", 32767)

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
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_parameter("SERVO10_FUNCTION", 29)
            self.set_parameter("LGR_DEPLOY_ALT", 1)
            self.set_parameter("LGR_RETRACT_ALT", 10)  # metres
            self.delay_sim_time(1)  # servo function maps only periodically updated
#            self.send_debug_trap()

            self.run_cmd(
                mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION,
                0,
                0,  # deploy
                0,
                0,
                0,
                0,
                0
            )

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

    def test_gsf(self):
        '''test the Gaussian Sum filter'''
        ex = None
        self.context_push()
        try:
            self.set_parameter("EK2_ENABLE", 1)
            self.reboot_sitl()
            self.takeoff(20, mode='LOITER')
            self.set_rc(2, 1400)
            self.delay_sim_time(5)
            self.set_rc(2, 1500)
            self.progress("Path: %s" % self.current_onboard_log_filepath())
            dfreader = self.dfreader_for_current_onboard_log()
            self.do_RTL()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e

        self.context_pop()
        self.reboot_sitl()

        if ex is not None:
            raise ex

        # ensure log messages present
        want = set(["XKY0", "XKY1", "NKY0", "NKY1"])
        still_want = want
        while len(still_want):
            m = dfreader.recv_match(type=want)
            if m is None:
                raise NotAchievedException("Did not get %s" % want)
            still_want.remove(m.get_type())

    def fly_rangefinder_mavlink(self):
        self.fly_rangefinder_mavlink_distance_sensor()

        # explicit test for the mavlink driver as it doesn't play so nice:
        self.set_parameter("SERIAL5_PROTOCOL", 1)
        self.set_parameter("RNGFND1_TYPE", 10)
        self.customise_SITL_commandline(['--uartF=sim:rf_mavlink'])

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        expected_alt = 5
        self.user_takeoff(alt_min=expected_alt)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise NotAchievedException("Mavlink rangefinder not working")
            rf = self.mav.recv_match(type="RANGEFINDER", timeout=1, blocking=True)
            if rf is None:
                raise NotAchievedException("Did not receive rangefinder message")
            gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if gpi is None:
                raise NotAchievedException("Did not receive GLOBAL_POSITION_INT message")
            if abs(rf.distance - gpi.relative_alt/1000.0) > 1:
                print("rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                      (rf.distance, gpi.relative_alt/1000.0))
                continue

            ds = self.mav.recv_match(
                type="DISTANCE_SENSOR",
                timeout=2,
                blocking=True,
            )
            if ds is None:
                raise NotAchievedException("Did not receive DISTANCE_SENSOR message")
            self.progress("Got: %s" % str(ds))
            if abs(ds.current_distance/100.0 - gpi.relative_alt/1000.0) > 1:
                print(
                    "distance sensor.current_distance (%f) disagrees with global-position-int.relative_alt (%s)" %
                    (ds.current_distance/100.0, gpi.relative_alt/1000.0))
                continue
            break
        self.progress("mavlink rangefinder OK")
        self.land_and_disarm()

    def fly_rangefinder_driver_maxbotix(self):
        ex = None
        try:
            self.context_push()

            self.start_subtest("No messages")
            rf = self.mav.recv_match(type="DISTANCE_SENSOR", timeout=5, blocking=True)
            if rf is not None:
                raise NotAchievedException("Receiving DISTANCE_SENSOR when I shouldn't be")

            self.start_subtest("Default address")
            self.set_parameter("RNGFND1_TYPE", 2)  # maxbotix
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            rf = self.mav.recv_match(type="DISTANCE_SENSOR", timeout=5, blocking=True)
            self.progress("Got (%s)" % str(rf))
            if rf is None:
                raise NotAchievedException("Didn't receive DISTANCE_SENSOR when I should've")

            self.start_subtest("Explicitly set to default address")
            self.set_parameter("RNGFND1_TYPE", 2)  # maxbotix
            self.set_parameter("RNGFND1_ADDR", 0x70)
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            rf = self.mav.recv_match(type="DISTANCE_SENSOR", timeout=5, blocking=True)
            self.progress("Got (%s)" % str(rf))
            if rf is None:
                raise NotAchievedException("Didn't receive DISTANCE_SENSOR when I should've")

            self.start_subtest("Explicitly set to non-default address")
            self.set_parameter("RNGFND1_ADDR", 0x71)
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            rf = self.mav.recv_match(type="DISTANCE_SENSOR", timeout=5, blocking=True)
            self.progress("Got (%s)" % str(rf))
            if rf is None:
                raise NotAchievedException("Didn't receive DISTANCE_SENSOR when I should've")

            self.start_subtest("Two MaxBotix RangeFinders")
            self.set_parameter("RNGFND1_TYPE", 2)  # maxbotix
            self.set_parameter("RNGFND1_ADDR", 0x70)
            self.set_parameter("RNGFND1_MIN_CM", 150)
            self.set_parameter("RNGFND2_TYPE", 2)  # maxbotix
            self.set_parameter("RNGFND2_ADDR", 0x71)
            self.set_parameter("RNGFND2_MIN_CM", 250)
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            for i in [0, 1]:
                rf = self.mav.recv_match(
                    type="DISTANCE_SENSOR",
                    timeout=5,
                    blocking=True,
                    condition="DISTANCE_SENSOR.id==%u" % i
                )
                self.progress("Got id==%u (%s)" % (i, str(rf)))
                if rf is None:
                    raise NotAchievedException("Didn't receive DISTANCE_SENSOR when I should've")
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

    def fly_rangefinder_drivers(self):
        self.set_parameter("RTL_ALT", 500)
        self.set_parameter("RTL_ALT_TYPE", 1)
        drivers = [
            ("lightwareserial", 8),  # autodetected between this and -binary
            ("lightwareserial-binary", 8),
            ("ulanding_v0", 11),
            ("ulanding_v1", 11),
            ("leddarone", 12),
            ("maxsonarseriallv", 13),
            ("nmea", 17),
            ("wasp", 18),
            ("benewake_tf02", 19),
            ("blping", 23),
            ("benewake_tfmini", 20),
            ("lanbao", 26),
            ("benewake_tf03", 27),
            ("gyus42v2", 31),
        ]
        while len(drivers):
            do_drivers = drivers[0:3]
            drivers = drivers[3:]
            command_line_args = []
            for (offs, cmdline_argument, serial_num) in [(0, '--uartE', 4),
                                                         (1, '--uartF', 5),
                                                         (2, '--uartG', 6)]:
                if len(do_drivers) > offs:
                    (sim_name, rngfnd_param_value) = do_drivers[offs]
                    command_line_args.append("%s=sim:%s" %
                                             (cmdline_argument, sim_name))
                    serial_param_name = "SERIAL%u_PROTOCOL" % serial_num
                    self.set_parameter(serial_param_name, 9) # rangefinder
                    self.set_parameter("RNGFND%u_TYPE" % (offs+1), rngfnd_param_value)
            self.customise_SITL_commandline(command_line_args)
            self.fly_rangefinder_drivers_fly([x[0] for x in do_drivers])

        self.fly_rangefinder_mavlink()

        i2c_drivers = [
            ("maxbotixi2cxl", 2),
        ]
        while len(i2c_drivers):
            do_drivers = i2c_drivers[0:9]
            i2c_drivers = i2c_drivers[9:]
            count = 1
            for d in do_drivers:
                (sim_name, rngfnd_param_value) = d
                self.set_parameter("RNGFND%u_TYPE" % count, rngfnd_param_value)
                count += 1

            self.reboot_sitl()
            self.fly_rangefinder_drivers_fly([x[0] for x in do_drivers])

    def fly_ship_takeoff(self):
        # test ship takeoff
        self.wait_groundspeed(0, 2)
        self.set_parameter("SIM_SHIP_ENABLE", 1)
        self.set_parameter("SIM_SHIP_SPEED", 10)
        self.set_parameter("SIM_SHIP_DSIZE", 2)
        self.wait_ready_to_arm()
        # we should be moving with the ship
        self.wait_groundspeed(9, 11)
        self.takeoff(10)
        # above ship our speed drops to 0
        self.wait_groundspeed(0, 2)
        self.land_and_disarm()
        # ship will have moved on, so we land on the water which isn't moving
        self.wait_groundspeed(0, 2)

    def test_parameter_validation(self):
        # wait 10 seconds for initialisation
        self.delay_sim_time(10)
        self.progress("invalid; min must be less than max:")
        self.set_parameter("MOT_PWM_MIN", 100)
        self.set_parameter("MOT_PWM_MAX", 50)
        self.drain_mav()
        self.assert_prearm_failure("Check MOT_PWM_MIN/MAX")
        self.progress("invalid; min must be less than max (equal case):")
        self.set_parameter("MOT_PWM_MIN", 100)
        self.set_parameter("MOT_PWM_MAX", 100)
        self.drain_mav()
        self.assert_prearm_failure("Check MOT_PWM_MIN/MAX")
        self.progress("invalid; both must be non-zero or both zero (min=0)")
        self.set_parameter("MOT_PWM_MIN", 0)
        self.set_parameter("MOT_PWM_MAX", 100)
        self.drain_mav()
        self.assert_prearm_failure("Check MOT_PWM_MIN/MAX")
        self.progress("invalid; both must be non-zero or both zero (max=0)")
        self.set_parameter("MOT_PWM_MIN", 100)
        self.set_parameter("MOT_PWM_MAX", 0)
        self.drain_mav()
        self.assert_prearm_failure("Check MOT_PWM_MIN/MAX")

    def test_alt_estimate_prearm(self):
        self.context_push()
        ex = None
        try:
            # disable barometer so there is no altitude source
            self.set_parameter("SIM_BARO_DISABLE", 1)
            self.set_parameter("SIM_BARO2_DISABL", 1)
            self.wait_gps_disable(position_vertical=True)

            # turn off arming checks (mandatory arming checks will still be run)
            self.set_parameter("ARMING_CHECK", 0)

            # delay 12 sec to allow EKF to lose altitude estimate
            self.delay_sim_time(12)

            self.change_mode("ALT_HOLD")
            self.assert_prearm_failure("Need Alt Estimate")

            # force arm vehicle in stabilize to bypass barometer pre-arm checks
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

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.disarm_vehicle(force=True)
        if ex is not None:
            raise ex

    def test_ekf_source(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("EK3_ENABLE", 1)
            self.set_parameter("AHRS_EKF_TYPE", 3)
            self.wait_ready_to_arm()

            self.start_subtest("bad yaw source")
            self.set_parameter("EK3_SRC3_YAW", 17)
            self.assert_prearm_failure("Check EK3_SRC3_YAW")

            self.context_push()
            self.start_subtest("missing required yaw source")
            self.set_parameter("EK3_SRC3_YAW", 3) # External Yaw with Compass Fallback
            self.set_parameter("COMPASS_USE", 0)
            self.set_parameter("COMPASS_USE2", 0)
            self.set_parameter("COMPASS_USE3", 0)
            self.assert_prearm_failure("EK3 sources require Compass")
            self.context_pop()

        except Exception as e:
            self.disarm_vehicle(force=True)
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def test_replay_gps_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 1,
            "AHRS_TRIM_X": 0.01,
            "AHRS_TRIM_Y": -0.03,
            "GPS_TYPE2": 1,
            "GPS_POS1_X": 0.1,
            "GPS_POS1_Y": 0.2,
            "GPS_POS1_Z": 0.3,
            "GPS_POS2_X": -0.1,
            "GPS_POS2_Y": -0.02,
            "GPS_POS2_Z": -0.31,
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
            "SIM_GPS2_DISABLE": 0,
        })
        self.reboot_sitl()

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
        self.set_parameter("LOG_REPLAY", 1)
        self.set_parameter("LOG_DISARMED", 1)

        old_onboard_logs = sorted(self.log_list())
        self.fly_beacon_position()
        new_onboard_logs = sorted(self.log_list())

        log_difference = [x for x in new_onboard_logs if x not in old_onboard_logs]
        return log_difference[2]

    def test_replay_optical_flow_bit(self):
        self.set_parameter("LOG_REPLAY", 1)
        self.set_parameter("LOG_DISARMED", 1)

        old_onboard_logs = sorted(self.log_list())
        self.fly_optical_flow_limits()
        new_onboard_logs = sorted(self.log_list())

        log_difference = [x for x in new_onboard_logs if x not in old_onboard_logs]
        print("log difference: %s" % str(log_difference))
        return log_difference[0]

    def test_gps_blending(self):
        '''ensure we get dataflash log messages for blended instance'''

        self.context_push()

        ex = None

        try:
            # configure:
            self.set_parameter("GPS_TYPE2", 1)
            self.set_parameter("SIM_GPS2_TYPE", 1)
            self.set_parameter("SIM_GPS2_DISABLE", 0)
            self.set_parameter("GPS_AUTO_SWITCH", 2)
            self.reboot_sitl()

            # ensure we're seeing the second GPS:
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > 60:
                    raise NotAchievedException("Did not get good GPS2_RAW message")
                m = self.mav.recv_match(type='GPS2_RAW', blocking=True, timeout=1)
                self.progress("%s" % str(m))
                if m is None:
                    continue
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
            while True:
                m = dfreader.recv_match(type="GPS") # disarmed
                if m is None:
                    break
                try:
                    wanted.remove(m.I)
                except KeyError:
                    continue
                if len(wanted) == 0:
                    break

            if len(wanted):
                raise NotAchievedException("Did not get all three GPS types")
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e

        self.context_pop()

        self.reboot_sitl()

        if ex is not None:
            raise ex

    def test_callisto(self):
        self.customise_SITL_commandline(
            ["--defaults", ','.join(self.model_defaults_filepath('ArduCopter', 'Callisto')), ],
            model="octa-quad:@ROMFS/models/Callisto.json",
            wipe=True,
        )
        self.takeoff(10)
        self.do_RTL()

    def test_replay(self):
        '''test replay correctness'''
        self.progress("Building Replay")
        util.build_SITL('tools/Replay', clean=False, configure=False)

        self.test_replay_bit(self.test_replay_gps_bit)
        self.test_replay_bit(self.test_replay_beacon_bit)
        self.test_replay_bit(self.test_replay_optical_flow_bit)

    def test_replay_bit(self, bit):

        self.context_push()
        current_log_filepath = bit()

        self.progress("Running replay on (%s)" % current_log_filepath)

        util.run_cmd(['build/sitl/tools/Replay', current_log_filepath],
                     directory=util.topdir(), checkfail=True, show=True)

        self.context_pop()

        replay_log_filepath = self.current_onboard_log_filepath()
        self.progress("Replay log path: %s" % str(replay_log_filepath))

        check_replay = util.load_local_module("Tools/Replay/check_replay.py")

        ok = check_replay.check_log(replay_log_filepath, self.progress, verbose=True)
        if not ok:
            raise NotAchievedException("check_replay failed")

    # a wrapper around all the 1A,1B,1C..etc tests for travis
    def tests1(self):
        ret = ([])
        ret.extend(self.tests1a())
        ret.extend(self.tests1b())
        ret.extend(self.tests1c())
        ret.extend(self.tests1d())
        ret.extend(self.tests1e())
        return ret

    def tests1a(self):
        '''return list of all tests'''
        ret = super(AutoTestCopter, self).tests()  # about 5 mins and ~20 initial tests from autotest/common.py
        ret.extend([
            ("NavDelayTakeoffAbsTime",
             "Fly Nav Delay (takeoff)",
             self.fly_nav_takeoff_delay_abstime),  # 19s

            ("NavDelayAbsTime",
             "Fly Nav Delay (AbsTime)",
             self.fly_nav_delay_abstime),  # 20s

            ("NavDelay",
             "Fly Nav Delay",
             self.fly_nav_delay),  # 19s

            ("GuidedSubModeChange",
             "Test submode change",
             self.fly_guided_change_submode),

            ("LoiterToAlt",
             "Loiter-To-Alt",
             self.fly_loiter_to_alt),  # 25s

            ("PayLoadPlaceMission",
             "Payload Place Mission",
             self.fly_payload_place_mission),  # 44s

            ("PrecisionLoiterCompanion",
             "Precision Loiter (Companion)",
             self.fly_precision_companion),  # 29s

            ("PrecisionLandingSITL",
             "Precision Landing (SITL)",
             self.fly_precision_sitl),  # 29s

            ("SetModesViaModeSwitch",
             "Set modes via modeswitch",
             self.test_setting_modes_via_modeswitch),

            ("SetModesViaAuxSwitch",
             "Set modes via auxswitch",
             self.test_setting_modes_via_auxswitch),

            ("AuxSwitchOptions",
             "Test random aux mode options",
             self.test_aux_switch_options),

            ("AutoTune",
             "Fly AUTOTUNE mode",
             self.fly_autotune),  # 73s
        ])
        return ret

    def tests1b(self):
        '''return list of all tests'''
        ret = ([
            ("ThrowMode", "Fly Throw Mode", self.fly_throw_mode),

            ("BrakeMode", "Fly Brake Mode", self.fly_brake_mode),

            ("RecordThenPlayMission",
             "Use switches to toggle in mission, then fly it",
             self.fly_square),  # 27s

            ("ThrottleFailsafe",
             "Test Throttle Failsafe",
             self.fly_throttle_failsafe),  # 173s

            ("GCSFailsafe",
             "Test GCS Failsafe",
             self.fly_gcs_failsafe),  # 239s

            # this group has the smallest runtime right now at around
            #  5mins, so add more tests here, till its around
            #  9-10mins, then make a new group
        ])
        return ret

    def tests1c(self):
        '''return list of all tests'''
        ret = ([
            ("BatteryFailsafe",
             "Fly Battery Failsafe",
             self.fly_battery_failsafe),  # 164s

            ("StabilityPatch",
             "Fly stability patch",
             lambda: self.fly_stability_patch(30)),  # 17s

            ("OBSTACLE_DISTANCE_3D",
             "Test proximity avoidance slide behaviour in 3D",
             self.OBSTACLE_DISTANCE_3D),  # ??s

            ("AC_Avoidance_Proximity",
             "Test proximity avoidance slide behaviour",
             self.fly_proximity_avoidance_test),  # 41s

            ("AC_Avoidance_Fence",
             "Test fence avoidance slide behaviour",
             self.fly_fence_avoidance_test),

            ("AC_Avoidance_Beacon",
             "Test beacon avoidance slide behaviour",
             self.fly_beacon_avoidance_test),  # 28s

            ("BaroWindCorrection",
             "Test wind estimation and baro position error compensation",
             self.fly_wind_baro_compensation),

            ("SetpointGlobalPos",
             "Test setpoint global position",
             lambda: self.test_set_position_global_int()),

            ("SetpointGlobalVel",
             "Test setpoint global velocity",
             lambda: self.test_set_velocity_global_int()),

        ])
        return ret

    def tests1d(self):
        '''return list of all tests'''
        ret = ([
            ("HorizontalFence",
             "Test horizontal fence",
             self.fly_fence_test),  # 20s

            ("HorizontalAvoidFence",
             "Test horizontal Avoidance fence",
             self.fly_fence_avoid_test),

            ("MaxAltFence",
             "Test Max Alt Fence",
             self.fly_alt_max_fence_test),  # 26s

            ("MinAltFence",
             "Test Min Alt Fence",
             self.fly_alt_min_fence_test), #26s

            ("AutoTuneSwitch",
             "Fly AUTOTUNE on a switch",
             self.fly_autotune_switch),  # 105s

            ("GPSGlitchLoiter",
             "GPS Glitch Loiter Test",
             self.fly_gps_glitch_loiter_test),  # 30s

            ("GPSGlitchAuto",
             "GPS Glitch Auto Test",
             self.fly_gps_glitch_auto_test),

            ("ModeAltHold",
             "Test AltHold Mode",
             self.test_mode_ALT_HOLD),

            ("ModeLoiter",
             "Test Loiter Mode",
             self.loiter),

            ("SimpleMode",
             "Fly in SIMPLE mode",
             self.fly_simple),

            ("SuperSimpleCircle",
             "Fly a circle in SUPER SIMPLE mode",
             self.fly_super_simple),  # 38s

            ("ModeCircle",
             "Fly CIRCLE mode",
             self.fly_circle),  # 27s

            ("MagFail",
             "Test magnetometer failure",
             self.test_mag_fail),

            ("OpticalFlowLimits",
             "Fly Optical Flow limits",
             self.fly_optical_flow_limits),  # 27s

            ("MotorFail",
             "Fly motor failure test",
             self.fly_motor_fail),

            ("Flip",
             "Fly Flip Mode",
             self.fly_flip),

            ("CopterMission",
             "Fly copter mission",
             self.fly_auto_test),  # 37s

            ("SplineLastWaypoint",
             "Test Spline as last waypoint",
             self.test_spline_last_waypoint),

            ("Gripper",
             "Test gripper",
             self.test_gripper), # 28s

            ("TestGripperMission",
             "Test Gripper mission items",
             self.test_gripper_mission),

            ("VisionPosition",
             "Fly Vision Position",
             self.fly_vision_position), # 24s

            ("GPSViconSwitching",
             "Fly GPS and Vicon Switching",
             self.fly_gps_vicon_switching),
        ])
        return ret

    def tests1e(self):
        '''return list of all tests'''
        ret = ([
            ("BeaconPosition",
             "Fly Beacon Position",
             self.fly_beacon_position), # 56s

            ("RTLSpeed",
             "Fly RTL Speed",
             self.fly_rtl_speed),

            ("Mount",
             "Test Camera/Antenna Mount",
             self.test_mount),  # 74s

            ("Button",
             "Test Buttons",
             self.test_button),

            ("ShipTakeoff",
             "Fly Simulated Ship Takeoff",
             self.fly_ship_takeoff),

            ("RangeFinder",
             "Test RangeFinder Basic Functionality",
             self.test_rangefinder),  # 23s

            ("SurfaceTracking",
             "Test Surface Tracking",
             self.test_surface_tracking),  # 45s

            ("Parachute",
             "Test Parachute Functionality",
             self.test_parachute),

            ("ParameterChecks",
             "Test Arming Parameter Checks",
             self.test_parameter_checks),

            ("ManualThrottleModeChange",
             "Check manual throttle mode changes denied on high throttle",
             self.fly_manual_throttle_mode_change),

            ("MANUAL_CONTROL",
             "Test mavlink MANUAL_CONTROL",
             self.test_manual_control),

            ("ZigZag",
             "Fly ZigZag Mode",
             self.fly_zigzag_mode),  # 58s

            ("PosHoldTakeOff",
             "Fly POSHOLD takeoff",
             self.fly_poshold_takeoff),

            ("FOLLOW",
             "Fly follow mode",
             self.fly_follow_mode),  # 80s

            ("RangeFinderDrivers",
             "Test rangefinder drivers",
             self.fly_rangefinder_drivers),  # 62s

            ("MaxBotixI2CXL",
             "Test maxbotix rangefinder drivers",
             self.fly_rangefinder_driver_maxbotix),  # 62s

            ("MAVProximity",
             "Test MAVLink proximity driver",
             self.fly_proximity_mavlink_distance_sensor,
             ),

            ("ParameterValidation",
             "Test parameters are checked for validity",
             self.test_parameter_validation),

            ("AltTypes",
             "Test Different Altitude Types",
             self.test_altitude_types),

            ("RichenPower",
             "Test RichenPower generator",
             self.test_richenpower),

            ("IE24",
             "Test IntelligentEnergy 2.4kWh generator",
             self.test_ie24),

            ("LogUpload",
             "Log upload",
             self.log_upload),
        ])
        return ret

    # a wrapper around all the 2A,2B,2C..etc tests for travis
    def tests2(self):
        ret = ([])
        ret.extend(self.tests2a())
        ret.extend(self.tests2b())
        return ret

    def tests2a(self):
        '''return list of all tests'''
        ret = ([
            # something about SITLCompassCalibration appears to fail
            # this one, so we put it first:
            ("FixedYawCalibration",
             "Test Fixed Yaw Calibration",  # about 20 secs
             self.test_fixed_yaw_calibration),

            # we run this single 8min-and-40s test on its own, apart from
            #   requiring FixedYawCalibration right before it because without it, it fails to calibrate
            ("SITLCompassCalibration", # this autotest appears to interfere with FixedYawCalibration, no idea why.
             "Test SITL onboard compass calibration",
             self.test_mag_calibration),
        ])
        return ret

    def tests2b(self):  # this block currently around 9.5mins here
        '''return list of all tests'''
        ret = ([
            Test("MotorVibration",
                 "Fly motor vibration test",
                 self.fly_motor_vibration),

            Test("DynamicNotches",
                 "Fly Dynamic Notches",
                 self.fly_dynamic_notches,
                 attempts=4),

            Test("GyroFFT",
                 "Fly Gyro FFT",
                 self.fly_gyro_fft,
                 attempts=4),

            Test("GyroFFTHarmonic",
                 "Fly Gyro FFT Harmonic Matching",
                 self.fly_gyro_fft_harmonic,
                 attempts=4),

            Test("CompassReordering",
                 "Test Compass reordering when priorities are changed",
                 self.test_mag_reordering),  # 40sec?

            Test("CRSF",
                 "Test RC CRSF",
                 self.test_crsf),  # 20secs ish

            Test("MotorTest",
                 "Run Motor Tests",
                 self.test_motortest),  # 20secs ish

            Test("AltEstimation",
                 "Test that Alt Estimation is mandatory for ALT_HOLD",
                 self.test_alt_estimate_prearm),  # 20secs ish

            Test("EKFSource",
                 "Check EKF Source Prearms work",
                 self.test_ekf_source),

            Test("GSF",
                 "Check GSF",
                 self.test_gsf),

            Test("GPSBlending",
                 "Test GPS Blending",
                 self.test_gps_blending),

            Test("DataFlash",
                 "Test DataFlash Block backend",
                 self.test_dataflash_sitl),

            Test("DataFlashErase",
                 "Test DataFlash Block backend erase",
                 self.test_dataflash_erase),

            Test("Callisto",
                 "Test Callisto",
                 self.test_callisto),

            Test("Replay",
                 "Test Replay",
                 self.test_replay),

            Test("LogUpload",
                 "Log upload",
                 self.log_upload),
        ])
        return ret

    def testcan(self):
        ret = ([
            ("CANGPSCopterMission",
             "Fly copter mission",
             self.fly_auto_test_using_can_gps),
        ])
        return ret

    def tests(self):
        ret = []
        ret.extend(self.tests1())
        ret.extend(self.tests2())
        return ret

    def disabled_tests(self):
        return {
            "Parachute": "See https://github.com/ArduPilot/ardupilot/issues/4702",
            "HorizontalAvoidFence": "See https://github.com/ArduPilot/ardupilot/issues/11525",
            "AltEstimation": "See https://github.com/ArduPilot/ardupilot/issues/15191",
        }


class AutoTestHeli(AutoTestCopter):

    def log_name(self):
        return "HeliCopter"

    def default_frame(self):
        return "heli"

    def sitl_start_location(self):
        return SITL_START_LOCATION_AVC

    def is_heli(self):
        return True

    def rc_defaults(self):
        ret = super(AutoTestHeli, self).rc_defaults()
        ret[8] = 1000
        ret[3] = 1000 # collective
        return ret

    @staticmethod
    def get_position_armable_modes_list():
        '''filter THROW mode out of armable modes list; Heli is special-cased'''
        ret = AutoTestCopter.get_position_armable_modes_list()
        ret = filter(lambda x : x != "THROW", ret)
        return ret

    def loiter_requires_position(self):
        self.progress("Skipping loiter-requires-position for heli; rotor runup issues")

    def get_collective_out(self):
        servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        chan_pwm = (servo.servo1_raw + servo.servo2_raw + servo.servo3_raw)/3.0
        return chan_pwm

    def rotor_runup_complete_checks(self):
        # Takeoff and landing in Loiter
        TARGET_RUNUP_TIME = 10
        self.zero_throttle()
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        coll = servo.servo1_raw
        coll = coll + 50
        self.set_parameter("H_RSC_RUNUP_TIME", TARGET_RUNUP_TIME)
        self.progress("Initiate Runup by putting some throttle")
        self.set_rc(8, 2000)
        self.set_rc(3, 1700)
        self.progress("Collective threshold PWM %u" % coll)
        tstart = self.get_sim_time()
        self.progress("Wait that collective PWM pass threshold value")
        servo = self.mav.recv_match(condition='SERVO_OUTPUT_RAW.servo1_raw>%u' % coll, blocking=True)
        runup_time = self.get_sim_time() - tstart
        self.progress("Collective is now at PWM %u" % servo.servo1_raw)
        self.mav.wait_heartbeat()
        if runup_time < TARGET_RUNUP_TIME:
            self.zero_throttle()
            self.set_rc(8, 1000)
            self.disarm_vehicle()
            self.mav.wait_heartbeat()
            raise NotAchievedException("Takeoff initiated before runup time complete %u" % runup_time)
        self.progress("Runup time %u" % runup_time)
        self.zero_throttle()
        self.set_rc(8, 1000)
        self.land_and_disarm()
        self.mav.wait_heartbeat()

    # fly_avc_test - fly AVC mission
    def fly_avc_test(self):
        # Arm
        self.change_mode('STABILIZE')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)

        # upload mission from file
        self.progress("# Load copter_AVC2013_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_AVC2013_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_AVC2013_mission failed")

        self.progress("Fly AVC mission from 1 to %u" % num_wp)
        self.set_current_waypoint(1)

        # wait for motor runup
        self.delay_sim_time(20)

        # switch into AUTO mode and raise throttle
        self.change_mode('AUTO')
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # set throttle to minimum
        self.zero_throttle()

        # wait for disarm
        self.wait_disarmed()
        self.progress("MOTORS DISARMED OK")

        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

        self.progress("AVC mission completed: passed!")

    def fly_heli_poshold_takeoff(self):
        """ensure vehicle stays put until it is ready to fly"""
        self.context_push()

        ex = None
        try:
            self.set_parameter("PILOT_TKOFF_ALT", 700)
            self.change_mode('POSHOLD')
            self.zero_throttle()
            self.set_rc(8, 1000)
            self.wait_ready_to_arm()
            # Arm
            self.arm_vehicle()
            self.progress("Raising rotor speed")
            self.set_rc(8, 2000)
            self.progress("wait for rotor runup to complete")
            self.wait_servo_channel_value(8, 1660, timeout=10)
            self.delay_sim_time(20)
            # check we are still on the ground...
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            max_relalt_mm = 1000
            if abs(m.relative_alt) > max_relalt_mm:
                raise NotAchievedException("Took off prematurely (abs(%f)>%f)" %
                                           (m.relative_alt, max_relalt_mm))
            self.progress("Pushing collective past half-way")
            self.set_rc(3, 1600)
            self.delay_sim_time(0.5)
            self.progress("Bringing back to hover collective")
            self.set_rc(3, 1500)

            # make sure we haven't already reached alt:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if abs(m.relative_alt) > 500:
                raise NotAchievedException("Took off too fast")

            self.progress("Monitoring takeoff-to-alt")
            self.wait_altitude(6.9, 8, relative=True)

            self.progress("Making sure we stop at our takeoff altitude")
            tstart = self.get_sim_time()
            while self.get_sim_time() - tstart < 5:
                m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                delta = abs(7000 - m.relative_alt)
                self.progress("alt=%f delta=%f" % (m.relative_alt/1000,
                                                   delta/1000))
                if delta > 1000:
                    raise NotAchievedException("Failed to maintain takeoff alt")
            self.progress("takeoff OK")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.land_and_disarm()
        self.set_rc(8, 1000)

        self.context_pop()

        if ex is not None:
            raise ex

    def fly_heli_stabilize_takeoff(self):
        """"""
        self.context_push()

        ex = None
        try:
            self.change_mode('STABILIZE')
            self.set_rc(3, 1000)
            self.set_rc(8, 1000)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_rc(8, 2000)
            self.progress("wait for rotor runup to complete")
            self.wait_servo_channel_value(8, 1660, timeout=10)
            self.delay_sim_time(20)
            # check we are still on the ground...
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if abs(m.relative_alt) > 100:
                raise NotAchievedException("Took off prematurely")
            self.progress("Pushing throttle past half-way")
            self.set_rc(3, 1600)

            self.progress("Monitoring takeoff")
            self.wait_altitude(6.9, 8, relative=True)

            self.progress("takeoff OK")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.land_and_disarm()
        self.set_rc(8, 1000)

        self.context_pop()

        if ex is not None:
            raise ex

    def fly_spline_waypoint(self, timeout=600):
        """ensure basic spline functionality works"""
        self.load_mission("copter_spline_mission.txt")
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.delay_sim_time(20)
        self.change_mode("AUTO")
        self.set_rc(3, 1500)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Vehicle did not disarm after mission")
            if not self.armed():
                break
            self.delay_sim_time(1)
        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

    def fly_autorotation(self, timeout=600):
        """ensure basic spline functionality works"""
        self.set_parameter("AROT_ENABLE", 1)
        start_alt = 100 # metres
        self.set_parameter("PILOT_TKOFF_ALT", start_alt * 100)
        self.change_mode('POSHOLD')
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        self.wait_servo_channel_value(8, 1660, timeout=10)
        self.delay_sim_time(20)
        self.set_rc(3, 2000)
        self.wait_altitude(start_alt - 1,
                           (start_alt + 5),
                           relative=True,
                           timeout=timeout)
        self.context_collect('STATUSTEXT')
        self.progress("Triggering autorotate by raising interlock")
        self.set_rc(8, 1000)
        self.wait_statustext("SS Glide Phase", check_context=True)
        self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                             check_context=True,
                             regex=True)
        speed = float(self.re_match.group(1))
        if speed > 30:
            raise NotAchievedException("Hit too hard")
        self.wait_disarmed()

    def set_rc_default(self):
        super(AutoTestHeli, self).set_rc_default()
        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

    def tests(self):
        '''return list of all tests'''
        ret = AutoTest.tests(self)
        ret.extend([
            ("AVCMission", "Fly AVC mission", self.fly_avc_test),

            ("RotorRunUp",
             "Test rotor runup",
             self.rotor_runup_complete_checks),

            ("PosHoldTakeOff",
             "Fly POSHOLD takeoff",
             self.fly_heli_poshold_takeoff),

            ("StabilizeTakeOff",
             "Fly stabilize takeoff",
             self.fly_heli_stabilize_takeoff),

            ("SplineWaypoint",
             "Fly Spline Waypoints",
             self.fly_spline_waypoint),

            ("AutoRotation",
             "Fly AutoRotation",
             self.fly_autorotation),

            ("LogUpload",
             "Log upload",
             self.log_upload),
        ])
        return ret

    def disabled_tests(self):
        return {
            "SplineWaypoint": "See https://github.com/ArduPilot/ardupilot/issues/14593",
        }


class AutoTestCopterTests1(AutoTestCopter):
    def tests(self):
        return self.tests1()


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


class AutoTestCopterTests2(AutoTestCopter):
    def tests(self):
        return self.tests2()


class AutoTestCopterTests2a(AutoTestCopter):
    def tests(self):
        return self.tests2a()


class AutoTestCopterTests2b(AutoTestCopter):
    def tests(self):
        return self.tests2b()


class AutoTestCAN(AutoTestCopter):

    def tests(self):
        return self.testcan()
