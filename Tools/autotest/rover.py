#!/usr/bin/env python

# Drive Rover in SITL
from __future__ import print_function

import copy
import os
import shutil
import sys
import time

from common import AutoTest
from pysim import util
from pysim import vehicleinfo

from common import AutoTestTimeoutException
from common import MsgRcvTimeoutException
from common import NotAchievedException
from common import PreconditionFailedException

from pymavlink import mavutil

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

SITL_START_LOCATION = mavutil.location(40.071374969556928,
                                       -105.22978898137808,
                                       1583.702759,
                                       246)


class AutoTestRover(AutoTest):
    @staticmethod
    def get_not_armable_mode_list():
        return ["RTL", "SMART_RTL"]

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return ["FOLLOW"]

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return ["GUIDED", "LOITER", "STEERING", "AUTO"]

    @staticmethod
    def get_normal_armable_modes_list():
        return ["ACRO", "HOLD", "MANUAL"]

    def log_name(self):
        return "Rover"

    def test_filepath(self):
         return os.path.realpath(__file__)

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduRover_Tests/" + name + "/"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_frame(self):
        return "rover"

    def is_rover(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_ROLL"))

    ##########################################################
    #   TESTS DRIVE
    ##########################################################
    # Drive a square in manual mode
    def drive_square(self, side=50):
        """Drive a square, Driving N then E ."""

        self.context_push()
        ex = None
        try:
            self.progress("TEST SQUARE")
            self.set_parameter("RC7_OPTION", 7)
            self.set_parameter("RC9_OPTION", 58)

            self.mavproxy.send('switch 5\n')
            self.wait_mode('MANUAL')

            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.clear_wp(9)

            # first aim north
            self.progress("\nTurn right towards north")
            self.reach_heading_manual(10)
            # save bottom left corner of box as home AND waypoint
            self.progress("Save HOME")
            self.save_wp()

            self.progress("Save WP")
            self.save_wp()

            # pitch forward to fly north
            self.progress("\nGoing north %u meters" % side)
            self.reach_distance_manual(side)
            # save top left corner of square as waypoint
            self.progress("Save WP")
            self.save_wp()

            # roll right to fly east
            self.progress("\nGoing east %u meters" % side)
            self.reach_heading_manual(100)
            self.reach_distance_manual(side)
            # save top right corner of square as waypoint
            self.progress("Save WP")
            self.save_wp()

            # pitch back to fly south
            self.progress("\nGoing south %u meters" % side)
            self.reach_heading_manual(190)
            self.reach_distance_manual(side)
            # save bottom right corner of square as waypoint
            self.progress("Save WP")
            self.save_wp()

            # roll left to fly west
            self.progress("\nGoing west %u meters" % side)
            self.reach_heading_manual(280)
            self.reach_distance_manual(side)
            # save bottom left corner of square (should be near home) as waypoint
            self.progress("Save WP")
            self.save_wp()

            self.progress("Checking number of saved waypoints")
            num_wp = self.save_mission_to_file(
                os.path.join(testdir, "ch7_mission.txt"))
            expected = 7 # home + 6 toggled in
            if num_wp != expected:
                raise NotAchievedException("Did not get %u waypoints; got %u" %
                                           (expected, num_wp))

            # TODO: actually drive the mission

            self.clear_wp(9)
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e

        self.disarm_vehicle()
        self.context_pop()

        if ex:
            raise ex

    def drive_left_circuit(self):
        """Drive a left circuit, 50m on a side."""
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')
        self.set_rc(3, 2000)

        self.progress("Driving left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(1, 1000)
            self.wait_heading(270 - (90*i), accuracy=10)
            self.set_rc(1, 1500)
            self.progress("Starting leg %u" % i)
            self.wait_distance(50, accuracy=7)
        self.set_rc(3, 1500)
        self.progress("Circuit complete")

    # def test_throttle_failsafe(self, home, distance_min=10, side=60,
    #                            timeout=300):
    #     """Fly east, Failsafe, return, land."""
    #
    #     self.mavproxy.send('switch 6\n')  # manual mode
    #     self.wait_mode('MANUAL')
    #     self.mavproxy.send("param set FS_ACTION 1\n")
    #
    #     # first aim east
    #     self.progress("turn east")
    #     if not self.reach_heading_manual(135):
    #         return False
    #
    #     # fly east 60 meters
    #     self.progress("# Going forward %u meters" % side)
    #     if not self.reach_distance_manual(side):
    #         return False
    #
    #     # pull throttle low
    #     self.progress("# Enter Failsafe")
    #     self.mavproxy.send('rc 3 900\n')
    #
    #     tstart = self.get_sim_time()
    #     success = False
    #     while self.get_sim_time() < tstart + timeout and not success:
    #         m = self.mav.recv_match(type='VFR_HUD', blocking=True)
    #         pos = self.mav.location()
    #         home_distance = self.get_distance(home, pos)
    #         self.progress("Alt: %u  HomeDistance: %.0f" %
    #                       (m.alt, home_distance))
    #         # check if we've reached home
    #         if home_distance <= distance_min:
    #             self.progress("RTL Complete")
    #             success = True
    #
    #     # reduce throttle
    #     self.mavproxy.send('rc 3 1500\n')
    #     self.mavproxy.expect('APM: Failsafe ended')
    #     self.mavproxy.send('switch 2\n')  # manual mode
    #     self.wait_heartbeat()
    #     self.wait_mode('MANUAL')
    #
    #     if success:
    #         self.progress("Reached failsafe home OK")
    #         return True
    #     else:
    #         self.progress("Failed to reach Home on failsafe RTL - "
    #         "timed out after %u seconds" % timeout)
    #         return False

    def test_sprayer(self):
        """Test sprayer functionality."""
        self.context_push()
        ex = None
        try:
            rc_ch = 5
            pump_ch = 5
            spinner_ch = 6
            pump_ch_min = 1050
            pump_ch_trim = 1520
            pump_ch_max = 1950
            spinner_ch_min = 975
            spinner_ch_trim = 1510
            spinner_ch_max = 1975

            self.set_parameter("SPRAY_ENABLE", 1)

            self.set_parameter("SERVO%u_FUNCTION" % pump_ch, 22)
            self.set_parameter("SERVO%u_MIN" % pump_ch, pump_ch_min)
            self.set_parameter("SERVO%u_TRIM" % pump_ch, pump_ch_trim)
            self.set_parameter("SERVO%u_MAX" % pump_ch, pump_ch_max)

            self.set_parameter("SERVO%u_FUNCTION" % spinner_ch, 23)
            self.set_parameter("SERVO%u_MIN" % spinner_ch, spinner_ch_min)
            self.set_parameter("SERVO%u_TRIM" % spinner_ch, spinner_ch_trim)
            self.set_parameter("SERVO%u_MAX" % spinner_ch, spinner_ch_max)

            self.set_parameter("SIM_SPR_ENABLE", 1)
            self.set_parameter("SIM_SPR_PUMP", pump_ch)
            self.set_parameter("SIM_SPR_SPIN", spinner_ch)

            self.set_parameter("RC%u_OPTION" % rc_ch, 15)
            self.set_parameter("LOG_DISARMED", 1)

            self.reboot_sitl()

            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.progress("test bootup state - it's zero-output!")
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

            self.progress("Testing speed-ramping")
            self.set_rc(3, 1700) # start driving forward

            # this is somewhat empirical...
            self.wait_servo_channel_value(pump_ch, 1695, timeout=60)

            self.progress("Sprayer OK")
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex:
            raise ex

    def drive_max_rcin(self, timeout=30):
        """Test max RC inputs"""
        self.context_push()
        ex = None

        try:
            self.progress("Testing max RC inputs")
            self.change_mode("MANUAL")

            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.set_rc(3, 2000)
            self.set_rc(1, 1000)

            tstart = self.get_sim_time()
            while self.get_sim_time_cached() - tstart < timeout:
                m = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=1)
                if m is not None:
                    self.progress("Current speed: %f" % m.groundspeed)

            # reduce throttle
            self.set_rc(3, 1500)
            self.set_rc(1, 1500)

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e

        self.disarm_vehicle()
        self.context_pop()

        if ex:
            raise ex

    #################################################
    # AUTOTEST ALL
    #################################################
    def drive_mission(self, filename):
        """Drive a mission from a file."""
        self.progress("Driving mission %s" % filename)
        self.load_mission(filename)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mavproxy.send('switch 4\n')  # auto mode
        self.set_rc(3, 1500)
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 4, max_dist=5)
        self.mavproxy.expect("Mission Complete")
        self.disarm_vehicle()
        self.progress("Mission OK")

    def test_gripper_mission(self):
        self.load_mission("rover-gripper-mission.txt")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mavproxy.expect("Gripper Grabbed")
        self.mavproxy.expect("Gripper Released")
        self.mavproxy.expect("Mission Complete")
        self.disarm_vehicle()

    def do_get_banner(self):
        target_sysid = self.sysid_thismav()
        target_compid = 1
        self.mav.mav.command_long_send(
            target_sysid,
            target_compid,
            mavutil.mavlink.MAV_CMD_DO_SEND_BANNER,
            1, # confirmation
            1, # send it
            0,
            0,
            0,
            0,
            0,
            0)
        start = time.time()
        while True:
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is not None and "ArduRover" in m.text:
                self.progress("banner received: %s" % m.text)
                return
            if time.time() - start > 10:
                break

        raise MsgRcvTimeoutException("banner not received")

    def drive_brake_get_stopping_distance(self, speed):
        # measure our stopping distance:
        old_cruise_speed = self.get_parameter('CRUISE_SPEED')
        old_accel_max = self.get_parameter('ATC_ACCEL_MAX')

        # controller tends not to meet cruise speed (max of ~14 when 15
        # set), thus *1.2
        self.set_parameter('CRUISE_SPEED', speed*1.2)
        # at time of writing, the vehicle is only capable of 10m/s/s accel
        self.set_parameter('ATC_ACCEL_MAX', 15)
        self.change_mode("STEERING")
        self.set_rc(3, 2000)
        self.wait_groundspeed(15, 100)
        initial = self.mav.location()
        initial_time = time.time()
        while time.time() - initial_time < 2:
            # wait for a position update from the autopilot
            start = self.mav.location()
            if start != initial:
                break
        self.set_rc(3, 1500)
        self.wait_groundspeed(0, 0.2)  # why do we not stop?!
        initial = self.mav.location()
        initial_time = time.time()
        while time.time() - initial_time < 2:
            # wait for a position update from the autopilot
            stop = self.mav.location()
            if stop != initial:
                break
        delta = self.get_distance(start, stop)

        self.set_parameter('CRUISE_SPEED', old_cruise_speed)
        self.set_parameter('ATC_ACCEL_MAX', old_accel_max)

        return delta

    def drive_brake(self):
        old_using_brake = self.get_parameter('ATC_BRAKE')
        old_cruise_speed = self.get_parameter('CRUISE_SPEED')

        self.set_parameter('CRUISE_SPEED', 15)
        self.set_parameter('ATC_BRAKE', 0)

        self.arm_vehicle()

        distance_without_brakes = self.drive_brake_get_stopping_distance(15)

        # brakes on:
        self.set_parameter('ATC_BRAKE', 1)
        distance_with_brakes = self.drive_brake_get_stopping_distance(15)
        # revert state:
        self.set_parameter('ATC_BRAKE', old_using_brake)
        self.set_parameter('CRUISE_SPEED', old_cruise_speed)

        delta = distance_without_brakes - distance_with_brakes
        if delta < distance_without_brakes * 0.05:  # 5% isn't asking for much
            self.disarm_vehicle()
            raise NotAchievedException("""
Brakes have negligible effect (with=%0.2fm without=%0.2fm delta=%0.2fm)
""" %
                                       (distance_with_brakes,
                                        distance_without_brakes,
                                        delta))

        self.disarm_vehicle()

        self.progress(
            "Brakes work (with=%0.2fm without=%0.2fm delta=%0.2fm)" %
            (distance_with_brakes, distance_without_brakes, delta))

    def drive_rtl_mission_max_distance_from_home(self):
        '''maximum distance allowed from home at end'''
        return 6.5

    def drive_rtl_mission(self, timeout=120):
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.load_mission("rtl.txt")
        self.change_mode("AUTO")

        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("Didn't see wp 3")
            m = self.mav.recv_match(type='MISSION_CURRENT',
                                    blocking=True,
                                    timeout=1)
            self.progress("MISSION_CURRENT: %s" % str(m))
            if m.seq == 3:
                break

        self.drain_mav();

        m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
                                blocking=True,
                                timeout=1)
        if m is None:
            raise MsgRcvTimeoutException(
                "Did not receive NAV_CONTROLLER_OUTPUT message")

        wp_dist_min = 5
        if m.wp_dist < wp_dist_min:
            raise PreconditionFailedException(
                "Did not start at least %f metres from destination (is=%f)" %
                (wp_dist_min, m.wp_dist))

        self.progress("NAV_CONTROLLER_OUTPUT.wp_dist looks good (%u >= %u)" %
                      (m.wp_dist, wp_dist_min,))

        # wait for mission to complete
        self.mavproxy.expect("Mission Complete")

        # the EKF doesn't pull us down to 0 speed:
        self.wait_groundspeed(0, 0.5, timeout=600)

        # current Rover blows straight past the home position and ends
        # up ~6m past the home point.
        home_distance = self.distance_to_home()
        home_distance_min = 5.5
        home_distance_max = self.drive_rtl_mission_max_distance_from_home()
        if home_distance > home_distance_max:
            raise NotAchievedException(
                "Did not stop near home (%f metres distant (%f > want > %f))" %
                (home_distance, home_distance_min, home_distance_max))
        self.disarm_vehicle()
        self.progress("RTL Mission OK (%fm)" % home_distance)


    def drive_fence_ac_avoidance(self):
        self.context_push()
        ex = None
        try:
            self.load_fence("rover-fence-ac-avoid.txt")
            self.set_parameter("FENCE_ENABLE", 0)
            self.set_parameter("PRX_TYPE", 10)
            self.set_parameter("RC10_OPTION", 40) # proximity-enable
            self.reboot_sitl()
            # start = self.mav.location()
            self.wait_ready_to_arm()
            self.arm_vehicle()
            # first make sure we can breach the fence:
            self.set_rc(10, 1000)
            self.change_mode("ACRO")
            self.set_rc(3, 1550)
            self.wait_distance_to_home(25, 100000, timeout=60)
            self.change_mode("RTL")
            self.mavproxy.expect("APM: Reached destination")
            # now enable avoidance and make sure we can't:
            self.set_rc(10, 2000)
            self.change_mode("ACRO")
            self.wait_groundspeed(0, 0.7, timeout=60)
            # watch for speed zero
            self.wait_groundspeed(0, 0.2, timeout=120)

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.mavproxy.send("fence clear\n")
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex:
            raise ex

    def test_servorelayevents(self):
        self.do_set_relay(0, 0)
        off = self.get_parameter("SIM_PIN_MASK")
        self.do_set_relay(0, 1)
        on = self.get_parameter("SIM_PIN_MASK")
        if on == off:
            raise NotAchievedException(
                "Pin mask unchanged after relay cmd")
        self.progress("Pin mask changed after relay command")

    def test_setting_modes_via_mavproxy_switch(self):
        fnoo = [(1, 'MANUAL'),
                (2, 'MANUAL'),
                (3, 'RTL'),
                # (4, 'AUTO'),  # no mission, can't set auto
                (5, 'RTL'),  # non-existant mode, should stay in RTL
                (6, 'MANUAL')]
        for (num, expected) in fnoo:
            self.mavproxy.send('switch %u\n' % num)
            self.wait_mode(expected)

    def test_setting_modes_via_mavproxy_mode_command(self):
        fnoo = [(1, 'ACRO'),
                (3, 'STEERING'),
                (4, 'HOLD'),
                ]
        for (num, expected) in fnoo:
            self.mavproxy.send('mode manual\n')
            self.wait_mode("MANUAL")
            self.mavproxy.send('mode %u\n' % num)
            self.wait_mode(expected)
            self.mavproxy.send('mode manual\n')
            self.wait_mode("MANUAL")
            self.mavproxy.send('mode %s\n' % expected)
            self.wait_mode(expected)

    def test_setting_modes_via_modeswitch(self):
        # test setting of modes through mode switch
        self.context_push()
        ex = None
        try:
            self.set_parameter("MODE_CH", 8)
            self.set_rc(8, 1000)
            # mavutil.mavlink.ROVER_MODE_HOLD:
            self.set_parameter("MODE6", 4)
            # mavutil.mavlink.ROVER_MODE_ACRO
            self.set_parameter("MODE5", 1)
            self.set_rc(8, 1800) # PWM for mode6
            self.wait_mode("HOLD")
            self.set_rc(8, 1700) # PWM for mode5
            self.wait_mode("ACRO")
            self.set_rc(8, 1800) # PWM for mode6
            self.wait_mode("HOLD")
            self.set_rc(8, 1700) # PWM for mode5
            self.wait_mode("ACRO")
        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def test_setting_modes_via_auxswitches(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("MODE5", 1)
            self.mavproxy.send('switch 1\n')  # random mode
            self.wait_heartbeat()
            self.change_mode('MANUAL')
            self.mavproxy.send('switch 5\n')  # acro mode
            self.wait_mode("ACRO")
            self.set_rc(9, 1000)
            self.set_rc(10, 1000)
            self.set_parameter("RC9_OPTION", 53) # steering
            self.set_parameter("RC10_OPTION", 54) # hold
            self.set_rc(9, 1900)
            self.wait_mode("STEERING")
            self.set_rc(10, 1900)
            self.wait_mode("HOLD")

            # reset both switches - should go back to ACRO
            self.set_rc(9, 1000)
            self.set_rc(10, 1000)
            self.wait_mode("ACRO")

            self.set_rc(9, 1900)
            self.wait_mode("STEERING")
            self.set_rc(10, 1900)
            self.wait_mode("HOLD")

            self.set_rc(10, 1000) # this re-polls the mode switch
            self.wait_mode("ACRO")
            self.set_rc(9, 1000)
        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def test_rc_override_cancel(self):
        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.zero_throttle()
        self.arm_vehicle()
        # start moving forward a little:
        normal_rc_throttle = 1700
        throttle_override = 1900

        self.progress("Establishing baseline RC input")
        self.set_rc(3, normal_rc_throttle)
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not get rc change")
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            if m.chan3_raw == normal_rc_throttle:
                break

        self.progress("Set override with RC_CHANNELS_OVERRIDE")
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not override")
            self.progress("Sending throttle of %u" % (throttle_override,))
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                throttle_override, # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw

            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            self.progress("chan3=%f want=%f" % (m.chan3_raw, throttle_override))
            if m.chan3_raw == throttle_override:
                break

        self.progress("disabling override and making sure we revert to RC input in good time")
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 0.5:
                raise AutoTestTimeoutException("Did not cancel override")
            self.progress("Sending cancel of throttle override")
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                0,     # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw
            self.do_timesync_roundtrip()
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            self.progress("chan3=%f want=%f" % (m.chan3_raw, normal_rc_throttle))
            if m.chan3_raw == normal_rc_throttle:
                break
        self.disarm_vehicle()

    def test_rc_overrides(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("RC12_OPTION", 46)
            self.reboot_sitl()

            self.mavproxy.send('switch 6\n')  # Manual mode
            self.wait_mode('MANUAL')
            self.wait_ready_to_arm()
            self.set_rc(3, 1500)  # throttle at zero
            self.arm_vehicle()
            # start moving forward a little:
            normal_rc_throttle = 1700
            self.set_rc(3, normal_rc_throttle)
            self.wait_groundspeed(5, 100)

            # allow overrides:
            self.set_rc(12, 2000)

            # now override to stop:
            throttle_override = 1500

            tstart = self.get_sim_time_cached()
            while True:
                if self.get_sim_time_cached() - tstart > 10:
                    raise AutoTestTimeoutException("Did not reach speed")
                self.progress("Sending throttle of %u" % (throttle_override,))
                self.mav.mav.rc_channels_override_send(
                    1, # target system
                    1, # targe component
                    65535, # chan1_raw
                    65535, # chan2_raw
                    throttle_override, # chan3_raw
                    65535, # chan4_raw
                    65535, # chan5_raw
                    65535, # chan6_raw
                    65535, # chan7_raw
                    65535) # chan8_raw

                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                want_speed = 2.0
                self.progress("Speed=%f want=<%f" % (m.groundspeed, want_speed))
                if m.groundspeed < want_speed:
                    break

            # now override to stop - but set the switch on the RC
            # transmitter to deny overrides; this should send the
            # speed back up to 5 metres/second:
            self.set_rc(12, 1000)

            throttle_override = 1500
            tstart = self.get_sim_time_cached()
            while True:
                if self.get_sim_time_cached() - tstart > 10:
                    raise AutoTestTimeoutException("Did not speed back up")
                self.progress("Sending throttle of %u" % (throttle_override,))
                self.mav.mav.rc_channels_override_send(
                    1, # target system
                    1, # targe component
                    65535, # chan1_raw
                    65535, # chan2_raw
                    throttle_override, # chan3_raw
                    65535, # chan4_raw
                    65535, # chan5_raw
                    65535, # chan6_raw
                    65535, # chan7_raw
                    65535) # chan8_raw

                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                want_speed = 5.0
                self.progress("Speed=%f want=>%f" % (m.groundspeed, want_speed))

                if m.groundspeed > want_speed:
                    break

            # re-enable RC overrides
            self.set_rc(12, 2000)

            # check we revert to normal RC inputs when gcs overrides cease:
            self.progress("Waiting for RC to revert to normal RC input")
            self.wait_rc_channel_value(3, normal_rc_throttle, timeout=10)

            self.start_subtest("Check override time of zero disables overrides")
            old = self.get_parameter("RC_OVERRIDE_TIME")
            ch = 2
            self.set_rc(ch, 1000)
            channels = [65535] * 18
            ch_override_value = 1700
            channels[ch-1] = ch_override_value
            channels[7] = 1234 # that's channel 8!
            self.progress("Sending override message %u" % ch_override_value)
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                *channels
            )
            # long timeout required here as we may have sent a lot of
            # things via MAVProxy...
            self.wait_rc_channel_value(ch, ch_override_value, timeout=30)
            self.set_parameter("RC_OVERRIDE_TIME", 0)
            self.wait_rc_channel_value(ch, 1000)
            self.set_parameter("RC_OVERRIDE_TIME", old)
            self.wait_rc_channel_value(ch, ch_override_value)

            ch_override_value = 1720
            channels[ch-1] = ch_override_value
            self.progress("Sending override message %u" % ch_override_value)
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                *channels
            )
            self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
            self.set_parameter("RC_OVERRIDE_TIME", 0)
            self.wait_rc_channel_value(ch, 1000)
            self.set_parameter("RC_OVERRIDE_TIME", old)

            self.progress("Ensuring timeout works")
            self.wait_rc_channel_value(ch, 1000, timeout=5)
            self.delay_sim_time(10)

            self.set_parameter("RC_OVERRIDE_TIME", 10)
            self.progress("Sending override message")

            ch_override_value = 1730
            channels[ch-1] = ch_override_value
            self.progress("Sending override message %u" % ch_override_value)
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                *channels
            )
            self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
            tstart = self.get_sim_time()
            self.progress("Waiting for channel to revert to 1000 in ~10s")
            self.wait_rc_channel_value(ch, 1000, timeout=15)
            delta = self.get_sim_time() - tstart
            if delta > 12:
                raise NotAchievedException("Took too long to revert RC channel value (delta=%f)" % delta)
            min_delta = 9
            if delta < min_delta:
                raise NotAchievedException("Didn't take long enough to revert RC channel value (delta=%f want>=%f)" %
                                           (delta, min_delta))
            self.progress("Disabling RC override timeout")
            self.set_parameter("RC_OVERRIDE_TIME", -1)
            ch_override_value = 1740
            channels[ch-1] = ch_override_value
            self.progress("Sending override message %u" % ch_override_value)
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                *channels
            )
            self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
            tstart = self.get_sim_time()
            while True:
                # warning: this is get_sim_time() and can slurp messages on you!
                delta = self.get_sim_time() - tstart
                if delta > 20:
                    break
                m = self.mav.recv_match(type='RC_CHANNELS',
                                        blocking=True,
                                        timeout=1)
                if m is None:
                    raise NotAchievedException("Did not get RC_CHANNELS")
                channel_field = "chan%u_raw" % ch
                m_value = getattr(m, channel_field)
                if m_value != ch_override_value:
                    raise NotAchievedException("Value reverted after %f seconds when it should not have (got=%u) (want=%u)" % (delta, m_value, ch_override_value))
            self.set_parameter("RC_OVERRIDE_TIME", old)

        except Exception as e:
            self.progress("Exception caught: %s" %
                          self.get_exception_stacktrace(e))
            ex = e

        self.context_pop()
        self.disarm_vehicle()
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def test_manual_control(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("RC12_OPTION", 46) # enable/disable rc overrides
            self.reboot_sitl()

            self.change_mode("MANUAL")
            self.wait_ready_to_arm()
            self.zero_throttle()
            self.arm_vehicle()
            self.progress("start moving forward a little")
            normal_rc_throttle = 1700
            self.set_rc(3, normal_rc_throttle)
            self.wait_groundspeed(5, 100)

            self.progress("allow overrides")
            self.set_rc(12, 2000)

            self.progress("now override to stop")
            throttle_override_normalized = 0
            expected_throttle = 0 # in VFR_HUD

            tstart = self.get_sim_time_cached()
            while True:
                if self.get_sim_time_cached() - tstart > 10:
                    raise AutoTestTimeoutException("Did not reach speed")
                self.progress("Sending normalized throttle of %d" % (throttle_override_normalized,))
                self.mav.mav.manual_control_send(
                    1, # target system
                    32767, # x (pitch)
                    32767, # y (roll)
                    throttle_override_normalized, # z (thrust)
                    32767, # r (yaw)
                    0) # button mask

                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                want_speed = 2.0
                self.progress("Speed=%f want=<%f  throttle=%u want=%u" %
                              (m.groundspeed, want_speed, m.throttle, expected_throttle))
                if m.groundspeed < want_speed and m.throttle == expected_throttle:
                    break

            self.progress("now override to stop - but set the switch on the RC transmitter to deny overrides; this should send the speed back up to 5 metres/second")
            self.set_rc(12, 1000)

            throttle_override_normalized = 500
            expected_throttle = 36 # in VFR_HUD, corresponding to normal_rc_throttle adjusted for channel min/max

            tstart = self.get_sim_time_cached()
            while True:
                if self.get_sim_time_cached() - tstart > 10:
                    raise AutoTestTimeoutException("Did not stop")
                self.progress("Sending normalized throttle of %u" % (throttle_override_normalized,))
                self.mav.mav.manual_control_send(
                    1, # target system
                    32767, # x (pitch)
                    32767, # y (roll)
                    throttle_override_normalized, # z (thrust)
                    32767, # r (yaw)
                    0) # button mask

                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                want_speed = 5.0

                self.progress("Speed=%f want=>%f  throttle=%u want=%u" %
                              (m.groundspeed, want_speed, m.throttle, expected_throttle))
                if m.groundspeed > want_speed and m.throttle == expected_throttle:
                    break

            # re-enable RC overrides
            self.set_rc(12, 2000)

            # check we revert to normal RC inputs when gcs overrides cease:
            self.progress("Waiting for RC to revert to normal RC input")
            self.wait_rc_channel_value(3, normal_rc_throttle, timeout=10)

        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.context_pop()
        self.disarm_vehicle()
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def test_camera_mission_items(self):
        self.context_push()
        ex = None
        try:
            self.load_mission("rover-camera-mission.txt")
            self.wait_ready_to_arm()
            self.change_mode("AUTO")
            self.wait_ready_to_arm()
            self.arm_vehicle()
            prev_cf = None
            while True:
                cf = self.mav.recv_match(type='CAMERA_FEEDBACK', blocking=True)
                if prev_cf is None:
                    prev_cf = cf
                    continue
                dist_travelled = self.get_distance_int(prev_cf, cf)
                prev_cf = cf
                mc = self.mav.messages.get("MISSION_CURRENT", None)
                if mc is None:
                    continue
                elif mc.seq == 2:
                    expected_distance = 2
                elif mc.seq == 4:
                    expected_distance = 5
                elif mc.seq == 5:
                    break
                else:
                    continue
                self.progress("Expected distance %f got %f" %
                              (expected_distance, dist_travelled))
                error = abs(expected_distance - dist_travelled)
                # Rover moves at ~5m/s; we appear to do something at
                # 5Hz, so we do see over a meter of error!
                max_error = 1.5
                if error > max_error:
                    raise NotAchievedException("Camera distance error: %f (%f)" %
                                               (error, max_error))

            self.disarm_vehicle()
        except Exception as e:
            self.progress("Exception caught")
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def test_do_set_mode_via_command_long(self):
        self.do_set_mode_via_command_long("HOLD")
        self.do_set_mode_via_command_long("MANUAL")

    def test_mavproxy_do_set_mode_via_command_long(self):
        self.mavproxy_do_set_mode_via_command_long("HOLD")
        self.mavproxy_do_set_mode_via_command_long("MANUAL")

    def test_sysid_enforce(self):
        '''Run the same arming code with correct then incorrect SYSID'''
        self.context_push()
        ex = None
        try:
            # if set_parameter is ever changed to not use MAVProxy
            # this test is going to break horribly.  Sorry.
            self.set_parameter("SYSID_MYGCS", 255) # assume MAVProxy does this!
            self.set_parameter("SYSID_ENFORCE", 1) # assume MAVProxy does this!

            self.change_mode('MANUAL')

            self.progress("make sure I can arm ATM")
            self.wait_ready_to_arm()
            self.arm_vehicle(timeout=5)
            self.disarm_vehicle()

            # temporarily set a different system ID than MAVProxy:
            self.progress("Attempting to arm vehicle myself")
            old_srcSystem = self.mav.mav.srcSystem
            try:
                self.mav.mav.srcSystem = 243
                self.arm_vehicle(timeout=5)
                self.disarm_vehicle()
                success = False
            except AutoTestTimeoutException as e:
                success = True
            self.mav.mav.srcSystem = old_srcSystem
            if not success:
                raise NotAchievedException(
                    "Managed to arm with SYSID_ENFORCE set")

            self.progress("Attempting to arm vehicle from vehicle component")
            old_srcSystem = self.mav.mav.srcSystem
            comp_arm_exception = None
            try:
                self.mav.mav.srcSystem = 1
                self.arm_vehicle(timeout=5)
                self.disarm_vehicle()
            except Exception as e:
                comp_arm_exception = e
            self.mav.mav.srcSystem = old_srcSystem
            if comp_arm_exception is not None:
                raise comp_arm_exception

        except Exception as e:
            self.progress("Exception caught")
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def test_rally_points(self):
        self.reboot_sitl() # to ensure starting point is as expected

        self.load_rally("rover-test-rally.txt")
        accuracy = self.get_parameter("WP_RADIUS")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.reach_heading_manual(10)
        self.reach_distance_manual(50)

        self.change_mode("RTL")
        # location copied in from rover-test-rally.txt:
        loc = mavutil.location(40.071553,
                               -105.229401,
                               0,
                               0)
        self.wait_location(loc, accuracy=accuracy)
        self.disarm_vehicle()

    def string_for_frame(self, frame):
        return mavutil.mavlink.enums["MAV_FRAME"][frame].name

    def frames_equivalent(self, f1, f2):
        pairs = [
            (mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT),
            (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
            (mavutil.mavlink.MAV_FRAME_GLOBAL,
             mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        ]
        for pair in pairs:
            if (f1 == pair[0] and f2 == pair[1]):
                return True
            if (f1 == pair[1] and f2 == pair[0]):
                return True
        return f1 == f2;

    def check_mission_items_same(self, check_atts, want, got, epsilon=None,skip_first_item=False):
        self.progress("Checking mission items same")
        if epsilon is None:
            epsilon = 1
        if len(want) != len(got):
            raise NotAchievedException("Incorrect item count (want=%u got=%u)" % (len(want), len(got)))
        self.progress("Checking %u items" % len(want))
        for i in range(0, len(want)):
            if skip_first_item and i == 0:
                continue
            item = want[i]
            downloaded_item = got[i]

            check_atts = ['mission_type', 'command', 'x', 'y', 'seq', 'param1']
            # z is not preserved

            self.progress("Comparing (%s) and (%s)" % (str(item), str(downloaded_item)))

            for att in check_atts:
                item_val = getattr(item, att)
                downloaded_item_val = getattr(downloaded_item, att)
                if abs(item_val - downloaded_item_val) > epsilon:
                    raise NotAchievedException(
                        "Item %u (%s) has different %s after download want=%s got=%s (got-item=%s)" %
                        (i, str(item), att, str(item_val), str(downloaded_item_val), str(downloaded_item)))
                # for waypoint items ensure z and frame are preserved:
            self.progress("Type is %u" % got[0].mission_type)
            if got[0].mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                item_val = getattr(item, 'frame')
                downloaded_item_val = getattr(downloaded_item, 'frame')
                if not self.frames_equivalent(item_val, downloaded_item_val):
                    raise NotAchievedException("Frame not same (got=%s want=%s)" %
                                               (self.string_for_frame(downloaded_item_val),
                                                self.string_for_frame(item_val)))
                if abs(item.z - downloaded_item.z) > 0.00001:
                    raise NotAchievedException("Z not preserved (got=%f want=%f)" %
                                               (item.z, downloaded_item.z))

    def check_fence_items_same(self, want, got):
        check_atts = ['mission_type', 'command', 'x', 'y', 'seq', 'param1']
        return self.check_mission_items_same(check_atts, want, got)

    def check_mission_waypoint_items_same(self, want, got):
        check_atts = ['mission_type', 'command', 'x', 'y', 'z', 'seq', 'param1']
        return self.check_mission_items_same(check_atts, want, got, skip_first_item=True)

    def check_mission_item_upload_download(self, items, itype, mission_type):
        self.progress("check %s _upload/download: upload %u items" %
                      (itype, len(items),))
        self.upload_using_mission_protocol(mission_type, items)
        self.progress("check %s upload/download: download items" % itype)
        downloaded_items = self.download_using_mission_protocol(mission_type)
        self.progress("Downloaded items: (%s)" % str(downloaded_items))
        if len(items) != len(downloaded_items):
            raise NotAchievedException("Did not download same number of items as uploaded want=%u got=%u" % (len(items), len(downloaded_items)))
        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            self.check_fence_items_same(items, downloaded_items)
        elif mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            self.check_mission_waypoint_items_same(items, downloaded_items)
        else:
            raise NotAchievedException("Unhandled")

    def check_fence_upload_download(self, items):
        self.check_mission_item_upload_download(
            items,
            "fence",
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

    def check_mission_upload_download(self, items):
        self.check_mission_item_upload_download(
            items,
            "waypoints",
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def fence_with_bad_frame(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 *1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_zero_vertex_count(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 *1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_wrong_vertex_count(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                2, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 *1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_multiple_return_points(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 *1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 *1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_invalid_latlon(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(100 * 1e7), # bad latitude. bad.
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_multiple_return_points_with_bad_sequence_numbers(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 * 1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(2.0 * 1e7), # latitude
                int(2.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_which_exceeds_storage_space(self, target_system=1, target_component=1):
        ret = []
        for i in range(0, 60):
            ret.append(self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                i, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                10, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 * 1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            )
        return ret

    def fences_which_should_not_upload(self, target_system=1, target_component=1):
        return [ ("Bad Frame", self.fence_with_bad_frame(target_system=target_system, target_component=target_component)),
                 ("Zero Vertex Count", self.fence_with_zero_vertex_count(target_system=target_system, target_component=target_component)),
                 ("Wrong Vertex Count", self.fence_with_wrong_vertex_count(target_system=target_system, target_component=target_component)),
                 ("Multiple return points", self.fence_with_multiple_return_points(target_system=target_system, target_component=target_component)),
                 ("Invalid lat/lon", self.fence_with_invalid_latlon(target_system=target_system, target_component=target_component)),
                 ("Multiple Return points with bad sequence numbers", self.fence_with_multiple_return_points_with_bad_sequence_numbers(target_system=target_system, target_component=target_component)),
                 ("Fence which exceeds storage space", self.fence_which_exceeds_storage_space(target_system=target_system, target_component=target_component)),
                 ]


    def fence_with_single_return_point(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 *1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]
    def fence_with_single_return_point_and_5_vertex_inclusion(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 *1e7), # latitude
                int(1.0017 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 *1e7), # latitude
                int(1.0001 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                4, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 *1e7), # latitude
                int(1.0002 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                5, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 *1e7), # latitude
                int(1.0003 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_many_exclusion_circles(self, count=50, target_system=1, target_component=1):
        ret = []
        for i in range(0, count):
            lat_deg = 1.0003 + count/10
            lng_deg = 1.0002 + count/10
            item = self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                i, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                count, # p1
                0, # p2
                0, # p3
                0, # p4
                int(lat_deg *1e7), # latitude
                int(lng_deg *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
            ret.append(item)
        return ret

    def fence_with_many_exclusion_polyfences(self, target_system=1, target_component=1):
        ret = []
        seq = 0
        for fencenum in range(0,4):
            pointcount = fencenum + 6
            for p in range(0, pointcount):
                lat_deg = 1.0003 + p/10 + fencenum/100
                lng_deg = 1.0002 + p/10 + fencenum/100
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    0, # current
                    0, # autocontinue
                    pointcount, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(lat_deg *1e7), # latitude
                    int(lng_deg *1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                ret.append(item)
                seq += 1
        return ret

    def fences_which_should_upload(self, target_system=1, target_component=1):
        return [
            ("Single Return Point", self.fence_with_single_return_point(target_system=target_system, target_component=target_component)),
            ( "Return and 5-vertex-inclusion", self.fence_with_single_return_point_and_5_vertex_inclusion(target_system=target_system, target_component=target_component) ),
            ( "Many exclusion circles", self.fence_with_many_exclusion_circles(target_system=target_system, target_component=target_component) ),
            ( "Many exclusion polyfences", self.fence_with_many_exclusion_polyfences(target_system=target_system, target_component=target_component) ),
            ( "Empty fence", [] ),
            ]


    def assert_fence_does_not_upload(self, fence, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        # upload single item using mission item protocol:
        upload_failed = False
        try:
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                               fence)
        except NotAchievedException:
            # TODO: make sure we failed for correct reason
            upload_failed = True
        if not upload_failed:
            raise NotAchievedException("Uploaded fence when should not be possible")
        self.progress("Fence rightfully bounced")

    def fencepoint_protocol_epsilon(self):
        return 0.00002

    def roundtrip_fencepoint_protocol(self, offset, count, lat, lng, target_system=1, target_component=1):
        self.progress("Sending FENCE_POINT offs=%u count=%u" % (offset, count))
        self.mav.mav.fence_point_send(target_system,
                                      target_component,
                                      offset,
                                      count,
                                      lat,
                                      lng)

        self.progress("Requesting fence point")
        m = self.get_fence_point(offset, target_system=target_system, target_component=target_component)
        if abs(m.lat - lat) > self.fencepoint_protocol_epsilon():
            raise NotAchievedException("Did not get correct lat in fencepoint: got=%f want=%f" % (m.lat, lat))
        if abs(m.lng - lng) > self.fencepoint_protocol_epsilon():
            raise NotAchievedException("Did not get correct lng in fencepoint: got=%f want=%f" % (m.lng, lng))
        self.progress("Roundtrip OK")

    def roundtrip_fence_using_fencepoint_protocol(self, loc_list, target_system=1, target_component=1, ordering=None):
        count = len(loc_list)
        offset = 0
        self.set_parameter("FENCE_TOTAL", count)
        if ordering is None:
            ordering = range(count)
        elif len(ordering) != len(loc_list):
            raise ValueError("ordering list length mismatch")

        for offset in ordering:
            loc = loc_list[offset]
            self.roundtrip_fencepoint_protocol(offset,
                                               count,
                                               loc.lat,
                                               loc.lng,
                                               target_system,
                                               target_component)

        self.progress("Validating uploaded fence")
        returned_count = self.get_parameter("FENCE_TOTAL")
        if returned_count != count:
            raise NotAchievedException("Returned count mismatch (want=%u got=%u)" %
                                       (count, returned_count))
        for i in range(count):
            self.progress("Requesting fence point")
            m = self.get_fence_point(offset, target_system=target_system, target_component=target_component)
            if abs(m.lat-loc.lat) > self.fencepoint_protocol_epsilon():
                raise NotAchievedException("Returned lat mismatch (want=%f got=%f" %
                                           (loc.lat, m.lat))
            if abs(m.lng-loc.lng) > self.fencepoint_protocol_epsilon():
                raise NotAchievedException("Returned lng mismatch (want=%f got=%f" %
                                           (loc.lng, m.lng))
            if m.count != count:
                raise NotAchievedException("Count mismatch (want=%u got=%u)" %
                                           (count, m.count))

    def send_fencepoint_expect_statustext(self, offset, count, lat, lng, statustext_fragment, target_system=1, target_component=1, timeout=10):
        self.mav.mav.fence_point_send(target_system,
                                      target_component,
                                      offset,
                                      count,
                                      lat,
                                      lng)

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get error message back")
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            self.progress("statustext: %s (want='%s')" %
                          (str(m), statustext_fragment))
            if m is None:
                continue
            if statustext_fragment in m.text:
                break

    def get_fence_point(self, idx, target_system=1, target_component=1):
        self.mav.mav.fence_fetch_point_send(target_system,
                                            target_component,
                                            idx)
        m = self.mav.recv_match(type="FENCE_POINT", blocking=True, timeout=2)
        print("m: %s" % str(m))
        if m is None:
            raise NotAchievedException("Did not get fence return point back")
        if m.idx != idx:
            raise NotAchievedException("Invalid idx returned (want=%u got=%u)" %
                                       (idx, m.seq))
        return m

    def test_gcs_fence_centroid(self, target_system=1, target_component=1):
        self.start_subtest("Ensuring if we don't have a centroid it gets calculated")
        items = self.test_gcs_fence_need_centroid(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        centroid = self.get_fence_point(0)
        want_lat = 1.0001
        want_lng = 1.00005
        if abs(centroid.lat - want_lat) > 0.000001:
            raise NotAchievedException("Centroid lat not as expected (want=%f got=%f)" % (want_lat, centroid.lat))
        if abs(centroid.lng - want_lng) > 0.000001:
            raise NotAchievedException("Centroid lng not as expected (want=%f got=%f)" % (want_lng, centroid.lng))


    def test_gcs_fence_update_fencepoint(self, target_system=1, target_component=1):
        self.start_subtest("Ensuring we can move a fencepoint")
        items = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        item_seq = 2
        item = items[item_seq]
        print("item is (%s)" % str(item))
        self.progress("original x=%d" % item.x)
        item.x += int(0.1 * 1e7)
        self.progress("new x=%d" % item.x)
        self.progress("try to overwrite item %u" % item_seq)
        self.mav.mav.mission_write_partial_list_send(
            target_system,
            target_component,
            item_seq,
            item_seq,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, item_seq)
        item.pack(self.mav.mav)
        self.mav.mav.send(item)
        self.progress("Answered request for fence point %u" % item_seq)

        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        downloaded_items2 = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if downloaded_items2[item_seq].x != item.x:
            raise NotAchievedException("Item did not update")
        self.check_fence_items_same([items[0], items[1], item, items[3]], downloaded_items2)

    def test_gcs_fence_boring_triangle(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 *1e7), # latitude
                int(1.0001 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.00015 *1e7), # latitude
                int(1.00015 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ])

    def test_gcs_fence_need_centroid(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 *1e7), # latitude
                int(1.0001 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 *1e7), # latitude
                int(1.0001 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ])

    def click_location_from_item(self, item):
        self.mavproxy.send("click %f %f\n" % (item.x*1e-7, item.y*1e-7))

    def test_gcs_fence_via_mavproxy(self, target_system=1, target_component=1):
        self.start_subtest("Fence via MAVProxy")
        if not self.mavproxy_can_do_mision_item_protocols():
            return
        self.start_subsubtest("fence addcircle")
        self.mavproxy.send("fence clear\n")
        self.delay_sim_time(1)
        radius = 20
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            0, # current
            0, # autocontinue
            radius, # p1
            0, # p2
            0, # p3
            0, # p4
            int(1.0017 *1e7), # latitude
            int(1.0017 *1e7), # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("item is (%s)" % str(item))
        self.click_location_from_item(item)
        self.mavproxy.send("fence addcircle inc %u\n" % radius)
        self.delay_sim_time(1)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("downloaded items: %s" % str(downloaded_items))
        self.check_fence_items_same([item], downloaded_items)

        radius_exc = 57.3
        item2 = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            radius_exc, # p1
            0, # p2
            0, # p3
            0, # p4
            int(1.0017 *1e7), # latitude
            int(1.0017 *1e7), # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.click_location_from_item(item2)
        self.mavproxy.send("fence addcircle exc %f\n" % radius_exc)
        self.delay_sim_time(1)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("downloaded items: %s" % str(downloaded_items))
        self.check_fence_items_same([item, item2], downloaded_items)
        self.end_subsubtest("fence addcircle")

        self.start_subsubtest("fence addpoly")
        self.mavproxy.send("fence clear\n")
        self.delay_sim_time(1)
        pointcount = 7
        self.mavproxy.send("fence addpoly inc 20 %u 37.2\n" % pointcount) # radius, pointcount, rotaiton
        self.delay_sim_time(5)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != pointcount:
            raise NotAchievedException("Did not get expected number of points returned (want=%u got=%u)" % (pointcount, len(downloaded_items)))
        self.end_subsubtest("fence addpoly")

        self.start_subsubtest("fence movepolypoint")
        self.mavproxy.send("fence clear\n")
        self.delay_sim_time(1)
        triangle = self.test_gcs_fence_boring_triangle(target_system=target_system,
                                                   target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           triangle)
        self.mavproxy.send("fence list\n")
        self.delay_sim_time(1)
        triangle[2].x += 500
        triangle[2].y += 700
        self.click_location_from_item(triangle[2])
        self.mavproxy.send("fence movepolypoint 0 2\n")
        self.delay_sim_time(10)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.check_fence_items_same(triangle, downloaded_items)
        self.end_subsubtest("fence movepolypoint")

        self.start_subsubtest("fence enable and disable")
        self.mavproxy.send("fence enable\n")
        self.mavproxy.expect("fence enabled")
        self.mavproxy.send("fence disable\n")
        self.mavproxy.expect("fence disabled")
        self.end_subsubtest("fence enable and disable")

#        MANUAL> usage: fence <addcircle|addpoly|changealt|clear|disable|draw|enable|list|load|move|movemulti|movepolypoint|param|remove|save|savecsv|savelocal|show|status|undo|update>

    def test_gcs_fence(self):
        target_system = 1
        target_component = 1

        self.progress("Testing FENCE_POINT protocol")

        self.start_subtest("FENCE_TOTAL manipulation")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE);
        self.assert_parameter_value("FENCE_TOTAL", 0)

        self.set_parameter("FENCE_TOTAL", 5)
        self.assert_parameter_value("FENCE_TOTAL", 5)

        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE);
        self.assert_parameter_value("FENCE_TOTAL", 0)

        self.progress("sending out-of-range fencepoint")
        self.send_fencepoint_expect_statustext(0,
                                               0,
                                               1.2345,
                                               5.4321,
                                               "index past total",
                                               target_system=target_component,
                                               target_component=target_component)

        self.progress("sending another out-of-range fencepoint")
        self.send_fencepoint_expect_statustext(0,
                                               1,
                                               1.2345,
                                               5.4321,
                                               "bad count",
                                               target_system=target_component,
                                               target_component=target_component)

        self.set_parameter("FENCE_TOTAL", 1)
        self.assert_parameter_value("FENCE_TOTAL", 1)

        self.send_fencepoint_expect_statustext(0,
                                               1,
                                               1.2345,
                                               5.4321,
                                               "Invalid FENCE_TOTAL",
                                               target_system=target_component,
                                               target_component=target_component)

        self.set_parameter("FENCE_TOTAL", 5)
        self.progress("Checking default points")
        for i in range(5):
            m = self.get_fence_point(i)
            if m.count != 5:
                raise NotAchievedException("Unexpected count in fence point (want=%u got=%u" %
                                           (5, m.count))
            if m.lat != 0 or m.lng != 0:
                raise NotAchievedException("Unexpected lat/lon in fencepoint")

        self.progress("Storing a return point")
        self.roundtrip_fencepoint_protocol(0, 5, 1.2345, 5.4321, target_system=target_system, target_component=target_component)

        lat = 2.345
        lng = 4.321
        self.roundtrip_fencepoint_protocol(0, 5, lat, lng, target_system=target_system, target_component=target_component)

        if not self.mavproxy_can_do_mision_item_protocols():
            self.progress("MAVProxy too old to do fence point protocols")
            return

        self.progress("Download with new protocol")
        items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(items) != 1:
            raise NotAchievedException("Unexpected fencepoint count (want=%u got=%u)" % (1, len(items)))
        if items[0].command != mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
            raise NotAchievedException("Fence return point not of correct type expected (%u) got %u" % (items[0].command, mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT))
        if items[0].frame != mavutil.mavlink.MAV_FRAME_GLOBAL:
            raise NotAchievedException("Unexpected frame want=%s got=%s," %
                                       (self.string_for_frame(mavutil.mavlink.MAV_FRAME_GLOBAL),
                                        self.string_for_frame(items[0].frame)))
        got_lat = items[0].x
        want_lat = lat * 1e7
        if abs(got_lat - want_lat) > 1:
            raise NotAchievedException("Disagree in lat (got=%f want=%f)" % (got_lat, want_lat))
        if abs(items[0].y - lng * 1e7) > 1:
            raise NotAchievedException("Disagree in lng")
        if items[0].seq != 0:
            raise NotAchievedException("Disagree in offset")
        self.progress("Downloaded with new protocol OK")

        # upload using mission protocol:
        items = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)

        self.progress("Download with new protocol")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != len(items):
            raise NotAchievedException("Did not download expected number of items (wanted=%u got=%u)" %
                                       (len(items), len(downloaded_items)))
        self.assert_parameter_value("FENCE_TOTAL", len(items) +1)  # +1 for closing
        self.progress("Ensuring fence items match what we sent up")
        self.check_fence_items_same(items, downloaded_items)

        # now check centroid
        self.progress("Requesting fence return point")
        self.mav.mav.fence_fetch_point_send(target_system,
                                            target_component,
                                            0)
        m = self.mav.recv_match(type="FENCE_POINT", blocking=True, timeout=1)
        print("m: %s" % str(m))

        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.progress("Checking count post-nuke")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                  target_system=target_system,
                                  target_component=target_component)
        self.assert_mission_count_on_link(self.mav,
                                          0,
                                          target_system,
                                          target_component,
                                          mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        self.start_subtest("Ensuring bad fences get bounced")
        for fence in self.fences_which_should_not_upload(target_system=target_system, target_component=target_component):
            (name, items) = fence
            self.progress("Ensuring (%s) gets bounced" % (name,))
            self.assert_fence_does_not_upload(items)

        self.start_subtest("Ensuring good fences don't get bounced")
        for fence in self.fences_which_should_upload(target_system=target_system, target_component=target_component):
            (name, items) = fence
            self.progress("Ensuring (%s) gets uploaded" % (name,))
            self.check_fence_upload_download(items)
            self.progress("(%s) uploaded just fine" % (name,))

        self.test_gcs_fence_update_fencepoint(target_system=target_system,
                                              target_component=target_component)

        self.test_gcs_fence_centroid(target_system=target_system,
                                     target_component=target_component)

        self.test_gcs_fence_via_mavproxy(target_system=target_system,
                                         target_component=target_component)

    # explode the write_type_to_storage method
    # FIXME: test converting invalid fences / minimally valid fences / normal fences
    # FIXME: show that uploading smaller items take up less space
    # FIXME: add test for consecutive breaches within the manual recovery period
    # FIXME: ensure truncation does the right thing by fence_total

    # FIXME: test vehicle escape from outside inclusion zones to
    # inside inclusion zones (and inside exclusion zones to outside
    # exclusion zones)
    # FIXME: add test that a fence with edges that cross can't be uploaded
    # FIXME: add a test that fences enclose an area (e.g. all the points aren't the same value!

    def test_offboard(self, timeout=90):
        self.load_mission("rover-guided-mission.txt")
        self.wait_ready_to_arm(require_absolute=True)
        self.arm_vehicle()
        self.change_mode("AUTO")

        offboard_expected_duration = 10 # see mission file

        if self.mav.messages.get("SET_POSITION_TARGET_GLOBAL_INT", None):
            raise PreconditionFailedException("Already have SET_POSITION_TARGET_GLOBAL_INT")

        tstart = self.get_sim_time_cached()
        last_heartbeat_sent = 0
        got_sptgi = False
        magic_waypoint_tstart = 0
        magic_waypoint_tstop = 0
        while True:
            now = self.get_sim_time_cached()
            if now - last_heartbeat_sent > 1:
                last_heartbeat_sent = now
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                            0,
                                            0,
                                            0)

            if now - tstart > timeout:
                raise AutoTestTimeoutException("Didn't complete")
            magic_waypoint = 3
            mc = self.mav.recv_match(type=["MISSION_CURRENT", "STATUSTEXT"],
                                     blocking=False)
            if mc is not None:
                print("%s" % str(mc))
                if mc.get_type() == "STATUSTEXT":
                    if "Mission Complete" in mc.text:
                        break
                    continue
                if mc.seq == magic_waypoint:
                    print("At magic waypoint")
                    if magic_waypoint_tstart == 0:
                        magic_waypoint_tstart = self.get_sim_time_cached()
                    sptgi = self.mav.messages.get("SET_POSITION_TARGET_GLOBAL_INT", None)
                    if sptgi is not None:
                        got_sptgi = True
                elif mc.seq > magic_waypoint:
                    if magic_waypoint_tstop == 0:
                        magic_waypoint_tstop = self.get_sim_time_cached()

        self.disarm_vehicle()
        offboard_duration = magic_waypoint_tstop - magic_waypoint_tstart
        if abs(offboard_duration - offboard_expected_duration) > 1:
            raise NotAchievedException("Did not stay in offboard control for correct time (want=%f got=%f)" %
                                       (offboard_expected_duration, offboard_duration))

        if not got_sptgi:
            raise NotAchievedException("Did not get sptgi message")
        print("spgti: %s" % str(sptgi))

    def assert_mission_count_on_link(self, mav, expected_count, target_system, target_component, mission_type):
        self.drain_mav_unparsed(mav=mav, freshen_sim_time=True)
        self.progress("waiting for a message - any message....")
        m = mav.recv_match(blocking=True, timeout=1)
        self.progress("Received (%s)" % str(m))

        if not mav.mavlink20():
            raise NotAchievedException("Not doing mavlink2")
        mav.mav.mission_request_list_send(target_system,
                                          target_component,
                                          mission_type)
        self.assert_receive_mission_count_on_link(mav,
                                                  expected_count,
                                                  target_system,
                                                  target_component,
                                                  mission_type)

    def assert_receive_mission_count_on_link(self,
                                             mav,
                                             expected_count,
                                             target_system,
                                             target_component,
                                             mission_type,
                                             expected_target_system=None,
                                             expected_target_component=None,
                                             timeout=120):
        if expected_target_system is None:
            expected_target_system = mav.mav.srcSystem
        if expected_target_component is None:
            expected_target_component = mav.mav.srcComponent
        self.progress("Waiting for mission count of (%u) from (%u:%u) to (%u:%u)" %
                      (expected_count, target_system, target_component, expected_target_system, expected_target_component))

        tstart = self.get_sim_time_cached()
        while True:
            delta = self.get_sim_time_cached() - tstart
            if delta > timeout:
                raise NotAchievedException("Did not receive MISSION_COUNT on link after %fs" % delta)
            m = mav.recv_match(blocking=True, timeout=1)
            if m is None:
                self.progress("No messages")
                continue
#            self.progress("Received (%s)" % str(m))
            if m.get_type() == "MISSION_ACK":
                if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise NotAchievedException("Expected MAV_MISSION_ACCEPTED, got (%s)" % m)
            if m.get_type() == "MISSION_COUNT":
                break
        if m.target_system != expected_target_system:
            raise NotAchievedException("Incorrect target system in MISSION_COUNT (want=%u got=%u)" %
                                       (expected_target_system, m.target_system))
        if m.target_component != expected_target_component:
            raise NotAchievedException("Incorrect target component in MISSION_COUNT")
        if m.mission_type != mission_type:
            raise NotAchievedException("Did not get expected mission type (want=%u got=%u)" % (mission_type, m.mission_type))
        if m.count != expected_count:
            raise NotAchievedException("Bad count received (want=%u got=%u)" %
                                       (expected_count, m.count))
        self.progress("Asserted mission count (type=%u) is %u after %fs" % (
            (mission_type, m.count, delta)))

    def get_mission_item_int_on_link(self, item, mav, target_system, target_component, mission_type):
        self.drain_mav(mav=mav, unparsed=True)
        mav.mav.mission_request_int_send(target_system,
                                         target_component,
                                         item,
                                         mission_type)
        m = mav.recv_match(type='MISSION_ITEM_INT',
                           blocking=True,
                           timeout=60,
                           condition='MISSION_ITEM_INT.mission_type==%u' % mission_type)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_ITEM_INT")
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission item of incorrect type")
        if m.target_system != mav.mav.srcSystem:
            raise NotAchievedException("Unexpected target system %u want=%u" %
                                       (m.target_system, mav.mav.srcSystem))
        if m.seq != item:
            raise NotAchievedException("Incorrect sequence number on received item got=%u want=%u" %
            (m.seq, item))
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission type incorrect on received item (want=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.target_component != mav.mav.srcComponent:
            raise NotAchievedException("Unexpected target component %u want=%u" %
                                       (m.target_component, mav.mav.srcComponent))
        return m

    def get_mission_item_on_link(self, item, mav, target_system, target_component, mission_type):
        self.drain_mav(mav=mav, unparsed=True)
        mav.mav.mission_request_send(target_system,
                                     target_component,
                                     item,
                                     mission_type)
        m = mav.recv_match(type='MISSION_ITEM',
                           blocking=True,
                           timeout=60)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_ITEM")
        if m.target_system != mav.mav.srcSystem:
            raise NotAchievedException("Unexpected target system %u want=%u" %
                                       (m.target_system, mav.mav.srcSystem))
        if m.seq != item:
            raise NotAchievedException("Incorrect sequence number on received item_int got=%u want=%u" %
            (m.seq, item))
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission type incorrect on received item_int (want=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.target_component != mav.mav.srcComponent:
            raise NotAchievedException("Unexpected target component %u want=%u" %
                                       (m.target_component, mav.mav.srcComponent))
        return m

    def assert_receive_mission_item_request(self, mission_type, seq):
        self.progress("Expecting request for item %u" % seq)
        m = self.mav.recv_match(type='MISSION_REQUEST',
                                blocking=True,
                                timeout=1)
        if m is None:
            raise NotAchievedException("Did not get MISSION_REQUEST")
        if m.mission_type != mission_type:
            raise NotAchievedException("Incorrect mission type (wanted=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.seq != seq:
            raise NotAchievedException("Unexpected sequence number (want=%u got=%u)" % (seq, m.seq))
        self.progress("Received item request OK")

    def assert_receive_mission_ack(self, mission_type,
                                   want_type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
                                   target_system=None,
                                   target_component=None,
                                   mav=None):
        if mav is None:
            mav = self.mav
        if target_system is None:
            target_system = mav.mav.srcSystem
        if target_component is None:
            target_component = mav.mav.srcComponent
        self.progress("Expecting mission ack")
        m = mav.recv_match(type='MISSION_ACK',
                           blocking=True,
                           timeout=5)
        self.progress("Received ACK (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Expected mission ACK")
        if m.target_system != target_system:
            raise NotAchievedException("ACK not targetted at correct system want=%u got=%u" %
                                       (target_system, m.target_system))
        if m.target_component != target_component:
            raise NotAchievedException("ACK not targetted at correct component")
        if m.mission_type != mission_type:
            raise NotAchievedException("Unexpected mission type %u want=%u" %
                                       (m.mission_type, mission_type))
        if m.type != want_type:
            raise NotAchievedException("Expected ack type got %u got %u" %
                                       (want_type, m.type))

    def assert_filepath_content(self, filepath, want):
        with open(filepath) as f:
            got = f.read()
        if want != got:
            raise NotAchievedException("Did not get expected file content (want=%s) (got=%s)" % (want, got))

    def mavproxy_can_do_mision_item_protocols(self):
        return False
        mavproxy_version = self.mavproxy_version()
        if not self.mavproxy_version_gt(1, 8, 12):
            self.progress("MAVProxy is too old; skipping tests")
            return False
        return True

    def check_rally_items_same(self, want, got, epsilon=None):
        check_atts = ['mission_type', 'command', 'x', 'y', 'z', 'seq', 'param1']
        return self.check_mission_items_same(check_atts, want, got, epsilon=epsilon)

    def click_three_in(self, target_system=1, target_component=1):
        self.mavproxy.send('rally clear\n')
        self.drain_mav_unparsed()
        # there are race conditions in MAVProxy.  Beware.
        self.mavproxy.send("click 1.0 1.0\n")
        self.mavproxy.send("rally add\n")
        self.delay_sim_time(1)
        self.mavproxy.send("click 2.0 2.0\n")
        self.mavproxy.send("rally add\n")
        self.delay_sim_time(1)
        self.mavproxy.send("click 3.0 3.0\n")
        self.mavproxy.send("rally add\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            3,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )

    def test_gcs_rally_via_mavproxy(self, target_system=1, target_component=1):
        self.start_subtest("Testing mavproxy CLI for rally points")
        if not self.mavproxy_can_do_mision_item_protocols():
            return

        self.start_subsubtest("rally add")
        self.mavproxy.send('rally clear\n')
        lat_s = "-5.6789"
        lng_s = "98.2341"
        lat = float(lat_s)
        lng = float(lng_s)
        self.mavproxy.send('click %s %s\n' % (lat_s, lng_s))
        self.drain_mav_unparsed()
        self.mavproxy.send('rally add\n')
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        self.delay_sim_time(5)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 1:
            raise NotAchievedException("Unexpected count (got=%u want=1)" %
                                       (len(downloaded_items), ))
        if (downloaded_items[0].x - int(lat *1e7)) > 1:
            raise NotAchievedException("Bad rally lat.  Want=%d got=%d" %
                                       (int(lat*1e7), downloaded_items[0].x))
        if (downloaded_items[0].y - int(lng *1e7)) > 1:
            raise NotAchievedException("Bad rally lng.  Want=%d got=%d" %
                                       (int(lng*1e7), downloaded_items[0].y))
        if (downloaded_items[0].z - int(90)) > 1:
            raise NotAchievedException("Bad rally alt.  Want=90 got=%d" %
                                       (downloaded_items[0].y))
        self.end_subsubtest("rally add")

        self.start_subsubtest("rally list")
        util.pexpect_drain(self.mavproxy)
        self.mavproxy.send('rally list\n')
        self.mavproxy.expect("Saved 1 rally items to ([^\s]*)\s")
        filename = self.mavproxy.match.group(1)
        self.assert_rally_filepath_content(filename, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	90.000000	0
''')
        self.end_subsubtest("rally list")

        self.start_subsubtest("rally save")
        util.pexpect_drain(self.mavproxy)
        save_tmppath = self.buildlogs_path("rally-testing-tmp.txt")
        self.mavproxy.send('rally save %s\n' % save_tmppath)
        self.mavproxy.expect("Saved 1 rally items to ([^\s]*)\s")
        filename = self.mavproxy.match.group(1)
        if filename != save_tmppath:
            raise NotAchievedException("Bad save filepath; want=%s got=%s" % (save_tmppath, filename))
        self.assert_rally_filepath_content(filename, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	90.000000	0
''')
        self.end_subsubtest("rally save")


        self.start_subsubtest("rally savecsv")
        util.pexpect_drain(self.mavproxy)
        csvpath = self.buildlogs_path("rally-testing-tmp.csv")
        self.mavproxy.send('rally savecsv %s\n' % csvpath)
        self.mavproxy.expect('"Seq","Frame"')
        expected_content = '''"Seq","Frame","Cmd","P1","P2","P3","P4","X","Y","Z"
"0","Rel","NAV_RALLY_POINT","0.0","0.0","0.0","0.0","-5.67890024185","98.2341003418","90.0"
'''
        if sys.version_info[0] >= 3:
            # greater precision output by default
            expected_content = '''"Seq","Frame","Cmd","P1","P2","P3","P4","X","Y","Z"
"0","Rel","NAV_RALLY_POINT","0.0","0.0","0.0","0.0","-5.678900241851807","98.23410034179688","90.0"
'''
        self.assert_filepath_content(csvpath, expected_content)

        self.end_subsubtest("rally savecsv")

        self.start_subsubtest("rally load")
        self.drain_mav()
        self.mavproxy.send('rally clear\n')
        self.assert_mission_count_on_link(self.mav,
                                          0,
                                          target_system,
                                          target_component,
                                          mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

        # warning: uses file saved from previous test
        self.start_subtest("Check rally load from filepath")
        self.mavproxy.send('rally load %s\n' % save_tmppath)
        self.mavproxy.expect("Loaded 1 rally items from ([^\s]*)\s")
        self.mavproxy.expect("Sent all .* rally items") # notional race condition here
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 1:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if abs(int(downloaded_items[0].x) - int(lat*1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (lat*1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(lng*1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (lng*1e7, downloaded_items[0].y))
        self.end_subsubtest("rally load")

        self.start_subsubtest("rally changealt")
        self.mavproxy.send('rally clear\n')
        self.mavproxy.send("click 1.0 1.0\n")
        self.mavproxy.send("rally add\n")
        self.mavproxy.send("click 2.0 2.0\n")
        self.mavproxy.send("rally add\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.drain_mav()
        self.mavproxy.send("rally changealt 1 17.6\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        self.delay_sim_time(10)
        self.mavproxy.send("rally changealt 2 19.1\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        self.delay_sim_time(10)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if abs(int(downloaded_items[0].x) - int(1*1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (1*1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(1*1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (1*1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[0].z) - int(17.6)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.6, downloaded_items[0].z))

        if abs(int(downloaded_items[1].x) - int(2*1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (2*1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[1].y) - int(2*1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (2*1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[1].z) - int(19.1)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (19.1, downloaded_items[1].z))

        self.progress("Now change two at once")
        self.mavproxy.send("rally changealt 1 17.3 2\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))

        if abs(int(downloaded_items[0].x) - int(1*1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (1*1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(1*1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (1*1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[0].z) - int(17.3)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.3, downloaded_items[0].z))

        if abs(int(downloaded_items[1].x) - int(2*1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (2*1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[1].y) - int(2*1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (2*1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[1].z) - int(17.3)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.3, downloaded_items[0].z))

        self.end_subsubtest("rally changealt")

        self.start_subsubtest("rally move")
        self.mavproxy.send('rally clear\n')
        self.mavproxy.send("click 1.0 1.0\n")
        self.mavproxy.send("rally add\n")
        self.mavproxy.send("click 2.0 2.0\n")
        self.mavproxy.send("rally add\n")
        self.delay_sim_time(5)
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.mavproxy.send("click 3.0 3.0\n")
        self.mavproxy.send("rally move 2\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        self.mavproxy.send("click 4.12345 4.987654\n")
        self.mavproxy.send("rally move 1\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)

        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if downloaded_items[0].x != 41234500:
            raise NotAchievedException("Bad latitude")
        if downloaded_items[0].y != 49876540:
            raise NotAchievedException("Bad longitude")
        if downloaded_items[0].z != 90:
            raise NotAchievedException("Bad altitude (want=%u got=%u)" %
                                       (90, downloaded_items[0].z))

        if downloaded_items[1].x != 30000000:
            raise NotAchievedException("Bad latitude")
        if downloaded_items[1].y != 30000000:
            raise NotAchievedException("Bad longitude")
        if downloaded_items[1].z != 90:
            raise NotAchievedException("Bad altitude (want=%u got=%u)" %
                                       (90, downloaded_items[1].z))
        self.end_subsubtest("rally move")

        self.start_subsubtest("rally movemulti")
        self.drain_mav_unparsed()
        self.mavproxy.send('rally clear\n')
        self.drain_mav_unparsed()
        # there are race conditions in MAVProxy.  Beware.
        self.mavproxy.send("click 1.0 1.0\n")
        self.mavproxy.send("rally add\n")
        self.mavproxy.send("click 2.0 2.0\n")
        self.mavproxy.send("rally add\n")
        self.mavproxy.send("click 3.0 3.0\n")
        self.mavproxy.send("rally add\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            3,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        click_lat = 2.0
        click_lon = 3.0
        unmoved_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(unmoved_items) != 3:
            raise NotAchievedException("Unexpected item count")
        self.mavproxy.send("click %f %f\n" % (click_lat, click_lon))
        self.mavproxy.send("rally movemulti 2 1 3\n")
        # MAVProxy currently sends three separate items up.  That's
        # not great and I don't want to lock that behaviour in here.
        self.delay_sim_time(10)
        self.drain_mav_unparsed()
        expected_moved_items = copy.copy(unmoved_items)
        expected_moved_items[0].x = 1.0 * 1e7
        expected_moved_items[0].y = 2.0 * 1e7
        expected_moved_items[1].x = 2.0 * 1e7
        expected_moved_items[1].y = 3.0 * 1e7
        expected_moved_items[2].x = 3.0 * 1e7
        expected_moved_items[2].y = 4.0 * 1e7
        moved_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        # we're moving an entire degree in latitude; quite an epsilon required...
        self.check_rally_items_same(expected_moved_items, moved_items, epsilon=10000)

        self.progress("now move back and rotate through 90 degrees")
        self.mavproxy.send("click %f %f\n" % (2, 2))
        self.mavproxy.send("rally movemulti 2 1 3 90\n")

        # MAVProxy currently sends three separate items up.  That's
        # not great and I don't want to lock that behaviour in here.
        self.delay_sim_time(10)
        self.drain_mav_unparsed()
        expected_moved_items = copy.copy(unmoved_items)
        expected_moved_items[0].x = 3.0 * 1e7
        expected_moved_items[0].y = 1.0 * 1e7
        expected_moved_items[1].x = 2.0 * 1e7
        expected_moved_items[1].y = 2.0 * 1e7
        expected_moved_items[2].x = 1.0 * 1e7
        expected_moved_items[2].y = 3.0 * 1e7
        moved_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        # we're moving an entire degree in latitude; quite an epsilon required...
        self.check_rally_items_same(expected_moved_items, moved_items, epsilon=12000)
        self.end_subsubtest("rally movemulti")

        self.start_subsubtest("rally param")
        self.mavproxy.send("rally param 3 2 5\n")
        self.mavproxy.expect("Set param 2 for 3 to 5.000000")
        self.end_subsubtest("rally param")

        self.start_subsubtest("rally remove")
        self.click_three_in(target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.progress("Removing last in list")
        self.mavproxy.send("rally remove 3\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        fewer_downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(fewer_downloaded_items) != 2:
            raise NotAchievedException("Unexpected download list length")
        shorter_items = copy.copy(pure_items)
        shorter_items = shorter_items[0:2]
        self.check_rally_items_same(shorter_items, fewer_downloaded_items)

        self.progress("Removing first in list")
        self.mavproxy.send("rally remove 1\n")
        self.delay_sim_time(5)
        self.drain_mav_unparsed()
        self.assert_mission_count_on_link(
            self.mav,
            1,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        fewer_downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(fewer_downloaded_items) != 1:
            raise NotAchievedException("Unexpected download list length")
        shorter_items = shorter_items[1:]
        self.check_rally_items_same(shorter_items, fewer_downloaded_items)

        self.progress("Removing remaining item")
        self.mavproxy.send("rally remove 1\n")
        self.delay_sim_time(5)
        self.drain_mav_unparsed()
        self.assert_mission_count_on_link(
            self.mav,
            0,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.end_subsubtest("rally remove")

        self.start_subsubtest("rally show")
        # what can we test here?
        self.mavproxy.send("rally show %s\n" % save_tmppath)
        self.end_subsubtest("rally show")

        # savelocal must be run immediately after show!
        self.start_subsubtest("rally savelocal")
        util.pexpect_drain(self.mavproxy)
        savelocal_path = self.buildlogs_path("rally-testing-tmp-local.txt")
        self.mavproxy.send('rally savelocal %s\n' % savelocal_path)
        self.delay_sim_time(5)
        self.assert_rally_filepath_content(savelocal_path, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	90.000000	0
''')
        self.end_subsubtest("rally savelocal")

        self.start_subsubtest("rally status")
        self.click_three_in(target_system=target_system, target_component=target_component)
        self.mavproxy.send("rally status\n")
        self.mavproxy.expect("Have 3 of 3 rally items")
        self.mavproxy.send("rally clear\n")
        self.mavproxy.send("rally status\n")
        self.mavproxy.expect("Have 0 of 0 rally items")
        self.end_subsubtest("rally status")

        self.start_subsubtest("rally undo")
        self.progress("Testing undo-remove")
        self.click_three_in(target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.progress("Removing first in list")
        self.mavproxy.send("rally remove 1\n")
        self.delay_sim_time(5)
        self.drain_mav_unparsed()
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.mavproxy.send("rally undo\n")
        self.delay_sim_time(5)
        undone_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, undone_items)

        self.progress("Testing undo-move")

        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.mavproxy.send("click 4.12345 4.987654\n")
        self.mavproxy.send("rally move 1\n")
        # move has already been tested, assume it works...
        self.delay_sim_time(5)
        self.drain_mav_unparsed()
        self.mavproxy.send("rally undo\n")
        self.delay_sim_time(5)
        self.drain_mav_unparsed()
        undone_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, undone_items)

        self.end_subsubtest("rally undo")

        self.start_subsubtest("rally update")
        self.click_three_in(target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        rally_update_tmpfilepath= self.buildlogs_path("rally-tmp-update.txt")
        self.mavproxy.send("rally save %s\n" % rally_update_tmpfilepath)
        self.delay_sim_time(5)
        self.progress("Moving waypoint")
        self.mavproxy.send("click 13.0 13.0\n")
        self.mavproxy.send("rally move 1\n")
        self.delay_sim_time(5)
        self.progress("Reverting to original")
        self.mavproxy.send("rally update %s\n" % rally_update_tmpfilepath)
        self.delay_sim_time(5)
        reverted_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, reverted_items)

        self.progress("Making sure specifying a waypoint to be updated works")
        self.mavproxy.send("click 13.0 13.0\n")
        self.mavproxy.send("rally move 1\n")
        self.delay_sim_time(5)
        self.mavproxy.send("click 17.0 17.0\n")
        self.mavproxy.send("rally move 2\n")
        self.delay_sim_time(5)
        self.progress("Reverting to original item 2")
        self.mavproxy.send("rally update %s 2\n" % rally_update_tmpfilepath)
        self.delay_sim_time(5)
        reverted_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if reverted_items[0].x != 130000000:
            raise NotAchievedException("Expected item1 x to stay changed (got=%u want=%u)" % (reverted_items[0].x, 130000000))
        if reverted_items[1].x == 170000000:
            raise NotAchievedException("Expected item2 x to revert")

        self.end_subsubtest("rally update")

# MANUAL> usage: rally <add|alt|changealt|clear|list|load|move|movemulti|param|remove|save|savecsv|savelocal|show|status|undo|update>

    def test_gcs_rally(self):
        target_system = 1
        target_component = 1

        self.test_gcs_rally_via_mavproxy(target_system=target_system,
                                         target_component=target_component)

        self.mavproxy.send('rally clear\n')
        self.delay_sim_time(1)
        if self.get_parameter("RALLY_TOTAL") != 0:
            raise NotAchievedException("Failed to clear rally points")

        old_srcSystem = self.mav.mav.srcSystem

        # stop MAVProxy poking the autopilot:
        self.mavproxy.send('module unload rally\n')
        self.mavproxy.expect("Unloaded module rally")
        self.mavproxy.send('module unload wp\n')
        self.mavproxy.expect("Unloaded module wp")
        self.drain_mav()
        try:
            item1_lat = int(2.0000 *1e7)
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
                    int(1.0000 *1e7), # latitude
                    int(1.0000 *1e7), # longitude
                    31.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    1, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    item1_lat, # latitude
                    int(2.0000 *1e7), # longitude
                    32.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    2, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(3.0000 *1e7), # latitude
                    int(3.0000 *1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            ]
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            print("Got items (%s)" % str(items))
            if len(downloaded) != len(items):
                raise NotAchievedException("Did not download correct number of items want=%u got=%u" % (len(downloaded), len(items)))

            rally_total = self.get_parameter("RALLY_TOTAL")
            if rally_total != len(downloaded):
                raise NotAchievedException("Unexpected rally point count: want=%u got=%u" % (len(items), rally_total))

            self.progress("Pruning count by setting parameter (urgh)")
            self.set_parameter("RALLY_TOTAL", 2)
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if len(downloaded) != 2:
                raise NotAchievedException("Failed to prune rally points by setting parameter.  want=%u got=%u" % (2, len(downloaded)))

            self.progress("Uploading a third item using old protocol")
            new_item2_lat = int(6.0 *1e7)
            self.set_parameter("RALLY_TOTAL", 3)
            self.mav.mav.rally_point_send(target_system,
                                          target_component,
                                          2, # sequence number
                                          3, # total count
                                          new_item2_lat,
                                          int(7.0 *1e7),
                                          15,
                                          0, # "break" alt?!
                                          0, # "land dir"
                                          0) # flags
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if len(downloaded) != 3:
                raise NotAchievedException("resetting rally point count didn't change items returned")
            if downloaded[2].x != new_item2_lat:
                raise NotAchievedException("Bad lattitude in downloaded item: want=%u got=%u" % (new_item2_lat, downloaded[2].x))

            self.progress("Grabbing original item 1 using original protocol")
            self.mav.mav.rally_fetch_point_send(target_system,
                                                target_component,
                                                1)
            m = self.mav.recv_match(type="RALLY_POINT", blocking=True, timeout=1)
            if m.target_system != 255:
                raise NotAchievedException("Bad target_system on received rally point (want=%u got=%u)" % (255, m.target_system))
            if m.target_component != 250: # autotest's component ID
                raise NotAchievedException("Bad target_component on received rally point")
            if m.lat != item1_lat:
                raise NotAchievedException("Bad latitude on received rally point")

            self.start_subtest("Test upload lockout and timeout")
            self.progress("Starting upload from normal sysid")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            len(items),
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.drain_mav() # throw away requests for items
            self.mav.mav.srcSystem = 243

            self.progress("Attempting upload from sysid=%u" %
                          (self.mav.mav.srcSystem,))
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            len(items),
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED)

            self.progress("Attempting download from sysid=%u" %
                          (self.mav.mav.srcSystem,))
            self.mav.mav.mission_request_list_send(target_system,
                                                   target_component,
                                                   mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED)

            # wait for the upload from sysid=1 to time out:
            tstart = self.get_sim_time()
            got_statustext = False
            got_ack = False
            while True:
                if got_statustext and got_ack:
                    self.progress("Got both ack and statustext")
                    break
                if self.get_sim_time_cached() - tstart > 100:
                    raise NotAchievedException("Did not get both ack and statustext")
                m = self.mav.recv_match(type=['STATUSTEXT','MISSION_ACK'], blocking=True, timeout=1)
                if m is None:
                    continue
                self.progress("Got (%s)" % str(m))
                if m.get_type() == 'STATUSTEXT':
                    if "upload timeout" in m.text:
                        got_statustext = True
                        self.progress("Received desired statustext")
                    continue
                if m.get_type() == 'MISSION_ACK':
                    if m.target_system != old_srcSystem:
                        raise NotAchievedException("Incorrect sourcesystem")
                    if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                        raise NotAchievedException("Incorrect result")
                    if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_RALLY:
                        raise NotAchievedException("Incorrect mission_type")
                    got_ack = True
                    self.progress("Received desired ACK")
                    continue
                raise NotAchievedException("Huh?")

            self.progress("Now trying to upload empty mission after timeout")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            0,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.start_subtest("Check rally upload/download across separate links")
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            self.progress("ensure a mavlink1 connection can't do anything useful with new item types")
            self.set_parameter("SERIAL2_PROTOCOL", 1)
            self.reboot_sitl()
            mav2 = mavutil.mavlink_connection("tcp:localhost:5763",
                                              robust_parsing=True,
                                              source_system = 7,
                                              source_component=7)
            mav2.mav.mission_request_list_send(target_system,
                                               target_component,
                                               mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            # so this looks a bit odd; the other end isn't sending
            # mavlink2 so can't fill in the extension here.
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                want_type=mavutil.mavlink.MAV_MISSION_UNSUPPORTED,
                mav=mav2,
            )
            # this relies on magic upgrade to serial2:
            self.set_parameter("SERIAL2_PROTOCOL", 2)
            expected_count = 3
            self.progress("Assert mission count on new link")
            self.assert_mission_count_on_link(mav2, expected_count, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Assert mission count on original link")
            self.assert_mission_count_on_link(self.mav, expected_count, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Get first item on new link")
            m2 = self.get_mission_item_int_on_link(2, mav2, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Get first item on original link")
            m = self.get_mission_item_int_on_link(2, self.mav, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if m2.x != m.x:
                raise NotAchievedException("mission items do not match (%d vs %d)" % (m2.x, m.x))
            self.get_mission_item_on_link(2, self.mav, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            # ensure we get nacks for bad mission item requests:
            self.mav.mav.mission_request_send(target_system,
                                              target_component,
                                              65,
                                              mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE,
            )
            self.mav.mav.mission_request_int_send(target_system,
                                                  target_component,
                                                  65,
                                                  mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE,
            )

            self.start_subtest("Should enforce items come from correct GCS")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)
            self.progress("Attempting to upload from bad sysid")
            old_sysid = self.mav.mav.srcSystem
            self.mav.mav.srcSystem = 17
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.mav.mav.srcSystem = old_sysid
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED,
                                            target_system=17)
            self.progress("Sending from correct sysid")
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Attempt to send item on different link to that which we are sending requests on")
            self.progress("Sending count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            2,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)
            old_mav2_system = mav2.mav.srcSystem
            old_mav2_component = mav2.mav.srcComponent
            mav2.mav.srcSystem = self.mav.mav.srcSystem
            mav2.mav.srcComponent = self.mav.mav.srcComponent
            self.progress("Sending item on second link")
            # note that the routing table in ArduPilot now will say
            # this sysid/compid is on both links which may cause
            # weirdness...
            items[0].pack(mav2.mav)
            self.drain_mav(mav=self.mav, unparsed=True)
            mav2.mav.send(items[0])
            mav2.mav.srcSystem = old_mav2_system
            mav2.mav.srcComponent = old_mav2_component
            # we continue to receive requests on the original link:
            m = self.mav.recv_match(type='MISSION_REQUEST',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                raise NotAchievedException("Did not get mission request")
            if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_RALLY:
                raise NotAchievedException("Mission request of incorrect type")
            if m.seq != 1:
                raise NotAchievedException("Unexpected sequence number (expected=%u got=%u)" % (1, m.seq))
            items[1].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[1])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Upload mission and rally points at same time")
            self.progress("Sending rally count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            3,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)

            self.progress("Sending wp count")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            3,
                                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 0)

            self.progress("Answering request for mission item 0")
            self.drain_mav(mav=self.mav, unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 *1e7), # latitude
                int(1.2000 *1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 1)

            self.progress("Answering request for rally point 0")
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.progress("Expecting request for rally item 1")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Answering request for rally point 1")
            items[1].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[1])
            self.progress("Expecting request for rally item 2")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)

            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[2])
            self.progress("Expecting mission ack for rally")
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.progress("Answering request for waypoints item 1")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 *1e7), # latitude
                int(1.2000 *1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 2)

            self.progress("Answering request for waypoints item 2")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 *1e7), # latitude
                int(1.2000 *1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            self.start_subtest("Test write-partial-list")
            self.progress("Clearing rally points using count-send")
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                               target_system=target_system,
                               target_component=target_component)
            self.progress("Should not be able to set items completely past the waypoint count")
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                17,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("Should not be able to set items overlapping the waypoint count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                0,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("try to overwrite items 1 and 2")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                1,
                2,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Try shoving up an incorrectly sequenced item")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
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
                int(1.1000 *1e7), # latitude
                int(1.2000 *1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Try shoving up an incorrectly sequenced item (but within band)")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 *1e7), # latitude
                int(1.2000 *1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Now provide correct item")
            item1_latitude = int(1.2345*1e7)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                item1_latitude, # latitude
                int(1.2000 *1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)
            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[2])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("TODO: ensure partial mission write was good")

            self.start_subtest("clear mission types")
            self.assert_mission_count_on_link(self.mav, 3, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 3, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 0, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 3, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_mission_count_on_link(self.mav, 0, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 0, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            self.start_subtest("try sending out-of-range counts")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            230)
            self.assert_receive_mission_ack(230,
                                            want_type=mavutil.mavlink.MAV_MISSION_UNSUPPORTED)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            16000,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_NO_SPACE)

        except Exception as e:
            self.progress("Received exception (%s)" % self.get_exception_stacktrace(e))
            self.mav.mav.srcSystem = old_srcSystem
            raise e
        self.mavproxy.send('module load rally\n')
        self.mavproxy.expect("Loaded module rally")
        self.mavproxy.send('module load wp\n')
        self.mavproxy.expect("Loaded module wp")
        self.reboot_sitl()

    def test_gcs_mission(self):
        target_system = 1
        target_component = 1
        self.mavproxy.send('wp clear\n')
        self.delay_sim_time(1)
        if self.get_parameter("MIS_TOTAL") != 0:
            raise NotAchievedException("Failed to clear mission")
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
        if m is None:
            raise NotAchievedException("Did not get expected MISSION_CURRENT")
        if m.seq != 0:
            raise NotAchievedException("Bad mission current")
        self.load_mission("rover-gripper-mission.txt")
        set_wp = 1
        self.mavproxy.send('wp set %u\n' % set_wp)
        self.drain_mav()
        m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
        if m is None:
            raise NotAchievedException("Did not get expected MISSION_CURRENT")
        if m.seq != set_wp:
            raise NotAchievedException("Bad mission current.  want=%u got=%u" %
                                       (set_wp, m.seq))

        self.start_subsubtest("wp changealt")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        changealt_item = 1
        oldalt = downloaded_items[changealt_item].z
        want_newalt = 37.2
        self.mavproxy.send('wp changealt %u %f\n' % (changealt_item, want_newalt))
        self.delay_sim_time(5)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        if abs(downloaded_items[changealt_item].z -  want_newalt) > 0.0001:
            raise NotAchievedException(
                "changealt didn't (want=%f got=%f)" %
                (want_newalt, downloaded_items[changealt_item].z))
        self.end_subsubtest("wp changealt")

        self.start_subsubtest("wp sethome")
        new_home_lat = 3.14159
        new_home_lng = 2.71828
        self.mavproxy.send('click %f %f\n' % (new_home_lat, new_home_lng))
        self.mavproxy.send('wp sethome\n')
        self.delay_sim_time(5)
        # any way to close the loop on this one?
        # downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        # if abs(downloaded_items[0].x - new_home_lat) > 0.0001:
        #     raise NotAchievedException("wp sethome didn't work")
        # if abs(downloaded_items[0].y - new_home_lng) > 0.0001:
        #     raise NotAchievedException("wp sethome didn't work")
        self.end_subsubtest("wp sethome")

        self.start_subsubtest("wp slope")
        self.mavproxy.send('wp slope\n')
        self.mavproxy.expect("WP3: slope 0.1")
        self.delay_sim_time(5)
        self.end_subsubtest("wp slope")

        if not self.mavproxy_can_do_mision_item_protocols():
            # adding based on click location yet to be merged into MAVProxy
            return

        self.start_subsubtest("wp split")
        self.mavproxy.send("wp clear\n")
        self.delay_sim_time(5)
        self.mavproxy.send("wp list\n")
        self.delay_sim_time(5)
        items = [
            None,
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 *1e7), # latitude
                int(1.0 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(2.0 *1e7), # latitude
                int(2.0 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        ]
        self.mavproxy.send("click 5 5\n") # space for home position
        self.mavproxy.send("wp add\n")
        self.delay_sim_time(5)
        self.click_location_from_item(items[1])
        self.mavproxy.send("wp add\n")
        self.delay_sim_time(5)
        self.click_location_from_item(items[2])
        self.mavproxy.send("wp add\n")
        self.delay_sim_time(5)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.check_mission_waypoint_items_same(items, downloaded_items)
        self.mavproxy.send("wp split 2\n")
        self.delay_sim_time(5)
        items_with_split_in = [
            items[0],
            items[1],
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.5 *1e7), # latitude
                int(1.5 *1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            items[2],
        ]
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.check_mission_waypoint_items_same(items_with_split_in,
                                               downloaded_items)

        # MANUAL> usage: wp <changealt|clear|draw|editor|list|load|loop|move|movemulti|noflyadd|param|remove|save|savecsv|savelocal|set|sethome|show|slope|split|status|undo|update>

    def wait_location_sending_target(self, loc, target_system=1, target_component=1, timeout=60, max_delta=2):
        tstart = self.get_sim_time()
        last_sent = 0

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("Did not get to location")
            if now - last_sent > 1:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat*1.0e7),
                    int(loc.lng*1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                self.progress("Target: (%s)" % str(m), send_statustext=False)
            elif t == "GLOBAL_POSITION_INT":
                self.progress("Position: (%s)" % str(m), send_statustext=False)
                delta = self.get_distance(mavutil.location(m.lat*1e-7, m.lon*1e-7, 0, 0),
                                          loc)
                self.progress("delta: %s" % str(delta), send_statustext=False)
                if delta < max_delta:
                    self.progress("Reached destination")

    def drive_somewhere_breach_boundary_and_rtl(self, loc, target_system=1, target_component=1, timeout=60):
        tstart = self.get_sim_time()
        last_sent = 0
        seen_fence_breach = False

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not breach boundary + RTL")
            if now - last_sent > 1:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat*1.0e7),
                    int(loc.lng*1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                print("Target: (%s)" % str(m))
            elif t == "GLOBAL_POSITION_INT":
                print("Position: (%s)" % str(m))
            elif t == "FENCE_STATUS":
                print("Fence: %s" % str(m))
                if m.breach_status != 0:
                    seen_fence_breach = True
                    self.progress("Fence breach detected!")
                    if m.breach_type != mavutil.mavlink.FENCE_BREACH_BOUNDARY:
                        raise NotAchievedException("Breach of unexpected type")
            if self.mode_is("RTL", cached=True) and seen_fence_breach:
                break
        self.wait_distance_to_home(3, 7, timeout=30)

    def drive_somewhere_stop_at_boundary(self,
                                         loc,
                                         expected_stopping_point,
                                         expected_distance_epsilon=1.0,
                                         target_system=1,
                                         target_component=1,
                                         timeout=120):
        tstart = self.get_sim_time()
        last_sent = 0
        seen_fence_breach = False

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')
        at_stopping_point = False
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not arrive and stop at boundary")
            if now - last_sent > 1:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat*1.0e7),
                    int(loc.lng*1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                print("Target: (%s)" % str(m))
            elif t == "GLOBAL_POSITION_INT":
                print("Position: (%s)" % str(m))
                delta = self.get_distance(mavutil.location(m.lat*1e-7, m.lon*1e-7, 0, 0),
                                          mavutil.location(expected_stopping_point.lat, expected_stopping_point.lng, 0, 0))
                print("delta: %s want_delta<%f" % (str(delta), expected_distance_epsilon))
                at_stopping_point = delta < expected_distance_epsilon
            elif t == "VFR_HUD":
                print("groundspeed: %f" % m.groundspeed)
                if at_stopping_point:
                    if m.groundspeed < 1:
                        self.progress("Seemed to have stopped at stopping point")
                        return

    def assert_fence_breached(self):
        m = self.mav.recv_match(type='FENCE_STATUS',
                                blocking=True,
                                timeout=10)
        if m is None:
            raise NotAchievedException("Not receiving fence notifications?")
        if m.breach_status != 1:
            raise NotAchievedException("Expected to be breached")

    def wait_fence_not_breached(self, timeout=5):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("Fence remains breached")
            m = self.mav.recv_match(type='FENCE_STATUS',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                self.progress("No FENCE_STATUS received")
                continue
            self.progress("STATUS: %s" % str(m))
            if m.breach_status == 0:
                break

    def test_poly_fence_noarms(self, target_system=1, target_component=1):
        '''various tests to ensure we can't arm when in breach of a polyfence'''
        self.start_subtest("Ensure PolyFence arming checks work")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.delay_sim_time(5) # let breaches clear
        # FIXME: should we allow this?
        self.progress("Ensure we can arm with no poly in place")
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        self.test_poly_fence_noarms_exclusion_circle(target_system=target_system,
                                                     target_component=target_component)
        self.test_poly_fence_noarms_inclusion_circle(target_system=target_system,
                                                     target_component=target_component)
        self.test_poly_fence_noarms_exclusion_polyfence(target_system=target_system,
                                                        target_component=target_component)
        self.test_poly_fence_noarms_inclusion_polyfence(target_system=target_system,
                                                        target_component=target_component)

    def test_poly_fence_noarms_exclusion_circle(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when within an exclusion circle")

        here = self.mav.location()

        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(here.lat*1e7), # latitude
                int(here.lng*1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(self.offset_location_ne(here, 100, 100).lat*1e7), # latitude
                int(here.lng*1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        if self.arm_motors_with_rc_input():
            raise NotAchievedException(
                "Armed when within exclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_inclusion_circle(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when outside an inclusion circle (but within another")

        here = self.mav.location()

        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(here.lat*1e7), # latitude
                int(here.lng*1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(self.offset_location_ne(here, 100, 100).lat*1e7), # latitude
                int(here.lng*1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ];
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        if self.arm_motors_with_rc_input():
            raise NotAchievedException(
                "Armed when outside an inclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_exclusion_polyfence(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when inside an exclusion polyfence (but outside another")

        here = self.mav.location()

        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            [
                [ # east
                    self.offset_location_ne(here, -50, 20), # bl
                    self.offset_location_ne(here, 50, 20), # br
                    self.offset_location_ne(here, 50, 40), # tr
                    self.offset_location_ne(here, -50, 40), # tl,
                ], [ # over the top of the vehicle
                    self.offset_location_ne(here, -50, -50), # bl
                    self.offset_location_ne(here, -50, 50), # br
                    self.offset_location_ne(here, 50, 50), # tr
                    self.offset_location_ne(here, 50, -50), # tl,
                ]
            ]
        )
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        if self.arm_motors_with_rc_input():
            raise NotAchievedException(
                "Armed when within polygon exclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_inclusion_polyfence(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when outside an inclusion polyfence (but within another")

        here = self.mav.location()

        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            [
                [ # east
                    self.offset_location_ne(here, -50, 20), # bl
                    self.offset_location_ne(here, 50, 20), # br
                    self.offset_location_ne(here, 50, 40), # tr
                    self.offset_location_ne(here, -50, 40), # tl,
                ], [ # over the top of the vehicle
                    self.offset_location_ne(here, -50, -50), # bl
                    self.offset_location_ne(here, -50, 50), # br
                    self.offset_location_ne(here, 50, 50), # tr
                    self.offset_location_ne(here, 50, -50), # tl,
                ]
            ]
        )
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        if self.arm_motors_with_rc_input():
            raise NotAchievedException(
                "Armed when outside polygon inclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_fence_upload_timeouts_1(self, target_system=1, target_component=1):
        self.start_subtest("fence_upload timeouts 1")
        self.progress("Telling victim there will be one item coming")
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        1,
                                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'],
                                blocking=True,
                                timeout=1)
        self.progress("Got (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Did not get ACK or mission request")

        if m.get_type() == "MISSION_ACK":
            raise NotAchievedException("Expected MISSION_REQUEST")

        if m.seq != 0:
            raise NotAchievedException("Expected request for seq=0")

        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
        tstart = self.get_sim_time()
        rerequest_count = 0
        received_text = False
        received_ack = False
        while True:
            if received_ack and received_text:
                break
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not get expected ack and statustext")
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK', 'STATUSTEXT'],
                                    blocking=True,
                                    timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.progress("Did not receive any messages")
                continue
            if m.get_type() == "MISSION_REQUEST":
                if m.seq != 0:
                    raise NotAchievedException("Received request for invalid seq")
                if m.target_system != self.mav.mav.srcSystem:
                    raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
                if m.target_component != self.mav.mav.srcComponent:
                    raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
                rerequest_count += 1
                self.progress("Valid re-request received.")
                continue
            if m.get_type() == "MISSION_ACK":
                if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    raise NotAchievedException("Wrong mission type")
                if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                    raise NotAchievedException("Wrong result")
                received_ack = True
                continue
            if m.get_type() == "STATUSTEXT":
                if "upload time" in m.text:
                    received_text = True
                continue
        if rerequest_count < 3:
            raise NotAchievedException("Expected several re-requests of mission item")
        self.end_subtest("fence upload timeouts 1")

    def expect_request_for_item(self, item):
        m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'],
                                blocking=True,
                                timeout=1)
        self.progress("Got (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Did not get ACK or mission request")

        if m.get_type() == "MISSION_ACK":
            raise NotAchievedException("Expected MISSION_REQUEST")

        if m.seq != item.seq:
            raise NotAchievedException("Expected request for seq=%u" % item.seq)

        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("Incorrect target component in MISSION_REQUEST")


    def test_fence_upload_timeouts_2(self, target_system=1, target_component=1):
        self.start_subtest("fence upload timeouts 2")
        self.progress("Telling victim there will be two items coming")
        # avoid a timeout race condition where ArduPilot re-requests a
        # fence point before we receive and respond to the first one.
        # Since ArduPilot has a 1s timeout on re-requesting, This only
        # requires a round-trip delay of 1/speedup seconds to trigger
        # - and that has been seen in practise on Travis
        old_speedup = self.get_parameter("SIM_SPEEDUP")
        self.set_parameter("SIM_SPEEDUP", 1)
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        2,
                                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.progress("Sending item with seq=0")
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            1, # p1 radius
            0, # p2
            0, # p3
            0, # p4
            int(1.1 *1e7), # latitude
            int(1.2 *1e7), # longitude
            33.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.expect_request_for_item(item)
        item.pack(self.mav.mav)
        self.mav.mav.send(item)

        self.progress("Sending item with seq=1")
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            1, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            1, # p1 radius
            0, # p2
            0, # p3
            0, # p4
            int(1.1 *1e7), # latitude
            int(1.2 *1e7), # longitude
            33.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        self.expect_request_for_item(item)

        self.set_parameter("SIM_SPEEDUP", old_speedup)

        self.progress("Now waiting for a timeout")
        tstart = self.get_sim_time()
        rerequest_count = 0
        received_text = False
        received_ack = False
        while True:
            if received_ack and received_text:
                break
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not get expected ack and statustext")
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK', 'STATUSTEXT'],
                                    blocking=True,
                                    timeout=0.1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.progress("Did not receive any messages")
                continue
            if m.get_type() == "MISSION_REQUEST":
                if m.seq != 1:
                    raise NotAchievedException("Received request for invalid seq")
                if m.target_system != self.mav.mav.srcSystem:
                    raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
                if m.target_component != self.mav.mav.srcComponent:
                    raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
                rerequest_count += 1
                self.progress("Valid re-request received.")
                continue
            if m.get_type() == "MISSION_ACK":
                if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    raise NotAchievedException("Wrong mission type")
                if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                    raise NotAchievedException("Wrong result")
                received_ack = True
                continue
            if m.get_type() == "STATUSTEXT":
                if "upload time" in m.text:
                    received_text = True
                continue
        if rerequest_count < 3:
            raise NotAchievedException("Expected several re-requests of mission item")
        self.end_subtest("fence upload timeouts 2")

    def test_fence_upload_timeouts(self, target_system=1, target_component=1):
        self.test_fence_upload_timeouts_1(target_system=target_system,
                                          target_component=target_component)
        self.test_fence_upload_timeouts_2(target_system=target_system,
                                          target_component=target_component)

    def test_poly_fence_compatability_ordering(self, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        here = self.mav.location()
        self.progress("try uploading return point last")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[1, 2, 3, 4, 5, 0])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        self.progress("try uploading return point in middle")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[1, 2, 3, 0, 4, 5])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        self.progress("try closing point in middle")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[0, 1, 2, 5, 3, 4])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        # this is expected to fail as we don't return the closing
        # point correctly until the first is uploaded
        self.progress("try closing point first")
        failed = False
        try:
            self.roundtrip_fence_using_fencepoint_protocol([
                self.offset_location_ne(here, 0, 0), # bl // return point
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
                self.offset_location_ne(here, -50, 20), # closing point
            ], ordering=[5, 0, 1, 2, 3, 4])
        except NotAchievedException as e:
            failed = "got=0.000000 want=" in str(e)
        if not failed:
            raise NotAchievedException("Expected failure, did not get it")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        self.progress("try (almost) reverse order")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[4, 3, 2, 1, 0, 5])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

    def test_poly_fence_compatability(self, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        self.test_poly_fence_compatability_ordering(target_system=target_system, target_component=target_component)

        here = self.mav.location()

        self.progress("Playing with changing point count")
        self.roundtrip_fence_using_fencepoint_protocol(
            [
                self.offset_location_ne(here, 0, 0), # bl // return point
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
                self.offset_location_ne(here, -50, 20), # closing point
                ])
        self.roundtrip_fence_using_fencepoint_protocol(
            [
                self.offset_location_ne(here, 0, 0), # bl // return point
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, -50, 40), # tl,
                self.offset_location_ne(here, -50, 20), # closing point
                ])
        self.roundtrip_fence_using_fencepoint_protocol(
            [
                self.offset_location_ne(here, 0, 0), # bl // return point
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
                self.offset_location_ne(here, -50, 20), # closing point
                ])

    def test_poly_fence_reboot_survivability(self):
        here = self.mav.location()

        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            [
                [ # east
                    self.offset_location_ne(here, -50, 20), # bl
                    self.offset_location_ne(here, 50, 20), # br
                    self.offset_location_ne(here, 50, 40), # tr
                    self.offset_location_ne(here, -50, 40), # tl,
                ], [ # over the top of the vehicle
                    self.offset_location_ne(here, -50, -50), # bl
                    self.offset_location_ne(here, -50, 50), # br
                    self.offset_location_ne(here, 50, 50), # tr
                    self.offset_location_ne(here, 50, -50), # tl,
                ]
            ]
        )
        self.reboot_sitl()
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        downloaded_len = len(downloaded_items)
        if downloaded_len != 8:
            raise NotAchievedException("Items did not survive reboot (want=%u got=%u)" %
                                       (8, downloaded_len))

    def test_poly_fence(self):
        '''test fence-related functions'''
        target_system = 1
        target_component = 1

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        here = self.mav.location()
        self.progress("here: %f %f" % (here.lat, here.lng))
        self.set_parameter("FENCE_ENABLE", 1)
        self.set_parameter("AVOID_ENABLE", 0)

#        self.set_parameter("SIM_SPEEDUP", 1)

        self.test_poly_fence_compatability()

        self.test_fence_upload_timeouts()

        self.test_poly_fence_noarms(target_system=target_system, target_component=target_component)

        self.arm_vehicle()

        self.test_poly_fence_inclusion(here, target_system=target_system, target_component=target_component)
        self.test_poly_fence_exclusion(here, target_system=target_system, target_component=target_component)

        self.disarm_vehicle()

        self.test_poly_fence_reboot_survivability()


    def test_poly_fence_inclusion_overlapping_inclusion_circles(self, here, target_system=1, target_component=1):
        self.start_subtest("Overlapping circular inclusion")
        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            [
                {
                    "radius": 30,
                    "loc": self.offset_location_ne(here, -20, 0),
                },
                {
                    "radius": 30,
                    "loc": self.offset_location_ne(here, 20, 0),
                },
            ])
        self.mavproxy.send("fence list\n")

        self.delay_sim_time(5)
        self.progress("Drive outside top circle")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.delay_sim_time(5)
        self.progress("Drive outside bottom circle")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_inclusion(self, here, target_system=1, target_component=1):
        self.progress("Circle and Polygon inclusion")
        self.test_poly_fence_inclusion_overlapping_inclusion_circles(here, target_system=target_system, target_component=target_component)

        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            [
                [
                    self.offset_location_ne(here, -40, -20), # tl
                    self.offset_location_ne(here, 50, -20), # tr
                    self.offset_location_ne(here, 50, 20), # br
                    self.offset_location_ne(here, -40, 20), # bl,
                ],
                {
                    "radius": 30,
                    "loc": self.offset_location_ne(here, -20, 0),
                },
            ])

        self.delay_sim_time(5)
        self.mavproxy.send("fence list\n")
        self.progress("Drive outside polygon")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.delay_sim_time(5)
        self.progress("Drive outside circle")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            [
                [
                    self.offset_location_ne(here, -20, -25), # tl
                    self.offset_location_ne(here, 50, -25), # tr
                    self.offset_location_ne(here, 50, 15), # br
                    self.offset_location_ne(here, -20, 15), # bl,
                ],
                [
                    self.offset_location_ne(here, 20, -20), # tl
                    self.offset_location_ne(here, -50, -20), # tr
                    self.offset_location_ne(here, -50, 20), # br
                    self.offset_location_ne(here, 20, 20), # bl,
                ],
            ])

        self.delay_sim_time(5)
        self.mavproxy.send("fence list\n")
        self.progress("Drive outside top polygon")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.delay_sim_time(5)
        self.progress("Drive outside bottom polygon")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_exclusion(self, here, target_system=1, target_component=1):

        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            [
                [ # east
                    self.offset_location_ne(here, -50, 20), # bl
                    self.offset_location_ne(here, 50, 20), # br
                    self.offset_location_ne(here, 50, 40), # tr
                    self.offset_location_ne(here, -50, 40), # tl,
                ], [ # west
                    self.offset_location_ne(here, -50, -20), # tl
                    self.offset_location_ne(here, 50, -20), # tr
                    self.offset_location_ne(here, 50, -40), # br
                    self.offset_location_ne(here, -50, -40), # bl,
                ], {
                    "radius": 30,
                    "loc": self.offset_location_ne(here, -60, 0),
                },
            ])
        self.delay_sim_time(5)
        self.mavproxy.send("fence list\n")

        self.progress("Breach eastern boundary")
        fence_middle = self.offset_location_ne(here, 0, 30)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)

        self.progress("delaying - hack to work around manual recovery bug")
        self.delay_sim_time(5)

        self.progress("Breach western boundary")
        fence_middle = self.offset_location_ne(here, 0, -30)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)

        self.progress("delaying - hack to work around manual recovery bug")
        self.delay_sim_time(5)

        self.progress("Breach southern circle")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)


    def drive_smartrtl(self):
        self.change_mode("STEERING")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # drive two sides of a square, make sure we don't go back through
        # the middle of the square
        self.progress("Driving North")
        self.reach_heading_manual(0)
        self.set_rc(3, 2000)
        self.delay_sim_time(5)
        self.set_rc(3, 1000)
        self.wait_groundspeed(0, 1)
        loc = self.mav.location()
        self.progress("Driving East")
        self.set_rc(3, 2000)
        self.reach_heading_manual(90)
        self.set_rc(3, 2000)
        self.delay_sim_time(5)
        self.set_rc(3, 1000)

        self.progress("Entering smartrtl")
        self.change_mode("SMART_RTL")

        self.progress("Ensure we go via intermediate point")
        self.wait_distance_to_location(loc, 0, 5)

        self.progress("Ensure we get home")
        self.wait_distance_to_home(3, 7, timeout=30)

        self.disarm_vehicle()

    def test_motor_test(self):
        '''AKA run-rover-run'''
        magic_throttle_value = 1812
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                     1, # p1 - motor instance
                     mavutil.mavlink.MOTOR_TEST_THROTTLE_PWM, # p2 - throttle type
                     magic_throttle_value, # p3 - throttle
                     5, # p4 - timeout
                     1, # p5 - motor count
                     0, # p6 - test order (see MOTOR_TEST_ORDER)
                     0, # p7
        )
        self.mav.motors_armed_wait()
        self.progress("Waiting for magic throttle value")
        self.wait_servo_channel_value(3, magic_throttle_value)
        self.wait_servo_channel_value(3, self.get_parameter("RC3_TRIM", 5), timeout=10)
        self.wait_disarmed()

    def test_poly_fence_object_avoidance_guided(self, target_system=1, target_component=1):
        if not self.mavproxy_can_do_mision_item_protocols():
            return

        self.test_poly_fence_object_avoidance_guided_pathfinding(
            target_system=target_system,
            target_component=target_component)
        return
        # twosquares is currently disabled because of the requirement to have an inclusion fence (which it doesn't have ATM)
        # self.test_poly_fence_object_avoidance_guided_two_squares(
        #     target_system=target_system,
        #     target_component=target_component)

    def test_poly_fence_object_avoidance_auto(self, target_system=1, target_component=1):
        self.load_fence("rover-path-planning-fence.txt")
        self.load_mission("rover-path-planning-mission.txt")
        self.context_push()
        ex = None
        try:
            self.set_parameter("AVOID_ENABLE", 3)
            self.set_parameter("OA_TYPE", 2)
            self.set_parameter("FENCE_MARGIN", 0) # FIXME: https://github.com/ArduPilot/ardupilot/issues/11601
            self.reboot_sitl()
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_parameter("FENCE_ENABLE", 1)
            self.mavproxy.send("fence list\n")
            # target_loc is copied from the mission file
            target_loc = mavutil.location(40.073799, -105.229156)
            self.wait_location(target_loc, timeout=300)
            # mission has RTL as last item
            self.wait_distance_to_home(3, 7, timeout=300)
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def send_guided_mission_item(self, loc, target_system=1, target_component=1):
        self.mav.mav.mission_item_send (
            target_system,
            target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, # current
            0, # autocontinue
            0, # param1
            0, # param2
            0, # param3
            0, # param4
            loc.lat, # x
            loc.lng, # y
            0 # z
        )

    def test_poly_fence_object_avoidance_guided_pathfinding(self, target_system=1, target_component=1):
        self.load_fence("rover-path-planning-fence.txt")
        self.context_push()
        ex = None
        try:
            self.set_parameter("AVOID_ENABLE", 3)
            self.set_parameter("OA_TYPE", 2)
            self.set_parameter("FENCE_MARGIN", 0) # FIXME: https://github.com/ArduPilot/ardupilot/issues/11601
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_parameter("FENCE_ENABLE", 1)
            self.mavproxy.send("fence list\n")
            target_loc = mavutil.location(40.073800, -105.229172)
            self.send_guided_mission_item(target_loc,
                                          target_system=target_system,
                                          target_component=target_component)
            self.wait_location(target_loc, timeout=300)
            self.do_RTL(timeout=300)
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_wheelencoders(self):
        '''make sure wheel encoders are generally working'''
        ex = None
        try:
            self.set_parameter("WENC_TYPE", 10)
            self.set_parameter("EK3_ENABLE", 1)
            self.set_parameter("AHRS_EKF_TYPE", 3)
            self.reboot_sitl()
            self.change_mode("LOITER")
            self.wait_ready_to_arm()
            self.change_mode("MANUAL")
            self.arm_vehicle()
            self.set_rc(3, 1600)

            m = self.mav.recv_match(type='WHEEL_DISTANCE', blocking=True, timeout=5)
            if m is None:
                raise NotAchievedException("Did not get WHEEL_DISTANCE")

            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > 10:
                    break
                dist_home = self.distance_to_home(use_cached_home=True)
                m = self.mav.messages.get("WHEEL_DISTANCE")
                delta = abs(m.distance[0] - dist_home)
                self.progress("dist-home=%f wheel-distance=%f delta=%f" %
                              (dist_home, m.distance[0], delta))
                if delta > 5:
                    raise NotAchievedException("wheel distance incorrect")
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            self.disarm_vehicle()
            ex = e
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_poly_fence_object_avoidance_guided_two_squares(self, target_system=1, target_component=1):
        self.start_subtest("Ensure we can steer around obstacles in guided mode")
        here = self.mav.location()
        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            [
                [ # east
                    self.offset_location_ne(here, -50, 20), # bl
                    self.offset_location_ne(here, 50, 10), # tl
                    self.offset_location_ne(here, 50, 30), # tr
                    self.offset_location_ne(here, -50, 40), # br,
                ],
                [ # further east (and south
                    self.offset_location_ne(here, -60, 60), # bl
                    self.offset_location_ne(here, 40, 70), # tl
                    self.offset_location_ne(here, 40, 90), # tr
                    self.offset_location_ne(here, -60, 80), # br,
                ],
            ])
        self.mavproxy.send("fence list\n")
        self.context_push()
        ex = None
        try:
            self.set_parameter("AVOID_ENABLE", 3)
            self.set_parameter("OA_TYPE", 2)
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.set_parameter("FENCE_ENABLE", 1)
            self.mavproxy.send("fence list\n")
            self.arm_vehicle()

            self.change_mode("GUIDED")
            target = mavutil.location(40.071382, -105.228340, 0, 0)
            self.send_guided_mission_item(target,
                                          target_system=target_system,
                                          target_component=target_component)
            self.wait_location(target, timeout=300)
            self.do_RTL()
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_poly_fence_avoidance_dont_breach_exclusion(self, target_system=1, target_component=1):
        self.start_subtest("Ensure we stop before breaching an exclusion fence")
        here = self.mav.location()
        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            [
                [ # east
                    self.offset_location_ne(here, -50, 20), # bl
                    self.offset_location_ne(here, 50, 20), # br
                    self.offset_location_ne(here, 50, 40), # tr
                    self.offset_location_ne(here, -50, 40), # tl,
                ], [ # west
                    self.offset_location_ne(here, -50, -20), # tl
                    self.offset_location_ne(here, 50, -20), # tr
                    self.offset_location_ne(here, 50, -40), # br
                    self.offset_location_ne(here, -50, -40), # bl,
                ], {
                    "radius": 30,
                    "loc": self.offset_location_ne(here, -60, 0),
                },
            ])
        self.mavproxy.send("fence list\n")
        self.set_parameter("FENCE_ENABLE", 1)
        self.set_parameter("AVOID_ENABLE", 3)
        fence_middle = self.offset_location_ne(here, 0, 30)
        # FIXME: this might be nowhere near "here"!
        expected_stopping_point = mavutil.location(40.0713376, -105.2295738, 0, 0)
        self.drive_somewhere_stop_at_boundary(
            fence_middle,
            expected_stopping_point,
            target_system=target_system,
            target_component=target_component,
            expected_distance_epsilon=3)
        self.set_parameter("AVOID_ENABLE", 0)
        self.do_RTL()

    def do_RTL(self, distance_min=3, distance_max=7, timeout=60):
        self.change_mode("RTL")
        self.wait_distance_to_home(distance_min, distance_max, timeout=timeout)

    def test_poly_fence_avoidance(self, target_system=1, target_component=1):
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("MANUAL")
        self.reach_heading_manual(180, turn_right=False)
        self.change_mode("GUIDED")

        self.test_poly_fence_avoidance_dont_breach_exclusion(target_system=target_system, target_component=target_component)

        self.disarm_vehicle()

    def test_poly_fence_object_avoidance_guided_bendy_ruler(self, target_system=1, target_component=1):
        if not self.mavproxy_can_do_mision_item_protocols():
            return
        self.load_fence("rover-path-bendyruler-fence.txt")
        self.context_push()
        ex = None
        try:
            self.set_parameter("AVOID_ENABLE", 3)
            self.set_parameter("OA_TYPE", 1)
            self.set_parameter("OA_LOOKAHEAD", 50)
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_parameter("FENCE_ENABLE", 1)
            self.set_parameter("WP_RADIUS", 5)
            self.mavproxy.send("fence list\n")
            target_loc = mavutil.location(40.071060, -105.227734, 0, 0)
            self.send_guided_mission_item(target_loc,
                                          target_system=target_system,
                                          target_component=target_component)
            # FIXME: we don't get within WP_RADIUS of our target?!
            self.wait_location(target_loc, timeout=300, accuracy=15)
            self.do_RTL(timeout=300)
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.disarm_vehicle()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_poly_fence_object_avoidance_bendy_ruler_easier(self, target_system=1, target_component=1):
        if not self.mavproxy_can_do_mision_item_protocols():
            return
        self.test_poly_fence_object_avoidance_auto_bendy_ruler_easier(target_system=target_system, target_component=target_component)
        self.test_poly_fence_object_avoidance_guided_bendy_ruler_easier(target_system=target_system, target_component=target_component)

    def test_poly_fence_object_avoidance_guided_bendy_ruler_easier(self, target_system=1, target_component=1):
        '''finish-line issue means we can't complete the harder one.  This
        test can go away once we've nailed that one.  The only
        difference here is the target point.
        '''
        if not self.mavproxy_can_do_mision_item_protocols():
            return
        self.load_fence("rover-path-bendyruler-fence.txt")
        self.context_push()
        ex = None
        try:
            self.set_parameter("AVOID_ENABLE", 3)
            self.set_parameter("OA_TYPE", 1)
            self.set_parameter("OA_LOOKAHEAD", 50)
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_parameter("FENCE_ENABLE", 1)
            self.set_parameter("WP_RADIUS", 5)
            self.mavproxy.send("fence list\n")
            target_loc = mavutil.location(40.071260, -105.227000, 0, 0)
            self.send_guided_mission_item(target_loc,
                                          target_system=target_system,
                                          target_component=target_component)
            # FIXME: we don't get within WP_RADIUS of our target?!
            self.wait_location(target_loc, timeout=300, accuracy=15)
            self.do_RTL(timeout=300)
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.disarm_vehicle()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_poly_fence_object_avoidance_auto_bendy_ruler_easier(self, target_system=1, target_component=1):
        '''finish-line issue means we can't complete the harder one.  This
        test can go away once we've nailed that one.  The only
        difference here is the target point.
        '''
        if not self.mavproxy_can_do_mision_item_protocols():
            return

        self.load_fence("rover-path-bendyruler-fence.txt")
        self.load_mission("rover-path-bendyruler-mission-easier.txt")
        self.context_push()
        ex = None
        try:
            self.set_parameter("AVOID_ENABLE", 3)
            self.set_parameter("OA_TYPE", 1)
            self.set_parameter("OA_LOOKAHEAD", 50)
            self.reboot_sitl()
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_parameter("FENCE_ENABLE", 1)
            self.set_parameter("WP_RADIUS", 5)
            self.mavproxy.send("fence list\n")
            target_loc = mavutil.location(40.071260, -105.227000, 0, 0)
            # target_loc is copied from the mission file
            self.wait_location(target_loc, timeout=300)
            # mission has RTL as last item
            self.wait_distance_to_home(3, 7, timeout=300)
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.disarm_vehicle()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_poly_fence_object_avoidance(self, target_system=1, target_component=1):
        if not self.mavproxy_can_do_mision_item_protocols():
            return

        self.test_poly_fence_object_avoidance_auto(
            target_system=target_system,
            target_component=target_component)
        self.test_poly_fence_object_avoidance_guided(
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_object_avoidance_bendy_ruler(self, target_system=1, target_component=1):
        if not self.mavproxy_can_do_mision_item_protocols():
            return
        # bendy Ruler isn't as flexible as Dijkstra for planning, so
        # it gets a simpler test:
        self.test_poly_fence_object_avoidance_guided_bendy_ruler(
            target_system=target_system,
            target_component=target_component,
        )

    def script_example_source_path(self, scriptname):
        return os.path.join(self.rootdir(), "libraries", "AP_Scripting", "examples", scriptname)

    def script_test_source_path(self, scriptname):
        return os.path.join(self.rootdir(), "libraries", "AP_Scripting", "tests", scriptname)

    def installed_script_path(self, scriptname):
        return os.path.join("scripts", scriptname)

    def install_script(self, source, scriptname):
        dest = self.installed_script_path(scriptname)
        destdir = os.path.dirname(dest)
        if not os.path.exists(destdir):
            os.mkdir(destdir)
        self.progress("Copying (%s) to (%s)" % (source, dest))
        shutil.copy(source, dest)

    def install_example_script(self, scriptname):
        source = self.script_example_source_path(scriptname)
        self.install_script(source, scriptname)

    def install_test_script(self, scriptname):
        source = self.script_test_source_path(scriptname)
        self.install_script(source, scriptname)

    def remove_example_script(self, scriptname):
        dest = self.installed_script_path(scriptname)
        try:
            os.unlink(dest)
        except IOError:
            pass
        except OSError:
            pass

    def test_scripting_simple_loop(self):
        self.start_subtest("Scripting simple loop")
        ex = None
        example_script = "simple_loop.lua"
        messages = []
        def my_message_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            messages.append(message)
        self.install_message_hook(my_message_hook)
        try:
            self.set_parameter("SCR_ENABLE", 1)
            self.install_example_script(example_script)
            self.reboot_sitl()
            self.delay_sim_time(10)
        except Exception as e:
            ex = e
        self.remove_example_script(example_script)
        self.reboot_sitl()

        self.remove_message_hook(my_message_hook)

        if ex is not None:
            raise ex

        # check all messages to see if we got our message
        count = 0
        for m in messages:
            if "hello, world" in m.text:
                count += 1
        self.progress("Got %u hellos" % count)
        if count < 3:
            raise NotAchievedException("Expected at least three hellos")

    def test_scripting_internal_test(self):
        self.start_subtest("Scripting internal test")
        ex = None
        test_scripts = ["scripting_test.lua","math.lua","strings.lua"]
        success_text = ["Internal tests passed","Math tests passed","String tests passed"]

        messages = []
        def my_message_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            messages.append(message)
        self.install_message_hook(my_message_hook)
        try:
            self.set_parameter("SCR_ENABLE", 1)
            self.set_parameter("SCR_HEAP_SIZE", 1024000)
            self.set_parameter("SCR_VM_I_COUNT", 1000000)

            for script in test_scripts:
                self.install_test_script(script)
                self.reboot_sitl()
                self.delay_sim_time(10)
                self.remove_example_script(script)

        except Exception as e:
            ex = e
        self.reboot_sitl()

        self.remove_message_hook(my_message_hook)

        if ex is not None:
            raise ex

        # check all messages to see if we got our message
        success = True
        for text in success_text:
            script_success = False
            for m in messages:
                if text in m.text:
                    script_success = True
            success = script_success and success
        self.progress("Success")
        if not success :
            raise NotAchievedException("Scripting internal test failed")

    def test_scripting_hello_world(self):
        self.start_subtest("Scripting hello world")
        ex = None
        example_script = "hello_world.lua"
        messages = []
        def my_message_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            messages.append(message)
        self.install_message_hook(my_message_hook)
        try:
            self.set_parameter("SCR_ENABLE", 1)
            self.install_example_script(example_script)
            self.reboot_sitl()
        except Exception as e:
            ex = e
        self.remove_example_script(example_script)
        self.reboot_sitl()

        self.remove_message_hook(my_message_hook)

        if ex is not None:
            raise ex

        # check all messages to see if we got our message
        for m in messages:
            if "hello, world" in m.text:
                return # success!
        raise NotAchievedException("Did not get expected text")

    def test_scripting_steering_and_throttle(self):
        self.start_subtest("Scripting square")
        ex = None
        example_script = "rover-set-steering-and-throttle.lua"
        try:
            self.install_example_script(example_script)
            self.set_parameter("SCR_ENABLE", 1)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_rc(6, 2000)
            tstart = self.get_sim_time()
            while not self.mode_is("HOLD"):
                if self.get_sim_time_cached() - tstart > 30:
                    raise NotAchievedException("Did not move to hold")
                m = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=1)
                if m is not None:
                    self.progress("Current speed: %f" % m.groundspeed)
            self.disarm_vehicle()
            self.reboot_sitl()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            self.disarm_vehicle()
            ex = e
        self.remove_example_script(example_script)
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def test_scripting(self):
        self.test_scripting_hello_world()
        self.test_scripting_simple_loop()
        self.test_scripting_internal_test()

    def test_mission_frame(self, frame, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                           target_system=target_system,
                           target_component=target_component)
        items = [
            # first item is ignored for missions
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 *1e7), # latitude
                int(1.0000 *1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        ]

        self.check_mission_upload_download(items)

    def test_mission_frames(self, target_system=1, target_component=1):
        for frame in (mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL):
            self.test_mission_frame(frame,
                                    target_system=1,
                                    target_component=1)

    def mavlink_time_boot_ms(self):
        '''returns a time suitable for putting into the time_boot_ms entry in mavlink packets'''
        return int(time.time() * 1000000)

    def mavlink_time_boot_us(self):
        '''returns a time suitable for putting into the time_boot_ms entry in mavlink packets'''
        return int(time.time() * 1000000000)

    def ap_proximity_mav_obstacle_distance_send(self, data):
        increment = data.get("increment", 0)
        increment_f = data.get("increment_f", 0.0)
        max_distance = data["max_distance"]
        invalid_distance = max_distance + 1  # per spec
        distances = data["distances"][:]
        distances.extend([invalid_distance] * (72-len(distances)))
        self.mav.mav.obstacle_distance_send(
            self.mavlink_time_boot_us(),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            distances,
            increment,
            data["min_distance"],
            data["max_distance"],
            increment_f,
            data["angle_offset"],
            mavutil.mavlink.MAV_FRAME_BODY_FRD
        );

    def send_obstacle_distances_expect_distance_sensor_messages(self, obstacle_distances_in, expect_distance_sensor_messages):
        self.delay_sim_time(11)  # allow obstacles to time out
        self.do_timesync_roundtrip()
        expect_distance_sensor_messages_copy = expect_distance_sensor_messages[:]
        last_sent = 0
        while True:
            now = self.get_sim_time_cached()
            if now - last_sent > 1:
                self.progress("Sending")
                self.ap_proximity_mav_obstacle_distance_send(obstacle_distances_in)
                last_sent = now
            m = self.mav.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.delay_sim_time(1)
                continue
            orientation = m.orientation
            found = False
            if m.current_distance == m.max_distance:
                # ignored
                continue
            for expected_distance_sensor_message in expect_distance_sensor_messages_copy:
                if expected_distance_sensor_message["orientation"] != orientation:
                    continue
                found = True
                if not expected_distance_sensor_message.get("__found__", False):
                    self.progress("Marking message as found")
                    expected_distance_sensor_message["__found__"] = True
                if (m.current_distance - expected_distance_sensor_message["distance"] > 1):
                    raise NotAchievedException("Bad distance for orient=%u want=%u got=%u" % (orientation, expected_distance_sensor_message["distance"], m.current_distance))
                break
            if not found:
                raise NotAchievedException("Got unexpected DISTANCE_SENSOR message")
            all_found = True
            for expected_distance_sensor_message in expect_distance_sensor_messages_copy:
                if not expected_distance_sensor_message.get("__found__", False):
                    self.progress("message still not found (orient=%u" % expected_distance_sensor_message["orientation"])
                    all_found = False
                    break
            if all_found:
                self.progress("Have now seen all expected messages")
                break

    def ap_proximity_mav(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("PRX_TYPE", 2)  # AP_Proximity_MAV
            self.set_parameter("OA_TYPE", 2)  # dijkstra
            self.set_parameter("OA_DB_OUTPUT", 3)  # send all items
            self.reboot_sitl()

            # 1 laser pointing straight forward:
            self.send_obstacle_distances_expect_distance_sensor_messages(
                {
                    "distances": [ 234 ],
                    "increment_f": 10,
                    "angle_offset": 0.0,
                    "min_distance": 0,
                    "max_distance": 1000, # cm
                }, [
                { "orientation": 0, "distance": 234 },
            ])


            # 5 lasers at front of vehicle, spread over 40 degrees:
            self.send_obstacle_distances_expect_distance_sensor_messages(
                {
                    "distances": [ 111, 222, 333, 444, 555 ],
                    "increment_f": 10,
                    "angle_offset": -20.0,
                    "min_distance": 0,
                    "max_distance": 1000, # cm
                }, [
                { "orientation": 0, "distance": 111 },
            ])

            # lots of dense readings (e.g. vision camera:
            distances = [0] * 72
            for i in range(0, 72):
                distances[i] = 1000 + 10*abs(36-i);

            self.send_obstacle_distances_expect_distance_sensor_messages(
                {
                    "distances": distances,
                    "increment_f": 90/72.0,
                    "angle_offset": -45.0,
                    "min_distance": 0,
                    "max_distance": 2000, # cm
                }, [
                { "orientation": 0, "distance": 1000 },
                { "orientation": 1, "distance": 1190 },
                { "orientation": 7, "distance": 1190 },
            ])

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_send_to_components(self):
        self.progress("Introducing ourselves to the autopilot as a component")
        old_srcSystem = self.mav.mav.srcSystem
        self.mav.mav.srcSystem = 1
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0)
        self.progress("Sending control message")
        self.mav.mav.digicam_control_send(
            1, # target_system
            1, # target_component
            1, # start or keep it up
            1, # zoom_pos
            0, # zoom_step
            0, # focus_lock
            1, # 1 shot or start filming
            17, # command id (de-dupe field)
            0, # extra_param
            0.0, # extra_value
        )
        self.mav.mav.srcSystem = old_srcSystem

        self.progress("Expecting a command long")
        tstart = self.get_sim_time_cached()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 2:
                raise NotAchievedException("Did not receive digicam_control message")
            m = self.mav.recv_match(type='COMMAND_LONG', blocking=True, timeout=0.1)
            self.progress("Message: %s" % str(m))
            if m is None:
                continue
            if m.command != mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL:
                raise NotAchievedException("Did not get correct command")
            if m.param6 != 17:
                raise NotAchievedException("Did not get correct command_id")
            break

    def test_skid_steer(self):
        model = "rover-skid"

        self.customise_SITL_commandline([],
                                        model=model,
                                        defaults_filepath=self.model_defaults_filepath("Rover",model))

        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("get a known heading to avoid worrying about wrap")
        # this is steering-type-two-paddles
        self.set_rc(1, 1400)
        self.set_rc(3, 1500)
        self.wait_heading(90)
        self.progress("straighten up")
        self.set_rc(1, 1500)
        self.set_rc(3, 1500)
        self.progress("steer one way")
        self.set_rc(1, 1600)
        self.set_rc(3, 1400)
        self.wait_heading(120)
        self.progress("steer the other")
        self.set_rc(1, 1400)
        self.set_rc(3, 1600)
        self.wait_heading(60)

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestRover, self).tests()

        ret.extend([
            ("MAVProxy_SetModeUsingSwitch",
             "Set modes via mavproxy switch",
             self.test_setting_modes_via_mavproxy_switch),

            ("MAVProxy_SetModeUsingMode",
             "Set modes via mavproxy mode command",
             self.test_setting_modes_via_mavproxy_mode_command),

            ("ModeSwitch",
             "Set modes via modeswitch",
             self.test_setting_modes_via_modeswitch),

            ("AuxModeSwitch",
             "Set modes via auxswitches",
             self.test_setting_modes_via_auxswitches),

            ("DriveRTL",
             "Drive an RTL Mission", self.drive_rtl_mission),

            ("SmartRTL",
             "Test SmartRTL",
             self.drive_smartrtl),

            ("DriveSquare",
             "Learn/Drive Square with Ch7 option",
             self.drive_square),

            ("DriveMaxRCIN",
             "Drive rover at max RC inputs",
             self.drive_max_rcin),

            ("DriveMission",
             "Drive Mission %s" % "rover1.txt",
             lambda: self.drive_mission("rover1.txt")),

            # disabled due to frequent failures in travis. This test needs re-writing
            # ("Drive Brake", self.drive_brake),

            ("GetBanner", "Get Banner", self.do_get_banner),

            ("DO_SET_MODE",
             "Set mode via MAV_COMMAND_DO_SET_MODE",
             self.test_do_set_mode_via_command_long),

            ("MAVProxy_DO_SET_MODE",
            "Set mode via MAV_COMMAND_DO_SET_MODE with MAVProxy",
             self.test_mavproxy_do_set_mode_via_command_long),

            ("ServoRelayEvents",
             "Test ServoRelayEvents",
             self.test_servorelayevents),

            ("RCOverrides", "Test RC overrides", self.test_rc_overrides),

            ("RCOverridesCancel", "Test RC overrides Cancel", self.test_rc_override_cancel),

            ("MANUAL_CONTROL", "Test mavlink MANUAL_CONTROL", self.test_manual_control),

            ("Sprayer", "Test Sprayer", self.test_sprayer),

            ("AC_Avoidance",
             "Test AC Avoidance switch",
             self.drive_fence_ac_avoidance),

            ("CameraMission",
             "Test Camera Mission Items",
             self.test_camera_mission_items),

            # Gripper test
            ("Gripper",
             "Test gripper",
             self.test_gripper),

            ("GripperMission",
             "Test Gripper Mission Items",
             self.test_gripper_mission),

            ("SET_MESSAGE_INTERVAL",
             "Test MAV_CMD_SET_MESSAGE_INTERVAL",
             self.test_set_message_interval),

            ("REQUEST_MESSAGE",
             "Test MAV_CMD_REQUEST_MESSAGE",
             self.test_request_message),

            ("SYSID_ENFORCE",
             "Test enforcement of SYSID_MYGCS",
             self.test_sysid_enforce),

            ("Button",
             "Test Buttons",
             self.test_button),

            ("Rally",
             "Test Rally Points",
             self.test_rally_points),

            ("Offboard",
             "Test Offboard Control",
             self.test_offboard),

            ("GCSFence",
             "Upload and download of fence",
             self.test_gcs_fence),

            ("GCSRally",
             "Upload and download of rally",
             self.test_gcs_rally),

            ("GCSMission",
             "Upload and download of mission",
             self.test_gcs_mission),

            ("MotorTest",
             "Motor Test triggered via mavlink",
             self.test_motor_test),

            ("WheelEncoders",
             "Ensure SITL wheel encoders work",
             self.test_wheelencoders),

            ("DataFlashOverMAVLink",
             "Test DataFlash over MAVLink",
             self.test_dataflash_over_mavlink),

            ("DataFlashSITL",
             "Test DataFlash SITL backend",
             self.test_dataflash_sitl),

            ("SkidSteer",
             "Check skid-steering",
             self.test_skid_steer),

            ("PolyFence",
             "PolyFence tests",
             self.test_poly_fence),

            ("PolyFenceAvoidance",
             "PolyFence avoidance tests",
             self.test_poly_fence_avoidance),

            ("PolyFenceObjectAvoidance",
             "PolyFence object avoidance tests",
             self.test_poly_fence_object_avoidance),

            ("PolyFenceObjectAvoidanceBendyRuler",
             "PolyFence object avoidance tests - bendy ruler",
             self.test_poly_fence_object_avoidance_bendy_ruler),

            ("SendToComponents",
             "Test ArduPilot send_to_components function",
             self.test_send_to_components),

            ("PolyFenceObjectAvoidanceBendyRulerEasier",
             "PolyFence object avoidance tests - easier bendy ruler test",
             self.test_poly_fence_object_avoidance_bendy_ruler_easier),

            ("Scripting",
             "Scripting test",
             self.test_scripting),

            ("ScriptingSteeringAndThrottle",
             "Scripting test - steering and throttle",
             self.test_scripting_steering_and_throttle),

            ("MissionFrames",
             "Upload/Download of items in different frames",
             self.test_mission_frames),

            ("AccelCal",
             "Accelerometer Calibration testing",
             self.accelcal),

            ("AP_Proximity_MAV",
             "Test MAV proximity backend",
             self.ap_proximity_mav),

            ("LogUpload",
             "Upload logs",
             self.log_upload),
            ])
        return ret

    def disabled_tests(self):
        return {
            "DriveMaxRCIN": "currently triggers Arithmetic Exception",
        }

    def rc_defaults(self):
        ret = super(AutoTestRover, self).rc_defaults()
        ret[3] = 1500
        ret[8] = 1800
        return ret

    def default_mode(self):
        return 'MANUAL'
