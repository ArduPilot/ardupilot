#!/usr/bin/env python

# Drive APMrover2 in SITL
from __future__ import print_function

import os
import pexpect
import time

from common import AutoTest

from common import AutoTestTimeoutException
from common import MsgRcvTimeoutException
from common import NotAchievedException
from common import PreconditionFailedException

from pysim import util

from pymavlink import mavutil

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

SITL_START_LOCATION = mavutil.location(40.071374969556928,
                                       -105.22978898137808,
                                       1583.702759,
                                       246)


class AutoTestRover(AutoTest):

    def log_name(self):
        return "APMrover2"

    def test_filepath(self):
         return os.path.realpath(__file__)

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
            self.set_parameter("RC8_OPTION", 58)

            self.mavproxy.send('switch 5\n')
            self.wait_mode('MANUAL')

            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.clear_wp()

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
                os.path.join(testdir, "rover-ch7_mission.txt"))
            expected = 7 # home + 6 toggled in
            if num_wp != expected:
                raise NotAchievedException("Did not get %u waypoints; got %u" %
                                           (expected, num_wp))

            # TODO: actually drive the mission

            self.clear_wp()
        except Exception as e:
            self.progress("Caught exception: %s" % str(e))
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
            self.fetch_parameters()
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
            self.progress("Caught exception: %s" % str(e))
            ex = e
        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
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
        self.wait_mode('HOLD', timeout=300)
        self.disarm_vehicle()
        self.progress("Mission OK")

    def test_gripper_mission(self):
        self.load_mission("rover-gripper-mission.txt")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mavproxy.expect("Gripper Grabbed")
        self.mavproxy.expect("Gripper Released")
        self.wait_mode("HOLD")
        self.disarm_vehicle()

    def do_get_banner(self):
        self.mavproxy.send("long DO_SEND_BANNER 1\n")
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

    def drive_rtl_mission(self):
        self.wait_ready_to_arm()
        self.arm_vehicle()

        mission_filepath = os.path.join("ArduRover-Missions", "rtl.txt")
        self.load_mission(mission_filepath)
        self.change_mode("AUTO")
        self.mavproxy.expect('Mission: 3 RTL')

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

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 600:
                raise NotAchievedException("Did not get home")
            self.progress("Distance home: %f (mode=%s)" %
                          (self.distance_to_home(), self.mav.flightmode))
            if self.mode_is('HOLD') or self.mode_is('LOITER'): # loiter for balancebot
                break

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


    def wait_distance_home_gt(self, distance, timeout=60):
        home_distance = None
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            # m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            distance_home = self.distance_to_home(use_cached_home=True)
            self.progress("distance_home=%f want=%f" % (distance_home, distance))
            if distance_home > distance:
                return
            self.drain_mav()
        raise NotAchievedException("Failed to get %fm from home (now=%f)" %
                                   (distance, home_distance))

    def drive_fence_ac_avoidance(self):
        self.context_push()
        ex = None
        try:
            avoid_filepath = os.path.join(self.mission_directory(),
                                          "rover-fence-ac-avoid.txt")
            self.mavproxy.send("fence load %s\n" % avoid_filepath)
            self.mavproxy.expect("Loaded 6 geo-fence")
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
            self.wait_distance_home_gt(25)
            self.change_mode("RTL")
            self.mavproxy.expect("APM: Reached destination")
            # now enable avoidance and make sure we can't:
            self.set_rc(10, 2000)
            self.change_mode("ACRO")
            self.wait_groundspeed(0, 0.7, timeout=60)
            # watch for speed zero
            self.wait_groundspeed(0, 0.2, timeout=120)

        except Exception as e:
            self.progress("Caught exception: %s" % str(e))
            ex = e
        self.context_pop()
        self.mavproxy.send("fence clear\n")
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex:
            raise ex

    def test_servorelayevents(self):
        self.mavproxy.send("relay set 0 0\n")
        off = self.get_parameter("SIM_PIN_MASK")
        self.mavproxy.send("relay set 0 1\n")
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
        self.mavproxy.send('rc 3 %u\n' % normal_rc_throttle)
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not get rc change")
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            if m.chan3_raw == normal_rc_throttle:
                break

        self.progress("Set override with RC_CHANNELS_OVERRIDE")
        tstart = self.get_sim_time_cached()
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
        tstart = self.get_sim_time_cached()
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
            self.mavproxy.send('rc 3 1500\n')  # throttle at zero
            self.arm_vehicle()
            # start moving forward a little:
            normal_rc_throttle = 1700
            self.mavproxy.send('rc 3 %u\n' % normal_rc_throttle)
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
                    raise AutoTestTimeoutException("Did not stop")
                print("Sending throttle of %u" % (throttle_override,))
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
                print("Speed=%f want=>%f" % (m.groundspeed, want_speed))

                if m.groundspeed > want_speed:
                    break

            # re-enable RC overrides
            self.set_rc(12, 2000)

            # check we revert to normal RC inputs when gcs overrides cease:
            self.progress("Waiting for RC to revert to normal RC input")
            while True:
                m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
                print("%s" % m)
                if m.chan3_raw == normal_rc_throttle:
                    break

        except Exception as e:
            self.progress("Exception caught")
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
            self.mavproxy.send('rc 3 %u\n' % normal_rc_throttle)
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
                print("Sending normalized throttle of %u" % (throttle_override_normalized,))
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
            while True:
                m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
                print("%s" % m)
                if m.chan3_raw == normal_rc_throttle:
                    break

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
                pass
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
                pass
            self.mav.mav.srcSystem = old_srcSystem
            if comp_arm_exception is not None:
                raise comp_arm_exception

        except Exception as e:
            self.progress("Exception caught")
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def drain_mav_seconds(self, seconds):
        tstart = self.get_sim_time_cached()
        while self.get_sim_time_cached() - tstart < seconds:
            self.drain_mav();
            self.delay_sim_time(0.5)

    def test_button(self):
        self.set_parameter("SIM_PIN_MASK", 0)
        self.set_parameter("BTN_ENABLE", 1)
        btn = 2
        pin = 3
        self.set_parameter("BTN_PIN%u" % btn, pin)
        self.drain_mav()
        m = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        self.progress("m: %s" % str(m))
        if m is None:
            raise NotAchievedException("Did not get BUTTON_CHANGE event")
        mask = 1<<btn
        if m.state & mask:
            raise NotAchievedException("Bit incorrectly set in mask (got=%u dontwant=%u)" % (m.state, mask))
        # SITL instantly reverts the pin to its old value
        m2 = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        self.progress("m2: %s" % str(m2))
        if m2 is None:
            raise NotAchievedException("Did not get repeat message")
        # wait for messages to stop coming:
        self.drain_mav_seconds(15)

        self.set_parameter("SIM_PIN_MASK", 0)
        m3 = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        self.progress("m3: %s" % str(m3))
        if m3 is None:
            raise NotAchievedException("Did not get new message")
        if m.last_change_ms == m3.last_change_ms:
            raise NotAchievedException("last_change_ms same as first message")
        if m3.state != 0:
            raise NotAchievedException("Didn't get expected mask back in message (mask=0 state=%u" % (m3.state))

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

    def test_gcs_fence(self):
        self.progress("Testing FENCE_POINT protocol")
        self.set_parameter("FENCE_TOTAL", 1)
        target_system = 1
        target_component = 1

        lat = 1.2345
        lng = 5.4321
        self.mav.mav.fence_point_send(target_system,
                                      target_component,
                                      0,
                                      1,
                                      lat,
                                      lng)
        self.progress("Requesting fence return point")
        self.mav.mav.fence_fetch_point_send(target_system,
                                            target_component,
                                            0)
        m = self.mav.recv_match(type="FENCE_POINT", blocking=True, timeout=1)
        print("m: %s" % str(m))
        if m is None:
            raise NotAchievedException("Did not get fence return point back")
        if abs(m.lat - lat) > 0.000001:
            raise NotAchievedException("Did not get correct lat in fencepoint: got=%f want=%f" % (m.lat, lat))
        if abs(m.lng - lng) > 0.000001:
            raise NotAchievedException("Did not get correct lng in fencepoint: got=%f want=%f" % (m.lng, lng))

        self.progress("Now testing a different value")
        lat = 2.345
        lng = 4.321
        self.mav.mav.fence_point_send(target_system,
                                      target_component,
                                      0,
                                      1,
                                      lat,
                                      lng)
        self.progress("Requesting fence return point")
        self.mav.mav.fence_fetch_point_send(target_system,
                                            target_component,
                                            0)
        m = self.mav.recv_match(type="FENCE_POINT", blocking=True, timeout=1)
        print("m: %s" % str(m))
        if abs(m.lat - lat) > 0.000001:
            raise NotAchievedException("Did not get correct lat in fencepoint: got=%f want=%f" % (m.lat, lat))
        if abs(m.lng - lng) > 0.000001:
            raise NotAchievedException("Did not get correct lng in fencepoint: got=%f want=%f" % (m.lng, lng))

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
            if self.mode_is("HOLD", cached=True):
                break

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
#            mc = self.mav.messages.get("MISSION_CURRENT", None)
            mc = self.mav.recv_match(type="MISSION_CURRENT", blocking=False)
            if mc is not None:
                print("%s" % str(mc))
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
        mav.mav.mission_request_list_send(target_system,
                                          target_component,
                                          mission_type)
        m = mav.recv_match(type="MISSION_COUNT", blocking=True, timeout=1)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_COUNT on link")
        if m.count != expected_count:
            raise NotAchievedException("Bad count received (want=%u got=%u)" %
                                       (expected_count, m.count))

    def get_mission_item_on_link(self, item, mav, target_system, target_component, mission_type):
        mav.mav.mission_request_int_send(target_system,
                                         target_component,
                                         item,
                                         mission_type)
        m = mav.recv_match(type='MISSION_ITEM_INT',
                           blocking=True,
                           timeout=1)
        if m is None:
            raise NotAchievedException("Did not receive mission item int")
        if m.target_system != mav.mav.srcSystem:
            raise NotAchievedException("Unexpected target system %u want=%u" %
                                       (m.target_system, mav.mav.srcSystem))
        if m.target_component != mav.mav.srcComponent:
            raise NotAchievedException("Unexpected target component %u want=%u" %
                                       (m.target_component, mav.mav.srcComponent))
        return m

    def clear_mission(self, mission_type, target_system, target_component):
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        0,
                                        mission_type)
        m = self.mav.recv_match(type='MISSION_ACK',
                                blocking=True,
                                timeout=1)
        if m is None:
            raise NotAchievedException("Expected ACK for clearing mission")
        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("ACK not targetted at correct system want=%u got=%u" %
                                       (self.mav.mav.srcSystem, m.target_system))
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("ACK not targetted at correct component want=%u got=%u" %
                                       (self.mav.mav.srcComponent, m.target_component))
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Expected MAV_MISSION_ACCEPTED got %u" %
                                       (m.type))

    def assert_receive_mission_item_request(self, mission_type, seq):
        self.progress("Expecting request for item %u" % seq)
        m = self.mav.recv_match(type='MISSION_REQUEST',
                                blocking=True,
                                timeout=1)
        if m is None:
            raise NotAchievedException("Did not get item request")
        if m.mission_type != mission_type:
            raise NotAchievedException("Incorrect mission type (wanted=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.seq != seq:
            raise NotAchievedException("Unexpected sequence number (want=%u got=%u)" % (seq, m.seq))
        self.progress("Received item request OK")

    def assert_receive_mission_ack(self, mission_type,
                                   want_type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
                                   target_system=None,
                                   target_component=None):
        if target_system is None:
            target_system = self.mav.mav.srcSystem
        if target_component is None:
            target_component = self.mav.mav.srcComponent
        self.progress("Expecting mission ack")
        m = self.mav.recv_match(type='MISSION_ACK',
                                blocking=True,
                                timeout=1)
        if m is None:
            raise NotAchievedException("Expected mission ACK")
        if m.target_system != target_system:
            raise NotAchievedException("ACK not targetted at correct system want=%u got=%u" %
                                       (self.mav.mav.srcSystem, m.target_system))
        if m.target_component != target_component:
            raise NotAchievedException("ACK not targetted at correct component")
        if m.mission_type != mission_type:
            raise NotAchievedException("Unexpected mission type %u want=%u" %
                                       (m.mission_type, mission_type))
        if m.type != want_type:
            raise NotAchievedException("Expected ack type got %u got %u" %
                                       (want_type, m.type))

    def test_gcs_rally(self):
        target_system = 1
        target_component = 1
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
        try:
            item1_lat = int(2.0000 *1e7)
            items = [
                self.mav.mav.mission_item_int_encode(
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
                    int(1.0000 *1e7), # latitude
                    int(1.0000 *1e7), # longitude
                    31.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
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
                    item1_lat, # latitude
                    int(2.0000 *1e7), # longitude
                    32.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
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
            self.mavproxy.expect("upload timeout")

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
            mav2 = mavutil.mavlink_connection("tcp:localhost:5763",
                                              robust_parsing=True,
                                              source_system = 7,
                                              source_component=7)
            expected_count = 3
            self.progress("Assert mision count on new link")
            self.assert_mission_count_on_link(mav2, expected_count, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Assert mission count on original link")
            self.assert_mission_count_on_link(self.mav, expected_count, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Get first item on new link")
            m2 = self.get_mission_item_on_link(2, mav2, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Get first item on original link")
            m = self.get_mission_item_on_link(2, self.mav, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if m2.x != m.x:
                raise NotAchievedException("mission items do not match (%d vs %d)" % (m2.x, m.x))
            self.start_subtest("Should enforce items come from correct GCS")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)
            self.progress("Attempting to upload from bad sysid")
            old_sysid = self.mav.mav.srcSystem
            self.mav.mav.srcSystem = 17
            items[0].pack(self.mav.mav)
            self.mav.mav.send(items[0])
            self.mav.mav.srcSystem = old_sysid
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED,
                                            target_system=17)
            self.progress("Sending from correct sysid")
            items[0].pack(self.mav.mav)
            self.mav.mav.send(items[0])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Attempt to send item on different link to that which we are sending requests on")
            self.progress("Sending count")
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
            mav2.mav.send(items[0])
            mav2.mav.srcSystem = old_mav2_system
            mav2.mav.srcComponent = old_mav2_component
            # we continue to receive requests on the original link:
            m = self.mav.recv_match(type='MISSION_REQUEST',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                raise NotAchievedException("Did not get mission request")
            if m.seq != 1:
                raise NotAchievedException("Unexpected sequence number (expected=%u got=%u)" % (1, m.seq))
            items[1].pack(self.mav.mav)
            self.mav.mav.send(items[1])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Upload mission and rally points at same time")
            self.progress("Sending rally count")
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
            wp = self.mav.mav.mission_item_int_send(
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
            self.mav.mav.send(items[0])
            self.progress("Expecting request for rally item 1")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Answering request for rally point 1")
            items[1].pack(self.mav.mav)
            self.mav.mav.send(items[1])
            self.progress("Expecting request for rally item 2")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)

            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.mav.mav.send(items[2])
            self.progress("Expecting mission ack for rally")
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.progress("Answering request for waypoints item 1")
            wp = self.mav.mav.mission_item_int_send(
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
                               target_system,
                               target_component)
            self.progress("Should not be able to set items completely past the waypoint count")
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                17,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("Should not be able to set items overlapping the waypoint count")
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                0,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("try to overwrite items 1 and 2")
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                1,
                2,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Try shoving up an incorrectly sequenced item")
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
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Try shoving up an incorrectly sequenced item (but within band)")
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
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Now provide correct item")
            item1_latitude = int(1.2345*1e7)
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
                item1_latitude, # latitude
                int(1.2000 *1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)
            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.mav.mav.send(items[2])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("TODO: ensure partial mission write was good")

            self.start_subtest("clear mission types")
            self.assert_mission_count_on_link(self.mav, 3, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 3, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 0, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 3, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_mission_count_on_link(self.mav, 0, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(self.mav, 0, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            self.start_subtest("try sending out-of-range counts")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            230)
            self.assert_receive_mission_ack(230,
                                            want_type=mavutil.mavlink.MAV_MISSION_UNSUPPORTED)
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

            ("DriveSquare",
             "Learn/Drive Square with Ch7 option",
             self.drive_square),

            ("DriveMission",
             "Drive Mission %s" % "rover1.txt",
             lambda: self.drive_mission("rover1.txt")),

            # disabled due to frequent failures in travis. This test needs re-writing
                # ("Drive Brake", self.drive_brake),

            ("GetBanner", "Get Banner", self.do_get_banner),

            ("GetCapabilities",
             "Get Capabilities",
             self.do_get_autopilot_capabilities),

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

            ("DataFlashOverMAVLink",
             "Test DataFlash over MAVLink",
             self.test_dataflash_over_mavlink),

            ("DownLoadLogs", "Download logs", lambda:
             self.log_download(
                 self.buildlogs_path("APMrover2-log.bin"),
                 upload_logs=len(self.fail_list) > 0)),
            ])
        return ret

    def rc_defaults(self):
        ret = super(AutoTestRover, self).rc_defaults()
        ret[3] = 1500
        ret[8] = 1800
        return ret;

    def default_mode(self):
        return 'MANUAL'
