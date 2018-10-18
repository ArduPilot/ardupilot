#!/usr/bin/env python

# Drive APMrover2 in SITL
from __future__ import print_function

import os
import pexpect
import time

from common import AutoTest

from common import MsgRcvTimeoutException
from common import NotAchievedException
from common import PreconditionFailedException

from pysim import util

from pymavlink import mavutil

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

# HOME = mavutil.location(-35.362938, 149.165085, 584, 270)
HOME = mavutil.location(40.071374969556928,
                        -105.22978898137808,
                        1583.702759,
                        246)


class AutoTestRover(AutoTest):
    def __init__(self,
                 binary,
                 valgrind=False,
                 gdb=False,
                 speedup=8,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 breakpoints=[],
                 **kwargs):
        super(AutoTestRover, self).__init__(**kwargs)
        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver
        self.breakpoints = breakpoints

        self.home = "%f,%f,%u,%u" % (HOME.lat,
                                     HOME.lng,
                                     HOME.alt,
                                     HOME.heading)
        self.homeloc = None
        self.speedup = speedup

        self.sitl = None
        self.hasInit = False

        self.log_name = "APMrover2"

    def init(self):
        if self.frame is None:
            self.frame = 'rover'

        self.sitl = util.start_SITL(self.binary,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver,
                                    breakpoints=self.breakpoints,
                                    wipe=True)
        self.mavproxy = util.start_MAVProxy_SITL(
            'APMrover2', options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        self.logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)
        self.try_symlink_tlog()

        self.progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Started simulator")

        self.get_mavlink_connection_going()

        self.hasInit = True

        self.apply_defaultfile_parameters()

        self.progress("Ready to start testing!")

    # def reset_and_arm(self):
    #     """Reset RC, set to MANUAL and arm."""
    #     self.mav.wait_heartbeat()
    #     # ensure all sticks in the middle
    #     self.set_rc_default()
    #     self.mavproxy.send('switch 1\n')
    #     self.mav.wait_heartbeat()
    #     self.disarm_vehicle()
    #     self.mav.wait_heartbeat()
    #     self.arm_vehicle()
    #

    # # TEST RC OVERRIDE
    # # TEST RC OVERRIDE TIMEOUT
    # def test_rtl(self, home, distance_min=5, timeout=250):
    #     """Return, land."""
    #     super(AutotestRover, self).test_rtl(home, distance_min, timeout)
    #
    # def test_mission(self, filename):
    #     """Test a mission from a file."""
    #     self.progress("Test mission %s" % filename)
    #     num_wp = self.load_mission_from_file(filename)
    #     self.mavproxy.send('wp set 1\n')
    #     self.mav.wait_heartbeat()
    #     self.mavproxy.send('switch 4\n')  # auto mode
    #     self.wait_mode('AUTO')
    #     ret = self.wait_waypoint(0, num_wp-1, max_dist=5, timeout=500)
    #
    #     if ret:
    #         self.mavproxy.expect("Mission Complete")
    #     self.mav.wait_heartbeat()
    #     self.wait_mode('HOLD')
    #     self.progress("test: MISSION COMPLETE: passed=%s" % ret)
    #     return ret

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

            self.clear_wp()

            # use LEARNING Mode
            self.mavproxy.send('switch 5\n')
            self.wait_mode('MANUAL')

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
            if num_wp != 6:
                raise NotAchievedException("Did not get 6 waypoints")

            # TODO: actually drive the mission

            self.clear_wp()
        except Exception as e:
            self.progress("Caught exception: %s" % str(e))
            ex = e
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
    #     self.mav.wait_heartbeat()
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
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex:
            raise ex

    #################################################
    # AUTOTEST ALL
    #################################################
    def drive_mission(self, filename):
        """Drive a mission from a file."""
        self.progress("Driving mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.set_rc(3, 1500)
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 4, max_dist=5)
        self.wait_mode('HOLD', timeout=300)
        self.progress("Mission OK")

    def drive_mission_rover1(self):
        self.drive_mission(os.path.join(testdir, "rover1.txt"))

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
        self.mavproxy.send("mode STEERING\n")
        self.wait_mode('STEERING')
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

        distance_without_brakes = self.drive_brake_get_stopping_distance(15)

        # brakes on:
        self.set_parameter('ATC_BRAKE', 1)
        distance_with_brakes = self.drive_brake_get_stopping_distance(15)
        # revert state:
        self.set_parameter('ATC_BRAKE', old_using_brake)
        self.set_parameter('CRUISE_SPEED', old_cruise_speed)

        delta = distance_without_brakes - distance_with_brakes
        if delta < distance_without_brakes * 0.05:  # 5% isn't asking for much
            raise NotAchievedException("""
Brakes have negligible effect (with=%0.2fm without=%0.2fm delta=%0.2fm)
""" % (distance_with_brakes,
                   distance_without_brakes,
                   delta))

        self.progress(
            "Brakes work (with=%0.2fm without=%0.2fm delta=%0.2fm)" %
            (distance_with_brakes, distance_without_brakes, delta))

    def drive_rtl_mission(self):
        mission_filepath = os.path.join(testdir,
                                        "ArduRover-Missions",
                                        "rtl.txt")
        self.mavproxy.send('wp load %s\n' % mission_filepath)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.set_rc(3, 1500)
        self.wait_mode('AUTO')
        self.mavproxy.expect('Executing RTL')

        m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
                                blocking=True,
                                timeout=0.1)
        if m is None:
            raise MsgRcvTimeoutException(
                "Did not receive NAV_CONTROLLER_OUTPUT message")

        wp_dist_min = 5
        if m.wp_dist < wp_dist_min:
            raise PreconditionFailedException(
                "Did not start at least %u metres from destination" %
                (wp_dist_min))

        self.progress("NAV_CONTROLLER_OUTPUT.wp_dist looks good (%u >= %u)" %
                      (m.wp_dist, wp_dist_min,))

        self.wait_mode('HOLD', timeout=600) # balancebot can take a long time!

        pos = self.mav.location()
        home_distance = self.get_distance(HOME, pos)
        home_distance_max = 5
        if home_distance > home_distance_max:
            raise NotAchievedException(
                "Did not get home (%u metres distant > %u)" %
                (home_distance, home_distance_max))
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')
        self.progress("RTL Mission OK")

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

    def test_rc_overrides(self):
        self.context_push();
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
            while True:
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
                want_speed = 2.0
                print("Speed=%f want=<%f" % (m.groundspeed, want_speed))
                if m.groundspeed < want_speed:
                    break

            # now override to stop - but set the switch on the RC
            # transmitter to deny overrides; this should send the
            # speed back up to 5 metres/second:
            self.set_rc(12, 1000)

            throttle_override = 1500
            while True:
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

        self.context_pop();
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def autotest(self):
        """Autotest APMrover2 in SITL."""
        self.check_test_syntax(test_file=os.path.realpath(__file__))
        if not self.hasInit:
            self.init()
        self.progress("Started simulator")

        self.fail_list = []
        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s" %
                          self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(8, 1800)
            self.progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)

            self.mavproxy.send('switch 6\n')  # Manual mode
            self.wait_mode('MANUAL')

            self.wait_ready_to_arm()
            self.run_test("Arm features", self.test_arm_feature)

            self.run_test("Set modes via mavproxy switch",
                          self.test_setting_modes_via_mavproxy_switch)

            self.run_test("Set modes via mavproxy mode command",
                          self.test_setting_modes_via_mavproxy_mode_command)

            self.run_test("Set modes via modeswitch",
                          self.test_setting_modes_via_modeswitch)

            self.run_test("Set modes via auxswitches",
                          self.test_setting_modes_via_auxswitches)

            self.arm_vehicle()

            self.run_test("Drive an RTL Mission", self.drive_rtl_mission)

            self.run_test("Learn/Drive Square with Ch7 option",
                          self.drive_square)

            self.run_test("Drive Mission %s" % "rover1.txt",
                          self.drive_mission_rover1)

            # disabled due to frequent failures in travis. This test needs re-writing
            # self.run_test("Drive Brake", self.drive_brake)

            self.run_test("Disarm Vehicle", self.disarm_vehicle)

            self.run_test("Get Banner", self.do_get_banner)

            self.run_test("Get Capabilities",
                          self.do_get_autopilot_capabilities)

            self.run_test("Set mode via MAV_COMMAND_DO_SET_MODE",
                          lambda: self.do_set_mode_via_command_long("HOLD"))
            self.mavproxy.send('switch 6\n')  # Manual mode
            self.wait_mode('MANUAL')
            self.run_test("Set mode via MAV_COMMAND_DO_SET_MODE with MAVProxy",
                          lambda: self.mavproxy_do_set_mode_via_command_long("HOLD"))

            self.run_test("Test ServoRelayEvents",
                          self.test_servorelayevents)

            self.run_test("Test RC overrides", self.test_rc_overrides)

            self.run_test("Test Sprayer", self.test_sprayer)

            self.run_test("Download logs", lambda:
                          self.log_download(
                              self.buildlogs_path("APMrover2-log.bin")))
    #        if not drive_left_circuit(self):
    #            self.progress("Failed left circuit")
    #            failed = True
    #        if not drive_RTL(self):
    #            self.progress("Failed RTL")
    #            failed = True

        except pexpect.TIMEOUT:
            self.progress("Failed with timeout")
            self.fail_list.append(("*timeout*", None))

        self.close()

        if len(self.fail_list):
            self.progress("FAILED STEPS: %s" % self.fail_list)
            return False
        return True
