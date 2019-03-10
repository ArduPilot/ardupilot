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

        self.speedup = speedup

        self.sitl = None

        self.log_name = "APMrover2"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def init(self):
        super(AutoTestRover, self).init(os.path.realpath(__file__))
        if self.frame is None:
            self.frame = 'rover'

        self.mavproxy_logfile = self.open_mavproxy_logfile()

        self.sitl = util.start_SITL(self.binary,
                                    model=self.frame,
                                    home=self.sitl_home(),
                                    speedup=self.speedup,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver,
                                    breakpoints=self.breakpoints,
                                    wipe=True)
        self.mavproxy = util.start_MAVProxy_SITL(
            'APMrover2',
            logfile=self.mavproxy_logfile,
            options=self.mavproxy_options())
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

        self.apply_defaultfile_parameters()

        self.progress("Ready to start testing!")

    def is_rover(self):
        return True

    def get_rudder_channel(self):
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

    def drive_rtl_mission(self):
        self.wait_ready_to_arm()
        self.arm_vehicle()

        mission_filepath = os.path.join("ArduRover-Missions", "rtl.txt")
        self.load_mission(mission_filepath)
        self.change_mode("AUTO")
        self.mavproxy.expect('Executing RTL')

        self.drain_mav();

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

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 600:
                raise NotAchievedException("Did not get home")
            self.progress("Distance home: %f (mode=%s)" %
                          (self.distance_to_home(), self.mav.flightmode))
            if self.mode_is('HOLD'):
                break

        # the EKF doesn't pull us down to 0 speed:
        self.wait_groundspeed(0, 0.5, timeout=600)

        # current Rover blows straight past the home position and ends
        # up ~6m past the home point.
        home_distance = self.distance_to_home()
        home_distance_min = 5.5
        home_distance_max = 6.5
        if home_distance > home_distance_max:
            raise NotAchievedException(
                "Did not stop near home (%f metres distant (%f > want > %f))" %
                (home_distance, home_distance_min, home_distance_max))
        self.disarm_vehicle()
        self.progress("RTL Mission OK (%fm)" % home_distance)


    def wait_distance_home_gt(self, distance, timeout=60):
        home_distance = None
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < timeout:
            # m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if self.distance_to_home() > distance:
                return
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
            self.mav.srcSystem = old_srcSystem
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
            self.mav.srcSystem = old_srcSystem
            if comp_arm_exception is not None:
                raise comp_arm_exception

        except Exception as e:
            self.progress("Exception caught")
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def test_rally_points(self):
        self.load_rally("rover-test-rally.txt")

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
        self.wait_location(loc, accuracy=self.get_parameter("WP_RADIUS"))
        self.disarm_vehicle()

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

            ("SYSID_ENFORCE",
             "Test enforcement of SYSID_MYGCS",
             self.test_sysid_enforce),

            ("Rally",
             "Test Rally Points",
             self.test_rally_points),

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
        ret[3] = 1000
        ret[8] = 1800
        return ret;

    def default_mode(self):
        return 'MANUAL'
