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
                 speedup=10,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 **kwargs):
        super(AutoTestRover, self).__init__(**kwargs)
        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver

        self.home = "%f,%f,%u,%u" % (HOME.lat,
                                     HOME.lng,
                                     HOME.alt,
                                     HOME.heading)
        self.homeloc = None
        self.speedup = speedup
        self.speedup_default = 10

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
                                    wipe=True)
        self.mavproxy = util.start_MAVProxy_SITL(
            'APMrover2', options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % logfile)

        buildlog = self.buildlogs_path("APMrover2-test.tlog")
        self.progress("buildlog=%s" % buildlog)
        if os.path.exists(buildlog):
            os.unlink(buildlog)
        try:
            os.link(logfile, buildlog)
        except Exception:
            pass

        self.progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Started simulator")

        # get a mavlink connection going
        connection_string = '127.0.0.1:19550'
        try:
            self.mav = mavutil.mavlink_connection(connection_string,
                                                  robust_parsing=True)
        except Exception as msg:
            self.progress("Failed to start mavlink connection on %s" %
                          connection_string)
            raise
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)
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
    # # TEST ARM RADIO
    # def test_arm_motors_radio(self):
    #     """Test Arming motors with radio."""
    #     self.progress("Test arming motors with radio")
    #     self.mavproxy.send('switch 6\n')  # stabilize/manual mode
    #     self.wait_mode('MANUAL')
    #     self.mavproxy.send('rc 3 1500\n')  # throttle at zero
    #     self.mavproxy.send('rc 1 2000\n')  # steer full right
    #     self.mavproxy.expect('APM: Throttle armed')
    #     self.mavproxy.send('rc 1 1500\n')
    #
    #     self.mav.motors_armed_wait()
    #     self.progress("MOTORS ARMED OK")
    #     return True
    #
    # # TEST DISARM RADIO
    # def test_disarm_motors_radio(self):
    #     """Test Disarm motors with radio."""
    #     self.progress("Test disarming motors with radio")
    #     self.mavproxy.send('switch 6\n')  # stabilize/manual mode
    #     self.wait_mode('MANUAL')
    #     self.mavproxy.send('rc 3 1500\n')  # throttle at zero
    #     self.mavproxy.send('rc 1 1000\n')  # steer full right
    #     tstart = self.get_sim_time()
    #     self.mav.wait_heartbeat()
    #     timeout = 15
    #     while self.get_sim_time() < tstart + timeout:
    #         self.mav.wait_heartbeat()
    #         if not self.mav.motors_armed():
    #             disarm_delay = self.get_sim_time() - tstart
    #             self.progress("MOTORS DISARMED OK WITH RADIO")
    #             self.mavproxy.send('rc 1 1500\n')  # steer full right
    #             self.mavproxy.send('rc 4 1500\n')  # yaw full right
    #             self.progress("Disarm in %ss" % disarm_delay)
    #             return True
    #     self.progress("FAILED TO DISARM WITH RADIO")
    #     return False
    #
    # # TEST AUTO DISARM
    # def test_autodisarm_motors(self):
    #     """Test Autodisarm motors."""
    #     self.progress("Test Autodisarming motors")
    #     self.mavproxy.send('switch 6\n')  # stabilize/manual mode
    #     #  NOT IMPLEMENTED ON ROVER
    #     self.progress("MOTORS AUTODISARMED OK")
    #     return True
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
        self.progress("TEST SQUARE")

        # use LEARNING Mode
        self.mavproxy.send('switch 5\n')
        self.wait_mode('MANUAL')

        # first aim north
        self.progress("\nTurn right towards north")
        self.reach_heading_manual(10)
        # save bottom left corner of box as waypoint
        self.progress("Save WP 1 & 2")
        self.save_wp()

        # pitch forward to fly north
        self.progress("\nGoing north %u meters" % side)
        self.reach_distance_manual(side)
        # save top left corner of square as waypoint
        self.progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        self.progress("\nGoing east %u meters" % side)
        self.reach_heading_manual(100)
        self.reach_distance_manual(side)
        # save top right corner of square as waypoint
        self.progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        self.progress("\nGoing south %u meters" % side)
        self.reach_heading_manual(190)
        self.reach_distance_manual(side)
        # save bottom right corner of square as waypoint
        self.progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        self.progress("\nGoing west %u meters" % side)
        self.reach_heading_manual(280)
        self.reach_distance_manual(side)
        # save bottom left corner of square (should be near home) as waypoint
        self.progress("Save WP 6")
        self.save_wp()

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
        self.wait_mode('HOLD')
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

        self.progress("banner not received")
        raise MsgRcvTimeoutException()

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
            self.progress("Brakes have negligible effect"
                          "(with=%0.2fm without=%0.2fm delta=%0.2fm)" %
                          (distance_with_brakes,
                           distance_without_brakes,
                           delta))
            raise NotAchievedException()

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
            self.progress("Did not receive NAV_CONTROLLER_OUTPUT message")
            raise MsgRcvTimeoutException()

        wp_dist_min = 5
        if m.wp_dist < wp_dist_min:
            self.progress("Did not start at least 5 metres from destination")
            raise PreconditionFailedException()

        self.progress("NAV_CONTROLLER_OUTPUT.wp_dist looks good (%u >= %u)" %
                      (m.wp_dist, wp_dist_min,))

        self.wait_mode('HOLD')

        pos = self.mav.location()
        home_distance = self.get_distance(HOME, pos)
        home_distance_max = 5
        if home_distance > home_distance_max:
            self.progress("Did not get home (%u metres distant > %u)" %
                          (home_distance, home_distance_max))
            raise NotAchievedException()
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')
        self.progress("RTL Mission OK")

    def test_servorelayevents(self):
        self.mavproxy.send("relay set 0 0\n")
        off = self.get_parameter("SIM_PIN_MASK")
        self.mavproxy.send("relay set 0 1\n")
        on = self.get_parameter("SIM_PIN_MASK")
        if on == off:
            self.progress("Pin mask unchanged after relay command")
            raise NotAchievedException()
        self.progress("Pin mask changed after relay command")

    def autotest(self):
        """Autotest APMrover2 in SITL."""
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
            self.progress("Waiting reading for arm")
            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.run_test("Drive an RTL Mission", self.drive_rtl_mission)

            self.run_test("Learn/Drive Square with Ch7 option",
                          self.drive_square)

            self.run_test("Drive Mission %s" % "rover1.txt",
                          self.drive_mission_rover1)

            self.run_test("Drive Brake", self.drive_brake)

            self.run_test("Disarm Vehicle", self.disarm_vehicle)

            self.run_test("Get Banner", self.do_get_banner)

            self.run_test("Get Capabilities",
                          self.do_get_autopilot_capabilities)

            self.run_test("Set mode via MAV_COMMAND_DO_SET_MODE",
                          self.do_set_mode_via_command_long)

            self.run_test("Test ServoRelayEvents",
                          self.test_servorelayevents)

            self.run_test("Download logs", lambda:
                          self.log_download(
                              self.buildlogs_path("APMrover2-log.bin")))
    #        if not drive_left_circuit(self):
    #            self.progress("Failed left circuit")
    #            failed = True
    #        if not drive_RTL(self):
    #            self.progress("Failed RTL")
    #            failed = True

        except pexpect.TIMEOUT as e:
            self.progress("Failed with timeout")
            self.fail_list.append(("*timeout*", None))

        self.close()

        if len(self.fail_list):
            self.progress("FAILED STEPS: %s" % self.fail_list)
            return False
        return True
