#!/usr/bin/env python

# Drive APMrover2 in SITL
from __future__ import print_function

import shutil

import pexpect

from common import *
from pysim import util
from pysim import vehicleinfo

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

# HOME = mavutil.location(-35.362938, 149.165085, 584, 270)
HOME = mavutil.location(40.071374969556928, -105.22978898137808, 1583.702759, 246)


class AutotestRover(Autotest):
    def __init__(self, binary, viewerip=None, use_map=False, valgrind=False, gdb=False, speedup=10, frame=None, params=None, gdbserver=False):
        super(AutotestRover, self).__init__()
        self.binary = binary
        self.options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
        self.viewerip = viewerip
        self.use_map = use_map
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver

        self.home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
        self.homeloc = None
        self.speedup = speedup
        self.speedup_default = 10

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.frame is None:
            self.frame = 'rover'

        if self.viewerip:
            self.options += " --out=%s:14550" % self.viewerip
        if self.use_map:
            self.options += ' --map'

        self.sitl = util.start_SITL(self.binary, wipe=True, model=self.frame, home=self.home,
                                    speedup=self.speedup_default)
        self.mavproxy = util.start_MAVProxy_SITL('APMrover2')

        progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        # setup test parameters
        vinfo = vehicleinfo.VehicleInfo()
        if self.params is None:
            self.params = vinfo.options["APMrover2"]["frames"][self.frame]["default_params_filename"]
        if not isinstance(self.params, list):
            self.params = [self.params]
        for x in self.params:
            self.mavproxy.send("param load %s\n" % os.path.join(testdir, x))
            self.mavproxy.expect('Loaded [0-9]+ parameters')
        self.set_parameter('LOG_REPLAY', 1)
        self.set_parameter('LOG_DISARMED', 1)
        progress("RELOADING SITL WITH NEW PARAMETERS")

        # restart with new parms
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        self.sitl = util.start_SITL(self.binary, model=self.frame, home=self.home, speedup=self.speedup,
                                    valgrind=self.valgrind, gdb=self.gdb, gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL('APMrover2', options=self.options)
        self.mavproxy.expect('Telemetry log: (\S+)')
        logfile = self.mavproxy.match.group(1)
        progress("LOGFILE %s" % logfile)

        buildlog = util.reltopdir("../buildlogs/APMrover2-test.tlog")
        progress("buildlog=%s" % buildlog)
        if os.path.exists(buildlog):
            os.unlink(buildlog)
        try:
            os.link(logfile, buildlog)
        except Exception:
            pass

        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, expect_callback)

        expect_list_clear()
        expect_list_extend([self.sitl, self.mavproxy])

        progress("Started simulator")

        # get a mavlink connection going
        try:
            self.mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
        except Exception as msg:
            progress("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
            raise
        self.mav.message_hooks.append(message_hook)
        self.mav.idle_hooks.append(idle_hook)
        self.hasInit = True
        progress("Ready to start testing!")

    def close(self):
        if self.use_map:
            self.mavproxy.send("module unload map\n")
            self.mavproxy.expect("Unloaded module map")

        self.mav.close()
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        valgrind_log = util.valgrind_log_filepath(binary=self.binary, model=self.frame)
        if os.path.exists(valgrind_log):
            os.chmod(valgrind_log, 0o644)
            shutil.copy(valgrind_log, util.reltopdir("../buildlogs/APMrover2-valgrind.log"))

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
    #     progress("Test arming motors with radio")
    #     self.mavproxy.send('switch 6\n')  # stabilize/manual mode
    #     self.wait_mode('MANUAL')
    #     self.mavproxy.send('rc 3 1500\n')  # throttle at zero
    #     self.mavproxy.send('rc 1 2000\n')  # steer full right
    #     self.mavproxy.expect('APM: Throttle armed')
    #     self.mavproxy.send('rc 1 1500\n')
    #
    #     self.mav.motors_armed_wait()
    #     progress("MOTORS ARMED OK")
    #     return True
    #
    # # TEST DISARM RADIO
    # def test_disarm_motors_radio(self):
    #     """Test Disarm motors with radio."""
    #     progress("Test disarming motors with radio")
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
    #             progress("MOTORS DISARMED OK WITH RADIO")
    #             self.mavproxy.send('rc 1 1500\n')  # steer full right
    #             self.mavproxy.send('rc 4 1500\n')  # yaw full right
    #             progress("Disarm in %ss" % disarm_delay)
    #             return True
    #     progress("FAILED TO DISARM WITH RADIO")
    #     return False
    #
    # # TEST AUTO DISARM
    # def test_autodisarm_motors(self):
    #     """Test Autodisarm motors."""
    #     progress("Test Autodisarming motors")
    #     self.mavproxy.send('switch 6\n')  # stabilize/manual mode
    #     #  NOT IMPLEMENTED ON ROVER
    #     progress("MOTORS AUTODISARMED OK")
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
    #     progress("Test mission %s" % filename)
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
    #     progress("test: MISSION COMPLETE: passed=%s" % ret)
    #     return ret

    ##########################################################
    #   TESTS DRIVE
    ##########################################################
    def drive_left_circuit(self):
        """Drive a left circuit, 50m on a side."""
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')
        self.set_rc(3, 2000)

        progress("Driving left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            progress("Starting turn %u" % i)
            self.set_rc(1, 1000)
            if not self.wait_heading(270 - (90*i), accuracy=10):
                return False
            self.set_rc(1, 1500)
            progress("Starting leg %u" % i)
            if not self.wait_distance(50, accuracy=7):
                return False
        self.set_rc(3, 1500)
        progress("Circuit complete")
        return True

    # def test_throttle_failsafe(self, home, distance_min=10, side=60, timeout=300):
    #     """Fly east, Failsafe, return, land."""
    #
    #     self.mavproxy.send('switch 6\n')  # manual mode
    #     self.wait_mode('MANUAL')
    #     self.mavproxy.send("param set FS_ACTION 1\n")
    #
    #     # first aim east
    #     progress("turn east")
    #     if not self.reach_heading_manual(135):
    #         return False
    #
    #     # fly east 60 meters
    #     progress("# Going forward %u meters" % side)
    #     if not self.reach_distance_manual(side):
    #         return False
    #
    #     # pull throttle low
    #     progress("# Enter Failsafe")
    #     self.mavproxy.send('rc 3 900\n')
    #
    #     tstart = self.get_sim_time()
    #     success = False
    #     while self.get_sim_time() < tstart + timeout and not success:
    #         m = self.mav.recv_match(type='VFR_HUD', blocking=True)
    #         pos = self.mav.location()
    #         home_distance = self.get_distance(home, pos)
    #         progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
    #         # check if we've reached home
    #         if home_distance <= distance_min:
    #             progress("RTL Complete")
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
    #         progress("Reached failsafe home OK")
    #         return True
    #     else:
    #         progress("Failed to reach Home on failsafe RTL - timed out after %u seconds" % timeout)
    #         return False

    #################################################
    # AUTOTEST ALL
    #################################################
    def drive_mission(self, filename):
        """Drive a mission from a file."""
        progress("Driving mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.set_rc(3, 1500)
        self.wait_mode('AUTO')
        if not self.wait_waypoint(1, 4, max_dist=5):
            return False
        self.wait_mode('HOLD')
        progress("Mission OK")
        return True

    def do_get_banner(self):
        self.mavproxy.send("long DO_SEND_BANNER 1\n")
        start = time.time()
        while True:
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None and "ArduRover" in m.text:
                progress("banner received: %s" % m.text)
                return True
            if time.time() - start > 10:
                break

        progress("banner not received")

        return False

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
            progress("Brakes have negligible effect (with=%0.2fm without=%0.2fm delta=%0.2fm)" % (distance_with_brakes, distance_without_brakes, delta))
            return False
        else:
            progress("Brakes work (with=%0.2fm without=%0.2fm delta=%0.2fm)" % (distance_with_brakes, distance_without_brakes, delta))

        return True

    def drive_rtl_mission(self):
        mission_filepath = os.path.join(testdir, "ArduRover-Missions", "rtl.txt")
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
            progress("Did not receive NAV_CONTROLLER_OUTPUT message")
            return False

        wp_dist_min = 5
        if m.wp_dist < wp_dist_min:
            progress("Did not start at least 5 metres from destination")
            return False

        progress("NAV_CONTROLLER_OUTPUT.wp_dist looks good (%u >= %u)" %
                 (m.wp_dist, wp_dist_min,))

        self.wait_mode('HOLD')

        pos = self.mav.location()
        home_distance = self.get_distance(HOME, pos)
        home_distance_max = 5
        if home_distance > home_distance_max:
            progress("Did not get home (%u metres distant > %u)" %
                     (home_distance, home_distance_max))
            return False
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')
        progress("RTL Mission OK")
        return True

    def autotest(self):
        """Autotest APMrover2 in SITL."""
        if not self.hasInit:
            self.init()
        progress("Started simulator")

        failed = False
        e = 'None'
        try:
            progress("Waiting for a heartbeat with mavlink protocol %s" % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(8, 1800)
            progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            progress("Home location: %s" % self.homeloc)
            self.mavproxy.send('switch 6\n')  # Manual mode
            self.wait_mode('MANUAL')
            progress("Waiting reading for arm")
            self.wait_ready_to_arm()
            if not self.arm_vehicle():
                progress("Failed to ARM")
                failed = True

            progress("#")
            progress("########## Drive an RTL mission  ##########")
            progress("#")
            # Drive a square in learning mode
            # self.reset_and_arm()
            if not self.drive_rtl_mission():
                progress("Failed RTL mission")
                failed = True

            progress("#")
            progress("########## Drive a square and save WPs with CH7 switch  ##########")
            progress("#")

            if not self.drive_mission(os.path.join(testdir, "rover1.txt")):
                progress("Failed mission")
                failed = True

            if not self.drive_brake():
                progress("Failed brake")
                failed = True

            if not self.disarm_vehicle():
                progress("Failed to DISARM")
                failed = True

            # do not move this to be the first test.  MAVProxy's dedupe
            # function may bite you.
            progress("Getting banner")
            if not self.do_get_banner():
                progress("FAILED: get banner")
                failed = True

            progress("Getting autopilot capabilities")
            if not self.do_get_autopilot_capabilities():
                progress("FAILED: get capabilities")
                failed = True

            progress("Setting mode via MAV_COMMAND_DO_SET_MODE")
            if not self.do_set_mode_via_command_long():
                failed = True

            # Throttle Failsafe
            progress("#")
            progress("########## Test Failsafe ##########")
            progress("#")
            # self.reset_and_arm()
            # if not self.test_throttle_failsafe(HOME, distance_min=4):
            #     progress("Throttle failsafe failed")
            #     sucess = False

            if not self.log_download(util.reltopdir("../buildlogs/APMrover2-log.bin")):
                progress("Failed log download")
                failed = True
    #        if not drive_left_circuit(self):
    #            progress("Failed left circuit")
    #            failed = True
    #        if not drive_RTL(self):
    #            progress("Failed RTL")
    #            failed = True

        except pexpect.TIMEOUT as e:
            progress("Failed with timeout")
            failed = True

        self.close()

        if failed:
            progress("FAILED: %s" % e)
            return False
        return True
