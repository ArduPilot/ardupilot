#!/usr/bin/env python
from __future__ import print_function
import os
import shutil
import pexpect

from common import *
from pysim import util
from pysim import vehicleinfo
from pymavlink import mavutil, mavwp

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-35.362938, 149.165085, 584, 270)


class AutotestRover(Autotest):
    def __init__(self, binary, viewerip=None, use_map=False, valgrind=False, gdb=False, speedup=10, frame=None, params=None):
        super(AutotestRover, self).__init__()
        self.binary = binary
        self.options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
        self.viewerip = viewerip
        self.use_map = use_map
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params

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
        self.mavproxy.send("param set LOG_REPLAY 1\n")
        self.mavproxy.send("param set LOG_DISARMED 1\n")
        progress("RELOADING SITL WITH NEW PARAMETERS")
        time.sleep(3)

        # restart with new parms
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        self.sitl = util.start_SITL(self.binary, model=self.frame, home=self.home, speedup=self.speedup,
                                    valgrind=self.valgrind, gdb=self.gdb)
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

    def reset_and_arm(self):
        """Reset RC, set to MANUAL and arm."""
        self.mav.wait_heartbeat()
        # ensure all sticks in the middle
        self.set_rc_default()
        self.mavproxy.send('switch 1\n')
        self.mav.wait_heartbeat()
        self.disarm_vehicle()
        self.mav.wait_heartbeat()
        self.arm_vehicle()

    ##################################################################
    #   IMPLEMENTATIONS
    ##################################################################
    # TEST ARM RADIO
    def test_arm_motors_radio(self):
        """Test Arming motors with radio."""
        progress("Test arming motors with radio")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        self.wait_mode('MANUAL')
        self.mavproxy.send('rc 3 1500\n')  # throttle at zero
        self.mavproxy.send('rc 1 2000\n')  # steer full right
        self.mavproxy.expect('APM: Throttle armed')
        self.mavproxy.send('rc 1 1500\n')

        self.mav.motors_armed_wait()
        progress("MOTORS ARMED OK")
        return True

    # TEST DISARM RADIO
    def test_disarm_motors_radio(self):
        """Test Disarm motors with radio."""
        progress("Test disarming motors with radio")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        self.wait_mode('MANUAL')
        self.mavproxy.send('rc 3 1500\n')  # throttle at zero
        self.mavproxy.send('rc 1 1000\n')  # steer full right
        tstart = self.get_sim_time()
        self.mav.wait_heartbeat()
        timeout = 15
        while self.get_sim_time() < tstart + timeout:
            self.mav.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_delay = self.get_sim_time() - tstart
                progress("MOTORS DISARMED OK WITH RADIO")
                self.mavproxy.send('rc 1 1500\n')  # steer full right
                self.mavproxy.send('rc 4 1500\n')  # yaw full right
                progress("Disarm in %ss" % disarm_delay)
                return True
        progress("FAILED TO DISARM WITH RADIO")
        return False

    # TEST AUTO DISARM
    def test_autodisarm_motors(self):
        """Test Autodisarm motors."""
        progress("Test Autodisarming motors")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        #  NOT IMPLEMENTED ON ROVER
        progress("MOTORS AUTODISARMED OK")
        return True

    # TEST RC OVERRIDE
    # TEST RC OVERRIDE TIMEOUT
    def test_rtl(self, home, distance_min=5, timeout=250):
        """Return, land."""
        super(AutotestRover, self).test_rtl(home, distance_min, timeout)

    def test_mission(self, filename):
        """Test a mission from a file."""
        progress("Test mission %s" % filename)
        num_wp = self.load_mission_from_file(filename)
        self.mavproxy.send('wp set 1\n')
        self.mav.wait_heartbeat()
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        ret = self.wait_waypoint(0, num_wp-1, max_dist=5, timeout=500)

        if ret:
            self.mavproxy.expect("Mission Complete")
        self.mav.wait_heartbeat()
        self.wait_mode('HOLD')
        progress("test: MISSION COMPLETE: passed=%s" % ret)
        return ret

##########################################################
#   TESTS DRIVE
##########################################################
# Drive a square in manual mode
    def drive_square(self, side=50):
        """Drive a square, Driving N then E ."""
        progress("TEST SQUARE")
        success = True

        # use LEARNING Mode
        self.mavproxy.send('switch 5\n')
        self.wait_mode('LEARNING')

        # first aim north
        progress("\nTurn right towards north")
        if not self.reach_heading(10):
            success = False

        # save bottom left corner of box as waypoint
        progress("Save WP 1 & 2")
        self.save_wp()

        # pitch forward to fly north
        progress("\nGoing north %u meters" % side)
        if not self.reach_distance(side):
            success = False

        # save top left corner of square as waypoint
        progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        progress("\nGoing east %u meters" % side)
        if not self.reach_heading(100):
            success = False
        if not self.reach_distance(side):
            success = False

        # save top right corner of square as waypoint
        progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        progress("\nGoing south %u meters" % side)
        if not self.reach_heading(190):
            success = False
        if not self.reach_distance(side):
            success = False

        # save bottom right corner of square as waypoint
        progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        progress("\nGoing west %u meters" % side)
        if not self.reach_heading(280):
            success = False
        if not self.reach_distance(side):
            success = False

        # save bottom left corner of square (should be near home) as waypoint
        progress("Save WP 6")
        self.save_wp()

        return success

    def drive_left_circuit(self):
        """Drive a left circuit, 50m on a side."""
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')
        self.mavproxy.send('rc 3 2000\n')

        progress("Driving left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            progress("Starting turn %u" % i)
            self.mavproxy.send('rc 1 1000\n')
            if not self.wait_heading(270 - (90*i), accuracy=10):
                return False
            self.mavproxy.send('rc 1 1500\n')
            progress("Starting leg %u" % i)
            if not self.wait_distance(50, accuracy=7):
                return False
        self.mavproxy.send('rc 3 1500\n')
        progress("Circuit complete")
        return True

    def test_throttle_failsafe(self, home, distance_min=10, side=60, timeout=300):
        """Fly east, Failsafe, return, land."""

        self.mavproxy.send('switch 6\n')  # manual mode
        self.wait_mode('MANUAL')
        self.mavproxy.send("param set FS_ACTION 1\n")

        # first aim east
        progress("turn east")
        if not self.reach_heading(135):
            return False

        # fly east 60 meters
        progress("# Going forward %u meters" % side)
        if not self.reach_distance(side):
            return False

        # pull throttle low
        progress("# Enter Failsafe")
        self.mavproxy.send('rc 3 900\n')

        tstart = self.get_sim_time()
        success = False
        while self.get_sim_time() < tstart + timeout and not success:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(home, pos)
            progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
            # check if we've reached home
            if home_distance <= distance_min:
                progress("RTL Complete")
                success = True

        # reduce throttle
        self.mavproxy.send('rc 3 1500\n')
        self.mavproxy.expect('APM: Failsafe ended')
        self.mavproxy.send('switch 2\n')  # manual mode
        self.mav.wait_heartbeat()
        self.wait_mode('MANUAL')

        if success:
            progress("Reached failsafe home OK")
            return True
        else:
            progress("Failed to reach Home on failsafe RTL - timed out after %u seconds" % timeout)
            return False

#################################################
# AUTOTEST ALL
#################################################
    def autotest(self):
        """Autotest APMrover2 in SITL."""
        if not self.hasInit:
            self.init()

        progress("Started simulator")

        sucess = True
        e = 'None'
        try:
            progress("Waiting for a heartbeat with mavlink protocol %s" % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            progress("Setting up RC parameters")
            self.set_rc_default()
            self.mavproxy.send('switch 1\n')  # Manual mode
            progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()
            homeloc = self.mav.location()
            progress("Home location: %s" % homeloc)
            self.mavproxy.send('switch 1\n')  # Manual mode
            self.wait_mode('MANUAL')
            progress("Waiting reading for arm")
            self.wait_ready_to_arm()

            progress("#")
            progress("########## Do common feature tests ##########")
            progress("#")
            if not self.test_common_feature():
                progress("Failed common feature")
                sucess = False

            progress("#")
            progress("########## Drive a square and save WPs with CH7 switch  ##########")
            progress("#")
            # Drive a square in learning mode
            self.reset_and_arm()
            if not self.drive_square():
                progress("Failed drive square")
                sucess = False

            # save the stored mission to file
            progress("# Save out the CH7 mission to file")
            if not self.save_mission_to_file(os.path.join(testdir, "ch7_mission.txt")):
                progress("save_mission_to_file failed")
                sucess = False

            # drive the stored mission
            progress("# Drive CH7 saved mission")
            if not self.test_mission(os.path.join(testdir, "ch7_mission.txt")):
                progress("drive ch7_mission failed")
                sucess = False

            # Throttle Failsafe
            progress("#")
            progress("########## Test Failsafe ##########")
            progress("#")
            self.reset_and_arm()
            if not self.test_throttle_failsafe(HOME, distance_min=4):
                progress("Throttle failsafe failed")
                sucess = False

            # # Battery failsafe
            # if not drive_battery_failsafe(mavproxy, mav):
            #     progress("Battery failsafe failed")
            #     failed = True
            #
            # # Fly GPS Glitch Loiter test
            # progress("# GPS Glitch Loiter Test")
            # if not fly_gps_glitch_loiter_test(mavproxy, mav, use_map):
            #     failed_test_msg = "fly_gps_glitch_loiter_test failed"
            #     progress(failed_test_msg)
            #     failed = True
            #
            # # RTL after GPS Glitch Loiter test
            # progress("# RTL #")
            # if not fly_RTL(mavproxy, mav):
            #     failed_test_msg = "fly_RTL failed"
            #     progress(failed_test_msg)
            #     failed = True
            #
            # # Fly GPS Glitch test in auto mode
            # progress("# GPS Glitch Auto Test")
            # if not fly_gps_glitch_auto_test(mavproxy, mav, use_map):
            #     failed_test_msg = "fly_gps_glitch_auto_test failed"
            #     progress(failed_test_msg)
            #     failed = True
            #
            # progress("# Fly copter mission")
            # if not drive_auto_test(mavproxy, mav):
            #     failed_test_msg = "fly_auto_test failed"
            #     progress(failed_test_msg)
            #     failed = True
            # else:
            #     progress("Flew copter mission OK")

            if not self.log_download(util.reltopdir("../buildlogs/APMrover2-log.bin")):
                progress("Failed log download")
                sucess = False

        except pexpect.TIMEOUT as e:
            progress("Failed with timeout")
            sucess = False

        self.close()

        if not sucess:
            progress("FAILED: %s" % e)
            return False
        return True
