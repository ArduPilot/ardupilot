#!/usr/bin/env python
from __future__ import print_function
import os
import shutil
import pexpect

from common_test import *
from pysim import util
from pymavlink import mavutil, mavwp

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-35.362938, 149.165085, 584, 270)


class AutotestRover(Autotest):
    def __init__(self, binary, viewerip=None, use_map=False, valgrind=False, gdb=False, speedup=10):
        self.binary = binary
        self.options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
        self.viewerip = viewerip
        self.use_map = use_map
        self.valgrind = valgrind
        self.gdb = gdb

        self.home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
        self.homeloc = None
        self.num_wp = 0
        self.speedup = speedup
        self.speedup_default = 10
        self.model = 'rover'

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.viewerip:
            self.options += " --out=%s:14550" % self.viewerip
        if self.use_map:
            self.options += ' --map'

        self.sitl = util.start_SITL(self.binary, wipe=True, model=self.model, home=self.home,
                                    speedup=self.speedup_default)
        self.mavproxy = util.start_MAVProxy_SITL('APMrover2')

        progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        # setup test parameters
        self.mavproxy.send("param load %s/default_params/rover.parm\n" % testdir)
        self.mavproxy.expect('Loaded [0-9]+ parameters')
        self.mavproxy.send("param set LOG_REPLAY 1\n")
        self.mavproxy.send("param set LOG_DISARMED 1\n")
        time.sleep(3)

        # restart with new parms
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        self.sitl = util.start_SITL(self.binary, model=self.model, home=self.home, speedup=self.speedup_default,
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

    def test_common_feature(self):
        failed = False
        # TEST ARMING/DISARM
        if not self.arm_vehicle():
            progress("Failed to ARM")
            failed = True
        if not self.disarm_vehicle():
            progress("Failed to DISARM")
            failed = True
        if not self.test_arm_motors_radio():
            progress("Failed to ARM with radio")
            failed = True
        if not self.test_disarm_motors_radio():
            progress("Failed to ARM with radio")
            failed = True
        if not self.test_autodisarm_motors():
            progress("Failed to AUTO DISARM")
            failed = True
        # TODO: Test failure on arm (with arming check)
        # TEST MISSION FILE
        progress("TEST LOADING MISSION")
        num_wp = self.load_mission_from_file(os.path.join(testdir, "all_msg_mission.txt"))
        if num_wp == 0:
            progress("load all_msg_mission failed")
            failed = True

        progress("TEST SAVING MISSION")
        num_wp_old = num_wp
        num_wp = self.save_mission_to_file(os.path.join(testdir, "all_msg_mission2.txt"))
        if num_wp != num_wp_old:
            progress("save all_msg_mission failed")
            failed = True

        progress("TEST CLEARING MISSION")
        self.mavproxy.send("wp clear\n")
        num_wp = mavwp.MAVWPLoader().count()
        if num_wp != 0:
            progress("clear mission failed")
            failed = True

        return failed

    ##################################################################
    #   IMPLEMENTATIONS
    ##################################################################
    # TEST ARM RADIO
    def test_arm_motors_radio(self):
        """Test Arming motors with radio."""
        progress("Test arming motors with radio")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.wait_mode('STABILIZE')
            self.mavproxy.send('rc 3 1000\n')  # throttle at zero
            self.mavproxy.send('rc 4 2000\n')  # yaw full right
            self.mavproxy.expect('APM: Arming motors')
            self.mavproxy.send('rc 4 1500\n')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
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
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.wait_mode('STABILIZE')
            self.mavproxy.send('rc 3 1000\n')  # throttle at zero
            self.mavproxy.send('rc 4 1000\n')  # yaw full right
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.wait_mode('MANUAL')
            self.mavproxy.send('rc 3 1500\n')  # throttle at zero
            self.mavproxy.send('rc 1 1000\n')  # steer full right

        tstart = self.get_sim_time()
        timeout = 15
        while self.get_sim_time() < tstart + timeout:
            if not self.mav.motors_armed():
                progress("MOTORS DISARMED OK WITH RADIO")
                self.mavproxy.send('rc 1 1500\n')  # steer full right
                self.mavproxy.send('rc 4 1500\n')  # yaw full right
                return True
        progress("FAILED TO DISARM WITH RADIO")
        return False

    # TEST AUTO DISARM
    def test_autodisarm_motors(self):
        """Test Autodisarm motors."""
        progress("Test Autodisarming motors")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.wait_mode('STABILIZE')
            self.mavproxy.send('rc 3 1000\n')  # throttle at zero
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            #  NOT IMPLEMENTED ON ROVER
            progress("MOTORS AUTODISARMED OK")
            return True
        self.arm_vehicle()

        tstart = self.get_sim_time()
        timeout = 15     #  TODO: adapt timeout with data from param
        while self.get_sim_time() < tstart + timeout:
            if not self.mav.motors_armed():
                progress("MOTORS AUTODISARMED OK ")
                self.mavproxy.send('rc 1 1500\n')  # steer full right
                self.mavproxy.send('rc 4 1500\n')  # yaw full right
                return True
        progress("FAILED TO AUTODISARMED")
        return False

    # TEST RC OVERRIDE
    # TEST RC OVERRIDE TIMEOUT
    def test_RTL(self, home, timeout=250):
        """Return, land."""
        progress("# Enter RTL")
        self.mavproxy.send('switch 3\n')
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(home, pos)
            progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
            if m.alt <= 1 and home_distance < 10:
                progress("RTL Complete")
                return True
        return False

    def test_mission(self, filename):
        """Test a mission from a file."""
        progress("Test mission %s" % filename)
        num_wp = self.load_mission_from_file(filename)
        self.mavproxy.send('wp set 1\n')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        ret = self.wait_waypoint(0, num_wp-1, max_dist=5, timeout=500)

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            expect_msg = "Reached command #%u" % (num_wp-1)
            if ret:
                self.mavproxy.expect(expect_msg)
            self.mavproxy.send('switch 5\n')  # loiter mode
            self.wait_mode('LOITER')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            print("NOT IMPLEMENTED")
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            expect_msg = "No commands."
            if ret:
                self.mavproxy.expect(expect_msg)
            self.wait_mode('HOLD')
        progress("test: MISSION COMPLETE: passed=%s" % ret)
        return ret

##########################################################
#   TESTS DRIVE
##########################################################
# TODO : TEST ARMING REQUIRE
# fly a square in stabilize mode
    def drive_square(self, side=50):
        """Drive a square, Driving N then E ."""
        progress("TEST SQUARE")
        success = True

        # ensure all sticks in the middle
        self.mavproxy.send('rc 1 1500\n')
        self.mavproxy.send('rc 2 1500\n')
        self.mavproxy.send('rc 3 1500\n')
        self.mavproxy.send('rc 4 1500\n')

        # use LEARNING Mode
        self.mavproxy.send('switch 5\n')
        self.wait_mode(self.mav, 'LEARNING')

        # first aim north
        progress("turn right towards north")
        if not self.reach_heading(10):
            success = False

        # save bottom left corner of box as waypoint
        progress("Save WP 1 & 2")
        self.save_wp()

        # pitch forward to fly north
        progress("Going north %u meters" % side)
        if not self.reach_distance(side):
            success = False

        # save top left corner of square as waypoint
        progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        progress("Going east %u meters" % side)
        if not self.reach_heading(100):
            success = False
        if not self.reach_distance(side):
            success = False

        # save top right corner of square as waypoint
        progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        progress("Going south %u meters" % side)
        if not self.reach_heading(190):
            success = False
        if not self.reach_distance(side):
            success = False

        # save bottom right corner of square as waypoint
        progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        progress("Going west %u meters" % side)
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

    def test_throttle_failsafe(self,   home, side=60, timeout=180):
        """Fly east, Failsafe, return, land."""

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            # switch to loiter mode temporarily to stop us from rising
            self.mavproxy.send('switch 5\n')
            self.wait_mode('LOITER')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.mavproxy.send('switch 6\n')  # manual mode
            self.wait_mode('MANUAL')
            self.mavproxy.send("param set FS_ACTION 1\n")

        # first aim east
        progress("turn east")
        if not self.reach_heading(135):
            return False

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            # raise throttle slightly to avoid hitting the ground
            self.mavproxy.send('rc 3 1600\n')
            # switch to stabilize mode
            self.mavproxy.send('switch 6\n')
            self.wait_mode( 'STABILIZE')
            self.mavproxy.send('rc 3 1500\n')

        # fly east 60 meters
        progress("# Going forward %u meters" % side)
        if not self.reach_distance(  side):
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
            if (m.alt - home.alt) <= 1 and home_distance < 10:
                success = True

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            # reduce throttle
            self.mavproxy.send('rc 3 1100\n')
            # switch back to stabilize mode
            self.mavproxy.send('switch 2\n')  # land mode
            self.wait_mode('LAND')
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.wait_mode('HOLD')
            # reduce throttle
            self.mavproxy.send('rc 3 1500\n')
            self.mavproxy.expect('APM: Failsafe ended')
            self.mavproxy.send('switch 2\n')  # manual mode
            self.mavproxy.send('switch 6\n')
            self.wait_mode('MANUAL')

        if success:
            progress("Reached failsafe home OK")
            return True
        else:
            progress("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
            return False



        #################################################
# AUTOTEST ALL
#################################################
    def drive_APMrover2(self):
        """Drive APMrover2 in SITL.

        you can pass viewerip as an IP address to optionally send fg and
        mavproxy packets too for local viewing of the mission in real time
        """

        progress("Started simulator")

        failed = False
        e = 'None'
        try:
            progress("Waiting for a heartbeat with mavlink protocol %s" % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            progress("Setting up RC parameters")
            self.setup_rc()
            self.set_switch_default()
            progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()
            homeloc = self.mav.location()
            progress("Home location: %s" % homeloc)
            self.mavproxy.send('switch 6\n')  # Manual mode
            self.wait_mode('MANUAL')
            progress("Waiting reading for arm")
            self.wait_ready_to_arm()

            progress("#")
            progress("########## Do common feature tests ##########")
            progress("#")
            if not self.test_common_feature():
                progress("Failed common feature")
                failed = True

            progress("#")
            progress("########## Drive a square and save WPs with CH7 switch  ##########")
            progress("#")
            # Drive a square in learning mode
            if not self.drive_square():
                progress("Failed drive square")
                failed = True

            # save the stored mission to file
            progress("# Save out the CH7 mission to file")
            if not self.save_mission_to_file(os.path.join(testdir, "ch7_mission.txt")):
                progress("save_mission_to_file failed")
                failed = True

            # drive the stored mission
            progress("# Drive CH7 saved mission")
            if not self.test_mission(os.path.join(testdir, "ch7_mission.txt")):
                progress("drive ch7_mission failed")
                failed = True

            # Throttle Failsafe
            progress("#")
            progress("########## Test Failsafe ##########")
            progress("#")
            if not self.test_throttle_failsafe(HOME):
                progress("Throttle failsafe failed")
                failed = True

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
                failed = True

        except pexpect.TIMEOUT as e:
            progress("Failed with timeout")
            failed = True

        self.mav.close()
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        valgrind_log = util.valgrind_log_filepath(binary=self.binary, model=self.model)
        if os.path.exists(valgrind_log):
            os.chmod(valgrind_log, 0o644)
            shutil.copy(valgrind_log, util.reltopdir("../buildlogs/APMrover2-valgrind.log"))

        if failed:
            progress("FAILED: %s" % e)
            return False
        return True
