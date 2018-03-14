#!/usr/bin/env python

# Fly ArduCopter in SITL
from __future__ import print_function
import math
import os
import shutil
import time

import pexpect
from pymavlink import mavutil, mavwp

from common import *
from pysim import util
from pysim import vehicleinfo

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-35.362938, 149.165085, 584, 270)
AVCHOME = mavutil.location(40.072842, -105.230575, 1586, 0)

# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize


class AutotestCopter(Autotest):
    def __init__(self, binary, viewerip=None, use_map=False, valgrind=False, gdb=False, speedup=10, frame=None, params=None, gdbserver=False):
        super(AutotestCopter, self).__init__()
        self.binary = binary
        self.options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=5'
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

        self.log_name = "ArduCopter"
        self.logfile = None
        self.buildlog = None
        self.copy_tlog = False

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.frame is None:
            self.frame = '+'

        if self.frame == 'heli':
            self.log_name = "HeliCopter"
        else:
            self.options += " --quadcopter"

        if self.viewerip:
            self.options += " --out=%s:14550" % self.viewerip
        if self.use_map:
            self.options += ' --map'

        self.sitl = util.start_SITL(self.binary, wipe=True, model=self.frame, home=self.home,
                                    speedup=self.speedup_default)
        self.mavproxy = util.start_MAVProxy_SITL('ArduCopter')

        progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        # setup test parameters
        vinfo = vehicleinfo.VehicleInfo()
        if self.params is None:
            self.params = vinfo.options["ArduCopter"]["frames"][self.frame]["default_params_filename"]
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
        self.mavproxy = util.start_MAVProxy_SITL('ArduCopter', options=self.options)
        self.mavproxy.expect('Telemetry log: (\S+)')
        self.logfile = self.mavproxy.match.group(1)
        progress("LOGFILE %s" % self.logfile)

        self.buildlog = util.reltopdir("../buildlogs/" + self.log_name + "-test.tlog")
        progress("buildlog=%s" % self.buildlog)
        self.copy_tlog = False
        if os.path.exists(self.buildlog):
            os.unlink(self.buildlog)
        try:
            os.link(self.logfile, self.buildlog)
        except Exception:
            progress("WARN: Failed to create symlink: " + self.logfile + " => " + self.buildlog + ", Will copy tlog manually to target location")
            self.copy_tlog = True

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
            shutil.copy(valgrind_log, util.reltopdir("../buildlogs/" + self.log_name + "-valgrind.log"))

        # [2014/05/07] FC Because I'm doing a cross machine build (source is on host, build is on guest VM) I cannot hard link
        # This flag tells me that I need to copy the data out
        if self.copy_tlog:
            shutil.copy(self.logfile, self.buildlog)

    def takeoff(self, alt_min=30, takeoff_throttle=1700):
        """Takeoff get to 30m altitude."""
        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        self.set_rc(3, takeoff_throttle)
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        if m.alt < alt_min:
            self.wait_altitude(alt_min, (alt_min + 5))
        self.hover()
        progress("TAKEOFF COMPLETE")
        return True

    def land(self, timeout=60):
        """Land the quad."""
        progress("STARTING LANDING")
        self.mavproxy.send('switch 2\n')  # land mode
        self.wait_mode('LAND')
        progress("Entered Landing Mode")
        ret = self.wait_altitude(-5, 1)
        progress("LANDING: ok= %s" % ret)
        return ret

    def hover(self, hover_throttle=1500):
        self.set_rc(3, hover_throttle)
        return True

    # loiter - fly south west, then hold loiter within 5m position and altitude
    def loiter(self, holdtime=10, maxaltchange=5, maxdistchange=5):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # first aim south east
        progress("turn south east")
        self.set_rc(4, 1580)
        if not self.wait_heading(170):
            return False
        self.set_rc(4, 1500)

        # fly south east 50m
        self.set_rc(2, 1100)
        if not self.wait_distance(50):
            return False
        self.set_rc(2, 1500)

        # wait for copter to slow moving
        if not self.wait_groundspeed(0, 2):
            return False

        success = True
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_altitude = m.alt
        start = self.mav.location()
        tstart = self.get_sim_time()
        tholdstart = self.get_sim_time()
        progress("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))
        while self.get_sim_time() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                progress("Loiter alt shifted %u meters (> limit of %u)" % (alt_delta, maxaltchange))
                success = False
            if delta > maxdistchange:
                progress("Loiter shifted %u meters (> limit of %u)" % (delta, maxdistchange))
                success = False
        if success:
            progress("Loiter OK for %u seconds" % holdtime)
        else:
            progress("Loiter FAILED")
        return success

    def change_alt(self, alt_min, climb_throttle=1920, descend_throttle=1080):
        """Change altitude."""
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        if m.alt < alt_min:
            progress("Rise to alt:%u from %u" % (alt_min, m.alt))
            self.set_rc(3, climb_throttle)
            self.wait_altitude(alt_min, (alt_min + 5))
        else:
            progress("Lower to alt:%u from %u" % (alt_min, m.alt))
            self.set_rc(3, descend_throttle)
            self.wait_altitude((alt_min - 5), alt_min)
        self.hover()
        return True

    #################################################
    #   TESTS FLY
    #################################################
    # def test_arm_motors_radio(self):
    #     super(AutotestCopter, self).test_arm_motors_radio()
    #
    # def test_disarm_motors_radio(self):
    #     super(AutotestCopter, self).test_disarm_motors_radio()
    #
    # def test_autodisarm_motors(self):
    #     super(AutotestCopter, self).test_autodisarm_motors()
    #
    # def test_rtl(self, home, distance_min=10, timeout=250):
    #     super(AutotestCopter, self).test_rtl(home, distance_min=10, timeout=250)
    #
    # def test_throttle_failsafe(self, home, distance_min=10, side=60, timeout=180):
    #     super(AutotestCopter, self).test_throttle_failsafe(home, distance_min=10, side=60, timeout=180)
    #
    # def test_mission(self, filename):
    #     super(AutotestCopter, self).test_mission(filename)

    # fly a square in stabilize mode
    def fly_square(self, side=50, timeout=300):
        """Fly a square, flying N then E ."""
        tstart = self.get_sim_time()
        success = True

        # ensure all sticks in the middle
        self.set_rc(1, 1500)
        self.set_rc(2, 1500)
        self.set_rc(3, 1500)
        self.set_rc(4, 1500)

        # switch to loiter mode temporarily to stop us from rising
        self.mavproxy.send('switch 5\n')
        self.wait_mode('LOITER')

        # first aim north
        progress("turn right towards north")
        self.set_rc(4, 1580)
        if not self.wait_heading(10):
            progress("Failed to reach heading")
            success = False
        self.set_rc(4, 1500)
        self.mav.recv_match(condition='RC_CHANNELS.chan4_raw==1500', blocking=True)

        # save bottom left corner of box as waypoint
        progress("Save WP 1 & 2")
        self.save_wp()

        # switch back to stabilize mode
        self.set_rc(3, 1500)
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')

        # pitch forward to fly north
        progress("Going north %u meters" % side)
        self.set_rc(2, 1300)
        if not self.wait_distance(side):
            progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(2, 1500)

        # save top left corner of square as waypoint
        progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        progress("Going east %u meters" % side)
        self.set_rc(1, 1700)
        if not self.wait_distance(side):
            progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(1, 1500)

        # save top right corner of square as waypoint
        progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        progress("Going south %u meters" % side)
        self.set_rc(2, 1700)
        if not self.wait_distance(side):
            progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(2, 1500)

        # save bottom right corner of square as waypoint
        progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        progress("Going west %u meters" % side)
        self.set_rc(1, 1300)
        if not self.wait_distance(side):
            progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(1, 1500)

        # save bottom left corner of square (should be near home) as waypoint
        progress("Save WP 6")
        self.save_wp()

        # descend to 10m
        progress("Descend to 10m in Loiter")
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')
        self.set_rc(3, 1300)
        time_left = timeout - (self.get_sim_time() - tstart)
        progress("timeleft = %u" % time_left)
        if time_left < 20:
            time_left = 20
        if not self.wait_altitude(-10, 10, time_left):
            progress("Failed to reach alt of 10m")
            success = False
        self.save_wp()

        return success

    def fly_RTL(self, side=60, timeout=250):
        """Return, land."""
        progress("# Enter RTL")
        self.mavproxy.send('switch 3\n')
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(HOME, pos)
            progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
            if m.alt <= 1 and home_distance < 10:
                return True
        return False

    def fly_throttle_failsafe(self, side=60, timeout=180):
        """Fly east, Failsafe, return, land."""

        # switch to loiter mode temporarily to stop us from rising
        self.mavproxy.send('switch 5\n')
        self.wait_mode('LOITER')

        # first aim east
        progress("turn east")
        self.set_rc(4, 1580)
        if not self.wait_heading(135):
            return False
        self.set_rc(4, 1500)

        # raise throttle slightly to avoid hitting the ground
        self.set_rc(3, 1600)

        # switch to stabilize mode
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')
        self.hover()

        # fly east 60 meters
        progress("# Going forward %u meters" % side)
        self.set_rc(2, 1350)
        if not self.wait_distance(side, 5, 60):
            return False
        self.set_rc(2, 1500)

        # pull throttle low
        progress("# Enter Failsafe")
        self.set_rc(3, 900)

        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(HOME, pos)
            progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
            # check if we've reached home
            if m.alt <= 1 and home_distance < 10:
                # reduce throttle
                self.set_rc(3, 1100)
                # switch back to stabilize
                self.mavproxy.send('switch 2\n')  # land mode
                self.wait_mode('LAND')
                progress("Waiting for disarm")
                self.mav.motors_disarmed_wait()
                progress("Reached failsafe home OK")
                self.mavproxy.send('switch 6\n')  # stabilize mode
                self.wait_mode('STABILIZE')
                self.set_rc(3, 1000)
                if not self.arm_vehicle():
                    progress("Failed to re-arm")
                    return False
                return True
        progress("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
        # reduce throttle
        self.set_rc(3, 1100)
        # switch back to stabilize mode
        self.mavproxy.send('switch 2\n')  # land mode
        self.wait_mode('LAND')
        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        return False

    def fly_battery_failsafe(self, timeout=30):
        # assume failure
        success = False

        # switch to loiter mode so that we hold position
        self.mavproxy.send('switch 5\n')
        self.wait_mode('LOITER')
        self.set_rc(3, 1500)

        # enable battery failsafe
        self.set_parameter('FS_BATT_ENABLE', 1)

        # trigger low voltage
        self.set_parameter('SIM_BATT_VOLTAGE', 10)

        # wait for LAND mode
        new_mode = self.wait_mode('LAND', 300)
        if new_mode == 'LAND':
            success = True

        # disable battery failsafe
        self.set_parameter('FS_BATT_ENABLE', 0)

        # return status
        if success:
            progress("Successfully entered LAND mode after battery failsafe")
        else:
            progress("Failed to enter LAND mode after battery failsafe")

        return success

    # fly_stability_patch - fly south, then hold loiter within 5m position and altitude and reduce 1 motor to 60% efficiency
    def fly_stability_patch(self, holdtime=30, maxaltchange=5, maxdistchange=10):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # first south
        progress("turn south")
        self.set_rc(4, 1580)
        if not self.wait_heading(180):
            return False
        self.set_rc(4, 1500)

        # fly west 80m
        self.set_rc(2, 1100)
        if not self.wait_distance(80):
            return False
        self.set_rc(2, 1500)

        # wait for copter to slow moving
        if not self.wait_groundspeed(0, 2):
            return False

        success = True
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_altitude = m.alt
        start = self.mav.location()
        tstart = self.get_sim_time()
        tholdstart = self.get_sim_time()
        progress("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))

        # cut motor 1 to 55% efficiency
        progress("Cutting motor 1 to 60% efficiency")
        self.mavproxy.send('param set SIM_ENGINE_MUL 0.60\n')

        while self.get_sim_time() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                progress("Loiter alt shifted %u meters (> limit of %u)" % (alt_delta, maxaltchange))
                success = False
            if delta > maxdistchange:
                progress("Loiter shifted %u meters (> limit of %u)" % (delta, maxdistchange))
                success = False

        # restore motor 1 to 100% efficiency
        self.mavproxy.send('param set SIM_ENGINE_MUL 1.0\n')

        if success:
            progress("Stability patch and Loiter OK for %u seconds" % holdtime)
        else:
            progress("Stability Patch FAILED")

        return success

    # fly_fence_test - fly east until you hit the horizontal circular fence
    def fly_fence_test(self, timeout=180):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # enable fence, disable avoidance
        self.mavproxy.send('param set FENCE_ENABLE 1\n')
        self.mavproxy.send('param set AVOID_ENABLE 0\n')

        # first east
        progress("turn east")
        self.set_rc(4, 1580)
        if not self.wait_heading(160):
            return False
        self.set_rc(4, 1500)

        # fly forward (east) at least 20m
        pitching_forward = True
        self.set_rc(2, 1100)
        if not self.wait_distance(20):
            return False

        # start timer
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(HOME, pos)
            progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
            # recenter pitch sticks once we reach home so we don't fly off again
            if pitching_forward and home_distance < 10:
                pitching_forward = False
                self.set_rc(2, 1500)
                # disable fence
                self.mavproxy.send('param set FENCE_ENABLE 0\n')
            if m.alt <= 1 and home_distance < 10:
                # reduce throttle
                self.set_rc(3, 1000)
                # switch mode to stabilize
                self.mavproxy.send('switch 2\n')  # land mode
                self.wait_mode('LAND')
                progress("Waiting for disarm")
                self.mav.motors_disarmed_wait()
                progress("Reached home OK")
                self.mavproxy.send('switch 6\n')  # stabilize mode
                self.wait_mode('STABILIZE')
                self.set_rc(3, 1000)
                self.mavproxy.send('arm uncheck all\n')  # remove if we ever clear battery failsafe flag on disarm
                if not self.arm_vehicle():
                    progress("Failed to re-arm")
                    self.mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm
                    return False
                self.mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm
                progress("Reached home OK")
                return True

        # disable fence, enable avoidance
        self.mavproxy.send('param set FENCE_ENABLE 0\n')
        self.mavproxy.send('param set AVOID_ENABLE 1\n')

        # reduce throttle
        self.set_rc(3, 1000)
        # switch mode to stabilize
        self.mavproxy.send('switch 2\n')  # land mode
        self.wait_mode('LAND')
        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        progress("Fence test failed to reach home - timed out after %u seconds" % timeout)
        return False

    # fly_alt_fence_test - fly up until you hit the fence
    def fly_alt_max_fence_test(self, timeout=180):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # enable fence, disable avoidance
        self.set_parameter('FENCE_ENABLE', 1)
        self.set_parameter('AVOID_ENABLE', 0)
        self.set_parameter('FENCE_TYPE', 1)

        if not self.change_alt(10):
            failed_test_msg = "change_alt climb failed"
            progress(failed_test_msg)
            return False

        # first east
        progress("turn east")
        self.set_rc(4, 1580)
        if not self.wait_heading(160):
            return False
        self.set_rc(4, 1500)

        # fly forward (east) at least 20m
        self.set_rc(2, 1100)
        if not self.wait_distance(20):
            return False

        # stop flying forward and start flying up:
        self.set_rc(2, 1500)
        self.set_rc(3, 1800)

        # wait for fence to trigger
        self.wait_mode('RTL')

        progress("Waiting for disarm")
        self.mav.motors_disarmed_wait()

        self.set_rc(3, 1000)

        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        self.mavproxy.send('arm uncheck all\n')  # remove if we ever clear battery failsafe flag on disarm
        if not self.arm_vehicle():
            progress("Failed to re-arm")
            self.mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm
            return False
        self.mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm

        return True

    def fly_gps_glitch_loiter_test(self, timeout=30, max_distance=20):
        """fly_gps_glitch_loiter_test.

         Fly south east in loiter and test reaction to gps glitch."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # turn on simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(True)

        # set-up gps glitch array
        glitch_lat = [0.0002996, 0.0006958, 0.0009431, 0.0009991, 0.0009444, 0.0007716, 0.0006221]
        glitch_lon = [0.0000717, 0.0000912, 0.0002761, 0.0002626, 0.0002807, 0.0002049, 0.0001304]
        glitch_num = len(glitch_lat)
        progress("GPS Glitches:")
        for i in range(1, glitch_num):
            progress("glitch %d %.7f %.7f" % (i, glitch_lat[i], glitch_lon[i]))

        # turn south east
        progress("turn south east")
        self.set_rc(4, 1580)
        if not self.wait_heading(150):
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            return False
        self.set_rc(4, 1500)

        # fly forward (south east) at least 60m
        self.set_rc(2, 1100)
        if not self.wait_distance(60):
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            return False
        self.set_rc(2, 1500)

        # wait for copter to slow down
        if not self.wait_groundspeed(0, 1):
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            return False

        # record time and position
        tstart = self.get_sim_time()
        tnow = tstart
        start_pos = self.sim_location()
        success = True

        # initialise current glitch
        glitch_current = 0
        progress("Apply first glitch")
        self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
        self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

        # record position for 30 seconds
        while tnow < tstart + timeout:
            tnow = self.get_sim_time()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # turn off glitching if we've reached the end of the glitch list
                if glitch_current >= glitch_num:
                    glitch_current = -1
                    progress("Completed Glitches")
                    self.mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
                    self.mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
                else:
                    progress("Applying glitch %u" % glitch_current)
                    # move onto the next glitch
                    self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
                    self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

            # start displaying distance moved after all glitches applied
            if glitch_current == -1:
                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                curr_pos = self.sim_location()
                moved_distance = self.get_distance(curr_pos, start_pos)
                progress("Alt: %u  Moved: %.0f" % (m.alt, moved_distance))
                if moved_distance > max_distance:
                    progress("Moved over %u meters, Failed!" % max_distance)
                    success = False

        # disable gps glitch
        if glitch_current != -1:
            glitch_current = -1
            self.mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
            self.mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        if success:
            progress("GPS glitch test passed!  stayed within %u meters for %u seconds" % (max_distance, timeout))
        else:
            progress("GPS glitch test FAILED!")
        return success

    # fly_gps_glitch_auto_test - fly mission and test reaction to gps glitch
    def fly_gps_glitch_auto_test(self, timeout=120):

        # set-up gps glitch array
        glitch_lat = [0.0002996, 0.0006958, 0.0009431, 0.0009991, 0.0009444, 0.0007716, 0.0006221]
        glitch_lon = [0.0000717, 0.0000912, 0.0002761, 0.0002626, 0.0002807, 0.0002049, 0.0001304]
        glitch_num = len(glitch_lat)
        progress("GPS Glitches:")
        for i in range(1, glitch_num):
            progress("glitch %d %.7f %.7f" % (i, glitch_lat[i], glitch_lon[i]))

        # Fly mission #1
        progress("# Load copter_glitch_mission")
        # load the waypoint count
        global num_wp
        num_wp = self.load_mission_from_file(os.path.join(testdir, "copter_glitch_mission.txt"))
        if not num_wp:
            progress("load copter_glitch_mission failed")
            return False

        # turn on simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(True)

        progress("test: Fly a mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')

        # switch into AUTO mode and raise throttle
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        self.set_rc(3, 1500)

        # wait until 100m from home
        if not self.wait_distance(100, 5, 60):
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            return False

        # record time and position
        tstart = self.get_sim_time()
        tnow = tstart

        # initialise current glitch
        glitch_current = 0
        progress("Apply first glitch")
        self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
        self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

        # record position for 30 seconds
        while glitch_current < glitch_num:
            tnow = self.get_sim_time()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # apply next glitch
                if glitch_current < glitch_num:
                    progress("Applying glitch %u" % glitch_current)
                    self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
                    self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

        # turn off glitching
        progress("Completed Glitches")
        self.mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
        self.mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')

        # continue with the mission
        ret = self.wait_waypoint(0, num_wp-1, timeout=500)

        # wait for arrival back home
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        pos = self.mav.location()
        dist_to_home = self.get_distance(HOME, pos)
        while dist_to_home > 5:
            if self.get_sim_time() > (tstart + timeout):
                progress("GPS Glitch testing failed - exceeded timeout %u seconds" % timeout)
                ret = False
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            dist_to_home = self.get_distance(HOME, pos)
            progress("Dist from home: %u" % dist_to_home)

        # turn off simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        progress("GPS Glitch test Auto completed: passed=%s" % ret)

        return ret

    #   fly_simple - assumes the simple bearing is initialised to be directly north
    #   flies a box with 100m west, 15 seconds north, 50 seconds east, 15 seconds south
    def fly_simple(self, side=50, timeout=120):

        failed = False

        # hold position in loiter
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # set SIMPLE mode for all flight modes
        self.mavproxy.send('param set SIMPLE 63\n')

        # switch to stabilize mode
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')
        self.set_rc(3, 1500)

        # fly south 50m
        progress("# Flying south %u meters" % side)
        self.set_rc(1, 1300)
        if not self.wait_distance(side, 5, 60):
            failed = True
        self.set_rc(1, 1500)

        # fly west 8 seconds
        progress("# Flying west for 8 seconds")
        self.set_rc(2, 1300)
        tstart = self.get_sim_time()
        while self.get_sim_time() < (tstart + 8):
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            delta = (self.get_sim_time() - tstart)
            # progress("%u" % delta)
        self.set_rc(2, 1500)

        # fly north 25 meters
        progress("# Flying north %u meters" % (side/2.0))
        self.set_rc(1, 1700)
        if not self.wait_distance(side/2, 5, 60):
            failed = True
        self.set_rc(1, 1500)

        # fly east 8 seconds
        progress("# Flying east for 8 seconds")
        self.set_rc(2, 1700)
        tstart = self.get_sim_time()
        while self.get_sim_time() < (tstart + 8):
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            delta = (self.get_sim_time() - tstart)
            # progress("%u" % delta)
        self.set_rc(2, 1500)

        # restore to default
        self.mavproxy.send('param set SIMPLE 0\n')

        # hover in place
        self.hover()
        return not failed

    # fly_super_simple - flies a circle around home for 45 seconds
    def fly_super_simple(self, timeout=45):

        failed = False

        # hold position in loiter
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # fly forward 20m
        progress("# Flying forward 20 meters")
        self.set_rc(2, 1300)
        if not self.wait_distance(20, 5, 60):
            failed = True
        self.set_rc(2, 1500)

        # set SUPER SIMPLE mode for all flight modes
        self.mavproxy.send('param set SUPER_SIMPLE 63\n')

        # switch to stabilize mode
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')
        self.set_rc(3, 1500)

        # start copter yawing slowly
        self.set_rc(4, 1550)

        # roll left for timeout seconds
        progress("# rolling left from pilot's point of view for %u seconds" % timeout)
        self.set_rc(1, 1300)
        tstart = self.get_sim_time()
        while self.get_sim_time() < (tstart + timeout):
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            delta = (self.get_sim_time() - tstart)

        # stop rolling and yawing
        self.set_rc(1, 1500)
        self.set_rc(4, 1500)

        # restore simple mode parameters to default
        self.mavproxy.send('param set SUPER_SIMPLE 0\n')

        # hover in place
        self.hover()
        return not failed

    # fly_circle - flies a circle with 20m radius
    def fly_circle(self, maxaltchange=10, holdtime=36):

        # hold position in loiter
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # face west
        progress("turn west")
        self.set_rc(4, 1580)
        if not self.wait_heading(270):
            return False
        self.set_rc(4, 1500)

        # set CIRCLE radius
        self.mavproxy.send('param set CIRCLE_RADIUS 3000\n')

        # fly forward (east) at least 100m
        self.set_rc(2, 1100)
        if not self.wait_distance(100):
            return False

        # return pitch stick back to middle
        self.set_rc(2, 1500)

        # set CIRCLE mode
        self.mavproxy.send('switch 1\n')  # circle mode
        self.wait_mode('CIRCLE')

        # wait
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_altitude = m.alt
        tstart = self.get_sim_time()
        tholdstart = self.get_sim_time()
        progress("Circle at %u meters for %u seconds" % (start_altitude, holdtime))
        while self.get_sim_time() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            progress("heading %u" % m.heading)

        progress("CIRCLE OK for %u seconds" % holdtime)
        return True

    # fly_auto_test - fly mission which tests a significant number of commands
    def fly_auto_test(self):

        # Fly mission #1
        progress("# Load copter_mission")
        # load the waypoint count
        global num_wp
        num_wp = self.load_mission_from_file(os.path.join(testdir, "copter_mission.txt"))
        if not num_wp:
            progress("load copter_mission failed")
            return False

        progress("test: Fly a mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')

        # switch into AUTO mode and raise throttle
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        self.set_rc(3, 1500)

        # fly the mission
        ret = self.wait_waypoint(0, num_wp-1, timeout=500)

        # land if mission failed
        if ret is False:
            self.land()

        # set throttle to minimum
        self.set_rc(3, 1000)

        # wait for disarm
        self.mav.motors_disarmed_wait()
        progress("MOTORS DISARMED OK")

        progress("Auto mission completed: passed=%s" % ret)

        return ret

    # fly_avc_test - fly AVC mission
    def fly_avc_test(self):

        # upload mission from file
        progress("# Load copter_AVC2013_mission")
        # load the waypoint count
        global num_wp
        num_wp = self.load_mission_from_file(os.path.join(testdir, "copter_AVC2013_mission.txt"))
        if not num_wp:
            progress("load copter_AVC2013_mission failed")
            return False

        progress("Fly AVC mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')

        # wait for motor runup
        self.wait_seconds(20)

        # switch into AUTO mode and raise throttle
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        self.set_rc(3, 1500)

        # fly the mission
        ret = self.wait_waypoint(0, num_wp-1, timeout=500)

        # set throttle to minimum
        self.set_rc(3, 1000)

        # wait for disarm
        self.mav.motors_disarmed_wait()
        progress("MOTORS DISARMED OK")

        progress("AVC mission completed: passed=%s" % ret)

        return ret

    def fly_mission(self, height_accuracy=-1.0, target_altitude=None):
        """Fly a mission from a file."""
        global num_wp
        progress("test: Fly a mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        ret = self.wait_waypoint(0, num_wp-1, timeout=500)
        progress("test: MISSION COMPLETE: passed=%s" % ret)
        # wait here until ready
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')
        return ret

    def autotest(self):
        """Autotest ArduCopter in SITL."""
        self.frame = '+'
        if not self.hasInit:
            self.init()

        failed = False
        failed_test_msg = "None"

        try:
            progress("Waiting for a heartbeat with mavlink protocol %s" % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(3, 1000)
            self.homeloc = self.mav.location()
            progress("Home location: %s" % self.homeloc)
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.mav.wait_heartbeat()
            self.wait_mode('STABILIZE')
            progress("Waiting reading for arm")
            self.wait_ready_to_arm()

            # Arm
            progress("# Arm motors")
            if not self.arm_vehicle():
                failed_test_msg = "arm_motors failed"
                progress(failed_test_msg)
                failed = True

            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Fly a square in Stabilize mode
            progress("#")
            progress("########## Fly a square and save WPs with CH7 switch ##########")
            progress("#")
            if not self.fly_square():
                failed_test_msg = "fly_square failed"
                progress(failed_test_msg)
                failed = True

            # save the stored mission to file
            progress("# Save out the CH7 mission to file")
            global num_wp
            num_wp = self.save_mission_to_file(os.path.join(testdir, "ch7_mission.txt"))
            if not num_wp:
                failed_test_msg = "save_mission_to_file failed"
                progress(failed_test_msg)
                failed = True

            # fly the stored mission
            progress("# Fly CH7 saved mission")
            if not self.fly_mission(height_accuracy=0.5, target_altitude=10):
                failed_test_msg = "fly ch7_mission failed"
                progress(failed_test_msg)
                failed = True

            # Throttle Failsafe
            progress("#")
            progress("########## Test Failsafe ##########")
            progress("#")
            if not self.fly_throttle_failsafe():
                failed_test_msg = "fly_throttle_failsafe failed"
                progress(failed_test_msg)
                failed = True

            # Takeoff
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Battery failsafe
            if not self.fly_battery_failsafe():
                failed_test_msg = "fly_battery_failsafe failed"
                progress(failed_test_msg)
                failed = True

            # Takeoff
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Stability patch
            progress("#")
            progress("########## Test Stability Patch ##########")
            progress("#")
            if not self.fly_stability_patch(30):
                failed_test_msg = "fly_stability_patch failed"
                progress(failed_test_msg)
                failed = True

            # RTL
            progress("# RTL #")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after stab patch failed"
                progress(failed_test_msg)
                failed = True

            # Takeoff
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Fence test
            progress("#")
            progress("########## Test Horizontal Fence ##########")
            progress("#")
            if not self.fly_fence_test(180):
                failed_test_msg = "fly_fence_test failed"
                progress(failed_test_msg)
                failed = True

            # Fence test
            progress("#")
            progress("########## Test Max Alt Fence ##########")
            progress("#")
            if not self.fly_alt_max_fence_test(180):
                failed_test_msg = "fly_alt_max_fence_test failed"
                progress(failed_test_msg)
                failed = True

            # Takeoff
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Fly GPS Glitch Loiter test
            progress("# GPS Glitch Loiter Test")
            if not self.fly_gps_glitch_loiter_test():
                failed_test_msg = "fly_gps_glitch_loiter_test failed"
                progress(failed_test_msg)
                failed = True

            # RTL after GPS Glitch Loiter test
            progress("# RTL #")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL failed"
                progress(failed_test_msg)
                failed = True

            # Fly GPS Glitch test in auto mode
            progress("# GPS Glitch Auto Test")
            if not self.fly_gps_glitch_auto_test():
                failed_test_msg = "fly_gps_glitch_auto_test failed"
                progress(failed_test_msg)
                failed = True

            # take-off ahead of next test
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Loiter for 10 seconds
            progress("#")
            progress("########## Test Loiter for 10 seconds ##########")
            progress("#")
            if not self.loiter():
                failed_test_msg = "loiter failed"
                progress(failed_test_msg)
                failed = True

            # Loiter Climb
            progress("#")
            progress("# Loiter - climb to 30m")
            progress("#")
            if not self.change_alt(30):
                failed_test_msg = "change_alt climb failed"
                progress(failed_test_msg)
                failed = True

            # Loiter Descend
            progress("#")
            progress("# Loiter - descend to 20m")
            progress("#")
            if not self.change_alt(20):
                failed_test_msg = "change_alt descend failed"
                progress(failed_test_msg)
                failed = True

            # RTL
            progress("#")
            progress("########## Test RTL ##########")
            progress("#")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after Loiter climb/descend failed"
                progress(failed_test_msg)
                failed = True

            # Takeoff
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Simple mode
            progress("# Fly in SIMPLE mode")
            if not self.fly_simple():
                failed_test_msg = "fly_simple failed"
                progress(failed_test_msg)
                failed = True

            # RTL
            progress("#")
            progress("########## Test RTL ##########")
            progress("#")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after simple mode failed"
                progress(failed_test_msg)
                failed = True

            # Takeoff
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Fly a circle in super simple mode
            progress("# Fly a circle in SUPER SIMPLE mode")
            if not self.fly_super_simple():
                failed_test_msg = "fly_super_simple failed"
                progress(failed_test_msg)
                failed = True

            # RTL
            progress("# RTL #")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after super simple mode failed"
                progress(failed_test_msg)
                failed = True

            # Takeoff
            progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                progress(failed_test_msg)
                failed = True

            # Circle mode
            progress("# Fly CIRCLE mode")
            if not self.fly_circle():
                failed_test_msg = "fly_circle failed"
                progress(failed_test_msg)
                failed = True

            # RTL
            progress("#")
            progress("########## Test RTL ##########")
            progress("#")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after circle failed"
                progress(failed_test_msg)
                failed = True

            progress("# Fly copter mission")
            if not self.fly_auto_test():
                failed_test_msg = "fly_auto_test failed"
                progress(failed_test_msg)
                failed = True
            else:
                progress("Flew copter mission OK")

            # wait for disarm
            self.mav.motors_disarmed_wait()

            if not self.log_download(util.reltopdir("../buildlogs/ArduCopter-log.bin")):
                failed_test_msg = "log_download failed"
                progress(failed_test_msg)
                failed = True

        except pexpect.TIMEOUT as e:
            progress("Failed with timeout")
            failed = True

        self.close()

        if failed:
            progress("FAILED: %s" % failed_test_msg)
            return False
        return True

    def autotest_heli(self):
        """Autotest Helicopter in SITL with AVC2013 mission."""
        self.frame = 'heli'
        if not self.hasInit:
            self.init()

        failed = False
        failed_test_msg = "None"

        try:
            self.mav.wait_heartbeat()
            self.set_rc_default()
            self.set_rc(3, 1000)
            self.homeloc = self.mav.location()

            progress("Lowering rotor speed")
            self.set_rc(8, 1000)
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.wait_mode('STABILIZE')
            self.wait_ready_to_arm()

            # Arm
            progress("# Arm motors")
            if not self.arm_vehicle():
                failed_test_msg = "arm_motors failed"
                progress(failed_test_msg)
                failed = True

            progress("Raising rotor speed")
            self.set_rc(8, 2000)

            progress("# Fly AVC mission")
            if not self.fly_avc_test():
                failed_test_msg = "fly_avc_test failed"
                progress(failed_test_msg)
                failed = True
            else:
                progress("Flew AVC mission OK")

            progress("Lowering rotor speed")
            self.set_rc(8, 1000)

            # mission includes disarm at end so should be ok to download logs now
            if not self.log_download(util.reltopdir("../buildlogs/Helicopter-log.bin")):
                failed_test_msg = "log_download failed"
                progress(failed_test_msg)
                failed = True

        except pexpect.TIMEOUT as failed_test_msg:
            failed_test_msg = "Timeout"
            failed = True

        self.close()

        if failed:
            progress("FAILED: %s" % failed_test_msg)
            return False
        return True
