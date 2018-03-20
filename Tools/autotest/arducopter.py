#!/usr/bin/env python

# Fly ArduCopter in SITL
from __future__ import print_function
import math
import os
import shutil

import pexpect
from pymavlink import mavutil

from pysim import util

from common import AutoTest

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


class AutoTestCopter(AutoTest):
    def __init__(self, binary,
                 valgrind=False,
                 gdb=False,
                 speedup=10,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 **kwargs):
        super(AutoTestCopter, self).__init__(**kwargs)
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

        self.log_name = "ArduCopter"
        self.logfile = None
        self.buildlog = None
        self.copy_tlog = False

        self.sitl = None
        self.hasInit = False

    def mavproxy_options(self):
        ret = super(AutoTestCopter, self).mavproxy_options()
        if self.frame != 'heli':
            ret.append('--quadcopter')
        return ret

    def sitl_streamrate(self):
        return 5

    def vehicleinfo_key(self):
        return 'ArduCopter'

    def init(self):
        if self.frame is None:
            self.frame = '+'

        if self.frame == 'heli':
            self.log_name = "HeliCopter"
            self.home = "%f,%f,%u,%u" % (AVCHOME.lat,
                                         AVCHOME.lng,
                                         AVCHOME.alt,
                                         AVCHOME.heading)

        self.apply_parameters_using_sitl()

        self.sitl = util.start_SITL(self.binary,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL(
            'ArduCopter', options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        self.logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)

        self.buildlog = self.buildlogs_path(self.log_name + "-test.tlog")
        self.progress("buildlog=%s" % self.buildlog)
        self.copy_tlog = False
        if os.path.exists(self.buildlog):
            os.unlink(self.buildlog)
        try:
            os.link(self.logfile, self.buildlog)
        except Exception:
            self.progress("WARN: Failed to create symlink: %s => %s, "
                          "will copy tlog manually to target location" %
                          (self.logfile, self.buildlog))
            self.copy_tlog = True

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
            self.progress("Failed to start mavlink connection on %s: %s" %
                          (connection_string, msg,))
            raise
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)
        self.hasInit = True
        self.progress("Ready to start testing!")

    def close(self):
        super(AutoTestCopter, self).close()

        # [2014/05/07] FC Because I'm doing a cross machine build
        # (source is on host, build is on guest VM) I cannot hard link
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
        self.progress("TAKEOFF COMPLETE")
        return True

    def land(self, timeout=60):
        """Land the quad."""
        self.progress("STARTING LANDING")
        self.mavproxy.send('switch 2\n')  # land mode
        self.wait_mode('LAND')
        self.progress("Entered Landing Mode")
        ret = self.wait_altitude(-5, 1)
        self.progress("LANDING: ok= %s" % ret)
        return ret

    def hover(self, hover_throttle=1500):
        self.set_rc(3, hover_throttle)
        return True

    # loiter - fly south west, then loiter within 5m position and altitude
    def loiter(self, holdtime=10, maxaltchange=5, maxdistchange=5):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # first aim south east
        self.progress("turn south east")
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
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))
        while self.get_sim_time() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            self.progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                self.progress("Loiter alt shifted %u meters (> limit of %u)" %
                              (alt_delta, maxaltchange))
                success = False
            if delta > maxdistchange:
                self.progress("Loiter shifted %u meters (> limit of %u)" %
                              (delta, maxdistchange))
                success = False
        if success:
            self.progress("Loiter OK for %u seconds" % holdtime)
        else:
            self.progress("Loiter FAILED")
        return success

    def change_alt(self, alt_min, climb_throttle=1920, descend_throttle=1080):
        """Change altitude."""
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        if m.alt < alt_min:
            self.progress("Rise to alt:%u from %u" % (alt_min, m.alt))
            self.set_rc(3, climb_throttle)
            self.wait_altitude(alt_min, (alt_min + 5))
        else:
            self.progress("Lower to alt:%u from %u" % (alt_min, m.alt))
            self.set_rc(3, descend_throttle)
            self.wait_altitude((alt_min - 5), alt_min)
        self.hover()
        return True

    #################################################
    #   TESTS FLY
    #################################################

    # fly a square in alt_hold mode
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
        self.progress("turn right towards north")
        self.set_rc(4, 1580)
        if not self.wait_heading(10):
            self.progress("Failed to reach heading")
            success = False
        self.set_rc(4, 1500)
        self.mav.recv_match(condition='RC_CHANNELS.chan4_raw==1500',
                            blocking=True)

        # save bottom left corner of box as waypoint
        self.progress("Save WP 1 & 2")
        self.save_wp()

        # switch back to stabilize mode
        self.set_rc(3, 1500)
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')

        # pitch forward to fly north
        self.progress("Going north %u meters" % side)
        self.set_rc(2, 1300)
        if not self.wait_distance(side):
            self.progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(2, 1500)

        # save top left corner of square as waypoint
        self.progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        self.progress("Going east %u meters" % side)
        self.set_rc(1, 1700)
        if not self.wait_distance(side):
            self.progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(1, 1500)

        # save top right corner of square as waypoint
        self.progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        self.progress("Going south %u meters" % side)
        self.set_rc(2, 1700)
        if not self.wait_distance(side):
            self.progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(2, 1500)

        # save bottom right corner of square as waypoint
        self.progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        self.progress("Going west %u meters" % side)
        self.set_rc(1, 1300)
        if not self.wait_distance(side):
            self.progress("Failed to reach distance of %u" % side)
            success = False
        self.set_rc(1, 1500)

        # save bottom left corner of square (should be near home) as waypoint
        self.progress("Save WP 6")
        self.save_wp()

        # descend to 10m
        self.progress("Descend to 10m in Loiter")
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')
        self.set_rc(3, 1300)
        time_left = timeout - (self.get_sim_time() - tstart)
        self.progress("timeleft = %u" % time_left)
        if time_left < 20:
            time_left = 20
        if not self.wait_altitude(-10, 10, time_left):
            self.progress("Failed to reach alt of 10m")
            success = False
        self.save_wp()

        return success

    def fly_RTL(self, side=60, timeout=250):
        """Return, land."""
        self.progress("# Enter RTL")
        self.mavproxy.send('switch 3\n')
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(HOME, pos)
            self.progress("Alt: %u  HomeDist: %.0f" % (m.alt, home_distance))
            if m.alt <= 1 and home_distance < 10:
                return True
        return False

    def fly_throttle_failsafe(self, side=60, timeout=180):
        """Fly east, Failsafe, return, land."""

        # switch to loiter mode temporarily to stop us from rising
        self.mavproxy.send('switch 5\n')
        self.wait_mode('LOITER')

        # first aim east
        self.progress("turn east")
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
        self.progress("# Going forward %u meters" % side)
        self.set_rc(2, 1350)
        if not self.wait_distance(side, 5, 60):
            return False
        self.set_rc(2, 1500)

        # pull throttle low
        self.progress("# Enter Failsafe")
        self.set_rc(3, 900)

        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(HOME, pos)
            self.progress("Alt: %u  HomeDist: %.0f" % (m.alt, home_distance))
            # check if we've reached home
            if m.alt <= 1 and home_distance < 10:
                # reduce throttle
                self.set_rc(3, 1100)
                # switch back to stabilize
                self.mavproxy.send('switch 2\n')  # land mode
                self.wait_mode('LAND')
                self.progress("Waiting for disarm")
                self.mav.motors_disarmed_wait()
                self.progress("Reached failsafe home OK")
                self.mavproxy.send('switch 6\n')  # stabilize mode
                self.wait_mode('STABILIZE')
                self.set_rc(3, 1000)
                if not self.arm_vehicle():
                    self.progress("Failed to re-arm")
                    return False
                return True
        self.progress("Failed to land on failsafe RTL - "
                      "timed out after %u seconds" % timeout)
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
            self.progress("Successfully entered LAND after battery failsafe")
        else:
            self.progress("Failed to enter LAND mode after battery failsafe")

        return success

    # fly_stability_patch - fly south, then hold loiter within 5m
    # position and altitude and reduce 1 motor to 60% efficiency
    def fly_stability_patch(self,
                            holdtime=30,
                            maxaltchange=5,
                            maxdistchange=10):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # first south
        self.progress("turn south")
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
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))

        # cut motor 1 to 55% efficiency
        self.progress("Cutting motor 1 to 60% efficiency")
        self.mavproxy.send('param set SIM_ENGINE_MUL 0.60\n')

        while self.get_sim_time() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            self.progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                self.progress("Loiter alt shifted %u meters (> limit of %u)" %
                              (alt_delta, maxaltchange))
                success = False
            if delta > maxdistchange:
                self.progress("Loiter shifted %u meters (> limit of %u)" %
                              (delta, maxdistchange))
                success = False

        # restore motor 1 to 100% efficiency
        self.mavproxy.send('param set SIM_ENGINE_MUL 1.0\n')

        if success:
            self.progress("Stability patch and Loiter OK for %u seconds" %
                          holdtime)
        else:
            self.progress("Stability Patch FAILED")

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
        self.progress("turn east")
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
            self.progress("Alt: %u  HomeDistance: %.0f" %
                          (m.alt, home_distance))
            # recenter pitch sticks once we're home so we don't fly off again
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
                self.progress("Waiting for disarm")
                self.mav.motors_disarmed_wait()
                self.progress("Reached home OK")
                self.mavproxy.send('switch 6\n')  # stabilize mode
                self.wait_mode('STABILIZE')
                self.set_rc(3, 1000)
                # remove if we ever clear battery failsafe flag on disarm:
                self.mavproxy.send('arm uncheck all\n')
                if not self.arm_vehicle():
                    self.progress("Failed to re-arm")
                    # remove if we ever clear battery failsafe flag on disarm:
                    self.mavproxy.send('arm check all\n')
                    return False
                # remove if we ever clear battery failsafe flag on disarm:
                self.mavproxy.send('arm check all\n')
                self.progress("Reached home OK")
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
        self.progress("Fence test failed to reach home - "
                      "timed out after %u seconds" % timeout)
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
            self.progress(failed_test_msg)
            return False

        # first east
        self.progress("turn east")
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

        self.progress("Waiting for disarm")
        self.mav.motors_disarmed_wait()

        self.set_rc(3, 1000)

        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        # remove if we ever clear battery failsafe flag on disarm
        self.mavproxy.send('arm uncheck all\n')
        if not self.arm_vehicle():
            self.progress("Failed to re-arm")
            # remove if we ever clear battery failsafe flag on disarm:
            self.mavproxy.send('arm check all\n')
            return False
        # remove if we ever clear battery failsafe flag on disarm:
        self.mavproxy.send('arm check all\n')

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
        self.progress("Apply first glitch")
        self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' %
                           glitch_lat[glitch_current])
        self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' %
                           glitch_lon[glitch_current])

        # record position for 30 seconds
        while tnow < tstart + timeout:
            tnow = self.get_sim_time()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # turn off glitching if we've reached the end of glitch list
                if glitch_current >= glitch_num:
                    glitch_current = -1
                    self.progress("Completed Glitches")
                    self.mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
                    self.mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
                else:
                    self.progress("Applying glitch %u" % glitch_current)
                    # move onto the next glitch
                    self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' %
                                       glitch_lat[glitch_current])
                    self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' %
                                       glitch_lon[glitch_current])

            # start displaying distance moved after all glitches applied
            if glitch_current == -1:
                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                curr_pos = self.sim_location()
                moved_distance = self.get_distance(curr_pos, start_pos)
                self.progress("Alt: %u  Moved: %.0f" % (m.alt, moved_distance))
                if moved_distance > max_distance:
                    self.progress("Moved over %u meters, Failed!" %
                                  max_distance)
                    success = False

        # disable gps glitch
        if glitch_current != -1:
            glitch_current = -1
            self.mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
            self.mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        if success:
            self.progress("GPS glitch test passed!"
                          "  stayed within %u meters for %u seconds" %
                          (max_distance, timeout))
        else:
            self.progress("GPS glitch test FAILED!")
        return success

    # fly_gps_glitch_auto_test - fly mission and test reaction to gps glitch
    def fly_gps_glitch_auto_test(self, timeout=120):

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
        global num_wp
        num_wp = self.load_mission("copter_glitch_mission.txt")
        if not num_wp:
            self.progress("load copter_glitch_mission failed")
            return False

        # turn on simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(True)

        self.progress("test: Fly a mission from 1 to %u" % num_wp)
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
        self.progress("Apply first glitch")
        self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' %
                           glitch_lat[glitch_current])
        self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' %
                           glitch_lon[glitch_current])

        # record position for 30 seconds
        while glitch_current < glitch_num:
            tnow = self.get_sim_time()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # apply next glitch
                if glitch_current < glitch_num:
                    self.progress("Applying glitch %u" % glitch_current)
                    self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' %
                                       glitch_lat[glitch_current])
                    self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' %
                                       glitch_lon[glitch_current])

        # turn off glitching
        self.progress("Completed Glitches")
        self.mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
        self.mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')

        # continue with the mission
        ret = self.wait_waypoint(0, num_wp-1, timeout=500)

        # wait for arrival back home
        self.mav.recv_match(type='VFR_HUD', blocking=True)
        pos = self.mav.location()
        dist_to_home = self.get_distance(HOME, pos)
        while dist_to_home > 5:
            if self.get_sim_time() > (tstart + timeout):
                self.progress("GPS Glitch testing failed"
                              "- exceeded timeout %u seconds" % timeout)
                ret = False
                break
            self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            dist_to_home = self.get_distance(HOME, pos)
            self.progress("Dist from home: %u" % dist_to_home)

        # turn off simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        self.progress("GPS Glitch test Auto completed: passed=%s" % ret)

        return ret

    #   fly_simple - assumes the simple bearing is initialised to be
    #   directly north flies a box with 100m west, 15 seconds north,
    #   50 seconds east, 15 seconds south
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
        self.progress("# Flying south %u meters" % side)
        self.set_rc(1, 1300)
        if not self.wait_distance(side, 5, 60):
            failed = True
        self.set_rc(1, 1500)

        # fly west 8 seconds
        self.progress("# Flying west for 8 seconds")
        self.set_rc(2, 1300)
        tstart = self.get_sim_time()
        while self.get_sim_time() < (tstart + 8):
            self.mav.recv_match(type='VFR_HUD', blocking=True)
        self.set_rc(2, 1500)

        # fly north 25 meters
        self.progress("# Flying north %u meters" % (side/2.0))
        self.set_rc(1, 1700)
        if not self.wait_distance(side/2, 5, 60):
            failed = True
        self.set_rc(1, 1500)

        # fly east 8 seconds
        self.progress("# Flying east for 8 seconds")
        self.set_rc(2, 1700)
        tstart = self.get_sim_time()
        while self.get_sim_time() < (tstart + 8):
            self.mav.recv_match(type='VFR_HUD', blocking=True)
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
        self.progress("# Flying forward 20 meters")
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
        self.progress("# rolling left from pilot's POV for %u seconds"
                      % timeout)
        self.set_rc(1, 1300)
        tstart = self.get_sim_time()
        while self.get_sim_time() < (tstart + timeout):
            self.mav.recv_match(type='VFR_HUD', blocking=True)

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
        self.progress("turn west")
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
        self.progress("Circle at %u meters for %u seconds" %
                      (start_altitude, holdtime))
        while self.get_sim_time() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("heading %u" % m.heading)

        self.progress("CIRCLE OK for %u seconds" % holdtime)
        return True

    # fly_auto_test - fly mission which tests a significant number of commands
    def fly_auto_test(self):

        # Fly mission #1
        self.progress("# Load copter_mission")
        # load the waypoint count
        global num_wp
        num_wp = self.load_mission("copter_mission.txt")
        if not num_wp:
            self.progress("load copter_mission failed")
            return False

        self.progress("test: Fly a mission from 1 to %u" % num_wp)
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
        self.progress("MOTORS DISARMED OK")

        self.progress("Auto mission completed: passed=%s" % ret)

        return ret

    def load_mission(self, mission):
        path = os.path.join(testdir, mission)
        return self.load_mission_from_file(path)

    # fly_avc_test - fly AVC mission
    def fly_avc_test(self):

        # upload mission from file
        self.progress("# Load copter_AVC2013_mission")
        # load the waypoint count
        global num_wp
        num_wp = self.load_mission("copter_AVC2013_mission.txt")
        if not num_wp:
            self.progress("load copter_AVC2013_mission failed")
            return False

        self.progress("Fly AVC mission from 1 to %u" % num_wp)
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
        self.progress("MOTORS DISARMED OK")

        self.progress("AVC mission completed: passed=%s" % ret)

        return ret

    def fly_mission(self, height_accuracy=-1.0, target_altitude=None):
        """Fly a mission from a file."""
        global num_wp
        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        ret = self.wait_waypoint(0, num_wp-1, timeout=500)
        self.progress("test: MISSION COMPLETE: passed=%s" % ret)
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
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(3, 1000)
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.mav.wait_heartbeat()
            self.wait_mode('STABILIZE')
            self.progress("Waiting reading for arm")
            self.wait_ready_to_arm()

            # Arm
            self.progress("# Arm motors")
            if not self.arm_vehicle():
                failed_test_msg = "arm_motors failed"
                self.progress(failed_test_msg)
                failed = True

            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Fly a square in Stabilize mode
            self.progress("#")
            self.progress("########## Fly a square and save WPs with CH7"
                          " switch ##########")
            self.progress("#")
            if not self.fly_square():
                failed_test_msg = "fly_square failed"
                self.progress(failed_test_msg)
                failed = True

            # save the stored mission to file
            self.progress("# Save out the CH7 mission to file")
            global num_wp
            num_wp = self.save_mission_to_file(os.path.join(testdir,
                                                            "ch7_mission.txt"))
            if not num_wp:
                failed_test_msg = "save_mission_to_file failed"
                self.progress(failed_test_msg)
                failed = True

            # fly the stored mission
            self.progress("# Fly CH7 saved mission")
            if not self.fly_mission(height_accuracy=0.5, target_altitude=10):
                failed_test_msg = "fly ch7_mission failed"
                self.progress(failed_test_msg)
                failed = True

            # Throttle Failsafe
            self.progress("#")
            self.progress("########## Test Failsafe ##########")
            self.progress("#")
            if not self.fly_throttle_failsafe():
                failed_test_msg = "fly_throttle_failsafe failed"
                self.progress(failed_test_msg)
                failed = True

            # Takeoff
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Battery failsafe
            if not self.fly_battery_failsafe():
                failed_test_msg = "fly_battery_failsafe failed"
                self.progress(failed_test_msg)
                failed = True

            # Takeoff
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Stability patch
            self.progress("#")
            self.progress("########## Test Stability Patch ##########")
            self.progress("#")
            if not self.fly_stability_patch(30):
                failed_test_msg = "fly_stability_patch failed"
                self.progress(failed_test_msg)
                failed = True

            # RTL
            self.progress("# RTL #")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after stab patch failed"
                self.progress(failed_test_msg)
                failed = True

            # Takeoff
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Fence test
            self.progress("#")
            self.progress("########## Test Horizontal Fence ##########")
            self.progress("#")
            if not self.fly_fence_test(180):
                failed_test_msg = "fly_fence_test failed"
                self.progress(failed_test_msg)
                failed = True

            # Fence test
            self.progress("#")
            self.progress("########## Test Max Alt Fence ##########")
            self.progress("#")
            if not self.fly_alt_max_fence_test(180):
                failed_test_msg = "fly_alt_max_fence_test failed"
                self.progress(failed_test_msg)
                failed = True

            # Takeoff
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Fly GPS Glitch Loiter test
            self.progress("# GPS Glitch Loiter Test")
            if not self.fly_gps_glitch_loiter_test():
                failed_test_msg = "fly_gps_glitch_loiter_test failed"
                self.progress(failed_test_msg)
                failed = True

            # RTL after GPS Glitch Loiter test
            self.progress("# RTL #")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL failed"
                self.progress(failed_test_msg)
                failed = True

            # Fly GPS Glitch test in auto mode
            self.progress("# GPS Glitch Auto Test")
            if not self.fly_gps_glitch_auto_test():
                failed_test_msg = "fly_gps_glitch_auto_test failed"
                self.progress(failed_test_msg)
                failed = True

            # take-off ahead of next test
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Loiter for 10 seconds
            self.progress("#")
            self.progress("########## Test Loiter for 10 seconds ##########")
            self.progress("#")
            if not self.loiter():
                failed_test_msg = "loiter failed"
                self.progress(failed_test_msg)
                failed = True

            # Loiter Climb
            self.progress("#")
            self.progress("# Loiter - climb to 30m")
            self.progress("#")
            if not self.change_alt(30):
                failed_test_msg = "change_alt climb failed"
                self.progress(failed_test_msg)
                failed = True

            # Loiter Descend
            self.progress("#")
            self.progress("# Loiter - descend to 20m")
            self.progress("#")
            if not self.change_alt(20):
                failed_test_msg = "change_alt descend failed"
                self.progress(failed_test_msg)
                failed = True

            # RTL
            self.progress("#")
            self.progress("########## Test RTL ##########")
            self.progress("#")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after Loiter climb/descend failed"
                self.progress(failed_test_msg)
                failed = True

            # Takeoff
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Simple mode
            self.progress("# Fly in SIMPLE mode")
            if not self.fly_simple():
                failed_test_msg = "fly_simple failed"
                self.progress(failed_test_msg)
                failed = True

            # RTL
            self.progress("#")
            self.progress("########## Test RTL ##########")
            self.progress("#")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after simple mode failed"
                self.progress(failed_test_msg)
                failed = True

            # Takeoff
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Fly a circle in super simple mode
            self.progress("# Fly a circle in SUPER SIMPLE mode")
            if not self.fly_super_simple():
                failed_test_msg = "fly_super_simple failed"
                self.progress(failed_test_msg)
                failed = True

            # RTL
            self.progress("# RTL #")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after super simple mode failed"
                self.progress(failed_test_msg)
                failed = True

            # Takeoff
            self.progress("# Takeoff")
            if not self.takeoff(10):
                failed_test_msg = "takeoff failed"
                self.progress(failed_test_msg)
                failed = True

            # Circle mode
            self.progress("# Fly CIRCLE mode")
            if not self.fly_circle():
                failed_test_msg = "fly_circle failed"
                self.progress(failed_test_msg)
                failed = True

            # RTL
            self.progress("#")
            self.progress("########## Test RTL ##########")
            self.progress("#")
            if not self.fly_RTL():
                failed_test_msg = "fly_RTL after circle failed"
                self.progress(failed_test_msg)
                failed = True

            self.progress("# Fly copter mission")
            if not self.fly_auto_test():
                failed_test_msg = "fly_auto_test failed"
                self.progress(failed_test_msg)
                failed = True
            else:
                self.progress("Flew copter mission OK")

            # wait for disarm
            self.mav.motors_disarmed_wait()

            log_filepath = self.buildlogs_path("ArduCopter-log.bin")
            if not self.log_download(log_filepath):
                failed_test_msg = "log_download failed"
                self.progress(failed_test_msg)
                failed = True

        except pexpect.TIMEOUT as e:
            self.progress("Failed with timeout")
            failed = True

        self.close()

        if failed:
            self.progress("FAILED: %s" % failed_test_msg)
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

            self.progress("Lowering rotor speed")
            self.set_rc(8, 1000)
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.wait_mode('STABILIZE')
            self.wait_ready_to_arm()

            # Arm
            self.progress("# Arm motors")
            if not self.arm_vehicle():
                failed_test_msg = "arm_motors failed"
                self.progress(failed_test_msg)
                failed = True

            self.progress("Raising rotor speed")
            self.set_rc(8, 2000)

            self.progress("# Fly AVC mission")
            if not self.fly_avc_test():
                failed_test_msg = "fly_avc_test failed"
                self.progress(failed_test_msg)
                failed = True
            else:
                self.progress("Flew AVC mission OK")

            self.progress("Lowering rotor speed")
            self.set_rc(8, 1000)

            # mission ends with disarm so should be ok to download logs now
            log_path = self.buildlogs_path("Helicopter-log.bin")
            if not self.log_download(log_path):
                failed_test_msg = "log_download failed"
                self.progress(failed_test_msg)
                failed = True

        except pexpect.TIMEOUT as failed_test_msg:
            failed_test_msg = "Timeout"
            failed = True

        self.close()

        if failed:
            self.progress("FAILED: %s" % failed_test_msg)
            return False
        return True
