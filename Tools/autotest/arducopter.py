#!/usr/bin/env python

# Fly ArduCopter in SITL
from __future__ import print_function
import math
import os
import shutil
import time

import pexpect
from pymavlink import mavutil

from pysim import util

from common import AutoTest
from common import NotAchievedException, AutoTestTimeoutException, PreconditionFailedException

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
                 breakpoints=[],
                 **kwargs):
        super(AutoTestCopter, self).__init__(**kwargs)
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

        self.log_name = "ArduCopter"

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

        self.sitl = util.start_SITL(self.binary,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver,
                                    breakpoints=self.breakpoints,
                                    vicon=True,
                                    wipe=True)
        self.mavproxy = util.start_MAVProxy_SITL(
            'ArduCopter', options=self.mavproxy_options())

        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        self.logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)
        self.try_symlink_tlog()

        self.progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        self.apply_defaultfile_parameters()

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Started simulator")

        self.get_mavlink_connection_going()

        self.hasInit = True
        self.progress("Ready to start testing!")

    def close(self):
        super(AutoTestCopter, self).close()

        # [2014/05/07] FC Because I'm doing a cross machine build
        # (source is on host, build is on guest VM) I cannot hard link
        # This flag tells me that I need to copy the data out
        if self.copy_tlog:
            shutil.copy(self.logfile, self.buildlog)

    def user_takeoff(self, alt_min=30):
        '''takeoff using mavlink takeoff command'''
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, # param1
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     alt_min # param7
        )
        self.progress("Ran command")
        self.wait_for_alt(alt_min)

    def takeoff(self, alt_min=30, takeoff_throttle=1700):
        """Takeoff get to 30m altitude."""
        self.progress("TAKEOFF")
        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        if not self.armed():
            self.progress("Waiting reading for arm")
            self.wait_ready_to_arm()
            self.set_rc(3, 1000)
            self.arm_vehicle()
        self.set_rc(3, takeoff_throttle)
        self.wait_for_alt(alt_min=alt_min)
        self.hover()
        self.progress("TAKEOFF COMPLETE")

    def wait_for_alt(self, alt_min=30):
        """Wait for altitude to be reached."""
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = m.relative_alt / 1000.0 # mm -> m
        if alt < alt_min:
            self.wait_altitude(alt_min - 1,
                               (alt_min + 5),
                               relative=True)

    def land(self, timeout=60):
        """Land the quad."""
        self.progress("STARTING LANDING")
        self.mavproxy.send('switch 2\n')  # land mode
        self.wait_mode('LAND', timeout=timeout)
        self.progress("Entered Landing Mode")
        self.wait_altitude(-5, 1, relative=True, timeout=timeout)
        self.progress("LANDING: ok!")

    def hover(self, hover_throttle=1500):
        self.set_rc(3, hover_throttle)

    # loiter - fly south west, then loiter within 5m position and altitude
    def loiter(self, holdtime=10, maxaltchange=5, maxdistchange=5):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # first aim south east
        self.progress("turn south east")
        self.set_rc(4, 1580)
        self.wait_heading(170)
        self.set_rc(4, 1500)

        # fly south east 50m
        self.set_rc(2, 1100)
        self.wait_distance(50)
        self.set_rc(2, 1500)

        # wait for copter to slow moving
        self.wait_groundspeed(0, 2)

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
                raise NotAchievedException()
            if delta > maxdistchange:
                self.progress("Loiter shifted %u meters (> limit of %u)" %
                              (delta, maxdistchange))
                raise NotAchievedException()
        self.progress("Loiter OK for %u seconds" % holdtime)

    def change_alt(self, alt_min, climb_throttle=1920, descend_throttle=1080):
        """Change altitude."""
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = m.relative_alt / 1000.0 # mm -> m
        if alt < alt_min:
            self.progress("Rise to alt:%u from %u" % (alt_min, alt))
            self.set_rc(3, climb_throttle)
            self.wait_altitude(alt_min, (alt_min + 5), relative=True)
        else:
            self.progress("Lower to alt:%u from %u" % (alt_min, alt))
            self.set_rc(3, descend_throttle)
            self.wait_altitude((alt_min - 5), alt_min, relative=True)
        self.hover()

    #################################################
    #   TESTS FLY
    #################################################

    # fly a square in alt_hold mode
    def fly_square(self, side=50, timeout=300):
        """Fly a square, flying N then E ."""
        tstart = self.get_sim_time()

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
        self.wait_heading(10)
        self.set_rc(4, 1500)
        self.mav.recv_match(condition='RC_CHANNELS.chan4_raw==1500',
                            blocking=True)

        # save bottom left corner of box as waypoint
        self.progress("Save WP 1 & 2")
        self.save_wp()

        # switch back to stabilize mode
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')

        # increase throttle a bit because we're about to pitch:
        self.set_rc(3, 1525)

        # pitch forward to fly north
        self.progress("Going north %u meters" % side)
        self.set_rc(2, 1300)
        self.wait_distance(side)
        self.set_rc(2, 1500)

        # save top left corner of square as waypoint
        self.progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        self.progress("Going east %u meters" % side)
        self.set_rc(1, 1700)
        self.wait_distance(side)
        self.set_rc(1, 1500)

        # save top right corner of square as waypoint
        self.progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        self.progress("Going south %u meters" % side)
        self.set_rc(2, 1700)
        self.wait_distance(side)
        self.set_rc(2, 1500)

        # save bottom right corner of square as waypoint
        self.progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        self.progress("Going west %u meters" % side)
        self.set_rc(1, 1300)
        self.wait_distance(side)
        self.set_rc(1, 1500)

        # save bottom left corner of square (should be near home) as waypoint
        self.progress("Save WP 6")
        self.save_wp()

        # reduce throttle again
        self.set_rc(3, 1500)

        # descend to 10m
        self.progress("Descend to 10m in Loiter")
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')
        self.set_rc(3, 1300)
        time_left = timeout - (self.get_sim_time() - tstart)
        self.progress("timeleft = %u" % time_left)
        if time_left < 20:
            time_left = 20
        self.wait_altitude(-10, 10, time_left, relative=True)
        self.set_rc(3, 1500)
        self.save_wp()

    # enter RTL mode and wait for the vehicle to disarm
    def fly_RTL(self, timeout=250):
        """Return, land."""
        self.progress("# Enter RTL")
        self.mavproxy.send('switch 3\n')
        self.set_rc(3, 1500)
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = m.relative_alt / 1000.0 # mm -> m
            pos = self.mav.location() # requires a GPS fix to function
            home_distance = self.get_distance(HOME, pos)
            home = ""
            if alt <= 1 and home_distance < 10:
                home = "HOME"
            self.progress("Alt: %u  HomeDist: %.0f %s" %
                          (alt, home_distance, home))
            # our post-condition is that we are disarmed:
            if not self.armed():
                return
        raise AutoTestTimeoutException()

    def fly_throttle_failsafe(self, side=60, timeout=180):
        """Fly east, Failsafe, return, land."""

        # switch to loiter mode temporarily to stop us from rising
        self.mavproxy.send('switch 5\n')
        self.wait_mode('LOITER')

        # first aim east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(135)
        self.set_rc(4, 1500)

        # raise throttle slightly to avoid hitting the ground
        self.set_rc(3, 1800)
        self.wait_altitude(20, 25, relative=True)

        # switch to stabilize mode
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')
        self.hover()

        # fly east 60 meters
        self.progress("# Going forward %u meters" % side)
        self.set_rc(2, 1350)
        self.wait_distance(side, 5, 60)
        self.set_rc(2, 1500)

        # pull throttle low
        self.progress("# Enter Failsafe")
        self.set_rc(3, 900)

        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = m.relative_alt / 1000.0 # mm -> m
            pos = self.mav.location()
            home_distance = self.get_distance(HOME, pos)
            self.progress("Alt: %u  HomeDist: %.0f" % (alt, home_distance))
            # check if we've reached home
            if alt <= 1 and home_distance < 10:
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
                self.arm_vehicle()
                return
        self.progress("Failed to land on failsafe RTL - "
                      "timed out after %u seconds" % timeout)
        # reduce throttle
        self.set_rc(3, 1100)
        # switch back to stabilize mode
        self.mavproxy.send('switch 2\n')  # land mode
        self.wait_mode('LAND')
        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        raise AutoTestTimeoutException()

    def fly_battery_failsafe(self, timeout=300):
        # switch to loiter mode so that we hold position
        self.mavproxy.send('switch 5\n')
        self.wait_mode('LOITER')
        self.set_rc(3, 1500)

        # enable battery failsafe
        self.set_parameter('BATT_FS_LOW_ACT', 1)

        # trigger low voltage
        self.set_parameter('SIM_BATT_VOLTAGE', 10)

        # wait for LAND mode. If unsuccessful an exception will be raised
        self.wait_mode('LAND', timeout=timeout)

        # disable battery failsafe
        self.set_parameter('BATT_FS_LOW_ACT', 0)
        self.set_parameter('SIM_BATT_VOLTAGE', 13)

        self.progress("Successfully entered LAND after battery failsafe")
        self.reboot_sitl()

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
        self.wait_heading(180)
        self.set_rc(4, 1500)

        # fly west 80m
        self.set_rc(2, 1100)
        self.wait_distance(80)
        self.set_rc(2, 1500)

        # wait for copter to slow moving
        self.wait_groundspeed(0, 2)

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_altitude = m.alt
        start = self.mav.location()
        tstart = self.get_sim_time()
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))

        # cut motor 1 to 55% efficiency
        self.progress("Cutting motor 1 to 60% efficiency")
        self.set_parameter("SIM_ENGINE_MUL", 0.60)

        while self.get_sim_time() < tstart + holdtime:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            self.progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                self.progress("Loiter alt shifted %u meters (> limit of %u)" %
                              (alt_delta, maxaltchange))
                raise NotAchievedException()
            if delta > maxdistchange:
                self.progress("Loiter shifted %u meters (> limit of %u)" %
                              (delta, maxdistchange))
                raise NotAchievedException()

        # restore motor 1 to 100% efficiency
        self.set_parameter("SIM_ENGINE_MUL", 1.0)

        self.progress("Stability patch and Loiter OK for %us" % holdtime)

    # fly_fence_test - fly east until you hit the horizontal circular fence
    def fly_fence_test(self, timeout=180):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # enable fence, disable avoidance
        self.set_parameter("FENCE_ENABLE", 1)
        self.set_parameter("AVOID_ENABLE", 0)

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160)
        self.set_rc(4, 1500)

        # fly forward (east) at least 20m
        pitching_forward = True
        self.set_rc(2, 1100)
        self.wait_distance(20)

        # start timer
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = m.relative_alt / 1000.0 # mm -> m
            pos = self.mav.location()
            home_distance = self.get_distance(HOME, pos)
            self.progress("Alt: %f  HomeDistance: %.0f" %
                          (alt, home_distance))
            # recenter pitch sticks once we're home so we don't fly off again
            if pitching_forward and home_distance < 10:
                pitching_forward = False
                self.set_rc(2, 1475)
                # disable fence
                self.set_parameter("FENCE_ENABLE", 0)
            if alt <= 1 and home_distance < 10:
                # reduce throttle
                self.set_rc(3, 1000)
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
                self.arm_vehicle()
                # remove if we ever clear battery failsafe flag on disarm:
                self.mavproxy.send('arm check all\n')
                self.progress("Reached home OK")
                return

        # disable fence, enable avoidance
        self.set_parameter("FENCE_ENABLE", 0)
        self.set_parameter("AVOID_ENABLE", 1)

        # reduce throttle
        self.set_rc(3, 1000)
        # switch mode to stabilize
        self.mavproxy.send('switch 2\n')  # land mode
        self.wait_mode('LAND')
        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        self.progress("Fence test failed to reach home - "
                      "timed out after %u seconds" % timeout)
        raise AutoTestTimeoutException()

    # fly_alt_fence_test - fly up until you hit the fence
    def fly_alt_max_fence_test(self):
        """Hold loiter position."""
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # enable fence, disable avoidance
        self.set_parameter("FENCE_ENABLE", 1)
        self.set_parameter("AVOID_ENABLE", 0)
        self.set_parameter("FENCE_TYPE", 1)

        self.change_alt(10)

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160)
        self.set_rc(4, 1500)

        # fly forward (east) at least 20m
        self.set_rc(2, 1100)
        self.wait_distance(20)

        # stop flying forward and start flying up:
        self.set_rc(2, 1500)
        self.set_rc(3, 1800)

        # wait for fence to trigger
        self.wait_mode('RTL', timeout=120)

        self.progress("Waiting for disarm")
        self.mav.motors_disarmed_wait()

        self.set_rc(3, 1000)

        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.wait_mode('STABILIZE')
        # remove if we ever clear battery failsafe flag on disarm
        self.mavproxy.send('arm uncheck all\n')
        self.arm_vehicle()
        # remove if we ever clear battery failsafe flag on disarm:
        self.mavproxy.send('arm check all\n')

    def fly_gps_glitch_loiter_test(self, timeout=30, max_distance=20):
        """fly_gps_glitch_loiter_test. Fly south east in loiter and test
        reaction to gps glitch."""
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
        try:
            self.wait_heading(150)
            self.set_rc(4, 1500)
            # fly forward (south east) at least 60m
            self.set_rc(2, 1100)
            self.wait_distance(60)
            self.set_rc(2, 1500)
            # wait for copter to slow down
        except Exception as e:
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            raise e

        # record time and position
        tstart = self.get_sim_time()
        tnow = tstart
        start_pos = self.sim_location()

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
                    self.set_parameter("SIM_GPS_GLITCH_X", 0)
                    self.set_parameter("SIM_GPS_GLITCH_Y", 0)
                else:
                    self.progress("Applying glitch %u" % glitch_current)
                    # move onto the next glitch
                    self.mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' %
                                       glitch_lat[glitch_current])
                    self.mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' %
                                       glitch_lon[glitch_current])

            # start displaying distance moved after all glitches applied
            if glitch_current == -1:
                m = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                        blocking=True)
                alt = m.alt/1000.0 # mm -> m
                curr_pos = self.sim_location()
                moved_distance = self.get_distance(curr_pos, start_pos)
                self.progress("Alt: %u  Moved: %.0f" % (alt, moved_distance))
                if moved_distance > max_distance:
                    self.progress("Moved over %u meters, Failed!" %
                                  max_distance)
                    raise NotAchievedException()

        # disable gps glitch
        if glitch_current != -1:
            self.set_parameter("SIM_GPS_GLITCH_X", 0)
            self.set_parameter("SIM_GPS_GLITCH_Y", 0)
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        self.progress("GPS glitch test passed!"
                      "  stayed within %u meters for %u seconds" %
                      (max_distance, timeout))

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
            raise NotAchievedException()

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
        try:
            self.wait_distance(100, 5, 60)
        except Exception as e:
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            raise e

        # record time and position
        tstart = self.get_sim_time()

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
        self.set_parameter("SIM_GPS_GLITCH_X", 0)
        self.set_parameter("SIM_GPS_GLITCH_Y", 0)

        # continue with the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # wait for arrival back home
        self.mav.recv_match(type='VFR_HUD', blocking=True)
        pos = self.mav.location()
        dist_to_home = self.get_distance(HOME, pos)
        while dist_to_home > 5:
            if self.get_sim_time() > (tstart + timeout):
                self.progress("GPS Glitch testing failed"
                              "- exceeded timeout %u seconds" % timeout)
                raise AutoTestTimeoutException()

            self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            dist_to_home = self.get_distance(HOME, pos)
            self.progress("Dist from home: %u" % dist_to_home)

        # turn off simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        self.progress("GPS Glitch test Auto completed: passed!")

    #   fly_simple - assumes the simple bearing is initialised to be
    #   directly north flies a box with 100m west, 15 seconds north,
    #   50 seconds east, 15 seconds south
    def fly_simple(self, side=50):

        # hold position in loiter
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # set SIMPLE mode for all flight modes
        self.set_parameter("SIMPLE", 63)

        # switch to stabilize mode
        self.mavproxy.send('switch 6\n')
        self.wait_mode('STABILIZE')
        self.set_rc(3, 1500)

        # fly south 50m
        self.progress("# Flying south %u meters" % side)
        self.set_rc(1, 1300)
        self.wait_distance(side, 5, 60)
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
        self.wait_distance(side/2, 5, 60)
        self.set_rc(1, 1500)

        # fly east 8 seconds
        self.progress("# Flying east for 8 seconds")
        self.set_rc(2, 1700)
        tstart = self.get_sim_time()
        while self.get_sim_time() < (tstart + 8):
            self.mav.recv_match(type='VFR_HUD', blocking=True)
        self.set_rc(2, 1500)

        # restore to default
        self.set_parameter("SIMPLE", 0)

        # hover in place
        self.hover()

    # fly_super_simple - flies a circle around home for 45 seconds
    def fly_super_simple(self, timeout=45):

        # hold position in loiter
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # fly forward 20m
        self.progress("# Flying forward 20 meters")
        self.set_rc(2, 1300)
        self.wait_distance(20, 5, 60)
        self.set_rc(2, 1500)

        # set SUPER SIMPLE mode for all flight modes
        self.set_parameter("SUPER_SIMPLE", 63)

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
        self.set_parameter("SUPER_SIMPLE", 0)

        # hover in place
        self.hover()

    # fly_circle - flies a circle with 20m radius
    def fly_circle(self, holdtime=36):
        # hold position in loiter
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        # face west
        self.progress("turn west")
        self.set_rc(4, 1580)
        self.wait_heading(270)
        self.set_rc(4, 1500)

        # set CIRCLE radius
        self.set_parameter("CIRCLE_RADIUS", 3000)

        # fly forward (east) at least 100m
        self.set_rc(2, 1100)
        self.wait_distance(100)
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

    # fly_autotune - autotune the virtual vehicle
    def fly_autotune(self):

        # hold position in loiter
        self.mavproxy.send('mode autotune\n')
        self.wait_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        sim_time_expected = 5000
        deadline = tstart + sim_time_expected
        while self.get_sim_time_cached() < deadline:
            now = self.get_sim_time_cached()
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("STATUSTEXT (%u<%u): %s" % (now, deadline, m.text))
            if "AutoTune: Success" in m.text:
                self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
                # near enough for now:
                return

        self.progress("AUTOTUNE failed (%u seconds)" %
                      (self.get_sim_time() - tstart))
        raise NotAchievedException()

    # fly_auto_test - fly mission which tests a significant number of commands
    def fly_auto_test(self):
        # Fly mission #1
        self.progress("# Load copter_mission")
        # load the waypoint count
        global num_wp
        num_wp = self.load_mission("copter_mission.txt")
        if not num_wp:
            self.progress("load copter_mission failed")
            raise NotAchievedException()

        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')

        # switch into AUTO mode and raise throttle
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # set throttle to minimum
        self.set_rc(3, 1000)

        # wait for disarm
        self.mav.motors_disarmed_wait()
        self.progress("MOTORS DISARMED OK")

        self.progress("Auto mission completed: passed!")

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
            raise NotAchievedException()

        self.progress("Fly AVC mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')

        # wait for motor runup
        self.wait_seconds(20)

        # switch into AUTO mode and raise throttle
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # set throttle to minimum
        self.set_rc(3, 1000)

        # wait for disarm
        self.mav.motors_disarmed_wait()
        self.progress("MOTORS DISARMED OK")

        self.progress("AVC mission completed: passed!")

    def fly_motor_fail(self, fail_servo=0, fail_mul=0.0, holdtime=30):
        """Test flight with reduced motor efficiency"""

        # we only expect an octocopter to survive ATM:
        servo_counts = {
            # 2: 6, # hexa
            3: 8,  # octa
            # 5: 6, # Y6
        }
        frame_class = int(self.get_parameter("FRAME_CLASS"))
        if frame_class not in servo_counts:
            self.progress("Test not relevant for frame_class %u" % frame_class)
            return

        servo_count = servo_counts[frame_class]

        if fail_servo < 0 or fail_servo > servo_count:
            raise ValueError('fail_servo outside range for frame class')

        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')

        self.change_alt(alt_min=50)

        # Get initial values
        start_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
        start_attitude = self.mav.recv_match(type='ATTITUDE', blocking=True)

        hover_time = 5
        try:
            tstart = self.get_sim_time()
            int_error_alt = 0
            int_error_yaw_rate = 0
            int_error_yaw = 0
            self.progress("Hovering for %u seconds" % hover_time)
            failed = False
            while self.get_sim_time() < tstart + holdtime + hover_time:
                ti = self.get_sim_time()

                servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                            blocking=True)
                hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
                attitude = self.mav.recv_match(type='ATTITUDE', blocking=True)

                if not failed and self.get_sim_time() - tstart > hover_time:
                    self.progress("Killing motor %u (%u%%)" %
                                  (fail_servo+1, fail_mul))
                    self.set_parameter("SIM_ENGINE_FAIL", fail_servo)
                    self.set_parameter("SIM_ENGINE_MUL", fail_mul)
                    failed = True

                if failed:
                    self.progress("Hold Time: %f/%f" %
                                  (self.get_sim_time()-tstart, holdtime))

                servo_pwm = [servo.servo1_raw,
                             servo.servo2_raw,
                             servo.servo3_raw,
                             servo.servo4_raw,
                             servo.servo5_raw,
                             servo.servo6_raw,
                             servo.servo7_raw,
                             servo.servo8_raw]

                self.progress("PWM output per motor")
                for i, pwm in enumerate(servo_pwm[0:servo_count]):
                    if pwm > 1900:
                        state = "oversaturated"
                    elif pwm < 1200:
                        state = "undersaturated"
                    else:
                        state = "OK"

                    if failed and i == fail_servo:
                        state += " (failed)"

                    self.progress("servo %u [pwm=%u] [%s]" % (i+1, pwm, state))

                alt_delta = hud.alt - start_hud.alt
                yawrate_delta = attitude.yawspeed - start_attitude.yawspeed
                yaw_delta = attitude.yaw - start_attitude.yaw

                self.progress("Alt=%fm (delta=%fm)" % (hud.alt, alt_delta))
                self.progress("Yaw rate=%f (delta=%f) (rad/s)" %
                              (attitude.yawspeed, yawrate_delta))
                self.progress("Yaw=%f (delta=%f) (deg)" %
                              (attitude.yaw, yaw_delta))

                dt = self.get_sim_time() - ti
                int_error_alt += abs(alt_delta/dt)
                int_error_yaw_rate += abs(yawrate_delta/dt)
                int_error_yaw += abs(yaw_delta/dt)
                self.progress("## Error Integration ##")
                self.progress("  Altitude: %fm" % int_error_alt)
                self.progress("  Yaw rate: %f rad/s" % int_error_yaw_rate)
                self.progress("  Yaw: %f deg" % int_error_yaw)
                self.progress("----")

                if alt_delta < -20:
                    self.progress("Vehicle is descending")
                    raise NotAchievedException()

            self.set_parameter("SIM_ENGINE_FAIL", 0)
            self.set_parameter("SIM_ENGINE_MUL", 1.0)
        except Exception as e:
            self.set_parameter("SIM_ENGINE_FAIL", 0)
            self.set_parameter("SIM_ENGINE_MUL", 1.0)
            raise e

        return True

    def fly_mission(self):
        """Fly a mission from a file."""
        global num_wp
        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.mavproxy.send('wp set 1\n')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        self.wait_waypoint(0, num_wp-1, timeout=500)
        self.progress("test: MISSION COMPLETE: passed!")
        # wait here until ready
        self.mavproxy.send('switch 5\n')  # loiter mode
        self.wait_mode('LOITER')
        self.set_rc(3, 1500)

    def fly_vision_position(self):
        """Disable GPS navigation, enable Vicon input."""
        # scribble down a location we can set origin to:

        self.progress("Waiting for location")
        self.mavproxy.send('switch 6\n')  # stabilize mode
        self.mav.wait_heartbeat()
        self.wait_mode('STABILIZE')
        self.progress("Waiting reading for arm")
        self.wait_ready_to_arm()

        old_pos = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print("old_pos=%s" % str(old_pos))

        self.context_push()

        ex = None
        try:
            self.set_parameter("GPS_TYPE", 0)
            self.set_parameter("EK2_GPS_TYPE", 3)
            self.set_parameter("SERIAL5_PROTOCOL", 1)
            self.reboot_sitl()
            # without a GPS or some sort of external prompting, AP
            # doesn't send system_time messages.  So prompt it:
            self.mav.mav.system_time_send(time.time() * 1000000, 0)
            self.mav.mav.set_gps_global_origin_send(1,
                                                    old_pos.lat,
                                                    old_pos.lon,
                                                    old_pos.alt)
            self.progress("Waiting for non-zero-lat")
            tstart = self.get_sim_time()
            while True:
                gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True)
                # self.progress("gpi=%s" % str(gpi))
                if gpi.lat != 0:
                    break

                if self.get_sim_time() - tstart > 10:
                    raise AutoTestTimeoutException()

            self.takeoff()
            self.set_rc(1, 1600)
            tstart = self.get_sim_time()
            while True:
                vicon_pos = self.mav.recv_match(type='VICON_POSITION_ESTIMATE',
                                                blocking=True)
                # print("vpe=%s" % str(vicon_pos))
                self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                    blocking=True)
                # self.progress("gpi=%s" % str(gpi))
                if vicon_pos.x > 40:
                    break

                if self.get_sim_time() - tstart > 100:
                    raise AutoTestTimeoutException()

            # recenter controls:
            self.set_rc(1, 1500)
            self.progress("# Enter RTL")
            self.mavproxy.send('switch 3\n')
            self.set_rc(3, 1500)
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 200:
                    raise NotAchievedException()
                self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                    blocking=True)
                # print("gpi=%s" % str(gpi))
                self.mav.recv_match(type='SIMSTATE',
                                    blocking=True)
                # print("ss=%s" % str(ss))
                # wait for RTL disarm:
                if not self.armed():
                    break

        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.context_pop()
        self.set_rc(3, 1000)
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def fly_nav_delay(self):
        """Fly a simple mission that has a delay in it."""

        self.load_mission("copter_nav_delay.txt")

        self.mavproxy.send('mode loiter\n')
        self.mav.wait_heartbeat()
        self.wait_mode('LOITER')
        self.progress("Waiting reading for arm")
        self.wait_ready_to_arm()

        self.context_push()

        ex = None
        try:
            self.arm_vehicle()
            self.mavproxy.send('mode auto\n')
            self.wait_mode('AUTO')
            self.set_rc(3, 1600)
            count_start = -1
            count_stop = -1
            tstart = self.get_sim_time()
            last_mission_current_msg = 0
            last_seq = None
            while self.armed(): # we RTL at end of mission
                now = self.get_sim_time_cached()
                if now - tstart > 120:
                    raise AutoTestTimeoutException()
                m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True)
                if ((now - last_mission_current_msg) > 1 or
                    m.seq != last_seq):
                    dist = None
                    x = self.mav.messages.get("NAV_CONTROLLER_OUTPUT", None)
                    if x is not None:
                        dist = x.wp_dist
                        self.progress("MISSION_CURRENT.seq=%u dist=%s" %
                                      (m.seq, dist))
                    last_mission_current_msg = self.get_sim_time_cached()
                    last_seq = m.seq
                if m.seq == 3:
                    self.progress("At delay item")
                    if count_start == -1:
                        count_start = now
                if m.seq > 3:
                    if count_stop == -1:
                        count_stop = now
            calculated_delay = count_stop - count_start
            want_delay = 59 # should reflect what's in the mission file
            self.progress("Stopped for %u seconds (want >=%u seconds)" %
                          (calculated_delay, want_delay))
            if calculated_delay < want_delay:
                raise NotAchievedException()

        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.set_rc(3, 1000)

        if ex is not None:
            raise ex

    def get_system_clock_utc(self, time_seconds):
        # this is a copy of ArduPilot's AP_RTC function!
        # separate time into ms, sec, min, hour and days but all expressed
        # in milliseconds
        time_ms = time_seconds * 1000
        ms = time_ms % 1000
        sec_ms = (time_ms % (60 * 1000)) - ms
        min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms
        hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms

        # convert times as milliseconds into appropriate units
        secs = sec_ms / 1000
        mins = min_ms / (60 * 1000)
        hours = hour_ms / (60 * 60 * 1000)
        return (hours, mins, secs, 0)

    def calc_delay(self, seconds):
        # delay-for-seconds has to be long enough that we're at the
        # waypoint before that time.  Otherwise we'll try to wait a
        # day....
        (hours,
         mins,
         secs,
         ms) = self.get_system_clock_utc(seconds)
        self.progress("Now is %uh %um %us" % (hours, mins, secs))
        secs += 17 # add seventeen seconds
        if secs >= 60:
            secs %= 60
            mins += 1 # add sixty seconds
        mins += 1
        if mins >= 60:
            mins %= 60
            hours += 1
        if hours >= 24:
            hours %= 24
        return (hours, mins, secs, 0)

    def reset_delay_item_seventyseven(self, seq):
        while True:
            self.progress("Requesting request for seq %u" % (seq,))
            self.mav.mav.mission_write_partial_list_send(1, # target system
                                                         1, # target component
                                                         seq, # start index
                                                         seq)
            req = self.mav.recv_match(type='MISSION_REQUEST',
                                      blocking=True,
                                      timeout=1)
            if req is not None and req.seq == seq:
                if req.get_srcSystem() == 255:
                    self.progress("Shutup MAVProxy")
                    continue
                # notionally this might be in the message cache before
                # we prompt for it... *shrug*
                break

        # we have received a request for the item.  Supply it:

        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command = mavutil.mavlink.MAV_CMD_NAV_DELAY
        # retrieve mission item and check it:
        tried_set = False
        while True:
            self.progress("Requesting item")
            self.mav.mav.mission_request_send(1,
                                              1,
                                              seq
            )
            st = self.mav.recv_match(type='MISSION_ITEM',
                                     blocking=True,
                                     timeout=1)
            if st is None:
                continue

            print("Item: %s" % str(st))
            if (tried_set and
                st.seq == seq and
                st.command == command and
                st.param2 == hours and
                st.param3 == mins and
                st.param4 == secs):
                return
            self.progress("Mission mismatch")

            m = None
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > 3:
                    raise NotAchievedException()
                self.mav.mav.mission_write_partial_list_send(1,
                                                             1,
                                                             seq,
                                                             seq)
                m = self.mav.recv_match(type='MISSION_REQUEST',
                                        blocking=True,
                                        timeout=1)
                if m is None:
                    continue
                if m.seq != st.seq:
                    continue
                break

            self.progress("Sending absolute-time mission item")

            # we have to change out the delay time...
            now = self.mav.messages["SYSTEM_TIME"]
            if now is None:
                raise PreconditionFailedException()
            if now.time_unix_usec == 0:
                raise PreconditionFailedException()
            (hours, mins, secs, ms) = self.calc_delay(
                now.time_unix_usec/1000000)

            self.progress("Delay until %uh %um %us" %
                          (hours, mins, secs))

            self.mav.mav.mission_item_send(
                1, # target system
                1, # target component
                seq, # seq
                frame, # frame
                command, # command
                0, # current
                1, # autocontinue
                0, # p1 (relative seconds)
                hours, # p2
                mins, # p3
                secs, # p4
                0, # p5
                0, # p6
                0) # p7
            tried_set = True
            ack = self.mav.recv_match(type='MISSION_ACK',
                                      blocking=True,
                                      timeout=1)
            self.progress("Received ack: %s" % str(ack))

    def fly_nav_delay_abstime(self):
        """fly a simple mission that has a delay in it"""

        self.load_mission("copter_nav_delay.txt")

        self.progress("Starting mission")

        self.mavproxy.send('mode loiter\n')  # stabilize mode
        self.mav.wait_heartbeat()
        self.wait_mode('LOITER')
        self.progress("Waiting reading for arm")
        self.wait_ready_to_arm()

        delay_item_seq = 3
        self.reset_delay_item_seventyseven(delay_item_seq)
        delay_for_seconds = 77
        reset_at_m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True)
        reset_at = reset_at_m.time_unix_usec/1000000

        self.context_push()

        ex = None
        try:
            self.arm_vehicle()
            self.mavproxy.send('mode auto\n')  # stabilize mode
            self.wait_mode('AUTO')
            self.set_rc(3, 1600)
            count_stop = -1
            tstart = self.get_sim_time()
            while self.armed(): # we RTL at end of mission
                now = self.get_sim_time()
                if now - tstart > 120:
                    raise AutoTestTimeoutException()
                m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True)
                self.progress("MISSION_CURRENT.seq=%u" % (m.seq,))
                if m.seq == delay_item_seq:
                    self.progress("At delay item")
                if m.seq > delay_item_seq:
                    if count_stop == -1:
                        count_stop_m = self.mav.recv_match(type='SYSTEM_TIME',
                                                           blocking=True)
                        count_stop = count_stop_m.time_unix_usec/1000000
            calculated_delay = count_stop - reset_at
            error = abs(calculated_delay - delay_for_seconds)
            self.progress("Stopped for %u seconds (want >=%u seconds)" %
                          (calculated_delay, delay_for_seconds))
            if error > 2:
                self.progress("Too far outside expectations")
                raise NotAchievedException()

        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.set_rc(3, 1000)

        if ex is not None:
            raise ex

    def fly_nav_takeoff_delay_abstime(self):
        """make sure taking off at a specific time works"""
        num_wp = self.load_mission("copter_nav_delay_takeoff.txt")

        self.progress("Starting mission")

        self.mavproxy.send('mode loiter\n')  # stabilize mode
        self.mav.wait_heartbeat()
        self.wait_mode('LOITER')
        self.progress("Waiting reading for arm")
        self.wait_ready_to_arm()

        delay_item_seq = 2
        self.reset_delay_item_seventyseven(delay_item_seq)
        delay_for_seconds = 77
        reset_at = self.get_sim_time_cached();

        self.context_push();

        ex = None
        try:
            self.arm_vehicle()
            self.mavproxy.send('mode auto\n')  # stabilize mode
            self.wait_mode('AUTO')
            self.set_rc(3, 1600)

            # should not take off for about least 77 seconds
            tstart = self.get_sim_time()
            took_off = False
            while self.armed():
                now = self.get_sim_time_cached()
                if now - tstart > 200:
                    # timeout
                    break
                m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True)
                now = self.get_sim_time_cached()
                self.progress("%s" % str(m))
                if m.seq > delay_item_seq:
                    if not took_off:
                        took_off = True
                        delta_time = now - reset_at
                        if abs(delta_time - delay_for_seconds) > 2:
                            self.progress("Did not take off on time "
                                          "measured=%f want=%f" %
                                          (delta_time, delay_for_seconds))
                            raise NotAchievedException()

            if not took_off:
                raise NotAchievedException()

        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.set_rc(3, 1000)

        if ex is not None:
            raise ex

    def test_setting_modes_via_modeswitch(self):
        self.context_push()
        ex = None
        try:
            fltmode_ch = 5
            self.set_parameter("FLTMODE_CH", fltmode_ch)
            self.set_rc(fltmode_ch, 1000) # PWM for mode1
            testmodes = [("FLTMODE1", 4, "GUIDED", 1165),
                         ("FLTMODE2", 13, "SPORT", 1295),
                         ("FLTMODE3", 6, "RTL", 1425),
                         ("FLTMODE4", 7, "CIRCLE", 1555),
                         ("FLTMODE5", 1, "ACRO", 1685),
                         ("FLTMODE6", 17, "BRAKE", 1815),
                         ]
            for mode in testmodes:
                (parm, parm_value, name, pwm) = mode
                self.set_parameter(parm, parm_value)

            for mode in reversed(testmodes):
                (parm, parm_value, name, pwm) = mode
                self.set_rc(fltmode_ch, pwm)
                self.wait_mode(name)

            for mode in testmodes:
                (parm, parm_value, name, pwm) = mode
                self.set_rc(fltmode_ch, pwm)
                self.wait_mode(name)

            for mode in reversed(testmodes):
                (parm, parm_value, name, pwm) = mode
                self.set_rc(fltmode_ch, pwm)
                self.wait_mode(name)

            self.mavproxy.send('switch 6\n')
            self.wait_mode("BRAKE")
            self.mavproxy.send('switch 5\n')
            self.wait_mode("ACRO")

        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def test_setting_modes_via_auxswitch(self):
        self.context_push()
        ex = None
        try:
            fltmode_ch = int(self.get_parameter("FLTMODE_CH"))
            self.set_rc(fltmode_ch, 1000)
            self.wait_mode("CIRCLE")
            self.set_rc(9, 1000)
            self.set_rc(10, 1000)
            self.set_parameter("RC9_OPTION", 18) # land
            self.set_parameter("RC10_OPTION", 55) # guided
            self.set_rc(9, 1900)
            self.wait_mode("LAND")
            self.set_rc(10, 1900)
            self.wait_mode("GUIDED")
            self.set_rc(10, 1000) # this re-polls the mode switch
            self.wait_mode("CIRCLE")
        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def fly_guided_change_submode(self):
        """"Ensure we can move around in guided after a takeoff command."""

        self.context_push()

        ex = None
        try:
            '''start by disabling GCS failsafe, otherwise we immediately disarm
            due to (apparently) not receiving traffic from the GCS for
            too long.  This is probably a function of --speedup'''
            self.set_parameter("FS_GCS_ENABLE", 0)
            self.mavproxy.send('mode guided\n')  # stabilize mode
            self.wait_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.user_takeoff(alt_min=10)

            startpos = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                           blocking=True)

            """yaw through absolute angles using MAV_CMD_CONDITION_YAW"""
            self.guided_achieve_heading(45)
            self.guided_achieve_heading(135)

            """move the vehicle using set_position_target_global_int"""
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 200:
                    raise NotAchievedException()
                # send a position-control command
                self.mav.mav.set_position_target_global_int_send(
                    0, # timestamp
                    1, # target system_id
                    1, # target component id
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b1111111111111000, # mask specifying use-only-lat-lon-alt
                    5, # lat
                    5, # lon
                    10, # alt
                    0, # vx
                    0, # vy
                    0, # vz
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw
                    0, # yawrate
                )
                pos = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True)
                delta = self.get_distance_int(startpos, pos)
                self.progress("delta=%f (want >10)" % delta)
                if delta > 10:
                    break

            self.progress("Landing")
            self.mavproxy.send('switch 2\n')  # land mode
            self.wait_mode('LAND')
            self.mav.motors_disarmed_wait()

        except Exception as e:
            self.progress("Exception caught")
            ex = e

        self.context_pop()
        self.set_rc(3, 1000)
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def autotest(self):
        """Autotest ArduCopter in SITL."""
        self.check_test_syntax(test_file=os.path.realpath(__file__))
        if not self.hasInit:
            self.init()

        self.fail_list = []

        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(3, 1000)
            self.run_test("Fly Nav Delay (takeoff)",
                          self.fly_nav_takeoff_delay_abstime)

            self.run_test("Fly Nav Delay (AbsTime)",
                          self.fly_nav_delay_abstime)

            self.run_test("Fly Nav Delay", self.fly_nav_delay)

            self.run_test("Test submode change",
                          self.fly_guided_change_submode)

            self.progress("Waiting for location")
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.mav.wait_heartbeat()
            self.wait_mode('STABILIZE')
            self.progress("Waiting reading for arm")
            self.wait_ready_to_arm()

            self.run_test("Set modes via modeswitch",
                          self.test_setting_modes_via_modeswitch)

            self.run_test("Set modes via auxswitch",
                          self.test_setting_modes_via_auxswitch)

            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.wait_mode('STABILIZE')

            self.mavproxy.send("wp clear\n")

            # Arm
            self.run_test("Arm features", self.test_arm_feature)
            self.arm_vehicle()

            # Takeoff
            self.run_test("Takeoff to test fly Square",
                          lambda: self.takeoff(10))

            # AutoTune mode
            self.run_test("Fly AUTOTUNE mode", self.fly_autotune)

            # Fly a square in Stabilize mode
            self.run_test("Fly a square and save WPs with CH7",
                          self.fly_square)

            # save the stored mission to file
            global num_wp
            num_wp = self.save_mission_to_file(os.path.join(testdir,
                                                            "ch7_mission.txt"))
            if not num_wp:
                self.fail_list.append("save_mission_to_file")
                self.progress("save_mission_to_file failed")

            # fly the stored mission
            self.run_test("Fly CH7 saved mission", self.fly_mission)

            # Throttle Failsafe
            self.run_test("Test Failsafe", self.fly_throttle_failsafe)

            # Takeoff
            self.run_test("Takeoff to test battery failsafe",
                          lambda: self.takeoff(10))

            # Battery failsafe
            self.run_test("Fly Battery Failsafe", self.fly_battery_failsafe)

            # Takeoff
            self.run_test("Takeoff to test stability patch",
                          lambda: self.takeoff(10))

            # Stability patch
            self.run_test("Fly stability patch",
                          lambda: self.fly_stability_patch(30))

            # RTL
            self.run_test("RTL after stab patch", self.fly_RTL)

            # Takeoff
            self.run_test("Takeoff to test horizontal fence",
                          lambda: self.takeoff(10))

            # Fence test
            self.run_test("Test horizontal fence",
                          lambda: self.fly_fence_test(180))

            # Takeoff
            self.run_test("Takeoff to test Max Alt fence",
                          lambda: self.takeoff(10))

            # Fence test
            self.run_test("Test Max Alt Fence", self.fly_alt_max_fence_test)

            # Takeoff
            self.run_test("Takeoff to test GPS glitch loiter",
                          lambda: self.takeoff(10))

            # Fly GPS Glitch Loiter test
            self.run_test("GPS Glitch Loiter Test",
                          self.fly_gps_glitch_loiter_test)

            # RTL after GPS Glitch Loiter test
            self.run_test("RTL after GPS Glitch Loiter test", self.fly_RTL)

            # Arm
            self.mavproxy.send('mode stabilize\n')  # stabilize mode
            self.wait_mode('STABILIZE')
            self.set_rc(3, 1000)
            self.run_test("Arm motors", self.arm_vehicle)

            # Fly GPS Glitch test in auto mode
            self.run_test("GPS Glitch Auto Test",
                          self.fly_gps_glitch_auto_test)

            # Takeoff
            self.run_test("Takeoff to test loiter", lambda: self.takeoff(10))

            # Loiter for 10 seconds
            self.run_test("Test Loiter for 10 seconds", self.loiter)

            # Loiter Climb
            self.run_test("Loiter - climb to 30m", lambda: self.change_alt(30))

            # Loiter Descend
            self.run_test("Loiter - descend to 20m",
                          lambda: self.change_alt(20))

            # RTL
            self.run_test("RTL after Loiter climb/descend", self.fly_RTL)

            # Takeoff
            self.run_test("Takeoff to test fly SIMPLE mode",
                          lambda: self.takeoff(10))

            # Simple mode
            self.run_test("Fly in SIMPLE mode", self.fly_simple)

            # RTL
            self.run_test("RTL after SIMPLE mode", self.fly_RTL)

            # Takeoff
            self.run_test("Takeoff to test circle in SUPER SIMPLE mode",
                          lambda: self.takeoff(10))

            # Fly a circle in super simple mode
            self.run_test("Fly a circle in SUPER SIMPLE mode",
                          self.fly_super_simple)

            # RTL
            self.run_test("RTL after SUPER SIMPLE mode", self.fly_RTL)

            # Takeoff
            self.run_test("Takeoff to test CIRCLE mode",
                          lambda: self.takeoff(10))

            # Circle mode
            self.run_test("Fly CIRCLE mode", self.fly_circle)

            # RTL
            self.run_test("RTL after CIRCLE mode", self.fly_RTL)

            # Arm
            self.set_rc(3, 1000)
            self.mavproxy.send('mode stabilize\n')  # stabilize mode
            self.wait_mode('STABILIZE')

            # Takeoff
            self.run_test("Takeoff to test motor failure",
                          lambda: self.takeoff(10))

            self.run_test("Fly motor failure test",
                          self.fly_motor_fail)

            # RTL
            self.run_test("RTL after motor failure test", self.fly_RTL)

            # Arm
            self.set_rc(3, 1000)
            self.mavproxy.send('mode stabilize\n')  # stabilize mode
            self.wait_mode('STABILIZE')
            self.run_test("Arm motors", self.arm_vehicle)

            # Fly auto test
            self.run_test("Fly copter mission", self.fly_auto_test)

            # land
            self.run_test("Fly copter mission", self.land)

            # wait for disarm
            self.mav.motors_disarmed_wait()

            # Gripper test
            self.run_test("Test gripper", self.test_gripper)

            '''vision position'''  # expects vehicle to be disarmed
            self.run_test("Fly Vision Position", self.fly_vision_position)

            # Download logs
            self.run_test("log download",
                          lambda: self.log_download(
                              self.buildlogs_path("ArduCopter-log.bin")))

        except pexpect.TIMEOUT:
            self.progress("Failed with timeout")
            self.fail_list.append("Failed with timeout")
        self.close()

        if len(self.fail_list):
            self.progress("FAILED : %s" % self.fail_list)
            return False
        return True

    def autotest_heli(self):
        """Autotest Helicopter in SITL with AVC2013 mission."""
        self.frame = 'heli'
        if not self.hasInit:
            self.init()

        self.fail_list = []

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
            self.run_test("Arm motors", self.arm_vehicle)
            self.progress("Raising rotor speed")
            self.set_rc(8, 2000)

            self.run_test("Fly AVC mission", self.fly_avc_test)

            self.progress("Lowering rotor speed")
            self.set_rc(8, 1000)

            # mission ends with disarm so should be ok to download logs now
            self.run_test("log download",
                          lambda: self.log_download(
                              self.buildlogs_path("Helicopter-log.bin")))

        except pexpect.TIMEOUT:
            self.fail_list.append("Failed with timeout")

        self.close()

        if len(self.fail_list):
            self.progress("FAILED: %s" % self.fail_list)
            return False
        return True
