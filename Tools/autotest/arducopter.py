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

vinfo = vehicleinfo.VehicleInfo()

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

#################################################
# CONFIG
#################################################
HOME = mavutil.location(-35.362938, 149.165085, 584, 270)
AVCHOME = mavutil.location(40.072842, -105.230575, 1586, 0)

homeloc = None
num_wp = 0

# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize


#################################################
# UTILITIES
#################################################
def takeoff(mavproxy, mav, alt_min=30, takeoff_throttle=1700):
    """Takeoff get to 30m altitude."""
    mavproxy.send('switch 6\n')  # stabilize mode
    wait_mode(mav, 'STABILIZE')
    set_rc(mavproxy, mav, 3, takeoff_throttle)
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    if (m.alt < alt_min):
        wait_altitude(mav, alt_min, (alt_min + 5))
    hover(mavproxy, mav)
    progress("TAKEOFF COMPLETE")
    return True


def land(mavproxy, mav, timeout=60):
    """Land the quad."""
    progress("STARTING LANDING")
    mavproxy.send('switch 2\n')  # land mode
    wait_mode(mav, 'LAND')
    progress("Entered Landing Mode")
    ret = wait_altitude(mav, -5, 1)
    progress("LANDING: ok= %s" % ret)
    return ret


def hover(mavproxy, mav, hover_throttle=1500):
    set_rc(mavproxy, mav, 3, hover_throttle)
    return True


# loiter - fly south west, then hold loiter within 5m position and altitude
def loiter(mavproxy, mav, holdtime=10, maxaltchange=5, maxdistchange=5):
    """Hold loiter position."""
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # first aim south east
    progress("turn south east")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 170):
        return False
    set_rc(mavproxy, mav, 4, 1500)

    # fly south east 50m
    set_rc(mavproxy, mav, 2, 1100)
    if not wait_distance(mav, 50):
        return False
    set_rc(mavproxy, mav, 2, 1500)

    # wait for copter to slow moving
    if not wait_groundspeed(mav, 0, 2):
        return False

    success = True
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    start = mav.location()
    tstart = get_sim_time(mav)
    tholdstart = get_sim_time(mav)
    progress("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))
    while get_sim_time(mav) < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        delta = get_distance(start, pos)
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


def change_alt(mavproxy, mav, alt_min, climb_throttle=1920, descend_throttle=1080):
    """Change altitude."""
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    if(m.alt < alt_min):
        progress("Rise to alt:%u from %u" % (alt_min, m.alt))
        set_rc(mavproxy, mav, 3, climb_throttle)
        wait_altitude(mav, alt_min, (alt_min + 5))
    else:
        progress("Lower to alt:%u from %u" % (alt_min, m.alt))
        set_rc(mavproxy, mav, 3, descend_throttle)
        wait_altitude(mav, (alt_min - 5), alt_min)
    hover(mavproxy, mav)
    return True


#################################################
#   TESTS FLY
#################################################
# fly a square in stabilize mode
def fly_square(mavproxy, mav, side=50, timeout=300):
    """Fly a square, flying N then E ."""
    tstart = get_sim_time(mav)
    success = True

    # ensure all sticks in the middle
    set_rc(mavproxy, mav, 1, 1500)
    set_rc(mavproxy, mav, 2, 1500)
    set_rc(mavproxy, mav, 3, 1500)
    set_rc(mavproxy, mav, 4, 1500)

    # switch to loiter mode temporarily to stop us from rising
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')

    # first aim north
    progress("turn right towards north")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 10):
        progress("Failed to reach heading")
        success = False
    set_rc(mavproxy, mav, 4, 1500)
    mav.recv_match(condition='RC_CHANNELS.chan4_raw==1500', blocking=True)

    # save bottom left corner of box as waypoint
    progress("Save WP 1 & 2")
    save_wp(mavproxy, mav)

    # switch back to stabilize mode
    set_rc(mavproxy, mav, 3, 1500)
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')

    # pitch forward to fly north
    progress("Going north %u meters" % side)
    set_rc(mavproxy, mav, 2, 1300)
    if not wait_distance(mav, side):
        progress("Failed to reach distance of %u" % side)
        success = False
    set_rc(mavproxy, mav, 2, 1500)

    # save top left corner of square as waypoint
    progress("Save WP 3")
    save_wp(mavproxy, mav)

    # roll right to fly east
    progress("Going east %u meters" % side)
    set_rc(mavproxy, mav, 1, 1700)
    if not wait_distance(mav, side):
        progress("Failed to reach distance of %u" % side)
        success = False
    set_rc(mavproxy, mav, 1, 1500)

    # save top right corner of square as waypoint
    progress("Save WP 4")
    save_wp(mavproxy, mav)

    # pitch back to fly south
    progress("Going south %u meters" % side)
    set_rc(mavproxy, mav, 2, 1700)
    if not wait_distance(mav, side):
        progress("Failed to reach distance of %u" % side)
        success = False
    set_rc(mavproxy, mav, 2, 1500)

    # save bottom right corner of square as waypoint
    progress("Save WP 5")
    save_wp(mavproxy, mav)

    # roll left to fly west
    progress("Going west %u meters" % side)
    set_rc(mavproxy, mav, 1, 1300)
    if not wait_distance(mav, side):
        progress("Failed to reach distance of %u" % side)
        success = False
    set_rc(mavproxy, mav, 1, 1500)

    # save bottom left corner of square (should be near home) as waypoint
    progress("Save WP 6")
    save_wp(mavproxy, mav)

    # descend to 10m
    progress("Descend to 10m in Loiter")
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')
    set_rc(mavproxy, mav, 3, 1300)
    time_left = timeout - (get_sim_time(mav) - tstart)
    progress("timeleft = %u" % time_left)
    if time_left < 20:
        time_left = 20
    if not wait_altitude(mav, -10, 10, time_left):
        progress("Failed to reach alt of 10m")
        success = False
    save_wp(mavproxy, mav)

    return success


def fly_RTL(mavproxy, mav, side=60, timeout=250):
    """Return, land."""
    progress("# Enter RTL")
    mavproxy.send('switch 3\n')
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        if(m.alt <= 1 and home_distance < 10):
            return True
    return False


def fly_throttle_failsafe(mavproxy, mav, side=60, timeout=180):
    """Fly east, Failsafe, return, land."""

    # switch to loiter mode temporarily to stop us from rising
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')

    # first aim east
    progress("turn east")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 135):
        return False
    set_rc(mavproxy, mav, 4, 1500)

    # raise throttle slightly to avoid hitting the ground
    set_rc(mavproxy, mav, 3, 1600)

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    hover(mavproxy, mav)

    # fly east 60 meters
    progress("# Going forward %u meters" % side)
    set_rc(mavproxy, mav, 2, 1350)
    if not wait_distance(mav, side, 5, 60):
        return False
    set_rc(mavproxy, mav, 2, 1500)

    # pull throttle low
    progress("# Enter Failsafe")
    set_rc(mavproxy, mav, 3, 900)

    tstart = get_sim_time(mav)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        # check if we've reached home
        if m.alt <= 1 and home_distance < 10:
            # reduce throttle
            set_rc(mavproxy, mav, 3, 1100)
            # switch back to stabilize
            mavproxy.send('switch 2\n')  # land mode
            wait_mode(mav, 'LAND')
            progress("Waiting for disarm")
            mav.motors_disarmed_wait()
            progress("Reached failsafe home OK")
            mavproxy.send('switch 6\n')  # stabilize mode
            wait_mode(mav, 'STABILIZE')
            set_rc(mavproxy, mav, 3, 1000)
            if not arm_vehicle(mavproxy, mav):
                progress("Failed to re-arm")
                return False
            return True
    progress("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
    # reduce throttle
    set_rc(mavproxy, mav, 3, 1100)
    # switch back to stabilize mode
    mavproxy.send('switch 2\n')  # land mode
    wait_mode(mav, 'LAND')
    mavproxy.send('switch 6\n')  # stabilize mode
    wait_mode(mav, 'STABILIZE')
    return False


def fly_battery_failsafe(mavproxy, mav, timeout=30):
    # assume failure
    success = False

    # switch to loiter mode so that we hold position
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')
    mavproxy.send("rc 3 1500\n")

    # enable battery failsafe
    mavproxy.send("param set FS_BATT_ENABLE 1\n")

    # trigger low voltage
    mavproxy.send('param set SIM_BATT_VOLTAGE 10\n')

    # wait for LAND mode
    new_mode = wait_mode(mav, 'LAND')
    if new_mode == 'LAND':
        success = True

    # disable battery failsafe
    mavproxy.send('param set FS_BATT_ENABLE 0\n')

    # return status
    if success:
        progress("Successfully entered LAND mode after battery failsafe")
    else:
        progress("Failed to enter LAND mode after battery failsafe")

    return success


# fly_stability_patch - fly south, then hold loiter within 5m position and altitude and reduce 1 motor to 60% efficiency
def fly_stability_patch(mavproxy, mav, holdtime=30, maxaltchange=5, maxdistchange=10):
    """Hold loiter position."""
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # first south
    progress("turn south")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 180):
        return False
    set_rc(mavproxy, mav, 4, 1500)

    # fly west 80m
    set_rc(mavproxy, mav, 2, 1100)
    if not wait_distance(mav, 80):
        return False
    set_rc(mavproxy, mav, 2, 1500)

    # wait for copter to slow moving
    if not wait_groundspeed(mav, 0, 2):
        return False

    success = True
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    start = mav.location()
    tstart = get_sim_time(mav)
    tholdstart = get_sim_time(mav)
    progress("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))

    # cut motor 1 to 55% efficiency
    progress("Cutting motor 1 to 60% efficiency")
    mavproxy.send('param set SIM_ENGINE_MUL 0.60\n')

    while get_sim_time(mav) < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        delta = get_distance(start, pos)
        alt_delta = math.fabs(m.alt - start_altitude)
        progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
        if alt_delta > maxaltchange:
            progress("Loiter alt shifted %u meters (> limit of %u)" % (alt_delta, maxaltchange))
            success = False
        if delta > maxdistchange:
            progress("Loiter shifted %u meters (> limit of %u)" % (delta, maxdistchange))
            success = False

    # restore motor 1 to 100% efficiency
    mavproxy.send('param set SIM_ENGINE_MUL 1.0\n')

    if success:
        progress("Stability patch and Loiter OK for %u seconds" % holdtime)
    else:
        progress("Stability Patch FAILED")

    return success


# fly_fence_test - fly east until you hit the horizontal circular fence
def fly_fence_test(mavproxy, mav, timeout=180):
    """Hold loiter position."""
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # enable fence, disable avoidance
    mavproxy.send('param set FENCE_ENABLE 1\n')
    mavproxy.send('param set AVOID_ENABLE 0\n')

    # first east
    progress("turn east")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 160):
        return False
    set_rc(mavproxy, mav, 4, 1500)

    # fly forward (east) at least 20m
    pitching_forward = True
    set_rc(mavproxy, mav, 2, 1100)
    if not wait_distance(mav, 20):
        return False

    # start timer
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        # recenter pitch sticks once we reach home so we don't fly off again
        if pitching_forward and home_distance < 10:
            pitching_forward = False
            set_rc(mavproxy, mav, 2, 1500)
            # disable fence
            mavproxy.send('param set FENCE_ENABLE 0\n')
        if m.alt <= 1 and home_distance < 10:
            # reduce throttle
            set_rc(mavproxy, mav, 3, 1000)
            # switch mode to stabilize
            mavproxy.send('switch 2\n')  # land mode
            wait_mode(mav, 'LAND')
            progress("Waiting for disarm")
            mav.motors_disarmed_wait()
            progress("Reached home OK")
            mavproxy.send('switch 6\n')  # stabilize mode
            wait_mode(mav, 'STABILIZE')
            set_rc(mavproxy, mav, 3, 1000)
            mavproxy.send('arm uncheck all\n')  # remove if we ever clear battery failsafe flag on disarm
            if not arm_vehicle(mavproxy, mav):
                progress("Failed to re-arm")
                mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm
                return False
            mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm
            progress("Reached home OK")
            return True

    # disable fence, enable avoidance
    mavproxy.send('param set FENCE_ENABLE 0\n')
    mavproxy.send('param set AVOID_ENABLE 1\n')

    # reduce throttle
    set_rc(mavproxy, mav, 3, 1000)
    # switch mode to stabilize
    mavproxy.send('switch 2\n')  # land mode
    wait_mode(mav, 'LAND')
    mavproxy.send('switch 6\n')  # stabilize mode
    wait_mode(mav, 'STABILIZE')
    progress("Fence test failed to reach home - timed out after %u seconds" % timeout)
    return False


# fly_alt_fence_test - fly up until you hit the fence
def fly_alt_max_fence_test(mavproxy, mav, timeout=180):
    """Hold loiter position."""
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # enable fence, disable avoidance
    set_parameter(mavproxy, 'FENCE_ENABLE', 1)
    set_parameter(mavproxy, 'AVOID_ENABLE', 0)
    set_parameter(mavproxy, 'FENCE_TYPE', 1)

    if not change_alt(mavproxy, mav, 10):
        failed_test_msg = "change_alt climb failed"
        progress(failed_test_msg)
        return False

    # first east
    progress("turn east")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 160):
        return False
    set_rc(mavproxy, mav, 4, 1500)

    # fly forward (east) at least 20m
    set_rc(mavproxy, mav, 2, 1100)
    if not wait_distance(mav, 20):
        return False

    # stop flying forward and start flying up:
    set_rc(mavproxy, mav, 2, 1500)
    set_rc(mavproxy, mav, 3, 1800)

    # wait for fence to trigger
    wait_mode(mav, 'RTL')

    progress("Waiting for disarm")
    mav.motors_disarmed_wait()

    set_rc(mavproxy, mav, 3, 1000)

    mavproxy.send('switch 6\n')  # stabilize mode
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('arm uncheck all\n')  # remove if we ever clear battery failsafe flag on disarm
    if not arm_vehicle(mavproxy, mav):
        progress("Failed to re-arm")
        mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm
        return False
    mavproxy.send('arm check all\n')  # remove if we ever clear battery failsafe flag on disarm

    return True

def fly_gps_glitch_loiter_test(mavproxy, mav, use_map=False, timeout=30, max_distance=20):
    """fly_gps_glitch_loiter_test.

     Fly south east in loiter and test reaction to gps glitch."""
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # turn on simulator display of gps and actual position
    if (use_map):
        show_gps_and_sim_positions(mavproxy, True)

    # set-up gps glitch array
    glitch_lat = [0.0002996, 0.0006958, 0.0009431, 0.0009991, 0.0009444, 0.0007716, 0.0006221]
    glitch_lon = [0.0000717, 0.0000912, 0.0002761, 0.0002626, 0.0002807, 0.0002049, 0.0001304]
    glitch_num = len(glitch_lat)
    progress("GPS Glitches:")
    for i in range(1, glitch_num):
        progress("glitch %d %.7f %.7f" % (i, glitch_lat[i], glitch_lon[i]))

    # turn south east
    progress("turn south east")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 150):
        if (use_map):
            show_gps_and_sim_positions(mavproxy, False)
        return False
    set_rc(mavproxy, mav, 4, 1500)

    # fly forward (south east) at least 60m
    set_rc(mavproxy, mav, 2, 1100)
    if not wait_distance(mav, 60):
        if (use_map):
            show_gps_and_sim_positions(mavproxy, False)
        return False
    set_rc(mavproxy, mav, 2, 1500)

    # wait for copter to slow down
    if not wait_groundspeed(mav, 0, 1):
        if (use_map):
            show_gps_and_sim_positions(mavproxy, False)
        return False

    # record time and position
    tstart = get_sim_time(mav)
    tnow = tstart
    start_pos = sim_location(mav)
    success = True

    # initialise current glitch
    glitch_current = 0
    progress("Apply first glitch")
    mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
    mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

    # record position for 30 seconds
    while tnow < tstart + timeout:
        tnow = get_sim_time(mav)
        desired_glitch_num = int((tnow - tstart) * 2.2)
        if desired_glitch_num > glitch_current and glitch_current != -1:
            glitch_current = desired_glitch_num
            # turn off glitching if we've reached the end of the glitch list
            if glitch_current >= glitch_num:
                glitch_current = -1
                progress("Completed Glitches")
                mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
                mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
            else:
                progress("Applying glitch %u" % glitch_current)
                # move onto the next glitch
                mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
                mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

        # start displaying distance moved after all glitches applied
        if (glitch_current == -1):
            m = mav.recv_match(type='VFR_HUD', blocking=True)
            curr_pos = sim_location(mav)
            moved_distance = get_distance(curr_pos, start_pos)
            progress("Alt: %u  Moved: %.0f" % (m.alt, moved_distance))
            if moved_distance > max_distance:
                progress("Moved over %u meters, Failed!" % max_distance)
                success = False

    # disable gps glitch
    if glitch_current != -1:
        glitch_current = -1
        mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
        mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
    if (use_map):
        show_gps_and_sim_positions(mavproxy, False)

    if success:
        progress("GPS glitch test passed!  stayed within %u meters for %u seconds" % (max_distance, timeout))
    else:
        progress("GPS glitch test FAILED!")
    return success


# fly_gps_glitch_auto_test - fly mission and test reaction to gps glitch
def fly_gps_glitch_auto_test(mavproxy, mav, use_map=False, timeout=120):

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
    global homeloc
    global num_wp
    num_wp = load_mission_from_file(mavproxy, os.path.join(testdir, "copter_glitch_mission.txt"))
    if not num_wp:
        progress("load copter_glitch_mission failed")
        return False

    # turn on simulator display of gps and actual position
    if (use_map):
        show_gps_and_sim_positions(mavproxy, True)

    progress("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')

    # switch into AUTO mode and raise throttle
    mavproxy.send('switch 4\n')  # auto mode
    wait_mode(mav, 'AUTO')
    set_rc(mavproxy, mav, 3, 1500)

    # wait until 100m from home
    if not wait_distance(mav, 100, 5, 60):
        if (use_map):
            show_gps_and_sim_positions(mavproxy, False)
        return False

    # record time and position
    tstart = get_sim_time(mav)
    tnow = tstart

    # initialise current glitch
    glitch_current = 0
    progress("Apply first glitch")
    mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
    mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

    # record position for 30 seconds
    while glitch_current < glitch_num:
        tnow = get_sim_time(mav)
        desired_glitch_num = int((tnow - tstart) * 2.2)
        if desired_glitch_num > glitch_current and glitch_current != -1:
            glitch_current = desired_glitch_num
            # apply next glitch
            if glitch_current < glitch_num:
                progress("Applying glitch %u" % glitch_current)
                mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
                mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

    # turn off glitching
    progress("Completed Glitches")
    mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
    mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')

    # continue with the mission
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)

    # wait for arrival back home
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    pos = mav.location()
    dist_to_home = get_distance(HOME, pos)
    while dist_to_home > 5:
        if get_sim_time(mav) > (tstart + timeout):
            progress("GPS Glitch testing failed - exceeded timeout %u seconds" % timeout)
            ret = False
            break
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        dist_to_home = get_distance(HOME, pos)
        progress("Dist from home: %u" % dist_to_home)

    # turn off simulator display of gps and actual position
    if (use_map):
        show_gps_and_sim_positions(mavproxy, False)

    progress("GPS Glitch test Auto completed: passed=%s" % ret)

    return ret


#   fly_simple - assumes the simple bearing is initialised to be directly north
#   flies a box with 100m west, 15 seconds north, 50 seconds east, 15 seconds south
def fly_simple(mavproxy, mav, side=50, timeout=120):

    failed = False

    # hold position in loiter
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # set SIMPLE mode for all flight modes
    mavproxy.send('param set SIMPLE 63\n')

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    set_rc(mavproxy, mav, 3, 1500)

    # fly south 50m
    progress("# Flying south %u meters" % side)
    set_rc(mavproxy, mav, 1, 1300)
    if not wait_distance(mav, side, 5, 60):
        failed = True
    set_rc(mavproxy, mav, 1, 1500)

    # fly west 8 seconds
    progress("# Flying west for 8 seconds")
    set_rc(mavproxy, mav, 2, 1300)
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < (tstart + 8):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (get_sim_time(mav) - tstart)
        # progress("%u" % delta)
    set_rc(mavproxy, mav, 2, 1500)

    # fly north 25 meters
    progress("# Flying north %u meters" % (side/2.0))
    set_rc(mavproxy, mav, 1, 1700)
    if not wait_distance(mav, side/2, 5, 60):
        failed = True
    set_rc(mavproxy, mav, 1, 1500)

    # fly east 8 seconds
    progress("# Flying east for 8 seconds")
    set_rc(mavproxy, mav, 2, 1700)
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < (tstart + 8):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (get_sim_time(mav) - tstart)
        # progress("%u" % delta)
    set_rc(mavproxy, mav, 2, 1500)

    # restore to default
    mavproxy.send('param set SIMPLE 0\n')

    # hover in place
    hover(mavproxy, mav)
    return not failed


# fly_super_simple - flies a circle around home for 45 seconds
def fly_super_simple(mavproxy, mav, timeout=45):

    failed = False

    # hold position in loiter
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # fly forward 20m
    progress("# Flying forward 20 meters")
    set_rc(mavproxy, mav, 2, 1300)
    if not wait_distance(mav, 20, 5, 60):
        failed = True
    set_rc(mavproxy, mav, 2, 1500)

    # set SUPER SIMPLE mode for all flight modes
    mavproxy.send('param set SUPER_SIMPLE 63\n')

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    set_rc(mavproxy, mav, 3, 1500)

    # start copter yawing slowly
    set_rc(mavproxy, mav, 4, 1550)

    # roll left for timeout seconds
    progress("# rolling left from pilot's point of view for %u seconds" % timeout)
    set_rc(mavproxy, mav, 1, 1300)
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < (tstart + timeout):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (get_sim_time(mav) - tstart)

    # stop rolling and yawing
    set_rc(mavproxy, mav, 1, 1500)
    set_rc(mavproxy, mav, 4, 1500)

    # restore simple mode parameters to default
    mavproxy.send('param set SUPER_SIMPLE 0\n')

    # hover in place
    hover(mavproxy, mav)
    return not failed


# fly_circle - flies a circle with 20m radius
def fly_circle(mavproxy, mav, maxaltchange=10, holdtime=36):

    # hold position in loiter
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')

    # face west
    progress("turn west")
    set_rc(mavproxy, mav, 4, 1580)
    if not wait_heading(mav, 270):
        return False
    set_rc(mavproxy, mav, 4, 1500)

    # set CIRCLE radius
    mavproxy.send('param set CIRCLE_RADIUS 3000\n')

    # fly forward (east) at least 100m
    set_rc(mavproxy, mav, 2, 1100)
    if not wait_distance(mav, 100):
        return False

    # return pitch stick back to middle
    set_rc(mavproxy, mav, 2, 1500)

    # set CIRCLE mode
    mavproxy.send('switch 1\n')  # circle mode
    wait_mode(mav, 'CIRCLE')

    # wait
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = get_sim_time(mav)
    tholdstart = get_sim_time(mav)
    progress("Circle at %u meters for %u seconds" % (start_altitude, holdtime))
    while get_sim_time(mav) < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        progress("heading %u" % m.heading)

    progress("CIRCLE OK for %u seconds" % holdtime)
    return True


# fly_auto_test - fly mission which tests a significant number of commands
def fly_auto_test(mavproxy, mav):

    # Fly mission #1
    progress("# Load copter_mission")
    # load the waypoint count
    global homeloc
    global num_wp
    num_wp = load_mission_from_file(mavproxy, os.path.join(testdir, "copter_mission.txt"))
    if not num_wp:
        progress("load copter_mission failed")
        return False

    progress("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')

    # switch into AUTO mode and raise throttle
    mavproxy.send('switch 4\n')  # auto mode
    wait_mode(mav, 'AUTO')
    set_rc(mavproxy, mav, 3, 1500)

    # fly the mission
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)

    # land if mission failed
    if ret is False:
        land(mavproxy, mav)

    # set throttle to minimum
    set_rc(mavproxy, mav, 3, 1000)

    # wait for disarm
    mav.motors_disarmed_wait()
    progress("MOTORS DISARMED OK")

    progress("Auto mission completed: passed=%s" % ret)

    return ret


# fly_avc_test - fly AVC mission
def fly_avc_test(mavproxy, mav):

    # upload mission from file
    progress("# Load copter_AVC2013_mission")
    # load the waypoint count
    global homeloc
    global num_wp
    num_wp = load_mission_from_file(mavproxy, os.path.join(testdir, "copter_AVC2013_mission.txt"))
    if not num_wp:
        progress("load copter_AVC2013_mission failed")
        return False

    progress("Fly AVC mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')

    # wait for motor runup
    wait_seconds(mav, 20)

    # switch into AUTO mode and raise throttle
    mavproxy.send('switch 4\n')  # auto mode
    wait_mode(mav, 'AUTO')
    set_rc(mavproxy, mav, 3, 1500)

    # fly the mission
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)

    # set throttle to minimum
    set_rc(mavproxy, mav, 3, 1000)

    # wait for disarm
    mav.motors_disarmed_wait()
    progress("MOTORS DISARMED OK")

    progress("AVC mission completed: passed=%s" % ret)

    return ret


def fly_mission(mavproxy, mav, height_accuracy=-1.0, target_altitude=None):
    """Fly a mission from a file."""
    global homeloc
    global num_wp
    progress("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')
    mavproxy.send('switch 4\n')  # auto mode
    wait_mode(mav, 'AUTO')
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)
    progress("test: MISSION COMPLETE: passed=%s" % ret)
    # wait here until ready
    mavproxy.send('switch 5\n')  # loiter mode
    wait_mode(mav, 'LOITER')
    return ret


def fly_ArduCopter(binary, viewerip=None, use_map=False, valgrind=False, gdb=False, frame=None, params=None, gdbserver=False, speedup=10):
    """Fly ArduCopter in SITL.

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time
    """
    global homeloc

    if frame is None:
        frame = '+'

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sitl = util.start_SITL(binary, wipe=True, model=frame, home=home, speedup=speedup)
    mavproxy = util.start_MAVProxy_SITL('ArduCopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    if params is None:
        params = vinfo.options["ArduCopter"]["frames"][frame]["default_params_filename"]
    if not isinstance(params, list):
        params = [params]
    for x in params:
        mavproxy.send("param load %s\n" % os.path.join(testdir, x))
        mavproxy.expect('Loaded [0-9]+ parameters')
    mavproxy.send("param set LOG_REPLAY 1\n")
    mavproxy.send("param set LOG_DISARMED 1\n")
    time.sleep(3)

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    sitl = util.start_SITL(binary, model=frame, home=home, speedup=speedup, valgrind=valgrind, gdb=gdb, gdbserver=gdbserver)
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter --streamrate=5'
    if viewerip:
        options += ' --out=%s:14550' % viewerip
    if use_map:
        options += ' --map'
    mavproxy = util.start_MAVProxy_SITL('ArduCopter', options=options)
    mavproxy.expect('Telemetry log: (\S+)\r\n')
    logfile = mavproxy.match.group(1)
    progress("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduCopter-test.tlog")
    progress("buildlog=%s" % buildlog)
    copy_tlog = False
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        progress("WARN: Failed to create symlink: " + logfile + " => " + buildlog + ", Will copy tlog manually to target location")
        copy_tlog = True

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, expect_callback)

    expect_list_clear()
    expect_list_extend([sitl, mavproxy])

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception as msg:
        progress("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)

    failed = False
    failed_test_msg = "None"

    try:
        progress("Waiting for a heartbeat with mavlink protocol %s" % mav.WIRE_PROTOCOL_VERSION)
        mav.wait_heartbeat()
        progress("Setting up RC parameters")
        set_rc_default(mavproxy)
        set_rc(mavproxy, mav, 3, 1000)
        homeloc = mav.location()

        progress("Home location: %s" % homeloc)
        mavproxy.send('switch 6\n')  # stabilize mode
        mav.wait_heartbeat()
        wait_mode(mav, 'STABILIZE')
        progress("Waiting reading for arm")
        wait_ready_to_arm(mav)

        # Arm
        progress("# Arm motors")
        if not arm_vehicle(mavproxy, mav):
            failed_test_msg = "arm_motors failed"
            progress(failed_test_msg)
            failed = True

        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Fly a square in Stabilize mode
        progress("#")
        progress("########## Fly a square and save WPs with CH7 switch ##########")
        progress("#")
        if not fly_square(mavproxy, mav):
            failed_test_msg = "fly_square failed"
            progress(failed_test_msg)
            failed = True

        # save the stored mission to file
        progress("# Save out the CH7 mission to file")
        global num_wp
        num_wp = save_mission_to_file(mavproxy, os.path.join(testdir, "ch7_mission.txt"))
        if not num_wp:
            failed_test_msg = "save_mission_to_file failed"
            progress(failed_test_msg)
            failed = True

        # fly the stored mission
        progress("# Fly CH7 saved mission")
        if not fly_mission(mavproxy, mav, height_accuracy=0.5, target_altitude=10):
            failed_test_msg = "fly ch7_mission failed"
            progress(failed_test_msg)
            failed = True

        # Throttle Failsafe
        progress("#")
        progress("########## Test Failsafe ##########")
        progress("#")
        if not fly_throttle_failsafe(mavproxy, mav):
            failed_test_msg = "fly_throttle_failsafe failed"
            progress(failed_test_msg)
            failed = True

        # Takeoff
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Battery failsafe
        if not fly_battery_failsafe(mavproxy, mav):
            failed_test_msg = "fly_battery_failsafe failed"
            progress(failed_test_msg)
            failed = True

        # Takeoff
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Stability patch
        progress("#")
        progress("########## Test Stability Patch ##########")
        progress("#")
        if not fly_stability_patch(mavproxy, mav, 30):
            failed_test_msg = "fly_stability_patch failed"
            progress(failed_test_msg)
            failed = True

        # RTL
        progress("# RTL #")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after stab patch failed"
            progress(failed_test_msg)
            failed = True

        # Takeoff
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Fence test
        progress("#")
        progress("########## Test Horizontal Fence ##########")
        progress("#")
        if not fly_fence_test(mavproxy, mav, 180):
            failed_test_msg = "fly_fence_test failed"
            progress(failed_test_msg)
            failed = True

        # Fence test
        progress("#")
        progress("########## Test Max Alt Fence ##########")
        progress("#")
        if not fly_alt_max_fence_test(mavproxy, mav, 180):
            failed_test_msg = "fly_alt_max_fence_test failed"
            progress(failed_test_msg)
            failed = True

        # Takeoff
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Fly GPS Glitch Loiter test
        progress("# GPS Glitch Loiter Test")
        if not fly_gps_glitch_loiter_test(mavproxy, mav, use_map):
            failed_test_msg = "fly_gps_glitch_loiter_test failed"
            progress(failed_test_msg)
            failed = True

        # RTL after GPS Glitch Loiter test
        progress("# RTL #")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL failed"
            progress(failed_test_msg)
            failed = True

        # Fly GPS Glitch test in auto mode
        progress("# GPS Glitch Auto Test")
        if not fly_gps_glitch_auto_test(mavproxy, mav, use_map):
            failed_test_msg = "fly_gps_glitch_auto_test failed"
            progress(failed_test_msg)
            failed = True

        # take-off ahead of next test
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Loiter for 10 seconds
        progress("#")
        progress("########## Test Loiter for 10 seconds ##########")
        progress("#")
        if not loiter(mavproxy, mav):
            failed_test_msg = "loiter failed"
            progress(failed_test_msg)
            failed = True

        # Loiter Climb
        progress("#")
        progress("# Loiter - climb to 30m")
        progress("#")
        if not change_alt(mavproxy, mav, 30):
            failed_test_msg = "change_alt climb failed"
            progress(failed_test_msg)
            failed = True

        # Loiter Descend
        progress("#")
        progress("# Loiter - descend to 20m")
        progress("#")
        if not change_alt(mavproxy, mav, 20):
            failed_test_msg = "change_alt descend failed"
            progress(failed_test_msg)
            failed = True

        # RTL
        progress("#")
        progress("########## Test RTL ##########")
        progress("#")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after Loiter climb/descend failed"
            progress(failed_test_msg)
            failed = True

        # Takeoff
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Simple mode
        progress("# Fly in SIMPLE mode")
        if not fly_simple(mavproxy, mav):
            failed_test_msg = "fly_simple failed"
            progress(failed_test_msg)
            failed = True

        # RTL
        progress("#")
        progress("########## Test RTL ##########")
        progress("#")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after simple mode failed"
            progress(failed_test_msg)
            failed = True

        # Takeoff
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Fly a circle in super simple mode
        progress("# Fly a circle in SUPER SIMPLE mode")
        if not fly_super_simple(mavproxy, mav):
            failed_test_msg = "fly_super_simple failed"
            progress(failed_test_msg)
            failed = True

        # RTL
        progress("# RTL #")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after super simple mode failed"
            progress(failed_test_msg)
            failed = True

        # Takeoff
        progress("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            progress(failed_test_msg)
            failed = True

        # Circle mode
        progress("# Fly CIRCLE mode")
        if not fly_circle(mavproxy, mav):
            failed_test_msg = "fly_circle failed"
            progress(failed_test_msg)
            failed = True

        # RTL
        progress("#")
        progress("########## Test RTL ##########")
        progress("#")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after circle failed"
            progress(failed_test_msg)
            failed = True

        progress("# Fly copter mission")
        if not fly_auto_test(mavproxy, mav):
            failed_test_msg = "fly_auto_test failed"
            progress(failed_test_msg)
            failed = True
        else:
            progress("Flew copter mission OK")

        # wait for disarm
        mav.motors_disarmed_wait()

        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduCopter-log.bin")):
            failed_test_msg = "log_download failed"
            progress(failed_test_msg)
            failed = True

    except pexpect.TIMEOUT as e:
        progress("Failed with timeout")
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    valgrind_log = util.valgrind_log_filepath(binary=binary, model='+')
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0o644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/ArduCopter-valgrind.log"))

    # [2014/05/07] FC Because I'm doing a cross machine build (source is on host, build is on guest VM) I cannot hard link
    # This flag tells me that I need to copy the data out
    if copy_tlog:
        shutil.copy(logfile, buildlog)

    if failed:
        progress("FAILED: %s" % failed_test_msg)
        return False
    return True


def fly_CopterAVC(binary, viewerip=None, use_map=False, valgrind=False, gdb=False, frame=None, params=None, gdbserver=False, speedup=10):
    """Fly ArduCopter in SITL for AVC2013 mission."""
    global homeloc

    if frame is None:
        frame = 'heli'

    home = "%f,%f,%u,%u" % (AVCHOME.lat, AVCHOME.lng, AVCHOME.alt, AVCHOME.heading)
    sitl = util.start_SITL(binary, wipe=True, model=frame, home=home, speedup=speedup)
    mavproxy = util.start_MAVProxy_SITL('ArduCopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    if params is None:
        params = vinfo.options["ArduCopter"]["frames"][frame]["default_params_filename"]
    if not isinstance(params, list):
        params = [params]
    for x in params:
        mavproxy.send("param load %s\n" % os.path.join(testdir, x))
        mavproxy.expect('Loaded [0-9]+ parameters')
    mavproxy.send("param set LOG_REPLAY 1\n")
    mavproxy.send("param set LOG_DISARMED 1\n")
    time.sleep(3)

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    sitl = util.start_SITL(binary, model='heli', home=home, speedup=speedup, valgrind=valgrind, gdb=gdb, gdbserver=gdbserver)
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=5'
    if viewerip:
        options += ' --out=%s:14550' % viewerip
    if use_map:
        options += ' --map'
    mavproxy = util.start_MAVProxy_SITL('ArduCopter', options=options)
    mavproxy.expect('Telemetry log: (\S+)\r\n')
    logfile = mavproxy.match.group(1)
    progress("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/CopterAVC-test.tlog")
    progress("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        pass

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, expect_callback)

    expect_list_clear()
    expect_list_extend([sitl, mavproxy])

    if use_map:
        mavproxy.send('map icon 40.072467969730496 -105.2314389590174\n')
        mavproxy.send('map icon 40.072600990533829 -105.23146100342274\n')

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception as msg:
        progress("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)

    failed = False
    failed_test_msg = "None"

    try:
        mav.wait_heartbeat()
        set_rc_default(mavproxy)
        set_rc(mavproxy, mav, 3, 1000)
        homeloc = mav.location()

        progress("Lowering rotor speed")
        set_rc(mavproxy, mav, 8, 1000)

        mavproxy.send('switch 6\n')  # stabilize mode
        wait_mode(mav, 'STABILIZE')
        wait_ready_to_arm(mav)

        # Arm
        progress("# Arm motors")
        if not arm_vehicle(mavproxy, mav):
            failed_test_msg = "arm_motors failed"
            progress(failed_test_msg)
            failed = True

        progress("Raising rotor speed")
        set_rc(mavproxy, mav, 8, 2000)

        progress("# Fly AVC mission")
        if not fly_avc_test(mavproxy, mav):
            failed_test_msg = "fly_avc_test failed"
            progress(failed_test_msg)
            failed = True
        else:
            progress("Flew AVC mission OK")

        progress("Lowering rotor speed")
        set_rc(mavproxy, mav, 8, 1000)

        # mission includes disarm at end so should be ok to download logs now
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/CopterAVC-log.bin")):
            failed_test_msg = "log_download failed"
            progress(failed_test_msg)
            failed = True

    except pexpect.TIMEOUT as failed_test_msg:
        failed_test_msg = "Timeout"
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    valgrind_log = util.valgrind_log_filepath(binary=binary, model='heli')
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0o644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/Helicopter-valgrind.log"))

    if failed:
        progress("FAILED: %s" % failed_test_msg)
        return False
    return True
