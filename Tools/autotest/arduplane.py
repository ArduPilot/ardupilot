#!/usr/bin/env python

# Fly ArduPlane in SITL
from __future__ import print_function
import math
import os
import shutil

import pexpect
from pymavlink import mavutil

from common import *
from pysim import util

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


HOME_LOCATION = '-35.362938,149.165085,585,354'
WIND = "0,180,0.2"  # speed,direction,variance

homeloc = None


def takeoff(mavproxy, mav):
    """Takeoff get to 30m altitude."""

    wait_ready_to_arm(mav)
    arm_vehicle(mavproxy, mav)

    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')

    # some rudder to counteract the prop torque
    set_rc(mavproxy, mav, 4, 1700)

    # some up elevator to keep the tail down
    set_rc(mavproxy, mav, 2, 1200)

    # get it moving a bit first
    set_rc(mavproxy, mav, 3, 1300)
    mav.recv_match(condition='VFR_HUD.groundspeed>6', blocking=True)

    # a bit faster again, straighten rudder
    set_rc(mavproxy, mav, 3, 1600)
    set_rc(mavproxy, mav, 4, 1500)
    mav.recv_match(condition='VFR_HUD.groundspeed>12', blocking=True)

    # hit the gas harder now, and give it some more elevator
    set_rc(mavproxy, mav, 2, 1100)
    set_rc(mavproxy, mav, 3, 2000)

    # gain a bit of altitude
    if not wait_altitude(mav, homeloc.alt+150, homeloc.alt+180, timeout=30):
        return False

    # level off
    set_rc(mavproxy, mav, 2, 1500)

    progress("TAKEOFF COMPLETE")
    return True


def fly_left_circuit(mavproxy, mav):
    """Fly a left circuit, 200m on a side."""
    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')
    set_rc(mavproxy, mav, 3, 2000)
    if not wait_level_flight(mavproxy, mav):
        return False

    progress("Flying left circuit")
    # do 4 turns
    for i in range(0, 4):
        # hard left
        progress("Starting turn %u" % i)
        set_rc(mavproxy, mav, 1, 1000)
        if not wait_heading(mav, 270 - (90*i), accuracy=10):
            return False
        set_rc(mavproxy, mav, 1, 1500)
        progress("Starting leg %u" % i)
        if not wait_distance(mav, 100, accuracy=20):
            return False
    progress("Circuit complete")
    return True


def fly_RTL(mavproxy, mav):
    """Fly to home."""
    progress("Flying home in RTL")
    mavproxy.send('switch 2\n')
    wait_mode(mav, 'RTL')
    if not wait_location(mav, homeloc, accuracy=120,
                         target_altitude=homeloc.alt+100, height_accuracy=20,
                         timeout=180):
        return False
    progress("RTL Complete")
    return True


def fly_LOITER(mavproxy, mav, num_circles=4):
    """Loiter where we are."""
    progress("Testing LOITER for %u turns" % num_circles)
    mavproxy.send('loiter\n')
    wait_mode(mav, 'LOITER')

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    initial_alt = m.alt
    progress("Initial altitude %u\n" % initial_alt)

    while num_circles > 0:
        if not wait_heading(mav, 0, accuracy=10, timeout=60):
            return False
        if not wait_heading(mav, 180, accuracy=10, timeout=60):
            return False
        num_circles -= 1
        progress("Loiter %u circles left" % num_circles)

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    final_alt = m.alt
    progress("Final altitude %u initial %u\n" % (final_alt, initial_alt))

    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    if abs(final_alt - initial_alt) > 20:
        progress("Failed to maintain altitude")
        return False

    progress("Completed Loiter OK")
    return True


def fly_CIRCLE(mavproxy, mav, num_circles=1):
    """Circle where we are."""
    progress("Testing CIRCLE for %u turns" % num_circles)
    mavproxy.send('mode CIRCLE\n')
    wait_mode(mav, 'CIRCLE')

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    initial_alt = m.alt
    progress("Initial altitude %u\n" % initial_alt)

    while num_circles > 0:
        if not wait_heading(mav, 0, accuracy=10, timeout=60):
            return False
        if not wait_heading(mav, 180, accuracy=10, timeout=60):
            return False
        num_circles -= 1
        progress("CIRCLE %u circles left" % num_circles)

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    final_alt = m.alt
    progress("Final altitude %u initial %u\n" % (final_alt, initial_alt))

    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    if abs(final_alt - initial_alt) > 20:
        progress("Failed to maintain altitude")
        return False

    progress("Completed CIRCLE OK")
    return True


def wait_level_flight(mavproxy, mav, accuracy=5, timeout=30):
    """Wait for level flight."""
    tstart = get_sim_time(mav)
    progress("Waiting for level flight")
    set_rc(mavproxy, mav, 1, 1500)
    set_rc(mavproxy, mav, 2, 1500)
    set_rc(mavproxy, mav, 4, 1500)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='ATTITUDE', blocking=True)
        roll = math.degrees(m.roll)
        pitch = math.degrees(m.pitch)
        progress("Roll=%.1f Pitch=%.1f" % (roll, pitch))
        if math.fabs(roll) <= accuracy and math.fabs(pitch) <= accuracy:
            progress("Attained level flight")
            return True
    progress("Failed to attain level flight")
    return False


def change_altitude(mavproxy, mav, altitude, accuracy=30):
    """Get to a given altitude."""
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')
    alt_error = mav.messages['VFR_HUD'].alt - altitude
    if alt_error > 0:
        set_rc(mavproxy, mav, 2, 2000)
    else:
        set_rc(mavproxy, mav, 2, 1000)
    if not wait_altitude(mav, altitude-accuracy/2, altitude+accuracy/2):
        return False
    set_rc(mavproxy, mav, 2, 1500)
    progress("Reached target altitude at %u" % mav.messages['VFR_HUD'].alt)
    return wait_level_flight(mavproxy, mav)


def axial_left_roll(mavproxy, mav, count=1):
    """Fly a left axial roll."""
    # full throttle!
    set_rc(mavproxy, mav, 3, 2000)
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False

    # fly the roll in manual
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'MANUAL')

    while count > 0:
        progress("Starting roll")
        set_rc(mavproxy, mav, 1, 1000)
        if not wait_roll(mav, -150, accuracy=90):
            set_rc(mavproxy, mav, 1, 1500)
            return False
        if not wait_roll(mav, 150, accuracy=90):
            set_rc(mavproxy, mav, 1, 1500)
            return False
        if not wait_roll(mav, 0, accuracy=90):
            set_rc(mavproxy, mav, 1, 1500)
            return False
        count -= 1

    # back to FBWA
    set_rc(mavproxy, mav, 1, 1500)
    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')
    set_rc(mavproxy, mav, 3, 1700)
    return wait_level_flight(mavproxy, mav)


def inside_loop(mavproxy, mav, count=1):
    """Fly a inside loop."""
    # full throttle!
    set_rc(mavproxy, mav, 3, 2000)
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False

    # fly the loop in manual
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'MANUAL')

    while count > 0:
        progress("Starting loop")
        set_rc(mavproxy, mav, 2, 1000)
        if not wait_pitch(mav, -60, accuracy=20):
            return False
        if not wait_pitch(mav, 0, accuracy=20):
            return False
        count -= 1

    # back to FBWA
    set_rc(mavproxy, mav, 2, 1500)
    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')
    set_rc(mavproxy, mav, 3, 1700)
    return wait_level_flight(mavproxy, mav)


def test_stabilize(mavproxy, mav, count=1):
    """Fly stabilize mode."""
    # full throttle!
    set_rc(mavproxy, mav, 3, 2000)
    set_rc(mavproxy, mav, 2, 1300)
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False
    set_rc(mavproxy, mav, 2, 1500)

    mavproxy.send("mode STABILIZE\n")
    wait_mode(mav, 'STABILIZE')

    count = 1
    while count > 0:
        progress("Starting roll")
        set_rc(mavproxy, mav, 1, 2000)
        if not wait_roll(mav, -150, accuracy=90):
            return False
        if not wait_roll(mav, 150, accuracy=90):
            return False
        if not wait_roll(mav, 0, accuracy=90):
            return False
        count -= 1

    set_rc(mavproxy, mav, 1, 1500)
    if not wait_roll(mav, 0, accuracy=5):
        return False

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')
    set_rc(mavproxy, mav, 3, 1700)
    return wait_level_flight(mavproxy, mav)


def test_acro(mavproxy, mav, count=1):
    """Fly ACRO mode."""
    # full throttle!
    set_rc(mavproxy, mav, 3, 2000)
    set_rc(mavproxy, mav, 2, 1300)
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False
    set_rc(mavproxy, mav, 2, 1500)

    mavproxy.send("mode ACRO\n")
    wait_mode(mav, 'ACRO')

    count = 1
    while count > 0:
        progress("Starting roll")
        set_rc(mavproxy, mav, 1, 1000)
        if not wait_roll(mav, -150, accuracy=90):
            return False
        if not wait_roll(mav, 150, accuracy=90):
            return False
        if not wait_roll(mav, 0, accuracy=90):
            return False
        count -= 1
    set_rc(mavproxy, mav, 1, 1500)

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    wait_level_flight(mavproxy, mav)

    mavproxy.send("mode ACRO\n")
    wait_mode(mav, 'ACRO')

    count = 2
    while count > 0:
        progress("Starting loop")
        set_rc(mavproxy, mav, 2, 1000)
        if not wait_pitch(mav, -60, accuracy=20):
            return False
        if not wait_pitch(mav, 0, accuracy=20):
            return False
        count -= 1

    set_rc(mavproxy, mav, 2, 1500)

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')
    set_rc(mavproxy, mav, 3, 1700)
    return wait_level_flight(mavproxy, mav)


def test_FBWB(mavproxy, mav, count=1, mode='FBWB'):
    """Fly FBWB or CRUISE mode."""
    mavproxy.send("mode %s\n" % mode)
    wait_mode(mav, mode)
    set_rc(mavproxy, mav, 3, 1700)
    set_rc(mavproxy, mav, 2, 1500)

    # lock in the altitude by asking for an altitude change then releasing
    set_rc(mavproxy, mav, 2, 1000)
    wait_distance(mav, 50, accuracy=20)
    set_rc(mavproxy, mav, 2, 1500)
    wait_distance(mav, 50, accuracy=20)

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    initial_alt = m.alt
    progress("Initial altitude %u\n" % initial_alt)

    progress("Flying right circuit")
    # do 4 turns
    for i in range(0, 4):
        # hard left
        progress("Starting turn %u" % i)
        set_rc(mavproxy, mav, 1, 1800)
        if not wait_heading(mav, 0 + (90*i), accuracy=20, timeout=60):
            set_rc(mavproxy, mav, 1, 1500)
            return False
        set_rc(mavproxy, mav, 1, 1500)
        progress("Starting leg %u" % i)
        if not wait_distance(mav, 100, accuracy=20):
            return False
    progress("Circuit complete")

    progress("Flying rudder left circuit")
    # do 4 turns
    for i in range(0, 4):
        # hard left
        progress("Starting turn %u" % i)
        set_rc(mavproxy, mav, 4, 1900)
        if not wait_heading(mav, 360 - (90*i), accuracy=20, timeout=60):
            set_rc(mavproxy, mav, 4, 1500)
            return False
        set_rc(mavproxy, mav, 4, 1500)
        progress("Starting leg %u" % i)
        if not wait_distance(mav, 100, accuracy=20):
            return False
    progress("Circuit complete")

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    final_alt = m.alt
    progress("Final altitude %u initial %u\n" % (final_alt, initial_alt))

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    if abs(final_alt - initial_alt) > 20:
        progress("Failed to maintain altitude")
        return False

    return wait_level_flight(mavproxy, mav)


def fly_mission(mavproxy, mav, filename, height_accuracy=-1, target_altitude=None):
    """Fly a mission from a file."""
    global homeloc
    progress("Flying mission %s" % filename)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('Flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')
    mavproxy.send('switch 1\n')  # auto mode
    wait_mode(mav, 'AUTO')
    if not wait_waypoint(mav, 1, 7, max_dist=60):
        return False
    if not wait_groundspeed(mav, 0, 0.5, timeout=60):
        return False
    mavproxy.expect("Auto disarmed")
    progress("Mission OK")
    return True


def fly_ArduPlane(binary, viewerip=None, use_map=False, valgrind=False, gdb=False, gdbserver=False, speedup=10):
    """Fly ArduPlane in SITL.

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time
    """
    global homeloc

    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
    if viewerip:
        options += " --out=%s:14550" % viewerip
    if use_map:
        options += ' --map'

    sitl = util.start_SITL(binary, model='plane-elevrev', home=HOME_LOCATION, speedup=speedup,
                          valgrind=valgrind, gdb=gdb, gdbserver=gdbserver,
                          defaults_file=os.path.join(testdir, 'default_params/plane-jsbsim.parm'))
    mavproxy = util.start_MAVProxy_SITL('ArduPlane', options=options)
    mavproxy.expect('Telemetry log: (\S+)\r\n')
    logfile = mavproxy.match.group(1)
    progress("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduPlane-test.tlog")
    progress("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        pass

    util.expect_setup_callback(mavproxy, expect_callback)

    mavproxy.expect('Received [0-9]+ parameters')

    expect_list_clear()
    expect_list_extend([sitl, mavproxy])

    progress("Started simulator")

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception as msg:
        progress("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)

    failed = False
    fail_list = []
    e = 'None'
    try:
        progress("Waiting for a heartbeat with mavlink protocol %s" % mav.WIRE_PROTOCOL_VERSION)
        mav.wait_heartbeat()
        progress("Setting up RC parameters")
        set_rc_default(mavproxy)
        set_rc(mavproxy, mav, 3, 1000)
        set_rc(mavproxy, mav, 8, 1800)
        progress("Waiting for GPS fix")
        mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
        mav.wait_gps_fix()
        while mav.location().alt < 10:
            mav.wait_gps_fix()
        homeloc = mav.location()
        progress("Home location: %s" % homeloc)
        if not takeoff(mavproxy, mav):
            progress("Failed takeoff")
            failed = True
            fail_list.append("takeoff")
        if not fly_left_circuit(mavproxy, mav):
            progress("Failed left circuit")
            failed = True
            fail_list.append("left_circuit")
        if not axial_left_roll(mavproxy, mav, 1):
            progress("Failed left roll")
            failed = True
            fail_list.append("left_roll")
        if not inside_loop(mavproxy, mav):
            progress("Failed inside loop")
            failed = True
            fail_list.append("inside_loop")
        if not test_stabilize(mavproxy, mav):
            progress("Failed stabilize test")
            failed = True
            fail_list.append("stabilize")
        if not test_acro(mavproxy, mav):
            progress("Failed ACRO test")
            failed = True
            fail_list.append("acro")
        if not test_FBWB(mavproxy, mav):
            progress("Failed FBWB test")
            failed = True
            fail_list.append("fbwb")
        if not test_FBWB(mavproxy, mav, mode='CRUISE'):
            progress("Failed CRUISE test")
            failed = True
            fail_list.append("cruise")
        if not fly_RTL(mavproxy, mav):
            progress("Failed RTL")
            failed = True
            fail_list.append("RTL")
        if not fly_LOITER(mavproxy, mav):
            progress("Failed LOITER")
            failed = True
            fail_list.append("LOITER")
        if not fly_CIRCLE(mavproxy, mav):
            progress("Failed CIRCLE")
            failed = True
            fail_list.append("LOITER")
        if not fly_mission(mavproxy, mav, os.path.join(testdir, "ap1.txt"), height_accuracy=10,
                           target_altitude=homeloc.alt+100):
            progress("Failed mission")
            failed = True
            fail_list.append("mission")
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduPlane-log.bin")):
            progress("Failed log download")
            failed = True
            fail_list.append("log_download")
    except pexpect.TIMEOUT as e:
        progress("Failed with timeout")
        failed = True
        fail_list.append("timeout")

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    valgrind_log = util.valgrind_log_filepath(binary=binary, model='plane-elevrev')
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0o644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/ArduPlane-valgrind.log"))

    if failed:
        progress("FAILED: %s" % e)
        progress("Fail list: %s" % fail_list)
        return False
    return True
