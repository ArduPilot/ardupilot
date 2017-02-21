#!/usr/bin/env python
from __future__ import print_function
import os
import shutil
import pexpect

from common import *
from common_test import *
from pysim import util
from pymavlink import mavutil, mavwp

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

#################################################
# CONFIG
#################################################

HOME = mavutil.location(-35.362938, 149.165085, 584, 270)
homeloc = None
num_wp = 0
speedup_default = 10


##########################################################
#   TESTS DRIVE
##########################################################
# TODO : TEST ARMING REQUIRE
# fly a square in stabilize mode
def drive_square(mavproxy, mav, side=50):
    """Drive a square, Driving N then E ."""
    progress("TEST SQUARE")
    success = True

    # ensure all sticks in the middle
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('rc 2 1500\n')
    mavproxy.send('rc 3 1500\n')
    mavproxy.send('rc 4 1500\n')

    # use LEARNING Mode
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LEARNING')

    # first aim north
    progress("turn right towards north")
    if not reach_heading(mavproxy, mav, 10):
        success = False

    # save bottom left corner of box as waypoint
    progress("Save WP 1 & 2")
    save_wp(mavproxy, mav)

    # pitch forward to fly north
    progress("Going north %u meters" % side)
    if not reach_distance(mavproxy, mav, side):
        success = False

    # save top left corner of square as waypoint
    progress("Save WP 3")
    save_wp(mavproxy, mav)

    # roll right to fly east
    progress("Going east %u meters" % side)
    if not reach_heading(mavproxy, mav, 100):
        success = False
    if not reach_distance(mavproxy, mav, side):
        success = False

    # save top right corner of square as waypoint
    progress("Save WP 4")
    save_wp(mavproxy, mav)

    # pitch back to fly south
    progress("Going south %u meters" % side)
    if not reach_heading(mavproxy, mav, 190):
        success = False
    if not reach_distance(mavproxy, mav, side):
        success = False

    # save bottom right corner of square as waypoint
    progress("Save WP 5")
    save_wp(mavproxy, mav)

    # roll left to fly west
    progress("Going west %u meters" % side)
    if not reach_heading(mavproxy, mav, 280):
        success = False
    if not reach_distance(mavproxy, mav, side):
        success = False

    # save bottom left corner of square (should be near home) as waypoint
    progress("Save WP 6")
    save_wp(mavproxy, mav)

    return success


def drive_left_circuit(mavproxy, mav):
    """Drive a left circuit, 50m on a side."""
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'MANUAL')
    mavproxy.send('rc 3 2000\n')

    progress("Driving left circuit")
    # do 4 turns
    for i in range(0, 4):
        # hard left
        progress("Starting turn %u" % i)
        mavproxy.send('rc 1 1000\n')
        if not wait_heading(mav, 270 - (90*i), accuracy=10):
            return False
        mavproxy.send('rc 1 1500\n')
        progress("Starting leg %u" % i)
        if not wait_distance(mav, 50, accuracy=7):
            return False
    mavproxy.send('rc 3 1500\n')
    progress("Circuit complete")
    return True



#################################################
# AUTOTEST ALL
#################################################
def drive_APMrover2(binary, viewerip=None, use_map=False, valgrind=False, gdb=False):
    """Drive APMrover2 in SITL.

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the mission in real time
    """
    global homeloc

    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
    if viewerip:
        options += " --out=%s:14550" % viewerip
    if use_map:
        options += ' --map'

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sitl = util.start_SITL(binary, wipe=True, model='rover', home=home, speedup=10)
    mavproxy = util.start_MAVProxy_SITL('APMrover2')

    progress("WAITING FOR PARAMETERS")
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load %s/default_params/rover.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')
    mavproxy.send("param set LOG_REPLAY 1\n")
    mavproxy.send("param set LOG_DISARMED 1\n")
    time.sleep(3)

    # restart with new parms
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    sitl = util.start_SITL(binary, model='rover', home=home, speedup=10, valgrind=valgrind, gdb=gdb)
    mavproxy = util.start_MAVProxy_SITL('APMrover2', options=options)
    mavproxy.expect('Telemetry log: (\S+)')
    logfile = mavproxy.match.group(1)
    progress("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/APMrover2-test.tlog")
    progress("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        pass

    mavproxy.expect('Received [0-9]+ parameters')

    util.expect_setup_callback(mavproxy, expect_callback)

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
    e = 'None'
    try:
        progress("Waiting for a heartbeat with mavlink protocol %s" % mav.WIRE_PROTOCOL_VERSION)
        mav.wait_heartbeat()
        progress("Setting up RC parameters")
        setup_rc(mavproxy)
        set_switch_default(mavproxy)
        progress("Waiting for GPS fix")
        mav.wait_gps_fix()
        homeloc = mav.location()
        progress("Home location: %s" % homeloc)
        mavproxy.send('switch 6\n')  # Manual mode
        wait_mode(mav, 'MANUAL')
        progress("Waiting reading for arm")
        wait_ready_to_arm(mavproxy)

        progress("#")
        progress("########## Do common feature tests ##########")
        progress("#")
        if not test_common_feature(mavproxy, mav):
            progress("Failed common feature")
            failed = True

        progress("#")
        progress("########## Drive a square and save WPs with CH7 switch  ##########")
        progress("#")
        # Drive a square in learning mode
        if not drive_square(mavproxy, mav):
            progress("Failed drive square")
            failed = True

        # save the stored mission to file
        progress("# Save out the CH7 mission to file")
        if not save_mission_to_file(mavproxy, os.path.join(testdir, "ch7_mission.txt")):
            progress("save_mission_to_file failed")
            failed = True

        # drive the stored mission
        progress("# Drive CH7 saved mission")
        if not test_mission(mavproxy, mav, os.path.join(testdir, "ch7_mission.txt")):
            progress("drive ch7_mission failed")
            failed = True

        # Throttle Failsafe
        progress("#")
        progress("########## Test Failsafe ##########")
        progress("#")
        if not test_throttle_failsafe(mavproxy, mav, HOME):
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

        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/APMrover2-log.bin")):
            progress("Failed log download")
            failed = True

    except pexpect.TIMEOUT as e:
        progress("Failed with timeout")
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    valgrind_log = util.valgrind_log_filepath(binary=binary, model='rover')
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0o644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/APMrover2-valgrind.log"))

    if failed:
        progress("FAILED: %s" % e)
        return False
    return True
