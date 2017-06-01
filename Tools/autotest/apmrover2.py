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

#################################################
# CONFIG
#################################################

# HOME = mavutil.location(-35.362938, 149.165085, 584, 270)
HOME = mavutil.location(40.071374969556928, -105.22978898137808, 1583.702759, 246)

homeloc = None
num_wp = 0
speedup_default = 10


##########################################################
#   TESTS DRIVE
##########################################################
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


def drive_RTL(mavproxy, mav):
    """Drive to home."""
    progress("Driving home in RTL")
    mavproxy.send('switch 3\n')
    if not wait_location(mav, homeloc, accuracy=22, timeout=90):
        return False
    progress("RTL Complete")
    return True


#################################################
# AUTOTEST ALL
#################################################
def drive_mission(mavproxy, mav, filename):
    """Drive a mission from a file."""
    global homeloc
    progress("Driving mission %s" % filename)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('Flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')
    mavproxy.send('switch 4\n')  # auto mode
    mavproxy.send('rc 3 1500\n')
    wait_mode(mav, 'AUTO')
    if not wait_waypoint(mav, 1, 4, max_dist=5):
        return False
    wait_mode(mav, 'HOLD')
    progress("Mission OK")
    return True

vinfo = vehicleinfo.VehicleInfo()

def drive_APMrover2(binary, viewerip=None, use_map=False, valgrind=False, gdb=False, frame=None, params=None):
    """Drive APMrover2 in SITL.

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the mission in real time
    """
    global homeloc

    if frame is None:
        frame = 'rover'

    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
    if viewerip:
        options += " --out=%s:14550" % viewerip
    if use_map:
        options += ' --map'

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sitl = util.start_SITL(binary, wipe=True, model=frame, home=home, speedup=10)
    mavproxy = util.start_MAVProxy_SITL('APMrover2')

    progress("WAITING FOR PARAMETERS")
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    if params is None:
        params = vinfo.options["APMrover2"]["frames"][frame]["default_params_filename"]
    if not isinstance(params, list):
        params = [params]
    for x in params:
        mavproxy.send("param load %s\n" % os.path.join(testdir, x))
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
        set_rc_default(mavproxy)
        mavproxy.send('rc 8 1800\n')
        progress("Waiting for GPS fix")
        mav.wait_gps_fix()
        homeloc = mav.location()
        progress("Home location: %s" % homeloc)
        mavproxy.send('switch 6\n')  # Manual mode
        wait_mode(mav, 'MANUAL')
        progress("Waiting reading for arm")
        wait_ready_to_arm(mavproxy)
        if not arm_vehicle(mavproxy, mav):
            progress("Failed to ARM")
            failed = True
        progress("#")
        progress("########## Drive a square and save WPs with CH7 switch  ##########")
        progress("#")
        # Drive a square in learning mode
        if not drive_mission(mavproxy, mav, os.path.join(testdir, "rover1.txt")):
            progress("Failed mission")
            failed = True
        if not disarm_vehicle(mavproxy, mav):
            progress("Failed to DISARM")
            failed = True
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/APMrover2-log.bin")):
            progress("Failed log download")
            failed = True
#        if not drive_left_circuit(mavproxy, mav):
#            progress("Failed left circuit")
#            failed = True
#        if not drive_RTL(mavproxy, mav):
#            progress("Failed RTL")
#            failed = True
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
