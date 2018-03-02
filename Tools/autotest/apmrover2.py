#!/usr/bin/env python

# Drive APMrover2 in SITL
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
    set_rc(mavproxy, mav, 3, 2000)

    progress("Driving left circuit")
    # do 4 turns
    for i in range(0, 4):
        # hard left
        progress("Starting turn %u" % i)
        set_rc(mavproxy, mav, 1, 1000)
        if not wait_heading(mav, 270 - (90*i), accuracy=10):
            return False
        set_rc(mavproxy, mav, 1, 1500)
        progress("Starting leg %u" % i)
        if not wait_distance(mav, 50, accuracy=7):
            return False
    set_rc(mavproxy, mav, 3, 1500)
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
    set_rc(mavproxy, mav, 3, 1500)
    wait_mode(mav, 'AUTO')
    if not wait_waypoint(mav, 1, 4, max_dist=5):
        return False
    wait_mode(mav, 'HOLD')
    progress("Mission OK")
    return True


def do_get_banner(mavproxy, mav):
    mavproxy.send("long DO_SEND_BANNER 1\n")
    start = time.time()
    while True:
        m = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if m is not None and "APM:Rover" in m.text:
            progress("banner received: %s" % (m.text))
            return True
        if time.time() - start > 10:
            break

    progress("banner not received")

    return False


def drive_brake_get_stopping_distance(mavproxy, mav, speed):
    # measure our stopping distance:
    old_cruise_speed = get_parameter(mavproxy, 'CRUISE_SPEED')
    old_accel_max = get_parameter(mavproxy, 'ATC_ACCEL_MAX')

    # controller tends not to meet cruise speed (max of ~14 when 15
    # set), thus *1.2
    set_parameter(mavproxy, 'CRUISE_SPEED', speed*1.2)
    # at time of writing, the vehicle is only capable of 10m/s/s accel
    set_parameter(mavproxy, 'ATC_ACCEL_MAX', 15)
    mavproxy.send("mode STEERING\n")
    wait_mode(mav, 'STEERING')
    set_rc(mavproxy, mav, 3, 2000)
    wait_groundspeed(mav, 15, 100)
    initial = mav.location()
    initial_time = time.time()
    while time.time() - initial_time < 2:
        # wait for a position update from the autopilot
        start = mav.location()
        if start != initial:
            break
    set_rc(mavproxy, mav, 3, 1500)
    wait_groundspeed(mav, 0, 0.2)  # why do we not stop?!
    initial = mav.location()
    initial_time = time.time()
    while time.time() - initial_time < 2:
        # wait for a position update from the autopilot
        stop = mav.location()
        if stop != initial:
            break
    delta = get_distance(start, stop)

    set_parameter(mavproxy, 'CRUISE_SPEED', old_cruise_speed)
    set_parameter(mavproxy, 'ATC_ACCEL_MAX', old_accel_max)

    return delta


def drive_brake(mavproxy, mav):
    old_using_brake = get_parameter(mavproxy, 'ATC_BRAKE')
    old_cruise_speed = get_parameter(mavproxy, 'CRUISE_SPEED')

    set_parameter(mavproxy, 'CRUISE_SPEED', 15)
    set_parameter(mavproxy, 'ATC_BRAKE', 0)

    distance_without_brakes = drive_brake_get_stopping_distance(mavproxy, mav, 15)

    # brakes on:
    set_parameter(mavproxy, 'ATC_BRAKE', 1)
    distance_with_brakes = drive_brake_get_stopping_distance(mavproxy, mav, 15)
    # revert state:
    set_parameter(mavproxy, 'ATC_BRAKE', old_using_brake)
    set_parameter(mavproxy, 'CRUISE_SPEED', old_cruise_speed)

    delta = distance_without_brakes - distance_with_brakes
    if delta < distance_without_brakes * 0.05:  # 5% isn't asking for much
        progress("Brakes have negligible effect (with=%0.2fm without=%0.2fm delta=%0.2fm)" % (distance_with_brakes, distance_without_brakes, delta))
        return False
    else:
        progress("Brakes work (with=%0.2fm without=%0.2fm delta=%0.2fm)" % (distance_with_brakes, distance_without_brakes, delta))

    return True


vinfo = vehicleinfo.VehicleInfo()


def drive_rtl_mission(mavproxy, mav):
    mission_filepath = os.path.join(testdir, "ArduRover-Missions", "rtl.txt")
    mavproxy.send('wp load %s\n' % mission_filepath)
    mavproxy.expect('Flight plan received')
    mavproxy.send('switch 4\n')  # auto mode
    set_rc(mavproxy, mav, 3, 1500)
    wait_mode(mav, 'AUTO')
    mavproxy.expect('Executing RTL')

    m = mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
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

    wait_mode(mav, 'HOLD')

    pos = mav.location()
    home_distance = get_distance(HOME, pos)
    home_distance_max = 5
    if home_distance > home_distance_max:
        progress("Did not get home (%u metres distant > %u)" %
              (home_distance,home_distance_max))
        return False
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'MANUAL')
    progress("RTL Mission OK")
    return True

def drive_APMrover2(binary, viewerip=None, use_map=False, valgrind=False, gdb=False, frame=None, params=None, gdbserver=False, speedup=10):
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
    sitl = util.start_SITL(binary, wipe=True, model=frame, home=home, speedup=speedup)
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
    set_parameter(mavproxy, 'LOG_REPLAY', 1)
    set_parameter(mavproxy, 'LOG_DISARMED', 1)

    # restart with new parms
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    sitl = util.start_SITL(binary, model='rover', home=home, speedup=speedup, valgrind=valgrind, gdb=gdb, gdbserver=gdbserver)
    mavproxy = util.start_MAVProxy_SITL('APMrover2', options=options)
    mavproxy.expect('Telemetry log: (\S+)\r\n')
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
        set_rc(mavproxy, mav, 8, 1800)
        progress("Waiting for GPS fix")
        mav.wait_gps_fix()
        homeloc = mav.location()
        progress("Home location: %s" % homeloc)
        mavproxy.send('switch 6\n')  # Manual mode
        wait_mode(mav, 'MANUAL')
        progress("Waiting reading for arm")
        wait_ready_to_arm(mav)
        if not arm_vehicle(mavproxy, mav):
            progress("Failed to ARM")
            failed = True

        progress("#")
        progress("########## Drive an RTL mission  ##########")
        progress("#")
        # Drive a square in learning mode
        if not drive_rtl_mission(mavproxy, mav):
            progress("Failed RTL mission")
            failed = True

        progress("#")
        progress("########## Drive a square and save WPs with CH7 switch  ##########")
        progress("#")
        # Drive a square in learning mode
        if not drive_mission(mavproxy, mav, os.path.join(testdir, "rover1.txt")):
            progress("Failed mission")
            failed = True

        if not drive_brake(mavproxy, mav):
            progress("Failed brake")
            failed = True

        if not disarm_vehicle(mavproxy, mav):
            progress("Failed to DISARM")
            failed = True

        # do not move this to be the first test.  MAVProxy's dedupe
        # function may bite you.
        progress("Getting banner")
        if not do_get_banner(mavproxy, mav):
            progress("FAILED: get banner")
            failed = True

        progress("Getting autopilot capabilities")
        if not do_get_autopilot_capabilities(mavproxy, mav):
            progress("FAILED: get capabilities")
            failed = True

        progress("Setting mode via MAV_COMMAND_DO_SET_MODE")
        if not do_set_mode_via_command_long(mavproxy, mav):
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
