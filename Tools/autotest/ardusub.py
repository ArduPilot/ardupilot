#!/usr/bin/env python

# Dive ArduSub in SITL
from __future__ import print_function
import os
import shutil

import pexpect
from pymavlink import mavutil

from common import *
from pysim import util

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


HOME = mavutil.location(33.810313, -118.393867, 0, 185)
homeloc = None


def dive_manual(mavproxy, mav):
    set_rc(mavproxy, mav, 3, 1600)
    set_rc(mavproxy, mav, 5, 1600)
    set_rc(mavproxy, mav, 6, 1550)

    if not wait_distance(mav, 50, accuracy=7, timeout=200):
        return False
    
    set_rc(mavproxy, mav, 4, 1550)
    
    if not wait_heading(mav, 0):
        return False
    
    set_rc(mavproxy, mav, 4, 1500)
    
    if not wait_distance(mav, 50, accuracy=7, timeout=100):
        return False
    
    set_rc(mavproxy, mav, 4, 1550)
    
    if not wait_heading(mav, 0):
        return False
    
    set_rc(mavproxy, mav, 4, 1500)
    set_rc(mavproxy, mav, 5, 1500)
    set_rc(mavproxy, mav, 6, 1100)
    
    if not wait_distance(mav, 75, accuracy=7, timeout=100):
        return False

    set_rc_default(mavproxy)

    disarm_vehicle(mavproxy, mav)
    progress("Manual dive OK")
    return True


def dive_mission(mavproxy, mav, filename):
    
    progress("Executing mission %s" % filename)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('Flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Saved [0-9]+ waypoints')
    set_rc_default(mavproxy)
    
    if not arm_vehicle(mavproxy, mav):
        progress("Failed to ARM")
        return False
    
    mavproxy.send('mode auto\n')
    wait_mode(mav, 'AUTO')
    
    if not wait_waypoint(mav, 1, 5, max_dist=5):
        return False

    disarm_vehicle(mavproxy, mav)

    progress("Mission OK")
    return True


def dive_ArduSub(binary, viewerip=None, use_map=False, valgrind=False, gdb=False, gdbserver=False, speedup=10):
    """Dive ArduSub in SITL.

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the mission in real time
    """
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
    if viewerip:
        options += " --out=%s:14550" % viewerip
    if use_map:
        options += ' --map'

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sitl = util.start_SITL(binary, model='vectored', wipe=True, home=home, speedup=speedup)
    mavproxy = util.start_MAVProxy_SITL('ArduSub')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load %s/default_params/sub.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')
    mavproxy.send('param set FS_GCS_ENABLE 0\n')
    mavproxy.send("param set LOG_REPLAY 1\n")
    mavproxy.send("param set LOG_DISARMED 1\n")
    time.sleep(3)

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    sitl = util.start_SITL(binary, model='vectored', home=home, speedup=speedup, valgrind=valgrind, gdb=gdb, gdbserver=gdbserver)
    mavproxy = util.start_MAVProxy_SITL('ArduSub', options=options)
    mavproxy.expect('Telemetry log: (\S+)\r\n')
    logfile = mavproxy.match.group(1)
    progress("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduSub-test.tlog")
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
        progress("Waiting for GPS fix")
        mav.wait_gps_fix()
        
        # wait for EKF and GPS checks to pass
        mavproxy.expect('IMU0 is using GPS')
        
        homeloc = mav.location()
        progress("Home location: %s" % homeloc)
        set_rc_default(mavproxy)
        if not arm_vehicle(mavproxy, mav):
            progress("Failed to ARM")
            failed = True
        if not dive_manual(mavproxy, mav):
            progress("Failed manual dive")
            failed = True
        if not dive_mission(mavproxy, mav, os.path.join(testdir, "sub_mission.txt")):
            progress("Failed auto mission")
            failed = True
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduSub-log.bin")):
            progress("Failed log download")
            failed = True
    except pexpect.TIMEOUT as e:
        progress("Failed with timeout")
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    valgrind_log = util.valgrind_log_filepath(binary=binary, model='sub')
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0o644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/APMrover2-valgrind.log"))

    if failed:
        progress("FAILED: %s" % e)
        return False
    return True
