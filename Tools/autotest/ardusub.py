# drive APMrover2 in SITL
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


def arm_sub(mavproxy, mav):
    for i in range(8):
        mavproxy.send('rc %d 1500\n' % (i+1))

    mavproxy.send('arm throttle\n')
    mavproxy.expect('ARMED')

    print("SUB ARMED")
    return True

def dive_manual(mavproxy, mav):
    mavproxy.send('rc 3 1600\n')
    mavproxy.send('rc 5 1600\n')
    mavproxy.send('rc 6 1550\n')

    if not wait_distance(mav, 50, accuracy=7, timeout=100):
        return False
    
    mavproxy.send('rc 4 1550\n')
    
    if not wait_heading(mav, 0):
        return False
    
    mavproxy.send('rc 4 1500\n')
    
    if not wait_distance(mav, 50, accuracy=7, timeout=100):
        return False
    
    mavproxy.send('rc 4 1550\n')
    
    if not wait_heading(mav, 0):
        return False
    
    mavproxy.send('rc 4 1500\n')
    mavproxy.send('rc 5 1500\n')
    mavproxy.send('rc 6 1100\n')
    
    if not wait_distance(mav, 75, accuracy=7, timeout=100):
        return False
    
    mavproxy.send('rc all 1500\n')
    
    mavproxy.send('disarm\n');

    # wait for disarm
    mav.motors_disarmed_wait()
    print("Manual dive OK")
    return True

def dive_mission(mavproxy, mav, filename):
    
    print("Executing mission %s" % filename)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('Flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Saved [0-9]+ waypoints')
    
    if not arm_sub(mavproxy, mav):
        print("Failed to ARM")
        return False
    
    mavproxy.send('mode auto\n')
    wait_mode(mav, 'AUTO')
    
    if not wait_waypoint(mav, 1, 5, max_dist=5):
        return False
    
    mavproxy.send('disarm\n');
    
    # wait for disarm
    mav.motors_disarmed_wait()

    print("Mission OK")
    return True

def dive_ArduSub(binary, viewerip=None, use_map=False, valgrind=False, gdb=False):
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
    sitl = util.start_SITL(binary, model='vectored', wipe=True, home=home, speedup=10)
    mavproxy = util.start_MAVProxy_SITL('ArduSub', options=options)
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

    sitl = util.start_SITL(binary, model='vectored', home=home, speedup=10, valgrind=valgrind, gdb=gdb)
    mavproxy = util.start_MAVProxy_SITL('ArduSub', options=options)
    mavproxy.expect('Telemetry log: (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduSub-test.tlog")
    print("buildlog=%s" % buildlog)
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

    print("Started simulator")

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception as msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)

    failed = False
    e = 'None'
    try:
        print("Waiting for a heartbeat with mavlink protocol %s" % mav.WIRE_PROTOCOL_VERSION)
        mav.wait_heartbeat()
        print("Waiting for GPS fix")
        mav.wait_gps_fix()
        
        # wait for EKF and GPS checks to pass
        mavproxy.expect('IMU0 is using GPS')
        
        homeloc = mav.location()
        print("Home location: %s" % homeloc)
        if not arm_sub(mavproxy, mav):
            print("Failed to ARM")
            failed = True
        if not dive_manual(mavproxy, mav):
            print("Failed manual dive")
            failed = True
        if not dive_mission(mavproxy, mav, os.path.join(testdir, "sub_mission.txt")):
            print("Failed auto mission")
            failed = True
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduSub-log.bin")):
            print("Failed log download")
            failed = True
    except pexpect.TIMEOUT as e:
        print("Failed with timeout")
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    valgrind_log = util.valgrind_log_filepath(binary=binary, model='sub')
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0o644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/APMrover2-valgrind.log"))

    if failed:
        print("FAILED: %s" % e)
        return False
    return True
