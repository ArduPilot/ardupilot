# fly ArduPlane QuadPlane in SITL

import os
import pexpect
import shutil
from pymavlink import mavutil

from common import *
from pysim import util

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


HOME_LOCATION = '-27.274439,151.290064,343,8.7'
MISSION = 'ArduPlane-Missions/Dalby-OBC2016.txt'
FENCE = 'ArduPlane-Missions/Dalby-OBC2016-fence.txt'
WIND = "0,180,0.2"  # speed,direction,variance

homeloc = None


def fly_mission(mavproxy, mav, filename, fence, height_accuracy=-1):
    """Fly a mission from a file."""
    print("Flying mission %s" % filename)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('Flight plan received')
    mavproxy.send('fence load %s\n' % fence)
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')
    mavproxy.send('mode AUTO\n')
    wait_mode(mav, 'AUTO')
    if not wait_waypoint(mav, 1, 19, max_dist=60, timeout=1200):
        return False
    mavproxy.expect('DISARMED')
    # wait for blood sample here
    mavproxy.send('wp set 20\n')
    mavproxy.send('arm throttle\n')
    mavproxy.expect('ARMED')
    if not wait_waypoint(mav, 20, 34, max_dist=60, timeout=1200):
        return False
    mavproxy.expect('DISARMED')
    print("Mission OK")
    return True


def fly_QuadPlane(binary, viewerip=None, use_map=False, valgrind=False, gdb=False):
    """Fly QuadPlane in SITL.

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time.
    """
    global homeloc

    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
    if viewerip:
        options += " --out=%s:14550" % viewerip
    if use_map:
        options += ' --map'

    sitl = util.start_SITL(binary, model='quadplane', wipe=True, home=HOME_LOCATION, speedup=10,
                          defaults_file=os.path.join(testdir, 'default_params/quadplane.parm'), valgrind=valgrind, gdb=gdb)
    mavproxy = util.start_MAVProxy_SITL('QuadPlane', options=options)
    mavproxy.expect('Telemetry log: (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/QuadPlane-test.tlog")
    print("buildlog=%s" % buildlog)
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
        mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
        mav.wait_gps_fix()
        while mav.location().alt < 10:
            mav.wait_gps_fix()
        homeloc = mav.location()
        print("Home location: %s" % homeloc)

        # wait for EKF and GPS checks to pass
        wait_seconds(mav, 30)

        mavproxy.send('arm throttle\n')
        mavproxy.expect('ARMED')

        if not fly_mission(mavproxy, mav,
                           os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016.txt"),
                           os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016-fence.txt")):
            print("Failed mission")
            failed = True
    except pexpect.TIMEOUT as e:
        print("Failed with timeout")
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)

    valgrind_log = sitl.valgrind_log_filepath()
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/QuadPlane-valgrind.log"))

    if failed:
        print("FAILED: %s" % e)
        return False
    return True
