# drive APMrover2 in SITL

import util, pexpect, sys, time, math, shutil, os
from common import *
from pymavlink import mavutil
import random

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))


#HOME=mavutil.location(-35.362938,149.165085,584,270)
HOME=mavutil.location(40.071374969556928,-105.22978898137808,1583.702759,246)
homeloc = None

def drive_left_circuit(mavproxy, mav):
    '''drive a left circuit, 50m on a side'''
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'MANUAL')
    mavproxy.send('rc 3 2000\n')

    print("Driving left circuit")
    # do 4 turns
    for i in range(0,4):
        # hard left
        print("Starting turn %u" % i)
        mavproxy.send('rc 1 1000\n')
        if not wait_heading(mav, 270 - (90*i), accuracy=10):
            return False
        mavproxy.send('rc 1 1500\n')
        print("Starting leg %u" % i)
        if not wait_distance(mav, 50, accuracy=7):
            return False
    mavproxy.send('rc 3 1500\n')
    print("Circuit complete")
    return True

def drive_RTL(mavproxy, mav):
    '''drive to home'''
    print("Driving home in RTL")
    mavproxy.send('switch 3\n')
    if not wait_location(mav, homeloc, accuracy=22, timeout=90):
        return False
    print("RTL Complete")
    return True

def setup_rc(mavproxy):
    '''setup RC override control'''
    for chan in [1,2,3,4,5,6,7]:
        mavproxy.send('rc %u 1500\n' % chan)
    mavproxy.send('rc 8 1800\n')


def drive_mission(mavproxy, mav, filename):
    '''drive a mission from a file'''
    global homeloc
    print("Driving mission %s" % filename)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')
    mavproxy.send('switch 4\n') # auto mode
    mavproxy.send('rc 3 1500\n')
    wait_mode(mav, 'AUTO')
    if not wait_waypoint(mav, 1, 4, max_dist=5):
        return False
    wait_mode(mav, 'HOLD')
    print("Mission OK")
    return True


def drive_APMrover2(viewerip=None, map=False):
    '''drive APMrover2 in SIL

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the mission in real time
    '''
    global homeloc

    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
    if viewerip:
        options += " --out=%s:14550" % viewerip
    if map:
        options += ' --map'

    sil = util.start_SIL('APMrover2', wipe=True)
    mavproxy = util.start_MAVProxy_SIL('APMrover2', options=options)
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send('param set SYSID_THISMAV %u\n' % random.randint(100, 200))
    mavproxy.send("param load %s/Rover.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')

    # restart with new parms
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    sim_cmd = util.reltopdir('Tools/autotest/pysim/sim_rover.py') + ' --rate=50 --home=%f,%f,%u,%u' % (
        HOME.lat, HOME.lng, HOME.alt, HOME.heading)

    runsim = pexpect.spawn(sim_cmd, logfile=sys.stdout, timeout=10)
    runsim.delaybeforesend = 0
    util.pexpect_autoclose(runsim)
    runsim.expect('Starting at lat')

    sil = util.start_SIL('APMrover2')
    mavproxy = util.start_MAVProxy_SIL('APMrover2', options=options)
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/APMrover2-test.tlog")
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
    expect_list_extend([runsim, sil, mavproxy])

    print("Started simulator")

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception, msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)

    failed = False
    e = 'None'
    try:
        print("Waiting for a heartbeat with mavlink protocol %s" % mav.WIRE_PROTOCOL_VERSION)
        mav.wait_heartbeat()
        print("Setting up RC parameters")
        setup_rc(mavproxy)
        print("Waiting for GPS fix")
        mav.wait_gps_fix()
        homeloc = mav.location()
        print("Home location: %s" % homeloc)
        if not drive_mission(mavproxy, mav, os.path.join(testdir, "rover1.txt")):
            print("Failed mission")
            failed = True
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/APMrover2-log.bin")):
            print("Failed log download")
            failed = True
#        if not drive_left_circuit(mavproxy, mav):
#            print("Failed left circuit")
#            failed = True
#        if not drive_RTL(mavproxy, mav):
#            print("Failed RTL")
#            failed = True
    except pexpect.TIMEOUT, e:
        print("Failed with timeout")
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    util.pexpect_close(runsim)

    if os.path.exists('APMrover2-valgrind.log'):
        os.chmod('APMrover2-valgrind.log', 0644)
        shutil.copy("APMrover2-valgrind.log", util.reltopdir("../buildlogs/APMrover2-valgrind.log"))

    if failed:
        print("FAILED: %s" % e)
        return False
    return True
