# fly ArduPlane in SIL

import util, pexpect, sys, time, math, shutil, os
from common import *
from pymavlink import mavutil
import random

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))


HOME_LOCATION='-35.362938,149.165085,585,354'
WIND="0,180,0.2" # speed,direction,variance

homeloc = None

def takeoff(mavproxy, mav):
    '''takeoff get to 30m altitude'''

    # wait for EKF to settle
    wait_seconds(mav, 15)

    mavproxy.send('arm throttle\n')
    mavproxy.expect('ARMED')
    
    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')

    # some rudder to counteract the prop torque
    mavproxy.send('rc 4 1700\n')

    # some up elevator to keep the tail down
    mavproxy.send('rc 2 1200\n')

    # get it moving a bit first
    mavproxy.send('rc 3 1300\n')
    mav.recv_match(condition='VFR_HUD.groundspeed>6', blocking=True)

    # a bit faster again, straighten rudder
    mavproxy.send('rc 3 1600\n')
    mavproxy.send('rc 4 1500\n')
    mav.recv_match(condition='VFR_HUD.groundspeed>12', blocking=True)

    # hit the gas harder now, and give it some more elevator
    mavproxy.send('rc 2 1100\n')
    mavproxy.send('rc 3 2000\n')

    # gain a bit of altitude
    if not wait_altitude(mav, homeloc.alt+150, homeloc.alt+180, timeout=30):
        return False

    # level off
    mavproxy.send('rc 2 1500\n')

    print("TAKEOFF COMPLETE")
    return True

def fly_left_circuit(mavproxy, mav):
    '''fly a left circuit, 200m on a side'''
    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')
    mavproxy.send('rc 3 2000\n')
    if not wait_level_flight(mavproxy, mav):
        return False

    print("Flying left circuit")
    # do 4 turns
    for i in range(0,4):
        # hard left
        print("Starting turn %u" % i)
        mavproxy.send('rc 1 1000\n')
        if not wait_heading(mav, 270 - (90*i), accuracy=10):
            return False
        mavproxy.send('rc 1 1500\n')
        print("Starting leg %u" % i)
        if not wait_distance(mav, 100, accuracy=20):
            return False
    print("Circuit complete")
    return True

def fly_RTL(mavproxy, mav):
    '''fly to home'''
    print("Flying home in RTL")
    mavproxy.send('switch 2\n')
    wait_mode(mav, 'RTL')
    if not wait_location(mav, homeloc, accuracy=120,
                         target_altitude=homeloc.alt+100, height_accuracy=20,
                         timeout=180):
        return False
    print("RTL Complete")
    return True

def fly_LOITER(mavproxy, mav, num_circles=4):
    '''loiter where we are'''
    print("Testing LOITER for %u turns" % num_circles)
    mavproxy.send('loiter\n')
    wait_mode(mav, 'LOITER')

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    initial_alt = m.alt
    print("Initial altitude %u\n" % initial_alt)

    while num_circles > 0:
        if not wait_heading(mav, 0, accuracy=10, timeout=60):
            return False
        if not wait_heading(mav, 180, accuracy=10, timeout=60):
            return False
        num_circles -= 1
        print("Loiter %u circles left" % num_circles)

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    final_alt = m.alt
    print("Final altitude %u initial %u\n" % (final_alt, initial_alt))

    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    if abs(final_alt - initial_alt) > 20:
        print("Failed to maintain altitude")
        return False

    print("Completed Loiter OK")
    return True

def fly_CIRCLE(mavproxy, mav, num_circles=1):
    '''circle where we are'''
    print("Testing CIRCLE for %u turns" % num_circles)
    mavproxy.send('mode CIRCLE\n')
    wait_mode(mav, 'CIRCLE')

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    initial_alt = m.alt
    print("Initial altitude %u\n" % initial_alt)

    while num_circles > 0:
        if not wait_heading(mav, 0, accuracy=10, timeout=60):
            return False
        if not wait_heading(mav, 180, accuracy=10, timeout=60):
            return False
        num_circles -= 1
        print("CIRCLE %u circles left" % num_circles)

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    final_alt = m.alt
    print("Final altitude %u initial %u\n" % (final_alt, initial_alt))

    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    if abs(final_alt - initial_alt) > 20:
        print("Failed to maintain altitude")
        return False

    print("Completed CIRCLE OK")
    return True


def wait_level_flight(mavproxy, mav, accuracy=5, timeout=30):
    '''wait for level flight'''
    tstart = get_sim_time(mav)
    print("Waiting for level flight")
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('rc 2 1500\n')
    mavproxy.send('rc 4 1500\n')
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='ATTITUDE', blocking=True)
        roll = math.degrees(m.roll)
        pitch = math.degrees(m.pitch)
        print("Roll=%.1f Pitch=%.1f" % (roll, pitch))
        if math.fabs(roll) <= accuracy and math.fabs(pitch) <= accuracy:
            print("Attained level flight")
            return True
    print("Failed to attain level flight")
    return False


def change_altitude(mavproxy, mav, altitude, accuracy=30):
    '''get to a given altitude'''
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')
    alt_error = mav.messages['VFR_HUD'].alt - altitude
    if alt_error > 0:
        mavproxy.send('rc 2 2000\n')
    else:
        mavproxy.send('rc 2 1000\n')
    if not wait_altitude(mav, altitude-accuracy/2, altitude+accuracy/2):
        return False
    mavproxy.send('rc 2 1500\n')
    print("Reached target altitude at %u" % mav.messages['VFR_HUD'].alt)
    return wait_level_flight(mavproxy, mav)


def axial_left_roll(mavproxy, mav, count=1):
    '''fly a left axial roll'''
    # full throttle!
    mavproxy.send('rc 3 2000\n')
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False

    # fly the roll in manual
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'MANUAL')

    while count > 0:
        print("Starting roll")
        mavproxy.send('rc 1 1000\n')
        if not wait_roll(mav, -150, accuracy=90):
            mavproxy.send('rc 1 1500\n')
            return False
        if not wait_roll(mav, 150, accuracy=90):
            mavproxy.send('rc 1 1500\n')
            return False
        if not wait_roll(mav, 0, accuracy=90):
            mavproxy.send('rc 1 1500\n')
            return False
        count -= 1

    # back to FBWA
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')
    mavproxy.send('rc 3 1700\n')
    return wait_level_flight(mavproxy, mav)


def inside_loop(mavproxy, mav, count=1):
    '''fly a inside loop'''
    # full throttle!
    mavproxy.send('rc 3 2000\n')
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False

    # fly the loop in manual
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'MANUAL')

    while count > 0:
        print("Starting loop")
        mavproxy.send('rc 2 1000\n')
        if not wait_pitch(mav, -60, accuracy=20):
            return False
        if not wait_pitch(mav, 0, accuracy=20):
            return False
        count -= 1

    # back to FBWA
    mavproxy.send('rc 2 1500\n')
    mavproxy.send('switch 4\n')
    wait_mode(mav, 'FBWA')
    mavproxy.send('rc 3 1700\n')
    return wait_level_flight(mavproxy, mav)


def test_stabilize(mavproxy, mav, count=1):
    '''fly stabilize mode'''
    # full throttle!
    mavproxy.send('rc 3 2000\n')
    mavproxy.send('rc 2 1300\n')
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False
    mavproxy.send('rc 2 1500\n')

    mavproxy.send("mode STABILIZE\n")
    wait_mode(mav, 'STABILIZE')

    count = 1
    while count > 0:
        print("Starting roll")
        mavproxy.send('rc 1 2000\n')
        if not wait_roll(mav, -150, accuracy=90):
            return False
        if not wait_roll(mav, 150, accuracy=90):
            return False
        if not wait_roll(mav, 0, accuracy=90):
            return False
        count -= 1

    mavproxy.send('rc 1 1500\n')
    if not wait_roll(mav, 0, accuracy=5):
        return False

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')
    mavproxy.send('rc 3 1700\n')
    return wait_level_flight(mavproxy, mav)

def test_acro(mavproxy, mav, count=1):
    '''fly ACRO mode'''
    # full throttle!
    mavproxy.send('rc 3 2000\n')
    mavproxy.send('rc 2 1300\n')
    if not change_altitude(mavproxy, mav, homeloc.alt+300):
        return False
    mavproxy.send('rc 2 1500\n')

    mavproxy.send("mode ACRO\n")
    wait_mode(mav, 'ACRO')

    count = 1
    while count > 0:
        print("Starting roll")
        mavproxy.send('rc 1 1000\n')
        if not wait_roll(mav, -150, accuracy=90):
            return False
        if not wait_roll(mav, 150, accuracy=90):
            return False
        if not wait_roll(mav, 0, accuracy=90):
            return False
        count -= 1
    mavproxy.send('rc 1 1500\n')

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    wait_level_flight(mavproxy, mav)

    mavproxy.send("mode ACRO\n")
    wait_mode(mav, 'ACRO')

    count = 2
    while count > 0:
        print("Starting loop")
        mavproxy.send('rc 2 1000\n')
        if not wait_pitch(mav, -60, accuracy=20):
            return False
        if not wait_pitch(mav, 0, accuracy=20):
            return False
        count -= 1

    mavproxy.send('rc 2 1500\n')

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')
    mavproxy.send('rc 3 1700\n')
    return wait_level_flight(mavproxy, mav)


def test_FBWB(mavproxy, mav, count=1, mode='FBWB'):
    '''fly FBWB or CRUISE mode'''
    mavproxy.send("mode %s\n" % mode)
    wait_mode(mav, mode)
    mavproxy.send('rc 3 1700\n')
    mavproxy.send('rc 2 1500\n')

    # lock in the altitude by asking for an altitude change then releasing
    mavproxy.send('rc 2 1000\n')
    wait_distance(mav, 50, accuracy=20)
    mavproxy.send('rc 2 1500\n')
    wait_distance(mav, 50, accuracy=20)

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    initial_alt = m.alt
    print("Initial altitude %u\n" % initial_alt)

    print("Flying right circuit")
    # do 4 turns
    for i in range(0,4):
        # hard left
        print("Starting turn %u" % i)
        mavproxy.send('rc 1 1800\n')
        if not wait_heading(mav, 0 + (90*i), accuracy=20, timeout=60):
            mavproxy.send('rc 1 1500\n')
            return False
        mavproxy.send('rc 1 1500\n')
        print("Starting leg %u" % i)
        if not wait_distance(mav, 100, accuracy=20):
            return False
    print("Circuit complete")

    print("Flying rudder left circuit")
    # do 4 turns
    for i in range(0,4):
        # hard left
        print("Starting turn %u" % i)
        mavproxy.send('rc 4 1900\n')
        if not wait_heading(mav, 360 - (90*i), accuracy=20, timeout=60):
            mavproxy.send('rc 4 1500\n')
            return False
        mavproxy.send('rc 4 1500\n')
        print("Starting leg %u" % i)
        if not wait_distance(mav, 100, accuracy=20):
            return False
    print("Circuit complete")

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    final_alt = m.alt
    print("Final altitude %u initial %u\n" % (final_alt, initial_alt))

    # back to FBWA
    mavproxy.send('mode FBWA\n')
    wait_mode(mav, 'FBWA')

    if abs(final_alt - initial_alt) > 20:
        print("Failed to maintain altitude")
        return False
    
    return wait_level_flight(mavproxy, mav)


def setup_rc(mavproxy):
    '''setup RC override control'''
    for chan in [1,2,4,5,6,7]:
        mavproxy.send('rc %u 1500\n' % chan)
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 8 1800\n')


def fly_mission(mavproxy, mav, filename, height_accuracy=-1, target_altitude=None):
    '''fly a mission from a file'''
    global homeloc
    print("Flying mission %s" % filename)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('Flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')
    mavproxy.send('switch 1\n') # auto mode
    wait_mode(mav, 'AUTO')
    if not wait_waypoint(mav, 1, 7, max_dist=60):
        return False
    if not wait_groundspeed(mav, 0, 0.5, timeout=60):
        return False
    print("Mission OK")
    return True


def fly_ArduPlane(binary, viewerip=None, map=False, valgrind=False):
    '''fly ArduPlane in SIL

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time
    '''
    global homeloc

    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
    if viewerip:
        options += " --out=%s:14550" % viewerip
    if map:
        options += ' --map'

    sil = util.start_SIL(binary, wipe=True, model='jsbsim', home=HOME_LOCATION, speedup=10)
    print("Starting MAVProxy")
    mavproxy = util.start_MAVProxy_SIL('ArduPlane', options=options)
    util.expect_setup_callback(mavproxy, expect_callback)

    mavproxy.expect('Telemetry log: (\S+)')
    mavproxy.expect('Received [0-9]+ parameters',timeout=3000)

    # setup test parameters
    mavproxy.send("param load %s/ArduPlane.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')
    mavproxy.send("param set LOG_REPLAY 1\n")
    mavproxy.send("param set LOG_DISARMED 1\n")
    time.sleep(3)

    mavproxy.send("param fetch\n")

    # restart with new parms
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    sil = util.start_SIL(binary, model='jsbsim', home=HOME_LOCATION, speedup=10, valgrind=valgrind)
    mavproxy = util.start_MAVProxy_SIL('ArduPlane', options=options)
    mavproxy.expect('Telemetry log: (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduPlane-test.tlog")
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
    expect_list_extend([sil, mavproxy])

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
        mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
        mav.wait_gps_fix()
        while mav.location().alt < 10:
            mav.wait_gps_fix()
        homeloc = mav.location()
        print("Home location: %s" % homeloc)
        if not takeoff(mavproxy, mav):
            print("Failed takeoff")
            failed = True
        if not fly_left_circuit(mavproxy, mav):
            print("Failed left circuit")
            failed = True
        if not axial_left_roll(mavproxy, mav, 1):
            print("Failed left roll")
            failed = True
        if not inside_loop(mavproxy, mav):
            print("Failed inside loop")
            failed = True
        if not test_stabilize(mavproxy, mav):
            print("Failed stabilize test")
            failed = True
        if not test_acro(mavproxy, mav):
            print("Failed ACRO test")
            failed = True
        if not test_FBWB(mavproxy, mav):
            print("Failed FBWB test")
            failed = True
        if not test_FBWB(mavproxy, mav, mode='CRUISE'):
            print("Failed CRUISE test")
            failed = True
        if not fly_RTL(mavproxy, mav):
            print("Failed RTL")
            failed = True
        if not fly_LOITER(mavproxy, mav):
            print("Failed LOITER")
            failed = True
        if not fly_CIRCLE(mavproxy, mav):
            print("Failed CIRCLE")
            failed = True
        if not fly_mission(mavproxy, mav, os.path.join(testdir, "ap1.txt"), height_accuracy = 10,
                           target_altitude=homeloc.alt+100):
            print("Failed mission")
            failed = True
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduPlane-log.bin")):
            print("Failed log download")
            failed = True
    except pexpect.TIMEOUT, e:
        print("Failed with timeout")
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    valgrind_log = sil.valgrind_log_filepath()
    if os.path.exists(valgrind_log):
        os.chmod(valgrind_log, 0644)
        shutil.copy(valgrind_log, util.reltopdir("../buildlogs/ArduPlane-valgrind.log"))

    if failed:
        print("FAILED: %s" % e)
        return False
    return True
