# fly ArduCopter in SIL

import util, pexpect, sys, time, math, shutil, os
from common import *
import mavutil, mavwp, random

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

FRAME='+'
TARGET='sitl'
HOME=mavutil.location(-35.362938,149.165085,584,270)

homeloc = None
num_wp = 0

def hover(mavproxy, mav, hover_throttle=1300):
    mavproxy.send('rc 3 %u\n' % hover_throttle)
    return True

def calibrate_level(mavproxy, mav):
    '''init the accelerometers'''
    print("Initialising accelerometers")
    mav.calibrate_level()
    mavproxy.expect(['APM: action received', 'COMMAND_ACK'])
    return True

def arm_motors(mavproxy, mav):
    '''arm motors'''
    print("Arming motors")
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 2000\n')
    mavproxy.expect('APM: ARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS ARMED OK")
    return True

def disarm_motors(mavproxy, mav):
    '''disarm motors'''
    print("Disarming motors")
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 1000\n')
    mavproxy.expect('APM: DISARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS DISARMED OK")
    return True


def takeoff(mavproxy, mav, alt_min = 30, takeoff_throttle=1510):
    '''takeoff get to 30m altitude'''
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 %u\n' % takeoff_throttle)
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    if (m.alt < alt_min):
        wait_altitude(mav, alt_min, (alt_min + 5))
    hover(mavproxy, mav)
    print("TAKEOFF COMPLETE")
    return True

def loiter(mavproxy, mav, holdtime=60, maxaltchange=20):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    start = mav.location()
    tstart = time.time()
    tholdstart = time.time()
    print("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))
    while time.time() < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        delta = get_distance(start, pos)
        print("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
        if math.fabs(m.alt - start_altitude) > maxaltchange:
            tholdstart = time.time()
        if time.time() - tholdstart > holdtime:
            print("Loiter OK for %u seconds" % holdtime)
            return True
    print("Loiter FAILED")
    return False

def change_alt(mavproxy, mav, alt_min, climb_throttle=1920, descend_throttle=1080):
    '''change altitude'''
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    if(m.alt < alt_min):
        print("Rise to alt:%u from %u" % (alt_min, m.alt))
        mavproxy.send('rc 3 %u\n' % climb_throttle)
        wait_altitude(mav, alt_min, (alt_min + 5))
    else:
        print("Lower to alt:%u from %u" % (alt_min, m.alt))
        mavproxy.send('rc 3 %u\n' % descend_throttle)
        wait_altitude(mav, (alt_min -5), alt_min)
    hover(mavproxy, mav)
    return True


def fly_square(mavproxy, mav, side=50, timeout=120):
    '''fly a square, flying N then E'''
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    tstart = time.time()
    failed = False

    print("Save WP 1 and 2")
    save_wp(mavproxy, mav)

    print("turn")
    hover(mavproxy, mav)
    mavproxy.send('rc 4 1610\n')
    if not wait_heading(mav, 0):
        return False
    mavproxy.send('rc 4 1500\n')

    print("Going north %u meters" % side)
    mavproxy.send('rc 2 1390\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 2 1500\n')

    print("Save WP 3")
    save_wp(mavproxy, mav)

    print("Going east %u meters" % side)
    mavproxy.send('rc 1 1610\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 1 1500\n')

    print("Save WP 4")
    save_wp(mavproxy, mav)

    print("Going south %u meters" % side)
    mavproxy.send('rc 2 1610\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 2 1500\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==1000', blocking=True)

    print("Save WP 5")
    save_wp(mavproxy, mav)

    print("Going west %u meters" % side)
    mavproxy.send('rc 1 1390\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 1 1500\n')

    print("Save WP 6")
    save_wp(mavproxy, mav)

    return not failed

def fly_RTL(mavproxy, mav, side=60, timeout=250):
    '''Fly, return, land'''
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    hover(mavproxy, mav)
    failed = False

    print("# Going forward %u meters" % side)
    mavproxy.send('rc 2 1350\n')
    if not wait_distance(mav, side, 5, 60):
        failed = True
    mavproxy.send('rc 2 1500\n')

    print("# Enter RTL")
    mavproxy.send('switch 3\n')
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        #delta = get_distance(start, pos)
        print("Alt: %u" % m.alt)
        if(m.alt <= 1):
            return True
    return True

def fly_failsafe(mavproxy, mav, side=60, timeout=180):
    '''Fly, Failsafe, return, land'''
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    hover(mavproxy, mav)
    failed = False

    print("# Going forward %u meters" % side)
    mavproxy.send('rc 2 1350\n')
    if not wait_distance(mav, side, 5, 60):
        failed = True
    mavproxy.send('rc 2 1500\n')

    print("# Enter Failsafe")
    mavproxy.send('rc 3 900\n')
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        print("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        if m.alt <= 1 and home_distance < 10:
            # switch modes to reset out of LAND
            mavproxy.send('switch 2\n')
            time.sleep(1)
            mavproxy.send('switch 6\n')
            print("Reached failsafe home OK")
            mavproxy.send('rc 3 1100\n')
            return True
    print("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
    # switch modes to reset out of LAND
    mavproxy.send('switch 2\n')
    time.sleep(1)
    mavproxy.send('switch 6\n')
    return False


def fly_simple(mavproxy, mav, side=60, timeout=120):
    '''fly Simple, flying N then E'''
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1400\n')

    tstart = time.time()
    failed = False

    print("# Going forward %u meters" % side)
    mavproxy.send('rc 2 1390\n')
    if not wait_distance(mav, side, 10, 60):
        failed = True
    mavproxy.send('rc 2 1500\n')

    print("# Going east for 30 seconds")
    mavproxy.send('rc 1 1610\n')
    tstart = time.time()
    while time.time() < (tstart + 30):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (time.time() - tstart)
        #print("%u" % delta)
    mavproxy.send('rc 1 1500\n')

    print("# Going back %u meters" % side)
    mavproxy.send('rc 2 1610\n')
    if not wait_distance(mav, side, 10, 60):
        failed = True
    mavproxy.send('rc 2 1500\n')
    #restore to default
    mavproxy.send('param set SIMPLE 0\n')
    hover(mavproxy, mav)
    return not failed




def land(mavproxy, mav, timeout=60):
    '''land the quad'''
    print("STARTING LANDING")
    mavproxy.send('switch 2\n')
    wait_mode(mav, 'LAND')
    print("Entered Landing Mode")
    ret = wait_altitude(mav, -5, 1)
    print("LANDING: ok= %s" % ret)
    return ret


def circle(mavproxy, mav, maxaltchange=10, holdtime=90, timeout=35):
    '''fly circle'''
    print("FLY CIRCLE")
    mavproxy.send('switch 1\n') # CIRCLE mode
    wait_mode(mav, 'CIRCLE')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = time.time()
    tholdstart = time.time()
    print("Circle at %u meters for %u seconds" % (start_altitude, holdtime))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("heading %u" % m.heading)

    print("CIRCLE OK for %u seconds" % holdtime)
    return True


def fly_mission(mavproxy, mav, height_accuracy=-1, target_altitude=None):
    '''fly a mission from a file'''
    global homeloc
    global num_wp
    print("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')
    mavproxy.send('switch 4\n') # auto mode
    wait_mode(mav, 'AUTO')
    #wait_altitude(mav, 30, 40)
    ret = wait_waypoint(mav, 0, num_wp, timeout=500)
    print("test: MISSION COMPLETE: passed=%s" % ret)
    # wait here until ready
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')

    return ret

    #if not wait_distance(mav, 30, timeout=120):
    #    return False
    #if not wait_location(mav, homeloc, timeout=600, target_altitude=target_altitude, height_accuracy=height_accuracy):
    #    return False


def load_mission_from_file(mavproxy, mav, filename):
    '''load a mission from a file'''
    global num_wp
    wploader = mavwp.MAVWPLoader()
    wploader.load(filename)
    num_wp = wploader.count()
    print("loaded mission with %u waypoints" % num_wp)
    return True

def upload_mission_from_file(mavproxy, mav, filename):
    '''Upload a mission from a file to APM'''
    global num_wp
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')
    return True

def save_mission_to_file(mavproxy, mav, filename):
    global num_wp
    mavproxy.send('wp save %s\n' % filename)
    mavproxy.expect('Saved ([0-9]+) waypoints')
    num_wp = int(mavproxy.match.group(1))
    print("num_wp: %d" % num_wp)
    return True

def setup_rc(mavproxy):
    '''setup RC override control'''
    for chan in range(1,9):
        mavproxy.send('rc %u 1500\n' % chan)
    # zero throttle
    mavproxy.send('rc 3 1000\n')


def fly_ArduCopter(viewerip=None):
    '''fly ArduCopter in SIL

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time
    '''
    global homeloc

    if TARGET != 'sitl':
        util.build_SIL('ArduCopter', target=TARGET)

    sim_cmd = util.reltopdir('Tools/autotest/pysim/sim_multicopter.py') + ' --frame=%s --rate=400 --home=%f,%f,%u,%u' % (
        FRAME, HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sim_cmd += ' --wind=6,45,.3'
    if viewerip:
        sim_cmd += ' --fgout=%s:5503' % viewerip

    sil = util.start_SIL('ArduCopter', wipe=True)
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send('param set SYSID_THISMAV %u\n' % random.randint(100, 200))
    mavproxy.send("param load %s/ArduCopter.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    sil = util.start_SIL('ArduCopter', height=HOME.alt)
    sim = pexpect.spawn(sim_cmd, logfile=sys.stdout, timeout=10)
    sim.delaybeforesend = 0
    util.pexpect_autoclose(sim)
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter --streamrate=5'
    if viewerip:
        options += ' --out=%s:14550' % viewerip
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduCopter-test.mavlog")
    print("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    os.link(logfile, buildlog)

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, expect_callback)

    expect_list_clear()
    expect_list_extend([sim, sil, mavproxy])

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
        mav.wait_heartbeat()
        setup_rc(mavproxy)
        homeloc = mav.location()

        print("# Calibrate level")
        if not calibrate_level(mavproxy, mav):
            print("calibrate_level failed")
            failed = True

        print("# Arm motors")
        if not arm_motors(mavproxy, mav):
            print("arm_motors failed")
            failed = True

        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        print("# Test RTL")
        if not fly_RTL(mavproxy, mav):
            print("RTL failed")
            failed = True

        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        print("# Test Failsafe")
        if not fly_failsafe(mavproxy, mav):
            print("FS failed")
            failed = True

        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # Loiter for 30 seconds
        print("# Loiter for 45 seconds")
        if not loiter(mavproxy, mav, 45):
            print("loiter failed")
            failed = True

        print("# Change alt to 60m")
        if not change_alt(mavproxy, mav, 60):
            print("change_alt failed")
            failed = True

        print("# Change alt to 20m")
        if not change_alt(mavproxy, mav, 20):
            print("change_alt failed")
            failed = True

        print("# Change alt to 20m")
        if not change_alt(mavproxy, mav, 20):
            print("change_alt failed")
            failed = True


        print("# Fly A square")
        if not fly_square(mavproxy, mav):
            print("fly_square failed")
            failed = True

        print("# Land")
        if not land(mavproxy, mav):
            print("land failed")
            failed = True

        print("Save landing WP")
        save_wp(mavproxy, mav)

        # save the stored mission
        print("# Save out the C7 mission")
        if not save_mission_to_file(mavproxy, mav, os.path.join(testdir, "ch7_mission.txt")):
            print("save_mission_to_file failed")
            failed = True

        # save the stored mission
        print("# Fly CH 7 saved mission")
        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            print("fly_mission failed")
            failed = True

        #set SIMPLE mode
        mavproxy.send('param set SIMPLE 63\n')

        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        print("# Fly in SIMPLE mode")
        if not fly_simple(mavproxy, mav):
            print("fly_simple failed")
            failed = True


        print("# Upload mission1")
        if not upload_mission_from_file(mavproxy, mav, os.path.join(testdir, "mission2.txt")):
            print("upload_mission_from_file failed")
            failed = True

        # this grabs our mission count
        print("# store mission1 locally")
        if not load_mission_from_file(mavproxy, mav, os.path.join(testdir, "mission2.txt")):
            print("load_mission_from_file failed")
            failed = True

        print("# Fly mission 2")
        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            print("fly_mission failed")
            failed = True
        else:
            print("Flew mission2 OK")

        print("# Land")
        if not land(mavproxy, mav):
            print("land failed")
            failed = True

        #print("# disarm motors")
        #if not disarm_motors(mavproxy, mav):
        #    print("disarm_motors failed")
        #    failed = True
    except pexpect.TIMEOUT, e:
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    util.pexpect_close(sim)

    if os.path.exists('ArduCopter-valgrind.log'):
        os.chmod('ArduCopter-valgrind.log', 0644)
        shutil.copy("ArduCopter-valgrind.log", util.reltopdir("../buildlogs/ArduCopter-valgrind.log"))

    if failed:
        print("FAILED: %s" % e)
        return False
    return True




