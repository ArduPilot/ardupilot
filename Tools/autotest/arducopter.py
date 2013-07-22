# fly ArduCopter in SITL
# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize

import util, pexpect, sys, time, math, shutil, os
from common import *
from pymavlink import mavutil, mavwp
import random

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

FRAME='+'
TARGET='sitl'
HOME=mavutil.location(-35.362938,149.165085,584,270)
AVCHOME=mavutil.location(40.072842,-105.230575,1586,0)

homeloc = None
num_wp = 0

def hover(mavproxy, mav, hover_throttle=1450):
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
    mav.motors_armed_wait()
    print("MOTORS ARMED OK")
    return True

def disarm_motors(mavproxy, mav):
    '''disarm motors'''
    print("Disarming motors")
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 1000\n')
    mavproxy.expect('APM: DISARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    mav.motors_disarmed_wait()
    print("MOTORS DISARMED OK")
    return True


def takeoff(mavproxy, mav, alt_min = 30, takeoff_throttle=1700):
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

# loiter - fly south west, then hold loiter within 5m position and altitude
def loiter(mavproxy, mav, holdtime=30, maxaltchange=5, maxdistchange=5):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # first aim south east
    print("turn south east")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 170):
        return False
    mavproxy.send('rc 4 1500\n')

    #fly south east 50m
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 50):
        return False
    mavproxy.send('rc 2 1500\n')

    # wait for copter to slow moving
    if not wait_groundspeed(mav, 0, 2):
        return False

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
            tholdstart = time.time()    # this will cause this test to timeout and fails
        if delta > maxdistchange:
            tholdstart = time.time()    # this will cause this test to timeout and fails
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

# fly a square in stabilize mode
def fly_square(mavproxy, mav, side=50, timeout=120):
    '''fly a square, flying N then E'''
    tstart = time.time()
    failed = False

    # ensure all sticks in the middle
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('rc 2 1500\n')
    mavproxy.send('rc 3 1500\n')
    mavproxy.send('rc 4 1500\n')

    # switch to loiter mode temporarily to stop us from rising
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')

    # first aim north
    print("turn right towards north")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 10):
        return False
    mavproxy.send('rc 4 1500\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan4_raw==1500', blocking=True)

    # save bottom left corner of box as waypoint
    print("Save WP 1 & 2")
    save_wp(mavproxy, mav)

    # switch back to stabilize mode
    mavproxy.send('rc 3 1400\n')
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')

    # pitch forward to fly north
    print("Going north %u meters" % side)
    mavproxy.send('rc 2 1300\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 2 1500\n')

    # save top left corner of square as waypoint
    print("Save WP 3")
    save_wp(mavproxy, mav)

    # roll right to fly east
    print("Going east %u meters" % side)
    mavproxy.send('rc 1 1700\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 1 1500\n')

    # save top right corner of square as waypoint
    print("Save WP 4")
    save_wp(mavproxy, mav)

    # pitch back to fly south
    print("Going south %u meters" % side)
    mavproxy.send('rc 2 1700\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 2 1500\n')

    # save bottom right corner of square as waypoint
    print("Save WP 5")
    save_wp(mavproxy, mav)

    # roll left to fly west
    print("Going west %u meters" % side)
    mavproxy.send('rc 1 1300\n')
    if not wait_distance(mav, side):
        failed = True
    mavproxy.send('rc 1 1500\n')

    # save bottom left corner of square (should be near home) as waypoint
    print("Save WP 6")
    save_wp(mavproxy, mav)

    return not failed

def fly_RTL(mavproxy, mav, side=60, timeout=250):
    '''Return, land'''
    print("# Enter RTL")
    mavproxy.send('switch 3\n')
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        print("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        if(m.alt <= 1 and home_distance < 10):
            return True
    return False

def fly_throttle_failsafe(mavproxy, mav, side=60, timeout=180):
    '''Fly east, Failsafe, return, land'''

    # switch to loiter mode temporarily to stop us from rising
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')

    # first aim east
    print("turn east")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 135):
        return False
    mavproxy.send('rc 4 1500\n')

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    hover(mavproxy, mav)
    failed = False

    # fly east 60 meters
    print("# Going forward %u meters" % side)
    mavproxy.send('rc 2 1350\n')
    if not wait_distance(mav, side, 5, 60):
        failed = True
    mavproxy.send('rc 2 1500\n')

    # pull throttle low
    print("# Enter Failsafe")
    mavproxy.send('rc 3 900\n')

    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        print("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        # check if we've reached home
        if m.alt <= 1 and home_distance < 10:
            # reduce throttle
            mavproxy.send('rc 3 1100\n')
            # switch back to stabilize
            mavproxy.send('switch 2\n') # land mode
            wait_mode(mav, 'LAND')
            mavproxy.send('switch 6\n') # stabilize mode
            wait_mode(mav, 'STABILIZE')
            print("Reached failsafe home OK")
            return True
    print("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
    # reduce throttle
    mavproxy.send('rc 3 1100\n')
    # switch back to stabilize mode
    mavproxy.send('switch 2\n') # land mode
    wait_mode(mav, 'LAND')
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    return False

# fly_stability_patch - fly south, then hold loiter within 5m position and altitude and reduce 1 motor to 60% efficiency
def fly_stability_patch(mavproxy, mav, holdtime=30, maxaltchange=5, maxdistchange=10):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # first south
    print("turn south")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 180):
        return False
    mavproxy.send('rc 4 1500\n')

    #fly west 80m
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 80):
        return False
    mavproxy.send('rc 2 1500\n')

    # wait for copter to slow moving
    if not wait_groundspeed(mav, 0, 2):
        return False

    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    start = mav.location()
    tstart = time.time()
    tholdstart = time.time()
    print("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))

    # cut motor 1 to 55% efficiency
    print("Cutting motor 1 to 55% efficiency")
    mavproxy.send('param set SIM_ENGINE_MUL 0.55\n')

    while time.time() < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        delta = get_distance(start, pos)
        print("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
        if math.fabs(m.alt - start_altitude) > maxaltchange:
            tholdstart = time.time()    # this will cause this test to timeout and fails
        if delta > maxdistchange:
            tholdstart = time.time()    # this will cause this test to timeout and fails
        if time.time() - tholdstart > holdtime:
            print("Stability patch and Loiter OK for %u seconds" % holdtime)
            # restore motor 1 to 100% efficiency
            mavproxy.send('param set SIM_ENGINE_MUL 1.0\n')
            return True
    print("Stability Patch FAILED")
    # restore motor 1 to 100% efficiency
    mavproxy.send('param set SIM_ENGINE_MUL 1.0\n')
    return False

# fly_fence_test - fly east until you hit the horizontal circular fence
def fly_fence_test(mavproxy, mav, timeout=180):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # enable fence
    mavproxy.send('param set FENCE_ENABLE 1\n')

    # first east
    print("turn east")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 160):
        return False
    mavproxy.send('rc 4 1500\n')

    # fly forward (east) at least 20m
    pitching_forward = True
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 20):
        return False

    # start timer
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        print("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        # recenter pitch sticks once we reach home so we don't fly off again
        if pitching_forward and home_distance < 10 :
            pitching_forward = False
            mavproxy.send('rc 2 1500\n')
            # disable fence
            mavproxy.send('param set FENCE_ENABLE 0\n')
        if m.alt <= 1 and home_distance < 10:
            # reduce throttle
            mavproxy.send('rc 3 1000\n')
            # switch mode to stabilize
            mavproxy.send('switch 2\n') # land mode
            wait_mode(mav, 'LAND')
            mavproxy.send('switch 6\n') # stabilize mode
            wait_mode(mav, 'STABILIZE')
            print("Reached home OK")
            return True
    # disable fence
    mavproxy.send('param set FENCE_ENABLE 0\n')
    # reduce throttle
    mavproxy.send('rc 3 1000\n')
    # switch mode to stabilize
    mavproxy.send('switch 2\n') # land mode
    wait_mode(mav, 'LAND')
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    print("Fence test failed to reach home - timed out after %u seconds" % timeout)
    return False

#fly_simple - assumes the simple bearing is initialised to be directly north
#   flies a box with 100m west, 15 seconds north, 50 seconds east, 15 seconds south
def fly_simple(mavproxy, mav, side=100, timeout=120):

    #set SIMPLE mode
    mavproxy.send('param set SIMPLE 63\n')

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1400\n')

    tstart = time.time()
    failed = False

    # fly west 100m
    print("# Flying west %u meters" % side)
    mavproxy.send('rc 1 1300\n')
    if not wait_distance(mav, side, 5, 60):
        failed = True
    mavproxy.send('rc 1 1500\n')

    # fly north 15 seconds
    print("# Flying north for 15 seconds")
    mavproxy.send('rc 2 1300\n')
    tstart = time.time()
    while time.time() < (tstart + 15):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (time.time() - tstart)
        #print("%u" % delta)
    mavproxy.send('rc 2 1500\n')

    # fly east 50 meters
    print("# Flying east %u meters" % (side/2.0))
    mavproxy.send('rc 1 1700\n')
    if not wait_distance(mav, side/2, 5, 60):
        failed = True
    mavproxy.send('rc 1 1500\n')

    # fly south 15 seconds
    print("# Flying south for 15 seconds")
    mavproxy.send('rc 2 1700\n')
    tstart = time.time()
    while time.time() < (tstart + 15):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (time.time() - tstart)
        #print("%u" % delta)
    mavproxy.send('rc 2 1500\n')

    #restore to default
    mavproxy.send('param set SIMPLE 0\n')

    #hover in place
    hover(mavproxy, mav)
    return not failed

#fly_circle - flies a circle with 20m radius
def fly_circle(mavproxy, mav, maxaltchange=10, holdtime=36):

    # hold position in loiter
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # face west
    print("turn west")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 270):
        return False
    mavproxy.send('rc 4 1500\n')

    #set CIRCLE radius
    mavproxy.send('param set CIRCLE_RADIUS 30\n')

    # fly forward (east) at least 100m
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 100):
        return False

    # return pitch stick back to middle
    mavproxy.send('rc 2 1500\n')

    # set CIRCLE mode
    mavproxy.send('switch 1\n') # circle mode
    wait_mode(mav, 'CIRCLE')

    # wait 
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = time.time()
    tholdstart = time.time()
    print("Circle at %u meters for %u seconds" % (start_altitude, holdtime))
    while time.time() < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("heading %u" % m.heading)

    print("CIRCLE OK for %u seconds" % holdtime)
    return True

def land(mavproxy, mav, timeout=60):
    '''land the quad'''
    print("STARTING LANDING")
    mavproxy.send('switch 2\n') # land mode
    wait_mode(mav, 'LAND')
    print("Entered Landing Mode")
    ret = wait_altitude(mav, -5, 1)
    print("LANDING: ok= %s" % ret)
    return ret

def fly_mission(mavproxy, mav, height_accuracy=-1, target_altitude=None):
    '''fly a mission from a file'''
    global homeloc
    global num_wp
    print("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')
    mavproxy.send('switch 4\n') # auto mode
    wait_mode(mav, 'AUTO')
    #wait_altitude(mav, 30, 40)
    ret = wait_waypoint(mav, 0, num_wp, timeout=500, mode='AUTO')
    print("test: MISSION COMPLETE: passed=%s" % ret)
    # wait here until ready
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')
    return ret

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


def fly_ArduCopter(viewerip=None, map=False):
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
    if map:
        options += ' --map'
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduCopter-test.tlog")
    print("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        pass

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

        # Arm
        print("# Arm motors")
        if not arm_motors(mavproxy, mav):
            print("arm_motors failed")
            failed = True

        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # Fly a square in Stabilize mode
        print("#")
        print("########## Fly A square and save WPs with CH7 switch ##########")
        print("#")
        if not fly_square(mavproxy, mav):
            print("fly_square failed")
            failed = True

        print("# Land")
        if not land(mavproxy, mav):
            print("land failed")
            failed = True

        print("Save landing WP")
        save_wp(mavproxy, mav)
        mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==1000', blocking=True)

        # save the stored mission to file
        print("# Save out the CH7 mission to file")
        if not save_mission_to_file(mavproxy, mav, os.path.join(testdir, "ch7_mission.txt")):
            print("save_mission_to_file failed")
            failed = True

        # fly the stored mission
        print("# Fly CH7 saved mission")
        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            print("fly_mission failed")
            failed = True

        # Throttle Failsafe
        print("#")
        print("########## Test Failsafe ##########")
        print("#")
        if not fly_throttle_failsafe(mavproxy, mav):
            print("FS failed")
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # Stability patch
        print("#")
        print("########## Test Stability Patch ##########")
        print("#")
        if not fly_stability_patch(mavproxy, mav, 30):
            print("Stability Patch failed")
            failed = True

        # RTL
        print("# RTL #")
        if not fly_RTL(mavproxy, mav):
            print("RTL failed")
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # Fence test
        print("#")
        print("########## Test Horizontal Fence ##########")
        print("#")
        if not fly_fence_test(mavproxy, mav, 180):
            print("Fence test failed")
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # Loiter for 30 seconds
        print("#")
        print("########## Test Loiter for 30 seconds ##########")
        print("#")
        if not loiter(mavproxy, mav, 30):
            print("loiter failed")
            failed = True

        # Loiter Climb
        print("#")
        print("# Loiter - climb to 60m")
        print("#")
        if not change_alt(mavproxy, mav, 60):
            print("change_alt failed")
            failed = True

        # Loiter Descend
        print("#")
        print("# Loiter - descend to 20m")
        print("#")
        if not change_alt(mavproxy, mav, 20):
            print("change_alt failed")
            failed = True

        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # RTL
        print("#")
        print("########## Test RTL ##########")
        print("#")
        if not fly_RTL(mavproxy, mav):
            print("RTL failed")
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # Simple mode
        print("# Fly in SIMPLE mode")
        if not fly_simple(mavproxy, mav):
            print("fly_simple failed")
            failed = True

        # RTL
        print("#")
        print("########## Test RTL ##########")
        print("#")
        if not fly_RTL(mavproxy, mav):
            print("RTL failed")
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            print("takeoff failed")
            failed = True

        # Circle mode
        print("# Fly CIRCLE mode")
        if not fly_circle(mavproxy, mav):
            print("fly_circle failed")
            failed = True

        # RTL
        print("#")
        print("########## Test RTL ##########")
        print("#")
        if not fly_RTL(mavproxy, mav):
            print("RTL failed")
            failed = True

        # Fly mission #1
        print("# Upload copter_mission")
        if not upload_mission_from_file(mavproxy, mav, os.path.join(testdir, "copter_mission.txt")):
            print("upload_mission_from_file failed")
            failed = True

        # this grabs our mission count
        print("# store copter_mission locally")
        if not load_mission_from_file(mavproxy, mav, os.path.join(testdir, "copter_mission.txt")):
            print("load_mission_from_file failed")
            failed = True

        print("# Fly mission 1")
        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            print("fly_mission failed")
            failed = True
        else:
            print("Flew mission 1 OK")

        #mission includes LAND at end so should be ok to disamr
        print("# disarm motors")
        if not disarm_motors(mavproxy, mav):
            print("disarm_motors failed")
            failed = True

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


def fly_CopterAVC(viewerip=None, map=False):
    '''fly ArduCopter in SIL for AVC2013 mission
    '''
    global homeloc

    if TARGET != 'sitl':
        util.build_SIL('ArduCopter', target=TARGET)

    sim_cmd = util.reltopdir('Tools/autotest/pysim/sim_multicopter.py') + ' --frame=%s --rate=400 --home=%f,%f,%u,%u' % (
        FRAME, AVCHOME.lat, AVCHOME.lng, AVCHOME.alt, AVCHOME.heading)
    if viewerip:
        sim_cmd += ' --fgout=%s:5503' % viewerip

    sil = util.start_SIL('ArduCopter', wipe=True)
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send('param set SYSID_THISMAV %u\n' % random.randint(100, 200))
    mavproxy.send("param load %s/CopterAVC.parm\n" % testdir)
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
    if map:
        options += ' --map'
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/CopterAVC-test.tlog")
    print("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        pass

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, expect_callback)

    expect_list_clear()
    expect_list_extend([sim, sil, mavproxy])

    if map:
        mavproxy.send('map icon 40.072467969730496 -105.2314389590174\n')
        mavproxy.send('map icon 40.072600990533829 -105.23146100342274\n')        

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

        # Arm
        print("# Arm motors")
        if not arm_motors(mavproxy, mav):
            print("arm_motors failed")
            failed = True

        # Fly mission #1
        print("# Upload AVC mission")
        if not upload_mission_from_file(mavproxy, mav, os.path.join(testdir, "AVC2013.txt")):
            print("upload_mission_from_file failed")
            failed = True

        # this grabs our mission count
        print("# store mission1 locally")
        if not load_mission_from_file(mavproxy, mav, os.path.join(testdir, "AVC2013.txt")):
            print("load_mission_from_file failed")
            failed = True

        print("# raising throttle")
        mavproxy.send('rc 3 1300\n')

        print("# Fly mission 1")
        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            print("fly_mission failed")
            failed = True
        else:
            print("Flew mission 1 OK")

        print("# lowering throttle")
        mavproxy.send('rc 3 1000\n')

        #mission includes LAND at end so should be ok to disamr
        print("# disarm motors")
        if not disarm_motors(mavproxy, mav):
            print("disarm_motors failed")
            failed = True

    except pexpect.TIMEOUT, e:
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    util.pexpect_close(sim)

    if failed:
        print("FAILED: %s" % e)
        return False
    return True
