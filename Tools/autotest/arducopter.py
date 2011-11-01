# fly ArduCopter in SIL

import util, pexpect, sys, time, math, shutil, os

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

sys.path.insert(0, util.reltopdir('../pymavlink'))
import mavutil

HOME_LOCATION='-35.362938,149.165085,650,270'

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

def message_hook(mav, msg):
    '''called as each mavlink msg is received'''
    global expect_list
    if msg.get_type() in [ 'NAV_CONTROLLER_OUTPUT' ]:
        print(msg)
    for p in expect_list:
        try:
            p.read_nonblocking(100, timeout=0)
        except pexpect.TIMEOUT:
            pass

def expect_callback(e):
    '''called when waiting for a expect pattern'''
    global expect_list
    for p in expect_list:
        if p == e:
            continue
        try:
            while p.read_nonblocking(100, timeout=0):
                pass
        except pexpect.TIMEOUT:
            pass


class location(object):
    '''represent a GPS coordinate'''
    def __init__(self, lat, lng, alt=0):
        self.lat = lat
        self.lng = lng
        self.alt = alt

def get_distance(loc1, loc2):
    '''get ground distance between two locations'''
    dlat 		= loc2.lat - loc1.lat
    dlong		= loc2.lng - loc1.lng
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(loc1, loc2):
    '''get bearing from loc1 to loc2'''
    off_x = loc2.lng - loc1.lng
    off_y = loc2.lat - loc1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

def current_location(mav):
    '''return current location'''
    return location(mav.messages['GPS_RAW'].lat,
                    mav.messages['GPS_RAW'].lon,
                    mav.messages['VFR_HUD'].alt)

def wait_altitude(mav, alt_min, alt_max, timeout=30):
    '''wait for a given altitude range'''
    tstart = time.time()
    print("Waiting for altitude between %u and %u" % (alt_min, alt_max))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Altitude %u" % m.alt)
        if m.alt >= alt_min and m.alt <= alt_max:
            return True
    print("Failed to attain altitude range")
    return False
                    

def arm_motors(mavproxy):
    '''arm motors'''
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.expect('STABILIZE>')
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 2000\n')
    mavproxy.expect('APM: ARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS ARMED OK")

def disarm_motors(mavproxy):
    '''disarm motors'''
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 1000\n')
    mavproxy.expect('APM: DISARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS DISARMED OK")
    

def takeoff(mavproxy, mav):
    '''takeoff get to 30m altitude'''
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.expect('STABILIZE>')
    mavproxy.send('rc 3 1500\n')
    wait_altitude(mav, 30, 40)
    print("TAKEOFF COMPLETE")


def loiter(mavproxy, mav, maxaltchange=10, holdtime=10, timeout=60):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    mavproxy.expect('LOITER>')
    mavproxy.send('status\n')
    mavproxy.expect('>')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = time.time()
    tholdstart = time.time()
    print("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Altitude %u" % m.alt)
        if math.fabs(m.alt - start_altitude) > maxaltchange:
            tholdstart = time.time()
        if time.time() - tholdstart > holdtime:
            print("Loiter OK for %u seconds" % holdtime)
            return True
    print("Loiter FAILED")
    return False


def wait_heading(mav, heading, accuracy=5, timeout=30):
    '''wait for a given heading'''
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Heading %u" % m.heading)
        if math.fabs(m.heading - heading) <= accuracy:
            return True
    print("Failed to attain heading %u" % heading)
    return False


def wait_distance(mav, distance, accuracy=5, timeout=30):
    '''wait for flight of a given distance'''
    tstart = time.time()
    start = current_location(mav)
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='GPS_RAW', blocking=True)
        pos = current_location(mav)
        delta = get_distance(start, pos)
        print("Distance %.2f meters" % delta)
        if math.fabs(delta - distance) <= accuracy:
            return True
    print("Failed to attain distance %u" % distance)
    return False

def wait_location(mav, loc, accuracy=5, timeout=30):
    '''wait for arrival at a location'''
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='GPS_RAW', blocking=True)
        pos = current_location(mav)
        delta = get_distance(loc, pos)
        print("Distance %.2f meters" % delta)
        if delta <= accuracy:
            return True
    print("Failed to attain location")
    return False


def fly_square(mavproxy, mav, side=50, timeout=120):
    '''fly a square, flying N then E'''
    mavproxy.send('switch 6\n')
    mavproxy.expect('STABILIZE>')
    tstart = time.time()
    mavproxy.send('rc 3 1430\n')
    mavproxy.send('rc 4 1610\n')
    if not wait_heading(mav, 0):
        return False
    mavproxy.send('rc 4 1500\n')

    print("Going north %u meters" % side)
    mavproxy.send('rc 2 1390\n')
    ok = wait_distance(mav, side)
    mavproxy.send('rc 2 1500\n')

    print("Going east %u meters" % side)
    mavproxy.send('rc 1 1610\n')
    ok = wait_distance(mav, side)
    mavproxy.send('rc 1 1500\n')

    print("Going south %u meters" % side)
    mavproxy.send('rc 2 1610\n')
    ok = wait_distance(mav, side)
    mavproxy.send('rc 2 1500\n')

    print("Going west %u meters" % side)
    mavproxy.send('rc 1 1390\n')
    ok = wait_distance(mav, side)
    mavproxy.send('rc 1 1500\n')
    return ok


    

def land(mavproxy, mav, timeout=60):
    '''land the quad'''
    print("STARTING LANDING")
    mavproxy.send('switch 6\n')
    mavproxy.expect('STABILIZE>')
    mavproxy.send('status\n')
    mavproxy.expect('>')

    # start by dropping throttle till we have lost 5m
    mavproxy.send('rc 3 1380\n')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    wait_altitude(mav, 0, m.alt-5)

    # now let it settle gently
    mavproxy.send('rc 3 1400\n')
    tstart = time.time()
    if wait_altitude(mav, -5, 0):
        print("LANDING OK")
        return True
    else:
        print("LANDING FAILED")
        return False


def fly_mission(mavproxy, mav, filename, timeout=120):
    '''fly a mission from a file'''
    startloc = current_location(mav)
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')
    mavproxy.send('switch 1\n') # auto mode
    mavproxy.expect('AUTO>')
    wait_distance(mav, 30, timeout=120)
    wait_location(mav, startloc, timeout=300)


def setup_rc(mavproxy):
    '''setup RC override control'''
    for chan in range(1,9):
        mavproxy.send('rc %u 1500\n' % chan)
    # zero throttle
    mavproxy.send('rc 3 1000\n')


def fly_ArduCopter():
    '''fly ArduCopter in SIL'''
    global expect_list

    util.rmfile('eeprom.bin')
    sil = util.start_SIL('ArduCopter')
    mavproxy = util.start_MAVProxy_SIL('ArduCopter')
    mavproxy.expect('Please Run Setup')

    # we need to restart it after eeprom erase
    mavproxy.close()
    sil.close()
    sil = util.start_SIL('ArduCopter')
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--fgout=127.0.0.1:5502 --fgin=127.0.0.1:5501 --out=127.0.0.1:14550 --quadcopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load %s/ArduCopter.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')

    # reboot with new parameters
    mavproxy.close()
    sil.close()
    sil = util.start_SIL('ArduCopter')
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--fgout=127.0.0.1:5502 --fgin=127.0.0.1:5501 --out=127.0.0.1:14550 --out=192.168.2.15:14550 --quadcopter --streamrate=1')
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)
    mavproxy.expect("Ready to FLY")
    mavproxy.expect('Received [0-9]+ parameters')

    util.expect_setup_callback(mavproxy, expect_callback)

    # start hil_quad.py
    util.run_cmd('pkill -f hil_quad.py', checkfail=False)
    hquad = pexpect.spawn(util.reltopdir('../HILTest/hil_quad.py') + ' --fgout=192.168.2.15:9123 --home=%s' % HOME_LOCATION,
                        logfile=sys.stdout, timeout=10)
    hquad.expect('Starting at')

    expect_list.extend([hquad, sil, mavproxy])

    # get a mavlink connection going
    mav = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)
    mav.message_hooks.append(message_hook)

    failed = False
    try:
        mav.wait_heartbeat()
        mav.recv_match(type='GPS_RAW')
        setup_rc(mavproxy)
        arm_motors(mavproxy)
        takeoff(mavproxy, mav)
        fly_square(mavproxy, mav)
        loiter(mavproxy, mav)
        land(mavproxy, mav)
        fly_mission(mavproxy, mav, os.path.join(testdir, "mission1.txt"))
        disarm_motors(mavproxy)
    except pexpect.TIMEOUT, e:
        failed = True

    mavproxy.close()
    sil.close()
    hquad.close()

    shutil.copy(logfile, util.reltopdir("../buildlogs/ArduCopter-test.mavlog"))
    util.run_cmd(util.reltopdir("../pymavlink/examples/mavtogpx.py") + " " + util.reltopdir("../buildlogs/ArduCopter-test.mavlog"))
    util.run_cmd(util.reltopdir("../bin/gpxtokml") + " " + util.reltopdir("../buildlogs/ArduCopter-test.mavlog.gpx"))

    if failed:
        print("FAILED: %s" % e)
        sys.exit(1)
        
