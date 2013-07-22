import util, pexpect, time, math
from pymavlink import mavwp

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

def expect_list_clear():
    '''clear the expect list'''
    global expect_list
    for p in expect_list[:]:
        expect_list.remove(p)

def expect_list_extend(list):
    '''extend the expect list'''
    global expect_list
    expect_list.extend(list)

def idle_hook(mav):
    '''called when waiting for a mavlink message'''
    global expect_list
    for p in expect_list:
        util.pexpect_drain(p)

def message_hook(mav, msg):
    '''called as each mavlink msg is received'''
    idle_hook(mav)

def expect_callback(e):
    '''called when waiting for a expect pattern'''
    global expect_list
    for p in expect_list:
        if p == e:
            continue
        util.pexpect_drain(p)

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

def wait_altitude(mav, alt_min, alt_max, timeout=30):
    climb_rate = 0
    previous_alt = 0
    '''wait for a given altitude range'''
    tstart = time.time()
    print("Waiting for altitude between %u and %u" % (alt_min, alt_max))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        climb_rate =  m.alt - previous_alt
        previous_alt = m.alt
        print("Wait Altitude: Cur:%u, min_alt:%u, climb_rate: %u" % (m.alt, alt_min , climb_rate))
        if abs(climb_rate) > 0:
            tstart = time.time();
        if m.alt >= alt_min and m.alt <= alt_max:
            print("Altitude OK")
            return True
    print("Failed to attain altitude range")
    return False

def wait_groundspeed(mav, gs_min, gs_max, timeout=30):
    '''wait for a given ground speed range'''
    tstart = time.time()
    print("Waiting for groundspeed between %.1f and %.1f" % (gs_min, gs_max))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Wait groundspeed %.1f, target:%.1f" % (m.groundspeed, gs_min))
        if m.groundspeed >= gs_min and m.groundspeed <= gs_max:
            return True
    print("Failed to attain groundspeed range")
    return False


def wait_roll(mav, roll, accuracy, timeout=30):
    '''wait for a given roll in degrees'''
    tstart = time.time()
    print("Waiting for roll of %u" % roll)
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='ATTITUDE', blocking=True)
        r = math.degrees(m.roll)
        print("Roll %u" % r)
        if math.fabs(r - roll) <= accuracy:
            print("Attained roll %u" % roll)
            return True
    print("Failed to attain roll %u" % roll)
    return False

def wait_pitch(mav, pitch, accuracy, timeout=30):
    '''wait for a given pitch in degrees'''
    tstart = time.time()
    print("Waiting for pitch of %u" % pitch)
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='ATTITUDE', blocking=True)
        r = math.degrees(m.pitch)
        print("Pitch %u" % r)
        if math.fabs(r - pitch) <= accuracy:
            print("Attained pitch %u" % pitch)
            return True
    print("Failed to attain pitch %u" % pitch)
    return False



def wait_heading(mav, heading, accuracy=5, timeout=30):
    '''wait for a given heading'''
    tstart = time.time()
    print("Waiting for heading %u with accuracy %u" % (heading, accuracy))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Heading %u" % m.heading)
        if math.fabs(m.heading - heading) <= accuracy:
            print("Attained heading %u" % heading)
            return True
    print("Failed to attain heading %u" % heading)
    return False


def wait_distance(mav, distance, accuracy=5, timeout=30):
    '''wait for flight of a given distance'''
    tstart = time.time()
    start = mav.location()
    while time.time() < tstart + timeout:
        pos = mav.location()
        delta = get_distance(start, pos)
        print("Distance %.2f meters" % delta)
        if math.fabs(delta - distance) <= accuracy:
            print("Attained distance %.2f meters OK" % delta)
            return True
        if delta > (distance + accuracy):
            print("Failed distance - overshoot delta=%f distance=%f" % (delta, distance))
            return False
    print("Failed to attain distance %u" % distance)
    return False


def wait_location(mav, loc, accuracy=5, timeout=30, target_altitude=None, height_accuracy=-1):
    '''wait for arrival at a location'''
    tstart = time.time()
    if target_altitude is None:
        target_altitude = loc.alt
    print("Waiting for location %.4f,%.4f at altitude %.1f height_accuracy=%.1f" % (
        loc.lat, loc.lng, target_altitude, height_accuracy))
    while time.time() < tstart + timeout:
        pos = mav.location()
        delta = get_distance(loc, pos)
        print("Distance %.2f meters alt %.1f" % (delta, pos.alt))
        if delta <= accuracy:
            if height_accuracy != -1 and math.fabs(pos.alt - target_altitude) > height_accuracy:
                continue
            print("Reached location (%.2f meters)" % delta)
            return True
    print("Failed to attain location")
    return False

def wait_waypoint(mav, wpnum_start, wpnum_end, allow_skip=True, max_dist=2, timeout=400, mode=None):
    '''wait for waypoint ranges'''
    tstart = time.time()
    # this message arrives after we set the current WP
    start_wp = mav.waypoint_current()
    current_wp = start_wp

    print("\ntest: wait for waypoint ranges start=%u end=%u\n\n" % (wpnum_start, wpnum_end))
    # if start_wp != wpnum_start:
    #    print("test: Expected start waypoint %u but got %u" % (wpnum_start, start_wp))
    #    return False

    while time.time() < tstart + timeout:
        seq = mav.waypoint_current()
        m = mav.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp_dist = m.wp_dist
        m = mav.recv_match(type='VFR_HUD', blocking=True)

        # if we exited the required mode, finish
        if mode is not None and mav.flightmode != mode:
            print('Exited %s mode' % mode)
            return True

        print("test: WP %u (wp_dist=%u Alt=%d), current_wp: %u, wpnum_end: %u" % (seq, wp_dist, m.alt, current_wp, wpnum_end))
        if seq == current_wp+1 or (seq > current_wp+1 and allow_skip):
            print("test: Starting new waypoint %u" % seq)
            tstart = time.time()
            current_wp = seq
            # the wp_dist check is a hack until we can sort out the right seqnum
            # for end of mission
        #if current_wp == wpnum_end or (current_wp == wpnum_end-1 and wp_dist < 2):
        if (current_wp == wpnum_end and wp_dist < max_dist):
            print("Reached final waypoint %u" % seq)
            return True
        if (seq >= 255):
            print("Reached final waypoint %u" % seq)
            return True
        if seq > current_wp+1:
            print("Failed: Skipped waypoint! Got wp %u expected %u" % (seq, current_wp+1))
            return False
    print("Failed: Timed out waiting for waypoint %u of %u" % (wpnum_end, wpnum_end))
    return False

def save_wp(mavproxy, mav):
    mavproxy.send('rc 7 2000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==2000', blocking=True)
    mavproxy.send('rc 7 1000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==1000', blocking=True)

def wait_mode(mav, mode):
    '''wait for a flight mode to be engaged'''
    print("Waiting for mode %s" % mode)
    mav.recv_match(condition='MAV.flightmode.upper()=="%s".upper()' % mode, blocking=True)
    print("Got mode %s" % mode)

def mission_count(filename):
    '''load a mission from a file and return number of waypoints'''
    wploader = mavwp.MAVWPLoader()
    wploader.load(filename)
    num_wp = wploader.count()
    return num_wp
