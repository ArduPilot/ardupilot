import util, pexpect, time, math, mavwp

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
#    if msg.get_type() in [ 'NAV_CONTROLLER_OUTPUT', 'GPS_RAW' ]:
#        print(msg)
    idle_hook(mav)

def expect_callback(e):
    '''called when waiting for a expect pattern'''
    global expect_list
    for p in expect_list:
        if p == e:
            continue
        util.pexpect_drain(p)

class location(object):
    '''represent a GPS coordinate'''
    def __init__(self, lat, lng, alt=0, heading=0):
        self.lat = lat
        self.lng = lng
        self.alt = alt
        self.heading = heading

    def __str__(self):
        return "lat=%.6f,lon=%.6f,alt=%.1f" % (self.lat, self.lng, self.alt)

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
    # ensure we have a position
    mav.recv_match(type='VFR_HUD', blocking=True)
    mav.recv_match(type='GPS_RAW', blocking=True)
    return location(mav.messages['GPS_RAW'].lat,
                    mav.messages['GPS_RAW'].lon,
                    mav.messages['VFR_HUD'].alt)

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
            return True
    print("Failed to attain pitch %u" % pitch)
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
        if(delta > (distance + accuracy)):
            return False
    print("Failed to attain distance %u" % distance)
    return False


def wait_location(mav, loc, accuracy=5, timeout=30, target_altitude=None, height_accuracy=-1):
    '''wait for arrival at a location'''
    tstart = time.time()
    if target_altitude is None:
        target_altitude = loc.alt
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='GPS_RAW', blocking=True)
        pos = current_location(mav)
        delta = get_distance(loc, pos)
        print("Distance %.2f meters" % delta)
        if delta <= accuracy:
            if height_accuracy != -1 and math.fabs(pos.alt - target_altitude) > height_accuracy:
                continue
            print("Reached location (%.2f meters)" % delta)
            return True
    print("Failed to attain location")
    return False

def wait_waypoint(mav, wpnum_start, wpnum_end, allow_skip=True, max_dist=2, timeout=400):
    '''wait for waypoint ranges'''
    tstart = time.time()
    # this message arrives after we set the current WP
    m = mav.recv_match(type='WAYPOINT_CURRENT', blocking=True)

    start_wp = m.seq
    current_wp = start_wp

    print("\ntest: wait for waypoint ranges start=%u end=%u\n\n" % (wpnum_start, wpnum_end))
    # if start_wp != wpnum_start:
    #    print("test: Expected start waypoint %u but got %u" % (wpnum_start, start_wp))
    #    return False

    while time.time() < tstart + timeout:
        m = mav.recv_match(type='WAYPOINT_CURRENT', blocking=True)
        seq = m.seq
        m = mav.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp_dist = m.wp_dist
        print("test: WP %u (wp_dist=%u), current_wp: %u, wpnum_end: %u" % (seq, wp_dist, current_wp, wpnum_end))
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
        if (current_wp == 255):
            print("Reached final waypoint %u" % seq)
            return True
        if seq > current_wp+1:
            print("Skipped waypoint! Got wp %u expected %u" % (seq, current_wp+1))
            return False
    print("Timed out waiting for waypoint %u of %u" % (wpnum_end, wpnum_end))
    return False

def save_wp(mavproxy, mav):
    mavproxy.send('rc 7 2000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==2000', blocking=True)
    mavproxy.send('rc 7 1000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==1000', blocking=True)

def wait_mode(mav, mode):
    '''wait for a flight mode to be engaged'''
    mav.recv_match(condition='MAV.flightmode=="%s"' % mode, blocking=True)

def mission_count(filename):
    '''load a mission from a file and return number of waypoints'''
    wploader = mavwp.MAVWPLoader()
    wploader.load(filename)
    num_wp = wploader.count()
    return num_wp
