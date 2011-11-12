import util, pexpect, time, math

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

def message_hook(mav, msg):
    '''called as each mavlink msg is received'''
    global expect_list
    if msg.get_type() in [ 'NAV_CONTROLLER_OUTPUT', 'GPS_RAW' ]:
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
    # ensure we have a position
    mav.recv_match(type='VFR_HUD', blocking=True)
    mav.recv_match(type='GPS_RAW', blocking=True)
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

def wait_waypoint(mav, wpnum_start, wpnum_end, allow_skip=True, timeout=60):
    '''wait for waypoint ranges'''
    tstart = time.time()
    m = mav.recv_match(type='WAYPOINT_CURRENT', blocking=True)

    start_wp = m.seq
    current_wp = start_wp

    print("\n***wait for waypoint ranges***\n\n\n")
    if start_wp != wpnum_start:
        print("Expected start waypoint %u but got %u" % (wpnum_start, start_wp))
        return False

    while time.time() < tstart + timeout:
        m = mav.recv_match(type='WAYPOINT_CURRENT', blocking=True)
        print("WP %u" % m.seq)
        if m.seq == current_wp:
            continue
        if m.seq == current_wp+1 or (m.seq > current_wp+1 and allow_skip):
            print("Starting new waypoint %u" % m.seq)
            tstart = time.time()
            current_wp = m.seq
            if current_wp == wpnum_end:
                print("Reached final waypoint %u" % m.seq)
                return True
        if m.seq > current_wp+1:
            print("Skipped waypoint! Got wp %u expected %u" % (m.seq, current_wp+1))
            return False
    print("Timed out waiting for waypoint %u" % wpnum_end)
    return False

def save_wp(mavproxy, mav):
    mavproxy.send('rc 7 2000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==2000', blocking=True)
    mavproxy.send('rc 7 1000\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan7_raw==1000', blocking=True)
    mavproxy.send('wp list\n')
