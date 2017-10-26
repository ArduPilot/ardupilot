from __future__ import print_function
import math
import time

from pymavlink import mavwp

from pysim import util

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

class AutoTestTimeoutException(Exception):
    pass

def wait_ready_to_arm(mav, timeout=None):
    # wait for EKF checks to pass
    return wait_ekf_happy(mav, timeout=timeout)

def wait_ekf_happy(mav, timeout=30):
    """Wait for EKF to be happy"""

    tstart = get_sim_time(mav)
    required_value = 831
    print("Waiting for EKF value %u" % (required_value))
    while timeout is None or get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True)
        current = m.flags
        if (tstart - get_sim_time(mav)) % 5 == 0:
            print("Wait EKF.flags: required:%u current:%u" % (required_value, current))
        if current == required_value:
            print("EKF Flags OK")
            return
    print("Failed to get EKF.flags=%u" % required_value)
    raise AutoTestTimeoutException()

def expect_list_clear():
    """clear the expect list."""
    global expect_list
    for p in expect_list[:]:
        expect_list.remove(p)


def expect_list_extend(list_to_add):
    """Extend the expect list."""
    global expect_list
    expect_list.extend(list_to_add)


def idle_hook(mav):
    """Called when waiting for a mavlink message."""
    global expect_list
    for p in expect_list:
        util.pexpect_drain(p)


def message_hook(mav, msg):
    """Called as each mavlink msg is received."""
    idle_hook(mav)


def expect_callback(e):
    """Called when waiting for a expect pattern."""
    global expect_list
    for p in expect_list:
        if p == e:
            continue
        util.pexpect_drain(p)


def get_distance(loc1, loc2):
    """Get ground distance between two locations."""
    dlat = loc2.lat - loc1.lat
    dlong = loc2.lng - loc1.lng
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(loc1, loc2):
    """Get bearing from loc1 to loc2."""
    off_x = loc2.lng - loc1.lng
    off_y = loc2.lat - loc1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def wait_seconds(mav, seconds_to_wait):
    tstart = get_sim_time(mav)
    tnow = tstart
    while tstart + seconds_to_wait > tnow:
        tnow = get_sim_time(mav)


def get_sim_time(mav):
    m = mav.recv_match(type='SYSTEM_TIME', blocking=True)
    return m.time_boot_ms * 1.0e-3

def set_parameter(mavproxy, name ,value):
    for i in range(1,10):
        mavproxy.send("param set %s %s\n" % (name, str(value)))
        mavproxy.send("param fetch %s\n" % (name))
        mavproxy.expect("%s = (.*)" % (name,))
        returned_value = mavproxy.match.group(1)
        if float(returned_value) == float(value):
            # yes, exactly equal.
            break
        print("Param fetch returned incorrect value (%s) vs (%s)" % (returned_value, value))

def get_parameter(mavproxy, name):
    mavproxy.send("param fetch %s\n" % (name))
    mavproxy.expect("%s = (.*)" % (name,))
    return float(mavproxy.match.group(1))

def wait_altitude(mav, alt_min, alt_max, timeout=30):
    """Wait for a given altitude range."""
    climb_rate = 0
    previous_alt = 0

    tstart = get_sim_time(mav)
    print("Waiting for altitude between %u and %u" % (alt_min, alt_max))
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        climb_rate = m.alt - previous_alt
        previous_alt = m.alt
        print("Wait Altitude: Cur:%u, min_alt:%u, climb_rate: %u" % (m.alt, alt_min, climb_rate))
        if m.alt >= alt_min and m.alt <= alt_max:
            print("Altitude OK")
            return True
    print("Failed to attain altitude range")
    return False


def wait_groundspeed(mav, gs_min, gs_max, timeout=30):
    """Wait for a given ground speed range."""
    tstart = get_sim_time(mav)
    print("Waiting for groundspeed between %.1f and %.1f" % (gs_min, gs_max))
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Wait groundspeed %.1f, target:%.1f" % (m.groundspeed, gs_min))
        if m.groundspeed >= gs_min and m.groundspeed <= gs_max:
            return True
    print("Failed to attain groundspeed range")
    return False


def wait_roll(mav, roll, accuracy, timeout=30):
    """Wait for a given roll in degrees."""
    tstart = get_sim_time(mav)
    print("Waiting for roll of %d at %s" % (roll, time.ctime()))
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='ATTITUDE', blocking=True)
        p = math.degrees(m.pitch)
        r = math.degrees(m.roll)
        print("Roll %d Pitch %d" % (r, p))
        if math.fabs(r - roll) <= accuracy:
            print("Attained roll %d" % roll)
            return True
    print("Failed to attain roll %d" % roll)
    return False


def wait_pitch(mav, pitch, accuracy, timeout=30):
    """Wait for a given pitch in degrees."""
    tstart = get_sim_time(mav)
    print("Waiting for pitch of %u at %s" % (pitch, time.ctime()))
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='ATTITUDE', blocking=True)
        p = math.degrees(m.pitch)
        r = math.degrees(m.roll)
        print("Pitch %d Roll %d" % (p, r))
        if math.fabs(p - pitch) <= accuracy:
            print("Attained pitch %d" % pitch)
            return True
    print("Failed to attain pitch %d" % pitch)
    return False


def wait_heading(mav, heading, accuracy=5, timeout=30):
    """Wait for a given heading."""
    tstart = get_sim_time(mav)
    print("Waiting for heading %u with accuracy %u" % (heading, accuracy))
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Heading %u" % m.heading)
        if math.fabs(m.heading - heading) <= accuracy:
            print("Attained heading %u" % heading)
            return True
    print("Failed to attain heading %u" % heading)
    return False


def wait_distance(mav, distance, accuracy=5, timeout=30):
    """Wait for flight of a given distance."""
    tstart = get_sim_time(mav)
    start = mav.location()
    while get_sim_time(mav) < tstart + timeout:
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
    """Wait for arrival at a location."""
    tstart = get_sim_time(mav)
    if target_altitude is None:
        target_altitude = loc.alt
    print("Waiting for location %.4f,%.4f at altitude %.1f height_accuracy=%.1f" % (
        loc.lat, loc.lng, target_altitude, height_accuracy))
    while get_sim_time(mav) < tstart + timeout:
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


def wait_waypoint(mav, wpnum_start, wpnum_end, allow_skip=True, max_dist=2, timeout=400):
    """Wait for waypoint ranges."""
    tstart = get_sim_time(mav)
    # this message arrives after we set the current WP
    start_wp = mav.waypoint_current()
    current_wp = start_wp
    mode = mav.flightmode

    print("\ntest: wait for waypoint ranges start=%u end=%u\n\n" % (wpnum_start, wpnum_end))
    # if start_wp != wpnum_start:
    #    print("test: Expected start waypoint %u but got %u" % (wpnum_start, start_wp))
    #    return False

    while get_sim_time(mav) < tstart + timeout:
        seq = mav.waypoint_current()
        m = mav.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp_dist = m.wp_dist
        m = mav.recv_match(type='VFR_HUD', blocking=True)

        # if we changed mode, fail
        if mav.flightmode != mode:
            print('Exited %s mode' % mode)
            return False

        print("test: WP %u (wp_dist=%u Alt=%d), current_wp: %u, wpnum_end: %u" % (seq, wp_dist, m.alt, current_wp, wpnum_end))
        if seq == current_wp+1 or (seq > current_wp+1 and allow_skip):
            print("test: Starting new waypoint %u" % seq)
            tstart = get_sim_time(mav)
            current_wp = seq
            # the wp_dist check is a hack until we can sort out the right seqnum
            # for end of mission
        # if current_wp == wpnum_end or (current_wp == wpnum_end-1 and wp_dist < 2):
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
    mavproxy.send('rc 7 1000\n')
    mav.recv_match(condition='RC_CHANNELS.chan7_raw==1000', blocking=True)
    wait_seconds(mav, 1)
    mavproxy.send('rc 7 2000\n')
    mav.recv_match(condition='RC_CHANNELS.chan7_raw==2000', blocking=True)
    wait_seconds(mav, 1)
    mavproxy.send('rc 7 1000\n')
    mav.recv_match(condition='RC_CHANNELS.chan7_raw==1000', blocking=True)
    wait_seconds(mav, 1)


def wait_mode(mav, mode, timeout=None):
    print("Waiting for mode %s" % mode)
    mav.recv_match(condition='MAV.flightmode.upper()=="%s".upper()' % mode, timeout=timeout, blocking=True)
    print("Got mode %s" % mode)
    return mav.flightmode


def mission_count(filename):
    """Load a mission from a file and return number of waypoints."""
    wploader = mavwp.MAVWPLoader()
    wploader.load(filename)
    num_wp = wploader.count()
    return num_wp


def sim_location(mav):
    """Return current simulator location."""
    from pymavlink import mavutil
    m = mav.recv_match(type='SIMSTATE', blocking=True)
    return mavutil.location(m.lat*1.0e-7, m.lng*1.0e-7, 0, math.degrees(m.yaw))


def log_download(mavproxy, mav, filename, timeout=360):
    """Download latest log."""
    mavproxy.send("log list\n")
    mavproxy.expect("numLogs")
    mav.wait_heartbeat()
    mav.wait_heartbeat()
    mavproxy.send("set shownoise 0\n")
    mavproxy.send("log download latest %s\n" % filename)
    mavproxy.expect("Finished downloading", timeout=timeout)
    mav.wait_heartbeat()
    mav.wait_heartbeat()
    return True
