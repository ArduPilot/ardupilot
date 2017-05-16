#!/usr/bin/env python
from __future__ import print_function
import math
import time

from pymavlink import mavwp, mavutil

from pysim import util

import os
from pymavlink import mavwp, mavutil

import sys
import abc

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

if sys.version_info[0] >= 3 and sys.version_info[1] >= 4:
    ABC = abc.ABC
else:
    ABC = abc.ABCMeta('ABC', (), {})


def progress(text):
    """Display sim_vehicle progress text"""
    print("AUTOTEST: " + text)


#################################################
# GENERAL UTILITIES
#################################################
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


class Autotest(ABC):
    def __init__(self):
        self.mavproxy = None
        self.mav = None

    #################################################
    # SIM UTILITIES
    #################################################
    def get_sim_time(self):
        m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True)
        return m.time_boot_ms * 1.0e-3
    
    def sim_location(self):
        """Return current simulator location."""
        from pymavlink import mavutil
        m = self.mav.recv_match(type='SIMSTATE', blocking=True)
        return mavutil.location(m.lat*1.0e-7, m.lng*1.0e-7, 0, math.degrees(m.yaw))
    
    def save_wp(self):
        self.mavproxy.send('rc 7 1000\n')
        self.mav.recv_match(condition='RC_CHANNELS.chan7_raw==1000', blocking=True)
        self.wait_seconds(1)
        self.mavproxy.send('rc 7 2000\n')
        self.mav.recv_match(condition='RC_CHANNELS.chan7_raw==2000', blocking=True)
        self.wait_seconds(1)
        self.mavproxy.send('rc 7 1000\n')
        self.mav.recv_match(condition='RC_CHANNELS.chan7_raw==1000', blocking=True)
        self.wait_seconds(1)
    
    def log_download(self, filename, timeout=360):
        """Download latest log."""
        self.mavproxy.send("log list\n")
        self.mavproxy.expect("numLogs")
        self.mav.wait_heartbeat()
        self.mav.wait_heartbeat()
        self.mavproxy.send("set shownoise 0\n")
        self.mavproxy.send("log download latest %s\n" % filename)
        self.mavproxy.expect("Finished downloading", timeout=timeout)
        self.mav.wait_heartbeat()
        self.mav.wait_heartbeat()
        return True
    
    def show_gps_and_sim_positions(self, on_off):
        if on_off is True:
            # turn on simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 1\n')
            self.mavproxy.send('map set showsimpos 1\n')
        else:
            # turn off simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 0\n')
            self.mavproxy.send('map set showsimpos 0\n')
    
    def mission_count(self, filename):
        """Load a mission from a file and return number of waypoints."""
        wploader = mavwp.MAVWPLoader()
        wploader.load(filename)
        num_wp = wploader.count()
        return num_wp
    
    def load_mission_from_file(self, filename):
        """Load a mission from a file to flight controller."""
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
    
        # update num_wp
        wploader = mavwp.MAVWPLoader()
        wploader.load(filename)
        num_wp = wploader.count()
        return num_wp
    
    def save_mission_to_file(self, filename):
        self.mavproxy.send('wp save %s\n' % filename)
        self.mavproxy.expect('Saved ([0-9]+) waypoints')
        num_wp = int(self.mavproxy.match.group(1))
        print("num_wp: %d" % num_wp)
        return num_wp
    
    def setup_rc(self):
        """Setup RC override control."""
        for chan in range(1, 9):
            self.mavproxy.send('rc %u 1500\n' % chan)
    
    def set_switch_default(self):
        """Set the flight mode switch to default value"""
        self.mavproxy.send('rc 8 1800\n')
    
    def set_throttle_zero(self):
        """Zero the throttle."""
        self.mavproxy.send('rc 3 1000\n')
    
    def arm_vehicle(self):
        self.mavproxy.send('arm throttle\n')
        self.mav.motors_armed_wait()
        print("ARMED")
        return True
    
    def disarm_vehicle(self):
        self.mavproxy.send('disarm\n')
        self.mav.motors_disarmed_wait()
        print("DISARMED")
        return True
    
    #################################################
    # NAVIGATION UTILITIES
    #################################################
    def get_distance(self, loc1, loc2):
        """Get ground distance between two locations."""
        dlat = loc2.lat - loc1.lat
        dlong = loc2.lng - loc1.lng
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
    def get_bearing(self, loc1, loc2):
        """Get bearing from loc1 to loc2."""
        off_x = loc2.lng - loc1.lng
        off_y = loc2.lat - loc1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing
    
    def reach_heading(self,  heading):
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.mavproxy.send('rc 4 1580\n')
            if not self.wait_heading(heading):
                progress("Failed to reach heading")
                return False
            self.mavproxy.send('rc 4 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan4_raw==1500', blocking=True)
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            print("NOT IMPLEMENTED")
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.mavproxy.send('rc 1 1700\n')
            self.mavproxy.send('rc 3 1550\n')
            if not self.wait_heading(heading):
                progress("Failed to reach heading")
                return False
            self.mavproxy.send('rc 3 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan3_raw==1500', blocking=True)
            self.mavproxy.send('rc 1 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan1_raw==1500', blocking=True)
        return True
    
    def reach_distance(self,  distance):
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.mavproxy.send('rc 2 1350\n')
            if not self.selfwait_distance(distance, 5, 60):
                progress("Failed to reach distance of %u" % distance)
                return False
            self.mavproxy.send('rc 2 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan2_raw==1500', blocking=True)
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            print("NOT IMPLEMENTED")
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.mavproxy.send('rc 3 1700\n')
            if not self.wait_distance(distance, accuracy=2):
                progress("Failed to reach distance of %u" % distance)
                return False
            self.mavproxy.send('rc 3 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan3_raw==1500', blocking=True)
        return True
    
    #################################################
    # WAIT UTILITIES
    #################################################
    def wait_seconds(self, seconds_to_wait):
        tstart = self.get_sim_time()
        tnow = tstart
        while tstart + seconds_to_wait > tnow:
            tnow = self.get_sim_time()
    
    def wait_altitude(self, alt_min, alt_max, timeout=30):
        """Wait for a given altitude range."""
        climb_rate = 0
        previous_alt = 0
    
        tstart = self.get_sim_time()
        print("Waiting for altitude between %u and %u" % (alt_min, alt_max))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            climb_rate = m.alt - previous_alt
            previous_alt = m.alt
            print("Wait Altitude: Cur:%u, min_alt:%u, climb_rate: %u" % (m.alt, alt_min, climb_rate))
            if m.alt >= alt_min and m.alt <= alt_max:
                print("Altitude OK")
                return True
        print("Failed to attain altitude range")
        return False
    
    def wait_groundspeed(self, gs_min, gs_max, timeout=30):
        """Wait for a given ground speed range."""
        tstart = self.get_sim_time()
        print("Waiting for groundspeed between %.1f and %.1f" % (gs_min, gs_max))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            print("Wait groundspeed %.1f, target:%.1f" % (m.groundspeed, gs_min))
            if m.groundspeed >= gs_min and m.groundspeed <= gs_max:
                return True
        print("Failed to attain groundspeed range")
        return False
    
    def wait_ready_to_arm(self):
        """Wait for EKF and GPS checks to pass."""
        self.mavproxy.expect('IMU0 is using GPS')
    
    def wait_roll(self, roll, accuracy, timeout=30):
        """Wait for a given roll in degrees."""
        tstart = self.get_sim_time()
        print("Waiting for roll of %d at %s" % (roll, time.ctime()))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='ATTITUDE', blocking=True)
            p = math.degrees(m.pitch)
            r = math.degrees(m.roll)
            print("Roll %d Pitch %d" % (r, p))
            if math.fabs(r - roll) <= accuracy:
                print("Attained roll %d" % roll)
                return True
        print("Failed to attain roll %d" % roll)
        return False
    
    def wait_pitch(self, pitch, accuracy, timeout=30):
        """Wait for a given pitch in degrees."""
        tstart = self.get_sim_time()
        print("Waiting for pitch of %u at %s" % (pitch, time.ctime()))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='ATTITUDE', blocking=True)
            p = math.degrees(m.pitch)
            r = math.degrees(m.roll)
            print("Pitch %d Roll %d" % (p, r))
            if math.fabs(p - pitch) <= accuracy:
                print("Attained pitch %d" % pitch)
                return True
        print("Failed to attain pitch %d" % pitch)
        return False
    
    def wait_heading(self, heading, accuracy=5, timeout=30):
        """Wait for a given heading."""
        tstart = self.get_sim_time()
        print("Waiting for heading %u with accuracy %u" % (heading, accuracy))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            print("Heading %u" % m.heading)
            if math.fabs(m.heading - heading) <= accuracy:
                print("Attained heading %u" % heading)
                return True
        print("Failed to attain heading %u" % heading)
        return False
    
    def wait_distance(self, distance, accuracy=5, timeout=30):
        """Wait for flight of a given distance."""
        tstart = self.get_sim_time()
        start = self.mav.location()
        while self.get_sim_time() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            print("Distance %.2f meters" % delta)
            if math.fabs(delta - distance) <= accuracy:
                print("Attained distance %.2f meters OK" % delta)
                return True
            if delta > (distance + accuracy):
                print("Failed distance - overshoot delta=%f distance=%f" % (delta, distance))
                return False
        print("Failed to attain distance %u" % distance)
        return False
    
    def wait_location(self, loc, accuracy=5, timeout=30, target_altitude=None, height_accuracy=-1):
        """Wait for arrival at a location."""
        tstart = self.get_sim_time()
        if target_altitude is None:
            target_altitude = loc.alt
        print("Waiting for location %.4f,%.4f at altitude %.1f height_accuracy=%.1f" % (
            loc.lat, loc.lng, target_altitude, height_accuracy))
        while self.get_sim_time() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(loc, pos)
            print("Distance %.2f meters alt %.1f" % (delta, pos.alt))
            if delta <= accuracy:
                if height_accuracy != -1 and math.fabs(pos.alt - target_altitude) > height_accuracy:
                    continue
                print("Reached location (%.2f meters)" % delta)
                return True
        print("Failed to attain location")
        return False
    
    def wait_waypoint(self, wpnum_start, wpnum_end, allow_skip=True, max_dist=2, timeout=400):
        """Wait for waypoint ranges."""
        tstart = self.get_sim_time()
        # this message arrives after we set the current WP
        start_wp = self.mav.waypoint_current()
        current_wp = start_wp
        mode = self.mav.flightmode
    
        print("\ntest: wait for waypoint ranges start=%u end=%u\n\n" % (wpnum_start, wpnum_end))
        # if start_wp != wpnum_start:
        #    print("test: Expected start waypoint %u but got %u" % (wpnum_start, start_wp))
        #    return False
    
        while self.get_sim_time() < tstart + timeout:
            seq = self.mav.waypoint_current()
            m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = m.wp_dist
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
    
            # if we changed mode, fail
            if self.mav.flightmode != mode:
                print('Exited %s mode' % mode)
                return False
    
            print("test: WP %u (wp_dist=%u Alt=%d), current_wp: %u, wpnum_end: %u" % (seq, wp_dist, m.alt, current_wp, wpnum_end))
            if seq == current_wp+1 or (seq > current_wp+1 and allow_skip):
                print("test: Starting new waypoint %u" % seq)
                tstart = self.get_sim_time()
                current_wp = seq
                # the wp_dist check is a hack until we can sort out the right seqnum
                # for end of mission
            # if current_wp == wpnum_end or (current_wp == wpnum_end-1 and wp_dist < 2):
            if current_wp == wpnum_end and wp_dist < max_dist:
                print("Reached final waypoint %u" % seq)
                return True
            if seq >= 255:
                print("Reached final waypoint %u" % seq)
                return True
            if seq > current_wp+1:
                print("Failed: Skipped waypoint! Got wp %u expected %u" % (seq, current_wp+1))
                return False
        print("Failed: Timed out waiting for waypoint %u of %u" % (wpnum_end, wpnum_end))
        return False
    
    def wait_mode(self, mode, timeout=None):
        print("Waiting for mode %s" % mode)
        self.mav.recv_match(condition='self.mav.flightmode.upper()=="%s".upper()' % mode, timeout=timeout, blocking=True)
        print("Got mode %s" % mode)
        return self.mav.flightmode

    @abc.abstractmethod
    def init(self):
        pass
#################################################
# TESTS COMMONS
#################################################
    @abc.abstractmethod
    def test_common_feature(self):
        failed = False
        # TEST ARMING/DISARM
        if not self.arm_vehicle():
            progress("Failed to ARM")
            failed = True
        if not self.disarm_vehicle():
            progress("Failed to DISARM")
            failed = True
        if not self.test_arm_motors_radio():
            progress("Failed to ARM with radio")
            failed = True
        if not self.test_disarm_motors_radio():
            progress("Failed to ARM with radio")
            failed = True
        if not self.test_autodisarm_motors():
            progress("Failed to AUTO DISARM")
            failed = True
        # TODO: Test failure on arm (with arming check)
        # TEST MISSION FILE
        progress("TEST LOADING MISSION")
        num_wp = self.load_mission_from_file(os.path.join(testdir, "all_msg_mission.txt"))
        if num_wp == 0:
            progress("load all_msg_mission failed")
            failed = True

        progress("TEST SAVING MISSION")
        num_wp_old = num_wp
        num_wp = self.save_mission_to_file(os.path.join(testdir, "all_msg_mission2.txt"))
        if num_wp != num_wp_old:
            progress("save all_msg_mission failed")
            failed = True

        progress("TEST CLEARING MISSION")
        self.mavproxy.send("wp clear\n")
        num_wp = mavwp.MAVWPLoader().count()
        if num_wp != 0:
            progress("clear mission failed")
            failed = True

        return failed

    # TESTS FAILSAFE
    @abc.abstractmethod
    def test_throttle_failsafe(self,   home, side=60, timeout=180):
        """Fly east, Failsafe, return, land."""

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            mavutil.mavlink.MAV_TYPE_HELICOPTER,
                            mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                            mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                            mavutil.mavlink.MAV_TYPE_COAXIAL,
                            mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            # switch to loiter mode temporarily to stop us from rising
            self.mavproxy.send('switch 5\n')
            self.wait_mode('LOITER')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.mavproxy.send('switch 6\n')  # manual mode
            self.wait_mode('MANUAL')
            self.mavproxy.send("param set FS_ACTION 1\n")

        # first aim east
        progress("turn east")
        if not self.reach_heading(135):
            return False

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            mavutil.mavlink.MAV_TYPE_HELICOPTER,
                            mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                            mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                            mavutil.mavlink.MAV_TYPE_COAXIAL,
                            mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            # raise throttle slightly to avoid hitting the ground
            self.mavproxy.send('rc 3 1600\n')
            # switch to stabilize mode
            self.mavproxy.send('switch 6\n')
            self.wait_mode( 'STABILIZE')
            self.mavproxy.send('rc 3 1500\n')

        # fly east 60 meters
        progress("# Going forward %u meters" % side)
        if not self.reach_distance(  side):
            return False

        # pull throttle low
        progress("# Enter Failsafe")
        self.mavproxy.send('rc 3 900\n')

        tstart = self.get_sim_time()
        success = False
        while self.get_sim_time() < tstart + timeout and not success:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(home, pos)
            progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
            # check if we've reached home
            if (m.alt - home.alt) <= 1 and home_distance < 10:
                success = True

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            # reduce throttle
            self.mavproxy.send('rc 3 1100\n')
            # switch back to stabilize mode
            self.mavproxy.send('switch 2\n')  # land mode
            self.wait_mode('LAND')
            self.mavproxy.send('switch 6\n')  # stabilize mode
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.wait_mode('HOLD')
            # reduce throttle
            self.mavproxy.send('rc 3 1500\n')
            self.mavproxy.expect('APM: Failsafe ended')
            self.mavproxy.send('switch 2\n')  # manual mode
            self.mavproxy.send('switch 6\n')
            self.wait_mode('MANUAL')

        if success:
            progress("Reached failsafe home OK")
            return True
        else:
            progress("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
            return False


##################################################################
#   IMPLEMENTATIONS
##################################################################
    # TEST ARM RADIO
    @abc.abstractmethod
    def test_arm_motors_radio(self):
        """Test Arming motors with radio."""
        progress("Test arming motors with radio")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.wait_mode('STABILIZE')
            self.mavproxy.send('rc 3 1000\n')  # throttle at zero
            self.mavproxy.send('rc 4 2000\n')  # yaw full right
            self.mavproxy.expect('APM: Arming motors')
            self.mavproxy.send('rc 4 1500\n')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.wait_mode('MANUAL')
            self.mavproxy.send('rc 3 1500\n')  # throttle at zero
            self.mavproxy.send('rc 1 2000\n')  # steer full right
            self.mavproxy.expect('APM: Throttle armed')
            self.mavproxy.send('rc 1 1500\n')

        self.mav.motors_armed_wait()
        progress("MOTORS ARMED OK")
        return True

    # TEST DISARM RADIO
    @abc.abstractmethod
    def test_disarm_motors_radio(self):
        """Test Disarm motors with radio."""
        progress("Test disarming motors with radio")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.wait_mode('STABILIZE')
            self.mavproxy.send('rc 3 1000\n')  # throttle at zero
            self.mavproxy.send('rc 4 1000\n')  # yaw full right
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.wait_mode('MANUAL')
            self.mavproxy.send('rc 3 1500\n')  # throttle at zero
            self.mavproxy.send('rc 1 1000\n')  # steer full right

        tstart = self.get_sim_time()
        timeout = 15
        while self.get_sim_time() < tstart + timeout:
            if not self.mav.motors_armed():
                progress("MOTORS DISARMED OK WITH RADIO")
                self.mavproxy.send('rc 1 1500\n')  # steer full right
                self.mavproxy.send('rc 4 1500\n')  # yaw full right
                return True
        progress("FAILED TO DISARM WITH RADIO")
        return False

    # TEST AUTO DISARM
    @abc.abstractmethod
    def test_autodisarm_motors(self):
        """Test Autodisarm motors."""
        progress("Test Autodisarming motors")
        self.mavproxy.send('switch 6\n')  # stabilize/manual mode
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.wait_mode('STABILIZE')
            self.mavproxy.send('rc 3 1000\n')  # throttle at zero
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.wait_mode('STABILIZE')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            #  NOT IMPLEMENTED ON ROVER
            progress("MOTORS AUTODISARMED OK")
            return True
        self.arm_vehicle()

        tstart = self.get_sim_time()
        timeout = 15     #  TODO: adapt timeout with data from param
        while self.get_sim_time() < tstart + timeout:
            if not self.mav.motors_armed():
                progress("MOTORS AUTODISARMED OK ")
                self.mavproxy.send('rc 1 1500\n')  # steer full right
                self.mavproxy.send('rc 4 1500\n')  # yaw full right
                return True
        progress("FAILED TO AUTODISARMED")
        return False

    # TEST RC OVERRIDE
    # TEST RC OVERRIDE TIMEOUT
    @abc.abstractmethod
    def test_RTL(self, home, timeout=250):
        """Return, land."""
        progress("# Enter RTL")
        self.mavproxy.send('switch 3\n')
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            pos = self.mav.location()
            home_distance = self.get_distance(home, pos)
            progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
            if m.alt <= 1 and home_distance < 10:
                progress("RTL Complete")
                return True
        return False

    @abc.abstractmethod
    def test_mission(self, filename):
        """Test a mission from a file."""
        progress("Test mission %s" % filename)
        num_wp = self.load_mission_from_file(filename)
        self.mavproxy.send('wp set 1\n')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.wait_mode('AUTO')
        ret = self.wait_waypoint(0, num_wp-1, max_dist=5, timeout=500)

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            mavutil.mavlink.MAV_TYPE_HELICOPTER,
                            mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                            mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                            mavutil.mavlink.MAV_TYPE_COAXIAL,
                            mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            expect_msg = "Reached command #%u" % (num_wp-1)
            if ret:
                self.mavproxy.expect(expect_msg)
            self.mavproxy.send('switch 5\n')  # loiter mode
            self.wait_mode('LOITER')
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            print("NOT IMPLEMENTED")
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            expect_msg = "No commands."
            if ret:
                self.mavproxy.expect(expect_msg)
            self.wait_mode('HOLD')
        progress("test: MISSION COMPLETE: passed=%s" % ret)
        return ret

#TEST Guided :
 # goto
 # set point position
    #set point velocity
 # set attitude