from __future__ import print_function

import abc
import math
import os
import shutil
import sys
import time

from pymavlink import mavwp, mavutil
from pysim import util, vehicleinfo

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

# Check python version for abstract base class
if sys.version_info[0] >= 3 and sys.version_info[1] >= 4:
        ABC = abc.ABC
else:
    ABC = abc.ABCMeta('ABC', (), {})


class ErrorException(Exception):
    """Base class for other exceptions"""
    pass


class AutoTestTimeoutException(ErrorException):
    pass


class WaitModeTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given mode change."""
    pass


class WaitAltitudeTimout(AutoTestTimeoutException):
    """Thrown when fails to achieve given altitude range."""
    pass


class WaitGroundSpeedTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given ground speed range."""
    pass


class WaitRollTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given roll in degrees."""
    pass


class WaitPitchTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given pitch in degrees."""
    pass


class WaitHeadingTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given heading."""
    pass


class WaitDistanceTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain distance"""
    pass


class WaitLocationTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain location"""
    pass


class WaitWaypointTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain waypoint ranges"""
    pass


class SetRCTimeout(AutoTestTimeoutException):
    """Thrown when fails to send RC commands"""
    pass


class MsgRcvTimeoutException(AutoTestTimeoutException):
    """Thrown when fails to receive an expected message"""
    pass


class NotAchievedException(ErrorException):
    """Thrown when fails to achieve a goal"""
    pass


class PreconditionFailedException(ErrorException):
    """Thrown when a precondition for a test is not met"""
    pass


class AutoTest(ABC):
    """Base abstract class.
    It implements the common function for all vehicle types.
    """
    def __init__(self,
                 viewerip=None,
                 use_map=False):
        self.mavproxy = None
        self.mav = None
        self.viewerip = viewerip
        self.use_map = use_map

    @staticmethod
    def progress(text):
        """Display autotest progress text."""
        print("AUTOTEST: " + text)

    # following two functions swiped from autotest.py:
    @staticmethod
    def buildlogs_dirpath():
        return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))

    def buildlogs_path(self, path):
        '''return a string representing path in the buildlogs directory'''
        bits = [self.buildlogs_dirpath()]
        if isinstance(path, list):
            bits.extend(path)
        else:
            bits.append(path)
        return os.path.join(*bits)

    def sitl_streamrate(self):
        '''allow subclasses to override SITL streamrate'''
        return 10

    def mavproxy_options(self):
        '''returns options to be passed to MAVProxy'''
        ret = ['--sitl=127.0.0.1:5501',
               '--out=127.0.0.1:19550',
               '--streamrate=%u' % self.sitl_streamrate()]
        if self.viewerip:
            ret.append("--out=%s:14550" % self.viewerip)
        if self.use_map:
            ret.append('--map')

        return ret

    def vehicleinfo_key(self):
        return self.log_name

    def apply_defaultfile_parameters(self):
        '''apply parameter file'''

        # setup test parameters
        vinfo = vehicleinfo.VehicleInfo()
        if self.params is None:
            frames = vinfo.options[self.vehicleinfo_key()]["frames"]
            self.params = frames[self.frame]["default_params_filename"]
        if not isinstance(self.params, list):
            self.params = [self.params]
        for x in self.params:
            self.mavproxy.send("param load %s\n" % os.path.join(testdir, x))
            self.mavproxy.expect('Loaded [0-9]+ parameters')
        self.set_parameter('LOG_REPLAY', 1)
        self.set_parameter('LOG_DISARMED', 1)
        self.reboot_sitl()

    def reboot_sitl(self):
        self.mavproxy.send("reboot\n")
        self.mavproxy.expect("tilt alignment complete")
        # empty mav to avoid getting old timestamps:
        if self.mav is not None:
            while self.mav.recv_match(blocking=False):
                pass
        # after reboot stream-rates may be zero.  Prompt MAVProxy to
        # send a rate-change message by changing away from our normal
        # stream rates and back again:
        if self.mav is not None:
            tstart = self.get_sim_time()
        while True:

            self.mavproxy.send("set streamrate %u\n" % (self.sitl_streamrate()*2))
            if self.mav is None:
                break

            if self.get_sim_time() - tstart > 10:
                raise AutoTestTimeoutException()

            m = self.mav.recv_match(type='SYSTEM_TIME',
                                    blocking=True,
                                    timeout=1)
            if m is not None:
                print("Received (%s)" % str(m))
                break
        self.mavproxy.send("set streamrate %u\n" % self.sitl_streamrate())
        self.progress("Reboot complete")

    def close(self):
        '''tidy up after running all tests'''
        if self.use_map:
            self.mavproxy.send("module unload map\n")
            self.mavproxy.expect("Unloaded module map")

        self.mav.close()
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        valgrind_log = util.valgrind_log_filepath(binary=self.binary,
                                                  model=self.frame)
        if os.path.exists(valgrind_log):
            os.chmod(valgrind_log, 0o644)
            shutil.copy(valgrind_log,
                        self.buildlogs_path("%s-valgrind.log" %
                                            self.log_name))

    #################################################
    # GENERAL UTILITIES
    #################################################
    def expect_list_clear(self):
        """clear the expect list."""
        global expect_list
        for p in expect_list[:]:
            expect_list.remove(p)

    def expect_list_extend(self, list_to_add):
        """Extend the expect list."""
        global expect_list
        expect_list.extend(list_to_add)

    def idle_hook(self, mav):
        """Called when waiting for a mavlink message."""
        global expect_list
        for p in expect_list:
            util.pexpect_drain(p)

    def message_hook(self, mav, msg):
        """Called as each mavlink msg is received."""
        self.idle_hook(mav)

    def expect_callback(self, e):
        """Called when waiting for a expect pattern."""
        global expect_list
        for p in expect_list:
            if p == e:
                continue
        util.pexpect_drain(p)

    #################################################
    # SIM UTILITIES
    #################################################
    def get_sim_time(self):
        """Get SITL time."""
        m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True)
        return m.time_boot_ms * 1.0e-3

    def sim_location(self):
        """Return current simulator location."""
        m = self.mav.recv_match(type='SIMSTATE', blocking=True)
        return mavutil.location(m.lat*1.0e-7,
                                m.lng*1.0e-7,
                                0,
                                math.degrees(m.yaw))

    def save_wp(self):
        """Trigger RC 7 to save waypoint."""
        self.mavproxy.send('rc 7 1000\n')
        self.mav.recv_match(condition='RC_CHANNELS.chan7_raw==1000',
                            blocking=True)
        self.wait_seconds(1)
        self.mavproxy.send('rc 7 2000\n')
        self.mav.recv_match(condition='RC_CHANNELS.chan7_raw==2000',
                            blocking=True)
        self.wait_seconds(1)
        self.mavproxy.send('rc 7 1000\n')
        self.mav.recv_match(condition='RC_CHANNELS.chan7_raw==1000',
                            blocking=True)
        self.wait_seconds(1)

    def log_download(self, filename, timeout=360):
        """Download latest log."""
        self.disarm_vehicle()
        self.mav.wait_heartbeat()
        self.mavproxy.send("log list\n")
        self.mavproxy.expect("numLogs")
        self.mav.wait_heartbeat()
        self.mav.wait_heartbeat()
        self.mavproxy.send("set shownoise 0\n")
        self.mavproxy.send("log download latest %s\n" % filename)
        self.mavproxy.expect("Finished downloading", timeout=timeout)
        self.mav.wait_heartbeat()
        self.mav.wait_heartbeat()

    def show_gps_and_sim_positions(self, on_off):
        """Allow to display gps and actual position on map."""
        if on_off is True:
            # turn on simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 1\n')
            self.mavproxy.send('map set showsimpos 1\n')
        else:
            # turn off simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 0\n')
            self.mavproxy.send('map set showsimpos 0\n')

    @staticmethod
    def mission_count(filename):
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
        """Save a mission to a file"""
        self.mavproxy.send('wp save %s\n' % filename)
        self.mavproxy.expect('Saved ([0-9]+) waypoints')
        num_wp = int(self.mavproxy.match.group(1))
        self.progress("num_wp: %d" % num_wp)
        return num_wp

    def set_rc_default(self):
        """Setup all simulated RC control to 1500."""
        for chan in range(1, 16):
            self.mavproxy.send('rc %u 1500\n' % chan)

    def set_rc(self, chan, pwm, timeout=5):
        """Setup a simulated RC control to a PWM value"""
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            self.mavproxy.send('rc %u %u\n' % (chan, pwm))
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            chan_pwm = getattr(m, "chan" + str(chan) + "_raw")
            if chan_pwm == pwm:
                return True
        self.progress("Failed to send RC commands to channel %s" % str(chan))
        raise SetRCTimeout()

    def armed(self):
        '''Return true if vehicle is armed and safetyoff'''
        return self.mav.motors_armed()

    def arm_vehicle(self):
        """Arm vehicle with mavlink arm message."""
        self.mavproxy.send('arm throttle\n')
        self.mav.motors_armed_wait()
        self.progress("ARMED")
        return True

    def disarm_vehicle(self):
        """Disarm vehicle with mavlink disarm message."""
        self.mavproxy.send('disarm\n')
        self.mav.motors_disarmed_wait()
        self.progress("DISARMED")
        return True

    def set_parameter(self, name, value):
        for i in range(1, 10):
            self.mavproxy.send("param set %s %s\n" % (name, str(value)))
            returned_value = self.get_parameter(name)
            if returned_value == float(value):
                # yes, exactly equal.
                break
            self.progress("Param fetch returned incorrect value (%s) vs (%s)"
                          % (returned_value, value))

    def get_parameter(self, name):
        self.mavproxy.send("param fetch %s\n" % name)
        self.mavproxy.expect("%s = ([-0-9.]*)\r\n" % (name,))
        return float(self.mavproxy.match.group(1))

    #################################################
    # UTILITIES
    #################################################
    @staticmethod
    def get_distance(loc1, loc2):
        """Get ground distance between two locations."""
        dlat = loc2.lat - loc1.lat
        dlong = loc2.lng - loc1.lng
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    @staticmethod
    def get_bearing(loc1, loc2):
        """Get bearing from loc1 to loc2."""
        off_x = loc2.lng - loc1.lng
        off_y = loc2.lat - loc1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def do_get_autopilot_capabilities(self):
        self.mavproxy.send("long REQUEST_AUTOPILOT_CAPABILITIES 1\n")
        m = self.mav.recv_match(type='AUTOPILOT_VERSION',
                                blocking=True,
                                timeout=10)
        if m is None:
            self.progress("AUTOPILOT_VERSION not received")
            raise NotAchievedException()
        self.progress("AUTOPILOT_VERSION received")

    def do_set_mode_via_command_long(self):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = 4  # hold
        start = time.time()
        while time.time() - start < 5:
            self.mavproxy.send("long DO_SET_MODE %u %u\n" %
                               (base_mode, custom_mode))
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=10)
            if m is None:
                raise ErrorException()
            if m.custom_mode == custom_mode:
                return
            time.sleep(0.1)
        return AutoTestTimeoutException()

    def reach_heading_manual(self, heading):
        """Manually direct the vehicle to the target heading."""
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.mavproxy.send('rc 4 1580\n')
            self.wait_heading(heading)
            self.mavproxy.send('rc 4 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan4_raw==1500',
                                blocking=True)
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.progress("NOT IMPLEMENTED")
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.mavproxy.send('rc 1 1700\n')
            self.mavproxy.send('rc 3 1550\n')
            self.wait_heading(heading)
            self.mavproxy.send('rc 3 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan3_raw==1500',
                                blocking=True)
            self.mavproxy.send('rc 1 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan1_raw==1500',
                                blocking=True)

    def reach_distance_manual(self,  distance):
        """Manually direct the vehicle to the target distance from home."""
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.mavproxy.send('rc 2 1350\n')
            self.wait_distance(distance, accuracy=5, timeout=60)
            self.mavproxy.send('rc 2 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan2_raw==1500',
                                blocking=True)
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.progress("NOT IMPLEMENTED")
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.mavproxy.send('rc 3 1700\n')
            self.wait_distance(distance, accuracy=2)
            self.mavproxy.send('rc 3 1500\n')
            self.mav.recv_match(condition='RC_CHANNELS.chan3_raw==1500',
                                blocking=True)

    #################################################
    # WAIT UTILITIES
    #################################################
    def wait_seconds(self, seconds_to_wait):
        """Wait some second in SITL time."""
        tstart = self.get_sim_time()
        tnow = tstart
        while tstart + seconds_to_wait > tnow:
            tnow = self.get_sim_time()

    def wait_altitude(self, alt_min, alt_max, timeout=30, relative=False):
        """Wait for a given altitude range."""
        climb_rate = 0
        previous_alt = 0

        tstart = self.get_sim_time()
        self.progress("Waiting for altitude between %u and %u" %
                      (alt_min, alt_max))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if m is None:
                continue
            if relative:
                alt = m.relative_alt/1000.0 # mm -> m
            else:
                alt = m.alt/1000.0 # mm -> m

            climb_rate = alt - previous_alt
            previous_alt = alt
            self.progress("Wait Altitude: Cur:%u, min_alt:%u, climb_rate: %u"
                          % (alt, alt_min, climb_rate))
            if alt >= alt_min and alt <= alt_max:
                self.progress("Altitude OK")
                return True
        self.progress("Failed to attain altitude range")
        raise WaitAltitudeTimout()

    def wait_groundspeed(self, gs_min, gs_max, timeout=30):
        """Wait for a given ground speed range."""
        tstart = self.get_sim_time()
        self.progress("Waiting for groundspeed between %.1f and %.1f" %
                      (gs_min, gs_max))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("Wait groundspeed %.1f, target:%.1f" %
                          (m.groundspeed, gs_min))
            if m.groundspeed >= gs_min and m.groundspeed <= gs_max:
                return True
        self.progress("Failed to attain groundspeed range")
        raise WaitGroundSpeedTimeout()

    def wait_roll(self, roll, accuracy, timeout=30):
        """Wait for a given roll in degrees."""
        tstart = self.get_sim_time()
        self.progress("Waiting for roll of %d at %s" % (roll, time.ctime()))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='ATTITUDE', blocking=True)
            p = math.degrees(m.pitch)
            r = math.degrees(m.roll)
            self.progress("Roll %d Pitch %d" % (r, p))
            if math.fabs(r - roll) <= accuracy:
                self.progress("Attained roll %d" % roll)
                return True
        self.progress("Failed to attain roll %d" % roll)
        raise WaitRollTimeout()

    def wait_pitch(self, pitch, accuracy, timeout=30):
        """Wait for a given pitch in degrees."""
        tstart = self.get_sim_time()
        self.progress("Waiting for pitch of %u at %s" % (pitch, time.ctime()))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='ATTITUDE', blocking=True)
            p = math.degrees(m.pitch)
            r = math.degrees(m.roll)
            self.progress("Pitch %d Roll %d" % (p, r))
            if math.fabs(p - pitch) <= accuracy:
                self.progress("Attained pitch %d" % pitch)
                return True
        self.progress("Failed to attain pitch %d" % pitch)
        raise WaitPitchTimeout()

    def wait_heading(self, heading, accuracy=5, timeout=30):
        """Wait for a given heading."""
        tstart = self.get_sim_time()
        self.progress("Waiting for heading %u with accuracy %u" %
                      (heading, accuracy))
        while self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("Heading %u" % m.heading)
            if math.fabs(m.heading - heading) <= accuracy:
                self.progress("Attained heading %u" % heading)
                return True
        self.progress("Failed to attain heading %u" % heading)
        raise WaitHeadingTimeout()

    def wait_distance(self, distance, accuracy=5, timeout=30):
        """Wait for flight of a given distance."""
        tstart = self.get_sim_time()
        start = self.mav.location()
        while self.get_sim_time() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            self.progress("Distance %.2f meters" % delta)
            if math.fabs(delta - distance) <= accuracy:
                self.progress("Attained distance %.2f meters OK" % delta)
                return True
            if delta > (distance + accuracy):
                self.progress("Failed distance - overshoot delta=%f dist=%f"
                              % (delta, distance))
                raise WaitDistanceTimeout()
        self.progress("Failed to attain distance %u" % distance)
        raise WaitDistanceTimeout()

    def wait_location(self,
                      loc,
                      accuracy=5,
                      timeout=30,
                      target_altitude=None,
                      height_accuracy=-1):
        """Wait for arrival at a location."""
        tstart = self.get_sim_time()
        if target_altitude is None:
            target_altitude = loc.alt
        self.progress("Waiting for location"
                      "%.4f,%.4f at altitude %.1f height_accuracy=%.1f" %
                      (loc.lat, loc.lng, target_altitude, height_accuracy))
        while self.get_sim_time() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(loc, pos)
            self.progress("Distance %.2f meters alt %.1f" % (delta, pos.alt))
            if delta <= accuracy:
                height_delta = math.fabs(pos.alt - target_altitude)
                if (height_accuracy != -1 and height_delta > height_accuracy):
                    continue
                self.progress("Reached location (%.2f meters)" % delta)
                return True
        self.progress("Failed to attain location")
        raise WaitLocationTimeout()

    def wait_waypoint(self,
                      wpnum_start,
                      wpnum_end,
                      allow_skip=True,
                      max_dist=2,
                      timeout=400):
        """Wait for waypoint ranges."""
        tstart = self.get_sim_time()
        # this message arrives after we set the current WP
        start_wp = self.mav.waypoint_current()
        current_wp = start_wp
        mode = self.mav.flightmode

        self.progress("\ntest: wait for waypoint ranges start=%u end=%u\n\n"
                      % (wpnum_start, wpnum_end))
        # if start_wp != wpnum_start:
        #    self.progress("test: Expected start waypoint %u but got %u" %
        #                  (wpnum_start, start_wp))
        #    raise WaitWaypointTimeout()

        while self.get_sim_time() < tstart + timeout:
            seq = self.mav.waypoint_current()
            m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
                                    blocking=True)
            wp_dist = m.wp_dist
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)

            # if we changed mode, fail
            if self.mav.flightmode != mode:
                self.progress('Exited %s mode' % mode)
                raise WaitWaypointTimeout()

            self.progress("test: WP %u (wp_dist=%u Alt=%d), current_wp: %u,"
                          "wpnum_end: %u" %
                          (seq, wp_dist, m.alt, current_wp, wpnum_end))
            if seq == current_wp+1 or (seq > current_wp+1 and allow_skip):
                self.progress("test: Starting new waypoint %u" % seq)
                tstart = self.get_sim_time()
                current_wp = seq
                # the wp_dist check is a hack until we can sort out
                # the right seqnum for end of mission
            # if current_wp == wpnum_end or (current_wp == wpnum_end-1 and
            #                                wp_dist < 2):
            if current_wp == wpnum_end and wp_dist < max_dist:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq >= 255:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq > current_wp+1:
                self.progress("Failed: Skipped waypoint! Got wp %u expected %u"
                              % (seq, current_wp+1))
                raise WaitWaypointTimeout()
        self.progress("Failed: Timed out waiting for waypoint %u of %u" %
                      (wpnum_end, wpnum_end))
        raise WaitWaypointTimeout()

    def wait_mode(self, mode, timeout=None):
        """Wait for mode to change."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None or mode not in mode_map:
            self.progress("Unknown mode '%s'" % mode)
            self.progress("Available modes '%s'" % mode_map.keys())
            raise ErrorException()
        self.progress("Waiting for mode %s" % mode)
        tstart = self.get_sim_time()
        hastimeout = False
        while self.mav.flightmode != mode and not hastimeout:
            if timeout is not None:
                hastimeout = self.get_sim_time() > tstart + timeout
            self.mav.wait_heartbeat()
        self.progress("Got mode %s" % mode)
        if self.mav.flightmode != mode and hastimeout:
            raise WaitModeTimeout()
        return True

    def wait_ready_to_arm(self, timeout=None):
        # wait for EKF checks to pass
        return self.wait_ekf_happy(timeout=timeout)

    def wait_ekf_happy(self, timeout=30):
        """Wait for EKF to be happy"""

        tstart = self.get_sim_time()
        required_value = 831
        self.progress("Waiting for EKF value %u" % required_value)
        while timeout is None or self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True)
            current = m.flags
            if (tstart - self.get_sim_time()) % 5 == 0:
                self.progress("Wait EKF.flags: required:%u current:%u" %
                              (required_value, current))
            if current == required_value:
                self.progress("EKF Flags OK")
                return True
        self.progress("Failed to get EKF.flags=%u" % required_value)
        raise AutoTestTimeoutException()

    def run_test(self, desc, function):
        self.progress("#")
        self.progress("########## %s ##########" % (desc))
        self.progress("#")

        try:
            function()
        except Exception as e:
            self.progress('FAILED: "%s": %s' % (desc, repr(e)))
            self.fail_list.append((desc, e))
            return
        self.progress('PASSED: "%s"' % desc)

    @abc.abstractmethod
    def init(self):
        """Initilialize autotest feature."""
        pass

    # def test_common_feature(self):
    #     """Common feature to test."""
    #     sucess = True
    #     # TEST ARMING/DISARM
    #     if not self.arm_vehicle():
    #         self.progress("Failed to ARM")
    #         sucess = False
    #     if not self.disarm_vehicle():
    #         self.progress("Failed to DISARM")
    #         sucess = False
    #     if not self.test_arm_motors_radio():
    #         self.progress("Failed to ARM with radio")
    #         sucess = False
    #     if not self.test_disarm_motors_radio():
    #         self.progress("Failed to ARM with radio")
    #         sucess = False
    #     if not self.test_autodisarm_motors():
    #         self.progress("Failed to AUTO DISARM")
    #         sucess = False
    #     # TODO: Test failure on arm (with arming check)
    #     # TEST MISSION FILE
    #     # TODO : rework that to work on autotest server
    #     # self.progress("TEST LOADING MISSION")
    #     # num_wp = self.load_mission_from_file(
    #                  os.path.join(testdir, "fake_mission.txt"))
    #     # if num_wp == 0:
    #     #     self.progress("Failed to load all_msg_mission")
    #     #     sucess = False
    #     #
    #     # self.progress("TEST SAVING MISSION")
    #     # num_wp_old = num_wp
    #     # num_wp = self.save_mission_to_file(os.path.join(testdir,
    #                                          "fake_mission2.txt"))
    #     # if num_wp != num_wp_old:
    #     #     self.progress("Failed to save all_msg_mission")
    #     #     sucess = False
    #
    #     self.progress("TEST CLEARING MISSION")
    #     self.mavproxy.send("wp clear\n")
    #     self.mavproxy.send('wp list\n')
    #     self.mavproxy.expect('Requesting [0-9]+ waypoints')
    #     num_wp = mavwp.MAVWPLoader().count()
    #     if num_wp != 0:
    #         self.progress("Failed to clear mission ")
    #         sucess = False
    #
    #     return sucess
    #
    # # TESTS FAILSAFE
    # @abc.abstractmethod
    # def test_throttle_failsafe(self, home, distance_min=10, side=60,
    #                            timeout=180):
    #     """Test that RTL success in case of thottle failsafe."""
    #     pass
    #
    # # TEST ARM RADIO
    # @abc.abstractmethod
    # def test_arm_motors_radio(self):
    #     """Test arming with RC sticks."""
    #     pass
    #
    # # TEST DISARM RADIO
    # @abc.abstractmethod
    # def test_disarm_motors_radio(self):
    #     """Test disarming with RC sticks."""
    #     pass
    #
    # # TEST AUTO DISARM
    # @abc.abstractmethod
    # def test_autodisarm_motors(self):
    #     """Test auto disarming."""
    #     pass
    #
    # # TEST RC OVERRIDE
    # # TEST RC OVERRIDE TIMEOUT
    # @abc.abstractmethod
    # def test_rtl(self, home, distance_min=10, timeout=250):
    #     """Test that RTL success."""
    #     self.progress("# Enter RTL")
    #     self.mavproxy.send('switch 3\n')
    #     tstart = self.get_sim_time()
    #     while self.get_sim_time() < tstart + timeout:
    #         m = self.mav.recv_match(type='VFR_HUD', blocking=True)
    #         pos = self.mav.location()
    #         home_distance = self.get_distance(home, pos)
    #         self.progress("Alt: %u  HomeDistance: %.0f" %
    #                        (m.alt, home_distance))
    #         if m.alt <= 1 and home_distance < distance_min:
    #             self.progress("RTL Complete")
    #             return True
    #     return False
    #
    # @abc.abstractmethod
    # def test_mission(self, filename):
    #     pass

    @abc.abstractmethod
    def autotest(self):
        """Autotest used by ArduPilot autotest CI."""
        pass
