from __future__ import print_function

import abc
import math
import os
import shutil
import sys
import time
import pexpect
import fnmatch

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


class Context(object):
    def __init__(self):
        self.parameters = []


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
        self.contexts = []
        self.context_push()
        self.buildlog = None
        self.copy_tlog = False
        self.logfile = None

    @staticmethod
    def progress(text):
        """Display autotest progress text."""
        print("AUTOTEST: " + text)

    # following two functions swiped from autotest.py:
    @staticmethod
    def buildlogs_dirpath():
        return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))

    def buildlogs_path(self, path):
        """Return a string representing path in the buildlogs directory."""
        bits = [self.buildlogs_dirpath()]
        if isinstance(path, list):
            bits.extend(path)
        else:
            bits.append(path)
        return os.path.join(*bits)

    def sitl_streamrate(self):
        """Allow subclasses to override SITL streamrate."""
        return 10

    def autotest_connection_hostport(self):
        '''returns host and port of connection between MAVProxy and autotest,
        colon-separated'''
        return "127.0.0.1:19550"

    def autotest_connection_string_from_mavproxy(self):
        return "tcpin:" + self.autotest_connection_hostport()

    def autotest_connection_string_to_mavproxy(self):
        return "tcp:" + self.autotest_connection_hostport()

    def mavproxy_options(self):
        """Returns options to be passed to MAVProxy."""
        ret = ['--sitl=127.0.0.1:5501',
               '--out=' + self.autotest_connection_string_from_mavproxy(),
               '--streamrate=%u' % self.sitl_streamrate()]
        if self.viewerip:
            ret.append("--out=%s:14550" % self.viewerip)
        if self.use_map:
            ret.append('--map')

        return ret

    def vehicleinfo_key(self):
        return self.log_name

    def apply_defaultfile_parameters(self):
        """Apply parameter file."""
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
        self.fetch_parameters()

    def fetch_parameters(self):
        self.mavproxy.send("param fetch\n")
        self.mavproxy.expect("Received [0-9]+ parameters")

    def reboot_sitl(self):
        """Reboot SITL instance and wait it to reconnect."""
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
        """Tidy up after running all tests."""
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

    def start_test(self, description):
        self.progress("#")
        self.progress("########## %s  ##########" % description)
        self.progress("#")

    def try_symlink_tlog(self):
        self.buildlog = self.buildlogs_path(self.log_name + "-test.tlog")
        self.progress("buildlog=%s" % self.buildlog)
        if os.path.exists(self.buildlog):
            os.unlink(self.buildlog)
        try:
            os.link(self.logfile, self.buildlog)
        except OSError as error:
            self.progress("OSError [%d]: %s" % (error.errno, error.strerror))
            self.progress("WARN: Failed to create symlink: %s => %s, "
                          "will copy tlog manually to target location" %
                          (self.logfile, self.buildlog))
            self.copy_tlog = True

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

    def get_sim_time_cached(self):
        """Get SITL time."""
        x = self.mav.messages.get("SYSTEM_TIME", None)
        if x is None:
            return self.get_sim_time()
        return x.time_boot_ms * 1.0e-3

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

    def clear_wp(self):
        """Trigger RC 8 to clear waypoint."""
        self.progress("Clearing waypoints")
        self.set_rc(8, 1000)
        self.wait_seconds(0.5)
        self.set_rc(8, 2000)
        self.wait_seconds(0.5)
        self.set_rc(8, 1000)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting 0 waypoints')

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
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('wp save %s\n' % filename)
        self.mavproxy.expect('Saved ([0-9]+) waypoints')
        num_wp = int(self.mavproxy.match.group(1))
        self.progress("num_wp: %d" % num_wp)
        return num_wp

    def set_rc_default(self):
        """Setup all simulated RC control to 1500."""
        for chan in range(1, 16):
            self.mavproxy.send('rc %u 1500\n' % chan)

    def set_rc(self, chan, pwm, timeout=20):
        """Setup a simulated RC control to a PWM value"""
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.mavproxy.send('rc %u %u\n' % (chan, pwm))
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            chan_pwm = getattr(m, "chan" + str(chan) + "_raw")
            if chan_pwm == pwm:
                return True
        self.progress("Failed to send RC commands to channel %s" % str(chan))
        raise SetRCTimeout()

    def set_throttle_zero(self):
        """Set throttle to zero."""
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.set_rc(3, 1500)
        else:
            self.set_rc(3, 1000)

    def armed(self):
        """Return true if vehicle is armed and safetyoff"""
        return self.mav.motors_armed()

    def arm_vehicle(self):
        """Arm vehicle with mavlink arm message."""
        self.progress("Arm motors with MAVLink cmd")
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     )
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            self.mav.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("Motors ARMED")
                return True
        self.progress("Unable to ARM with mavlink")
        raise AutoTestTimeoutException()

    def disarm_vehicle(self):
        """Disarm vehicle with mavlink disarm message."""
        self.progress("Disarm motors with MAVLink cmd")
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     0,  # DISARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     )
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            self.mav.wait_heartbeat()
            if not self.mav.motors_armed():
                self.progress("Motors DISARMED")
                return True
        self.progress("Unable to DISARM with mavlink")
        raise AutoTestTimeoutException()

    def mavproxy_arm_vehicle(self):
        """Arm vehicle with mavlink arm message send from MAVProxy."""
        self.progress("Arm motors with MavProxy")
        self.mavproxy.send('arm throttle\n')
        self.mav.motors_armed_wait()
        self.progress("ARMED")
        return True

    def mavproxy_disarm_vehicle(self):
        """Disarm vehicle with mavlink disarm message send from MAVProxy."""
        self.progress("Disarm motors with MavProxy")
        self.mavproxy.send('disarm\n')
        self.mav.motors_disarmed_wait()
        self.progress("DISARMED")
        return True

    def arm_motors_with_rc_input(self):
        """Arm motors with radio."""
        self.progress("Arm motors with radio")
        self.set_throttle_zero()
        self.mavproxy.send('rc 1 2000\n')
        tstart = self.get_sim_time()
        timeout = 15
        while self.get_sim_time() < tstart + timeout:
            self.mav.wait_heartbeat()
            if not self.mav.motors_armed():
                arm_delay = self.get_sim_time() - tstart
                self.progress("MOTORS ARMED OK WITH RADIO")
                self.mavproxy.send('rc 1 1500\n')
                self.progress("Arm in %ss" % arm_delay)  # TODO check arming time
                return True
        self.progress("FAILED TO ARM WITH RADIO")
        self.mavproxy.send('rc 1 1500\n')
        return False

    def disarm_motors_with_rc_input(self):
        """Disarm motors with radio."""
        self.progress("Disarm motors with radio")
        self.set_throttle_zero()
        self.mavproxy.send('rc 1 1000\n')
        tstart = self.get_sim_time()
        timeout = 15
        while self.get_sim_time() < tstart + timeout:
            self.mav.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_delay = self.get_sim_time() - tstart
                self.progress("MOTORS DISARMED OK WITH RADIO")
                self.mavproxy.send('rc 1 1500\n')
                self.progress("Disarm in %ss" % disarm_delay)  # TODO check disarming time
                return True
        self.progress("FAILED TO DISARM WITH RADIO")
        self.mavproxy.send('rc 1 1500\n')
        return False

    def autodisarm_motors(self):
        """Autodisarm motors."""
        self.progress("Autodisarming motors")
        self.set_throttle_zero()
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:  # NOT IMPLEMENTED ON ROVER
            self.progress("MOTORS AUTODISARMED OK")
            return True
        tstart = self.get_sim_time()
        timeout = 15
        while self.get_sim_time() < tstart + timeout:
            self.mav.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_delay = self.get_sim_time() - tstart
                self.progress("MOTORS AUTODISARMED")
                self.progress("Autodisarm in %ss" % disarm_delay)  # TODO check disarming time
                return True
        self.progress("FAILED TO AUTODISARM")
        return False

    @staticmethod
    def should_fetch_all_for_parameter_change(param_name):
        if fnmatch.fnmatch(param_name, "*_ENABLE") or fnmatch.fnmatch(param_name, "*_ENABLED"):
            return True
        if param_name in ["ARSPD_TYPE",
                          "ARSPD2_TYPE",
                          "BATT2_MONITOR",
                          "CAN_DRIVER",
                          "COMPASS_PMOT_EN",
                          "OSD_TYPE",
                          "RSSI_TYPE",
                          "WENC_TYPE"]:
            return True
        return False

    def set_parameter(self, name, value, add_to_context=True):
        """Set parameters from vehicle."""
        old_value = self.get_parameter(name, retry=2)
        for i in range(1, 10):
            self.mavproxy.send("param set %s %s\n" % (name, str(value)))
            returned_value = self.get_parameter(name)
            if returned_value == float(value):
                # yes, exactly equal.
                if add_to_context:
                    self.context_get().parameters.append((name, old_value))
                if self.should_fetch_all_for_parameter_change(name.upper()) and value != 0:
                    self.fetch_parameters()
                return
            self.progress("Param fetch returned incorrect value (%s) vs (%s)"
                          % (returned_value, value))
        raise ValueError()

    def get_parameter(self, name, retry=1, timeout=60):
        """Get parameters from vehicle."""
        for i in range(0, retry):
            self.mavproxy.send("param fetch %s\n" % name)
            try:
                self.mavproxy.expect("%s = ([-0-9.]*)\r\n" % (name,), timeout=timeout/retry)
                return float(self.mavproxy.match.group(1))
            except pexpect.TIMEOUT:
                if i < retry:
                    continue

    def context_get(self):
        """Get Saved parameters."""
        return self.contexts[-1]

    def context_push(self):
        """Save a copy of the parameters."""
        self.contexts.append(Context())

    def context_pop(self):
        """Set parameters to origin values in reverse order."""
        dead = self.contexts.pop()

        dead_parameters = dead.parameters
        dead_parameters.reverse()
        for p in dead_parameters:
            (name, old_value) = p
            self.set_parameter(name,
                               old_value,
                               add_to_context=False)

    def run_cmd(self,
                command,
                p1,
                p2,
                p3,
                p4,
                p5,
                p6,
                p7,
                want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        """Send a MAVLink command long."""
        self.mav.mav.command_long_send(1,
                                       1,
                                       command,
                                       1,  # confirmation
                                       p1,
                                       p2,
                                       p3,
                                       p4,
                                       p5,
                                       p6,
                                       p7)
        while True:
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
            self.progress("ACK received: %s" % str(m))
            if m.command == command:
                if m.result != want_result:
                    raise ValueError()
                break

    #################################################
    # UTILITIES
    #################################################
    @staticmethod
    def get_distance(loc1, loc2):
        """Get ground distance between two locations."""
        dlat = loc2.lat - loc1.lat
        try:
            dlong = loc2.lng - loc1.lng
        except AttributeError:
            dlong = loc2.lon - loc1.lon

        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    @staticmethod
    def get_distance_int(loc1, loc2):
        """Get ground distance between two locations in the normal "int" form
        - lat/lon multiplied by 1e7"""
        dlat = loc2.lat - loc1.lat
        try:
            dlong = loc2.lng - loc1.lng
        except AttributeError:
            dlong = loc2.lon - loc1.lon

        dlat /= 10000000.0
        dlong /= 10000000.0

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
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 10:
            # Cannot use run_cmd otherwise the respond is lost during the wait for ACK
            self.mav.mav.command_long_send(1,
                                           1,
                                           mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                                           0,  # confirmation
                                           1,  # 1: Request autopilot version
                                           0,
                                           0,
                                           0,
                                           0,
                                           0,
                                           0)
            m = self.mav.recv_match(type='AUTOPILOT_VERSION',
                                    blocking=True,
                                    timeout=10)
            if m is not None:
                self.progress("AUTOPILOT_VERSION received")
                return
        raise AutoTestTimeoutException()

    def do_set_mode_via_command_long(self):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = 4  # hold
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 5:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                         base_mode,
                         custom_mode,
                         0,
                         0,
                         0,
                         0,
                         0,
                         )
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=10)
            if m is None:
                raise ErrorException()
            if m.custom_mode == custom_mode:
                return
        raise AutoTestTimeoutException()

    def mavproxy_do_set_mode_via_command_long(self):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = 4  # hold
        start = self.get_sim_time()
        while self.get_sim_time() - start < 5:
            self.mavproxy.send("long DO_SET_MODE %u %u\n" %
                               (base_mode, custom_mode))
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=10)
            if m is None:
                raise ErrorException()
            if m.custom_mode == custom_mode:
                return
        raise AutoTestTimeoutException()

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

    def reach_distance_manual(self, distance):
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

    def guided_achieve_heading(self, heading):
        tstart = self.get_sim_time()
        self.run_cmd(mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                     heading,  # target angle
                     10,  # degrees/second
                     1,  # -1 is counter-clockwise, 1 clockwise
                     0,  # 1 for relative, 0 for absolute
                     0,  # p5
                     0,  # p6
                     0,  # p7
                     )
        while True:
            if self.get_sim_time() - tstart > 200:
                raise NotAchievedException()
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("heading=%f want=%f" % (m.heading, heading))
            if m.heading == heading:
                return

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
        last_heading = -1
        last_print_time = 0
        while True:
            now = self.get_sim_time_cached()
            if now - tstart >= timeout:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if (last_heading != m.heading or
                now - last_print_time > 1):
                self.progress("Heading %u (want %f +- %f)" % (
                        m.heading, heading, accuracy))
                last_print_time = now
            last_heading = heading
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

    def wait_servo_channel_value(self, channel, value, timeout=2):
        """wait for channel to hit value"""
        channel_field = "servo%u_raw" % channel
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException()
            m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                    blocking=True,
                                    timeout=remaining)
            m_value = getattr(m, channel_field, None)
            self.progress("SERVO_OUTPUT_RAW.%s=%u want=%u" %
                          (channel_field, m_value, value))
            if m_value is None:
                raise ValueError() #?
            if m_value == value:
                return

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
                if height_accuracy != -1 and height_delta > height_accuracy:
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

    def wait_mode(self, mode, timeout=60):
        """Wait for mode to change."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None or mode not in mode_map:
            self.progress("Unknown mode '%s'" % mode)
            self.progress("Available modes '%s'" % mode_map.keys())
            raise ErrorException()
        self.progress("Waiting for mode %s" % mode)
        tstart = self.get_sim_time()
        self.mav.wait_heartbeat()
        while self.mav.flightmode != mode:
            if (timeout is not None and
                    self.get_sim_time() > tstart + timeout):
                raise WaitModeTimeout()
            self.mav.wait_heartbeat()
        # self.progress("heartbeat mode %s Want: %s" % (
        # self.mav.flightmode, mode))
        self.progress("Got mode %s" % mode)

    def wait_ready_to_arm(self, timeout=None):
        # wait for EKF checks to pass
        self.progress("Waiting reading for arm")
        return self.wait_ekf_happy(timeout=timeout)

    def wait_ekf_happy(self, timeout=30):
        """Wait for EKF to be happy"""

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            return True

        tstart = self.get_sim_time()
        # all of these must be set for arming to happen:
        required_value = (mavutil.mavlink.EKF_ATTITUDE |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS |
                          mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_ABS)
        # none of these bits must be set for arming to happen:
        error_bits = (mavutil.mavlink.ESTIMATOR_CONST_POS_MODE |
                      mavutil.mavlink.ESTIMATOR_GPS_GLITCH |
                      mavutil.mavlink.ESTIMATOR_ACCEL_ERROR)
        self.progress("Waiting for EKF value %u" % required_value)
        while timeout is None or self.get_sim_time() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True)
            current = m.flags
            if (tstart - self.get_sim_time()) % 5 == 0:
                self.progress("Wait EKF.flags: required:%u current:%u" %
                              (required_value, current))
            errors = current & error_bits
            if errors:
                self.progress("Wait EKF.flags: errors=%u" % errors)
                continue
            if (current & required_value == required_value):
                self.progress("EKF Flags OK")
                return True
        self.progress("Failed to get EKF.flags=%u" % required_value)
        raise AutoTestTimeoutException()

    def wait_text(self, text, timeout=20, the_function=None):
        """Wait a specific STATUS_TEXT."""
        self.progress("Waiting for text : %s" % text.lower())
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            if the_function is not None:
                the_function()
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True)
            if text.lower() in m.text.lower():
                self.progress("Received expected text : %s" % m.text.lower())
                return True
        self.progress("Failed to received text : %s" % text.lower())
        raise AutoTestTimeoutException()

    def get_mavlink_connection_going(self):
        # get a mavlink connection going
        connection_string = self.autotest_connection_string_to_mavproxy()
        try:
            self.mav = mavutil.mavlink_connection(connection_string,
                                                  robust_parsing=True,
                                                  source_component=250)
        except Exception as msg:
            self.progress("Failed to start mavlink connection on %s: %s" %
                          (connection_string, msg,))
            raise
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)

    def run_test(self, desc, test_function, interact=False):
        self.start_test(desc)

        try:
            test_function()
        except Exception as e:
            self.progress('FAILED: "%s": %s' % (desc, repr(e)))
            self.fail_list.append((desc, e))
            if interact:
                self.progress("Starting MAVProxy interaction as directed")
                self.mavproxy.interact()
            return
        self.progress('PASSED: "%s"' % desc)

    def check_test_syntax(self, test_file):
        """Check mistake on autotest function syntax."""
        import re
        self.start_test("Check for syntax mistake in autotest lambda")
        if not os.path.isfile(test_file):
            self.progress("File %s does not exist" % test_file)
        test_file = test_file.rstrip('c')
        try:
            with open(test_file) as f:
                # check for lambda: test_function without paranthesis
                faulty_strings = re.findall(r"lambda\s*:\s*\w+.\w+\s*\)", f.read())
                if faulty_strings:
                    self.progress("Syntax error in autotest lamda at : ")
                    print(faulty_strings)
                    raise ErrorException()
        except ErrorException:
            self.progress('FAILED: "%s"' % "Check for syntax mistake in autotest lambda")
            exit(1)
        self.progress('PASSED: "%s"' % "Check for syntax mistake in autotest lambda")

    @abc.abstractmethod
    def init(self):
        """Initilialize autotest feature."""
        pass

    def test_arm_feature(self):
        """Common feature to test."""
        # TEST ARMING/DISARM
        if not self.arm_vehicle():
            self.progress("Failed to ARM")
            raise NotAchievedException()
        if not self.disarm_vehicle():
            self.progress("Failed to DISARM")
            raise NotAchievedException()
        if not self.mavproxy_arm_vehicle():
            self.progress("Failed to ARM")
            raise NotAchievedException()
        if not self.mavproxy_disarm_vehicle():
            self.progress("Failed to DISARM")
            raise NotAchievedException()
        if not self.arm_motors_with_rc_input():
            raise NotAchievedException()
        if not self.disarm_motors_with_rc_input():
            raise NotAchievedException()
        if not self.autodisarm_motors():
            raise NotAchievedException()
        # TODO : add failure test : arming check, wrong mode; Test arming magic; Same for disarm

    def test_gripper(self):
        self.context_push()
        self.set_parameter("GRIP_ENABLE", 1)
        self.fetch_parameters()
        self.set_parameter("GRIP_GRAB", 2000)
        self.set_parameter("GRIP_RELEASE", 1000)
        self.set_parameter("GRIP_TYPE", 1)
        self.set_parameter("SIM_GRPS_ENABLE", 1)
        self.set_parameter("SIM_GRPS_PIN", 8)
        self.set_parameter("SERVO8_FUNCTION", 28)
        self.set_parameter("SERVO8_MIN", 1000)
        self.set_parameter("SERVO8_MAX", 2000)
        self.set_parameter("RC9_OPTION", 19)
        self.reboot_sitl()
        self.progress("Waiting reading for arm")
        self.wait_ready_to_arm()
        self.progress("Test gripper with RC9_OPTION")
        self.progress("Releasing load")
        # non strict string matching because of catching text issue....
        self.wait_text("Gripper load releas", the_function=lambda: self.set_rc(9, 1000))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb", the_function=lambda: self.set_rc(9, 2000))
        self.progress("Releasing load")
        self.wait_text("Gripper load releas", the_function=lambda: self.set_rc(9, 1000))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb", the_function=lambda: self.set_rc(9, 2000))
        self.progress("Test gripper with Mavlink cmd")
        self.progress("Releasing load")
        self.wait_text("Gripper load releas",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_RELEASE,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_GRAB,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Releasing load")
        self.wait_text("Gripper load releas",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_RELEASE,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_GRAB,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.context_pop()
        self.reboot_sitl()
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
