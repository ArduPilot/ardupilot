from __future__ import print_function

import abc
import math
import itertools
import os
import re
import shutil
import sys
import time
import traceback
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

# https://stackoverflow.com/questions/616645/how-do-i-duplicate-sys-stdout-to-a-log-file-in-python
class TeeBoth(object):
    def __init__(self, name, mode, mavproxy_logfile):
        self.file = open(name, mode)
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        self.mavproxy_logfile = mavproxy_logfile
        self.mavproxy_logfile.set_fh(self)
        sys.stdout = self
        sys.stderr = self
    def close(self):
        sys.stdout = self.stdout
        sys.stderr = self.stderr
        self.mavproxy_logfile.set_fh(None)
        self.mavproxy_logfile = None
        self.file.close()
        self.file = None
    def write(self, data):
        self.file.write(data)
        self.stdout.write(data)
    def flush(self):
        self.file.flush()

class MAVProxyLogFile(object):
    def __init__(self):
        self.fh = None
    def close(self):
        pass
    def set_fh(self, fh):
        self.fh = fh
    def write(self, data):
        if self.fh is not None:
            self.fh.write(data)
        else:
            sys.stdout.write(data)
    def flush(self):
        if self.fh is not None:
            self.fh.flush()
        else:
            sys.stdout.flush()

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
        self.max_set_rc_timeout = 0
        self.last_wp_load = 0
        self.forced_post_test_sitl_reboots = 0
        self.skip_list = []

    @staticmethod
    def progress(text):
        """Display autotest progress text."""
        print("AUTOTEST: " + text)

    # following two functions swiped from autotest.py:
    @staticmethod
    def buildlogs_dirpath():
        return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))

    def open_mavproxy_logfile(self):
        return MAVProxyLogFile()

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
        old_bootcount= self.get_parameter('STAT_BOOTCNT')
        self.mavproxy.send("reboot\n")
        tstart = time.time()
        while True:
            if time.time() - tstart > 10:
                raise AutoTestTimeoutException("Did not detect reboot")
            try:
                if self.get_parameter('STAT_BOOTCNT', timeout=1) != old_bootcount:
                    break
            except NotAchievedException:
                pass

        # empty mav to avoid getting old timestamps:
        while self.mav.recv_match(blocking=False):
            pass

        self.initialise_after_reboot_sitl()

    def initialise_after_reboot_sitl(self):

        # after reboot stream-rates may be zero.  Prompt MAVProxy to
        # send a rate-change message by changing away from our normal
        # stream rates and back again:
        tstart = self.get_sim_time()
        while True:

            self.mavproxy.send("set streamrate %u\n" % (self.sitl_streamrate()+1))

            if self.get_sim_time() - tstart > 10:
                raise AutoTestTimeoutException("SYSTEM_TIME not received")

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

    def drain_mav(self):
        count = 0
        tstart = time.time()
        while self.mav.recv_match(type='SYSTEM_TIME', blocking=False) is not None:
            count += 1
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count/float(tdelta),)

        self.progress("Drained %u messages from mav (%s)" % (count, rate))

    #################################################
    # SIM UTILITIES
    #################################################
    def get_sim_time(self):
        """Get SITL time in seconds."""
        m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True)
        return m.time_boot_ms * 1.0e-3

    def get_sim_time_cached(self):
        """Get SITL time."""
        x = self.mav.messages.get("SYSTEM_TIME", None)
        if x is None:
            return self.get_sim_time()
        return x.time_boot_ms * 1.0e-3

    def delay_sim_time(self, delay):
        '''delay for delay seconds in simulation time'''
        m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True)
        start = m.time_boot_ms
        while True:
            m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True)
            if m.time_boot_ms - start > delay * 1000:
                return

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

    def log_download(self, filename, timeout=360, upload_logs=False):
        """Download latest log."""
        self.wait_heartbeat()
        self.mavproxy.send("log list\n")
        self.mavproxy.expect("numLogs")
        self.wait_heartbeat()
        self.wait_heartbeat()
        self.mavproxy.send("set shownoise 0\n")
        self.mavproxy.send("log download latest %s\n" % filename)
        self.mavproxy.expect("Finished downloading", timeout=timeout)
        self.wait_heartbeat()
        self.wait_heartbeat()
        if upload_logs and not os.getenv("AUTOTEST_NO_UPLOAD"):
            # optionally upload logs to server so we can see travis failure logs
            import datetime
            import glob
            import subprocess
            logdir = os.path.dirname(filename)
            datedir = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
            flist = glob.glob("logs/*.BIN")
            for e in ['BIN', 'bin', 'tlog']:
                flist += glob.glob(os.path.join(logdir, '*.%s' % e))
            print("Uploading %u logs to http://firmware.ardupilot.org/CI-Logs/%s" % (len(flist), datedir))
            cmd = ['rsync', '-avz'] + flist + ['cilogs@autotest.ardupilot.org::CI-Logs/%s/' % datedir]
            subprocess.call(cmd)

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

    def mission_directory(self):
        return testdir

    def assert_mission_files_same(self, file1, file2):
        self.progress("Comparing (%s) and (%s)" % (file1, file2, ))
        f1 = open(file1)
        f2 = open(file2)
        for l1, l2 in itertools.izip(f1, f2):
            if l1 == l2:
                # e.g. the first "QGC WPL 110" line
                continue
            if re.match("0\s", l1):
                # home changes...
                continue
            l1 = l1.rstrip()
            l2 = l2.rstrip()
            fields1 = re.split("\s+", l1)
            fields2 = re.split("\s+", l2)
            # line = int(fields1[0])
            t = int(fields1[3]) # mission item type
            for (count, (i1, i2)) in enumerate(itertools.izip(fields1, fields2)):
                if count == 2: # frame
                    if t in [mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                             mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                             mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                             mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                             mavutil.mavlink.MAV_CMD_DO_JUMP,
                             mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                             ]:
                        # ardupilot doesn't remember frame on these commands
                        if int(i1) == 3:
                            i1 = 0
                        if int(i2) == 3:
                            i2 = 0
                if count == 6: # param 3
                    if t in [mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME]:
                        # ardupilot canonicalises this to -1 for ccw or 1 for cw.
                        if float(i1) == 0:
                            i1 = 1.0
                        if float(i2) == 0:
                            i2 = 1.0
                if count == 7: # param 4
                    if t == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        # ardupilot canonicalises "0" to "1" param 4 (yaw)
                        if int(float(i1)) == 0:
                            i1 = 1
                        if int(float(i2)) == 0:
                            i2 = 1
                if 0 <= count <= 3 or 11 <= count <= 11:
                    if int(i1) != int(i2):
                        raise ValueError("Files have different content: (%s vs %s) (%s vs %s) (%d vs %d) (count=%u)" %
                                         (file1, file2, l1, l2, int(i1), int(i2), count))  # NOCI
                    continue
                if 4 <= count <= 10:
                    f_i1 = float(i1)
                    f_i2 = float(i2)
                    delta = abs(f_i1 - f_i2)
                    max_allowed_delta = 0.000009
                    if delta > max_allowed_delta:
                        raise ValueError(
                            ("Files have different (float) content: " +
                             "(%s) and (%s) " +
                             "(%s vs %s) " +
                             "(%f vs %f) " +
                             "(%.10f) " +
                             "(count=%u)") %
                            (file1, file2,
                             l1, l2,
                             f_i1, f_i2,
                             delta,
                             count)) # NOCI
                    continue
                raise ValueError("count %u not handled" % count)
        self.progress("Files same")

    def load_mission(self, filename):
        """Load a mission from a file to flight controller."""
        self.progress("Loading mission (%s)" % filename)
        path = os.path.join(self.mission_directory(), filename)
        tstart = self.get_sim_time_cached()
        while True:
            t2 = self.get_sim_time()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to do waypoint thing")
            self.mavproxy.send('wp load %s\n' % path)
            self.mavproxy.expect('Loaded ([0-9]+) waypoints from')
            load_count = self.mavproxy.match.group(1)
            # the following hack is to get around MAVProxy statustext deduping:
            while time.time() - self.last_wp_load < 3:
                self.progress("Waiting for MAVProxy de-dupe timer to expire")
                time.sleep(1)
            self.last_wp_load = time.time()
            self.mavproxy.expect("Flight plan received")
            self.mavproxy.send('wp list\n')
            self.mavproxy.expect('Requesting ([0-9]+) waypoints')
            request_count = self.mavproxy.match.group(1)
            if load_count != request_count:
                self.progress("request_count=%s != load_count=%s" %
                              (request_count, load_count))
                continue
            self.mavproxy.expect('Saved ([0-9]+) waypoints to (.+?way.txt)')
            save_count = self.mavproxy.match.group(1)
            if save_count != request_count:
                raise NotAchievedException("request count != load count")
            saved_filepath = util.reltopdir(self.mavproxy.match.group(2))
            saved_filepath = saved_filepath.rstrip()
            self.assert_mission_files_same(path, saved_filepath)
            break
        self.mavproxy.send('wp status\n')
        self.mavproxy.expect('Have (\d+) of (\d+)')
        status_have = self.mavproxy.match.group(1)
        status_want = self.mavproxy.match.group(2)
        if status_have != status_want:
            raise ValueError("status count mismatch")
        if status_have != save_count:
            raise ValueError("status have not equal to save count")

        # update num_wp
        wploader = mavwp.MAVWPLoader()
        wploader.load(path)
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

    def set_rc(self, chan, pwm, timeout=2000):
        """Setup a simulated RC control to a PWM value"""
        self.drain_mav()
        tstart = self.get_sim_time()
        wclock = time.time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.mavproxy.send('rc %u %u\n' % (chan, pwm))
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            chan_pwm = getattr(m, "chan" + str(chan) + "_raw")
            wclock_delta = time.time() - wclock
            sim_time_delta = self.get_sim_time_cached()-tstart
            if sim_time_delta == 0:
                time_ratio = None
            else:
                time_ratio = wclock_delta / sim_time_delta
            self.progress("set_rc (wc=%s st=%s r=%s): ch=%u want=%u got=%u" %
                          (wclock_delta,
                           sim_time_delta,
                           time_ratio,
                           chan,
                           pwm,
                           chan_pwm))
            if chan_pwm == pwm:
                delta = self.get_sim_time_cached() - tstart
                if delta > self.max_set_rc_timeout:
                    self.max_set_rc_timeout = delta
                return True
        raise SetRCTimeout("Failed to send RC commands to channel %s" % str(chan))

    def set_throttle_zero(self):
        """Set throttle to zero."""
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            self.set_rc(3, 1500)
        else:
            self.set_rc(3, 1000)

    def set_output_to_max(self, chan):
        """Set output to max with RC Radio taking into account REVERSED parameter."""
        is_reversed = self.get_parameter("RC%u_REVERSED" % chan)
        out_max = int(self.get_parameter("RC%u_MAX" % chan))
        out_min = int(self.get_parameter("RC%u_MIN" % chan))
        if is_reversed == 0:
            self.set_rc(chan, out_max)
        else:
            self.set_rc(chan, out_min)

    def set_output_to_min(self, chan):
        """Set output to min with RC Radio taking into account REVERSED parameter."""
        is_reversed = self.get_parameter("RC%u_REVERSED" % chan)
        out_max = int(self.get_parameter("RC%u_MAX" % chan))
        out_min = int(self.get_parameter("RC%u_MIN" % chan))
        if is_reversed == 0:
            self.set_rc(chan, out_min)
        else:
            self.set_rc(chan, out_max)

    def set_output_to_trim(self, chan):
        """Set output to trim with RC Radio."""
        out_trim = int(self.get_parameter("RC%u_TRIM" % chan))
        self.set_rc(chan, out_trim)

    def get_rudder_channel(self):
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            return int(self.get_parameter("RCMAP_YAW"))
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            return int(self.get_parameter("RCMAP_YAW"))
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            return int(self.get_parameter("RCMAP_ROLL"))
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_SUBMARINE:
            raise ErrorException("Arming with rudder is not supported by Submarine")

    def armed(self):
        """Return true if vehicle is armed and safetyoff"""
        return self.mav.motors_armed()

    def arm_vehicle(self, timeout=20):
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
                     timeout=timeout)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("Motors ARMED")
                return True
        raise AutoTestTimeoutException("Unable to ARM with mavlink")

    def disarm_vehicle(self, timeout=20):
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
                     timeout=timeout)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                self.progress("Motors DISARMED")
                return True
        raise AutoTestTimeoutException("Unable to DISARM with mavlink")

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

    def arm_motors_with_rc_input(self, timeout=20):
        """Arm motors with radio."""
        self.progress("Arm motors with radio")
        self.set_output_to_max(self.get_rudder_channel())
        tstart = self.get_sim_time()
        while True:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                arm_delay = self.get_sim_time() - tstart
                self.progress("MOTORS ARMED OK WITH RADIO")
                self.set_output_to_trim(self.get_rudder_channel())
                self.progress("Arm in %ss" % arm_delay)  # TODO check arming time
                return True
            tdelta = self.get_sim_time() - tstart
            print("Not armed after %f seconds" % (tdelta))
            if tdelta > timeout:
                break
        self.progress("FAILED TO ARM WITH RADIO")
        self.set_output_to_trim(self.get_rudder_channel())
        return False

    def disarm_motors_with_rc_input(self, timeout=20):
        """Disarm motors with radio."""
        self.progress("Disarm motors with radio")
        self.set_output_to_min(self.get_rudder_channel())
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_delay = self.get_sim_time() - tstart
                self.progress("MOTORS DISARMED OK WITH RADIO")
                self.set_output_to_trim(self.get_rudder_channel())
                self.progress("Disarm in %ss" % disarm_delay)  # TODO check disarming time
                return True
        self.progress("FAILED TO DISARM WITH RADIO")
        self.set_output_to_trim(self.get_rudder_channel())
        return False

    def arm_motors_with_switch(self, switch_chan, timeout=20):
        """Arm motors with switch."""
        self.progress("Arm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("MOTORS ARMED OK WITH SWITCH")
                return True
        self.progress("FAILED TO ARM WITH SWITCH")
        return False

    def disarm_motors_with_switch(self, switch_chan, timeout=20):
        """Disarm motors with switch."""
        self.progress("Disarm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 1000)
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                self.progress("MOTORS DISARMED OK WITH SWITCH")
                return True
        self.progress("FAILED TO DISARM WITH SWITCH")
        return False

    def autodisarm_motors(self):
        """Autodisarm motors."""
        self.progress("Autodisarming motors")
        if self.mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:  # NOT IMPLEMENTED ON ROVER
            self.progress("MOTORS AUTODISARMED OK")
            return True
        tstart = self.get_sim_time()
        timeout = 15
        while self.get_sim_time() < tstart + timeout:
            self.wait_heartbeat()
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

    def set_parameter(self, name, value, add_to_context=True, epsilon=0.00001):
        """Set parameters from vehicle."""
        old_value = self.get_parameter(name, retry=2)
        for i in range(1, 10):
            self.mavproxy.send("param set %s %s\n" % (name, str(value)))
            returned_value = self.get_parameter(name)
            delta = float(value) - returned_value
            if abs(delta) < epsilon:
                # yes, exactly equal.
                if add_to_context:
                    self.context_get().parameters.append((name, old_value))
                if self.should_fetch_all_for_parameter_change(name.upper()) and value != 0:
                    self.fetch_parameters()
                return
        raise ValueError("Param fetch returned incorrect value (%s) vs (%s)"
                         % (returned_value, value))

    def get_parameter(self, name, retry=1, timeout=60):
        """Get parameters from vehicle."""
        for i in range(0, retry):
            self.mavproxy.send("param fetch %s\n" % name)
            try:
                self.mavproxy.expect("%s = ([-0-9.]*)\r\n" % (name,), timeout=timeout/retry)
                return float(self.mavproxy.match.group(1))
            except pexpect.TIMEOUT:
                pass
        raise NotAchievedException("Failed to retrieve parameter")

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
                want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                timeout=10):
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
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get good COMMAND_ACK")
            m = self.mav.recv_match(type='COMMAND_ACK',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("ACK received: %s" % str(m))
            if m.command == command:
                if m.result != want_result:
                    raise ValueError("Expected %s got %s" % (want_result,
                                                             m.result))
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

    def change_mode(self, mode):
        '''change vehicle flightmode'''
        self.mavproxy.send('mode %s\n' % mode)
        self.wait_mode(mode)

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
        raise AutoTestTimeoutException("No AUTOPILOT_VERSION received")

    def get_mode_from_mode_mapping(self, mode):
        """Validate and return the mode number from a string or int."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None:
            raise ErrorException("No mode map")
        if isinstance(mode, str):
            if mode in mode_map:
                return mode_map.get(mode)
        if mode in mode_map.values():
            return mode
        self.progress("Available modes '%s'" % mode_map)
        raise ErrorException("Unknown mode '%s'" % mode)

    def do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message."""
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise AutoTestTimeoutException("Failed to change mode")
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
                                    timeout=5)
            if m is None:
                raise ErrorException("Heartbeat not received")
            if m.custom_mode == custom_mode:
                return

    def mavproxy_do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message with Mavproxy."""
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise AutoTestTimeoutException("Failed to change mode")
            self.mavproxy.send("long DO_SET_MODE %u %u\n" %
                               (base_mode, custom_mode))
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=5)
            if m is None:
                raise ErrorException("Did not receive heartbeat")
            if m.custom_mode == custom_mode:
                return True

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
                raise NotAchievedException("Did not achieve heading")
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
        last_wait_alt_msg = 0
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if m is None:
                continue
            if relative:
                alt = m.relative_alt/1000.0 # mm -> m
            else:
                alt = m.alt/1000.0 # mm -> m

            climb_rate = alt - previous_alt
            previous_alt = alt
            if self.get_sim_time_cached() - last_wait_alt_msg > 1:
                self.progress("Wait Altitude: Cur:%u, min_alt:%u, climb_rate: %u"
                              % (alt, alt_min, climb_rate))
                last_wait_alt_msg = self.get_sim_time_cached()
            if alt >= alt_min and alt <= alt_max:
                self.progress("Altitude OK")
                return True
        raise WaitAltitudeTimout("Failed to attain altitude range")

    def wait_groundspeed(self, gs_min, gs_max, timeout=30):
        """Wait for a given ground speed range."""
        self.progress("Waiting for groundspeed between %.1f and %.1f" %
                      (gs_min, gs_max))
        last_print = 0
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if self.get_sim_time_cached() - last_print > 1:
                self.progress("Wait groundspeed %.1f, target:%.1f" %
                              (m.groundspeed, gs_min))
                last_print = self.get_sim_time_cached()
            if m.groundspeed >= gs_min and m.groundspeed <= gs_max:
                return True
        raise WaitGroundSpeedTimeout("Failed to attain groundspeed range")

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
        raise WaitRollTimeout("Failed to attain roll %d" % roll)

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
        raise WaitPitchTimeout("Failed to attain pitch %d" % pitch)

    def wait_heading(self, heading, accuracy=5, timeout=30):
        """Wait for a given heading."""
        tstart = self.get_sim_time()
        self.progress("Waiting for heading %u with accuracy %u" %
                      (heading, accuracy))
        last_print_time = 0
        while True:
            now = self.get_sim_time_cached()
            if now - tstart >= timeout:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if now - last_print_time > 1:
                self.progress("Heading %u (want %f +- %f)" %
                              (m.heading, heading, accuracy))
                last_print_time = now
            if math.fabs(m.heading - heading) <= accuracy:
                self.progress("Attained heading %u" % heading)
                return True
        raise WaitHeadingTimeout("Failed to attain heading %u" % heading)

    def wait_distance(self, distance, accuracy=5, timeout=30):
        """Wait for flight of a given distance."""
        tstart = self.get_sim_time()
        start = self.mav.location()
        last_distance_message = 0
        while self.get_sim_time() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            if self.get_sim_time_cached() - last_distance_message >= 1:
                self.progress("Distance=%.2f meters want=%.2f" %
                              (delta, distance))
                last_distance_message = self.get_sim_time_cached()
            if math.fabs(delta - distance) <= accuracy:
                self.progress("Attained distance %.2f meters OK" % delta)
                return True
            if delta > (distance + accuracy):
                raise WaitDistanceTimeout("Failed distance - overshoot delta=%f dist=%f"
                                          % (delta, distance))
        raise WaitDistanceTimeout("Failed to attain distance %u" % distance)

    def wait_servo_channel_value(self, channel, value, timeout=2):
        """wait for channel to hit value"""
        channel_field = "servo%u_raw" % channel
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel never achieved value")
            m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field, None)
            self.progress("SERVO_OUTPUT_RAW.%s=%u want=%u" %
                          (channel_field, m_value, value))
            if m_value is None:
                raise ValueError("message (%s) has no field %s" %
                                 (str(m), channel_field))
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
        last_distance_message = 0
        while self.get_sim_time() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(loc, pos)
            if self.get_sim_time_cached() - last_distance_message >= 1:
                self.progress("Distance %.2f meters alt %.1f" % (delta, pos.alt))
                last_distance_message = self.get_sim_time_cached()
            if delta <= accuracy:
                height_delta = math.fabs(pos.alt - target_altitude)
                if height_accuracy != -1 and height_delta > height_accuracy:
                    continue
                self.progress("Reached location (%.2f meters)" % delta)
                return True
        raise WaitLocationTimeout("Failed to attain location")

    def wait_current_waypoint(self, wpnum, timeout=60):
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            seq = self.mav.waypoint_current()
            self.progress("Waiting for wp=%u current=%u" % (wpnum, seq))
            if seq == wpnum:
                break

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

        self.progress("wait for waypoint ranges start=%u end=%u"
                      % (wpnum_start, wpnum_end))
        # if start_wp != wpnum_start:
        #    raise WaitWaypointTimeout("test: Expected start waypoint %u "
        #                              "but got %u" %
        #                  (wpnum_start, start_wp))

        last_wp_msg = 0
        while self.get_sim_time_cached() < tstart + timeout:
            seq = self.mav.waypoint_current()
            m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
                                    blocking=True)
            wp_dist = m.wp_dist
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)

            # if we changed mode, fail
            if self.mav.flightmode != mode:
                raise WaitWaypointTimeout('Exited %s mode' % mode)

            if self.get_sim_time_cached() - last_wp_msg > 1:
                self.progress("WP %u (wp_dist=%u Alt=%d), current_wp: %u,"
                              "wpnum_end: %u" %
                              (seq, wp_dist, m.alt, current_wp, wpnum_end))
                last_wp_msg = self.get_sim_time_cached()
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
                raise WaitWaypointTimeout(("Skipped waypoint! Got wp %u expected %u"
                                           % (seq, current_wp+1)))
        raise WaitWaypointTimeout("Timed out waiting for waypoint %u of %u" %
                                  (wpnum_end, wpnum_end))

    def wait_mode(self, mode, timeout=60):
        """Wait for mode to change."""
        self.get_mode_from_mode_mapping(mode)
        self.progress("Waiting for mode %s" % mode)
        tstart = self.get_sim_time()
        self.wait_heartbeat()
        while self.mav.flightmode != mode:
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                    self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    self.get_sim_time() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
            self.wait_heartbeat()
        # self.progress("heartbeat mode %s Want: %s" % (
        # self.mav.flightmode, mode))
        self.progress("Got mode %s" % mode)

    def wait_ready_to_arm(self, timeout=None, require_absolute=True):
        # wait for EKF checks to pass
        self.progress("Waiting reading for arm")
        return self.wait_ekf_happy(timeout=timeout,
                                   require_absolute=require_absolute)

    def wait_heartbeat(self, *args, **x):
        '''as opposed to mav.wait_heartbeat, raises an exception on timeout'''
        self.drain_mav()
        m = self.mav.wait_heartbeat(*args, **x)
        if m is None:
            raise AutoTestTimeoutException("Did not receive heartbeat")

    def wait_ekf_happy(self, timeout=30, require_absolute=True):
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
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL)
        # none of these bits must be set for arming to happen:
        error_bits = (mavutil.mavlink.ESTIMATOR_CONST_POS_MODE |
                      mavutil.mavlink.ESTIMATOR_ACCEL_ERROR)
        if require_absolute:
            required_value |= (mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS |
                               mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                               mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_ABS)
            error_bits |= mavutil.mavlink.ESTIMATOR_GPS_GLITCH

        self.progress("Waiting for EKF value %u" % required_value)
        last_err_print_time = 0
        last_print_time = 0
        while timeout is None or self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True)
            current = m.flags
            if self.get_sim_time_cached() - last_print_time > 1:
                self.progress("Wait EKF.flags: required:%u current:%u" %
                              (required_value, current))
                last_print_time = self.get_sim_time_cached()
            errors = current & error_bits
            if errors and self.get_sim_time_cached() - last_err_print_time > 1:
                self.progress("Wait EKF.flags: errors=%u" % errors)
                last_err_print_time = self.get_sim_time_cached()
                continue
            if (current & required_value == required_value):
                self.progress("EKF Flags OK")
                return True
        raise AutoTestTimeoutException("Failed to get EKF.flags=%u" %
                                       required_value)

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
        self.progress()
        raise AutoTestTimeoutException("Failed to received text : %s" %
                                       text.lower())

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

    def check_sitl_reset(self):
        if self.armed():
            self.forced_post_test_sitl_reboots += 1
            self.reboot_sitl() # that'll learn it

    def run_one_test(self, name, desc, test_function, interact=False):
        '''new-style run-one-test used by run_tests'''
        test_output_filename = self.buildlogs_path("%s-%s.txt" %
                                                   (self.log_name, name))
        tee = TeeBoth(test_output_filename, 'w', self.mavproxy_logfile)

        prettyname = "%s (%s)" % (name, desc)
        self.start_test(prettyname)

        self.context_push()

        try:
            self.change_mode(self.default_mode())
            test_function()
        except Exception as e:
            self.progress("Exception caught: %s" % traceback.format_exc(e))
            self.context_pop()
            self.progress('FAILED: "%s": %s (see %s)' %
                          (prettyname, repr(e), test_output_filename))
            self.fail_list.append((prettyname, e, test_output_filename))
            if interact:
                self.progress("Starting MAVProxy interaction as directed")
                self.mavproxy.interact()
            tee.close()
            tee = None
            self.check_sitl_reset()
            return
        self.context_pop()
        self.progress('PASSED: "%s"' % prettyname)
        tee.close()
        tee = None

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
                    desc = ("Syntax error in autotest lambda at : " +
                            faulty_strings)
                    raise ErrorException(desc)
        except ErrorException:
            self.progress('FAILED: "%s"' % "Check for syntax mistake in autotest lambda")
            exit(1)
        self.progress('PASSED: "%s"' % "Check for syntax mistake in autotest lambda")

    @abc.abstractmethod
    def init(self):
        """Initilialize autotest feature."""
        pass

    def expect_command_ack(self, command):
        m = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if m is None:
            raise NotAchievedException()
        if m.command != command:
            raise ValueError()
        if m.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise NotAchievedException()

    def poll_home_position(self):
        old = self.mav.messages.get("HOME_POSITION", None)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 30:
                raise NotAchievedException("Failed to poll home position")
            self.mav.mav.command_long_send(
                1,
                1,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0)
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
            if m is None:
                continue
            if m.command != mavutil.mavlink.MAV_CMD_GET_HOME_POSITION:
                continue
            if m.result != 0:
                continue
            break
        m = self.mav.messages.get("HOME_POSITION", None)
        if old is not None and m._timestamp == old._timestamp:
            raise NotAchievedException("home position not updated")
        return m

    def test_arm_feature(self):
        """Common feature to test."""
        self.context_push()
        # TEST ARMING/DISARM
        self.set_parameter("ARMING_RUDDER", 2)  # allow arm and disarm with rudder on first tests
        interlock_channel = 8  # Plane got flighmode_ch on channel 8
        if self.mav.mav_type is not mavutil.mavlink.MAV_TYPE_HELICOPTER:  # heli don't need interlock option
            interlock_channel = 9
            self.set_parameter("RC%u_OPTION" % interlock_channel, 32)
        self.set_rc(interlock_channel, 1000)
        self.set_throttle_zero()
        self.start_test("Test normal arm and disarm features")
        self.wait_ready_to_arm()
        self.progress("default arm_vehicle() call")
        if not self.arm_vehicle():
            raise NotAchievedException("Failed to ARM")
        self.progress("default disarm_vehicle() call")
        if not self.disarm_vehicle():
            raise NotAchievedException("Failed to DISARM")
        self.progress("arm with mavproxy")
        if not self.mavproxy_arm_vehicle():
            raise NotAchievedException("Failed to ARM")
        self.progress("disarm with mavproxy")
        if not self.mavproxy_disarm_vehicle():
            raise NotAchievedException("Failed to DISARM")
        if self.mav.mav_type != mavutil.mavlink.MAV_TYPE_SUBMARINE:
            self.progress("arm with rc input")
            if not self.arm_motors_with_rc_input():
                raise NotAchievedException("Failed to arm with RC input")
            self.progress("disarm with rc input")
            if not self.disarm_motors_with_rc_input():
                raise NotAchievedException("Failed to disarm with RC input")

        # Disable auto disarm for next test
        # Rover and Sub don't have auto disarm
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.set_parameter("DISARM_DELAY", 0)
        elif self.mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            self.set_parameter("LAND_DISARMDELAY", 0)
        # Sub has no 'switches'
        if self.mav.mav_type != mavutil.mavlink.MAV_TYPE_SUBMARINE:
            self.start_test("Test arm and disarm with switch")
            arming_switch = 7
            self.set_parameter("RC%d_OPTION" % arming_switch, 41)
            self.set_rc(arming_switch, 1000)
            # delay so a transition is seen by the RC switch code:
            self.delay_sim_time(0.5)
            if not self.arm_motors_with_switch(arming_switch):
                raise NotAchievedException("Failed to arm with switch")
            if not self.disarm_motors_with_switch(arming_switch):
                raise NotAchievedException("Failed to disarm with switch")
            self.set_rc(arming_switch, 1000)
        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.start_test("Test arming failure with throttle too high")
            self.set_rc(3, 1800)
            try:
                if self.arm_vehicle():
                    raise NotAchievedException("Armed when throttle too high")
            except AutoTestTimeoutException():
                pass
            except ValueError:
                pass
            if self.arm_motors_with_rc_input():
                raise NotAchievedException(
                    "Armed via RC when throttle too high")
            if self.arm_motors_with_switch(arming_switch):
                raise NotAchievedException("Armed via RC when switch too high")
            self.set_throttle_zero()
            self.set_rc(arming_switch, 1000)

        # Sub doesn't have 'stick commands'
        if self.mav.mav_type != mavutil.mavlink.MAV_TYPE_SUBMARINE:
            self.start_test("Test arming failure with ARMING_RUDDER=0")
            self.set_parameter("ARMING_RUDDER", 0)
            if self.arm_motors_with_rc_input():
                raise NotAchievedException(
                    "Armed with rudder when ARMING_RUDDER=0")
            self.start_test("Test disarming failure with ARMING_RUDDER=0")
            self.arm_vehicle()
            if self.disarm_motors_with_rc_input():
                raise NotAchievedException(
                    "Disarmed with rudder when ARMING_RUDDER=0")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.start_test("Test disarming failure with ARMING_RUDDER=1")
            self.set_parameter("ARMING_RUDDER", 1)
            self.arm_vehicle()
            if self.disarm_motors_with_rc_input():
                raise NotAchievedException(
                    "Disarmed with rudder with ARMING_RUDDER=1")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.set_parameter("ARMING_RUDDER", 2)

        if self.mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_HELICOPTER,
                                 mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                 mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                 mavutil.mavlink.MAV_TYPE_COAXIAL,
                                 mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            self.start_test("Test arming failure with interlock enabled")
            self.set_rc(interlock_channel, 2000)
            if self.arm_motors_with_rc_input():
                raise NotAchievedException(
                    "Armed with RC input when interlock enabled")
            if self.arm_motors_with_switch(arming_switch):
                raise NotAchievedException(
                    "Armed with switch when interlock enabled")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.set_rc(arming_switch, 1000)
            self.set_rc(interlock_channel, 1000)
            if self.mav.mav_type is mavutil.mavlink.MAV_TYPE_HELICOPTER:
                self.start_test("Test motor interlock enable can't be set while disarmed")
                self.set_rc(interlock_channel, 2000)
                channel_field = "servo%u_raw" % interlock_channel
                interlock_value = self.get_parameter("SERVO%u_MIN" % interlock_channel)
                tstart = self.get_sim_time()
                while True:
                    if self.get_sim_time_cached() - tstart > 20:
                        self.set_rc(8, 1000)
                        break # success!
                    m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                            blocking=True,
                                            timeout=2)
                    if m is None:
                        continue
                    m_value = getattr(m, channel_field, None)
                    if m_value is None:
                        self.set_rc(8, 1000)
                        raise ValueError("Message has no %s field" %
                                         channel_field)
                    self.progress("SERVO_OUTPUT_RAW.%s=%u want=%u" %
                                  (channel_field, m_value, interlock_value))
                    if m_value != interlock_value:
                        self.set_rc(8, 1000)
                        raise NotAchievedException("Motor interlock was changed while disarmed")

            self.set_rc(8, 1000)
        self.progress("ALL PASS")
        self.context_pop()
        # TODO : add failure test : arming check, wrong mode; Test arming magic; Same for disarm

    def get_message_rate(self, victim_message, timeout):
        tstart = self.get_sim_time()
        count = 0
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type=victim_message,
                                    blocking=True,
                                    timeout=0.1
                                    )
            if m is not None:
                count += 1
        time_delta = self.get_sim_time_cached() - tstart
        self.progress("%s count after %f seconds: %u" %
                      (victim_message, time_delta, count))
        return count/time_delta

    def rate_to_interval_us(self, rate):
        return 1/float(rate)*1000000.0

    def test_rate(self, desc, in_rate, expected_rate):
        self.progress("###### %s" % desc)
        self.progress("Setting rate to %u" % in_rate)
        # SET_MESSAGE_INTERVAL rates are given in microseconds
        if in_rate == 0 or in_rate == -1:
            set_interval = in_rate
        else:
            set_interval = self.rate_to_interval_us(in_rate)

        self.mavproxy.send("long SET_MESSAGE_INTERVAL %u %d\n" %
                           (self.victim_message_id, set_interval))
        self.mav.recv_match(type='COMMAND_ACK', blocking=True)
        new_measured_rate = self.get_message_rate(self.victim_message, 10)
        self.progress("Measured rate: %f (want %u)" %
                      (new_measured_rate, expected_rate))
        if round(new_measured_rate) != expected_rate:
            raise NotAchievedException("Rate not achieved (got %f want %u)" %
                                       (new_measured_rate, expected_rate))

        # make sure get_message_interval works:
        self.mavproxy.send("long GET_MESSAGE_INTERVAL %u\n" %
                           (self.victim_message_id))
        m = self.mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)
        want = set_interval
        if set_interval == 0:
            want = self.rate_to_interval_us(expected_rate)

        if m.interval_us != want:
            raise NotAchievedException("Did not read same interval back from autopilot: want=%d got=%d)" %
            (want, m.interval_us))
        m = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
        if m.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise NotAchievedException("Expected ACCEPTED for reading message interval")

    def test_set_message_interval(self):
        self.victim_message = 'VFR_HUD'
        self.victim_message_id = mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD
        try:
            # tell MAVProxy to stop stuffing around with the rates:
            self.mavproxy.send("set streamrate -1\n")

            rate = round(self.get_message_rate(self.victim_message, 20))
            self.progress("Initial rate: %u" % rate)

            self.test_rate("Test set to %u" % (rate/2,), rate/2, rate/2)
            # this assumes the streamrates have not been played with:
            self.test_rate("Resetting original rate using 0-value", 0, rate)
            self.test_rate("Disabling using -1-value", -1, 0)
            self.test_rate("Resetting original rate", rate, rate)

            self.progress("try getting a message which is not ordinarily streamed out")
            rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
            if rate != 0:
                raise PreconditionFailedException("Already getting CAMERA_FEEDBACK")
            self.progress("try various message rates")
            for want_rate in range(5, 14):
                self.mavproxy.send(
                    "long SET_MESSAGE_INTERVAL %u %d\n" %
                    (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK,
                     self.rate_to_interval_us(want_rate)))
                rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
                self.progress("Want=%f got=%f" % (want_rate, rate))
                if rate != want_rate:
                    raise NotAchievedException("Did not get expected rate")


            self.progress("try at the main loop rate")
            # have to reset the speedup as MAVProxy can't keep up otherwise
            old_speedup = self.get_parameter("SIM_SPEEDUP")
            self.set_parameter("SIM_SPEEDUP", 1.0)
            # ArduPilot currently limits message rate to 80% of main loop rate:
            want_rate = self.get_parameter("SCHED_LOOP_RATE") * 0.8
            self.mavproxy.send("long SET_MESSAGE_INTERVAL %u %d\n" %
                               (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK,
                                self.rate_to_interval_us(want_rate)))
            rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
            self.set_parameter("SIM_SPEEDUP", old_speedup)
            self.progress("Want=%f got=%f" % (want_rate, rate))
            if abs(rate - want_rate) > 2:
                raise NotAchievedException("Did not get expected rate")

            non_existant_id = 145
            self.mavproxy.send("long GET_MESSAGE_INTERVAL %u\n" %
                               (non_existant_id))
            m = self.mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)
            if m.interval_us != 0:
                raise NotAchievedException("Supposed to get 0 back for unsupported stream")
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
            if m.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise NotAchievedException("Getting rate of unsupported message is still a success")

            sr = self.sitl_streamrate()
            self.mavproxy.send("set streamrate %u\n" % sr)

        except Exception as e:
            # tell MAVProxy to start stuffing around with the rates:
            sr = self.sitl_streamrate()
            self.mavproxy.send("set streamrate %u\n" % sr)
            raise e

    def clear_mission(self):
        self.mavproxy.send("wp clear\n")
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        num_wp = mavwp.MAVWPLoader().count()
        if num_wp != 0:
            raise NotAchievedException("Failed to clear mission")

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
        self.set_parameter("SERVO9_MIN", 1000)
        self.set_parameter("SERVO9_MAX", 2000)
        self.set_parameter("RC9_OPTION", 19)
        self.set_rc(9, 1500)
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
    #     # num_wp = self.load_mission(
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

    def initial_mode(self):
        '''return mode vehicle should start in with no RC inputs set'''
        return None

    def initial_mode_switch_mode(self):
        '''return mode vehicle should start in with default RC inputs set'''
        return None

    def wait_for_initial_mode(self):
        '''wait until we get a heartbeat with an expected initial mode (the
one specified in the vehicle constructor)'''
        want = self.initial_mode()
        if want is None:
            return
        self.progress("Waiting for initial mode %s" % want)
        self.wait_mode(want)

    def wait_for_mode_switch_poll(self):
        '''look for a transition from boot-up-mode (e.g. the flightmode
specificied in Copter's constructor) to the one specified by the mode
switch value'''
        want = self.initial_mode_switch_mode()
        if want is None:
            return
        self.progress("Waiting for mode-switch mode %s" % want)
        self.wait_mode(want)

    def start_subtest(self, description):
        self.progress("-")
        self.progress("---------- %s  ----------" % description)
        self.progress("-")

    def test_skipped(self, test, reason):
        (name, desc, func) = test
        self.progress("##### %s is skipped: %s" % (name, reason))
        self.skip_list.append((test, reason))

    def run_tests(self, tests):
        """Autotest vehicle in SITL."""
        self.check_test_syntax(test_file=os.path.realpath(__file__))
        if not self.hasInit:
            self.init()

        self.fail_list = []

        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.wait_heartbeat()
            self.wait_for_initial_mode()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.wait_for_mode_switch_poll()

            for test in tests:
                (name, desc, func) = test
                self.run_one_test(name, desc, func)

        except pexpect.TIMEOUT:
            self.progress("Failed with timeout")
            self.fail_list.append(["Failed with timeout", None, None])
        self.close()

        if len(self.skip_list):
            self.progress("Skipped tests:")
            for skipped in self.skip_list:
                (test, reason) = skipped
                (name, desc, func) = test
                print("  %s (see %s)" % (name, reason))

        if len(self.fail_list):
            self.progress("Failing tests:")
            for failure in self.fail_list:
                (desc, exception, debug_filename) = failure
                print("  %s (%s) (see %s)" % (desc, exception, debug_filename))
            return False

        return True

    def disabled_tests(self):
        return {}

    def tests(self):
        return []

    def post_tests_announcements(self):
        if self.forced_post_test_sitl_reboots != 0:
            print("Had to force-reset SITL %u times" %
                  (self.forced_post_test_sitl_reboots,))

    def autotest(self):
        """Autotest used by ArduPilot autotest CI."""
        all_tests = self.tests()
        disabled = self.disabled_tests()
        tests = []
        for test in all_tests:
            (name, desc, func) = test
            if name in disabled:
                self.test_skipped(test, disabled[name])
                continue
            tests.append(test)

        ret = self.run_tests(tests)
        self.post_tests_announcements()
        return ret
