from __future__ import print_function

import abc
import math
import os
import re
import shutil
import sys
import time
import traceback
import pexpect
import fnmatch
import operator

from MAVProxy.modules.lib import mp_util

from pymavlink import mavwp, mavutil, DFReader
from pymavlink import mavextra

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

try:
    from itertools import izip as zip
except ImportError:
    # probably python2
    pass

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

class ArmedAtEndOfTestException(ErrorException):
    """Created when test left vehicle armed"""
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
                 binary,
                 valgrind=False,
                 gdb=False,
                 speedup=8,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 lldb=False,
                 breakpoints=[],
                 disable_breakpoints=False,
                 viewerip=None,
                 use_map=False,
                 _show_test_timings=False):

        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.lldb = lldb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver
        self.breakpoints = breakpoints
        self.disable_breakpoints = disable_breakpoints
        self.speedup = speedup

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
        self.run_tests_called = False
        self._show_test_timings = _show_test_timings
        self.test_timings = dict()

    @staticmethod
    def progress(text):
        """Display autotest progress text."""
        print("AUTOTEST: " + text)

    # following two functions swiped from autotest.py:
    @staticmethod
    def buildlogs_dirpath():
        return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))

    def sitl_home(self):
        HOME = self.sitl_start_location()
        return "%f,%f,%u,%u" % (HOME.lat,
                                HOME.lng,
                                HOME.alt,
                                HOME.heading)

    def mavproxy_version(self):
        '''return the current version of mavproxy as a tuple e.g. (1,8,8)'''
        return util.MAVProxy_version()

    def mavproxy_version_gt(self, major, minor, point):
        if os.getenv("AUTOTEST_FORCE_MAVPROXY_VERSION", None) is not None:
            return True
        (got_major, got_minor, got_point) = self.mavproxy_version()
        print("Got: %s.%s.%s" % (got_major, got_minor, got_point))
        if got_major > major:
            return True
        elif got_major < major:
            return False
        if got_minor > minor:
            return True
        elif got_minor < minor:
            return False
        return got_point > point

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
               '--streamrate=%u' % self.sitl_streamrate(),
               '--cmd="set heartbeat %u"' % self.speedup]
        if self.viewerip:
            ret.append("--out=%s:14550" % self.viewerip)
        if self.use_map:
            ret.append('--map')

        return ret

    def vehicleinfo_key(self):
        return self.log_name()

    def repeatedly_apply_parameter_file(self, filepath):
        '''keep applying a parameter file until no parameters changed'''
        for i in range(0, 3):
            self.mavproxy.send("param load %s\n" % filepath)
            while True:
                line = self.mavproxy.readline()
                match = re.match(".*Loaded [0-9]+ parameters.*changed ([0-9]+)",
                                 line)
                if match is not None:
                    if int(match.group(1)) == 0:
                        return
                    break
        raise NotAchievedException()

    def apply_defaultfile_parameters(self):
        """Apply parameter file."""
        self.progress("Applying default parameters file")
        # setup test parameters
        vinfo = vehicleinfo.VehicleInfo()
        if self.params is None:
            frames = vinfo.options[self.vehicleinfo_key()]["frames"]
            self.params = frames[self.frame]["default_params_filename"]
        if not isinstance(self.params, list):
            self.params = [self.params]
        for x in self.params:
            self.repeatedly_apply_parameter_file(os.path.join(testdir, x))
        self.set_parameter('LOG_REPLAY', 1)
        self.set_parameter('LOG_DISARMED', 1)
        self.reboot_sitl()
        self.fetch_parameters()

    def count_lines_in_filepath(self, filepath):
        return len([i for i in open(filepath)])

    def count_expected_fence_lines_in_filepath(self, filepath):
        count = 0
        is_qgc = False
        for i in open(filepath):
            i = re.sub("#.*", "", i) # trim comments
            if i.isspace():
                # skip empty lines
                continue
            if re.match("QGC", i):
                # skip QGC header line
                is_qgc = True
                continue
            count += 1
        if is_qgc:
            count += 2 # file doesn't include return point + closing point
        return count

    def load_fence_using_mavproxy(self, filename):
        self.set_parameter("FENCE_TOTAL", 0)
        filepath = os.path.join(self.mission_directory(), filename)
        count = self.count_expected_fence_lines_in_filepath(filepath)
        self.mavproxy.send('fence load %s\n' % filepath)
#        self.mavproxy.expect("Loaded %u (geo-)?fence" % count)
        tstart = self.get_sim_time_cached()
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to do load")
            newcount = self.get_parameter("FENCE_TOTAL")
            self.progress("fence total: %u want=%u" % (newcount, count))
            if count == newcount:
                break
            self.delay_sim_time(1)

    def load_fence(self, filename):
        self.load_fence_using_mavproxy(filename)

    def fetch_parameters(self):
        self.mavproxy.send("param fetch\n")
        self.mavproxy.expect("Received [0-9]+ parameters")

    def reboot_sitl_mav(self, required_bootcount=None):
        """Reboot SITL instance using mavlink and wait for it to reconnect."""
        old_bootcount = self.get_parameter('STAT_BOOTCNT')
        # ardupilot SITL may actually NAK the reboot; replace with
        # run_cmd when we don't do that.
        self.mav.mav.command_long_send(self.sysid_thismav(),
                                       1,
                                       mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       1,  # confirmation
                                       1, # reboot autopilot
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0)
        self.detect_and_handle_reboot(old_bootcount, required_bootcount=required_bootcount)

    def reboot_sitl(self, required_bootcount=None):
        """Reboot SITL instance and wait for it to reconnect."""
        self.progress("Rebooting SITL")
        self.reboot_sitl_mav(required_bootcount=required_bootcount)
        self.assert_simstate_location_is_at_startup_location()

    def reboot_sitl_mavproxy(self, required_bootcount=None):
        """Reboot SITL instance using MAVProxy and wait for it to reconnect."""
        old_bootcount = self.get_parameter('STAT_BOOTCNT')
        self.mavproxy.send("reboot\n")
        self.detect_and_handle_reboot(old_bootcount, required_bootcount=required_bootcount)

    def detect_and_handle_reboot(self, old_bootcount, required_bootcount=None, timeout=10):
        tstart = time.time()
        if required_bootcount is None:
            required_bootcount = old_bootcount + 1
        while True:
            if time.time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not detect reboot")
            try:
                current_bootcount = self.get_parameter('STAT_BOOTCNT', timeout=1)
                print("current=%s required=%u" % (str(current_bootcount), required_bootcount))
                if current_bootcount == required_bootcount:
                    break
            except NotAchievedException:
                pass

        # empty mav to avoid getting old timestamps:
        self.drain_mav()

        self.initialise_after_reboot_sitl()

    def set_streamrate(self, streamrate):
        tstart = time.time()
        while True:
            if time.time() - tstart > 10:
                raise AutoTestTimeoutException("stream rate change failed")

            self.mavproxy.send("set streamrate %u\n" % (streamrate))
            self.mavproxy.send("set streamrate\n")
            self.mavproxy.expect('.*streamrate ([0-9]+)', timeout=1)
            rate = self.mavproxy.match.group(1)
            print("rate: %s" % str(rate))
            if int(rate) == int(streamrate):
                break

        if streamrate <= 0:
            return

        self.drain_mav()
        m = self.mav.recv_match(type='SYSTEM_TIME',
                                blocking=True,
                                timeout=10)
        print("Received (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Did not get SYSTEM_TIME")
        self.drain_mav()

    def initialise_after_reboot_sitl(self):

        # after reboot stream-rates may be zero.  Prompt MAVProxy to
        # send a rate-change message by changing away from our normal
        # stream rates and back again:
        self.set_streamrate(self.sitl_streamrate()+1)
        self.set_streamrate(self.sitl_streamrate())
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
                                            self.log_name()))

    def start_test(self, description):
        self.progress("#")
        self.progress("########## %s  ##########" % description)
        self.progress("#")

    def try_symlink_tlog(self):
        self.buildlog = self.buildlogs_path(self.log_name() + "-test.tlog")
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

    def drain_all_pexpects(self):
        global expect_list
        for p in expect_list:
            util.pexpect_drain(p)

    def idle_hook(self, mav):
        """Called when waiting for a mavlink message."""
        self.drain_all_pexpects()

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

    def drain_mav_unparsed(self):
        count = 0
        tstart = time.time()
        while True:
            this = self.mav.recv(1000000)
            if len(this) == 0:
                break
            count += len(this)
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count/float(tdelta),)
        self.progress("Drained %u bytes from mav (%s).  These were unparsed." % (count, rate))

    def drain_mav(self, mav=None):
        if mav is None:
            mav = self.mav
        count = 0
        tstart = time.time()
        while mav.recv_match(blocking=False) is not None:
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
        """Get SITL time in seconds."""
        x = self.mav.messages.get("SYSTEM_TIME", None)
        if x is None:
            raise NotAchievedException("No cached time available")
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

    def save_wp(self, ch=7):
        """Trigger RC Aux to save waypoint."""
        self.set_rc(ch, 1000)
        self.wait_seconds(1)
        self.set_rc(ch, 2000)
        self.wait_seconds(1)
        self.set_rc(ch, 1000)
        self.wait_seconds(1)

    def clear_wp(self, ch=8):
        """Trigger RC Aux to clear waypoint."""
        self.progress("Clearing waypoints")
        self.set_rc(ch, 1000)
        self.wait_seconds(0.5)
        self.set_rc(ch, 2000)
        self.wait_seconds(0.5)
        self.set_rc(ch, 1000)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting 0 waypoints')

    def log_download(self, filename, timeout=360, upload_logs=False):
        """Download latest log."""
        self.wait_heartbeat()
        self.mavproxy.send("module load log\n")
        self.mavproxy.expect("Loaded module log")
        self.mavproxy.send("log list\n")
        self.mavproxy.expect("numLogs")
        self.wait_heartbeat()
        self.wait_heartbeat()
        self.mavproxy.send("set shownoise 0\n")
        self.mavproxy.send("log download latest %s\n" % filename)
        self.mavproxy.expect("Finished downloading", timeout=timeout)
        self.drain_mav_unparsed()
        self.wait_heartbeat()
        self.wait_heartbeat()
        if upload_logs and os.getenv("AUTOTEST_UPLOAD"):
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
        return wploader.count()

    def install_message_hook(self, hook):
        self.mav.message_hooks.append(hook)

    def remove_message_hook(self, hook):
        oldlen = len(self.mav.message_hooks)
        self.mav.message_hooks = list(filter(lambda x : x != hook,
                                        self.mav.message_hooks))
        if len(self.mav.message_hooks) == oldlen:
            raise NotAchievedException("Failed to remove hook")

    def rootdir(self):
        this_dir = os.path.dirname(__file__)
        return os.path.realpath(os.path.join(this_dir, "../.."))

    def mission_directory(self):
        return testdir

    def assert_mission_files_same(self, file1, file2):
        self.progress("Comparing (%s) and (%s)" % (file1, file2, ))
        f1 = open(file1)
        f2 = open(file2)
        for l1, l2 in zip(f1, f2):
            l1 = l1.rstrip("\r\n")
            l2 = l2.rstrip("\r\n")
            if l1 == l2:
                # e.g. the first "QGC WPL 110" line
                continue
            if re.match(r"0\s", l1):
                # home changes...
                continue
            l1 = l1.rstrip()
            l2 = l2.rstrip()
            fields1 = re.split(r"\s+", l1)
            fields2 = re.split(r"\s+", l2)
            # line = int(fields1[0])
            t = int(fields1[3]) # mission item type
            for (count, (i1, i2)) in enumerate(zip(fields1, fields2)):
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

    def assert_rally_files_same(self, file1, file2):
        self.progress("Comparing (%s) and (%s)" % (file1, file2, ))
        f1 = open(file1)
        f2 = open(file2)
        lines_f1 = f1.readlines()
        lines_f2 = f2.readlines()
        self.assert_rally_content_same(lines_f1, lines_f2)

    def assert_rally_filepath_content(self, file1, content):
        f1 = open(file1)
        lines_f1 = f1.readlines()
        lines_content = content.split("\n")
        print("lines content: %s" % str(lines_content))
        self.assert_rally_content_same(lines_f1, lines_content)

    def assert_rally_content_same(self, f1, f2):
        '''check each line in f1 matches one-to-one with f2'''
        for l1, l2 in zip(f1, f2):
            print("l1: %s" % l1)
            print("l2: %s" % l2)
            l1 = l1.rstrip("\n")
            l2 = l2.rstrip("\n")
            l1 = l1.rstrip("\r")
            l2 = l2.rstrip("\r")
            if l1 == l2:
                # e.g. the first "QGC WPL 110" line
                continue
            if re.match(r"0\s", l1):
                # home changes...
                continue
            l1 = l1.rstrip()
            l2 = l2.rstrip()
            print("al1: %s" % str(l1))
            print("al2: %s" % str(l2))
            fields1 = re.split(r"\s+", l1)
            fields2 = re.split(r"\s+", l2)
            # line = int(fields1[0])
            t = int(fields1[3]) # mission item type
            for (count, (i1, i2)) in enumerate(zip(fields1, fields2)):
                # if count == 2: # frame
                #     if t in [mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                #              mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                #              mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                #              mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                #              mavutil.mavlink.MAV_CMD_DO_JUMP,
                #              mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                #              ]:
                #         # ardupilot doesn't remember frame on these commands
                #         if int(i1) == 3:
                #             i1 = 0
                #         if int(i2) == 3:
                #             i2 = 0
                # if count == 6: # param 3
                #     if t in [mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME]:
                #         # ardupilot canonicalises this to -1 for ccw or 1 for cw.
                #         if float(i1) == 0:
                #             i1 = 1.0
                #         if float(i2) == 0:
                #             i2 = 1.0
                # if count == 7: # param 4
                #     if t == mavutil.mavlink.MAV_CMD_NAV_LAND:
                #         # ardupilot canonicalises "0" to "1" param 4 (yaw)
                #         if int(float(i1)) == 0:
                #             i1 = 1
                #         if int(float(i2)) == 0:
                #             i2 = 1
                if 0 <= count <= 3 or 11 <= count <= 11:
                    if int(i1) != int(i2):
                        raise ValueError("Rally points different: (%s vs %s) (%d vs %d) (count=%u)" %
                                         (l1, l2, int(i1), int(i2), count))  # NOCI
                    continue
                if 4 <= count <= 10:
                    f_i1 = float(i1)
                    f_i2 = float(i2)
                    delta = abs(f_i1 - f_i2)
                    max_allowed_delta = 0.000009
                    if delta > max_allowed_delta:
                        raise ValueError(
                            ("Rally has different (float) content: " +
                             "(%s vs %s) " +
                             "(%f vs %f) " +
                             "(%.10f) " +
                             "(count=%u)") %
                            (l1, l2,
                             f_i1, f_i2,
                             delta,
                             count)) # NOCI
                    continue
                raise ValueError("count %u not handled" % count)
        self.progress("Rally content same")

    def load_rally(self, filename):
        """Load rally points from a file to flight controller."""
        self.progress("Loading rally points (%s)" % filename)
        path = os.path.join(self.mission_directory(), filename)
        self.mavproxy.send('rally load %s\n' % path)
        self.mavproxy.expect("Loaded")

    def load_sample_mission(self):
        self.load_mission(self.sample_mission_filename())

    def load_mission(self, filename):
        """Load a mission from a file to flight controller."""
        self.progress("Loading mission (%s)" % filename)
        path = os.path.join(self.mission_directory(), filename)
        tstart = self.get_sim_time_cached()
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to do waypoint thing")
            # the following hack is to get around MAVProxy statustext deduping:
            while time.time() - self.last_wp_load < 3:
                self.progress("Waiting for MAVProxy de-dupe timer to expire")
                time.sleep(1)
            self.mavproxy.send('wp load %s\n' % path)
            self.mavproxy.expect('Loaded ([0-9]+) waypoints from')
            load_count = self.mavproxy.match.group(1)
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
            # warning: this assumes MAVProxy was started in the CWD!
            # on the autotest server we invoke autotest.py one-up from
            # the git root, like this:
            # timelimit 32000 APM/Tools/autotest/autotest.py --timeout=30000 > buildlogs/autotest-output.txt 2>&1
            # that means the MAVProxy log files are not reltopdir!
            saved_filepath = self.mavproxy.match.group(2)
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

        wploader = mavwp.MAVWPLoader()
        wploader.load(path)
        num_wp = wploader.count()
        if num_wp != int(status_have):
            raise ValueError("num_wp=%u != status_have=%u" %
            (num_wp, int(status_have)))
        if num_wp == 0:
            raise ValueError("No waypoints loaded?!")
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

    def rc_defaults(self):
        return {
            1: 1500,
            2: 1500,
            3: 1500,
            4: 1500,
            5: 1500,
            6: 1500,
            7: 1500,
            8: 1500,
            9: 1500,
            10: 1500,
            11: 1500,
            12: 1500,
            13: 1500,
            14: 1500,
            15: 1500,
            16: 1500,
        }

    def set_rc_from_map(self, _map, timeout=2000):
        copy = _map.copy()
        tstart = self.get_sim_time()
        while len(copy.keys()):
            if self.get_sim_time_cached() - tstart > timeout:
                raise SetRCTimeout("Failed to set RC channels")
            for chan in copy:
                value = copy[chan]
                self.send_set_rc(chan, value)
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            self.progress("m: %s" % m)
            new = dict()
            for chan in copy:
                chan_pwm = getattr(m, "chan" + str(chan) + "_raw")
                if chan_pwm != copy[chan]:
                    new[chan] = copy[chan]
            copy = new

    def set_rc_default(self):
        """Setup all simulated RC control to 1500."""
        _defaults = self.rc_defaults()
        self.set_rc_from_map(_defaults)

    def check_rc_defaults(self):
        """Ensure all rc outputs are at defaults"""
        _defaults = self.rc_defaults()
        m = self.mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=5)
        if m is None:
            raise NotAchievedException("No RC_CHANNELS messages?!")
        need_set = {}
        for chan in _defaults:
            default_value = _defaults[chan]
            current_value = getattr(m, "chan" + str(chan) + "_raw")
            if default_value != current_value:
                self.progress("chan=%u needs resetting is=%u want=%u" %
                              (chan, current_value, default_value))
                need_set[chan] = default_value
        self.set_rc_from_map(need_set)

    def send_set_rc(self, chan, pwm):
        self.mavproxy.send('rc %u %u\n' % (chan, pwm))

    def set_rc(self, chan, pwm, timeout=2000):
        """Setup a simulated RC control to a PWM value"""
        self.drain_mav()
        tstart = self.get_sim_time()
        wclock = time.time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.send_set_rc(chan, pwm)
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

    def location_offset_ne(self, location, north, east):
        '''move location in metres'''
        print("old: %f %f" % (location.lat, location.lng))
        (lat, lng) = mp_util.gps_offset(location.lat, location.lng, east, north)
        location.lat = lat
        location.lng = lng
        print("new: %f %f" % (location.lat, location.lng))

    def zero_throttle(self):
        """Set throttle to zero."""
        if self.is_rover():
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

    def get_stick_arming_channel(self):
        """Return the Rudder channel number as set in parameter."""
        raise ErrorException("Rudder parameter is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    def get_disarm_delay(self):
        """Return disarm delay value."""
        raise ErrorException("Disarm delay is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    def arming_test_mission(self):
        """Load arming test mission.
        This mission is used to allow to change mode to AUTO. For each vehicle
        it get an unlimited wait waypoint and the starting takeoff if needed."""
        return None

    def armed(self):
        """Return true if vehicle is armed and safetyoff"""
        return self.mav.motors_armed()

    def send_mavlink_arm_command(self):
        target_sysid = 1
        target_compid = 1
        self.mav.mav.command_long_send(
            target_sysid,
            target_compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1, # confirmation
            1,  # ARM
            0,
            0,
            0,
            0,
            0,
            0)

    def set_analog_rangefinder_parameters(self):
        self.set_parameter("RNGFND1_TYPE", 1)
        self.set_parameter("RNGFND1_MIN_CM", 0)
        self.set_parameter("RNGFND1_MAX_CM", 4000)
        self.set_parameter("RNGFND1_SCALING", 12.12)
        self.set_parameter("RNGFND1_PIN", 0)

    def arm_vehicle(self, timeout=20):
        """Arm vehicle with mavlink arm message."""
        self.progress("Arm motors with MAVLink cmd")
        self.drain_mav()
        tstart = self.get_sim_time()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        while self.get_sim_time_cached() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("Motors ARMED")
                return True
        raise AutoTestTimeoutException("Failed to ARM with mavlink")

    def disarm_vehicle(self, timeout=60, force=False):
        """Disarm vehicle with mavlink disarm message."""
        self.progress("Disarm motors with MAVLink cmd")
        self.drain_mav_unparsed()
        p2 = 0
        if force:
            p2 = 21196 # magic force disarm value
        tstart = self.get_sim_time()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     0,  # DISARM
                     p2,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        while self.get_sim_time_cached() - tstart < timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                self.progress("Motors DISARMED")
                return True
        raise AutoTestTimeoutException("Failed to DISARM with mavlink")

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
        self.set_output_to_max(self.get_stick_arming_channel())
        tstart = self.get_sim_time()
        while True:
            self.wait_heartbeat()
            tdelta = self.get_sim_time_cached() - tstart
            if self.mav.motors_armed():
                self.progress("MOTORS ARMED OK WITH RADIO")
                self.set_output_to_trim(self.get_stick_arming_channel())
                self.progress("Arm in %ss" % tdelta)  # TODO check arming time
                return True
            print("Not armed after %f seconds" % (tdelta))
            if tdelta > timeout:
                break
        self.progress("Failed to ARM with radio")
        self.set_output_to_trim(self.get_stick_arming_channel())
        return False

    def disarm_motors_with_rc_input(self, timeout=20):
        """Disarm motors with radio."""
        self.progress("Disarm motors with radio")
        self.set_output_to_min(self.get_stick_arming_channel())
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_delay = self.get_sim_time_cached() - tstart
                self.progress("MOTORS DISARMED OK WITH RADIO")
                self.set_output_to_trim(self.get_stick_arming_channel())
                self.progress("Disarm in %ss" % disarm_delay)  # TODO check disarming time
                return True
        self.progress("Failed to DISARM with radio")
        self.set_output_to_trim(self.get_stick_arming_channel())
        return False

    def arm_motors_with_switch(self, switch_chan, timeout=20):
        """Arm motors with switch."""
        self.progress("Arm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("MOTORS ARMED OK WITH SWITCH")
                return True
        self.progress("Failed to ARM with switch")
        return False

    def disarm_motors_with_switch(self, switch_chan, timeout=20):
        """Disarm motors with switch."""
        self.progress("Disarm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 1000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                self.progress("MOTORS DISARMED OK WITH SWITCH")
                return True
        self.progress("Failed to DISARM with switch")
        return False

    def disarm_wait(self, timeout=10):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not disarm")
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                return

    def wait_autodisarm_motors(self):
        """Wait for Autodisarm motors within disarm delay
        this feature is only available in copter (DISARM_DELAY) and plane (LAND_DISARMDELAY)."""
        self.progress("Wait autodisarming motors")
        disarm_delay = self.get_disarm_delay()
        tstart = self.get_sim_time()
        timeout = disarm_delay * 2
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_time = self.get_sim_time_cached() - tstart
                self.progress("MOTORS AUTODISARMED")
                self.progress("Autodisarm in %ss, expect less than %ss" % (disarm_time, disarm_delay))
                return disarm_time <= disarm_delay
        raise AutoTestTimeoutException("Failed to AUTODISARM")

    def set_autodisarm_delay(self, delay):
        """Set autodisarm delay"""
        raise ErrorException("Auto disarm is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

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

    def set_parameter(self, name, value, add_to_context=True, epsilon=0.0002, retries=10):
        """Set parameters from vehicle."""
        self.progress("Setting %s to %f" % (name, value))
        old_value = self.get_parameter(name, retry=2)
        for i in range(1, retries+2):
            self.mavproxy.send("param set %s %s\n" % (name, str(value)))
            returned_value = self.get_parameter(name)
            delta = float(value) - returned_value
            if abs(delta) < epsilon:
                # yes, near-enough-to-equal.
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
                try:
                    # sometimes race conditions garble the MAVProxy output
                    ret = float(self.mavproxy.match.group(1))
                except ValueError as e:
                    continue
                return ret
            except pexpect.TIMEOUT:
                pass
        raise NotAchievedException("Failed to retrieve parameter (%s)" % name)

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

    def sysid_thismav(self):
        return 1

    def run_cmd_int(self,
                    command,
                    p1,
                    p2,
                    p3,
                    p4,
                    x,
                    y,
                    z,
                    want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    timeout=10,
                    target_sysid=None,
                    target_compid=None,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT):

        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        if target_compid is None:
            target_compid = 1

        """Send a MAVLink command int."""
        self.mav.mav.command_int_send(target_sysid,
                                      target_compid,
                                      frame,
                                      command,
                                      0, # current
                                      0, # autocontinue
                                      p1,
                                      p2,
                                      p3,
                                      p4,
                                      x,
                                      y,
                                      z)
        self.run_cmd_get_ack(command, want_result, timeout)

    def send_cmd(self,
                 command,
                 p1,
                 p2,
                 p3,
                 p4,
                 p5,
                 p6,
                 p7,
                 target_sysid=None,
                 target_compid=None,
                 ):
        """Send a MAVLink command long."""
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        if target_compid is None:
            target_compid = 1
        self.mav.mav.command_long_send(target_sysid,
                                       target_compid,
                                       command,
                                       1,  # confirmation
                                       p1,
                                       p2,
                                       p3,
                                       p4,
                                       p5,
                                       p6,
                                       p7)

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
                target_sysid=None,
                target_compid=None,
                timeout=10,
                quiet=False):
        self.send_cmd(command,
                      p1,
                      p2,
                      p3,
                      p4,
                      p5,
                      p6,
                      p7,
                      target_sysid=target_sysid,
                      target_compid=target_compid,
        )
        self.run_cmd_get_ack(command, want_result, timeout, quiet=quiet)

    def run_cmd_get_ack(self, command, want_result, timeout, quiet=False):
        tstart = self.get_sim_time_cached()
        while True:
            delta_time = self.get_sim_time_cached() - tstart
            if delta_time > timeout:
                raise AutoTestTimeoutException("Did not get good COMMAND_ACK within %fs" % timeout)
            m = self.mav.recv_match(type='COMMAND_ACK',
                                    blocking=True,
                                    timeout=0.1)
            if m is None:
                continue
            if not quiet:
                self.progress("ACK received: %s (%fs)" % (str(m), delta_time))
            if m.command == command:
                if m.result != want_result:
                    raise ValueError("Expected %s got %s" % (want_result,
                                                             m.result))
                break

    #################################################
    # UTILITIES
    #################################################
    @staticmethod
    def longitude_scale(lat):
        ret = math.cos(lat * (math.radians(1)));
        print("scale=%f" % ret)
        return ret

    @staticmethod
    def get_distance(loc1, loc2):
        """Get ground distance between two locations."""
        return AutoTest.get_distance_accurate(loc1, loc2)
        # dlat = loc2.lat - loc1.lat
        # try:
        #     dlong = loc2.lng - loc1.lng
        # except AttributeError:
        #     dlong = loc2.lon - loc1.lon

        # return math.sqrt((dlat*dlat) + (dlong*dlong)*AutoTest.longitude_scale(loc2.lat)) * 1.113195e5

    @staticmethod
    def get_distance_accurate(loc1, loc2):
        """Get ground distance between two locations."""
        try:
            lon1 = loc1.lng
            lon2 = loc2.lng
        except AttributeError:
            lon1 = loc1.lon
            lon2 = loc2.lon

        return mp_util.gps_distance(loc1.lat, lon1, loc2.lat, lon2)

    @staticmethod
    def get_latlon_attr(loc, attrs):
        '''return any found latitude attribute from loc'''
        ret = None
        for attr in attrs:
            if hasattr(loc, attr):
                ret = getattr(loc, attr)
                break
        if ret is None:
            raise ValueError("None of %s in loc" % str(attrs))
        return ret

    @staticmethod
    def get_lat_attr(loc):
        '''return any found latitude attribute from loc'''
        return AutoTest.get_latlon_attr(loc, ["lat", "latitude"])

    @staticmethod
    def get_lon_attr(loc):
        '''return any found latitude attribute from loc'''
        return AutoTest.get_latlon_attr(loc, ["lng", "lon", "longitude"])

    @staticmethod
    def get_distance_int(loc1, loc2):
        """Get ground distance between two locations in the normal "int" form
        - lat/lon multiplied by 1e7"""
        loc1_lat = AutoTest.get_lat_attr(loc1)
        loc2_lat = AutoTest.get_lat_attr(loc2)
        loc1_lon = AutoTest.get_lon_attr(loc1)
        loc2_lon = AutoTest.get_lon_attr(loc2)

        return AutoTest.get_distance_accurate(
            mavutil.location(loc1_lat*1e-7, loc1_lon*1e-7),
            mavutil.location(loc2_lat*1e-7, loc2_lon*1e-7))

        dlat = loc2_lat - loc1_lat
        dlong = loc2_lon - loc1_lon

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

    def change_mode(self, mode, timeout=60):
        '''change vehicle flightmode'''
        self.progress("Changing mode to %s" % mode)
        self.mavproxy.send('mode %s\n' % mode)
        tstart = self.get_sim_time()
        while not self.mode_is(mode):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                    self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    self.get_sim_time_cached() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
            self.mavproxy.send('mode %s\n' % mode)
        self.progress("Got mode %s" % mode)

    def capable(self, capability):
        return self.get_autopilot_capabilities() & capability

    def assert_capability(self, capability):
        if not self.capable(capability):
            name = mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"][capability].name
            raise NotAchievedException("AutoPilot does not have capbility %s" % (name,))

    def assert_no_capability(self, capability):
        if self.capable(capability):
            name = mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"][capability].name
            raise NotAchievedException("AutoPilot has feature %s (when it shouln't)" % (name,))

    def get_autopilot_capabilities(self):
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
        if m is None:
            raise NotAchievedException("Did not get AUTOPILOT_VERSION")
        return m.capabilities

    def do_get_autopilot_capabilities(self):
        self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT)

    def get_mode_from_mode_mapping(self, mode):
        """Validate and return the mode number from a string or int."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None:
            mav_type = self.mav.messages['HEARTBEAT'].type
            mav_autopilot = self.mav.messages['HEARTBEAT'].autopilot
            raise ErrorException("No mode map for (mav_type=%s mav_autopilot=%s)" % (mav_type, mav_autopilot))
        if isinstance(mode, str):
            if mode in mode_map:
                return mode_map.get(mode)
        if mode in mode_map.values():
            return mode
        self.progress("Available modes '%s'" % mode_map)
        raise ErrorException("Unknown mode '%s'" % mode)

    def run_cmd_do_set_mode(self,
                            mode,
                            timeout=30,
                            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     base_mode,
                     custom_mode,
                     0,
                     0,
                     0,
                     0,
                     0,
                     want_result=want_result,
                     timeout=timeout
        )

    def do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message."""
        tstart = self.get_sim_time()
        want_custom_mode = self.get_mode_from_mode_mapping(mode)
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise AutoTestTimeoutException("Failed to change mode")
            self.run_cmd_do_set_mode(mode, timeout=10)
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=5)
            if m is None:
                raise ErrorException("Heartbeat not received")
            self.progress("Got mode=%u want=%u" % (m.custom_mode, want_custom_mode))
            if m.custom_mode == want_custom_mode:
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

    def reach_heading_manual(self, heading, turn_right=True):
        """Manually direct the vehicle to the target heading."""
        if self.is_copter():
            self.mavproxy.send('rc 4 1580\n')
            self.wait_heading(heading)
            self.set_rc(4, 1500)
        if self.is_plane():
            self.progress("NOT IMPLEMENTED")
        if self.is_rover():
            steering_pwm = 1700
            if not turn_right:
                steering_pwm = 1300
            self.mavproxy.send('rc 1 %u\n' % steering_pwm)
            self.mavproxy.send('rc 3 1550\n')
            self.wait_heading(heading)
            self.set_rc(3, 1500)
            self.set_rc(1, 1500)

    def assert_vehicle_location_is_at_startup_location(self, dist_max=1):
        here = self.mav.location()
        start_loc = self.sitl_start_location()
        dist = self.get_distance(here, start_loc)
        data = "dist=%f max=%f (here: %s start-loc: %s)" % (dist, dist_max, here, start_loc)

        if dist > dist_max:
            raise NotAchievedException("Far from startup location: %s" % data)
        self.progress("Close to startup location: %s" % data)

    def assert_simstate_location_is_at_startup_location(self, dist_max=1):
        simstate_loc = self.sim_location()
        start_loc = self.sitl_start_location()
        dist = self.get_distance(simstate_loc, start_loc)
        data = "dist=%f max=%f (simstate: %s start-loc: %s)" % (dist, dist_max, simstate_loc, start_loc)

        if dist > dist_max:
            raise NotAchievedException("simstate from startup location: %s" % data)
        self.progress("Simstate Close to startup location: %s" % data)

    def reach_distance_manual(self, distance):
        """Manually direct the vehicle to the target distance from home."""
        if self.is_copter():
            self.set_rc(2, 1350)
            self.wait_distance(distance, accuracy=5, timeout=60)
            self.set_rc(2, 1500)
        if self.is_plane():
            self.progress("NOT IMPLEMENTED")
        if self.is_rover():
            self.mavproxy.send('rc 3 1700\n')
            self.wait_distance(distance, accuracy=2)
            self.set_rc(3, 1500)

    def guided_achieve_heading(self, heading):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not achieve heading")
            self.run_cmd(mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                         heading,  # target angle
                         10,  # degrees/second
                         1,  # -1 is counter-clockwise, 1 clockwise
                         0,  # 1 for relative, 0 for absolute
                         0,  # p5
                         0,  # p6
                         0,  # p7
            )
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("heading=%d want=%d" % (m.heading, int(heading)))
            if m.heading == int(heading):
                return

    def do_set_relay(self, relay_num, on_off, timeout=10):
        """Set relay with a command long message."""
        self.progress("Set relay %d to %d" % (relay_num, on_off))
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                     relay_num,
                     on_off,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)

    def do_set_relay_mavproxy(self, relay_num, on_off):
        """Set relay with mavproxy."""
        self.progress("Set relay %d to %d" % (relay_num, on_off))
        self.mavproxy.send('module load relay\n')
        self.mavproxy.expect("Loaded module relay")
        self.mavproxy.send("relay set %d %d\n" % (relay_num, on_off))
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
        self.progress("Waiting for altitude between %.02f and %.02f" %
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
                self.progress("Wait Altitude: Cur:%.02f min_alt:%.02f climb_rate: %.02f"
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
                self.progress("Wait groundspeed %.3f, target:%.3f" %
                              (m.groundspeed, gs_min))
                last_print = self.get_sim_time_cached()
            if m.groundspeed >= gs_min and m.groundspeed <= gs_max:
                return True
        raise WaitGroundSpeedTimeout("Failed to attain groundspeed range")

    def wait_roll(self, roll, accuracy, timeout=30):
        """Wait for a given roll in degrees."""
        tstart = self.get_sim_time()
        self.progress("Waiting for roll of %d at %s" % (roll, time.ctime()))
        while self.get_sim_time_cached() < tstart + timeout:
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
        self.progress("Waiting for pitch of %.02f at %s" % (pitch, time.ctime()))
        while self.get_sim_time_cached() < tstart + timeout:
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
        self.progress("Waiting for heading %.02f with accuracy %.02f" %
                      (heading, accuracy))
        last_print_time = 0
        while True:
            now = self.get_sim_time_cached()
            if now - tstart >= timeout:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if now - last_print_time > 1:
                self.progress("Heading %d (want %f +- %f)" %
                              (m.heading, heading, accuracy))
                last_print_time = now
            if math.fabs(m.heading - heading) <= accuracy:
                self.progress("Attained heading %d" % heading)
                return True
        raise WaitHeadingTimeout("Failed to attain heading %d" % heading)

    def wait_distance(self, distance, accuracy=5, timeout=30):
        """Wait for flight of a given distance."""
        tstart = self.get_sim_time()
        start = self.mav.location()
        last_distance_message = 0
        while self.get_sim_time_cached() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(start, pos)
            if self.get_sim_time_cached() - last_distance_message >= 1:
                self.progress("Distance=%.2f meters want=%.2f" %
                              (delta, distance))
                last_distance_message = self.get_sim_time_cached()
            if math.fabs(delta - distance) <= accuracy:
                self.progress("Attained distance %.02f meters OK" % delta)
                return True
            if delta > (distance + accuracy):
                raise WaitDistanceTimeout("Failed distance - overshoot delta=%f dist=%f"
                                          % (delta, distance))
        raise WaitDistanceTimeout("Failed to attain distance %u" % distance)

    def wait_distance_to_location(self, location, distance_min, distance_max, timeout=30):
        last_distance_message = 0
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(location, pos)
            if self.get_sim_time_cached() - last_distance_message >= 1:
                self.progress("Distance=%.2f meters want <%.2f and >%.2f" %
                              (delta, distance_min, distance_max))
                last_distance_message = self.get_sim_time_cached()
            if (delta >= distance_min and delta <= distance_max):
                self.progress("Attained distance %.02f meters OK" % delta)
                return True
        raise WaitDistanceTimeout("Failed to attain distance <%.2f and >%.2f" %
                                  (distance_min, distance_max))

    def wait_servo_channel_value(self, channel, value, timeout=2, comparator=operator.eq):
        """wait for channel value comparison (default condition is equality)"""
        channel_field = "servo%u_raw" % channel
        opstring = ("%s" % comparator)[-3:-1]
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel value condition not met")
            m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field, None)
            self.progress("want SERVO_OUTPUT_RAW.%s=%u %s %u" %
                          (channel_field, m_value, opstring, value))
            if m_value is None:
                raise ValueError("message (%s) has no field %s" %
                                 (str(m), channel_field))
            if comparator(m_value, value):
                return

    def wait_rc_channel_value(self, channel, value, timeout=2):
        """wait for channel to hit value"""
        channel_field = "chan%u_raw" % channel
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel never achieved value")
            m = self.mav.recv_match(type='RC_CHANNELS',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field)
            self.progress("RC_CHANNELS.%s=%u want=%u" %
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
        closest = 10000000
        last = 0
        while self.get_sim_time() < tstart + timeout:
            pos = self.mav.location()
            delta = self.get_distance(loc, pos)
            if self.get_sim_time_cached() - last_distance_message >= 1:
                self.progress("Distance %.2f meters alt %.1f" % (delta, pos.alt))
                last_distance_message = self.get_sim_time_cached()
            if closest > delta:
                closest = delta
            last = delta
            if delta <= accuracy:
                height_delta = math.fabs(pos.alt - target_altitude)
                if height_accuracy != -1 and height_delta > height_accuracy:
                    continue
                self.progress("Reached location (%.2f meters)" % delta)
                return True
        raise WaitLocationTimeout("Failed to attain location (want=<%f) (closest=%f) (last=%f)" % (accuracy, closest, last))

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
                self.progress("WP %u (wp_dist=%u Alt=%.02f), current_wp: %u,"
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

    def mode_is(self, mode, cached=False, drain_mav=True):
        if not cached:
            self.wait_heartbeat(drain_mav=drain_mav)
        try:
            return self.get_mode_from_mode_mapping(self.mav.flightmode) == self.get_mode_from_mode_mapping(mode)
        except Exception as e:
            pass
        # assume this is a number....
        return self.mav.messages['HEARTBEAT'].custom_mode == mode

    def wait_mode(self, mode, timeout=60):
        """Wait for mode to change."""
        self.progress("Waiting for mode %s" % mode)
        tstart = self.get_sim_time()
        while not self.mode_is(mode, drain_mav=False):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                    self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    self.get_sim_time_cached() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)

    def wait_gps_sys_status_not_present_or_enabled_and_healthy(self, timeout=30):
        self.progress("Waiting for GPS health")
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("GPS status bits did not become good")
            m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            if m is None:
                continue
            if (not (m.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not present")
                return
            if (not (m.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not enabled")
                continue
            if (not (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not healthy")
                continue
            self.progress("GPS healthy")
            return

    def wait_ready_to_arm(self, timeout=None, require_absolute=True):
        # wait for EKF checks to pass
        self.progress("Waiting for ready to arm")
        self.wait_ekf_happy(timeout=timeout, require_absolute=require_absolute)
        if require_absolute:
            self.wait_gps_sys_status_not_present_or_enabled_and_healthy()

    def wait_heartbeat(self, drain_mav=True, *args, **x):
        '''as opposed to mav.wait_heartbeat, raises an exception on timeout'''
        if drain_mav:
            self.drain_mav()
        m = self.mav.wait_heartbeat(*args, **x)
        if m is None:
            raise AutoTestTimeoutException("Did not receive heartbeat")

    def wait_ekf_happy(self, timeout=30, require_absolute=True):
        """Wait for EKF to be happy"""

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            return True

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
        self.wait_ekf_flags(required_value, error_bits, timeout=timeout)

    def wait_ekf_flags(self, required_value, error_bits, timeout=30):
        self.progress("Waiting for EKF value %u" % required_value)
        last_print_time = 0
        tstart = self.get_sim_time()
        while timeout is None or self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            errors = current & error_bits
            everything_ok = (errors == 0 and
                             current & required_value == required_value)
            if everything_ok or self.get_sim_time_cached() - last_print_time > 1:
                self.progress("Wait EKF.flags: required:%u current:%u errors=%u" %
                              (required_value, current, errors))
                last_print_time = self.get_sim_time_cached()
            if everything_ok:
                self.progress("EKF Flags OK")
                return True
        raise AutoTestTimeoutException("Failed to get EKF.flags=%u" %
                                       required_value)

    def wait_gps_disable(self, timeout=30):
        """Disable GPS and wait for EKF to report the end of assistance from GPS."""
        self.set_parameter("SIM_GPS_DISABLE", 1)
        tstart = self.get_sim_time()

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            self.progress("GPS disable skipped")
            return 

        # all of these must NOT be set for arming NOT to happen:
        not_required_value = mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL
        self.progress("Waiting for EKF not having bits %u" % not_required_value)
        last_print_time = 0
        while timeout is None or self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            if self.get_sim_time_cached() - last_print_time > 1:
                self.progress("Wait EKF.flags: not required:%u current:%u" %
                              (not_required_value, current))
                last_print_time = self.get_sim_time_cached()
            if current & not_required_value != not_required_value:
                self.progress("GPS disable OK")
                return
        raise AutoTestTimeoutException("Failed to get EKF.flags=%u disabled" % not_required_value)

    def wait_text(self, *args, **kwargs):
        self.wait_statustext(*args, **kwargs)

    def wait_statustext(self, text, timeout=20, the_function=None):
        """Wait a specific STATUS_TEXT."""
        self.progress("Waiting for text : %s" % text.lower())
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            if the_function is not None:
                the_function()
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=0.1)
            if m is None:
                continue
            if text.lower() in m.text.lower():
                self.progress("Received expected text : %s" % m.text.lower())
                return True
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

    def show_test_timings_key_sorter(self, t):
        (k, v) = t
        return ((v, k))

    def show_test_timings(self):
        longest = 0
        for desc in self.test_timings.keys():
            if len(desc) > longest:
                longest = len(desc)
        for desc, test_time in sorted(self.test_timings.iteritems(),
                                 key=self.show_test_timings_key_sorter):
            fmt = "%" + str(longest) + "s: %.2fs"
            self.progress(fmt % (desc, test_time))

    def send_statustext(self, text):
        if sys.version_info.major >= 3 and not isinstance(text, bytes):
            text = bytes(text, "ascii")
        self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, text)

    def get_exception_stacktrace(self, e):
        if sys.version_info[0] >= 3:
            ret = "%s\n" % e
            ret += ''.join(traceback.format_exception(etype=type(e),
                                                      value=e,
                                                      tb=e.__traceback__))
            return ret
        return traceback.format_exc(e)

    def run_one_test(self, name, desc, test_function, interact=False):
        '''new-style run-one-test used by run_tests'''
        test_output_filename = self.buildlogs_path("%s-%s.txt" %
                                                   (self.log_name(), name))
        tee = TeeBoth(test_output_filename, 'w', self.mavproxy_logfile)

        prettyname = "%s (%s)" % (name, desc)
        self.send_statustext(prettyname)
        self.start_test(prettyname)

        self.context_push()

        start_time = time.time()

        ex = None
        try:
            self.check_rc_defaults()
            self.change_mode(self.default_mode())
            self.drain_mav()
            self.drain_all_pexpects()

            test_function()
        except Exception as e:
            self.progress("Exception caught: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.test_timings[desc] = time.time() - start_time
        self.context_pop()

        passed = True
        if ex is not None:
            passed = False

        self.wait_heartbeat()
        if self.armed() and not self.is_tracker():
            ex = ArmedAtEndOfTestException("Still armed at end of test")
            self.progress("Armed at end of test; force-rebooting SITL")
            self.disarm_vehicle(force=True)
            self.forced_post_test_sitl_reboots += 1
            self.progress("Force-resetting SITL")
            self.reboot_sitl() # that'll learn it
            passed = False

        if passed:
            self.progress('PASSED: "%s"' % prettyname)
        else:
            self.progress('FAILED: "%s": %s (see %s)' %
                          (prettyname, repr(ex), test_output_filename))
            self.fail_list.append((prettyname, ex, test_output_filename))
            if interact:
                self.progress("Starting MAVProxy interaction as directed")
                self.mavproxy.interact()

        self.clear_mission_using_mavproxy()

        tee.close()

    def check_test_syntax(self, test_file):
        """Check mistake on autotest function syntax."""
        self.start_test("Check for syntax mistake in autotest lambda")
        if not os.path.isfile(test_file):
            self.progress("File %s does not exist" % test_file)
        test_file = test_file.rstrip('c')
        try:
            with open(test_file) as f:
                # check for lambda: test_function without paranthesis
                faulty_strings = re.findall(r"lambda\s*:\s*\w+.\w+\s*\)", f.read())
                if faulty_strings:
                    desc = "Syntax error in autotest lambda at : \n"
                    for x in range(len(faulty_strings)):
                        desc += faulty_strings[x] + "\n"
                    raise ErrorException(desc)
        except ErrorException as msg:
            self.progress("FAILED: Check for syntax mistake in autotest lambda. \n" + str(msg))
            exit(1)
        self.progress("PASSED: Check for syntax mistake in autotest lambda")

    def uses_vicon(self):
        return False

    def defaults_filepath(self):
        return None

    def start_mavproxy(self):
        self.progress("Starting MAVProxy")
        self.mavproxy = util.start_MAVProxy_SITL(
            self.vehicleinfo_key(),
            logfile=self.mavproxy_logfile,
            options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        self.logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)
        self.try_symlink_tlog()

        self.progress("Waiting for Parameters")
        self.mavproxy.expect('Received [0-9]+ parameters')

    def init(self):
        """Initilialize autotest feature."""
        self.check_test_syntax(test_file=self.test_filepath())

        self.mavproxy_logfile = self.open_mavproxy_logfile()

        if self.frame is None:
            self.frame = self.default_frame()

        self.progress("Starting simulator")
        self.sitl = util.start_SITL(self.binary,
                                    breakpoints=self.breakpoints,
                                    disable_breakpoints=self.disable_breakpoints,
                                    defaults_file=self.defaults_filepath(),
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver,
                                    lldb=self.lldb,
                                    home=self.sitl_home(),
                                    model=self.frame,
                                    speedup=self.speedup,
                                    valgrind=self.valgrind,
                                    vicon=self.uses_vicon(),
                                    wipe=True,
                                    )

        self.start_mavproxy()

        self.progress("Starting MAVLink connection")
        self.get_mavlink_connection_going()

        self.apply_defaultfile_parameters()

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Ready to start testing!")

    def upload_using_mission_protocol(self, mission_type, items):
        '''mavlink2 required'''
        target_system = 1
        target_component = 1
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        len(items),
                                        mission_type)
        tstart = self.get_sim_time_cached()
        remaining_to_send = set(range(0, len(items)))
        sent = set()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("timeout uploading %s" % str(mission_type))
            if len(remaining_to_send) == 0:
                self.progress("All sent")
                break
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'],
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue

            if m.get_type() == 'MISSION_ACK':
                if (m.target_system == 255 and
                    m.target_component == 0 and
                    m.type == 1 and
                    m.mission_type == 0):
                    # this is just MAVProxy trying to screw us up
                    continue
                else:
                    raise NotAchievedException("Received unexpected mission ack %s" % str(m))

            self.progress("Handling request for item %u/%u" % (m.seq, len(items)-1))
            self.progress("Item (%s)" % str(items[m.seq]))
            if m.seq in sent:
                self.progress("received duplicate request for item %u" % m.seq)
                continue

            if m.seq not in remaining_to_send:
                raise NotAchievedException("received request for unknown item %u" % m.seq)

            if m.mission_type != mission_type:
                raise NotAchievedException("received request for item from wrong mission type")

            if items[m.seq].mission_type != mission_type:
                raise NotAchievedException("supplied item not of correct mission type")
            if items[m.seq].target_system != target_system:
                raise NotAchievedException("supplied item not of correct target system")
            if items[m.seq].target_component != target_component:
                raise NotAchievedException("supplied item not of correct target component")
            if items[m.seq].seq != m.seq:
                raise NotAchievedException("supplied item has incorrect sequence number (%u vs %u)" % (items[m.seq].seq, m.seq))

            items[m.seq].pack(self.mav.mav)
            self.mav.mav.send(items[m.seq])
            remaining_to_send.discard(m.seq)
            sent.add(m.seq)
        m = self.mav.recv_match(type='MISSION_ACK',
                                blocking=True,
                                timeout=1)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_ACK")
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission ack not of expected mission type")
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Mission upload failed (%s)" %
                                       (mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name),)
        self.progress("Upload of all %u items succeeded" % len(items))

    def download_using_mission_protocol(self, mission_type, verbose=False, timeout=10):
        '''mavlink2 required'''
        target_system = 1
        target_component = 1
        self.drain_mav_unparsed()
        self.progress("Sending mission_request_list")
        self.mav.mav.mission_request_list_send(target_system,
                                               target_component,
                                               mission_type)

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get MISSION_COUNT packet")
            m = self.mav.recv_match(blocking=True, timeout=0.1)
            if verbose:
                self.progress(str(m))
            if m.get_type() == 'MISSION_ACK':
                if m.target_system == 255 and m.target_component == 0:
                    # this was for MAVProxy
                    continue
                self.progress("Mission ACK: %s" % str(m))
                raise NotAchievedException("Received MISSION_ACK while waiting for MISSION_COUNT")
            if m.get_type() != 'MISSION_COUNT':
                continue
            if m is None:
                raise NotAchievedException("Did not get MISSION_COUNT response")
            if m.target_component != 250: # FIXME: constant?!
                continue
            if m.mission_type != mission_type:
                raise NotAchievedException("Mission count response of incorrect type")
            break

        items = []
        tstart = self.get_sim_time_cached()
        remaining_to_receive = set(range(0, m.count))
        next_to_request = 0
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("timeout downloading type=%s" %
                                           (mavutil.mavlink.enums["MAV_MISSION_TYPE"][mission_type].name))
            if len(remaining_to_receive) == 0:
                self.progress("All received")
                return items
            self.progress("Requesting item %u" % next_to_request)
            self.mav.mav.mission_request_int_send(target_system,
                                                  target_component,
                                                  next_to_request,
                                                  mission_type)
            m = self.mav.recv_match(type='MISSION_ITEM_INT',
                                    blocking=True,
                                    timeout=5,
                                    condition='MISSION_ITEM_INT.mission_type==%u' % mission_type)
            self.progress("Got (%s)" % str(m))
            if m is None:
                raise NotAchievedException("Did not receive MISSION_ITEM_INT")
            if m.mission_type != mission_type:
                raise NotAchievedException("Received waypoint of wrong type")
            if m.seq != next_to_request:
                raise NotAchievedException("Received waypoint is out of sequence")
            self.progress("Item %u OK" % m.seq)
            items.append(m)
            next_to_request += 1
            remaining_to_receive.discard(m.seq)

    def poll_home_position(self, quiet=False, timeout=30):
        old = self.mav.messages.get("HOME_POSITION", None)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Failed to poll home position")
            if not quiet:
                self.progress("Sending MAV_CMD_GET_HOME_POSITION")
            try:
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    quiet=quiet)
            except ValueError as e:
                continue
            m = self.mav.messages.get("HOME_POSITION", None)
            if m is None:
                continue
            if old is None:
                break
            if m._timestamp != old._timestamp:
                break
        return m

    def distance_to_home(self, use_cached_home=False):
        m = self.mav.messages.get("HOME_POSITION", None)
        if use_cached_home is False or m is None:
            m = self.poll_home_position(quiet=True)
        here = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                   blocking=True)
        return self.get_distance_int(m, here)

    def home_position_as_mav_location(self):
        m = self.poll_home_position()
        return mavutil.location(m.latitude*1.0e-7, m.longitude*1.0e-7, m.altitude*1.0e-3, 0)

    def offset_location_ne(self, location, metres_north, metres_east):
        '''return a new location offset from passed-in location'''
        (target_lat, target_lng) = mavextra.gps_offset(location.lat,
                                                       location.lng,
                                                       metres_east,
                                                       metres_north)
        return mavutil.location(target_lat,
                                target_lng,
                                location.alt,
                                location.heading)

    def monitor_groundspeed(self, want, tolerance=0.5, timeout=5):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if m.groundspeed > want+tolerance:
                raise NotAchievedException("Too fast (%f > %f)" %
                                           (m.groundspeed, want))
            if m.groundspeed < want-tolerance:
                raise NotAchievedException("Too slow (%f < %f)" %
                                           (m.groundspeed, want))
            self.progress("GroundSpeed OK (got=%f) (want=%f)" %
                          (m.groundspeed, want))

    def fly_test_set_home(self):
        if self.is_tracker():
            # tracker starts armed...
            self.disarm_vehicle(force=True)
        self.reboot_sitl()

        # HOME_POSITION is used as a surrogate for origin until we
        # start emitting GPS_GLOBAL_ORIGIN
        orig_home = self.poll_home_position()
        if orig_home is None:
            raise AutoTestTimeoutException()
        self.progress("Original home: %s" % str(orig_home))
        # original home should be close to SITL home...
        start_loc = self.sitl_start_location()
        self.progress("SITL start loc: %s" % str(start_loc))
        if abs(orig_home.latitude * 1.0e-7 - start_loc.lat) > 0.0000001:
            raise ValueError("homes differ in lat")
        if abs(orig_home.longitude * 1.0e-7 - start_loc.lng) > 0.0000001:
            raise ValueError("homes differ in lon")
        if self.is_rover():
            self.progress("### Rover skipping altitude check unti position fixes in")
        else:
            home_alt_m = orig_home.altitude * 1.0e-3
            if abs(home_alt_m - start_loc.alt) > 2: # metres
                raise ValueError("homes differ in alt got=%fm want=%fm" %
                                 (home_alt_m, start_loc.alt))
        new_x = orig_home.latitude + 1000
        new_y = orig_home.longitude + 2000
        new_z = orig_home.altitude + 300000 # 300 metres
        print("new home: %s %s %s" % (str(new_x), str(new_y), str(new_z)))
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         new_x,
                         new_y,
                         new_z/1000.0, # mm => m
                         )

        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        got_home_latitude = home.latitude
        got_home_longitude = home.longitude
        got_home_altitude = home.altitude
        if (got_home_latitude != new_x or
            got_home_longitude != new_y or
            abs(got_home_altitude - new_z) > 100): # float-conversion issues
            self.reboot_sitl()
            raise NotAchievedException(
                "Home mismatch got=(%f, %f, %f) set=(%f, %f, %f)" %
                (got_home_latitude, got_home_longitude, got_home_altitude,
                new_x, new_y, new_z))

        self.progress("monitoring home to ensure it doesn't drift at all")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            home = self.poll_home_position(quiet=True)
            self.progress("home: %s" % str(home))
            if (home.latitude != got_home_latitude or
                home.longitude != got_home_longitude or
                home.altitude != got_home_altitude): # float-conversion issues
                self.reboot_sitl()
                raise NotAchievedException("home is drifting")

        self.progress("Waiting for EKF to start")
        self.wait_ready_to_arm()
        self.progress("now use lat=0, lon=0 to reset home to current location")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         0, # lat
                         0, # lon
                         new_z/1000.0, # mm => m
                         )
        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        if self.distance_to_home(use_cached_home=True) > 1:
            raise NotAchievedException("Setting home to current location did not work")

        self.progress("Setting home elsewhere again")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         new_x,
                         new_y,
                         new_z/1000.0, # mm => m
                         )
        if self.distance_to_home() < 10:
            raise NotAchievedException("Setting home to location did not work")

        self.progress("use param1=1 to reset home to current location")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         1, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         37, # lat
                         21, # lon
                         new_z/1000.0, # mm => m
                         )
        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        if self.distance_to_home() > 1:
            raise NotAchievedException("Setting home to current location did not work")

        if self.is_tracker():
            # tracker starts armed...
            self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def test_dataflash_over_mavlink(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("LOG_BACKEND_TYPE", 2)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.mavproxy.send('arm throttle\n')
            self.mavproxy.expect('PreArm: Logging failed')
            self.mavproxy.send("module load dataflash_logger\n")
            self.mavproxy.send("dataflash_logger set verbose 1\n")
            self.mavproxy.expect('logging started')
            self.mavproxy.send("dataflash_logger set verbose 0\n")
            self.delay_sim_time(1)
            self.drain_mav() # hopefully draining COMMAND_ACK from that failed arm
            self.arm_vehicle()
            tstart = self.get_sim_time()
            last_status = 0
            while True:
                now = self.get_sim_time()
                if now - tstart > 60:
                    break
                if now - last_status > 5:
                    last_status = now
                    self.mavproxy.send('dataflash_logger status\n')
                    # seen on autotest: Active Rate(3s):97.790kB/s Block:164 Missing:0 Fixed:0 Abandoned:0
                    self.mavproxy.expect("Active Rate\([0-9]s\):([0-9]+[.][0-9]+)")
                    rate = self.mavproxy.match.group(1)
                    self.progress("Rate: %f" % float(rate))
                    if rate < 50:
                        raise NotAchievedException("Exceptionally low transfer rate")
            self.disarm_vehicle()
        except Exception as e:
            self.disarm_vehicle()
            self.progress("Exception (%s) caught" % str(e))
            ex = e
        self.context_pop()
        self.mavproxy.send("module unload dataflash_logger\n")
        self.mavproxy.expect("Unloaded module dataflash_logger")
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_dataflash_sitl(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("LOG_BACKEND_TYPE", 5)
            self.reboot_sitl()
            self.drain_mav() # hopefully draining COMMAND_ACK from that failed arm
            self.mavproxy.send("module load log\n")
#            self.mavproxy.expect("Loaded module log\n")
            self.mavproxy.send("log erase\n")
            self.mavproxy.send("log list\n")
            self.mavproxy.expect("Log 1  numLogs 1 lastLog 1 size")
            self.reboot_sitl()
            self.mavproxy.send("log list\n")
            self.mavproxy.expect("Log 1  numLogs 2 lastLog 2 size")
        except Exception as e:
            self.progress("Exception (%s) caught" % str(e))
            ex = e
        self.mavproxy.send("module unload log\n")
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_arm_feature(self):
        """Common feature to test."""
        # TEST ARMING/DISARM
        if self.get_parameter("ARMING_CHECK") != 1.0 and not self.is_sub():
            raise ValueError("Arming check should be 1")
        if not self.is_sub() and not self.is_tracker():
            self.set_parameter("ARMING_RUDDER", 2)  # allow arm and disarm with rudder on first tests
        if self.is_copter():
            interlock_channel = 8  # Plane got flighmode_ch on channel 8
            if not self.is_heli():  # heli don't need interlock option
                interlock_channel = 9
                self.set_parameter("RC%u_OPTION" % interlock_channel, 32)
            self.set_rc(interlock_channel, 1000)
        self.zero_throttle()
        # Disable auto disarm for next tests
        # Rover and Sub don't have auto disarm
        if self.is_copter() or self.is_plane():
            self.set_autodisarm_delay(0)
        self.start_subtest("Test normal arm and disarm features")
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

        if not self.is_sub():
            self.start_subtest("Test arm with rc input")
            if not self.arm_motors_with_rc_input():
                raise NotAchievedException("Failed to arm with RC input")
            self.progress("disarm with rc input")
            if self.is_balancebot():
                self.progress("balancebot can't disarm with RC input")
                self.disarm_vehicle()
            else:
                if not self.disarm_motors_with_rc_input():
                    raise NotAchievedException("Failed to disarm with RC input")

            self.start_subtest("Test arm and disarm with switch")
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

            if self.is_copter():
                self.start_subtest("Test arming failure with throttle too high")
                self.set_rc(3, 1800)
                try:
                    if self.arm_vehicle():
                        raise NotAchievedException("Armed when throttle too high")
                except ValueError:
                    pass
                if self.arm_motors_with_rc_input():
                    raise NotAchievedException(
                        "Armed via RC when throttle too high")
                if self.arm_motors_with_switch(arming_switch):
                    raise NotAchievedException("Armed via RC when switch too high")
                self.zero_throttle()
                self.set_rc(arming_switch, 1000)

            # Sub doesn't have 'stick commands'
            self.start_subtest("Test arming failure with ARMING_RUDDER=0")
            self.set_parameter("ARMING_RUDDER", 0)
            if self.arm_motors_with_rc_input():
                raise NotAchievedException(
                    "Armed with rudder when ARMING_RUDDER=0")
            self.start_subtest("Test disarming failure with ARMING_RUDDER=0")
            self.arm_vehicle()
            if self.disarm_motors_with_rc_input():
                raise NotAchievedException(
                    "Disarmed with rudder when ARMING_RUDDER=0")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.start_subtest("Test disarming failure with ARMING_RUDDER=1")
            self.set_parameter("ARMING_RUDDER", 1)
            self.arm_vehicle()
            if self.disarm_motors_with_rc_input():
                raise NotAchievedException(
                    "Disarmed with rudder with ARMING_RUDDER=1")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.set_parameter("ARMING_RUDDER", 2)

            if self.is_copter():
                self.start_subtest("Test arming failure with interlock enabled")
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
                if self.is_heli():
                    self.start_subtest("Test motor interlock enable can't be set while disarmed")
                    self.set_rc(interlock_channel, 2000)
                    channel_field = "servo%u_raw" % interlock_channel
                    interlock_value = self.get_parameter("SERVO%u_MIN" % interlock_channel)
                    tstart = self.get_sim_time()
                    while True:
                        if self.get_sim_time_cached() - tstart > 20:
                            self.set_rc(interlock_channel, 1000)
                            break # success!
                        m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                                blocking=True,
                                                timeout=2)
                        if m is None:
                            continue
                        m_value = getattr(m, channel_field, None)
                        if m_value is None:
                            self.set_rc(interlock_channel, 1000)
                            raise ValueError("Message has no %s field" %
                                             channel_field)
                        self.progress("SERVO_OUTPUT_RAW.%s=%u want=%u" %
                                      (channel_field, m_value, interlock_value))
                        if m_value != interlock_value:
                            self.set_rc(interlock_channel, 1000)
                            raise NotAchievedException("Motor interlock was changed while disarmed")
                self.set_rc(interlock_channel, 1000)

        self.start_subtest("Test all mode arming")
        if self.arming_test_mission() is not None:
            self.load_mission(self.arming_test_mission())

        for mode in self.mav.mode_mapping():
            self.drain_mav()
            self.start_subtest("Mode : %s" % mode)
            if mode == "FOLLOW":
                self.set_parameter("FOLL_ENABLE", 1)
            if mode in self.get_normal_armable_modes_list():
                self.progress("Armable mode : %s" % mode)
                self.change_mode(mode)
                self.arm_vehicle()
                if not self.disarm_vehicle():
                    raise NotAchievedException("Failed to DISARM")
                self.progress("PASS arm mode : %s" % mode)
            if mode in self.get_not_armable_mode_list():
                if mode in self.get_not_disarmed_settable_modes_list():
                    self.progress("Not settable mode : %s" % mode)
                    try:
                        self.change_mode(mode)
                    except AutoTestTimeoutException:
                        self.progress("PASS not able to set mode : %s disarmed" % mode)
                    except ValueError:
                        self.progress("PASS not able to set mode : %s disarmed" % mode)
                else:
                    self.progress("Not armable mode : %s" % mode)
                    self.change_mode(mode)
                    self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 1,  # ARM
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 want_result=mavutil.mavlink.MAV_RESULT_FAILED
                                 )
                self.progress("PASS not able to arm in mode : %s" % mode)
            if mode in self.get_position_armable_modes_list():
                self.progress("Armable mode needing Position : %s" % mode)
                self.wait_ekf_happy()
                self.change_mode(mode)
                self.arm_vehicle()
                self.wait_heartbeat()
                if not self.disarm_vehicle():
                    raise NotAchievedException("Failed to DISARM")
                self.progress("PASS arm mode : %s" % mode)
                self.progress("Not armable mode without Position : %s" % mode)
                self.wait_gps_disable()
                self.change_mode(mode)
                self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             1,  # ARM
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             want_result=mavutil.mavlink.MAV_RESULT_FAILED
                             )
                self.set_parameter("SIM_GPS_DISABLE", 0)
                self.wait_ekf_happy() # EKF may stay unhappy for a while
                self.progress("PASS not able to arm without Position in mode : %s" % mode)
            if mode in self.get_no_position_not_settable_modes_list():
                self.progress("Setting mode need Position : %s" % mode)
                self.wait_ekf_happy()
                self.wait_gps_disable()
                try:
                    self.change_mode(mode)
                except AutoTestTimeoutException:
                    self.set_parameter("SIM_GPS_DISABLE", 0)
                    self.progress("PASS not able to set mode without Position : %s" % mode)
                except ValueError:
                    self.set_parameter("SIM_GPS_DISABLE", 0)
                    self.progress("PASS not able to set mode without Position : %s" % mode)
            if mode == "FOLLOW":
                self.set_parameter("FOLL_ENABLE", 0)
        self.change_mode(self.default_mode())
        if self.armed():
            if not self.disarm_vehicle():
                raise NotAchievedException("Failed to DISARM")
        self.progress("ALL PASS")
    # TODO : Test arming magic;

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

    def set_message_rate_hz(self, id, rate_hz):
        '''set a message rate in Hz; 0 for original, -1 to disable'''
        if rate_hz == 0 or rate_hz == -1:
            set_interval = rate_hz
        else:
            set_interval = self.rate_to_interval_us(rate_hz)
        self.run_cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     id,
                     set_interval,
                     0,
                     0,
                     0,
                     0,
                     0)

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
                self.drain_mav()
                rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
                self.progress("Want=%u got=%u" % (want_rate, rate))
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

            self.drain_mav()

            self.progress("Resetting CAMERA_FEEDBACK rate to zero")
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK, -1)

            non_existant_id = 145
            self.mavproxy.send("long GET_MESSAGE_INTERVAL %u\n" %
                               (non_existant_id))
            m = self.mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)
            if m.interval_us != 0:
                raise NotAchievedException("Supposed to get 0 back for unsupported stream")
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
            if m.result != mavutil.mavlink.MAV_RESULT_FAILED:
                raise NotAchievedException("Getting rate of unsupported message is a failure")

            sr = self.sitl_streamrate()
            self.mavproxy.send("set streamrate %u\n" % sr)

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            # tell MAVProxy to start stuffing around with the rates:
            sr = self.sitl_streamrate()
            self.mavproxy.send("set streamrate %u\n" % sr)
            raise e

    def test_request_message(self, timeout=60):
        rate = round(self.get_message_rate("CAMERA_FEEDBACK", 10))
        if rate != 0:
            raise PreconditionFailedException("Receving camera feedback")
        # temporarily use a constant in place of
        # mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE until we have a
        # pymavlink release:
        self.run_cmd(512,
                     180,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        m = self.mav.recv_match(type='CAMERA_FEEDBACK', blocking=True, timeout=1)
        if m is None:
            raise NotAchievedException("Requested CAMERA_FEEDBACK did not arrive")

    def clear_mission(self, mission_type, target_system=1, target_component=1):
        '''clear mision_type from autopilot.  Note that this does NOT actually
        send a MISSION_CLEAR_ALL message
        '''
        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_ALL:
            # recurse
            if not self.is_tracker() and not self.is_plane():
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            if not self.is_sub() and not self.is_tracker():
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.last_wp_load = time.time()
            return

        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        0,
                                        mission_type)
        m = self.mav.recv_match(type='MISSION_ACK',
                                blocking=True,
                                timeout=5)
        if m is None:
            raise NotAchievedException("Expected ACK for clearing mission")
        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("ACK not targetted at correct system want=%u got=%u" %
                                       (self.mav.mav.srcSystem, m.target_system))
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("ACK not targetted at correct component want=%u got=%u" %
                                       (self.mav.mav.srcComponent, m.target_component))
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Expected MAV_MISSION_ACCEPTED got %s" %
                                       (mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name,))

        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            self.last_wp_load = time.time()


    def clear_fence_using_mavproxy(self, timeout=10):
        self.mavproxy.send("fence clear\n")
        tstart = self.get_sim_time_cached()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("FENCE_TOTAL did not go to zero")
            if self.get_parameter("FENCE_TOTAL") == 0:
                break

    def clear_fence(self):
        self.clear_fence_using_mavproxy()

    def clear_mission_using_mavproxy(self):
        self.mavproxy.send("wp clear\n")
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        num_wp = mavwp.MAVWPLoader().count()
        if num_wp != 0:
            raise NotAchievedException("Failed to clear mission")
        self.last_wp_load = time.time()

    def test_sensor_config_error_loop(self):
        '''test the sensor config error loop works and that parameter sets are persistent'''
        parameter_name = "SERVO8_MIN"
        old_parameter_value = self.get_parameter(parameter_name)
        old_sim_baro_count = self.get_parameter("SIM_BARO_COUNT")
        new_parameter_value = old_parameter_value + 5
        ex = None
        try:
            self.set_parameter("STAT_BOOTCNT", 0)
            self.set_parameter("SIM_BARO_COUNT", 0)

            if self.is_tracker():
                # starts armed...
                self.progress("Disarming tracker")
                self.disarm_vehicle(force=True)

            self.reboot_sitl(required_bootcount=1);
            self.progress("Waiting for 'Check BRD_TYPE'")
            self.mavproxy.expect("Check BRD_TYPE");
            self.progress("Setting %s to %f" % (parameter_name, new_parameter_value))
            self.set_parameter(parameter_name, new_parameter_value)
        except Exception as e:
            ex = e

        self.progress("Resetting SIM_BARO_COUNT")
        self.set_parameter("SIM_BARO_COUNT", old_sim_baro_count)

        if self.is_tracker():
            # starts armed...
            self.progress("Disarming tracker")
            self.disarm_vehicle(force=True)

        self.progress("Calling reboot-sitl ")
        self.reboot_sitl(required_bootcount=2);

        if ex is not None:
            raise ex

        if self.get_parameter(parameter_name) != new_parameter_value:
            raise NotAchievedException("Parameter value did not stick")

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
        self.progress("Waiting for ready to arm")
        self.wait_ready_to_arm()
        self.progress("Test gripper with RC9_OPTION")
        self.progress("Releasing load")
        # non strict string matching because of catching text issue....
        self.wait_text("Gripper load releas", the_function=lambda: self.send_set_rc(9, 1000))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb", the_function=lambda: self.send_set_rc(9, 2000))
        self.progress("Releasing load")
        self.wait_text("Gripper load releas", the_function=lambda: self.send_set_rc(9, 1000))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb", the_function=lambda: self.send_set_rc(9, 2000))
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

    def is_copter(self):
        return False

    def is_sub(self):
        return False

    def is_plane(self):
        return False

    def is_rover(self):
        return False

    def is_balancebot(self):
        return False

    def is_heli(self):
        return False

    def is_tracker(self):
        return False

    def initial_mode(self):
        '''return mode vehicle should start in with no RC inputs set'''
        return None

    def initial_mode_switch_mode(self):
        '''return mode vehicle should start in with default RC inputs set'''
        return None

    def upload_fences_from_locations(self,
                                     vertex_type,
                                     list_of_list_of_locs,
                                     target_system=1,
                                     target_component=1):
        seq = 0
        items = []
        for locs in list_of_list_of_locs:
            if type(locs) == dict:
                # circular fence
                if vertex_type == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    v = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
                else:
                    v = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    v,
                    0, # current
                    0, # autocontinue
                    locs["radius"], # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(locs["loc"].lat *1e7), # latitude
                    int(locs["loc"].lng *1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                seq += 1
                items.append(item)
                continue
            count = len(locs)
            for loc in locs:
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    vertex_type,
                    0, # current
                    0, # autocontinue
                    count, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(loc.lat *1e7), # latitude
                    int(loc.lng *1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                seq += 1
                items.append(item)

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)

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

    def start_subsubtest(self, description):
        self.progress(".")
        self.progress(".......... %s  .........." % description)
        self.progress(".")

    def end_subtest(self, description):
        '''TODO: sanity checks?'''
        pass

    def end_subsubtest(self, description):
        '''TODO: sanity checks?'''
        pass

    def test_skipped(self, test, reason):
        (name, desc, func) = test
        self.progress("##### %s is skipped: %s" % (name, reason))
        self.skip_list.append((test, reason))

    def last_onboard_log(self):
        '''return number of last onboard log'''
        self.mavproxy.send("module load log\n")
        self.mavproxy.expect("Loaded module log")
        self.mavproxy.send("log list\n")
        self.mavproxy.expect("lastLog ([0-9]+)")
        num_log = int(self.mavproxy.match.group(1))
        self.mavproxy.send("module unload log\n")
        self.mavproxy.expect("Unloaded module log")
        return num_log

    def current_onboard_log_filepath(self):
        '''return filepath to currently open dataflash log'''
        return os.path.join("logs/%08u.BIN" % self.last_onboard_log())

    def dfreader_for_current_onboard_log(self):
        return DFReader.DFReader_binary(self.current_onboard_log_filepath(),
                                        zero_time_base=True);

    def current_onboard_log_contains_message(self, messagetype):
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type=messagetype)
        print("m=%s" % str(m))
        return m is not None

    def run_tests(self, tests):
        """Autotest vehicle in SITL."""
        if self.run_tests_called:
            raise ValueError("run_tests called twice")
        self.run_tests_called = True

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
            if not self.is_tracker(): # FIXME - more to the point, fix Tracker's mission handling
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)

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

    def test_parameters(self):
        '''general small tests for parameter system'''
        if self.is_tracker():
            # uses CMD_TOTAL not MIS_TOTAL, and it's in a scalr not a
            # group and it's generally all bad.
            return
        self.start_subtest("Ensure GCS is not be able to set MIS_TOTAL")
        old_mt = self.get_parameter("MIS_TOTAL")
        ex = None
        try:
            self.set_parameter("MIS_TOTAL", 17, retries=0)
        except ValueError as e:
            ex = e
        if ex is None:
            raise NotAchievedException("Set parameter when I shouldn't have")
        if old_mt != self.get_parameter("MIS_TOTAL"):
            raise NotAchievedException("Total has changed")

        self.start_subtest("Ensure GCS is able to set other MIS_ parameters")
        self.set_parameter("MIS_OPTIONS", 1)
        if self.get_parameter("MIS_OPTIONS") != 1:
            raise NotAchievedException("Failed to set MIS_OPTIONS")

    def disabled_tests(self):
        return {}

    def test_parameter_checks_poscontrol(self, param_prefix):
        self.wait_ready_to_arm()
        self.context_push()
        self.set_parameter("%s_POSXY_P" % param_prefix, -1)
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=4,
                     want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.context_pop()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=4,
                     want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.disarm_vehicle()

    def test_pid_tuning(self):
        self.progress("making sure we're not getting PID_TUNING messages")
        m = self.mav.recv_match(type='PID_TUNING', blocking=True, timeout=5)
        if m is not None:
            raise PreconditionFailedException("Receiving PID_TUNING already")
        self.set_parameter("GCS_PID_MASK", 1)
        self.progress("making sure we are now getting PID_TUNING messages")
        m = self.mav.recv_match(type='PID_TUNING', blocking=True, timeout=5)
        if m is None:
            raise PreconditionFailedException("Did not start to get PID_TUNING message")

    def sample_mission_filename(self):
        return "flaps.txt"

    def test_advanced_failsafe(self):
        self.context_push()
        ex = None
        try:
            self.drain_mav()
            self.assert_no_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)
            self.set_parameter("AFS_ENABLE", 1)
            self.drain_mav()
            self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)
            self.set_parameter("AFS_TERM_ACTION", 42)
            self.load_sample_mission()
            self.change_mode("AUTO") # must go to auto for AFS to latch on
            self.change_mode("MANUAL")
            self.start_subtest("RC Failure")
            self.set_parameter("AFS_RC_FAIL_TIME", 1)
            self.set_parameter("SIM_RC_FAIL", 1)
            self.wait_statustext("Terminating due to RC failure")
            self.set_parameter("AFS_RC_FAIL_TIME", 0)
            self.set_parameter("SIM_RC_FAIL", 0)
            self.set_parameter("AFS_TERMINATE", 0)

            if not self.is_plane():
                # plane requires a polygon fence...
                self.start_subtest("Altitude Limit breach")
                self.set_parameter("AFS_AMSL_LIMIT", 100)
                self.mavproxy.send("fence enable\n")
                self.wait_statustext("Terminating due to fence breach")
                self.set_parameter("AFS_AMSL_LIMIT", 0)
                self.set_parameter("AFS_TERMINATE", 0)
                self.mavproxy.send("fence disable\n")

            self.start_subtest("GPS Failure")
            self.set_parameter("AFS_MAX_GPS_LOSS", 1)
            self.set_parameter("SIM_GPS_DISABLE", 1)
            self.wait_statustext("AFS State: GPS_LOSS")
            self.set_parameter("SIM_GPS_DISABLE", 0)
            self.set_parameter("AFS_MAX_GPS_LOSS", 0)
            self.set_parameter("AFS_TERMINATE", 0)

            self.send_cmd(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                          1,  # terminate
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
            )
            self.wait_statustext("Terminating due to GCS request")

        except Exception as e:
            ex = e
        self.mavproxy.send("fence disable\n")
        if ex is not None:
            raise ex

    def tests(self):
        return [
            ("PIDTuning",
             "Test PID Tuning",
             self.test_pid_tuning),

            ("ArmFeatures", "Arm features", self.test_arm_feature),

            ("SetHome",
            "Test Set Home",
             self.fly_test_set_home),

            ("SensorConfigErrorLoop",
             "Test Sensor Config Error Loop",
             self.test_sensor_config_error_loop),

            ("Parameters",
             "Test Parameter Set/Get",
             self.test_parameters),
        ]

    def post_tests_announcements(self):
        if self._show_test_timings:
            self.show_test_timings()
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
