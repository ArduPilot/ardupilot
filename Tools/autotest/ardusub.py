#!/usr/bin/env python

# Dive ArduSub in SITL
from __future__ import print_function
import os
import shutil

import pexpect
from pymavlink import mavutil

from common import *
from pysim import util
from pysim import vehicleinfo

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(33.810313, -118.393867, 0, 185)


class AutotestSub(Autotest):
    def __init__(self, binary, viewerip=None, use_map=False, valgrind=False, gdb=False, speedup=10, frame=None, params=None, gdbserver=False):
        super(AutotestSub, self).__init__()
        self.binary = binary
        self.options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
        self.viewerip = viewerip
        self.use_map = use_map
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver

        self.home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
        self.homeloc = None
        self.speedup = speedup
        self.speedup_default = 10

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.frame is None:
            self.frame = 'vectored'

        if self.viewerip:
            self.options += " --out=%s:14550" % self.viewerip
        if self.use_map:
            self.options += ' --map'

        self.sitl = util.start_SITL(self.binary, wipe=True, model=self.frame, home=self.home,
                                    speedup=self.speedup_default)
        self.mavproxy = util.start_MAVProxy_SITL('ArduSub')

        progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        # setup test parameters
        vinfo = vehicleinfo.VehicleInfo()
        if self.params is None:
            self.params = vinfo.options["ArduSub"]["frames"][self.frame]["default_params_filename"]
        if not isinstance(self.params, list):
            self.params = [self.params]
        for x in self.params:
            self.mavproxy.send("param load %s\n" % os.path.join(testdir, x))
            self.mavproxy.expect('Loaded [0-9]+ parameters')
        self.set_parameter('LOG_REPLAY', 1)
        self.set_parameter('LOG_DISARMED', 1)
        progress("RELOADING SITL WITH NEW PARAMETERS")

        # restart with new parms
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        self.sitl = util.start_SITL(self.binary, model=self.frame, home=self.home, speedup=self.speedup,
                                    valgrind=self.valgrind, gdb=self.gdb, gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL('ArduSub', options=self.options)
        self.mavproxy.expect('Telemetry log: (\S+)')
        logfile = self.mavproxy.match.group(1)
        progress("LOGFILE %s" % logfile)

        buildlog = util.reltopdir("../buildlogs/ArduSub-test.tlog")
        progress("buildlog=%s" % buildlog)
        if os.path.exists(buildlog):
            os.unlink(buildlog)
        try:
            os.link(logfile, buildlog)
        except Exception:
            pass

        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, expect_callback)

        expect_list_clear()
        expect_list_extend([self.sitl, self.mavproxy])

        progress("Started simulator")

        # get a mavlink connection going
        try:
            self.mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
        except Exception as msg:
            progress("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
            raise
        self.mav.message_hooks.append(message_hook)
        self.mav.idle_hooks.append(idle_hook)
        self.hasInit = True
        progress("Ready to start testing!")

    def close(self):
        if self.use_map:
            self.mavproxy.send("module unload map\n")
            self.mavproxy.expect("Unloaded module map")

        self.mav.close()
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        valgrind_log = util.valgrind_log_filepath(binary=self.binary, model=self.frame)
        if os.path.exists(valgrind_log):
            os.chmod(valgrind_log, 0o644)
            shutil.copy(valgrind_log, util.reltopdir("../buildlogs/ArduSub-valgrind.log"))

    # def test_arm_motors_radio(self):
    #     super(AutotestSub, self).test_arm_motors_radio()
    #
    # def test_disarm_motors_radio(self):
    #     super(AutotestSub, self).test_disarm_motors_radio()
    #
    # def test_autodisarm_motors(self):
    #     super(AutotestSub, self).test_autodisarm_motors()
    #
    # def test_rtl(self, home, distance_min=10, timeout=250):
    #     super(AutotestSub, self).test_rtl(home, distance_min=10, timeout=250)
    #
    # def test_throttle_failsafe(self, home, distance_min=10, side=60, timeout=180):
    #     super(AutotestSub, self).test_throttle_failsafe(home, distance_min=10, side=60, timeout=180)
    #
    # def test_mission(self, filename):
    #     super(AutotestSub, self).test_mission(filename)

    def dive_manual(self):
        self.set_rc(3, 1600)
        self.set_rc(5, 1600)
        self.set_rc(6, 1550)

        if not self.wait_distance(50, accuracy=7, timeout=200):
            return False

        self.set_rc(4, 1550)

        if not self.wait_heading(0):
            return False

        self.set_rc(4, 1500)

        if not self.wait_distance(50, accuracy=7, timeout=100):
            return False

        self.set_rc(4, 1550)

        if not self.wait_heading(0):
            return False

        self.set_rc(4, 1500)
        self.set_rc(5, 1500)
        self.set_rc(6, 1100)

        if not self.wait_distance(75, accuracy=7, timeout=100):
            return False

        self.set_rc_default()

        self.disarm_vehicle()
        progress("Manual dive OK")
        return True

    def dive_mission(self, filename):

        progress("Executing mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Saved [0-9]+ waypoints')
        self.set_rc_default()

        if not self.arm_vehicle():
            progress("Failed to ARM")
            return False

        self.mavproxy.send('mode auto\n')
        self.wait_mode('AUTO')

        if not self.wait_waypoint(1, 5, max_dist=5):
            return False

        self.disarm_vehicle()

        progress("Mission OK")
        return True

    def autotest(self):
        """Autotest ArduSub in SITL."""
        if not self.hasInit:
            self.init()

        failed = False
        e = 'None'
        try:
            progress("Waiting for a heartbeat with mavlink protocol %s" % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()

            # wait for EKF and GPS checks to pass
            self.mavproxy.expect('IMU0 is using GPS')

            self.homeloc = self.mav.location()
            progress("Home location: %s" % self.homeloc)
            self.set_rc_default()
            if not self.arm_vehicle():
                progress("Failed to ARM")
                failed = True
            if not self.dive_manual():
                progress("Failed manual dive")
                failed = True
            if not self.dive_mission(os.path.join(testdir, "sub_mission.txt")):
                progress("Failed auto mission")
                failed = True
            if not self.log_download(util.reltopdir("../buildlogs/ArduSub-log.bin")):
                progress("Failed log download")
                failed = True
        except pexpect.TIMEOUT as e:
            progress("Failed with timeout")
            failed = True

        self.close()

        if failed:
            progress("FAILED: %s" % e)
            return False
        return True
