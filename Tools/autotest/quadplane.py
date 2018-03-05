#!/usr/bin/env python

# Fly ArduPlane QuadPlane in SITL
from __future__ import print_function
import os
import pexpect
import shutil
from pymavlink import mavutil

from common import *
from pysim import util

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-27.274439, 151.290064, 343, 8.7)
MISSION = 'ArduPlane-Missions/Dalby-OBC2016.txt'
FENCE = 'ArduPlane-Missions/Dalby-OBC2016-fence.txt'
WIND = "0,180,0.2"  # speed,direction,variance


class AutotestQuadPlane(Autotest):
    def __init__(self, binary, viewerip=None, use_map=False, valgrind=False, gdb=False, speedup=10, frame=None, params=None, gdbserver=False):
        super(AutotestQuadPlane, self).__init__()
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

        self.log_name = "ArduCopter"
        self.logfile = None
        self.buildlog = None
        self.copy_tlog = False

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.frame is None:
            self.frame = 'quadplane'

        if self.viewerip:
            self.options += " --out=%s:14550" % self.viewerip
        if self.use_map:
            self.options += ' --map'

        self.sitl = util.start_SITL(self.binary, wipe=True, model=self.frame, home=self.home, speedup=self.speedup,
                                    defaults_file=os.path.join(testdir, 'default_params/quadplane.parm'),
                                    valgrind=self.valgrind, gdb=self.gdb, gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL('QuadPlane', options=self.options)
        self.mavproxy.expect('Telemetry log: (\S+)')
        logfile = self.mavproxy.match.group(1)
        progress("LOGFILE %s" % logfile)

        buildlog = util.reltopdir("../buildlogs/QuadPlane-test.tlog")
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
            shutil.copy(valgrind_log, util.reltopdir("../buildlogs/QuadPlane-valgrind.log"))

    # def test_arm_motors_radio(self):
    #     super(AutotestQuadPlane, self).test_arm_motors_radio()
    #
    # def test_disarm_motors_radio(self):
    #     super(AutotestQuadPlane, self).test_disarm_motors_radio()
    #
    # def test_autodisarm_motors(self):
    #     super(AutotestQuadPlane, self).test_autodisarm_motors()
    #
    # def test_rtl(self, home, distance_min=10, timeout=250):
    #     super(AutotestQuadPlane, self).test_rtl(home, distance_min=10, timeout=250)
    #
    # def test_throttle_failsafe(self, home, distance_min=10, side=60, timeout=180):
    #     super(AutotestQuadPlane, self).test_throttle_failsafe(home, distance_min=10, side=60, timeout=180)
    #
    # def test_mission(self, filename):
    #     super(AutotestQuadPlane, self).test_mission(filename)

    def fly_mission(self, filename, fence, height_accuracy=-1):
        """Fly a mission from a file."""
        progress("Flying mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('fence load %s\n' % fence)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('mode AUTO\n')
        self.wait_mode('AUTO')
        if not self.wait_waypoint(1, 19, max_dist=60, timeout=1200):
            return False
        self.mavproxy.expect('DISARMED')
        # wait for blood sample here
        self.mavproxy.send('wp set 20\n')
        self.arm_vehicle()
        if not self.wait_waypoint(20, 34, max_dist=60, timeout=1200):
            return False
        self.mavproxy.expect('DISARMED')
        progress("Mission OK")
        return True

    def autotest(self):
        """Autotest QuadPlane in SITL."""
        self.frame = 'quadplane'
        if not self.hasInit:
            self.init()

        failed = False
        e = 'None'
        try:
            progress("Waiting for a heartbeat with mavlink protocol %s" % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            progress("Waiting for GPS fix")
            self.mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
            self.mav.wait_gps_fix()
            while self.mav.location().alt < 10:
                self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            progress("Home location: %s" % self.homeloc)

            # wait for EKF and GPS checks to pass
            progress("Waiting reading for arm")
            self.wait_seconds(30)

            self.arm_vehicle()

            if not self.fly_mission(os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016.txt"),
                                    os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016-fence.txt")):
                progress("Failed mission")
                failed = True
        except pexpect.TIMEOUT as e:
            progress("Failed with timeout")
            failed = True

        self.close()

        if failed:
            progress("FAILED: %s" % e)
            return False
        return True
