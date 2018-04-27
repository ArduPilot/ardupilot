#!/usr/bin/env python

# Fly ArduPlane QuadPlane in SITL
from __future__ import print_function
import os
import pexpect
from pymavlink import mavutil

from common import AutoTest
from pysim import util

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-27.274439, 151.290064, 343, 8.7)
MISSION = 'ArduPlane-Missions/Dalby-OBC2016.txt'
FENCE = 'ArduPlane-Missions/Dalby-OBC2016-fence.txt'
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestQuadPlane(AutoTest):
    def __init__(self,
                 binary,
                 valgrind=False,
                 gdb=False,
                 speedup=10,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 **kwargs):
        super(AutoTestQuadPlane, self).__init__(**kwargs)
        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver

        self.home = "%f,%f,%u,%u" % (HOME.lat,
                                     HOME.lng,
                                     HOME.alt,
                                     HOME.heading)
        self.homeloc = None
        self.speedup = speedup
        self.speedup_default = 10

        self.log_name = "QuadPlane"
        self.logfile = None
        self.buildlog = None
        self.copy_tlog = False

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.frame is None:
            self.frame = 'quadplane'

        defaults_file = os.path.join(testdir, 'default_params/quadplane.parm')
        self.sitl = util.start_SITL(self.binary,
                                    wipe=True,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    defaults_file=defaults_file,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL(
            'QuadPlane', options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % logfile)

        buildlog = self.buildlogs_path("QuadPlane-test.tlog")
        self.progress("buildlog=%s" % buildlog)
        if os.path.exists(buildlog):
            os.unlink(buildlog)
        try:
            os.link(logfile, buildlog)
        except Exception:
            pass

        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Started simulator")

        # get a mavlink connection going
        connection_string = '127.0.0.1:19550'
        try:
            self.mav = mavutil.mavlink_connection(connection_string,
                                                  robust_parsing=True)
        except Exception as msg:
            self.progress("Failed to start mavlink connection on %s: %s" %
                          (connection_string, msg))
            raise
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)
        self.hasInit = True
        self.progress("Ready to start testing!")

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
    #     super(AutotestQuadPlane, self).test_rtl(home,
    #                              distance_min=10, timeout=250)
    #
    # def test_throttle_failsafe(self, home, distance_min=10,
    #                                   side=60, timeout=180):
    #     super(AutotestQuadPlane, self).test_throttle_failsafe(home,
    #                          distance_min=10, side=60, timeout=180)
    #
    # def test_mission(self, filename):
    #     super(AutotestQuadPlane, self).test_mission(filename)

    def fly_mission(self, filename, fence, height_accuracy=-1):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('fence load %s\n' % fence)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('mode AUTO\n')
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 19, max_dist=60, timeout=1200)

        self.mavproxy.expect('DISARMED')
        # wait for blood sample here
        self.mavproxy.send('wp set 20\n')
        self.arm_vehicle()
        self.wait_waypoint(20, 34, max_dist=60, timeout=1200)

        self.mavproxy.expect('DISARMED')
        self.progress("Mission OK")

    def autotest(self):
        """Autotest QuadPlane in SITL."""
        if not self.hasInit:
            self.init()

        self.fail_list = []

        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Waiting for GPS fix")
            self.mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
            self.mav.wait_gps_fix()
            while self.mav.location().alt < 10:
                self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)

            # wait for EKF and GPS checks to pass
            self.progress("Waiting reading for arm")
            self.wait_seconds(30)

            self.arm_vehicle()

            m = os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016.txt")
            f = os.path.join(testdir,
                             "ArduPlane-Missions/Dalby-OBC2016-fence.txt")

            self.run_test("Mission", lambda: self.fly_mission(m, f))
        except pexpect.TIMEOUT as e:
            self.progress("Failed with timeout")
            self.fail_list.append("Failed with timeout")

        self.close()

        if len(self.fail_list):
            self.progress("FAILED: %s" % self.fail_list)
            return False
        return True
