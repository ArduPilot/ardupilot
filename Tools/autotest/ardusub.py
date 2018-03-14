#!/usr/bin/env python

# Dive ArduSub in SITL
from __future__ import print_function
import os
import shutil

import pexpect
from pymavlink import mavutil

from pysim import util
from pysim import vehicleinfo

from common import AutoTest

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(33.810313, -118.393867, 0, 185)


class AutoTestSub(AutoTest):
    def __init__(self,
                 binary,
                 viewerip=None,
                 use_map=False,
                 valgrind=False,
                 gdb=False,
                 speedup=10,
                 frame=None,
                 params=None,
                 gdbserver=False):
        super(AutoTestSub, self).__init__()
        self.binary = binary
        self.options = ('--sitl=127.0.0.1:5501 --out=127.0.0.1:19550'
                        ' --streamrate=10')
        self.viewerip = viewerip
        self.use_map = use_map
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

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.frame is None:
            self.frame = 'vectored'

        if self.viewerip:
            self.options += " --out=%s:14550" % self.viewerip
        if self.use_map:
            self.options += ' --map'

        self.sitl = util.start_SITL(self.binary,
                                    wipe=True,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup_default)
        self.mavproxy = util.start_MAVProxy_SITL('ArduSub')

        self.progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        # setup test parameters
        vinfo = vehicleinfo.VehicleInfo()
        if self.params is None:
            frames = vinfo.options["ArduSub"]["frames"]
            self.params = frames[self.frame]["default_params_filename"]
        if not isinstance(self.params, list):
            self.params = [self.params]
        for x in self.params:
            self.mavproxy.send("param load %s\n" % os.path.join(testdir, x))
            self.mavproxy.expect('Loaded [0-9]+ parameters')
        self.set_parameter('LOG_REPLAY', 1)
        self.set_parameter('LOG_DISARMED', 1)
        self.progress("RELOADING SITL WITH NEW PARAMETERS")

        # restart with new parms
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(self.sitl)

        self.sitl = util.start_SITL(self.binary,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL('ArduSub',
                                                 options=self.options)
        self.mavproxy.expect('Telemetry log: (\S+)')
        logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % logfile)

        buildlog = util.reltopdir("../buildlogs/ArduSub-test.tlog")
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
                          (connection_string, msg,))
            raise
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)
        self.hasInit = True
        self.progress("Ready to start testing!")

    def close(self):
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
                        self.buildlogs_path("ArduSub-valgrind.log"))

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
        self.progress("Manual dive OK")
        return True

    def dive_mission(self, filename):

        self.progress("Executing mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Saved [0-9]+ waypoints')
        self.set_rc_default()

        if not self.arm_vehicle():
            self.progress("Failed to ARM")
            return False

        self.mavproxy.send('mode auto\n')
        self.wait_mode('AUTO')

        if not self.wait_waypoint(1, 5, max_dist=5):
            return False

        self.disarm_vehicle()

        self.progress("Mission OK")
        return True

    def autotest(self):
        """Autotest ArduSub in SITL."""
        if not self.hasInit:
            self.init()

        failed = False
        e = 'None'
        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.mavproxy.send('param set FS_GCS_ENABLE 0\n')
            self.progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()

            # wait for EKF and GPS checks to pass
            self.mavproxy.expect('IMU0 is using GPS')

            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)
            self.set_rc_default()
            if not self.arm_vehicle():
                self.progress("Failed to ARM")
                failed = True
            if not self.dive_manual():
                self.progress("Failed manual dive")
                failed = True
            if not self.dive_mission(os.path.join(testdir, "sub_mission.txt")):
                self.progress("Failed auto mission")
                failed = True
            if not self.log_download(self.buildlogs_path("ArduSub-log.bin")):
                self.progress("Failed log download")
                failed = True
        except pexpect.TIMEOUT as e:
            self.progress("Failed with timeout")
            failed = True

        self.close()

        if failed:
            self.progress("FAILED: %s" % e)
            return False
        return True
