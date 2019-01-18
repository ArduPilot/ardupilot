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
                 breakpoints=[],
                 **kwargs):
        super(AutoTestQuadPlane, self).__init__(**kwargs)
        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver
        self.breakpoints = breakpoints

        self.home = "%f,%f,%u,%u" % (HOME.lat,
                                     HOME.lng,
                                     HOME.alt,
                                     HOME.heading)
        self.homeloc = None
        self.speedup = speedup

        self.log_name = "QuadPlane"
        self.logfile = None

        self.sitl = None
        self.hasInit = False

    def init(self):
        if self.frame is None:
            self.frame = 'quadplane'

        self.mavproxy_logfile = self.open_mavproxy_logfile()

        defaults_file = os.path.join(testdir, 'default_params/quadplane.parm')
        self.sitl = util.start_SITL(self.binary,
                                    wipe=True,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    defaults_file=defaults_file,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver,
                                    breakpoints=self.breakpoints,
                                    )
        self.mavproxy = util.start_MAVProxy_SITL(
            'QuadPlane', options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        self.logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)
        self.try_symlink_tlog()

        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Started simulator")

        self.get_mavlink_connection_going()

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
        self.load_mission(filename)
        self.mavproxy.send('fence load %s\n' % fence)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mavproxy.send('mode AUTO\n')
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 19, max_dist=60, timeout=1200)

        self.mav.motors_disarmed_wait()
        # wait for blood sample here
        self.mavproxy.send('wp set 20\n')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(20, 34, max_dist=60, timeout=1200)

        self.mav.motors_disarmed_wait()
        self.progress("Mission OK")

    def fly_qautotune(self):
        self.change_mode("QHOVER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1800)
        self.wait_altitude(30,
                           40,
                           relative=True,
                           timeout=30)
        self.set_rc(3, 1500)
        self.change_mode("QAUTOTUNE")
        tstart = self.get_sim_time()
        sim_time_expected = 5000
        deadline = tstart + sim_time_expected
        while self.get_sim_time_cached() < deadline:
            now = self.get_sim_time_cached()
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("STATUSTEXT (%u<%u): %s" % (now, deadline, m.text))
            if "AutoTune: Success" in m.text:
                self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
                # near enough for now:
                self.change_mode("QLAND")
                self.mavproxy.expect("AutoTune: Saved gains for Roll Pitch Yaw")
                self.mav.motors_disarmed_wait()
                return
        self.mav.motors_disarmed_wait()

    def default_mode(self):
        return "FBWA"

    def tests(self):
        '''return list of all tests'''
        m = os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016.txt")
        f = os.path.join(testdir,
                         "ArduPlane-Missions/Dalby-OBC2016-fence.txt")

        ret = super(AutoTestQuadPlane, self).tests()
        ret.extend([
            ("ArmFeatures", "Arm features", self.test_arm_feature),

            ("QAutoTune", "Fly QAUTOTUNE mode", self.fly_qautotune),

            ("Mission", "Dalby Mission",
             lambda: self.fly_mission(m, f))
        ])
        return ret
