#!/usr/bin/env python

# Fly ArduPlane QuadPlane in SITL
from __future__ import print_function
import os
import pexpect
from pymavlink import mavutil

from common import AutoTest
from common import AutoTestTimeoutException

from pysim import util
from pysim import vehicleinfo
import operator


# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-27.274439, 151.290064, 343, 8.7)
MISSION = 'ArduPlane-Missions/Dalby-OBC2016.txt'
FENCE = 'ArduPlane-Missions/Dalby-OBC2016-fence.txt'
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestQuadPlane(AutoTest):

    def default_frame(self):
        return "quadplane"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def log_name(self):
        return "QuadPlane"

    def apply_defaultfile_parameters(self):
        # plane passes in a defaults_file in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        vinfo = vehicleinfo.VehicleInfo()
        defaults_file = vinfo.options["ArduPlane"]["frames"][self.frame]["default_params_filename"]
        if isinstance(defaults_file, str):
            defaults_file = [defaults_file]
        defaults_list = []
        for d in defaults_file:
            defaults_list.append(os.path.join(testdir, d))
        return ','.join(defaults_list)

    def is_plane(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("LAND_DISARMDELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("LAND_DISARMDELAY", delay)

    def test_motor_mask(self):
        """Check operation of output_motor_mask"""
        """copter tailsitters will add condition: or (int(self.get_parameter('Q_TAILSIT_MOTMX')) & 1)"""
        if not(int(self.get_parameter('Q_TILT_MASK')) & 1):
            self.progress("output_motor_mask not in use")
            return
        self.progress("Testing output_motor_mask")
        self.wait_ready_to_arm()

        """Default channel for Motor1 is 5"""
        self.progress('Assert that SERVO5 is Motor1')
        assert(33 == self.get_parameter('SERVO5_FUNCTION'))

        modes = ('MANUAL', 'FBWA', 'QHOVER')
        for mode in modes:
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.arm_vehicle()
            self.progress("Raising throttle")
            self.set_rc(3, 1800)
            self.progress("Waiting for Motor1 to start")
            self.wait_servo_channel_value(5, 1100, comparator=operator.gt)

            self.set_rc(3, 1000)
            self.disarm_vehicle()
            self.wait_ready_to_arm()

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
                break
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.set_rc(3, 1200)
        self.wait_altitude(-5, 1, relative=True, timeout=30)
        while self.get_sim_time_cached() < deadline:
            self.mavproxy.send('disarm\n')
            try:
                self.wait_text("AutoTune: Saved gains for Roll Pitch Yaw", timeout=0.5)
            except AutoTestTimeoutException as e:
                continue
            break
        self.mav.motors_disarmed_wait()

    def test_pid_tuning(self):
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestQuadPlane, self).test_pid_tuning()

    def test_parameter_checks(self):
        self.test_parameter_checks_poscontrol("Q_P")

    def default_mode(self):
        return "MANUAL"

    def disabled_tests(self):
        return {
            "QAutoTune": "See https://github.com/ArduPilot/ardupilot/issues/10411",
        }

    def tests(self):
        '''return list of all tests'''
        m = os.path.join(testdir, "ArduPlane-Missions/Dalby-OBC2016.txt")
        f = os.path.join(testdir,
                         "ArduPlane-Missions/Dalby-OBC2016-fence.txt")

        ret = super(AutoTestQuadPlane, self).tests()
        ret.extend([
            ("TestMotorMask", "Test output_motor_mask", self.test_motor_mask),

            ("QAutoTune", "Fly QAUTOTUNE mode", self.fly_qautotune),

            ("ParameterChecks",
             "Test Arming Parameter Checks",
             self.test_parameter_checks),

            ("Mission", "Dalby Mission",
             lambda: self.fly_mission(m, f))
        ])
        return ret
