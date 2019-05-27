#!/usr/bin/env python

# Fly ArduSoar in SITL
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
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 585, 354)
MISSION = 'ArduPlane-Missions/CMAC-soar.txt'
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestSoaring(AutoTest):

    def default_frame(self):
        return "plane-soaring"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def log_name(self):
        return "Soaring"

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

    def fly_mission(self, filename, fence, height_accuracy=-1):

        self.context_push()
        ex = None
        try:
            self.set_parameter("SOAR_ENABLE", 1)
            self.repeatedly_apply_parameter_file('default_params/plane-soaring.parm')
            self.load_mission("ArduPlane-Missions/CMAC-soar.txt")

            self.mavproxy.send("wp set 1\n")
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            
            # Enable thermalling RC
            rc_chan = self.get_parameter('SOAR_ENABLE_CH')
            self.send_set_rc(rc_chan, 1900)

            # Wait to detect thermal
            self.progress("Waiting for thermal")
            self.wait_mode('LOITER',timeout=600)

            # Wait to climb to SOAR_ALT_MAX
            self.progress("Waiting for climb to max altitude")
            alt_max = self.get_parameter('SOAR_ALT_MAX')
            self.wait_altitude(alt_max-10, alt_max, timeout=600, relative=True)

            # Wait for AUTO
            self.progress("Waiting for AUTO mode")
            self.wait_mode('AUTO')

            # Disable thermals
            self.set_parameter("SIM_THML_SCENARI", 0)


           # Wait to descent to SOAR_ALT_MIN
            self.progress("Waiting for glide to min altitude")
            alt_min = self.get_parameter('SOAR_ALT_MIN')
            self.wait_altitude(alt_min-10, alt_min, timeout=600, relative=True)

            self.progress("Waiting for throttle up")
            self.wait_servo_channel_value(3, 1200, timeout=2, comparator=operator.gt)

            self.progress("Waiting for climb to cutoff altitude")
            alt_ctf = self.get_parameter('SOAR_ALT_CUTOFF')
            self.wait_altitude(alt_ctf-10, alt_ctf, timeout=600, relative=True)

            # Now set FBWB mode
            self.change_mode('FBWB')
            self.wait_seconds(5)

            # Now disable soaring (should hold altitude)
            self.set_parameter("SOAR_ENABLE", 0)
            self.wait_seconds(10)

            #And reenable. This should force throttle-down
            self.set_parameter("SOAR_ENABLE", 1)
            self.wait_seconds(10)

            # Now wait for descent and check RTL
            self.wait_altitude(alt_min-10, alt_min, timeout=600, relative=True)

            self.progress("Waiting for RTL")
            self.wait_mode('RTL')

            alt_rtl = self.get_parameter('ALT_HOLD_RTL')/100

            # Wait for climb to  RTL.
            self.progress("Waiting for climb to RTL altitude")
            self.wait_altitude(alt_rtl-5, alt_rtl+5, timeout=60, relative=True)

            # Back to auto
            self.change_mode('AUTO')

            # Reenable thermals
            self.set_parameter("SIM_THML_SCENARI", 1)

            # Disable soaring using RC channel.
            self.send_set_rc(rc_chan, 1100)

            # Wait to get back to waypoint before thermal.
            self.progress("Waiting to get back to position")
            self.wait_current_waypoint(3,timeout=1200)

            # Enable soaring with mode changes suppressed)
            self.send_set_rc(rc_chan, 1500)

            # Make sure this causes throttle down.
            self.wait_servo_channel_value(3, 1200, timeout=2, comparator=operator.lt)

            self.progress("Waiting for next WP with no loiter")
            self.wait_waypoint(4,4,timeout=1200,max_dist=120)

            # Disarm
            self.disarm_vehicle()

        except Exception as e:
            self.progress("Exception caught")
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

        self.progress("Mission OK")

    def default_mode(self):
        return "MANUAL"

    def test_pid_tuning(self):
        # Do nothing, covered by fly.ArduPlane
        self.progress("Skipped")

    def test_arm_feature(self):
        # Do nothing, covered by fly.ArduPlane
        self.progress("Skipped")

    def fly_test_set_home(self):
        # Do nothing, covered by fly.ArduPlane        
        self.progress("Skipped")

    def tests(self):
        '''return list of all tests'''
        m = os.path.join(testdir, "ArduPlane-Missions/CMAC-soar.txt")
        f = os.path.join(testdir,
                         "ArduPlane-Missions/Dalby-OBC2016-fence.txt")

        ret = super(AutoTestSoaring, self).tests()
        ret.extend([
            ("Mission", "CMAC Mission",
             lambda: self.fly_mission(m, f))
        ])
        return ret
