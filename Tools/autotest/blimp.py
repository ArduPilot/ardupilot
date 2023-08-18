'''
Fly Blimp in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
import os
import shutil

from pymavlink import mavutil

from common import AutoTest

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 584, 270)

# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize


class AutoTestBlimp(AutoTest):
    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return []

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return []

    @staticmethod
    def get_normal_armable_modes_list():
        return []

    def log_name(self):
        return "Blimp"

    def default_mode(self):
        return "MANUAL"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def default_speedup(self):
        return 100

    def set_current_test_name(self, name):
        self.current_test_name_directory = "Blimp_Tests/" + name + "/"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_streamrate(self):
        return 5

    def vehicleinfo_key(self):
        return 'Blimp'

    def default_frame(self):
        return "Blimp"

    def apply_defaultfile_parameters(self):
        # Blimp passes in a defaults_filepath in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        return self.model_defaults_filepath(self.frame)

    def wait_disarmed_default_wait_time(self):
        return 120

    def close(self):
        super(AutoTestBlimp, self).close()

        # [2014/05/07] FC Because I'm doing a cross machine build
        # (source is on host, build is on guest VM) I cannot hard link
        # This flag tells me that I need to copy the data out
        if self.copy_tlog:
            shutil.copy(self.logfile, self.buildlog)

    def is_blimp(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("DISARM_DELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("DISARM_DELAY", delay)

    def Speed(self):
        '''test we can move'''
        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # make sure we don't drift:
        start = self.mav.location()
        self.set_rc(2, 2000)
        self.wait_distance_to_location(start, 2, 10, timeout=40)
        self.disarm_vehicle()

    def tests(self):
        '''return list of all tests'''
        # ret = super(AutoTestBlimp, self).tests()
        ret = []
        ret.extend([
            self.Speed,
        ])
        return ret

    def disabled_tests(self):
        return {
        }
