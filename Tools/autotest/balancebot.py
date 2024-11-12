'''
Drive a BalanceBot in SITL

AP_FLAKE8_CLEAN

'''

from __future__ import print_function

import os

from rover import AutoTestRover

import vehicle_test_suite
from vehicle_test_suite import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


class AutoTestBalanceBot(AutoTestRover):

    def log_name(self):
        return "BalanceBot"

    def vehicleinfo_key(self):
        return "Rover"

    def init(self):
        if self.frame is None:
            self.frame = 'balancebot'
        super(AutoTestBalanceBot, self).init()

    def DO_SET_MODE(self):
        '''Set mode via MAV_COMMAND_DO_SET_MODE'''
        self.do_set_mode_via_command_long("HOLD")
        self.do_set_mode_via_command_long("MANUAL")

    def rc_defaults(self):
        ret = super(AutoTestBalanceBot, self).rc_defaults()
        ret[3] = 1500
        return ret

    def is_balancebot(self):
        return True

    def drive_rtl_mission_max_distance_from_home(self):
        '''maximum distance allowed from home at end'''
        '''balancebot tends to wander backwards, away from the target'''
        return 8

    def DriveRTL(self):
        '''Drive an RTL Mission'''
        # if we Hold then the balancebot continues to wander
        # indefinitely at ~1m/s, hence we set to Acro
        self.set_parameter("MIS_DONE_BEHAVE", 2)
        super(AutoTestBalanceBot, self).DriveRTL()

    def TestWheelEncoder(self):
        '''make sure wheel encoders are generally working'''
        ex = None
        try:
            self.set_parameter("WENC_TYPE", 10)
            self.set_parameter("AHRS_EKF_TYPE", 10)
            self.reboot_sitl()
            self.set_parameter("WENC2_TYPE", 10)
            self.set_parameter("WENC_POS_Y", 0.075)
            self.set_parameter("WENC2_POS_Y", -0.075)
            self.reboot_sitl()
            self.change_mode("HOLD")
            self.wait_ready_to_arm()
            self.change_mode("ACRO")
            self.arm_vehicle()
            self.set_rc(3, 1600)

            m = self.assert_receive_message('WHEEL_DISTANCE', timeout=5)

            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > 10:
                    break
                dist_home = self.distance_to_home(use_cached_home=True)
                m = self.mav.messages.get("WHEEL_DISTANCE")
                delta = abs(m.distance[0] - dist_home)
                self.progress("dist-home=%f wheel-distance=%f delta=%f" %
                              (dist_home, m.distance[0], delta))
                if delta > 5:
                    raise NotAchievedException("wheel distance incorrect")
            self.disarm_vehicle()
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            self.disarm_vehicle()
            ex = e
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def DriveMission(self):
        '''Drive Mission rover1.txt'''
        self.drive_mission("balancebot1.txt", strict=False)

    def tests(self):
        '''return list of all tests'''

        '''note that while AutoTestBalanceBot inherits from Rover we don't
inherit Rover's tests!'''
        ret = vehicle_test_suite.TestSuite.tests(self)

        ret.extend([
            self.DriveRTL,
            self.DriveMission,
            self.TestWheelEncoder,
            self.MAV_CMD_DO_SEND_BANNER,
            self.DO_SET_MODE,
            self.ServoRelayEvents,
        ])
        return ret

    def default_mode(self):
        return 'MANUAL'
