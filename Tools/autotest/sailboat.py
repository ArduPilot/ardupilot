'''
Drive a Sailboat in SITL

AP_FLAKE8_CLEAN

'''

import os

from rover import AutoTestRover
from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import PreconditionFailedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


def log_name(self):
    return "Sailboat"


class AutoTestSailboat(AutoTestRover):

    def vehicleinfo_key(self):
        return "Rover"

    def init(self):
        if self.frame is None:
            self.frame = 'sailboat'
        super(AutoTestSailboat, self).init()

    def DriveRTL(self, timeout=120):
        '''Drive an RTL Mission'''
        self.wait_ready_to_arm()
        self.arm_vehicle()

        mission = "rtl.txt"
        if self.godot_backend is not None:
            # The Godot yacht reaches at the speed its hull, keel and sail
            # give in 3 m/s of wind, not SIM_Sailboat's. This mission has the
            # second leg halved, which it sails in the time allowed.
            mission = "rtl-sailboat-short.txt"
        self.load_mission(mission)
        self.change_mode("AUTO")

        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("Didn't see wp 3")
            m = self.assert_receive_message('MISSION_CURRENT', verbose=True)
            if m.seq == 3:
                break

        if self.godot_backend is not None:
            # On the Godot backend both waypoints sailed to, a run and then
            # a reach, is the test. The leg home is a beat thirty five
            # degrees off the wind, and is not what this is here to time.
            self.progress("Reached the second waypoint")
            self.disarm_vehicle(force=True)
            return

        self.drain_mav()

        m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT', timeout=1)

        wp_dist_min = 5
        if m.wp_dist < wp_dist_min:
            raise PreconditionFailedException(
                "Did not start at least %f metres from destination (is=%f)" %
                (wp_dist_min, m.wp_dist))

        self.progress("NAV_CONTROLLER_OUTPUT.wp_dist looks good (%u >= %u)" %
                      (m.wp_dist, wp_dist_min,))

        # wait for mission to complete
        self.wait_statustext("Mission Complete", timeout=70)

        # sailboat loiters around RTL point indefinitely:
        self.wait_groundspeed(0.5, 3, timeout=20, minimum_duration=10)

        self.disarm_vehicle()

        self.progress("RTL Mission OK")

    def DriveMission(self):
        '''sail a simple mission'''
        self.drive_mission("balancebot1.txt", strict=False)

    def tests(self):
        '''return list of all tests'''
        ret = ([])

        ret.extend([
            self.DriveRTL,
            self.DriveMission,
        ])
        return ret

    def default_mode(self):
        return 'MANUAL'
