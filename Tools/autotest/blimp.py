'''
Fly Blimp in SITL

AP_FLAKE8_CLEAN
'''

import os
import shutil

from pymavlink import mavutil

from vehicle_test_suite import TestSuite

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 584, 0)

# Flight mode switch positions are set-up in blimp.parm to be
#   switch 1 = Land
#   switch 2 = Manual
#   switch 3 = Velocity
#   switch 4 = Loiter
#   switch 5 = Manual
#   switch 6 = Manual


class AutoTestBlimp(TestSuite):
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

    def FlyManual(self):
        '''test manual mode'''
        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        acc = 0.5

        # make sure we don't drift:
        bl  = self.mav.location()
        tl  = self.offset_location_ne(location=bl, metres_north=2, metres_east=0)
        ttl = self.offset_location_ne(location=bl, metres_north=4, metres_east=0)
        tr  = self.offset_location_ne(location=bl, metres_north=4, metres_east=2)
        ttr = self.offset_location_ne(location=bl, metres_north=4, metres_east=4)

        if self.mavproxy is not None:
            self.mavproxy.send(f"map icon {bl.lat} {bl.lng} flag\n")
            self.mavproxy.send(f"map icon {tl.lat} {tl.lng} flag\n")
            self.mavproxy.send(f"map icon {ttl.lat} {ttl.lng} flag\n")
            self.mavproxy.send(f"map icon {tr.lat} {tr.lng} flag\n")
            self.mavproxy.send(f"map icon {ttr.lat} {ttr.lng} flag\n")

        self.set_rc(2, 2000)
        self.wait_distance_to_location(tl, 0, acc, timeout=10)
        self.set_rc(2, 1500)
        self.wait_distance_to_location(ttl, 0, acc, timeout=15)
        self.set_rc(1, 2000)
        self.wait_distance_to_location(tr, 0, acc, timeout=10)
        self.set_rc(1, 1500)
        self.wait_distance_to_location(ttr, 0, acc, timeout=15)
        self.change_mode('RTL')
        self.wait_distance_to_location(bl, 0, 0.5, timeout=30, minimum_duration=5) # make sure it can hold position
        self.change_mode('MANUAL')

        self.wait_distance_to_location(bl, 0, acc, timeout=5) # make sure we haven't moved from the spot

        self.set_rc(3, 2000)
        self.wait_altitude(5, 5.5, relative=True, timeout=15)
        self.set_rc(3, 1500)

        self.wait_distance_to_location(bl, 0, acc, timeout=5) # make sure we haven't moved from the spot

        self.set_rc(4, 1000)
        self.wait_heading(340, accuracy=5, timeout=5) # short timeout to check yawrate
        self.set_rc(4, 1500)

        self.wait_distance_to_location(bl, 0, acc, timeout=5) # make sure we haven't moved from the spot

        self.set_rc(3, 1000)
        self.wait_altitude(0, 0.5, relative=True, timeout=20)
        self.set_rc(3, 1500)

        self.wait_distance_to_location(bl, 0, acc, timeout=5) # make sure we haven't moved from the spot

        self.set_rc(4, 2000)
        self.wait_heading(135, accuracy=5, timeout=10) # short timeout to check yawrate
        self.set_rc(4, 1500)

        self.wait_distance_to_location(bl, 0, acc, timeout=5) # make sure we haven't moved from the spot

        self.disarm_vehicle()

    def FlyLoiter(self):
        '''test loiter mode'''

        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        siz = 5
        tim = 60

        # make sure we don't drift:
        bl = self.mav.location()
        tl = self.offset_location_ne(location=bl, metres_north=siz, metres_east=0)
        tr = self.offset_location_ne(location=bl, metres_north=siz, metres_east=siz)
        br = self.offset_location_ne(location=bl, metres_north=0, metres_east=siz)

        print("Locations are:")
        print("bottom left  ", bl.lat, bl.lng)
        print("top left     ", tl.lat, tl.lng)
        print("top right    ", tr.lat, tr.lng)
        print("bottom right ", br.lat, br.lng)

        if self.mavproxy is not None:
            self.mavproxy.send(f"map icon {bl.lat} {bl.lng} flag\n")
            self.mavproxy.send(f"map icon {tl.lat} {tl.lng} flag\n")
            self.mavproxy.send(f"map icon {tr.lat} {tr.lng} flag\n")
            self.mavproxy.send(f"map icon {br.lat} {br.lng} flag\n")

        self.set_parameter("SIMPLE_MODE", 1)

        self.set_rc(2, 2000)
        self.wait_distance_to_location(tl, 0, 0.2, timeout=tim)
        self.set_rc(2, 1500)

        self.set_rc(1, 2000)
        self.wait_distance_to_location(tr, 0, 0.5, timeout=tim)
        self.set_rc(1, 1500)

        self.set_rc(2, 1000)
        self.wait_distance_to_location(br, 0, 0.5, timeout=tim)
        self.set_rc(2, 1500)

        self.set_rc(1, 1000)
        self.wait_distance_to_location(bl, 0, 0.5, timeout=tim)
        self.set_rc(1, 1500)

        fin = self.mav.location()

        self.set_rc(4, 1700)
        self.wait_heading(135, accuracy=2, timeout=tim)
        self.set_rc(4, 1500)

        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.set_rc(3, 2000)
        self.wait_altitude(5, 5.5, relative=True, timeout=60)
        self.set_rc(3, 1000)
        self.wait_altitude(0, 0.5, relative=True, timeout=60)
        self.set_rc(3, 1500)

        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.set_rc(4, 1300)
        self.wait_heading(0, accuracy=2, timeout=tim)
        self.set_rc(4, 1500)

        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.disarm_vehicle()

    def PREFLIGHT_Pressure(self):
        '''test triggering pressure calibration with mavlink command'''
        # as airspeed is not instantiated on Blimp we expect to
        # instantly get back an accepted.
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p3=1, # p3, baro
        )

    def tests(self):
        '''return list of all tests'''
        # ret = super(AutoTestBlimp, self).tests()
        ret = []
        ret.extend([
            self.FlyManual,
            self.FlyLoiter,
            self.PREFLIGHT_Pressure,
        ])
        return ret

    def disabled_tests(self):
        return {
        }
