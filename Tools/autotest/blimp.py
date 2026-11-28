'''
Fly Blimp in SITL

AP_FLAKE8_CLEAN
'''

import os
import shutil

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import TestSuite

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = Location(-35.362938, 149.165085, 584, AltFrame.ABSOLUTE)
SITL_START_HEADING = 0

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

    def set_current_test_name(self, name):
        self.current_test_name_directory = "Blimp_Tests/" + name + "/"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    def sitl_streamrate(self):
        return 5

    def vehicleinfo_key(self):
        return 'Blimp'

    def default_frame(self):
        return "blimp"

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

    def FlyManualFinned(self):
        '''test manual mode on the finned blimp frame'''
        speed_accuracy = 0.07
        heading_accuracy = 15

        def stop_blimp():
            self.progress("Stopping.")
            self.set_rc_from_map({1: 1500, 2: 1500, 3: 1500, 4: 1500})
            self.wait_speed_vector(Vector3(0, 0, 0), accuracy=speed_accuracy, timeout=30)
            self.wait_yaw_speed(0, 0.2, 10)

        def stop_blimp_wait_heading():
            stop_blimp()
            self.wait_heading(0, accuracy=heading_accuracy, timeout=2)

        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Target speed is to 2 significant figures.

        self.progress("Moving forward.")
        self.set_rc(2, 2000)
        self.wait_speed_vector(Vector3(0.33, 0, 0), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Moving right.")
        self.set_rc(1, 2000)
        self.wait_speed_vector(Vector3(0, 0.33, 0), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Moving backward.")
        self.set_rc(2, 1000)
        self.wait_speed_vector(Vector3(-0.33, 0, 0), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Moving left.")
        self.set_rc(1, 1000)
        self.wait_speed_vector(Vector3(0, -0.33, 0), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Moving up.")
        self.set_rc(3, 2000)
        self.wait_speed_vector(Vector3(0, 0, -0.40), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Moving down.")
        self.set_rc(3, 1000)
        self.wait_speed_vector(Vector3(0, 0, 0.40), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Yawing right.")
        self.set_rc(4, 2000)
        self.wait_yaw_speed(0.40, accuracy=speed_accuracy, timeout=15)
        stop_blimp()

        self.progress("Yawing left.")
        self.set_rc(4, 1000)
        self.wait_yaw_speed(-0.40, accuracy=speed_accuracy, timeout=15)
        stop_blimp()

        self.disarm_vehicle()

    def FlyManualFourMotor(self):
        '''test manual mode on the four-motor blimp frame'''
        self.customise_SITL_commandline(
            [],
            model='blimp-motor',
            defaults_filepath=self.model_defaults_filepath('blimp-motor'),
        )

        speed_accuracy = 0.1
        heading_accuracy = 10

        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        def stop_blimp():
            self.progress("Stopping.")
            self.set_rc(1, 1500)
            self.set_rc(2, 1500)
            self.set_rc(3, 1500)
            self.set_rc(4, 1500)
            self.wait_speed_vector(Vector3(0, 0, 0), accuracy=speed_accuracy, timeout=20)
            self.wait_yaw_speed(0, 0.2, 10)

        def stop_blimp_wait_heading():
            stop_blimp()
            self.wait_heading(0, accuracy=heading_accuracy, timeout=2)

        self.progress("Moving forward.")
        self.set_rc(2, 2000)
        self.wait_speed_vector(Vector3(1.6, 0, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving right.")
        self.set_rc(1, 2000)
        self.wait_speed_vector(Vector3(0, 1.1, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving backward.")
        self.set_rc(2, 1000)
        self.wait_speed_vector(Vector3(-1.6, 0, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving left.")
        self.set_rc(1, 1000)
        self.wait_speed_vector(Vector3(0, -1.1, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving up.")
        self.set_rc(3, 2000)
        self.wait_speed_vector(Vector3(0, 0, -1.1), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Moving down.")
        self.set_rc(3, 1000)
        self.wait_speed_vector(Vector3(0, 0, 1.1), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Yawing right.")
        self.set_rc(4, 2000)
        self.wait_yaw_speed(1.0, accuracy=speed_accuracy, timeout=10)
        stop_blimp()

        self.progress("Yawing left.")
        self.set_rc(4, 1000)
        self.wait_yaw_speed(-1.0, accuracy=speed_accuracy, timeout=10)
        stop_blimp()

        self.disarm_vehicle()

    def FlyLoiter(self):
        '''test loiter mode'''

        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        siz = 5
        tim = 60

        # make sure we don't drift:
        bl = self.get_location()
        tl = self.offset_location_ne(location=bl, metres_north=siz, metres_east=0)
        tr = self.offset_location_ne(location=bl, metres_north=siz, metres_east=siz)
        br = self.offset_location_ne(location=bl, metres_north=0, metres_east=siz)

        if self.mavproxy is not None:
            self.mavproxy.send(f"map icon {bl.lat} {bl.lng} flag\n")
            self.mavproxy.send(f"map icon {tl.lat} {tl.lng} flag\n")
            self.mavproxy.send(f"map icon {tr.lat} {tr.lng} flag\n")
            self.mavproxy.send(f"map icon {br.lat} {br.lng} flag\n")

        self.set_parameter("SIMPLE_MODE", 1)

        self.progress("Flying to top left corner.")
        self.set_rc(2, 2000)
        self.wait_distance_to_location(tl, 0, 0.2, timeout=tim)
        self.set_rc(2, 1500)

        self.progress("Flying to top right corner.")
        self.set_rc(1, 2000)
        self.wait_distance_to_location(tr, 0, 0.5, timeout=tim)
        self.set_rc(1, 1500)

        self.progress("Flying to bottom right corner.")
        self.set_rc(2, 1000)
        self.wait_distance_to_location(br, 0, 0.5, timeout=tim)
        self.set_rc(2, 1500)

        self.progress("Flying to bottom left corner.")
        self.set_rc(1, 1000)
        self.wait_distance_to_location(bl, 0, 0.5, timeout=tim)
        self.set_rc(1, 1500)

        fin = self.get_location()

        self.progress("Yawing right.")
        self.set_rc(4, 1700)
        self.wait_heading(135, accuracy=2, timeout=tim)
        self.set_rc(4, 1500)
        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.progress("Going up and back down.")
        self.set_rc(3, 2000)
        self.wait_altitude(5, 5.5, relative=True, timeout=60)
        self.set_rc(3, 1000)
        self.wait_altitude(0, 0.5, relative=True, timeout=60)
        self.set_rc(3, 1500)
        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.progress("Yawing left.")
        self.set_rc(4, 1300)
        self.wait_heading(0, accuracy=2, timeout=tim)
        self.set_rc(4, 1500)
        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.disarm_vehicle()

    def FlyAuto(self):
        '''test AUTO mode with a small mission '''
        self.customise_SITL_commandline(
            [],
            model='blimp-motor',
            defaults_filepath=self.model_defaults_filepath('blimp-motor'),
        )

        alt = 1  # metres above home (corner/default)
        alt_high = 1.5
        alt_low = 0.5
        size = 5  # metres square

        self.progress("Uploading simple relhome mission.")

        # build a simple relative-to-home mission using helper
        # items are tuples: (MAV_CMD, metres_north, metres_east, alt, [opts])
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,      0,      alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size/2, -1,     alt_high),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size,   0,      alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size-1, size/2, alt_low),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size,   size,   alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size/2, size-1, alt_high),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,      size,   alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,      0,      alt),
        ])

        # ensure we start from first waypoint
        self.set_current_waypoint(0, check_afterwards=False)

        # switch to AUTO and arm
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_statustext("Mission Complete", timeout=100)
        # always disarm at end of test
        self.disarm_vehicle()

    def SIMCompare(self):
        '''compare logged EKF2 and EKF3 estimates against simulator truth'''
        self.set_parameters({
            'AHRS_EKF_TYPE': 3,
            'EK2_ENABLE': 1,
            'EK3_ENABLE': 1,
        })
        self.reboot_sitl()

        # loiter and translate to give the estimators something to track:
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        bl = self.get_location()
        tl = self.offset_location_ne(location=bl, metres_north=5, metres_east=0)
        self.set_rc(2, 2000)
        self.wait_distance_to_location(tl, 0, 0.2, timeout=60)
        self.set_rc(2, 1500)
        self.delay_sim_time(10, reason="vehicle to settle")
        self.disarm_vehicle()

        self.assert_ekfs_match_sim_state()

    def UTMGlobalPosition(self):
        '''test UTM_GLOBAL_POSITION message sending'''
        self.wait_ready_to_arm()
        m = self.poll_message("UTM_GLOBAL_POSITION")
        if all(b == 0 for b in m.uas_id):
            raise NotAchievedException("UAS ID is all zeros")
        if m.flags & mavutil.mavlink.UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE == 0:
            raise NotAchievedException("UAS_ID_AVAILABLE flag not set")

    def PREFLIGHT_Pressure(self):
        '''test triggering pressure calibration with mavlink command'''
        # as airspeed is not instantiated on Blimp we expect to
        # instantly get back an accepted.
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p3=1, # p3, baro
        )

    HAB_RC_TERMINATE = 7
    HAB_RC_RELEASE = 8
    HAB_RC_VENT = 9
    HAB_RC_BALLAST = 10

    def start_hab(self, extra_parameters=None):
        '''start the high altitude balloon model, with its vent, ballast,
        termination and launch release driven from RC inputs'''
        # the balloon is released as soon as the release input is high
        for chan in self.HAB_RC_VENT, self.HAB_RC_BALLAST, self.HAB_RC_TERMINATE, self.HAB_RC_RELEASE:
            self.set_rc(chan, 1000)
        self.customise_SITL_commandline(
            [],
            model='hab',
            defaults_filepath=self.model_defaults_filepath('hab'),
        )
        if extra_parameters is not None:
            self.set_parameters(extra_parameters)
            self.reboot_sitl()

    def get_altitude(self, relative=False, timeout=30, altitude_source=None):
        '''returns the altitude in metres. The streamed position is used,
        as asking for the message each time fails on a busy machine in
        the thousands of times a long balloon flight looks at it'''
        if altitude_source is None:
            m = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=timeout)
            if relative:
                return m.relative_alt * 0.001
            return m.alt * 0.001
        return super(AutoTestBlimp, self).get_altitude(relative=relative, timeout=timeout, altitude_source=altitude_source)

    def release_hab(self):
        self.context_collect('STATUSTEXT')
        self.progress("Releasing balloon")
        self.set_rc(self.HAB_RC_RELEASE, 2000)
        self.wait_statustext("SIM HAB released", timeout=30, check_context=True)

    def HABFloat(self):
        '''test buoyancy, venting and ballast on the high altitude balloon model'''
        self.start_hab()

        def velocity_east():
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            return m.vy * 0.01

        # wind from the west near the ground, from the east in the stratosphere
        self.set_parameters({
            'SIM_HAB_WND_ENA': 1,
            'SIM_HAB_W1_ALT': 0,
            'SIM_HAB_W1_SPD': 5,
            'SIM_HAB_W1_DIR': 270,
            'SIM_HAB_W2_ALT': 10000,
            'SIM_HAB_W2_SPD': 8,
            'SIM_HAB_W2_DIR': 90,
        })

        self.wait_ready_to_arm()
        launch_alt = self.get_altitude()
        self.release_hab()

        self.progress("Checking ascent rate and low level wind")
        self.wait_altitude(launch_alt + 300, launch_alt + 600, timeout=300)
        self.wait_climbrate(2.5, 3.5, timeout=60, minimum_duration=10)
        if velocity_east() < 3:
            raise NotAchievedException("Not drifting east in the low level wind")

        # a zero pressure envelope floats where the air it displaces
        # when full matches the mass of the system including lift gas
        # the gas arrives colder than the air around it and the balloon
        # keeps rising slowly as it warms, so allow time to settle.
        self.progress("Waiting for float")
        self.wait_altitude(17100, 17500, timeout=9000, minimum_duration=1800)
        self.wait_climbrate(-0.2, 0.2, timeout=600)
        float_alt = self.get_altitude()
        if velocity_east() > -5:
            raise NotAchievedException("Not drifting west in the upper level wind")

        self.progress("Venting gas to descend")
        self.set_rc(self.HAB_RC_VENT, 2000)
        self.delay_sim_time(180, reason="gas to vent")
        self.set_rc(self.HAB_RC_VENT, 1000)
        self.wait_climbrate(-2.5, -0.6, timeout=300, minimum_duration=10)

        # with the envelope no longer full the gas volume does not
        # recover as the balloon sinks, so the descent continues until
        # ballast is released
        self.wait_altitude(float_alt - 1000, float_alt - 300, timeout=1800)
        self.wait_climbrate(-2.5, -0.6, timeout=60, minimum_duration=10)

        self.progress("Releasing ballast to climb")
        self.set_rc(self.HAB_RC_BALLAST, 2000)
        self.delay_sim_time(80, reason="ballast to drop")
        self.set_rc(self.HAB_RC_BALLAST, 1000)
        self.wait_climbrate(1, 3, timeout=300, minimum_duration=10)

        self.progress("Waiting for float at a higher ceiling")
        self.wait_altitude(float_alt + 200, float_alt + 600, timeout=5400, minimum_duration=1800)
        self.wait_climbrate(-0.2, 0.2, timeout=600)

        self.progress("Terminating flight")
        self.set_rc(self.HAB_RC_TERMINATE, 2000)
        self.wait_statustext("SIM HAB terminated", timeout=30, check_context=True)
        self.wait_climbrate(-30, -15, timeout=120, minimum_duration=10)

    def HABSealed(self):
        '''test a sealed envelope holding altitude as its gas cools'''
        self.start_hab(extra_parameters={
            'SIM_HAB_ENV_TYPE': 1,
        })
        self.wait_ready_to_arm()
        self.release_hab()

        # the relief valve leaves the envelope holding more gas than a
        # zero pressure envelope, so it floats a little lower
        self.progress("Waiting for float")
        self.wait_altitude(16900, 17400, timeout=9000, minimum_duration=1800)
        self.wait_climbrate(-0.2, 0.2, timeout=600)
        float_alt = self.get_altitude()

        self.progress("Cooling the gas within the superpressure margin")
        self.set_parameter('SIM_HAB_SUPERHT', -5)
        self.wait_altitude(float_alt - 50, float_alt + 50, timeout=2000, minimum_duration=1800)

        self.progress("Cooling the gas until superpressure is lost")
        self.set_parameter('SIM_HAB_SUPERHT', -25)
        self.wait_altitude(float_alt - 1500, float_alt - 500, timeout=3600)

    def tests(self):
        '''return list of all tests'''
        ret = []
        ret.extend([
            self.FlyManualFinned,
            self.FlyManualFourMotor,
            self.FlyLoiter,
            self.FlyAuto,
            self.SIMCompare,
            self.PREFLIGHT_Pressure,
            self.UTMGlobalPosition,
            self.HABFloat,
            self.HABSealed,
        ])
        return ret

    def disabled_tests(self):
        return {
        }
