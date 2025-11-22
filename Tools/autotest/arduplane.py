'''
Fly ArduPlane in SITL

AP_FLAKE8_CLEAN
'''

import copy
import math
import os
import signal
import time

from pymavlink import quaternion
from pymavlink import mavutil

from pymavlink.rotmat import Vector3

import vehicle_test_suite

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import OldpymavlinkException
from vehicle_test_suite import PreconditionFailedException
from vehicle_test_suite import Test
from vehicle_test_suite import WaitModeTimeout
from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK

from pysim import vehicleinfo
from pysim import util

import operator

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 585, 354)
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestPlane(vehicle_test_suite.TestSuite):
    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return ["FOLLOW"]

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return ["GUIDED", "AUTO"]

    @staticmethod
    def get_normal_armable_modes_list():
        return ["MANUAL", "STABILIZE", "ACRO"]

    def log_name(self):
        return "ArduPlane"

    def default_speedup(self):
        return 100

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def defaults_filepath(self):
        return os.path.join(testdir, 'default_params/plane-jsbsim.parm')

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduPlane_Tests/" + name + "/"

    def default_frame(self):
        return "plane-elevrev"

    def apply_defaultfile_parameters(self):
        # plane passes in a defaults_filepath in place of applying
        # parameters afterwards.
        pass

    def is_plane(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("LAND_DISARMDELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("LAND_DISARMDELAY", delay)

    def takeoff(self, alt=150, alt_max=None, relative=True, mode=None, timeout=None):
        """Takeoff to altitude."""

        if mode == "TAKEOFF":
            return self.takeoff_in_TAKEOFF(alt=alt, relative=relative, timeout=timeout)

        return self.takeoff_in_FBWA(alt=alt, alt_max=alt_max, relative=relative, timeout=timeout)

    def takeoff_in_TAKEOFF(self, alt=150, relative=True, mode=None, alt_epsilon=2, timeout=None):
        if relative is not True:
            raise ValueError("Only relative alt supported ATM")
        self.change_mode("TAKEOFF")
        self.context_push()
        self.set_parameter('TKOFF_ALT', alt)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_altitude(alt-alt_epsilon, alt+alt_epsilon, relative=True, timeout=timeout)
        self.context_pop()

    def takeoff_in_FBWA(self, alt=150, alt_max=None, relative=True, mode=None, timeout=30):
        if alt_max is None:
            alt_max = alt + 30

        self.change_mode("FBWA")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # some rudder to counteract the prop torque
        self.set_rc(4, 1700)

        # some up elevator to keep the tail down
        self.set_rc(2, 1200)

        # get it moving a bit first
        self.set_rc(3, 1300)
        self.wait_groundspeed(6, 100)

        # a bit faster again, straighten rudder
        self.set_rc_from_map({
            3: 1600,
            4: 1500,
        })
        self.wait_groundspeed(12, 100)

        # hit the gas harder now, and give it some more elevator
        self.set_rc_from_map({
            2: 1100,
            3: 2000,
        })

        # gain a bit of altitude
        self.wait_altitude(alt, alt_max, timeout=timeout, relative=relative)

        # level off
        self.set_rc(2, 1500)

        self.progress("TAKEOFF COMPLETE")

    def fly_left_circuit(self):
        """Fly a left circuit, 200m on a side."""
        self.change_mode('FBWA')
        self.set_rc(3, 2000)
        self.wait_level_flight()

        self.progress("Flying left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(1, 1000)
            self.wait_heading(270 - (90*i), accuracy=10)
            self.set_rc(1, 1500)
            self.progress("Starting leg %u" % i)
            self.wait_distance(100, accuracy=20)
        self.progress("Circuit complete")

    def fly_RTL(self):
        """Fly to home."""
        self.progress("Flying home in RTL")
        target_loc = self.home_position_as_mav_location()
        target_loc.alt += 100
        self.change_mode('RTL')
        self.wait_location(target_loc,
                           accuracy=120,
                           height_accuracy=20,
                           timeout=180)
        self.progress("RTL Complete")

    def NeedEKFToArm(self):
        """Ensure the EKF must be healthy for the vehicle to arm."""
        self.progress("Ensuring we need EKF to be healthy to arm")
        self.set_parameter("SIM_GPS1_ENABLE", 0)
        self.context_collect("STATUSTEXT")
        tstart = self.get_sim_time()
        success = False
        for run_cmd in self.run_cmd, self.run_cmd_int:
            while not success:
                if self.get_sim_time_cached() - tstart > 60:
                    raise NotAchievedException("Did not get correct failure reason")
                run_cmd(mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS)
                try:
                    self.wait_statustext(".*AHRS: not using configured AHRS type.*", timeout=1, check_context=True, regex=True)
                    success = True
                    continue
                except AutoTestTimeoutException:
                    pass

        self.set_parameter("SIM_GPS1_ENABLE", 1)
        self.wait_ready_to_arm()

    def fly_LOITER(self, num_circles=4):
        """Loiter where we are."""
        self.progress("Testing LOITER for %u turns" % num_circles)
        self.change_mode('LOITER')

        m = self.assert_receive_message('VFR_HUD')
        initial_alt = m.alt
        self.progress("Initial altitude %u\n" % initial_alt)

        while num_circles > 0:
            self.wait_heading(0, accuracy=10, timeout=60)
            self.wait_heading(180, accuracy=10, timeout=60)
            num_circles -= 1
            self.progress("Loiter %u circles left" % num_circles)

        m = self.assert_receive_message('VFR_HUD')
        final_alt = m.alt
        self.progress("Final altitude %u initial %u\n" %
                      (final_alt, initial_alt))

        self.change_mode('FBWA')

        if abs(final_alt - initial_alt) > 20:
            raise NotAchievedException("Failed to maintain altitude")

        self.progress("Completed Loiter OK")

    def fly_CIRCLE(self, num_circles=1):
        """Circle where we are."""
        self.progress("Testing CIRCLE for %u turns" % num_circles)
        self.change_mode('CIRCLE')

        m = self.assert_receive_message('VFR_HUD')
        initial_alt = m.alt
        self.progress("Initial altitude %u\n" % initial_alt)

        while num_circles > 0:
            self.wait_heading(0, accuracy=10, timeout=60)
            self.wait_heading(180, accuracy=10, timeout=60)
            num_circles -= 1
            self.progress("CIRCLE %u circles left" % num_circles)

        m = self.assert_receive_message('VFR_HUD')
        final_alt = m.alt
        self.progress("Final altitude %u initial %u\n" %
                      (final_alt, initial_alt))

        self.change_mode('FBWA')

        if abs(final_alt - initial_alt) > 20:
            raise NotAchievedException("Failed to maintain altitude")

        self.progress("Completed CIRCLE OK")

    def wait_level_flight(self, accuracy=5, timeout=30):
        """Wait for level flight."""
        tstart = self.get_sim_time()
        self.progress("Waiting for level flight")
        self.set_rc(1, 1500)
        self.set_rc(2, 1500)
        self.set_rc(4, 1500)
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.assert_receive_message('ATTITUDE')
            roll = math.degrees(m.roll)
            pitch = math.degrees(m.pitch)
            self.progress("Roll=%.1f Pitch=%.1f" % (roll, pitch))
            if math.fabs(roll) <= accuracy and math.fabs(pitch) <= accuracy:
                self.progress("Attained level flight")
                return
        raise NotAchievedException("Failed to attain level flight")

    def change_altitude(self, altitude, accuracy=30, relative=False):
        """Get to a given altitude."""
        if relative:
            altitude += self.home_position_as_mav_location().alt
        self.change_mode('FBWA')
        alt_error = self.mav.messages['VFR_HUD'].alt - altitude
        if alt_error > 0:
            self.set_rc(2, 2000)
        else:
            self.set_rc(2, 1000)
        self.wait_altitude(altitude-accuracy/2, altitude+accuracy/2)
        self.set_rc(2, 1500)
        self.progress("Reached target altitude at %u" %
                      self.mav.messages['VFR_HUD'].alt)
        return self.wait_level_flight()

    def axial_left_roll(self, count=1):
        """Fly a left axial roll."""
        # full throttle!
        self.set_rc(3, 2000)
        self.change_altitude(300, relative=True)

        # fly the roll in manual
        self.change_mode('MANUAL')

        while count > 0:
            self.progress("Starting roll")
            self.set_rc(1, 1000)
            try:
                self.wait_roll(-150, accuracy=90)
                self.wait_roll(150, accuracy=90)
                self.wait_roll(0, accuracy=90)
            except Exception as e:
                self.set_rc(1, 1500)
                raise e
            count -= 1

        # back to FBWA
        self.set_rc(1, 1500)
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
        return self.wait_level_flight()

    def inside_loop(self, count=1):
        """Fly a inside loop."""
        # full throttle!
        self.set_rc(3, 2000)
        self.change_altitude(300, relative=True)
        # fly the loop in manual
        self.change_mode('MANUAL')

        while count > 0:
            self.progress("Starting loop")
            self.set_rc(2, 1000)
            self.wait_pitch(-60, accuracy=20)
            self.wait_pitch(0, accuracy=20)
            count -= 1

        # back to FBWA
        self.set_rc(2, 1500)
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
        return self.wait_level_flight()

    def set_attitude_target(self, tolerance=10):
        """Test setting of attitude target in guided mode."""
        self.change_mode("GUIDED")

        steps = [{"name": "roll-over",         "roll": 60, "pitch": 0,  "yaw": 0, "throttle": 0, "type_mask": 0b10000001},
                 {"name": "roll-back",         "roll": 0,  "pitch": 0,  "yaw": 0, "throttle": 0, "type_mask": 0b10000001},
                 {"name": "pitch-up+throttle", "roll": 0,  "pitch": 20, "yaw": 0, "throttle": 1, "type_mask": 0b11000010},
                 {"name": "pitch-back",        "roll": 0,  "pitch": 0,  "yaw": 0, "throttle": 0, "type_mask": 0b10000010}]

        state_wait = "wait"
        state_hold = "hold"
        try:
            for step in steps:
                step_start = self.get_sim_time_cached()
                state = state_wait
                state_start = self.get_sim_time_cached()
                while True:
                    m = self.mav.recv_match(type='ATTITUDE',
                                            blocking=True,
                                            timeout=0.1)
                    now = self.get_sim_time_cached()
                    if now - step_start > 30:
                        raise AutoTestTimeoutException("Maneuvers not completed")
                    if m is None:
                        continue

                    angle_error = 0
                    if (step["type_mask"] & 0b00000001) or (step["type_mask"] == 0b10000000):
                        angle_error += abs(math.degrees(m.roll) - step["roll"])

                    if (step["type_mask"] & 0b00000010) or (step["type_mask"] == 0b10000000):
                        angle_error += abs(math.degrees(m.pitch) - step["pitch"])

                    if (step["type_mask"] & 0b00000100) or (step["type_mask"] == 0b10000000):
                        # Strictly we should angle wrap, by plane doesn't support yaw correctly anyway so its not tested here
                        angle_error += abs(math.degrees(m.yaw) - step["yaw"])

                    # Note were not checking throttle, however the SITL plane needs full throttle to meet the
                    # target pitch attitude, Pitch test will fail without throttle override

                    if state == state_wait:
                        # Reduced tolerance for initial trigger
                        if angle_error < (tolerance * 0.25):
                            state = state_hold
                            state_start = now

                        # Allow 10 seconds to reach attitude
                        if (now - state_start) > 10:
                            raise NotAchievedException(step["name"] + ": Failed to get to set attitude")

                    elif state == state_hold:
                        # Give 2 seconds to stabilize
                        if (now - state_start) > 2 and not (angle_error < tolerance):
                            raise NotAchievedException(step["name"] + ": Failed to hold set attitude")

                        # Hold for 10 seconds
                        if (now - state_start) > 12:
                            # move onto next step
                            self.progress("%s Done" % (step["name"]))
                            break

                    self.progress("%s %s error: %f" % (step["name"], state, angle_error))

                    time_boot_millis = 0 # FIXME
                    target_system = 1 # FIXME
                    target_component = 1 # FIXME
                    type_mask = step["type_mask"] ^ 0xFF # FIXME
                    # attitude in radians:
                    q = quaternion.Quaternion([math.radians(step["roll"]),
                                               math.radians(step["pitch"]),
                                               math.radians(step["yaw"])])
                    self.mav.mav.set_attitude_target_send(time_boot_millis,
                                                          target_system,
                                                          target_component,
                                                          type_mask,
                                                          q,
                                                          0, # roll rate, not used in AP
                                                          0, # pitch rate, not used in AP
                                                          0, # yaw rate, not used in AP
                                                          step["throttle"])
        except Exception as e:
            self.change_mode('FBWA')
            self.set_rc(3, 1700)
            raise e

        # back to FBWA
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
        self.wait_level_flight()

    def test_stabilize(self, count=1):
        """Fly stabilize mode."""
        # full throttle!
        self.set_rc(3, 2000)
        self.set_rc(2, 1300)
        self.change_altitude(300, relative=True)
        self.set_rc(2, 1500)

        self.change_mode('STABILIZE')

        while count > 0:
            self.progress("Starting roll")
            self.set_rc(1, 2000)
            self.wait_roll(-150, accuracy=90)
            self.wait_roll(150, accuracy=90)
            self.wait_roll(0, accuracy=90)
            count -= 1

        self.set_rc(1, 1500)
        self.wait_roll(0, accuracy=5)

        # back to FBWA
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
        return self.wait_level_flight()

    def test_acro(self, count=1):
        """Fly ACRO mode."""
        # full throttle!
        self.set_rc(3, 2000)
        self.set_rc(2, 1300)
        self.change_altitude(300, relative=True)
        self.set_rc(2, 1500)

        self.change_mode('ACRO')

        while count > 0:
            self.progress("Starting roll")
            self.set_rc(1, 1000)
            self.wait_roll(-150, accuracy=90)
            self.wait_roll(150, accuracy=90)
            self.wait_roll(0, accuracy=90)
            count -= 1
        self.set_rc(1, 1500)

        # back to FBWA
        self.change_mode('FBWA')

        self.wait_level_flight()

        self.change_mode('ACRO')

        count = 2
        while count > 0:
            self.progress("Starting loop")
            self.set_rc(2, 1000)
            self.wait_pitch(-60, accuracy=20)
            self.wait_pitch(0, accuracy=20)
            count -= 1

        self.set_rc(2, 1500)

        # back to FBWA
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
        return self.wait_level_flight()

    def test_FBWB(self, mode='FBWB'):
        """Fly FBWB or CRUISE mode."""
        self.change_mode(mode)
        self.set_rc(3, 1700)
        self.set_rc(2, 1500)

        # lock in the altitude by asking for an altitude change then releasing
        self.set_rc(2, 1000)
        self.wait_distance(50, accuracy=20)
        self.set_rc(2, 1500)
        self.wait_distance(50, accuracy=20)

        m = self.assert_receive_message('VFR_HUD')
        initial_alt = m.alt
        self.progress("Initial altitude %u\n" % initial_alt)

        self.progress("Flying right circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(1, 1800)
            try:
                self.wait_heading(0 + (90*i), accuracy=20, timeout=60)
            except Exception as e:
                self.set_rc(1, 1500)
                raise e
            self.set_rc(1, 1500)
            self.progress("Starting leg %u" % i)
            self.wait_distance(100, accuracy=20)
        self.progress("Circuit complete")

        self.progress("Flying rudder left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(4, 1900)
            try:
                self.wait_heading(360 - (90*i), accuracy=20, timeout=60)
            except Exception as e:
                self.set_rc(4, 1500)
                raise e
            self.set_rc(4, 1500)
            self.progress("Starting leg %u" % i)
            self.wait_distance(100, accuracy=20)
        self.progress("Circuit complete")

        m = self.assert_receive_message('VFR_HUD')
        final_alt = m.alt
        self.progress("Final altitude %u initial %u\n" %
                      (final_alt, initial_alt))

        # back to FBWA
        self.change_mode('FBWA')

        if abs(final_alt - initial_alt) > 20:
            raise NotAchievedException("Failed to maintain altitude")

        return self.wait_level_flight()

    def fly_mission(self, filename, mission_timeout=60.0, strict=True, quadplane=False):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        num_wp = self.load_mission(filename, strict=strict)-1
        self.fly_mission_waypoints(num_wp, mission_timeout=mission_timeout, quadplane=quadplane)

    def fly_mission_waypoints(self, num_wp, mission_timeout=60.0, quadplane=False):
        self.set_current_waypoint(0, check_afterwards=False)
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.change_mode('AUTO')
        self.wait_waypoint(1, num_wp, max_dist=60, timeout=mission_timeout)
        self.wait_groundspeed(0, 0.5, timeout=mission_timeout)
        if quadplane:
            self.wait_statustext("Throttle disarmed", timeout=200, check_context=True)
        else:
            self.wait_statustext("Auto disarmed", timeout=60, check_context=True)
        self.context_pop()
        self.progress("Mission OK")

    def DO_REPOSITION(self):
        '''Test mavlink DO_REPOSITION command'''
        self.progress("Takeoff")
        self.takeoff(alt=50)
        self.set_rc(3, 1500)
        self.progress("Entering guided and flying somewhere constant")
        self.change_mode("GUIDED")
        loc = self.mav.location()
        self.location_offset_ne(loc, 500, 500)

        new_alt = 100
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=int(loc.lat * 1e7),
            p6=int(loc.lng * 1e7),
            p7=new_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(new_alt-10, new_alt, timeout=30, relative=True)

        self.install_terrain_handlers_context()

        self.location_offset_ne(loc, 500, 500)
        terrain_height_wanted = 150
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            0,
            0,
            0,
            int(loc.lat*1e7),
            int(loc.lng*1e7),
            terrain_height_wanted,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )

        # move to specific terrain-relative altitude and hold for <n> seconds
        tstart = self.get_sim_time_cached()
        achieve_start = None
        tr = None
        while True:
            if self.get_sim_time_cached() - tstart > 120:
                raise NotAchievedException("Did not move to correct terrain alt")

            m = self.assert_receive_message('TERRAIN_REPORT')
            tr = m
            terrain_height_achieved = m.current_height
            self.progress("terrain_alt=%f want=%f" %
                          (terrain_height_achieved, terrain_height_wanted))
            if m is None:
                continue
            if abs(terrain_height_wanted - terrain_height_achieved) > 5:
                if achieve_start is not None:
                    self.progress("Achieve stop")
                    achieve_start = None
            elif achieve_start is None:
                self.progress("Achieve start")
                achieve_start = self.get_sim_time_cached()
            if achieve_start is not None:
                if self.get_sim_time_cached() - achieve_start > 10:
                    break
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        self.progress("TR: %s" % tr)
        self.progress("GPI: %s" % m)
        min_delta = 4
        delta = abs(m.relative_alt/1000.0 - tr.current_height)
        if abs(delta < min_delta):
            raise NotAchievedException("Expected altitude delta (want=%f got=%f)" %
                                       (min_delta, delta))

        self.fly_home_land_and_disarm(timeout=180)

    def ExternalPositionEstimate(self):
        '''Test mavlink EXTERNAL_POSITION_ESTIMATE command'''
        if not hasattr(mavutil.mavlink, 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE'):
            raise OldpymavlinkException("pymavlink too old; upgrade pymavlink to get MAV_CMD_EXTERNAL_POSITION_ESTIMATE")  # noqa
        self.change_mode("TAKEOFF")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_altitude(48, 52, relative=True)

        loc = self.mav.location()
        self.location_offset_ne(loc, 2000, 2000)

        # setting external position fail while we have GPS lock
        self.progress("set new position with GPS")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_EXTERNAL_POSITION_ESTIMATE,
            p1=self.get_sim_time()-1, # transmit time
            p2=0.5, # processing delay
            p3=50, # accuracy
            p5=int(loc.lat * 1e7),
            p6=int(loc.lng * 1e7),
            p7=float("NaN"),    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )

        self.progress("disable the GPS")
        self.run_auxfunc(
            65,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        # fly for a bit to get into non-aiding state
        self.progress("waiting 20 seconds")
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + 20:
            self.wait_heartbeat()

        self.progress("getting base position")
        gpi = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=5)
        loc = mavutil.location(gpi.lat*1e-7, gpi.lon*1e-7, 0, 0)

        self.progress("set new position with no GPS")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_EXTERNAL_POSITION_ESTIMATE,
            p1=self.get_sim_time()-1, # transmit time
            p2=0.5, # processing delay
            p3=50, # accuracy
            p5=gpi.lat+1,
            p6=gpi.lon+1,
            p7=float("NaN"),    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        self.progress("waiting 3 seconds")
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + 3:
            self.wait_heartbeat()

        gpi2 = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=5)
        loc2 = mavutil.location(gpi2.lat*1e-7, gpi2.lon*1e-7, 0, 0)
        dist = self.get_distance(loc, loc2)

        self.progress("dist is %.1f" % dist)
        if dist > 200:
            raise NotAchievedException("Position error dist=%.1f" % dist)

        self.progress("re-enable the GPS")
        self.run_auxfunc(
            65,
            0,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        self.progress("flying home")
        self.fly_home_land_and_disarm()

    def DeepStall(self):
        '''Test DeepStall Landing'''
        # self.fly_deepstall_absolute()
        self.fly_deepstall_relative()

    def fly_deepstall_absolute(self):
        self.start_subtest("DeepStall Relative Absolute")
        deepstall_elevator_pwm = 1661
        self.set_parameters({
            "LAND_TYPE": 1,
            "LAND_DS_ELEV_PWM": deepstall_elevator_pwm,
            "RTL_AUTOLAND": 1,
        })
        self.load_mission("plane-deepstall-mission.txt")
        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Waiting for deepstall messages")

        # note that the following two don't necessarily happen in this
        # order, but at very high speedups we may miss the elevator
        # PWM if we first look for the text (due to the get_sim_time()
        # in wait_servo_channel_value)

        self.context_collect('STATUSTEXT')

        # assume elevator is on channel 2:
        self.wait_servo_channel_value(2, deepstall_elevator_pwm, timeout=240)

        self.wait_text("Deepstall: Entry: ", check_context=True)

        self.disarm_wait(timeout=120)

        self.progress("Flying home")
        self.set_current_waypoint(0, check_afterwards=False)
        self.takeoff(10)
        self.set_parameter("LAND_TYPE", 0)
        self.fly_home_land_and_disarm()

    def fly_deepstall_relative(self):
        self.start_subtest("DeepStall Relative")
        deepstall_elevator_pwm = 1661
        self.set_parameters({
            "LAND_TYPE": 1,
            "LAND_DS_ELEV_PWM": deepstall_elevator_pwm,
            "RTL_AUTOLAND": 1,
        })
        self.load_mission("plane-deepstall-relative-mission.txt")
        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(4)

        # assume elevator is on channel 2:
        self.wait_servo_channel_value(2, deepstall_elevator_pwm, timeout=240)

        self.progress("Waiting for stage DEEPSTALL_STAGE_LAND")
        self.assert_receive_message(
            'DEEPSTALL',
            condition='DEEPSTALL.stage==6',
            timeout=240,
        )
        self.progress("Reached stage DEEPSTALL_STAGE_LAND")

        self.disarm_wait(timeout=120)
        self.set_current_waypoint(0, check_afterwards=False)

        self.progress("Flying home")
        self.set_current_waypoint(0, check_afterwards=False)
        self.takeoff(100)
        self.set_parameter("LAND_TYPE", 0)
        self.fly_home_land_and_disarm(timeout=240)

    def SmartBattery(self):
        '''Test smart battery logging etc'''
        self.set_parameters({
            "BATT_MONITOR": 16, # Maxell battery monitor
        })

        # Must reboot sitl after setting monitor type for SMBus parameters to be set due to dynamic group
        self.reboot_sitl()
        self.set_parameters({
            "BATT_I2C_BUS": 2,      # specified in SIM_I2C.cpp
            "BATT_I2C_ADDR": 11,    # specified in SIM_I2C.cpp
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()
        m = self.assert_receive_message('BATTERY_STATUS', timeout=10)
        if m.voltages_ext[0] == 65536:
            raise NotAchievedException("Flag value rather than voltage")
        if abs(m.voltages_ext[0] - 1000) > 300:
            raise NotAchievedException("Did not get good ext voltage (got=%f)" %
                                       (m.voltages_ext[0],))
        self.arm_vehicle()
        self.delay_sim_time(5)
        self.disarm_vehicle()
        if not self.current_onboard_log_contains_message("BCL2"):
            raise NotAchievedException("Expected BCL2 message")

    def context_push_do_change_speed(self):
        # the following lines ensure we revert these parameter values
        # - DO_CHANGE_AIRSPEED is a permanent vehicle change!
        self.context_push()
        self.set_parameters({
            "AIRSPEED_CRUISE": self.get_parameter("AIRSPEED_CRUISE"),
            "MIN_GROUNDSPEED": self.get_parameter("MIN_GROUNDSPEED"),
            "TRIM_THROTTLE": self.get_parameter("TRIM_THROTTLE"),
        })

    def DO_CHANGE_SPEED(self):
        '''Test DO_CHANGE_SPEED command/item'''
        self.set_parameters({
            "RTL_AUTOLAND": 1,
        })

        self.context_push_do_change_speed()
        self.DO_CHANGE_SPEED_mavlink_long()
        self.context_pop()

        self.set_current_waypoint(1)
        self.zero_throttle()

        self.context_push_do_change_speed()
        self.DO_CHANGE_SPEED_mavlink_int()
        self.context_pop()

        self.context_push_do_change_speed()
        self.DO_CHANGE_SPEED_mission()
        self.context_pop()

    def DO_CHANGE_SPEED_mission(self):
        '''test DO_CHANGE_SPEED as a mission item'''
        self.start_subtest("DO_CHANGE_SPEED_mission")
        self.load_mission("mission.txt")
        self.set_current_waypoint(1)

        self.progress("Takeoff")
        self.set_rc(3, 1000)
        self.takeoff(alt=10)
        self.set_rc(3, 1500)

        self.start_subtest("Check initial speed")

        self.change_mode('AUTO')

        checks = [
            (1, self.get_parameter("AIRSPEED_CRUISE")),
            (3, 10),
            (5, 20),
            (7, 15),
        ]

        for (current_waypoint, want_airspeed) in checks:
            self.wait_current_waypoint(current_waypoint, timeout=150)
            self.wait_airspeed(want_airspeed-1, want_airspeed+1, minimum_duration=5, timeout=120)

        self.fly_home_land_and_disarm()

    def DO_CHANGE_SPEED_mavlink_int(self):
        self.DO_CHANGE_SPEED_mavlink(self.run_cmd_int)

    def DO_CHANGE_SPEED_mavlink_long(self):
        self.DO_CHANGE_SPEED_mavlink(self.run_cmd)

    def DO_CHANGE_SPEED_mavlink(self, run_cmd_method):
        '''test DO_CHANGE_SPEED as a mavlink command'''
        self.progress("Takeoff")
        self.takeoff(alt=100, mode="TAKEOFF", timeout=120)
        self.set_rc(3, 1500)
        # ensure we know what the airspeed is:
        self.progress("Entering guided and flying somewhere constant")
        self.change_mode("GUIDED")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=12345, # lat* 1e7
            p6=12345, # lon* 1e7
            p7=100,   # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.delay_sim_time(10)
        self.progress("Ensuring initial speed is known and relatively constant")
        initial_speed = 22.0
        timeout = 15
        self.wait_airspeed(initial_speed-1, initial_speed+1, minimum_duration=5, timeout=timeout)

        self.start_subtest("Setting groundspeed")
        for new_target_groundspeed in initial_speed + 5, initial_speed + 2:
            run_cmd_method(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=1, # groundspeed
                p2=new_target_groundspeed,
                p3=-1, # throttle / no change
                p4=0, # absolute values
            )
            self.wait_groundspeed(new_target_groundspeed-2, new_target_groundspeed+2, timeout=80, minimum_duration=5)
            self.progress("Adding some wind, ensuring groundspeed holds")
            self.set_parameter("SIM_WIND_SPD", 5)
            self.delay_sim_time(5)
            self.wait_groundspeed(new_target_groundspeed-2, new_target_groundspeed+2, timeout=40, minimum_duration=5)
            self.set_parameter("SIM_WIND_SPD", 0)

        # clear target groundspeed
        run_cmd_method(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=1, # groundspeed
            p2=0,
            p3=-1, # throttle / no change
            p4=0, # absolute values
        )

        self.start_subtest("Setting airspeed")
        for new_target_airspeed in initial_speed - 5, initial_speed + 5:
            run_cmd_method(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=0, # airspeed
                p2=new_target_airspeed,
                p3=-1, # throttle / no change
                p4=0, # absolute values
            )
            self.wait_airspeed(new_target_airspeed-2, new_target_airspeed+2, minimum_duration=5)

        self.context_push()
        self.progress("Adding some wind, hoping groundspeed increases/decreases")
        self.set_parameters({
            "SIM_WIND_SPD": 7,
            "SIM_WIND_DIR": 270,
        })
        self.delay_sim_time(5)
        timeout = 10
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not achieve groundspeed delta")
            m = self.assert_receive_message('VFR_HUD')
            delta = abs(m.airspeed - m.groundspeed)
            want_delta = 5
            self.progress("groundspeed and airspeed should be different (have=%f want=%f)" % (delta, want_delta))
            if delta > want_delta:
                break
        self.context_pop()

        # cancel minimum groundspeed:
        run_cmd_method(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=0, # groundspeed
            p2=-2,  # return to default
            p3=0, # throttle / no change
            p4=0, # absolute values
        )
        # cancel airspeed:
        run_cmd_method(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=1, # airspeed
            p2=-2,  # return to default
            p3=0, # throttle / no change
            p4=0, # absolute values
        )

        self.start_subtest("Setting throttle")
        self.set_parameter('ARSPD_USE', 0)  # setting throttle only effective without airspeed
        for (set_throttle, expected_throttle) in (97, 79), (60, 51), (95, 77):
            run_cmd_method(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=3, # throttle
                p2=0,
                p3=set_throttle, # throttle / no change
                p4=0, # absolute values
            )
            self.wait_message_field_values('VFR_HUD', {
                "throttle": expected_throttle,
            }, minimum_duration=5, epsilon=5)

        self.fly_home_land_and_disarm(timeout=240)

    def fly_home_land_and_disarm(self, timeout=120):
        filename = "flaps.txt"
        self.progress("Using %s to fly home" % filename)
        self.load_generic_mission(filename)
        self.change_mode("AUTO")
        # don't set current waypoint to 8 unless we're distant from it
        # or we arrive instantly and never see it as our current
        # waypoint:
        self.wait_distance_to_waypoint(8, 100, 10000000)
        self.set_current_waypoint(8)
        # TODO: reflect on file to find this magic waypoint number?
        #        self.wait_waypoint(7, num_wp-1, timeout=500) # we
        #        tend to miss the final waypoint by a fair bit, and
        #        this is probably too noisy anyway?
        self.wait_disarmed(timeout=timeout)

    def TestFlaps(self):
        """Test flaps functionality."""
        filename = "flaps.txt"
        flaps_ch = 5
        flaps_ch_min = 1000
        flaps_ch_trim = 1500
        flaps_ch_max = 2000

        servo_ch = 5
        servo_ch_min = 1200
        servo_ch_trim = 1300
        servo_ch_max = 1800

        self.set_parameters({
            "SERVO%u_FUNCTION" % servo_ch: 3, # flapsauto
            "RC%u_OPTION" % flaps_ch: 208, # Flaps RCx_OPTION
            "LAND_FLAP_PERCNT": 50,
            "LOG_DISARMED": 1,
            "RTL_AUTOLAND": 1,

            "RC%u_MIN" % flaps_ch: flaps_ch_min,
            "RC%u_MAX" % flaps_ch: flaps_ch_max,
            "RC%u_TRIM" % flaps_ch: flaps_ch_trim,

            "SERVO%u_MIN" % servo_ch: servo_ch_min,
            "SERVO%u_MAX" % servo_ch: servo_ch_max,
            "SERVO%u_TRIM" % servo_ch: servo_ch_trim,
        })

        self.progress("check flaps are not deployed")
        self.set_rc(flaps_ch, flaps_ch_min)
        self.wait_servo_channel_value(servo_ch, servo_ch_min, timeout=3)
        self.progress("deploy the flaps")
        self.set_rc(flaps_ch, flaps_ch_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(servo_ch, servo_ch_max)
        tstop = self.get_sim_time_cached()
        delta_time = tstop - tstart
        delta_time_min = 0.5
        delta_time_max = 1.5
        if delta_time < delta_time_min or delta_time > delta_time_max:
            raise NotAchievedException((
                "Flaps Slew not working (%f seconds)" % (delta_time,)))
        self.progress("undeploy flaps")
        self.set_rc(flaps_ch, flaps_ch_min)
        self.wait_servo_channel_value(servo_ch, servo_ch_min)

        self.progress("Flying mission %s" % filename)
        self.load_mission(filename)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.start_subtest("flaps should deploy for landing")  # (RC input value used for position?!)
        self.wait_servo_channel_value(servo_ch, flaps_ch_trim, timeout=300)
        self.start_subtest("flaps should undeploy at the end")
        self.wait_servo_channel_value(servo_ch, servo_ch_min, timeout=30)

        self.progress("Flaps OK")

        # because we have used flaps, RC output 5 is now non-zero -
        # it's actually the SERVO5_MIN value of 1100 now.  The simulator
        # sees that as 100us above the zero-position (it is
        # 1000-to-2000 for flaps).  That slows the aircraft down!
        self.reboot_sitl()

    def TestRCRelay(self):
        '''Test Relay RC Channel Option'''
        self.set_parameters({
            "RELAY1_FUNCTION": 1, # Enable relay as a standard relay pin
            "RC12_OPTION": 28 # Relay On/Off
        })
        self.set_rc(12, 1000)
        self.reboot_sitl() # needed for RC12_OPTION and RELAY1_FUNCTION to take effect

        off = self.get_parameter("SIM_PIN_MASK")
        if off:
            raise PreconditionFailedException("SIM_MASK_PIN off")

        # allow time for the RC library to register initial value:
        self.delay_sim_time(1)

        self.set_rc(12, 2000)
        self.wait_heartbeat()
        self.wait_heartbeat()

        on = self.get_parameter("SIM_PIN_MASK")
        if not on:
            raise NotAchievedException("SIM_PIN_MASK doesn't reflect ON")
        self.set_rc(12, 1000)
        self.wait_heartbeat()
        self.wait_heartbeat()
        off = self.get_parameter("SIM_PIN_MASK")
        if off:
            raise NotAchievedException("SIM_PIN_MASK doesn't reflect OFF")

    def TestRCCamera(self):
        '''Test RC Option - Camera Trigger'''
        self.set_parameter("RC12_OPTION", 9) # CameraTrigger
        self.set_parameter("CAM1_TYPE", 1)   # Camera with servo trigger
        self.reboot_sitl() # needed for RC12_OPTION to take effect

        x = self.mav.messages.get("CAMERA_FEEDBACK", None)
        if x is not None:
            raise PreconditionFailedException("Receiving CAMERA_FEEDBACK?!")
        self.set_rc(12, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            x = self.mav.messages.get("CAMERA_FEEDBACK", None)
            if x is not None:
                break
            self.wait_heartbeat()
        self.set_rc(12, 1000)
        if x is None:
            raise NotAchievedException("No CAMERA_FEEDBACK message received")

        self.wait_ready_to_arm()

        original_alt = self.get_altitude()

        takeoff_alt = 30
        self.takeoff(takeoff_alt)
        self.set_rc(12, 2000)
        self.delay_sim_time(1)
        self.set_rc(12, 1000)
        x = self.mav.messages.get("CAMERA_FEEDBACK", None)
        if abs(x.alt_rel - takeoff_alt) > 10:
            raise NotAchievedException("Bad relalt (want=%f vs got=%f)" % (takeoff_alt, x.alt_rel))
        if abs(x.alt_msl - (original_alt+30)) > 10:
            raise NotAchievedException("Bad absalt (want=%f vs got=%f)" % (original_alt+30, x.alt_msl))

        self.change_mode('LOITER')
        self.delay_sim_time(10)

        self.context_collect('CAMERA_FEEDBACK')
        self.set_rc(12, 2000)
        self.delay_sim_time(1)
        self.set_rc(12, 1000)
        self.assert_received_message_field_values('CAMERA_FEEDBACK', {
            "roll": math.degrees(self.assert_receive_message('ATTITUDE').roll),
        }, check_context=True, epsilon=5.0)

        self.fly_home_land_and_disarm()

    def ThrottleFailsafe(self):
        '''Fly throttle failsafe'''
        self.change_mode('MANUAL')
        m = self.assert_receive_message('SYS_STATUS')
        receiver_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise PreconditionFailedException()
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise PreconditionFailedException()
        self.progress("Testing receiver health")
        if (not (m.onboard_control_sensors_health & receiver_bit)):
            raise PreconditionFailedException()

        self.progress("Ensure we know original throttle value")
        self.wait_rc_channel_value(3, 1000)

        self.set_parameter("THR_FS_VALUE", 960)
        self.progress("Failing receiver (throttle-to-950)")
        self.context_collect("HEARTBEAT")
        self.set_parameter("SIM_RC_FAIL", 2) # throttle-to-950
        self.wait_mode('RTL') # long failsafe
        if (self.get_mode_from_mode_mapping("CIRCLE") not in
                [x.custom_mode for x in self.context_stop_collecting("HEARTBEAT")]):
            raise NotAchievedException("Did not go via circle mode")
        self.progress("Ensure we've had our throttle squashed to 950")
        self.wait_rc_channel_value(3, 950)
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Got (%s)" % str(m))
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        # skip this until RC is fixed
#        self.progress("Testing receiver health")
#        if (m.onboard_control_sensors_health & receiver_bit):
#            raise NotAchievedException("Sensor healthy when it shouldn't be")
        self.set_parameter("SIM_RC_FAIL", 0)
        # have to allow time for RC to be fetched from SITL
        self.delay_sim_time(0.5)
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        self.progress("Testing receiver health")
        if (not (m.onboard_control_sensors_health & receiver_bit)):
            raise NotAchievedException("Receiver not healthy2")
        self.change_mode('MANUAL')

        self.progress("Failing receiver (no-pulses)")
        self.context_collect("HEARTBEAT")
        self.set_parameter("SIM_RC_FAIL", 1) # no-pulses
        self.wait_mode('RTL') # long failsafe
        if (self.get_mode_from_mode_mapping("CIRCLE") not in
                [x.custom_mode for x in self.context_stop_collecting("HEARTBEAT")]):
            raise NotAchievedException("Did not go via circle mode")
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Got (%s)" % str(m))
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        self.progress("Testing receiver health")
        if (m.onboard_control_sensors_health & receiver_bit):
            raise NotAchievedException("Sensor healthy when it shouldn't be")
        self.progress("Making RC work again")
        self.set_parameter("SIM_RC_FAIL", 0)
        # have to allow time for RC to be fetched from SITL
        self.progress("Giving receiver time to recover")
        self.delay_sim_time(0.5)
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        self.progress("Testing receiver health")
        if (not (m.onboard_control_sensors_health & receiver_bit)):
            raise NotAchievedException("Receiver not healthy")
        self.change_mode('MANUAL')

        self.progress("Ensure long failsafe can trigger when short failsafe disabled")
        self.context_push()
        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "FS_SHORT_ACTN": 3, # 3 means disabled
            "SIM_RC_FAIL": 1,
        })
        self.wait_statustext("Long failsafe on", check_context=True)
        self.wait_mode("RTL")
#            self.context_clear_collection("STATUSTEXT")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.wait_text("Long Failsafe Cleared", check_context=True)
        self.change_mode("MANUAL")

        self.progress("Trying again with THR_FS_VALUE")
        self.set_parameters({
            "THR_FS_VALUE": 960,
            "SIM_RC_FAIL": 2,
        })
        self.wait_statustext("Long Failsafe on", check_context=True)
        self.wait_mode("RTL")
        self.context_pop()

        self.start_subtest("Not use RC throttle input when THR_FAILSAFE==2")
        self.takeoff(100)
        self.set_rc(3, 1800)
        self.set_rc(1, 2000)
        self.wait_attitude(desroll=45, timeout=1)
        self.context_push()
        self.set_parameters({
            "THR_FAILSAFE": 2,
            "SIM_RC_FAIL": 1,  # no pulses
        })
        self.delay_sim_time(1)
        self.wait_attitude(desroll=0, timeout=5)
        self.assert_servo_channel_value(3, self.get_parameter("RC3_MIN"))
        self.set_parameters({
            "SIM_RC_FAIL": 0,  # fix receiver
        })
        self.zero_throttle()
        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()

    def ThrottleFailsafeFence(self):
        '''Fly fence survives throttle failsafe'''
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.progress("Checking fence is not present before being configured")
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Got (%s)" % str(m))
        if (m.onboard_control_sensors_enabled & fence_bit):
            raise NotAchievedException("Fence enabled before being configured")

        self.change_mode('MANUAL')
        self.wait_ready_to_arm()

        self.load_fence("CMAC-fence.txt")

        self.set_parameter("RC7_OPTION", 11) # AC_Fence uses Aux switch functionality
        self.set_parameter("FENCE_ACTION", 4) # Fence action Brake
        self.set_rc_from_map({
            3: 1000,
            7: 2000,
        }) # Turn fence on with aux function

        m = self.assert_receive_message('FENCE_STATUS', timeout=2, verbose=True)

        self.progress("Checking fence is initially OK")
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
                               present=True,
                               enabled=True,
                               healthy=True,
                               verbose=True,
                               timeout=30)

        self.set_parameter("THR_FS_VALUE", 960)
        self.progress("Failing receiver (throttle-to-950)")
        self.set_parameter("SIM_RC_FAIL", 2) # throttle-to-950
        self.wait_mode("CIRCLE")
        self.delay_sim_time(1) # give
        self.do_timesync_roundtrip()

        self.progress("Checking fence is OK after receiver failure (bind-values)")
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        m = self.assert_receive_message('SYS_STATUS')
        if (not (m.onboard_control_sensors_enabled & fence_bit)):
            raise NotAchievedException("Fence not enabled after RC fail")
        self.do_fence_disable() # Ensure the fence is disabled after test

    def GCSFailsafe(self):
        '''Ensure Long-Failsafe works on GCS loss'''
        self.start_subtest("Test Failsafe: RTL")
        self.load_sample_mission()
        self.set_parameters({
            "FS_GCS_ENABL": 1,
            "FS_LONG_ACTN": 1,
            "RTL_AUTOLAND": 1,
            "MAV_GCS_SYSID": self.mav.source_system,
        })
        self.takeoff()
        self.change_mode('LOITER')
        self.progress("Disconnecting GCS")
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL", timeout=10)
        self.set_heartbeat_rate(self.speedup)
        self.end_subtest("Completed RTL Failsafe test")

        self.start_subtest("Test Failsafe: FBWA Glide")
        self.set_parameters({
            "FS_LONG_ACTN": 2,
        })
        self.change_mode('AUTO')
        self.progress("Disconnecting GCS")
        self.set_heartbeat_rate(0)
        self.wait_mode("FBWA", timeout=10)
        self.set_heartbeat_rate(self.speedup)
        self.end_subtest("Completed FBWA Failsafe test")

        self.start_subtest("Test Failsafe: Deploy Parachute")
        self.load_mission("plane-parachute-mission.txt")
        self.set_current_waypoint(1)
        self.set_parameters({
            "CHUTE_ENABLED": 1,
            "CHUTE_TYPE": 10,
            "SERVO9_FUNCTION": 27,
            "SIM_PARA_ENABLE": 1,
            "SIM_PARA_PIN": 9,
            "FS_LONG_ACTN": 3,
        })
        self.change_mode("AUTO")
        self.progress("Disconnecting GCS")
        self.set_heartbeat_rate(0)
        self.wait_statustext("BANG", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.end_subtest("Completed Parachute Failsafe test")

    def TestGripperMission(self):
        '''Test Gripper mission items'''
        self.set_parameter("RTL_AUTOLAND", 1)
        self.load_mission("plane-gripper-mission.txt")
        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_statustext("Gripper Grabbed", timeout=60)
        self.wait_statustext("Gripper Released", timeout=60)
        self.wait_statustext("Auto disarmed", timeout=60)

    def assert_fence_sys_status(self, present, enabled, health):
        self.delay_sim_time(1)
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        tests = [
            ("present", present, m.onboard_control_sensors_present),
            ("enabled", enabled, m.onboard_control_sensors_enabled),
            ("health", health, m.onboard_control_sensors_health),
        ]
        bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        for test in tests:
            (name, want, field) = test
            got = (field & bit) != 0
            if want != got:
                raise NotAchievedException("fence status incorrect; %s want=%u got=%u" %
                                           (name, want, got))

    def MODE_SWITCH_RESET(self):
        '''test the MODE_SWITCH_RESET auxiliary function'''
        self.set_parameters({
            "RC9_OPTION": 96,
        })

        self.progress("Using RC to change modes")
        self.set_rc(8, 1500)
        self.wait_mode('FBWA')

        self.progress("Killing RC to engage RC failsafe")
        self.set_parameter('SIM_RC_FAIL', 1)
        self.wait_mode('RTL')

        self.progress("Reinstating RC")
        self.set_parameter('SIM_RC_FAIL', 0)

        self.progress("Ensuring we don't automatically revert mode")
        self.delay_sim_time(2)
        self.assert_mode_is('RTL')

        self.progress("Ensuring MODE_SWITCH_RESET switch resets to pre-failsafe mode")
        self.set_rc(9, 2000)
        self.wait_mode('FBWA')

    def FenceStatic(self):
        '''Test Basic Fence Functionality'''
        self.progress("Checking for bizarre healthy-when-not-present-or-enabled")
        self.set_parameter("FENCE_TYPE", 4) # Start by only setting polygon fences, otherwise fence will report present
        self.assert_fence_sys_status(False, False, True)
        self.load_fence("CMAC-fence.txt")
        m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=2)
        if m is not None:
            raise NotAchievedException("Got FENCE_STATUS unexpectedly")
        self.set_parameter("FENCE_ACTION", 0) # report only
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_ACTION", 1) # RTL
        self.assert_fence_sys_status(True, False, True)
        self.do_fence_enable()
        self.assert_fence_sys_status(True, True, True)
        m = self.assert_receive_message('FENCE_STATUS', timeout=2)
        if m.breach_status:
            raise NotAchievedException("Breached fence unexpectedly (%u)" %
                                       (m.breach_status))
        self.do_fence_disable()
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_ACTION", 1)
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_ACTION", 0)
        self.assert_fence_sys_status(True, False, True)
        self.clear_fence()
        if self.get_parameter("FENCE_TOTAL") != 0:
            raise NotAchievedException("Expected zero points remaining")
        self.assert_fence_sys_status(False, False, True)
        self.progress("Trying to enable fence with no points")
        self.do_fence_enable(want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # test a rather unfortunate behaviour:
        self.progress("Killing a live fence with fence-clear")
        self.load_fence("CMAC-fence.txt")
        self.set_parameter("FENCE_ACTION", 1) # AC_FENCE_ACTION_RTL_AND_LAND == 1. mavutil.mavlink.FENCE_ACTION_RTL == 4
        self.do_fence_enable()
        self.assert_fence_sys_status(True, True, True)
        self.clear_fence()
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE, False, False, True)
        if self.get_parameter("FENCE_TOTAL") != 0:
            raise NotAchievedException("Expected zero points remaining")
        self.assert_fence_sys_status(False, False, True)
        self.do_fence_disable()

        # ensure that a fence is present if it is tin can, min alt or max alt
        self.progress("Test other fence types (tin-can, min alt, max alt")
        self.set_parameter("FENCE_TYPE", 1) # max alt
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_TYPE", 8) # min alt
        self.assert_fence_sys_status(True, False, True)
        self.set_parameter("FENCE_TYPE", 2) # tin can
        self.assert_fence_sys_status(True, False, True)

        # Test cannot arm if outside of fence and fence is enabled
        self.progress("Test Arming while vehicle below FENCE_ALT_MIN")
        default_fence_alt_min = self.get_parameter("FENCE_ALT_MIN")
        self.set_parameter("FENCE_ALT_MIN", 50)
        self.set_parameter("FENCE_TYPE", 8) # Enables minimum altitude breaches
        self.do_fence_enable()
        self.delay_sim_time(2) # Allow breach to propagate
        self.assert_fence_enabled()

        self.try_arm(False, "Vehicle breaching Min Alt fence")
        self.do_fence_disable()
        self.set_parameter("FENCE_ALT_MIN", default_fence_alt_min)

        # Test arming outside inclusion zone
        self.progress("Test arming while Vehicle breaching of inclusion zone")
        self.set_parameter("FENCE_TYPE", 4) # Enables polygon fence types
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                mavutil.location(1.000, 1.000, 0, 0),
                mavutil.location(1.000, 1.001, 0, 0),
                mavutil.location(1.001, 1.001, 0, 0),
                mavutil.location(1.001, 1.000, 0, 0)
            ]
        )])
        self.delay_sim_time(10) # let fence check run so it loads-from-eeprom
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.delay_sim_time(2) # Allow breach to propagate
        self.try_arm(False, "Vehicle breaching Polygon fence")
        self.do_fence_disable()
        self.clear_fence()

        self.progress("Test arming while vehicle inside exclusion zone")
        self.set_parameter("FENCE_TYPE", 4) # Enables polygon fence types
        home_loc = self.mav.location()
        locs = [
            mavutil.location(home_loc.lat - 0.001, home_loc.lng - 0.001, 0, 0),
            mavutil.location(home_loc.lat - 0.001, home_loc.lng + 0.001, 0, 0),
            mavutil.location(home_loc.lat + 0.001, home_loc.lng + 0.001, 0, 0),
            mavutil.location(home_loc.lat + 0.001, home_loc.lng - 0.001, 0, 0),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, locs),
        ])
        self.delay_sim_time(10) # let fence check run so it loads-from-eeprom
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.delay_sim_time(2) # Allow breach to propagate
        self.try_arm(False, "Vehicle breaching Polygon fence")
        self.do_fence_disable()

    def test_fence_breach_circle_at(self, loc, disable_on_breach=False):
        self.load_fence("CMAC-fence.txt")
        want_radius = 100
        # when ArduPlane is fixed, remove this fudge factor
        REALLY_BAD_FUDGE_FACTOR = 1.16
        expected_radius = REALLY_BAD_FUDGE_FACTOR * want_radius
        self.set_parameters({
            "RTL_RADIUS": want_radius,
            "NAVL1_LIM_BANK": 60,
            "FENCE_ACTION": 1, # AC_FENCE_ACTION_RTL_AND_LAND == 1. mavutil.mavlink.FENCE_ACTION_RTL == 4
        })

        self.wait_ready_to_arm()  # need an origin to load fence

        self.do_fence_enable()
        self.assert_fence_sys_status(True, True, True)

        self.takeoff(alt=45, alt_max=300)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 30:
                raise NotAchievedException("Did not breach fence")
            m = self.assert_receive_message('FENCE_STATUS', timeout=2)
            if m.breach_status == 0:
                continue

            # we've breached; check our state;
            if m.breach_type != mavutil.mavlink.FENCE_BREACH_BOUNDARY:
                raise NotAchievedException("Unexpected breach type %u" %
                                           (m.breach_type,))
            if m.breach_count == 0:
                raise NotAchievedException("Unexpected breach count %u" %
                                           (m.breach_count,))
            self.assert_fence_sys_status(True, True, False)
            break

        self.wait_circling_point_with_radius(loc, expected_radius)
        self.do_fence_disable()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def FenceRTL(self):
        '''Test Fence RTL'''
        self.progress("Testing FENCE_ACTION_RTL no rally point")
        # have to disable the fence once we've breached or we breach
        # it as part of the loiter-at-home!
        self.test_fence_breach_circle_at(self.home_position_as_mav_location())

    def FenceRTLRally(self):
        '''Test Fence RTL Rally'''
        self.progress("Testing FENCE_ACTION_RTL with rally point")

        self.wait_ready_to_arm()
        loc = self.home_relative_loc_ne(50, -50)
        self.upload_rally_points_from_locations([loc])
        self.test_fence_breach_circle_at(loc)

    def FenceRetRally(self):
        """ Tests the FENCE_RET_RALLY flag, either returning to fence return point,
            or rally point """
        target_system = 1
        target_component = 1
        self.progress("Testing FENCE_ACTION_RTL with fence rally point")

        self.wait_ready_to_arm()

        # Grab a location for fence return point, and upload it.
        fence_loc = self.home_position_as_mav_location()
        self.location_offset_ne(fence_loc, 50, 50)
        fence_return_mission_items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(fence_loc.lat * 1e7), # latitude
                int(fence_loc.lng * 1e7), # longitude
                0, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE
            )
        ]
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           fence_return_mission_items)
        self.delay_sim_time(1)

        # Grab a location for rally point, and upload it.
        rally_loc = self.home_relative_loc_ne(-50, 50)
        self.upload_rally_points_from_locations([rally_loc])

        return_radius = 100
        return_alt = 80
        self.set_parameters({
            "RTL_RADIUS": return_radius,
            "FENCE_ACTION": 6, # Set Fence Action to Guided
            "FENCE_TYPE": 8,   # Only use fence floor
            "FENCE_RET_ALT": return_alt,
        })
        self.do_fence_enable()
        self.assert_fence_enabled()

        self.takeoff(alt=50, alt_max=300)
        # Trigger fence breach, fly to rally location
        self.set_parameters({
            "FENCE_RET_RALLY": 1,
            "FENCE_ALT_MIN": 60,
        })
        self.wait_circling_point_with_radius(rally_loc, return_radius)
        self.set_parameter("FENCE_ALT_MIN", 0) # Clear fence breach

        # 10 second fence min retrigger time
        self.delay_sim_time(15)

        # Fly up before re-triggering fence breach. Fly to fence return point
        self.change_altitude(30, relative=True)
        self.set_parameters({
            "FENCE_RET_RALLY": 0,
            "FENCE_ALT_MIN": 60,
        })
        self.wait_altitude(altitude_min=return_alt-3,
                           altitude_max=return_alt+3,
                           relative=True)
        self.wait_circling_point_with_radius(fence_loc, return_radius)
        self.do_fence_disable() # Disable fence so we can land
        self.fly_home_land_and_disarm() # Pack it up, we're going home.

    def TerrainRally(self):
        """ Tests terrain follow with a rally point """
        self.context_push()
        self.install_terrain_handlers_context()

        def terrain_following_above_80m(mav, m):
            if m.get_type() == 'TERRAIN_REPORT':
                if m.current_height < 50:
                    raise NotAchievedException(
                        "TERRAIN_REPORT.current_height below 50m %fm" % m.current_height)
            if m.get_type() == 'VFR_HUD':
                if m.groundspeed < 2:
                    raise NotAchievedException("hit ground")

        def terrain_wait_path(loc1, loc2, steps):
            '''wait till we have terrain for N steps from loc1 to loc2'''
            tstart = self.get_sim_time_cached()
            self.progress("Waiting for terrain data")
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > 60:
                    raise NotAchievedException("Did not get correct required terrain")
                for i in range(steps):
                    lat = loc1.lat + i * (loc2.lat-loc1.lat)/steps
                    lon = loc1.lng + i * (loc2.lng-loc1.lng)/steps
                    self.mav.mav.terrain_check_send(int(lat*1.0e7), int(lon*1.0e7))

                report = self.assert_receive_message('TERRAIN_REPORT', timeout=60)
                self.progress("Terrain pending=%u" % report.pending)
                if report.pending == 0:
                    break
            self.progress("Got required terrain")

        self.wait_ready_to_arm()
        homeloc = self.mav.location()

        guided_loc = mavutil.location(-35.39723762, 149.07284612, homeloc.alt+99.0, 0)
        rally_loc = mavutil.location(-35.3654952000, 149.1558698000, homeloc.alt+100, 0)

        terrain_wait_path(homeloc, rally_loc, 10)

        # set a rally point to the west of home
        self.upload_rally_points_from_locations([rally_loc])

        self.set_parameter("TKOFF_ALT", 100)
        self.change_mode("TAKEOFF")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("TERRAIN_FOLLOW", 1)
        self.wait_altitude(90, 120, timeout=30, relative=True)
        self.progress("Done takeoff")

        self.install_message_hook_context(terrain_following_above_80m)

        self.change_mode("GUIDED")
        self.send_do_reposition(guided_loc, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.progress("Flying to guided location")
        self.wait_location(
            guided_loc,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )

        self.progress("Reached guided location")
        self.set_parameter("RALLY_LIMIT_KM", 50)
        self.change_mode("RTL")
        self.progress("Flying to rally point")
        self.wait_location(
            rally_loc,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )
        self.progress("Reached rally point with TERRAIN_FOLLOW")

        # Fly back to guided location
        self.change_mode("GUIDED")
        self.send_do_reposition(guided_loc, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.progress("Flying to back to guided location")

        # Disable terrain following and re-load rally point with relative to terrain altitude
        self.set_parameter("TERRAIN_FOLLOW", 0)

        rally_item = [self.create_MISSION_ITEM_INT(
            mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
            x=int(rally_loc.lat*1e7),
            y=int(rally_loc.lng*1e7),
            z=rally_loc.alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY
        )]
        self.correct_wp_seq_numbers(rally_item)
        self.check_rally_upload_download(rally_item)

        # Once back at guided location re-trigger RTL
        self.wait_location(
            guided_loc,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )

        self.change_mode("RTL")
        self.progress("Flying to rally point")
        self.wait_location(
            rally_loc,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )
        self.progress("Reached rally point with terrain alt frame")

        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def Parachute(self):
        '''Test Parachute'''
        self.set_rc(9, 1000)
        self.set_parameters({
            "CHUTE_ENABLED": 1,
            "CHUTE_TYPE": 10,
            "SERVO9_FUNCTION": 27,
            "SIM_PARA_ENABLE": 1,
            "SIM_PARA_PIN": 9,
        })

        self.load_mission("plane-parachute-mission.txt")
        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_statustext("BANG", timeout=60)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def ParachuteSinkRate(self):
        '''Test Parachute (SinkRate triggering)'''
        self.set_rc(9, 1000)
        self.set_parameters({
            "CHUTE_ENABLED": 1,
            "CHUTE_TYPE": 10,
            "SERVO9_FUNCTION": 27,
            "SIM_PARA_ENABLE": 1,
            "SIM_PARA_PIN": 9,
            "CHUTE_CRT_SINK": 9,
        })

        self.progress("Takeoff")
        self.takeoff(alt=300)

        self.progress("Diving")
        self.set_rc(2, 2000)
        self.wait_statustext("BANG", timeout=60)

        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def run_subtest(self, desc, func):
        self.start_subtest(desc)
        func()

    def fly_ahrs2_test(self):
        '''check secondary estimator is looking OK'''

        ahrs2 = self.assert_receive_message('AHRS2', verbose=1)
        gpi = self.assert_receive_message('GLOBAL_POSITION_INT', verbose=1)
        if self.get_distance_int(gpi, ahrs2) > 10:
            raise NotAchievedException("Secondary location looks bad")

        self.check_attitudes_match()

    def MainFlight(self):
        '''Lots of things in one flight'''
        self.change_mode('MANUAL')

        self.progress("Asserting we do support transfer of fence via mission item protocol")
        self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)

        self.run_subtest("Takeoff", self.takeoff)

        self.run_subtest("Set Attitude Target", self.set_attitude_target)

        self.run_subtest("Fly left circuit", self.fly_left_circuit)

        self.run_subtest("Left roll", lambda: self.axial_left_roll(1))

        self.run_subtest("Inside loop", self.inside_loop)

        self.run_subtest("Stabilize test", self.test_stabilize)

        self.run_subtest("ACRO test", self.test_acro)

        self.run_subtest("FBWB test", self.test_FBWB)

        self.run_subtest("CRUISE test", lambda: self.test_FBWB(mode='CRUISE'))

        self.run_subtest("RTL test", self.fly_RTL)

        self.run_subtest("LOITER test", self.fly_LOITER)

        self.run_subtest("CIRCLE test", self.fly_CIRCLE)

        self.run_subtest("AHRS2 test", self.fly_ahrs2_test)

        self.run_subtest("Mission test",
                         lambda: self.fly_mission("ap1.txt", strict=False))

    def PitotBlockage(self):
        '''Test detection and isolation of a blocked pitot tube'''
        self.set_parameters({
            "ARSPD_OPTIONS": 15,
            "ARSPD_USE": 1,
            "SIM_WIND_SPD": 7,
            "SIM_WIND_DIR": 0,
            "ARSPD_WIND_MAX": 15,
        })
        self.takeoff(alt=50, mode='TAKEOFF')
        # simulate the effect of a blocked pitot tube
        self.set_parameter("ARSPD_RATIO", 0.1)
        self.delay_sim_time(10)
        if (self.get_parameter("ARSPD_USE") == 0):
            self.progress("Faulty Sensor Disabled")
        else:
            raise NotAchievedException("Airspeed Sensor Not Disabled")
        self.delay_sim_time(20)
        # simulate the effect of blockage partially clearing
        self.set_parameter("ARSPD_RATIO", 1.0)
        self.delay_sim_time(60)
        if (self.get_parameter("ARSPD_USE") == 0):
            self.progress("Faulty Sensor Remains Disabled")
        else:
            raise NotAchievedException("Fault Sensor Re-Enabled")
        # simulate the effect of blockage fully clearing
        self.set_parameter("ARSPD_RATIO", 2.0)
        self.delay_sim_time(60)
        if (self.get_parameter("ARSPD_USE") == 1):
            self.progress("Sensor Re-Enabled")
        else:
            raise NotAchievedException("Airspeed Sensor Not Re-Enabled")
        self.fly_home_land_and_disarm()

    def AIRSPEED_AUTOCAL(self):
        '''Test AIRSPEED_AUTOCAL'''
        self.progress("Ensure no AIRSPEED_AUTOCAL on ground")
        self.set_parameters({
            "ARSPD_AUTOCAL": 1,
            "ARSPD_PIN": 2,
            "ARSPD_RATIO": 0,
            "ARSPD2_RATIO": 4,
            "ARSPD2_TYPE": 3,  # MS5525
            "ARSPD2_BUS": 1,
            "ARSPD2_AUTOCAL": 1,
            "SIM_ARSPD2_OFS": 1900,  # default is 2013

            "RTL_AUTOLAND": 1,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        self.assert_not_receive_message('AIRSPEED_AUTOCAL', timeout=5)

        # these are boot-time calibration messages:
        self.wait_statustext('Airspeed 1 calibrated', check_context=True, timeout=30)
        self.wait_statustext('Airspeed 2 calibrated', check_context=True)

        mission_filepath = "flaps.txt"
        self.load_mission(mission_filepath)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("AUTO")
        self.progress("Ensure AIRSPEED_AUTOCAL in air")
        self.assert_receive_message('AIRSPEED_AUTOCAL')
        self.wait_statustext("Airspeed 0 ratio reset", check_context=True, timeout=70)
        self.wait_statustext("Airspeed 1 ratio reset", check_context=True, timeout=70)
        self.fly_home_land_and_disarm()

    def deadreckoning_main(self, disable_airspeed_sensor=False):
        self.context_push()
        self.set_parameter("EK3_OPTIONS", 1)
        self.set_parameter("AHRS_OPTIONS", 3)
        self.set_parameter("LOG_REPLAY", 1)
        self.reboot_sitl()
        self.wait_ready_to_arm()

        if disable_airspeed_sensor:
            max_allowed_divergence = 300
        else:
            max_allowed_divergence = 150
        self.install_message_hook_context(vehicle_test_suite.TestSuite.ValidateGlobalPositionIntAgainstSimState(self, max_allowed_divergence=max_allowed_divergence))  # noqa

        try:
            # wind is from the West:
            self.set_parameter("SIM_WIND_DIR", 270)
            # light winds:
            self.set_parameter("SIM_WIND_SPD", 10)
            if disable_airspeed_sensor:
                self.set_parameter("ARSPD_USE", 0)

            self.takeoff(50)
            loc = self.mav.location()
            self.location_offset_ne(loc, 500, 500)
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p1=0,
                p2=mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                p5=int(loc.lat * 1e7),
                p6=int(loc.lng * 1e7),
                p7=100,    # alt
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            )
            self.wait_location(loc, accuracy=100)
            self.progress("Orbit with GPS and learn wind")
            # allow longer to learn wind if there is no airspeed sensor
            if disable_airspeed_sensor:
                self.delay_sim_time(180)
            else:
                self.delay_sim_time(20)
            self.set_parameter("SIM_GPS1_ENABLE", 0)
            self.progress("Continue orbit without GPS")
            self.delay_sim_time(20)
            self.change_mode("RTL")
            self.wait_distance_to_home(100, 200, timeout=200)
            # go into LOITER to create additional time for a GPS re-enable test
            self.change_mode("LOITER")
            self.set_parameter("SIM_GPS1_ENABLE", 1)
            t_enabled = self.get_sim_time()
            # The EKF should wait for GPS checks to pass when we are still able to navigate using dead reckoning
            # to prevent bad GPS being used when coming back after loss of lock due to interence.
            self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=15)
            if self.get_sim_time() < (t_enabled+9):
                raise NotAchievedException("GPS use re-started too quickly")
            # wait for EKF and vehicle position to stabilise, then test response to jamming
            self.delay_sim_time(20)

            self.set_parameter("AHRS_OPTIONS", 1)
            self.set_parameter("SIM_GPS1_JAM", 1)
            self.delay_sim_time(13)
            self.set_parameter("SIM_GPS1_JAM", 0)
            t_enabled = self.get_sim_time()
            # The EKF should wait for GPS checks to pass when we are still able to navigate using dead reckoning
            # to prevent bad GPS being used when coming back after loss of lock due to interence.
            # The EKF_STATUS_REPORT does not tell us when the good to align check passes, so the minimum time
            # value of 3.0 seconds is an arbitrary value set on inspection of dataflash logs from this test
            self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=20)
            time_since_jamming_stopped = self.get_sim_time() - t_enabled
            if time_since_jamming_stopped < 3:
                raise NotAchievedException("GPS use re-started %f sec after jamming stopped" % time_since_jamming_stopped)
            self.set_rc(3, 1000)
            self.fly_home_land_and_disarm()
        finally:
            pass

        self.context_pop()
        self.reboot_sitl()

    def Deadreckoning(self):
        '''Test deadreckoning support'''
        self.deadreckoning_main()

    def DeadreckoningNoAirSpeed(self):
        '''Test deadreckoning support with no airspeed sensor'''
        self.deadreckoning_main(disable_airspeed_sensor=True)

    def ClimbBeforeTurn(self):
        '''Test climb-before-turn'''
        self.wait_ready_to_arm()
        self.set_parameters({
            "FLIGHT_OPTIONS": 0,
            "RTL_ALTITUDE": 80,
            "RTL_AUTOLAND": 1,
        })
        takeoff_alt = 10
        self.takeoff(alt=takeoff_alt)
        self.change_mode("CRUISE")
        self.wait_distance_to_home(500, 1000, timeout=60)
        self.change_mode("RTL")
        expected_alt = self.get_parameter("RTL_ALTITUDE")

        home = self.home_position_as_mav_location()
        distance = self.get_distance(home, self.mav.location())

        self.wait_altitude(expected_alt - 10, expected_alt + 10, relative=True, timeout=80)

        new_distance = self.get_distance(home, self.mav.location())
        # We should be closer to home.
        if new_distance > distance:
            raise NotAchievedException(
                "Expected to be closer to  home (was %fm, now %fm)."
                % (distance, new_distance)
            )

        self.fly_home_land_and_disarm()
        self.set_current_waypoint(0, check_afterwards=False)

        self.change_mode("MANUAL")
        self.set_rc(3, 1000)

        self.wait_ready_to_arm()
        self.set_parameters({
            "FLIGHT_OPTIONS": 16,
            "RTL_ALTITUDE": 100,
        })
        self.takeoff(alt=takeoff_alt)
        self.change_mode("CRUISE")
        self.wait_distance_to_home(500, 1000, timeout=60)
        self.change_mode("RTL")

        home = self.home_position_as_mav_location()
        distance = self.get_distance(home, self.mav.location())

        self.wait_altitude(expected_alt - 10, expected_alt + 10, relative=True, timeout=80)

        new_distance = self.get_distance(home, self.mav.location())
        # We should be farther from to home.
        if new_distance < distance:
            raise NotAchievedException(
                "Expected to be farther from home (was %fm, now %fm)."
                % (distance, new_distance)
            )

        self.fly_home_land_and_disarm(timeout=240)

    def RTL_CLIMB_MIN(self):
        '''Test RTL_CLIMB_MIN'''
        self.wait_ready_to_arm()
        rtl_climb_min = 100
        self.set_parameter("RTL_CLIMB_MIN", rtl_climb_min)
        takeoff_alt = 50
        self.takeoff(alt=takeoff_alt)
        self.change_mode('CRUISE')
        self.wait_distance_to_home(1000, 1500, timeout=60)
        post_cruise_alt = self.get_altitude(relative=True)
        self.change_mode('RTL')
        expected_alt = self.get_parameter("RTL_ALTITUDE")
        if expected_alt == -1:
            expected_alt = self.get_altitude(relative=True)

        # ensure we're about half-way-down at the half-way-home stage:
        self.wait_distance_to_nav_target(
            0,
            500,
            timeout=240,
        )
        alt = self.get_altitude(relative=True)
        expected_halfway_alt = expected_alt + (post_cruise_alt + rtl_climb_min - expected_alt)/2.0
        if abs(alt - expected_halfway_alt) > 30:
            raise NotAchievedException("Not half-way-down and half-way-home (want=%f got=%f" %
                                       (expected_halfway_alt, alt))
        self.progress("Half-way-down at half-way-home (want=%f vs got=%f)" %
                      (expected_halfway_alt, alt))

        rtl_radius = self.get_parameter("RTL_RADIUS")
        if rtl_radius == 0:
            rtl_radius = self.get_parameter("WP_LOITER_RAD")
        self.wait_distance_to_nav_target(
            0,
            rtl_radius,
            timeout=120,
        )
        alt = self.get_altitude(relative=True)
        if abs(alt - expected_alt) > 10:
            raise NotAchievedException(
                "Expected to have %fm altitude at end of RTL (got %f)" %
                (expected_alt, alt))
        self.fly_home_land_and_disarm()

    def sample_enable_parameter(self):
        return "Q_ENABLE"

    def RangeFinder(self):
        '''Test RangeFinder Basic Functionality'''
        self.progress("Making sure we don't ordinarily get RANGEFINDER")
        self.assert_not_receive_message('RANGEFINDER')
        self.assert_not_receive_message('DISTANCE_SENSOR')

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())

        '''ensure rangefinder gives height-above-ground'''
        self.load_mission("plane-gripper-mission.txt") # borrow this
        self.set_parameter("RTL_AUTOLAND", 1)
        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(5, 5, max_dist=100)
        rf = self.assert_receive_message('RANGEFINDER')
        ds = self.assert_receive_message('DISTANCE_SENSOR')
        gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
        if abs(rf.distance - gpi.relative_alt/1000.0) > 3:
            raise NotAchievedException(
                "rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                (rf.distance, gpi.relative_alt/1000.0))
        if abs(ds.current_distance*0.01 - gpi.relative_alt/1000.0) > 3:
            raise NotAchievedException(
                "distance_sensor alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                (ds.current_distance*0.01, gpi.relative_alt/1000.0))
        self.wait_statustext("Auto disarmed", timeout=60)

        self.progress("Ensure RFND messages in log")
        if not self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("No RFND messages in log")

    def rc_defaults(self):
        ret = super(AutoTestPlane, self).rc_defaults()
        ret[3] = 1000
        ret[8] = 1800
        return ret

    def initial_mode_switch_mode(self):
        return "MANUAL"

    def default_mode(self):
        return "MANUAL"

    def PIDTuning(self):
        '''Test PID Tuning'''
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestPlane, self).PIDTuning()

    def AuxModeSwitch(self):
        '''Set modes via auxswitches'''
        self.set_parameter("FLTMODE1", 1)  # circle
        self.set_rc(8, 950)
        self.wait_mode("CIRCLE")
        self.set_rc(9, 1000)
        self.set_rc(10, 1000)
        self.set_parameters({
            "RC9_OPTION": 4, # RTL
            "RC10_OPTION": 55, # guided
        })
        self.set_rc(9, 1900)
        self.wait_mode("RTL")
        self.set_rc(10, 1900)
        self.wait_mode("GUIDED")

        self.progress("resetting both switches - should go back to CIRCLE")
        self.set_rc(9, 1000)
        self.set_rc(10, 1000)
        self.wait_mode("CIRCLE")

        self.set_rc(9, 1900)
        self.wait_mode("RTL")
        self.set_rc(10, 1900)
        self.wait_mode("GUIDED")

        self.progress("Resetting switch should repoll mode switch")
        self.set_rc(10, 1000) # this re-polls the mode switch
        self.wait_mode("CIRCLE")
        self.set_rc(9, 1000)

    def wait_for_collision_threat_to_clear(self):
        '''wait to get a "clear" collision message", then slurp remaining
        messages'''
        last_collision = self.get_sim_time()
        while True:
            now = self.get_sim_time()
            if now - last_collision > 5:
                return
            self.progress("Waiting for collision message")
            m = self.mav.recv_match(type='COLLISION', blocking=True, timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                continue
            last_collision = now

    def SimADSB(self):
        '''Tests to ensure simulated ADSB sensor continues to function'''
        self.set_parameters({
            "SIM_ADSB_COUNT": 1,
            "ADSB_TYPE": 1,
        })
        self.reboot_sitl()
        self.assert_receive_message('ADSB_VEHICLE', timeout=30)

    def ADSBResumeActionResumeLoiter(self):
        '''ensure we resume auto mission or enter loiter'''
        self.set_parameters({
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_F_ACTION": mavutil.mavlink.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY,
            "AVD_F_RCVRY": 3,  # resume auto or loiter
        })
        self.reboot_sitl()
        self.takeoff(50)
        # fly North, create thread to east, wait for flying east
        self.start_subtest("Testing loiter resume")
        self.reach_heading_manual(0)
        here = self.mav.location()
        self.test_adsb_send_threatening_adsb_message(here, offset_ne=(0, 30))
        self.wait_mode('AVOID_ADSB')
        # recovery has the vehicle circling a point... but we don't
        # know which point.  So wait 'til it looks like it is
        # circling, then grab the point, then check we're circling
        # it...
        self.wait_heading(290)
        self.wait_heading(300)
        dest = self.position_target_loc()
        REALLY_BAD_FUDGE_FACTOR = 1.25  # FIXME
        expected_radius = REALLY_BAD_FUDGE_FACTOR * self.get_parameter('WP_LOITER_RAD')
        self.wait_circling_point_with_radius(dest, expected_radius)

        self.start_subtest("Testing mission resume")
        self.reach_heading_manual(270)
        self.load_generic_mission("CMAC-circuit.txt", strict=False)
        self.change_mode('AUTO')
        self.wait_current_waypoint(2)
        self.test_adsb_send_threatening_adsb_message(here, offset_ne=(0, 30))
        self.wait_mode('AVOID_ADSB')
        self.wait_mode('AUTO')

        self.fly_home_land_and_disarm()

    def ADSBFailActionRTL(self):
        '''test ADSB avoidance action of RTL'''
        # message ADSB_VEHICLE 37 -353632614 1491652305 0 584070 0 0 0 "bob" 3 1 255 17
        self.set_parameter("RC12_OPTION", 38) # avoid-adsb
        self.set_rc(12, 2000)
        self.set_parameters({
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_F_ACTION": mavutil.mavlink.MAV_COLLISION_ACTION_RTL,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        here = self.mav.location()
        self.change_mode("FBWA")
        self.delay_sim_time(2) # TODO: work out why this is required...
        self.test_adsb_send_threatening_adsb_message(here)
        self.progress("Waiting for collision message")
        m = self.assert_receive_message('COLLISION', timeout=4)
        if m.threat_level != 2:
            raise NotAchievedException("Expected some threat at least")
        if m.action != mavutil.mavlink.MAV_COLLISION_ACTION_RTL:
            raise NotAchievedException("Incorrect action; want=%u got=%u" %
                                       (mavutil.mavlink.MAV_COLLISION_ACTION_RTL, m.action))
        self.wait_mode("RTL")

        self.progress("Sending far-away ABSD_VEHICLE message")
        self.mav.mav.adsb_vehicle_send(
            37, # ICAO address
            int(here.lat+1 * 1e7),
            int(here.lng * 1e7),
            mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
            int(here.alt*1000 + 10000), # 10m up
            0, # heading in cdeg
            0, # horizontal velocity cm/s
            0, # vertical velocity cm/s
            "bob".encode("ascii"), # callsign
            mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
            1, # time since last communication
            65535, # flags
            17 # squawk
        )
        self.wait_for_collision_threat_to_clear()
        self.change_mode("FBWA")

        self.progress("Disabling ADSB-avoidance with RC channel")
        self.set_rc(12, 1000)
        self.delay_sim_time(1) # let the switch get polled
        self.test_adsb_send_threatening_adsb_message(here)
        m = self.assert_not_receive_message('COLLISION', timeout=4)

    def GuidedRequest(self, target_system=1, target_component=1):
        '''Test handling of MISSION_ITEM in guided mode'''
        self.progress("Takeoff")
        self.takeoff(alt=50)
        self.set_rc(3, 1500)
        self.start_subtest("Ensure command bounced outside guided mode")
        desired_relative_alt = 33
        loc = self.mav.location()
        self.location_offset_ne(loc, 300, 300)
        loc.alt += desired_relative_alt
        self.mav.mav.mission_item_int_send(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, # current - guided-mode request
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            int(loc.lat * 1e7), # latitude
            int(loc.lng * 1e7), # longitude
            loc.alt, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        m = self.assert_receive_message('MISSION_ACK', timeout=5)
        if m.type != mavutil.mavlink.MAV_MISSION_ERROR:
            raise NotAchievedException("Did not get appropriate error")

        self.start_subtest("Enter guided and flying somewhere constant")
        self.change_mode("GUIDED")
        self.mav.mav.mission_item_int_send(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, # current - guided-mode request
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            int(loc.lat * 1e7), # latitude
            int(loc.lng * 1e7), # longitude
            desired_relative_alt, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        m = self.assert_receive_message('MISSION_ACK', timeout=5)
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Did not get accepted response")
        self.wait_location(loc, accuracy=100) # based on loiter radius
        self.wait_altitude(altitude_min=desired_relative_alt-3,
                           altitude_max=desired_relative_alt+3,
                           relative=True,
                           timeout=30)

        self.start_subtest("changing alt with mission item in guided mode")

        # test changing alt only - NOTE - this is still a
        # NAV_WAYPOINT, not a changel-alt request!
        desired_relative_alt = desired_relative_alt + 50
        self.mav.mav.mission_item_int_send(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            3, # current - change-alt request
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0,
            desired_relative_alt, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        self.wait_altitude(altitude_min=desired_relative_alt-3,
                           altitude_max=desired_relative_alt+3,
                           relative=True,
                           timeout=30)

        self.fly_home_land_and_disarm()

    def LOITER(self):
        '''Test Loiter mode'''
        # first test old loiter behaviour
        self.set_parameter("FLIGHT_OPTIONS", 0)
        self.takeoff(alt=200)
        self.set_rc(3, 1500)
        self.change_mode("LOITER")
        self.progress("Doing a bit of loitering to start with")
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                break
            m = self.assert_receive_message('VFR_HUD')
            new_throttle = m.throttle
            alt = m.alt
            m = self.assert_receive_message('ATTITUDE', timeout=5)
            pitch = math.degrees(m.pitch)
            self.progress("Pitch:%f throttle:%u alt:%f" % (pitch, new_throttle, alt))
        m = self.assert_receive_message('VFR_HUD', timeout=5)
        initial_throttle = m.throttle
        initial_alt = m.alt
        self.progress("Initial throttle: %u" % initial_throttle)
        # pitch down, ensure throttle decreases:
        rc2_max = self.get_parameter("RC2_MAX")
        self.set_rc(2, int(rc2_max))
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            '''stick-mixing is pushing the aircraft down.  It doesn't want to go
            down (the target loiter altitude hasn't changed), so it
            tries to add energy by increasing the throttle.
            '''
            if now - tstart > 60:
                raise NotAchievedException("Did not see increase in throttle")
            m = self.assert_receive_message('VFR_HUD', timeout=5)
            new_throttle = m.throttle
            alt = m.alt
            m = self.assert_receive_message('ATTITUDE', timeout=5)
            pitch = math.degrees(m.pitch)
            self.progress("Pitch:%f throttle:%u alt:%f" % (pitch, new_throttle, alt))
            if new_throttle - initial_throttle > 20:
                self.progress("Throttle delta achieved")
                break
        self.progress("Centering elevator and ensuring we get back to loiter altitude")
        self.set_rc(2, 1500)
        self.wait_altitude(initial_alt-1, initial_alt+1)
        # Test new loiter behaviour
        self.set_parameter("FLIGHT_OPTIONS", 1 << 12)
        # should descend at max stick
        self.set_rc(2, int(rc2_max))
        self.wait_altitude(initial_alt - 110, initial_alt - 90, timeout=90)
        # should not climb back at mid stick
        self.set_rc(2, 1500)
        self.delay_sim_time(60)
        self.wait_altitude(initial_alt - 110, initial_alt - 90)
        # should climb at min stick
        self.set_rc(2, 1100)
        self.wait_altitude(initial_alt - 10, initial_alt + 10, timeout=90)
        # return stick to center and fly home
        self.set_rc(2, 1500)
        self.fly_home_land_and_disarm()

    def CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        self.plane_CPUFailsafe()

    def LargeMissions(self):
        '''Test Manipulation of Large missions'''
        self.load_mission("Kingaroy-vlarge.txt", strict=False)
        self.load_mission("Kingaroy-vlarge2.txt", strict=False)

    def Soaring(self):
        '''Test Soaring feature'''

        model = "plane-soaring"

        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath=self.model_defaults_filepath(model),
            wipe=True)

        self.load_mission('CMAC-soar.txt', strict=False)

        # Enable thermalling RC
        rc_chan = 0
        for i in range(8):
            rcx_option = self.get_parameter('RC{0}_OPTION'.format(i+1))
            if rcx_option == 88:
                rc_chan = i+1
                break

        if rc_chan == 0:
            raise NotAchievedException("Did not find soaring enable channel option.")

        self.set_rc_from_map({
            rc_chan: 1900,
        })

        self.set_parameters({
            "SOAR_VSPEED": 0.55,
            "SOAR_MIN_THML_S": 25,
        })

        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Wait to detect thermal
        self.progress("Waiting for thermal")
        self.wait_mode('THERMAL', timeout=600)

        # Wait to climb to SOAR_ALT_MAX
        self.progress("Waiting for climb to max altitude")
        alt_max = self.get_parameter('SOAR_ALT_MAX')
        self.wait_altitude(alt_max-10, alt_max, timeout=600, relative=True)

        # Wait for AUTO
        self.progress("Waiting for AUTO mode")
        self.wait_mode('AUTO')

        # Disable thermals
        self.set_parameter("SIM_THML_SCENARI", 0)

        # Wait to descend to SOAR_ALT_MIN
        self.progress("Waiting for glide to min altitude")
        alt_min = self.get_parameter('SOAR_ALT_MIN')
        self.wait_altitude(alt_min-10, alt_min, timeout=600, relative=True)

        self.progress("Waiting for throttle up")
        self.wait_servo_channel_value(3, 1200, timeout=5, comparator=operator.gt)

        self.progress("Waiting for climb to cutoff altitude")
        alt_ctf = self.get_parameter('SOAR_ALT_CUTOFF')
        self.wait_altitude(alt_ctf-10, alt_ctf, timeout=600, relative=True)

        # Allow time to suppress throttle and start descent.
        self.delay_sim_time(20)

        # Now set FBWB mode
        self.change_mode('FBWB')
        self.delay_sim_time(5)

        # Now disable soaring (should hold altitude)
        self.set_parameter("SOAR_ENABLE", 0)
        self.delay_sim_time(10)

        # And re-enable. This should force throttle-down
        self.set_parameter("SOAR_ENABLE", 1)
        self.delay_sim_time(10)

        # Now wait for descent and check throttle up
        self.wait_altitude(alt_min-10, alt_min, timeout=600, relative=True)

        self.progress("Waiting for climb")
        self.wait_altitude(alt_ctf-10, alt_ctf, timeout=600, relative=True)

        # Back to auto
        self.change_mode('AUTO')

        # Re-enable thermals
        self.set_parameter("SIM_THML_SCENARI", 1)

        # Disable soaring using RC channel.
        self.set_rc(rc_chan, 1100)

        # Wait to get back to waypoint before thermal.
        self.progress("Waiting to get back to position")
        self.wait_current_waypoint(3, timeout=1200)

        # Enable soaring with mode changes suppressed)
        self.set_rc(rc_chan, 1500)

        # Make sure this causes throttle down.
        self.wait_servo_channel_value(3, 1200, timeout=3, comparator=operator.lt)

        self.progress("Waiting for next WP with no thermalling")
        self.wait_waypoint(4, 4, timeout=1200, max_dist=120)

        # Disarm
        self.disarm_vehicle_expect_fail()

        self.progress("Mission OK")

    def SpeedToFly(self):
        '''Test soaring speed-to-fly'''

        model = "plane-soaring"

        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath=self.model_defaults_filepath(model),
            wipe=True)

        self.load_mission('CMAC-soar.txt', strict=False)

        self.set_parameters({
            "SIM_THML_SCENARI": 0, # Turn off environmental thermals.
            "SOAR_ALT_MAX": 1000,  # remove source of random failure
        })

        # Get thermalling RC channel
        rc_chan = 0
        for i in range(8):
            rcx_option = self.get_parameter('RC{0}_OPTION'.format(i+1))
            if rcx_option == 88:
                rc_chan = i+1
                break

        if rc_chan == 0:
            raise NotAchievedException("Did not find soaring enable channel option.")

        # Disable soaring
        self.set_rc(rc_chan, 1100)

        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Wait for to 400m before starting.
        self.wait_altitude(390, 400, timeout=600, relative=True)

        # Wait 10s to stabilize.
        self.delay_sim_time(30)

        # Enable soaring (no automatic thermalling)
        self.set_rc(rc_chan, 1500)

        self.set_parameters({
            "SOAR_CRSE_ARSPD": -1,  # Enable speed to fly.
            "SOAR_VSPEED": 1,  # Set appropriate McCready.
            "SIM_WIND_SPD": 0,
        })

        self.progress('Waiting a few seconds before determining the "trim" airspeed.')
        self.delay_sim_time(20)
        m = self.assert_receive_message('VFR_HUD')
        trim_airspeed = m.airspeed
        self.progress("Using trim_airspeed=%f" % (trim_airspeed,))

        min_airspeed = self.get_parameter("AIRSPEED_MIN")
        max_airspeed = self.get_parameter("AIRSPEED_MAX")

        if trim_airspeed > max_airspeed:
            raise NotAchievedException("trim airspeed > max_airspeed (%f>%f)" %
                                       (trim_airspeed, max_airspeed))
        if trim_airspeed < min_airspeed:
            raise NotAchievedException("trim airspeed < min_airspeed (%f<%f)" %
                                       (trim_airspeed, min_airspeed))

        self.progress("Adding updraft")
        self.set_parameters({
            "SIM_WIND_SPD": 1,
            'SIM_WIND_DIR_Z': 90,
        })
        self.progress("Waiting for vehicle to move slower in updraft")
        self.wait_airspeed(0, trim_airspeed-0.5, minimum_duration=10, timeout=120)

        self.progress("Adding downdraft")
        self.set_parameter('SIM_WIND_DIR_Z', -90)
        self.progress("Waiting for vehicle to move faster in downdraft")
        self.wait_airspeed(trim_airspeed+0.5, trim_airspeed+100, minimum_duration=10, timeout=120)

        self.progress("Zeroing wind and increasing McCready")
        self.set_parameters({
            "SIM_WIND_SPD": 0,
            "SOAR_VSPEED": 2,
        })
        self.progress("Waiting for airspeed to increase with higher VSPEED")
        self.wait_airspeed(trim_airspeed+0.5, trim_airspeed+100, minimum_duration=10, timeout=120)

        # mcReady tests don't work ATM, so just return early:
        # takes too long to land, so just make it all go away:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        return

        self.start_subtest('Test McReady values')
        # Disable soaring
        self.set_rc(rc_chan, 1100)

        # Wait for to 400m before starting.
        self.wait_altitude(390, 400, timeout=600, relative=True)

        # Enable soaring
        self.set_rc(rc_chan, 2000)

        self.progress("Find airspeed with 1m/s updraft and mcready=1")
        self.set_parameters({
            "SOAR_VSPEED": 1,
            "SIM_WIND_SPD": 1,
        })
        self.delay_sim_time(20)
        m = self.assert_receive_message('VFR_HUD')
        mcready1_speed = m.airspeed
        self.progress("airspeed is %f" % mcready1_speed)

        self.progress("Reducing McCready")
        self.set_parameters({
            "SOAR_VSPEED": 0.5,
        })
        self.progress("Waiting for airspeed to decrease with lower McReady")
        self.wait_airspeed(0, mcready1_speed-0.5, minimum_duration=10, timeout=120)

        self.progress("Increasing McCready")
        self.set_parameters({
            "SOAR_VSPEED": 1.5,
        })
        self.progress("Waiting for airspeed to decrease with lower McReady")
        self.wait_airspeed(mcready1_speed+0.5, mcready1_speed+100, minimum_duration=10, timeout=120)

        # takes too long to land, so just make it all go away:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def AirspeedDrivers(self):
        '''Test AirSpeed drivers'''
        airspeed_sensors = [
            ("MS5525", 3, 1),
            ("DLVR", 7, 2),
            ("SITL", 100, 0),
        ]
        for (name, t, bus) in airspeed_sensors:
            self.context_push()
            if bus is not None:
                self.set_parameter("ARSPD2_BUS", bus)
            self.set_parameter("ARSPD2_TYPE", t)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()

            # insert listener to compare airspeeds:
            airspeed = [None, None]
            # don't start testing until we've seen real speed from
            # both sensors.  This gets us out of the noise area.
            global initial_airspeed_threshold_reached
            initial_airspeed_threshold_reached = False

            def check_airspeeds(mav, m):
                global initial_airspeed_threshold_reached
                m_type = m.get_type()
                if (m_type == 'NAMED_VALUE_FLOAT' and
                        m.name == 'AS2'):
                    airspeed[1] = m.value
                elif m_type == 'VFR_HUD':
                    airspeed[0] = m.airspeed
                else:
                    return
                if airspeed[0] is None or airspeed[1] is None:
                    return
                if airspeed[0] < 2 or airspeed[1] < 2:
                    # this mismatch can occur on takeoff, or when we
                    # smack into the ground at the end of the mission
                    return
                if not initial_airspeed_threshold_reached:
                    if not (airspeed[0] > 10 or airspeed[1] > 10):
                        return
                    initial_airspeed_threshold_reached = True
                delta = abs(airspeed[0] - airspeed[1])
                if delta > 2:
                    raise NotAchievedException("Airspeed mismatch (as1=%f as2=%f)" % (airspeed[0], airspeed[1]))
            self.install_message_hook_context(check_airspeeds)
            self.fly_mission("ap1.txt", strict=False)
            if airspeed[0] is None:
                raise NotAchievedException("Never saw an airspeed1")
            if airspeed[1] is None:
                raise NotAchievedException("Never saw an airspeed2")
            self.context_pop()
        self.reboot_sitl()

    def TerrainMission(self):
        '''Test terrain following in mission'''
        self.install_terrain_handlers_context()

        num_wp = self.load_mission("ap-terrain.txt")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        global max_alt
        max_alt = 0

        def record_maxalt(mav, m):
            global max_alt
            if m.get_type()  != 'GLOBAL_POSITION_INT':
                return
            if m.relative_alt/1000.0 > max_alt:
                max_alt = m.relative_alt/1000.0

        self.install_message_hook_context(record_maxalt)

        self.fly_mission_waypoints(num_wp-1, mission_timeout=600)

        if max_alt < 200:
            raise NotAchievedException("Did not follow terrain")

    def TerrainMissionInterrupt(self):
        '''Test terrain following when resuming a mission'''
        self.install_terrain_handlers_context()

        self.load_mission("ap-terrain.txt")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.set_parameter('TECS_RLL2THR', 2)

        # Keep track of the maximum terrain alt.
        global max_terrain_alt
        max_terrain_alt = 0

        def record_maxalt(mav, m):
            global max_terrain_alt
            if m.get_type() != 'TERRAIN_REPORT':
                return
            if m.current_height > max_terrain_alt:
                max_terrain_alt = m.current_height

        self.context_push()

        self.set_parameter("WP_RADIUS", 100)  # Ensure the aircraft will get within 100.0m of the waypoint.

        # Start the mission.
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode('AUTO')

        # After waypoint 2, go to GUIDED.
        self.wait_waypoint(3, 3, max_dist=3150, timeout=600)
        self.progress("Entering guided and flying somewhere constant")
        self.change_mode("GUIDED")
        loc = self.mav.location()
        self.location_offset_ne(loc, 350, 0)
        new_alt = 280
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=int(loc.lat * 1e7),
            p6=int(loc.lng * 1e7),
            p7=new_alt,  # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )

        # Resume auto when we are close to the GUIDED waypoint and start tracking maximum terrain alt.
        self.wait_location(loc, accuracy=100)  # based on loiter radius
        self.change_mode('AUTO')
        self.install_message_hook_context(record_maxalt)

        self.wait_waypoint(3, 3, max_dist=100, timeout=600)

        self.context_pop()

        # We've flown enough.
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        # self.fly_mission_waypoints(num_wp-1, mission_timeout=600)
        if max_terrain_alt > 120:
            raise NotAchievedException("Did not follow terrain")

    def Terrain(self):
        '''test AP_Terrain'''
        self.reboot_sitl()  # we know the terrain height at CMAC

        self.install_terrain_handlers_context()

        self.wait_ready_to_arm()
        loc = self.mav.location()

        lng_int = int(loc.lng * 1e7)
        lat_int = int(loc.lat * 1e7)

        # FIXME: once we have a pre-populated terrain cache this
        # should require an instantly correct report to pass
        tstart = self.get_sim_time_cached()
        last_terrain_report_pending = -1
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                raise NotAchievedException("Did not get correct terrain report")

            self.mav.mav.terrain_check_send(lat_int, lng_int)

            report = self.assert_receive_message('TERRAIN_REPORT', timeout=60)
            self.progress(self.dump_message_verbose(report))
            if report.spacing != 0:
                break

            # we will keep trying to long as the number of pending
            # tiles is dropping:
            if last_terrain_report_pending == -1:
                last_terrain_report_pending = report.pending
            elif report.pending < last_terrain_report_pending:
                last_terrain_report_pending = report.pending
                tstart = now

            self.delay_sim_time(1)

        self.progress(self.dump_message_verbose(report))

        expected_terrain_height = 583.5
        if abs(report.terrain_height - expected_terrain_height) > 0.5:
            raise NotAchievedException("Expected terrain height=%f got=%f" %
                                       (expected_terrain_height, report.terrain_height))

    def TerrainLoiter(self):
        '''Test terrain following in loiter'''
        self.context_push()
        self.install_terrain_handlers_context()
        self.set_parameters({
            "TERRAIN_FOLLOW": 1, # enable terrain following in loiter
            "WP_LOITER_RAD": 2000, # set very large loiter rad to get some terrain changes
        })
        alt = 200
        self.takeoff(alt*0.9, alt*1.1)
        self.set_rc(3, 1500)
        self.change_mode("LOITER")
        self.progress("loitering at %um" % alt)
        tstart = self.get_sim_time()
        timeout = 60*15  # enough time to do one and a bit circles
        max_delta = 0
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                break
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            terrain = self.assert_receive_message('TERRAIN_REPORT')
            rel_alt = terrain.current_height
            self.progress("%um above terrain (%um bove home)" %
                          (rel_alt, gpi.relative_alt/1000.0))
            if rel_alt > alt*1.2 or rel_alt < alt * 0.8:
                raise NotAchievedException("Not terrain following")
            delta = abs(rel_alt - gpi.relative_alt/1000.0)
            if delta > max_delta:
                max_delta = delta
        want_max_delta = 30
        if max_delta < want_max_delta:
            raise NotAchievedException(
                "Expected terrain and home alts to vary more than they did (max=%u want=%u)" %
                (max_delta, want_max_delta))
        self.context_pop()
        self.progress("Returning home")
        self.fly_home_land_and_disarm(240)

    def TerrainLoiterToCircle(self):
        '''loiter terrain-relative.  Switch to Circle, maintain alt'''
        self.install_terrain_handlers_context()
        self.set_parameters({
            "TERRAIN_FOLLOW": 1, # enable terrain following in loiter
            "WP_LOITER_RAD": 2000, # set very large loiter rad to get some terrain changes
        })
        alt = 50
        self.takeoff(alt*0.9, alt*1.1, mode='TAKEOFF')
        self.change_mode('LOITER')
        self.delay_sim_time(10)
        self.change_mode('CIRCLE')
        self.wait_altitude(alt*0.9, alt*1.1, minimum_duration=10, relative=True)
        self.fly_home_land_and_disarm()

    def fly_external_AHRS(self, sim, eahrs_type, mission):
        """Fly with external AHRS"""
        self.customise_SITL_commandline(["--serial4=sim:%s" % sim])

        self.set_parameters({
            "EAHRS_TYPE": eahrs_type,
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS1_TYPE": 21,
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
        })
        self.reboot_sitl()
        self.delay_sim_time(5)
        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.fly_mission(mission)

    def wait_and_maintain_wind_estimate(
            self,
            want_speed,
            want_dir,
            timeout=10,
            speed_tolerance=0.5,
            dir_tolerance=5,
            **kwargs):
        '''wait for wind estimate to reach speed and direction'''

        def validator(last, _min, _max):
            '''returns false of spd or direction is too-far wrong'''
            (spd, di) = last
            _min_spd, _min_dir = _min
            _max_spd, _max_dir = _max
            if spd < _min_spd or spd > _max_spd:
                return False
            # my apologies to whoever is staring at this and wondering
            # why we're not wrapping angles here...
            if di < _min_dir or di > _max_dir:
                return False
            return True

        def value_getter():
            '''returns a tuple of (wind_speed, wind_dir), where wind_dir is 45 if
            wind is coming from NE'''
            m = self.assert_receive_message("WIND")
            return (m.speed, m.direction)

        class ValueAverager(object):
            def __init__(self):
                self.speed_average = -1
                self.dir_average = -1
                self.count = 0.0

            def add_value(self, value):
                (spd, di) = value
                if self.speed_average == -1:
                    self.speed_average = spd
                    self.dir_average = di
                else:
                    self.speed_average += spd
                    self.di_average += spd
                self.count += 1
                return (self.speed_average/self.count, self.dir_average/self.count)

            def reset(self):
                self.count = 0
                self.speed_average = -1
                self.dir_average = -1

        self.wait_and_maintain_range(
            value_name="WindEstimates",
            minimum=(want_speed-speed_tolerance, want_dir-dir_tolerance),
            maximum=(want_speed+speed_tolerance, want_dir+dir_tolerance),
            current_value_getter=value_getter,
            value_averager=ValueAverager(),
            validator=lambda last, _min, _max: validator(last, _min, _max),
            timeout=timeout,
            **kwargs
        )

    def WindEstimates(self):
        '''fly non-external AHRS, ensure wind estimate correct'''
        self.set_parameters({
            "SIM_WIND_SPD": 5,
            "SIM_WIND_DIR": 45,
        })
        self.wait_ready_to_arm()
        self.takeoff(70)  # default wind sim wind is a sqrt function up to 60m
        self.change_mode('LOITER')
        # use default estimator to determine when to check others:
        self.wait_and_maintain_wind_estimate(5, 45, timeout=120)

        for ahrs_type in 0, 2, 3, 10:
            self.start_subtest("Checking AHRS_EKF_TYPE=%u" % ahrs_type)
            self.set_parameter("AHRS_EKF_TYPE", ahrs_type)
            self.wait_and_maintain_wind_estimate(
                5, 45,
                speed_tolerance=1,
                timeout=30
            )
        self.fly_home_land_and_disarm()

    def VectorNavEAHRS(self):
        '''Test VectorNav EAHRS support'''
        self.fly_external_AHRS("VectorNav", 1, "ap1.txt")

    def MicroStrainEAHRS5(self):
        '''Test MicroStrain EAHRS series 5 support'''
        self.fly_external_AHRS("MicroStrain5", 2, "ap1.txt")

    def MicroStrainEAHRS7(self):
        '''Test MicroStrain EAHRS series 7 support'''
        self.fly_external_AHRS("MicroStrain7", 7, "ap1.txt")

    def InertialLabsEAHRS(self):
        '''Test InertialLabs EAHRS support'''
        self.fly_external_AHRS("ILabs", 5, "ap1.txt")

    def GpsSensorPreArmEAHRS(self):
        '''Test pre-arm checks related to EAHRS_SENSORS using the MicroStrain7 driver'''
        self.customise_SITL_commandline(["--serial4=sim:MicroStrain7"])

        self.set_parameters({
            "EAHRS_TYPE": 7,
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS1_TYPE": 0, # Disabled (simulate user setup error)
            "GPS2_TYPE": 0, # Disabled (simulate user setup error)
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
            "EAHRS_SENSORS": 13, # GPS is enabled
        })
        self.reboot_sitl()
        self.delay_sim_time(5)
        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )

        self.assert_prearm_failure("ExternalAHRS: Incorrect number", # Cut short due to message limits.
                                   timeout=30,
                                   other_prearm_failures_fatal=False)

        self.set_parameters({
            "EAHRS_TYPE": 7,
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS1_TYPE": 1, # Auto
            "GPS2_TYPE": 21, # EARHS
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
            "EAHRS_SENSORS": 13, # GPS is enabled
        })
        self.reboot_sitl()
        self.delay_sim_time(5)
        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )
        # Check prearm success with MicroStrain when the first GPS is occupied by another GPS.
        # This supports the use case of comparing MicroStrain dual antenna to another GPS.
        self.wait_ready_to_arm()

    def get_accelvec(self, m):
        return Vector3(m.xacc, m.yacc, m.zacc) * 0.001 * 9.81

    def get_gyrovec(self, m):
        return Vector3(m.xgyro, m.ygyro, m.zgyro) * 0.001 * math.degrees(1)

    def IMUTempCal(self):
        '''Test IMU temperature calibration'''
        self.progress("Setting up SITL temperature profile")
        self.set_parameters({
            "SIM_IMUT1_ENABLE" : 1,
            "SIM_IMUT1_ACC1_X" : 120000.000000,
            "SIM_IMUT1_ACC1_Y" : -190000.000000,
            "SIM_IMUT1_ACC1_Z" : 1493.864746,
            "SIM_IMUT1_ACC2_X" : -51.624416,
            "SIM_IMUT1_ACC2_Y" : 10.364172,
            "SIM_IMUT1_ACC2_Z" : -7878.000000,
            "SIM_IMUT1_ACC3_X" : -0.514242,
            "SIM_IMUT1_ACC3_Y" : 0.862218,
            "SIM_IMUT1_ACC3_Z" : -234.000000,
            "SIM_IMUT1_GYR1_X" : -5122.513817,
            "SIM_IMUT1_GYR1_Y" : -3250.470428,
            "SIM_IMUT1_GYR1_Z" : -2136.346676,
            "SIM_IMUT1_GYR2_X" : 30.720505,
            "SIM_IMUT1_GYR2_Y" : 17.778447,
            "SIM_IMUT1_GYR2_Z" : 0.765997,
            "SIM_IMUT1_GYR3_X" : -0.003572,
            "SIM_IMUT1_GYR3_Y" : 0.036346,
            "SIM_IMUT1_GYR3_Z" : 0.015457,
            "SIM_IMUT1_TMAX"   : 70.0,
            "SIM_IMUT1_TMIN"   : -20.000000,
            "SIM_IMUT2_ENABLE" : 1,
            "SIM_IMUT2_ACC1_X" : -160000.000000,
            "SIM_IMUT2_ACC1_Y" : 198730.000000,
            "SIM_IMUT2_ACC1_Z" : 27812.000000,
            "SIM_IMUT2_ACC2_X" : 30.658159,
            "SIM_IMUT2_ACC2_Y" : 32.085022,
            "SIM_IMUT2_ACC2_Z" : 1572.000000,
            "SIM_IMUT2_ACC3_X" : 0.102912,
            "SIM_IMUT2_ACC3_Y" : 0.229734,
            "SIM_IMUT2_ACC3_Z" : 172.000000,
            "SIM_IMUT2_GYR1_X" : 3173.925644,
            "SIM_IMUT2_GYR1_Y" : -2368.312836,
            "SIM_IMUT2_GYR1_Z" : -1796.497177,
            "SIM_IMUT2_GYR2_X" : 13.029696,
            "SIM_IMUT2_GYR2_Y" : -10.349280,
            "SIM_IMUT2_GYR2_Z" : -15.082653,
            "SIM_IMUT2_GYR3_X" : 0.004831,
            "SIM_IMUT2_GYR3_Y" : -0.020528,
            "SIM_IMUT2_GYR3_Z" : 0.009469,
            "SIM_IMUT2_TMAX"   : 70.000000,
            "SIM_IMUT2_TMIN"   : -20.000000,
            "SIM_IMUT_END"     : 45.000000,
            "SIM_IMUT_START"   : 3.000000,
            "SIM_IMUT_TCONST"  : 75.000000,
            "SIM_DRIFT_SPEED"  : 0,
            "INS_GYR_CAL"      : 0,
        })

        self.set_parameter("SIM_IMUT_FIXED", 12)
        self.progress("Running accel cal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )
        self.progress("Running gyro cal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=1,
            timeout=5,
        )
        self.set_parameters({
            "SIM_IMUT_FIXED": 0,
            "INS_TCAL1_ENABLE": 2,
            "INS_TCAL1_TMAX": 42,
            "INS_TCAL2_ENABLE": 2,
            "INS_TCAL2_TMAX": 42,
            "SIM_SPEEDUP": 200,
        })
        self.set_parameter("LOG_DISARMED", 1)
        self.reboot_sitl()

        self.progress("Waiting for IMU temperature")
        self.assert_reach_imu_temperature(43, timeout=600)

        if self.get_parameter("INS_TCAL1_ENABLE") != 1.0:
            raise NotAchievedException("TCAL1 did not complete")
        if self.get_parameter("INS_TCAL2_ENABLE") != 1.0:
            raise NotAchievedException("TCAL2 did not complete")

        self.progress("Logging with calibration enabled")
        self.reboot_sitl()

        self.assert_reach_imu_temperature(43, timeout=600)

        self.progress("Testing with compensation enabled")

        test_temperatures = range(10, 45, 5)
        corrected = {}
        uncorrected = {}

        for temp in test_temperatures:
            self.progress("Testing temperature %.1f" % temp)
            self.set_parameter("SIM_IMUT_FIXED", temp)
            self.delay_sim_time(2)
            for msg in ['RAW_IMU', 'SCALED_IMU2']:
                m = self.assert_receive_message(msg, timeout=2)
                temperature = m.temperature*0.01

                if abs(temperature - temp) > 0.2:
                    raise NotAchievedException("incorrect %s temperature %.1f should be %.1f" % (msg, temperature, temp))

                accel = self.get_accelvec(m)
                gyro = self.get_gyrovec(m)
                accel2 = accel + Vector3(0, 0, 9.81)

                corrected[temperature] = (accel2, gyro)

        self.progress("Testing with compensation disabled")
        self.set_parameters({
            "INS_TCAL1_ENABLE": 0,
            "INS_TCAL2_ENABLE": 0,
        })

        gyro_threshold = 0.2
        accel_threshold = 0.2

        for temp in test_temperatures:
            self.progress("Testing temperature %.1f" % temp)
            self.set_parameter("SIM_IMUT_FIXED", temp)
            self.wait_heartbeat()
            self.wait_heartbeat()
            for msg in ['RAW_IMU', 'SCALED_IMU2']:
                m = self.assert_receive_message(msg, timeout=2)
                temperature = m.temperature*0.01

                if abs(temperature - temp) > 0.2:
                    raise NotAchievedException("incorrect %s temperature %.1f should be %.1f" % (msg, temperature, temp))

                accel = self.get_accelvec(m)
                gyro = self.get_gyrovec(m)

                accel2 = accel + Vector3(0, 0, 9.81)
                uncorrected[temperature] = (accel2, gyro)

        for temp in test_temperatures:
            (accel, gyro) = corrected[temp]
            self.progress("Corrected gyro at %.1f %s" % (temp, gyro))
            self.progress("Corrected accel at %.1f %s" % (temp, accel))

        for temp in test_temperatures:
            (accel, gyro) = uncorrected[temp]
            self.progress("Uncorrected gyro at %.1f %s" % (temp, gyro))
            self.progress("Uncorrected accel at %.1f %s" % (temp, accel))

        bad_value = False
        for temp in test_temperatures:
            (accel, gyro) = corrected[temp]
            if gyro.length() > gyro_threshold:
                raise NotAchievedException("incorrect corrected at %.1f gyro %s" % (temp, gyro))

            if accel.length() > accel_threshold:
                raise NotAchievedException("incorrect corrected at %.1f accel %s" % (temp, accel))

            (accel, gyro) = uncorrected[temp]
            if gyro.length() > gyro_threshold*2:
                bad_value = True

            if accel.length() > accel_threshold*2:
                bad_value = True

        if not bad_value:
            raise NotAchievedException("uncompensated IMUs did not vary enough")

        # the above tests change the internal persistent state of the
        # vehicle in ways that autotest doesn't track (magically set
        # parameters).  So wipe the vehicle's eeprom:
        self.reset_SITL_commandline()

    def EKFlaneswitch(self):
        '''Test EKF3 Affinity and Lane Switching'''

        # new lane switch available only with EK3
        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "EK3_AFFINITY": 15, # enable affinity for all sensors
            "EK3_IMU_MASK": 3, # use only 2 IMUs
            "GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "SIM_BARO_COUNT": 2,
            "SIM_BAR2_DISABLE": 0,
            "ARSPD2_TYPE": 2,
            "ARSPD2_USE": 1,
            "ARSPD2_PIN": 2,
        })

        # some parameters need reboot to take effect
        self.reboot_sitl()

        self.lane_switches = []

        # add an EKF lane switch hook
        def statustext_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            # example msg: EKF3 lane switch 1
            if not message.text.startswith("EKF3 lane switch "):
                return
            newlane = int(message.text[-1])
            self.lane_switches.append(newlane)
        self.install_message_hook_context(statustext_hook)

        # get flying
        self.takeoff(alt=50)
        self.change_mode('CIRCLE')

        ###################################################################
        self.progress("Checking EKF3 Lane Switching trigger from all sensors")
        ###################################################################
        self.start_subtest("ACCELEROMETER: Change z-axis offset")
        # create an accelerometer error by changing the Z-axis offset
        self.context_collect("STATUSTEXT")
        old_parameter = self.get_parameter("INS_ACCOFFS_Z")
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=self.set_parameter("INS_ACCOFFS_Z", old_parameter + 5),
            check_context=True)
        if self.lane_switches != [1]:
            raise NotAchievedException("Expected lane switch 1, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("INS_ACCOFFS_Z", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("BAROMETER: Freeze to last measured value")
        self.context_collect("STATUSTEXT")
        # create a barometer error by inhibiting any pressure change while changing altitude
        old_parameter = self.get_parameter("SIM_BAR2_FREEZE")
        self.set_parameter("SIM_BAR2_FREEZE", 1)
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=lambda: self.set_rc(2, 2000),
            check_context=True)
        if self.lane_switches != [1, 0]:
            raise NotAchievedException("Expected lane switch 0, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_rc(2, 1500)
        self.set_parameter("SIM_BAR2_FREEZE", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("GPS: Apply GPS Velocity Error in NED")
        self.context_push()
        self.context_collect("STATUSTEXT")

        # create a GPS velocity error by adding a random 2m/s
        # noise on each axis
        def sim_gps_verr():
            self.set_parameters({
                "SIM_GPS1_VERR_X": self.get_parameter("SIM_GPS1_VERR_X") + 2,
                "SIM_GPS1_VERR_Y": self.get_parameter("SIM_GPS1_VERR_Y") + 2,
                "SIM_GPS1_VERR_Z": self.get_parameter("SIM_GPS1_VERR_Z") + 2,
            })
        self.wait_statustext(text="EKF3 lane switch", timeout=30, the_function=sim_gps_verr, check_context=True)
        if self.lane_switches != [1, 0, 1]:
            raise NotAchievedException("Expected lane switch 1, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.context_pop()
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("MAGNETOMETER: Change X-Axis Offset")
        self.context_collect("STATUSTEXT")
        # create a magnetometer error by changing the X-axis offset
        old_parameter = self.get_parameter("SIM_MAG2_OFS_X")
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=self.set_parameter("SIM_MAG2_OFS_X", old_parameter + 150),
            check_context=True)
        if self.lane_switches != [1, 0, 1, 0]:
            raise NotAchievedException("Expected lane switch 0, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("SIM_MAG2_OFS_X", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("AIRSPEED: Fail to constant value")
        self.context_push()
        self.context_collect("STATUSTEXT")

        old_parameter = self.get_parameter("SIM_ARSPD_FAIL")

        def fail_speed():
            self.change_mode("GUIDED")
            loc = self.mav.location()
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p5=int(loc.lat * 1e7),
                p6=int(loc.lng * 1e7),
                p7=50,    # alt
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.delay_sim_time(5)
            # create an airspeed sensor error by freezing to the
            # current airspeed then changing the airspeed demand
            # to a higher value and waiting for the TECS speed
            # loop to diverge
            m = self.assert_receive_message('VFR_HUD')
            self.set_parameter("SIM_ARSPD_FAIL", m.airspeed)
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=0, # airspeed
                p2=30,
                p3=-1, # throttle / no change
                p4=0, # absolute values
            )
        self.wait_statustext(text="EKF3 lane switch", timeout=30, the_function=fail_speed, check_context=True)
        if self.lane_switches != [1, 0, 1, 0, 1]:
            raise NotAchievedException("Expected lane switch 1, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("SIM_ARSPD_FAIL", old_parameter)
        self.change_mode('CIRCLE')
        self.context_pop()
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.progress("GYROSCOPE: Change Y-Axis Offset")
        self.context_collect("STATUSTEXT")
        # create a gyroscope error by changing the Y-axis offset
        old_parameter = self.get_parameter("INS_GYR2OFFS_Y")
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=self.set_parameter("INS_GYR2OFFS_Y", old_parameter + 1),
            check_context=True)
        if self.lane_switches != [1, 0, 1, 0, 1, 0]:
            raise NotAchievedException("Expected lane switch 0, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("INS_GYR2OFFS_Y", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        ###################################################################

        self.disarm_vehicle(force=True)

    def FenceAltCeilFloor(self):
        '''Tests the fence ceiling and floor'''
        self.set_parameters({
            "FENCE_TYPE": 9,     # Set fence type to max and min alt
            "FENCE_ACTION": 0,   # Set action to report
            "FENCE_ALT_MAX": 200,
            "FENCE_ALT_MIN": 100,
        })

        # Grab Home Position
        self.wait_ready_to_arm()
        startpos = self.mav.location()

        cruise_alt = 150
        self.takeoff(cruise_alt)

        # note that while we enable the fence here, since the action
        # is set to report-only the fence continues to show as
        # not-enabled in the assert calls below
        self.do_fence_enable()

        self.progress("Fly above ceiling and check for breach")
        self.change_altitude(startpos.alt + cruise_alt + 80)
        self.assert_fence_sys_status(True, False, False)

        self.progress("Return to cruise alt")
        self.change_altitude(startpos.alt + cruise_alt)

        self.progress("Ensure breach has clearned")
        self.assert_fence_sys_status(True, False, True)

        self.progress("Fly below floor and check for breach")
        self.change_altitude(startpos.alt + cruise_alt - 80)

        self.progress("Ensure breach has clearned")
        self.assert_fence_sys_status(True, False, False)

        self.do_fence_disable()

        self.fly_home_land_and_disarm(timeout=150)

    def FenceMinAltAutoEnable(self):
        '''Tests autoenablement of the alt min fence and fences on arming'''
        self.set_parameters({
            "FENCE_TYPE": 9,     # Set fence type to min alt and max alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 25,
            "FENCE_ALT_MAX": 100,
            "FENCE_AUTOENABLE": 3,
            "FENCE_ENABLE" : 0,
            "RTL_AUTOLAND" : 2,
        })

        # check we can takeoff again
        for i in [1, 2]:
            # Grab Home Position
            self.wait_ready_to_arm()
            self.arm_vehicle()
            # max alt fence should now be enabled
            if i == 1:
                self.assert_fence_enabled()

            self.takeoff(alt=50, mode='TAKEOFF')
            self.change_mode("FBWA")
            self.set_rc(3, 1100)    # lower throttle

            self.progress("Waiting for RTL")
            tstart = self.get_sim_time()
            mode = "RTL"
            while not self.mode_is(mode, drain_mav=False):
                self.mav.messages['HEARTBEAT'].custom_mode
                self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                    self.mav.flightmode, mode, self.get_altitude(relative=True)))
                if (self.get_sim_time_cached() > tstart + 120):
                    raise WaitModeTimeout("Did not change mode")
            self.progress("Got mode %s" % mode)
            self.fly_home_land_and_disarm()
            self.change_mode("FBWA")
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)
            self.set_current_waypoint(0, check_afterwards=False)
            self.set_rc(3, 1000)    # lower throttle

    def FenceMinAltEnableAutoland(self):
        '''Tests autolanding when alt min fence is enabled'''
        self.set_parameters({
            "FENCE_TYPE": 12,     # Set fence type to min alt and max alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 20,
            "FENCE_AUTOENABLE": 0,
            "FENCE_ENABLE" : 1,
            "RTL_AUTOLAND" : 2,
        })

        # Grab Home Position
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.takeoff(alt=50, mode='TAKEOFF')
        self.change_mode("FBWA")
        self.set_rc(3, 1100)    # lower throttle

        self.progress("Waiting for RTL")
        tstart = self.get_sim_time()
        mode = "RTL"
        while not self.mode_is(mode, drain_mav=False):
            self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                self.mav.flightmode, mode, self.get_altitude(relative=True)))
            if (self.get_sim_time_cached() > tstart + 120):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)
        # switch to FBWA
        self.change_mode("FBWA")
        self.set_rc(3, 1500)    # raise throttle
        self.wait_altitude(25, 35, timeout=50, relative=True)
        self.set_rc(3, 1000)    # lower throttle
        # Now check we can land
        self.fly_home_land_and_disarm()

    def FenceMinAltAutoEnableAbort(self):
        '''Tests autoenablement of the alt min fence and fences on arming'''
        self.set_parameters({
            "FENCE_TYPE": 8,     # Set fence type to min alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 25,
            "FENCE_ALT_MAX": 100,
            "FENCE_AUTOENABLE": 3,
            "FENCE_ENABLE" : 0,
            "RTL_AUTOLAND" : 2,
        })

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.takeoff(alt=50, mode='TAKEOFF')
        self.change_mode("FBWA")
        # min alt fence should now be enabled
        self.assert_fence_enabled()
        self.set_rc(3, 1100)    # lower throttle

        self.progress("Waiting for RTL")
        tstart = self.get_sim_time()
        mode = "RTL"
        while not self.mode_is(mode, drain_mav=False):
            self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                self.mav.flightmode, mode, self.get_altitude(relative=True)))
            if (self.get_sim_time_cached() > tstart + 120):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)

        self.load_generic_mission("flaps.txt")
        self.change_mode("AUTO")
        self.wait_distance_to_waypoint(8, 100, 10000000)
        self.set_current_waypoint(8)
        # abort the landing
        self.wait_altitude(10, 20, timeout=200, relative=True)
        self.change_mode("CRUISE")
        self.set_rc(2, 1200)
        # self.set_rc(3, 1600)    # raise throttle
        self.wait_altitude(30, 40, timeout=200, relative=True)
        # min alt fence should now be re-enabled
        self.assert_fence_enabled()

        self.change_mode("AUTO")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)
        self.fly_home_land_and_disarm(timeout=150)

    def FenceAutoEnableDisableSwitch(self):
        '''Tests autoenablement of regular fences and manual disablement'''
        self.set_parameters({
            "FENCE_TYPE": 9,     # Set fence type to min alt, max alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 50,
            "FENCE_ALT_MAX": 100,
            "FENCE_AUTOENABLE": 2,
            "FENCE_OPTIONS" : 1,
            "FENCE_ENABLE" : 1,
            "FENCE_RADIUS" : 300,
            "FENCE_RET_ALT" : 0,
            "FENCE_RET_RALLY" : 0,
            "FENCE_TOTAL" : 0,
            "RTL_ALTITUDE" : 75,
            "TKOFF_ALT" : 75,
            "RC7_OPTION" : 11,   # AC_Fence uses Aux switch functionality
        })
        self.reboot_sitl()
        self.context_collect("STATUSTEXT")

        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        # Grab Home Position
        self.assert_receive_message('HOME_POSITION')
        self.set_rc(7, 1000) # Turn fence off with aux function, does not impact later auto-enable

        self.wait_ready_to_arm()

        self.progress("Check fence disabled at boot")
        m = self.assert_receive_message('SYS_STATUS')
        if (m.onboard_control_sensors_enabled & fence_bit):
            raise NotAchievedException("Fence is enabled at boot")

        cruise_alt = 75
        self.takeoff(cruise_alt, mode='TAKEOFF')

        self.progress("Fly above ceiling and check there is a breach")
        self.change_mode('FBWA')
        self.set_rc(3, 2000)
        self.set_rc(2, 1000)

        self.wait_statustext("Max Alt fence breached", timeout=10, check_context=True)
        self.wait_mode('RTL')

        m = self.assert_receive_message('SYS_STATUS')
        if (m.onboard_control_sensors_health & fence_bit):
            raise NotAchievedException("Fence ceiling not breached")

        self.set_rc(3, 1500)
        self.set_rc(2, 1500)

        self.progress("Wait for RTL alt reached")
        self.wait_altitude(cruise_alt-5, cruise_alt+5, relative=True, timeout=30)

        self.progress("Return to cruise alt")
        self.set_rc(3, 1500)
        self.change_altitude(cruise_alt, relative=True)

        self.progress("Check fence breach cleared")
        m = self.assert_receive_message('SYS_STATUS')
        if (not (m.onboard_control_sensors_health & fence_bit)):
            raise NotAchievedException("Fence breach not cleared")

        self.progress("Fly above ceiling and check there is a breach and switch to AUTOLAND mode")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "FENCE_ACTION": 8,   # Set action to AUTOLAND if possible
        })

        self.set_rc(3, 2000)
        self.set_rc(2, 1000)

        self.wait_statustext("Max Alt fence breached", timeout=10, check_context=True)
        self.wait_mode(26) # AUTOLAND,need pymavlink .43 to use text name
        self.assert_fence_sys_status(True, True, False)

        self.set_rc(3, 1500)
        self.set_rc(2, 1500)

        self.progress("Wait for cruise alt reached")
        self.wait_altitude(
            cruise_alt-5,
            cruise_alt+5,
            relative=True,
            timeout=30,
        )

        self.progress("Check fence breach cleared")
        self.assert_fence_sys_status(True, True, True)

        self.context_pop()
        self.change_mode('FBWA')
        self.progress("Fly below floor and check for breach")
        self.set_rc(2, 2000)
        self.wait_statustext("Min Alt fence breached", timeout=10, check_context=True)
        self.wait_mode("RTL")
        m = self.assert_receive_message('SYS_STATUS')
        if (m.onboard_control_sensors_health & fence_bit):
            raise NotAchievedException("Fence floor not breached")

        self.change_mode("FBWA")

        self.progress("Fly above floor and check fence is enabled")
        self.set_rc(3, 2000)
        self.change_altitude(75, relative=True)
        m = self.assert_receive_message('SYS_STATUS')
        if (not (m.onboard_control_sensors_enabled & fence_bit)):
            raise NotAchievedException("Fence Floor not enabled")

        self.progress("Toggle fence enable/disable")
        self.set_rc(7, 2000)
        self.delay_sim_time(2)
        self.set_rc(7, 1000)
        self.delay_sim_time(2)

        self.progress("Check fence is disabled")
        m = self.assert_receive_message('SYS_STATUS')
        if (m.onboard_control_sensors_enabled & fence_bit):
            raise NotAchievedException("Fence disable with switch failed")

        self.progress("Fly below floor and check for no breach")
        self.change_altitude(40, relative=True)
        m = self.assert_receive_message('SYS_STATUS')
        if (not (m.onboard_control_sensors_health & fence_bit)):
            raise NotAchievedException("Fence floor breached")

        self.progress("Return to cruise alt")
        self.set_rc(3, 1500)
        self.change_altitude(cruise_alt, relative=True)
        self.fly_home_land_and_disarm(timeout=250)

    def FenceCircleExclusionAutoEnable(self):
        '''Tests autolanding when alt min fence is enabled'''
        self.set_parameters({
            "FENCE_TYPE": 2,     # Set fence type to circle
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_AUTOENABLE": 2,
            "FENCE_ENABLE" : 0,
            "RTL_AUTOLAND" : 2,
        })

        fence_loc = self.home_position_as_mav_location()
        self.location_offset_ne(fence_loc, 300, 0)

        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, {
                "radius" : 100,
                "loc" : fence_loc
            }
        )])

        self.takeoff(alt=50, mode='TAKEOFF')
        self.change_mode("FBWA")
        self.set_rc(3, 1100)    # lower throttle

        self.progress("Waiting for RTL")
        self.wait_mode('RTL')
        # Now check we can land
        self.fly_home_land_and_disarm()

    def FenceEnableDisableSwitch(self):
        '''Tests enablement and disablement of fences on a switch'''
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.set_parameters({
            "FENCE_TYPE": 4,     # Set fence type to polyfence
            "FENCE_ACTION": 6,   # Set action to GUIDED
            "FENCE_ALT_MIN": 10,
            "FENCE_ENABLE" : 0,
            "RC7_OPTION" : 11,   # AC_Fence uses Aux switch functionality
        })

        self.progress("Checking fence is not present before being configured")
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Got (%s)" % str(m))
        if (m.onboard_control_sensors_enabled & fence_bit):
            raise NotAchievedException("Fence enabled before being configured")

        self.wait_ready_to_arm()
        # takeoff at a lower altitude to avoid immediately breaching polyfence
        self.takeoff(alt=25)
        self.change_mode("FBWA")

        self.load_fence("CMAC-fence.txt")

        self.set_rc_from_map({
            3: 1500,
            7: 2000,
        }) # Turn fence on with aux function

        m = self.assert_receive_message('FENCE_STATUS', timeout=2, verbose=True)

        self.progress("Checking fence is initially OK")
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
                               present=True,
                               enabled=True,
                               healthy=True,
                               verbose=False,
                               timeout=30)

        self.progress("Waiting for GUIDED")
        tstart = self.get_sim_time()
        mode = "GUIDED"
        while not self.mode_is(mode, drain_mav=False):
            self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                self.mav.flightmode, mode, self.get_altitude(relative=True)))
            if (self.get_sim_time_cached() > tstart + 120):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)
        # check we are in breach
        self.assert_fence_enabled()

        self.set_rc_from_map({
            7: 1000,
        }) # Turn fence off with aux function

        # wait to no longer be in breach
        self.delay_sim_time(5)
        self.assert_fence_disabled()

        self.fly_home_land_and_disarm(timeout=250)
        self.do_fence_disable() # Ensure the fence is disabled after test

    def FenceEnableDisableAux(self):
        '''Tests enablement and disablement of fences via aux command'''
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        enable = 0
        self.set_parameters({
            "FENCE_TYPE": 12,     # Set fence type to polyfence + AltMin
            "FENCE_ALT_MIN": 10,
            "FENCE_ENABLE" : enable,
        })

        if not enable:
            self.progress("Checking fence is not present before being configured")
            m = self.assert_receive_message('SYS_STATUS')
            self.progress("Got (%s)" % str(m))
            if (m.onboard_control_sensors_enabled & fence_bit):
                raise NotAchievedException("Fence enabled before being configured")

        self.load_fence("CMAC-fence.txt")

        self.wait_ready_to_arm()
        # takeoff at a lower altitude to avoid immediately breaching polyfence
        self.takeoff(alt=25)
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        self.run_auxfunc(
            11,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        m = self.assert_receive_message('FENCE_STATUS', timeout=2, verbose=True)

        self.progress("Checking fence is initially OK")
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
                               present=True,
                               enabled=True,
                               healthy=True,
                               verbose=False,
                               timeout=30)

        self.progress("Waiting for RTL")
        tstart = self.get_sim_time()
        mode = "RTL"
        while not self.mode_is(mode, drain_mav=False):
            self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                self.mav.flightmode, mode, self.get_altitude(relative=True)))
            if (self.get_sim_time_cached() > tstart + 120):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)
        # check we are in breach
        self.assert_fence_enabled()
        self.assert_fence_sys_status(True, True, False)

        # wait until we get home
        self.wait_distance_to_home(50, 100, timeout=200)
        # now check we are now not in breach
        self.assert_fence_sys_status(True, True, True)
        # Turn fence off with aux function
        self.run_auxfunc(
            11,
            0,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        # switch back to cruise
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        # re-enable the fences
        self.run_auxfunc(
            11,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        m = self.assert_receive_message('FENCE_STATUS', timeout=2, verbose=True)

        self.progress("Waiting for RTL")
        tstart = self.get_sim_time()
        mode = "RTL"
        while not self.mode_is(mode, drain_mav=False):
            self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                self.mav.flightmode, mode, self.get_altitude(relative=True)))
            if (self.get_sim_time_cached() > tstart + 120):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)

        # wait to no longer be in breach
        self.wait_distance_to_home(50, 100, timeout=200)
        self.assert_fence_sys_status(True, True, True)

        # fly home and land with fences still enabled
        self.fly_home_land_and_disarm(timeout=250)
        self.do_fence_disable() # Ensure the fence is disabled after test

    def FenceBreachedChangeMode(self):
        '''Tests manual mode change after fence breach, as set with FENCE_OPTIONS'''
        """ Attempts to change mode while a fence is breached.
            mode should change should fail if fence option bit is set"""
        self.set_parameters({
            "FENCE_ACTION": 1,
            "FENCE_TYPE": 4,
        })
        home_loc = self.mav.location()
        locs = [
            mavutil.location(home_loc.lat - 0.001, home_loc.lng - 0.001, 0, 0),
            mavutil.location(home_loc.lat - 0.001, home_loc.lng + 0.001, 0, 0),
            mavutil.location(home_loc.lat + 0.001, home_loc.lng + 0.001, 0, 0),
            mavutil.location(home_loc.lat + 0.001, home_loc.lng - 0.001, 0, 0),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, locs),
        ])
        self.delay_sim_time(1)
        self.wait_ready_to_arm()
        self.takeoff(alt=50)
        self.change_mode("CRUISE")
        self.wait_distance(250, accuracy=15)

        self.progress("Enable fence and initiate fence action")
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.wait_mode("RTL") # We should RTL because of fence breach

        self.progress("User mode change to cruise should retrigger fence action")
        try:
            # mode change should time out, 'WaitModeTimeout' exception is the desired result
            # can't wait too long or the vehicle will be inside fence and allow the mode change
            self.change_mode("CRUISE", timeout=10)
            raise NotAchievedException("Should not change mode in fence breach")
        except WaitModeTimeout:
            pass
        except Exception as e:
            raise e

        # enable mode change
        self.set_parameter("FENCE_OPTIONS", 0)
        self.progress("Check user mode change to LOITER is allowed")
        self.change_mode("LOITER")

        # Fly for 20 seconds and make sure still in LOITER mode
        self.delay_sim_time(20)
        if not self.mode_is("LOITER"):
            raise NotAchievedException("Fence should not re-trigger")

        # reset options parameter
        self.set_parameter("FENCE_OPTIONS", 1)

        self.progress("Test complete, disable fence and come home")
        self.do_fence_disable()
        self.fly_home_land_and_disarm()

    def FenceNoFenceReturnPoint(self):
        '''Tests calculated return point during fence breach when no fence return point present'''
        """ Attempts to change mode while a fence is breached.
            This should revert to the mode specified by the fence action. """
        want_radius = 100 # Fence Return Radius
        self.set_parameters({
            "FENCE_ACTION": 6,
            "FENCE_TYPE": 4,
            "RTL_RADIUS": want_radius,
            "NAVL1_LIM_BANK": 60,
        })
        home_loc = self.mav.location()
        locs = [
            mavutil.location(home_loc.lat - 0.003, home_loc.lng - 0.001, 0, 0),
            mavutil.location(home_loc.lat - 0.003, home_loc.lng + 0.003, 0, 0),
            mavutil.location(home_loc.lat + 0.001, home_loc.lng + 0.003, 0, 0),
            mavutil.location(home_loc.lat + 0.001, home_loc.lng - 0.001, 0, 0),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, locs),
        ])
        self.delay_sim_time(1)
        self.wait_ready_to_arm()
        self.takeoff(alt=50)
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        self.progress("Enable fence and initiate fence action")
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.wait_mode("GUIDED", timeout=120) # We should RTL because of fence breach
        self.delay_sim_time(60)

        items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(items) != 4:
            raise NotAchievedException("Unexpected fencepoint count (want=%u got=%u)" % (4, len(items)))

        # Check there are no fence return points specified still
        for fence_loc in items:
            if fence_loc.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
                raise NotAchievedException(
                    "Unexpected fence return point found (%u) got %u" %
                    (fence_loc.command,
                     mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT))

        # Work out the approximate return point when no fence return point present
        # Logic taken from AC_PolyFence_loader.cpp
        min_loc = self.mav.location()
        max_loc = self.mav.location()
        for new_loc in locs:
            if new_loc.lat < min_loc.lat:
                min_loc.lat = new_loc.lat
            if new_loc.lng < min_loc.lng:
                min_loc.lng = new_loc.lng
            if new_loc.lat > max_loc.lat:
                max_loc.lat = new_loc.lat
            if new_loc.lng > max_loc.lng:
                max_loc.lng = new_loc.lng

        # Generate the return location based on min and max locs
        ret_lat = (min_loc.lat + max_loc.lat) / 2
        ret_lng = (min_loc.lng + max_loc.lng) / 2
        ret_loc = mavutil.location(ret_lat, ret_lng, 0, 0)
        self.progress("Return loc: (%s)" % str(ret_loc))

        # Wait for guided return to vehicle calculated fence return location
        self.wait_distance_to_location(ret_loc, 90, 110)
        self.wait_circling_point_with_radius(ret_loc, 92)

        self.progress("Test complete, disable fence and come home")
        self.do_fence_disable()
        self.fly_home_land_and_disarm()

    def FenceNoFenceReturnPointInclusion(self):
        '''Tests using home as fence return point when none is present, and no inclusion fence is uploaded'''
        """ Test result when a breach occurs and No fence return point is present and
            no inclusion fence is present and exclusion fence is present """
        want_radius = 100 # Fence Return Radius

        self.set_parameters({
            "FENCE_ACTION": 6,
            "FENCE_TYPE": 2,
            "FENCE_RADIUS": 300,
            "RTL_RADIUS": want_radius,
            "NAVL1_LIM_BANK": 60,
        })

        self.clear_fence()

        self.delay_sim_time(1)
        self.wait_ready_to_arm()
        home_loc = self.mav.location()
        self.takeoff(alt=50)
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        self.progress("Enable fence and initiate fence action")
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.wait_mode("GUIDED") # We should RTL because of fence breach
        self.delay_sim_time(30)

        items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(items) != 0:
            raise NotAchievedException("Unexpected fencepoint count (want=%u got=%u)" % (0, len(items)))

        # Check there are no fence return points specified still
        for fence_loc in items:
            if fence_loc.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
                raise NotAchievedException(
                    "Unexpected fence return point found (%u) got %u" %
                    (fence_loc.command,
                     mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT))

        # Wait for guided return to vehicle calculated fence return location
        self.wait_distance_to_location(home_loc, 90, 110)
        self.wait_circling_point_with_radius(home_loc, 92)

        self.progress("Test complete, disable fence and come home")
        self.do_fence_disable()
        self.fly_home_land_and_disarm()

    def FenceDisableUnderAction(self):
        '''Tests Disabling fence while undergoing action caused by breach'''
        """ Fence breach will cause the vehicle to enter guided mode.
            Upon breach clear, check the vehicle is in the expected mode"""
        self.set_parameters({
            "FENCE_ALT_MIN": 50, # Sets the fence floor
            "FENCE_TYPE": 8,     # Only use fence floor for breaches
        })
        self.wait_ready_to_arm()

        def attempt_fence_breached_disable(start_mode, end_mode, expected_mode, action):
            self.set_parameter("FENCE_ACTION", action)   # Set Fence Action to Guided
            self.change_mode(start_mode)
            self.arm_vehicle()
            self.do_fence_enable()
            self.assert_fence_enabled()
            self.wait_mode(expected_mode)
            self.do_fence_disable()
            self.assert_fence_disabled()
            self.wait_mode(end_mode)
            self.disarm_vehicle(force=True)

        attempt_fence_breached_disable(start_mode="FBWA", end_mode="RTL", expected_mode="RTL", action=1)
        attempt_fence_breached_disable(start_mode="FBWA", end_mode="FBWA", expected_mode="GUIDED", action=6)
        attempt_fence_breached_disable(start_mode="FBWA", end_mode="FBWA", expected_mode="GUIDED", action=7)

    def _MAV_CMD_DO_AUX_FUNCTION(self, run_cmd):
        '''Test triggering Auxiliary Functions via mavlink'''
        self.context_collect('STATUSTEXT')
        self.run_auxfunc(64, 2, run_cmd=run_cmd)  # 64 == reverse throttle
        self.wait_statustext("RevThrottle: ENABLE", check_context=True)
        self.run_auxfunc(64, 0, run_cmd=run_cmd)
        self.wait_statustext("RevThrottle: DISABLE", check_context=True)
        self.run_auxfunc(65, 2, run_cmd=run_cmd)  # 65 == GPS_DISABLE

        self.start_subtest("Bad auxfunc")
        self.run_auxfunc(
            65231,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            run_cmd=run_cmd,
        )

        self.start_subtest("Bad switchpos")
        self.run_auxfunc(
            62,
            17,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
            run_cmd=run_cmd,
        )

    def MAV_CMD_DO_AUX_FUNCTION(self):
        '''Test triggering Auxiliary Functions via mavlink'''
        self._MAV_CMD_DO_AUX_FUNCTION(run_cmd=self.run_cmd)
        self._MAV_CMD_DO_AUX_FUNCTION(run_cmd=self.run_cmd_int)

    def FlyEachFrame(self):
        '''Fly each supported internal frame'''
        vinfo = vehicleinfo.VehicleInfo()
        vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
            "plane-tailsitter": "unstable in hover; unflyable in cruise",
            "quadplane-can" : "needs CAN periph",
            "stratoblimp" : "not expected to fly normally",
            "glider" : "needs balloon lift",
        }
        for frame in sorted(vinfo_options["frames"].keys()):
            self.start_subtest("Testing frame (%s)" % str(frame))
            if frame in known_broken_frames:
                self.progress("Actually, no I'm not - it is known-broken (%s)" %
                              (known_broken_frames[frame]))
                continue
            frame_bits = vinfo_options["frames"][frame]
            print("frame_bits: %s" % str(frame_bits))
            if frame_bits.get("external", False):
                self.progress("Actually, no I'm not - it is an external simulation")
                continue
            model = frame_bits.get("model", frame)
            defaults = self.model_defaults_filepath(frame)
            if not isinstance(defaults, list):
                defaults = [defaults]
            self.customise_SITL_commandline(
                [],
                defaults_filepath=defaults,
                model=model,
                wipe=True,
            )
            mission_file = "basic.txt"
            quadplane = self.get_parameter('Q_ENABLE')
            if quadplane:
                mission_file = "basic-quadplane.txt"
            tailsitter = self.get_parameter('Q_TAILSIT_ENABLE')
            if tailsitter:
                # tailsitter needs extra re-boot to pick up the rotated AHRS view
                self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.fly_mission(mission_file, strict=False, quadplane=quadplane, mission_timeout=400.0)
            self.wait_disarmed()

    def AutoLandMode(self):
        '''Test AUTOLAND mode'''
        self.set_parameters({
            "AUTOLAND_DIR_OFF": 45,
            "TERRAIN_FOLLOW": 1,
            "AUTOLAND_CLIMB": 300,
        })
        self.customise_SITL_commandline(["--home", "-35.362938,149.165085,585,173"])
        self.context_collect('STATUSTEXT')
        self.load_mission("autoland_mission.txt")
        self.install_terrain_handlers_context()
        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_text("Autoland direction", check_context=True)
        self.wait_waypoint(2, 2, max_dist=100)
        self.change_mode(26) # AUTOLAND need .43 pymavlink to use text name
        self.wait_disarmed(400)
        self.progress("Check the landed heading matches takeoff plus offset")
        self.wait_heading(218, accuracy=5, timeout=1)
        loc = mavutil.location(-35.362938, 149.165085, 585, 218)
        if self.get_distance(loc, self.mav.location()) > 35:
            raise NotAchievedException("Did not land close to home")
        self.set_parameters({
            "TKOFF_OPTIONS": 2,
        })
        self.wait_ready_to_arm()
        self.set_autodisarm_delay(0)
        self.arm_vehicle()
        self.progress("Check the set dir on arm option")
        self.wait_text("Autoland direction", check_context=True)

    def RCDisableAirspeedUse(self):
        '''Test RC DisableAirspeedUse option'''
        self.set_parameter("RC9_OPTION", 106)
        self.delay_sim_time(5)
        self.set_rc(9, 1000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            True,
            True)
        self.set_rc(9, 2000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            False,
            True)
        self.set_rc(9, 1000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            True,
            True)

    def WatchdogHome(self):
        '''Ensure home is restored after watchdog reset'''
        if self.gdb:
            # we end up signalling the wrong process.  I think.
            # Probably need to have a "sitl_pid()" method to get the
            # ardupilot process's PID.
            self.progress("######## Skipping WatchdogHome test under GDB")
            return

        ex = None
        try:
            self.progress("Enabling watchdog")
            self.set_parameter("BRD_OPTIONS", 1 << 0)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.progress("Explicitly setting home to a known location")
            orig_home = self.poll_home_position()
            new_home = orig_home
            new_home.latitude = new_home.latitude + 1000
            new_home.longitude = new_home.longitude + 2000
            new_home.altitude = new_home.altitude + 300000 # 300 metres
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                p5=new_home.latitude,
                p6=new_home.longitude,
                p7=new_home.altitude/1000.0, # mm => m
            )
            old_bootcount = self.get_parameter('STAT_BOOTCNT')
            self.progress("Forcing watchdog reset")
            os.kill(self.sitl.pid, signal.SIGALRM)
            self.detect_and_handle_reboot(old_bootcount)
            self.wait_statustext("WDG:")
            self.wait_statustext("IMU1 is using GPS")  # won't be come armable
            self.progress("Verifying home position")
            post_reboot_home = self.poll_home_position()
            delta = self.get_distance_int(new_home, post_reboot_home)
            max_delta = 1
            if delta > max_delta:
                raise NotAchievedException(
                    "New home not where it should be (dist=%f) (want=%s) (got=%s)" %
                    (delta, str(new_home), str(post_reboot_home)))
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.reboot_sitl()

        if ex is not None:
            raise ex

    def AUTOTUNE(self):
        '''Test AutoTune mode'''
        self.run_autotune()

        # Values that are set to constants
        # If these are changed then the expected tune parameters should also change
        self.assert_parameter_value_pct("RLL2SRV_TCONST", 0.5, 0)
        self.assert_parameter_value_pct("RLL2SRV_RMAX", 75, 0)
        self.assert_parameter_value_pct("RLL_RATE_IMAX", 0.666, 0.01) # allow some small error to account for floating point stuff  # noqa:E501
        self.assert_parameter_value_pct("RLL_RATE_FLTT", 3.183, 0.01)
        self.assert_parameter_value_pct("RLL_RATE_FLTE", 0, 0)
        self.assert_parameter_value_pct("RLL_RATE_FLTD", 10.0, 0)
        self.assert_parameter_value_pct("RLL_RATE_SMAX", 150.0, 0)

        self.assert_parameter_value_pct("PTCH2SRV_TCONST", 0.75, 0)
        self.assert_parameter_value_pct("PTCH2SRV_RMAX_UP", 75, 0)
        self.assert_parameter_value_pct("PTCH2SRV_RMAX_DN", 75, 0)
        self.assert_parameter_value_pct("PTCH_RATE_IMAX", 0.666, 0.01)
        self.assert_parameter_value_pct("PTCH_RATE_FLTT", 2.122, 0.01)
        self.assert_parameter_value_pct("PTCH_RATE_FLTE", 0, 0)
        self.assert_parameter_value_pct("PTCH_RATE_FLTD", 10, 0)
        self.assert_parameter_value_pct("PTCH_RATE_SMAX", 150, 0)

        # Check tuned values, targets derived from running tests multiple times and taking average
        # Expect within 2%
        # Note that I is not checked directly, its value is derived from P, FF, and TCONST which are all checked.
        self.assert_parameter_value_pct("RLL_RATE_P", 1.222702146, 2)
        self.assert_parameter_value_pct("RLL_RATE_FF", 0.229291457, 2)

        self.assert_parameter_value_pct("PTCH_RATE_FF", 0.503520715, 5)

        # there are sometimes multiple solutions for roll but the distribution
        # is much more skewed than pitch below
        try:
            self.assert_parameter_value_pct("RLL_RATE_D", 0.070284024, 2)
        except ValueError:
            self.assert_parameter_value_pct("RLL_RATE_D", 0.091369226, 2) # added 2025-10

        # There seem to be multiple solutions for pitch. I'm not sure why this is.
        # Each value is quite consistent because of the fixed steps that autotune takes
        try:
            # Expect this about 84% of the time
            self.assert_parameter_value_pct("PTCH_RATE_P", 1.746079683, 2)
        except ValueError:
            try:
                # 12%
                self.assert_parameter_value_pct("PTCH_RATE_P", 1.343138218, 2)
            except ValueError:
                # 4%
                self.assert_parameter_value_pct("PTCH_RATE_P", 2.26990366, 2)

        try:
            # 64%
            self.assert_parameter_value_pct("PTCH_RATE_D", 0.108, 2)
        except ValueError:
            try:
                # 28%
                self.assert_parameter_value_pct("PTCH_RATE_D", 0.141, 2)
            except ValueError:
                try:
                    # 4%
                    self.assert_parameter_value_pct("PTCH_RATE_D", 0.049, 2)
                except ValueError:
                    # 4%
                    try:
                        self.assert_parameter_value_pct("PTCH_RATE_D", 0.0836, 2)
                    except ValueError:
                        self.assert_parameter_value_pct("PTCH_RATE_D", 0.0380, 2) # added 2025-10

    def run_autotune(self):
        self.takeoff(100)
        self.change_mode('AUTOTUNE')
        self.context_collect('STATUSTEXT')
        tstart = self.get_sim_time()
        axis = "Roll"
        rc_value = 1000
        while True:
            timeout = 600
            if self.get_sim_time() - tstart > timeout:
                raise NotAchievedException("Did not complete within %u seconds" % timeout)
            try:
                m = self.wait_statustext("%s: Finished" % axis, check_context=True, timeout=0.1)
                self.progress("Got %s" % str(m))
                if axis == "Roll":
                    axis = "Pitch"
                    # Center sticks to allow roll to return to neutral before starting pitch
                    self.set_rc(1, 1500)
                    self.set_rc(2, 1500)
                    self.delay_sim_time(15)

                    # Reset toggle value so the initial input is in a consistent direction
                    rc_value = 1000

                elif axis == "Pitch":
                    break
                else:
                    raise ValueError("Bug: %s" % axis)
            except AutoTestTimeoutException:
                pass
            self.delay_sim_time(1)

            if rc_value == 1000:
                rc_value = 2000
            elif rc_value == 2000:
                rc_value = 1000
            elif rc_value == 1000:
                rc_value = 2000
            else:
                raise ValueError("Bug")

            if axis == "Roll":
                self.set_rc(1, rc_value)
                self.set_rc(2, 1500)
            elif axis == "Pitch":
                self.set_rc(1, 1500)
                self.set_rc(2, rc_value)
            else:
                raise ValueError("Bug")

        tdelta = self.get_sim_time() - tstart
        self.progress("Finished in %0.1f seconds" % (tdelta,))

        self.set_rc(1, 1500)
        self.set_rc(2, 1500)

        self.change_mode('FBWA')
        self.fly_home_land_and_disarm(timeout=tdelta+240)

    def AutotuneFiltering(self):
        '''Test AutoTune mode with filter updates disabled'''
        self.set_parameters({
            "AUTOTUNE_OPTIONS": 3,
            # some filtering is required for autotune to complete
            "RLL_RATE_FLTD": 10,
            "PTCH_RATE_FLTD": 10,
            "RLL_RATE_FLTT": 20,
            "PTCH_RATE_FLTT": 20,
        })
        self.run_autotune()

    def LandingDrift(self):
        '''Circuit with baro drift'''
        self.customise_SITL_commandline([], wipe=True)

        self.set_analog_rangefinder_parameters()

        self.set_parameters({
            "SIM_BARO_DRIFT": -0.02,
            "SIM_TERRAIN": 0,
            "RNGFND_LANDING": 1,
            "LAND_SLOPE_RCALC": 2,
            "LAND_ABORT_DEG": 1,
        })

        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Load and start mission
        self.load_mission("ap-circuit.txt", strict=True)
        self.set_current_waypoint(1, check_afterwards=True)
        self.change_mode('AUTO')
        self.wait_current_waypoint(1, timeout=5)
        self.wait_groundspeed(0, 10, timeout=5)

        # Wait for landing waypoint
        self.wait_current_waypoint(9, timeout=1200)

        # Wait for landing restart
        self.wait_current_waypoint(5, timeout=60)

        # Wait for landing waypoint (second attempt)
        self.wait_current_waypoint(9, timeout=1200)

        self.wait_disarmed(timeout=180)

    def TakeoffAuto1(self):
        '''Test the behaviour of an AUTO takeoff, pt1.'''
        '''
        Conditions:
        - ARSPD_USE=1
        - TKOFF_OPTIONS[0]=0
        - TKOFF_THR_MAX < THR_MAX
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 1.0,
            "THR_MAX": 100.0,
            "TKOFF_THR_MAX": 80.0,
            "TKOFF_THR_MINACC": 3.0,
            "TECS_PITCH_MAX": 35.0,
            "PTCH_LIM_MAX_DEG": 35.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
        })

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.wait_ready_to_arm()

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.load_mission("catapult.txt", strict=True)
        self.change_mode('AUTO')

        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Wait until we're midway through the climb.
        test_alt = 50
        self.wait_altitude(test_alt, test_alt+2, relative=True)

        # Ensure that by then the aircraft still goes at max allowed throttle.
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for landing waypoint.
        self.wait_current_waypoint(11, timeout=1200)
        self.wait_disarmed(120)

    def TakeoffAuto2(self):
        '''Test the behaviour of an AUTO takeoff, pt2.'''
        '''
        Conditions:
        - ARSPD_USE=0
        - TKOFF_OPTIONS[0]=0
        - TKOFF_THR_MAX > THR_MAX
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "THR_MAX": 80.0,
            "TKOFF_THR_MAX": 100.0,
            "TKOFF_THR_MINACC": 3.0,
            "TECS_PITCH_MAX": 35.0,
            "PTCH_LIM_MAX_DEG": 35.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
        })

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.wait_ready_to_arm()

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.load_mission("catapult.txt", strict=True)
        self.change_mode('AUTO')

        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Wait until we're midway through the climb.
        test_alt = 50
        self.wait_altitude(test_alt, test_alt+2, relative=True)

        # Ensure that by then the aircraft still goes at max allowed throttle.
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for landing waypoint.
        self.wait_current_waypoint(11, timeout=1200)
        self.wait_disarmed(120)

    def TakeoffAuto3(self):
        '''Test the behaviour of an AUTO takeoff, pt3.'''
        '''
        Conditions:
        - ARSPD_USE=1
        - TKOFF_OPTIONS[0]=1
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 1.0,
            "THR_MAX": 80.0,
            "THR_MIN": 0.0,
            "TKOFF_OPTIONS": 1.0,
            "TKOFF_THR_MAX": 100.0,
            "TKOFF_THR_MINACC": 3.0,
            "TECS_PITCH_MAX": 35.0,
            "TKOFF_THR_MAX_T": 3.0,
            "PTCH_LIM_MAX_DEG": 35.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
        })

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.wait_ready_to_arm()

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.load_mission("catapult.txt", strict=True)
        self.change_mode('AUTO')

        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Ensure that TKOFF_THR_MAX_T is respected.
        self.delay_sim_time(self.get_parameter("TKOFF_THR_MAX_T")-1)
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Ensure that after that the aircraft does not go full throttle anymore.
        test_alt = 50
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        w = vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            3,  # throttle
            1000+10*self.get_parameter("TKOFF_THR_MAX")-10,
            comparator=operator.lt,
            minimum_duration=1,
        )
        w.run()

        # Wait for landing waypoint.
        self.wait_current_waypoint(11, timeout=1200)
        self.wait_disarmed(120)

    def TakeoffAuto4(self):
        '''Test the behaviour of an AUTO takeoff, pt4.'''
        '''
        Conditions:
        - ARSPD_USE=0
        - TKOFF_OPTIONS[0]=1
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "THR_MAX": 80.0,
            "THR_MIN": 0.0,
            "TKOFF_OPTIONS": 1.0,
            "TKOFF_THR_MAX": 100.0,
            "TKOFF_THR_MINACC": 3.0,
            "TECS_PITCH_MAX": 35.0,
            "TKOFF_THR_MAX_T": 3.0,
            "PTCH_LIM_MAX_DEG": 35.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
        })

        self.wait_ready_to_arm()

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.load_mission("catapult.txt", strict=True)
        self.change_mode('AUTO')

        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Ensure that TKOFF_THR_MAX_T is respected.
        self.delay_sim_time(self.get_parameter("TKOFF_THR_MAX_T")-1)
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Ensure that after that the aircraft still goes to maximum throttle.
        test_alt = 50
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for landing waypoint.
        self.wait_current_waypoint(11, timeout=1200)
        self.wait_disarmed(120)

    def TakeoffTakeoff1(self):
        '''Test the behaviour of a takeoff in TAKEOFF mode, pt1.'''
        '''
        Conditions:
        - ARSPD_USE=1
        - TKOFF_OPTIONS[0]=0
        - TKOFF_THR_MAX < THR_MAX
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 1.0,
            "THR_MAX": 100.0,
            "TKOFF_LVL_ALT": 30.0,
            "TKOFF_ALT": 80.0,
            "TKOFF_OPTIONS": 0.0,
            "TKOFF_THR_MINACC": 3.0,
            "TKOFF_THR_MAX": 80.0,
            "TECS_PITCH_MAX": 35.0,
            "PTCH_LIM_MAX_DEG": 35.0,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Check whether we're at max throttle below TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")-10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Check whether we're still at max throttle past TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")+10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for the takeoff to complete.
        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-5, target_alt, relative=True)

        # Wait a bit for the Takeoff altitude to settle.
        self.delay_sim_time(5)

        self.fly_home_land_and_disarm()

    def TakeoffTakeoff2(self):
        '''Test the behaviour of a takeoff in TAKEOFF mode, pt2.'''
        '''
        Conditions:
        - ARSPD_USE=1
        - TKOFF_OPTIONS[0]=1
        - TKOFF_THR_MAX < THR_MAX
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 1.0,
            "THR_MAX": 100.0,
            "TKOFF_LVL_ALT": 30.0,
            "TKOFF_ALT": 80.0,
            "TKOFF_OPTIONS": 1.0,
            "TKOFF_THR_MINACC": 3.0,
            "TKOFF_THR_MAX": 80.0,
            "TECS_PITCH_MAX": 35.0,
            "PTCH_LIM_MAX_DEG": 35.0,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Check whether we're at max throttle below TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")-10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Check whether we've receded from max throttle past TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")+10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        thr_min = 1000+10*(self.get_parameter("TKOFF_THR_MIN"))-1
        thr_max = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))-10
        self.assert_servo_channel_range(3, thr_min, thr_max)

        # Wait for the takeoff to complete.
        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-5, target_alt, relative=True)

        # Wait a bit for the Takeoff altitude to settle.
        self.delay_sim_time(5)

        self.fly_home_land_and_disarm()

    def TakeoffTakeoff3(self):
        '''Test the behaviour of a takeoff in TAKEOFF mode, pt3.'''
        '''
        This is the same as case #1, but with disabled airspeed sensor.

        Conditions:
        - ARSPD_USE=0
        - TKOFF_OPTIONS[0]=0
        - TKOFF_THR_MAX < THR_MAX
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "THR_MAX": 100.0,
            "TKOFF_DIST": 400.0,
            "TKOFF_LVL_ALT": 30.0,
            "TKOFF_ALT": 100.0,
            "TKOFF_OPTIONS": 0.0,
            "TKOFF_THR_MINACC": 3.0,
            "TKOFF_THR_MAX": 80.0,
            "TECS_PITCH_MAX": 35.0,
            "PTCH_LIM_MAX_DEG": 35.0,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # we expect to maintain this throttle level past the takeoff
        # altitude through to our takeoff altitude:
        expected_takeoff_throttle = 1000+10*self.get_parameter("TKOFF_THR_MAX")

        # Check whether we're at max throttle below TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")-10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        w = vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            3,  # throttle
            expected_takeoff_throttle,
            epsilon=10,
            minimum_duration=1,
        )
        w.run()

        # Check whether we're still at max throttle past TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")+10
        self.wait_altitude(test_alt, test_alt+2, relative=True)

        w = vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            3,  # throttle
            expected_takeoff_throttle,
            epsilon=10,
            minimum_duration=1,
        )
        w.run()

        # Wait for the takeoff to complete.
        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-2.5, target_alt+2.5, relative=True, minimum_duration=10, timeout=30)

        self.reboot_sitl(force=True)

    def TakeoffTakeoff4(self):
        '''Test the behaviour of a takeoff in TAKEOFF mode, pt4.'''
        '''
        This is the same as case #3, but with almost stock parameters and without a catapult.

        Conditions:
        - ARSPD_USE=0
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Check whether we're at max throttle below TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")-10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Check whether we're still at max throttle past TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")+10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for the takeoff to complete.
        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-5, target_alt, relative=True)

        # Wait a bit for the Takeoff altitude to settle.
        self.delay_sim_time(5)

        self.fly_home_land_and_disarm()

    def TakeoffTakeoff5(self):
        '''Test the behaviour of a takeoff with no compass'''
        self.set_parameters({
            "COMPASS_USE": 0,
            "COMPASS_USE2": 0,
            "COMPASS_USE3": 0,
        })
        import copy
        start_loc = copy.copy(SITL_START_LOCATION)
        start_loc.heading = 175
        self.customise_SITL_commandline(["--home=%.9f,%.9f,%.2f,%.1f" % (
            start_loc.lat, start_loc.lng, start_loc.alt, start_loc.heading)])
        self.reboot_sitl()
        self.change_mode("TAKEOFF")

        # waiting for the EKF to be happy won't work
        self.delay_sim_time(20)
        self.arm_vehicle()

        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-5, target_alt, relative=True)

        # Wait a bit for the Takeoff altitude to settle.
        self.delay_sim_time(5)

        bearing_margin = 35
        loc = self.mav.location()
        bearing_from_home = self.get_bearing(start_loc, loc)
        if bearing_from_home < 0:
            bearing_from_home += 360
        if abs(bearing_from_home - start_loc.heading) > bearing_margin:
            raise NotAchievedException(f"Did not takeoff in the right direction {bearing_from_home}")

        self.fly_home_land_and_disarm()

    def TakeoffGround(self):
        '''Test a rolling TAKEOFF.'''

        self.set_parameters({
            "TKOFF_ROTATE_SPD": 15.0,
            "TKOFF_PLIM_SEC": 0,  # Ensure TKOFF_PLIM_SEC doesn't interfere with the test.
            "TKOFF_DIST": 500,  # Ensure stall prevention doesn't interfere with the test.
            "TKOFF_ALT": 100  # Ditto
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Check that we demand minimum pitch below rotation speed.
        self.wait_groundspeed(8, 10)
        m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT', timeout=5)
        nav_pitch = m.nav_pitch
        if nav_pitch > 5.1 or nav_pitch < 4.9:
            raise NotAchievedException(f"Did not achieve correct takeoff pitch ({nav_pitch}).")

        # Check whether we've achieved correct target pitch after rotation.
        self.wait_groundspeed(24, 25)
        m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT', timeout=5)
        nav_pitch = m.nav_pitch
        if nav_pitch > 15.1 or nav_pitch < 14.9:
            raise NotAchievedException(f"Did not achieve correct takeoff pitch ({nav_pitch}).")

        self.fly_home_land_and_disarm()

    def TakeoffIdleThrottle(self):
        '''Apply idle throttle before takeoff.'''
        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "TKOFF_THR_IDLE": 20.0,
            "TKOFF_THR_MINSPD": 3.0,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Ensure that the throttle rises to idle throttle.
        expected_idle_throttle = 1000+10*self.get_parameter("TKOFF_THR_IDLE")
        self.assert_servo_channel_range(3, expected_idle_throttle-10, expected_idle_throttle+10)

        # Launch the catapult
        self.set_servo(6, 1000)

        self.delay_sim_time(5)
        self.change_mode('RTL')

        self.fly_home_land_and_disarm()

    def TakeoffBadLevelOff(self):
        '''Ensure that the takeoff can be completed under 0 pitch demand.'''
        '''
        When using no airspeed, the pitch level-off will eventually command 0
        pitch demand. Ensure that the plane can climb the final 2m to deem the
        takeoff complete.
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
            defaults_filepath=self.model_defaults_filepath("plane")
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "PTCH_TRIM_DEG": -10.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
            "TKOFF_ALT": 50.0,
            "TKOFF_DIST": 1000.0,
            "TKOFF_THR_MAX": 75.0,
            "TKOFF_THR_MINACC": 3.0,
        })

        self.load_mission("flaps_tkoff_50.txt")
        self.change_mode('AUTO')

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Wait until we've reached the takeoff altitude.
        target_alt = 50
        self.wait_altitude(target_alt-1, target_alt+1, relative=True, timeout=30)

        self.delay_sim_time(5)

        self.disarm_vehicle(force=True)

    def TakeoffLevelOffWind(self):
        '''Ensure the level-off functionality works.'''
        '''
        This is primarily targeted to test whether the level-off angle works
        correctly even though the groundspeed eventually drops in face of wind.
        '''
        tkoff_alt = 100.
        self.set_parameters({
            "TKOFF_ROTATE_SPD": 15.0,
            "TKOFF_ALT": tkoff_alt,
            "TKOFF_DIST": 500,  # Ensure stall prevention doesn't interfere with the test.
            "TKOFF_PLIM_SEC": 5  # Give some more time to detect the level-off.
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Wait until we're well past the rotation.
        self.wait_groundspeed(24, 25)

        self.set_parameters({
            "SIM_WIND_DIR": 0.0,  # Set North wind.
            "SIM_WIND_SPD": 10.0  # Enough to bring groundspeed below cruise speed.
        })

        self.wait_altitude(tkoff_alt-10, tkoff_alt, relative=True)
        self.wait_level_flight(accuracy=10, timeout=1)  # Ensure we have roughly level-off.
        self.delay_sim_time(5)

        self.change_mode('AUTOLAND')
        self.wait_disarmed(timeout=180)

    def DCMFallback(self):
        '''Really annoy the EKF and force fallback'''
        self.reboot_sitl()
        self.delay_sim_time(30)

        self.takeoff(50)
        self.change_mode('CIRCLE')
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "EK3_POS_I_GATE": 0,
            "SIM_GPS1_HZ": 1,
            "SIM_GPS1_LAG_MS": 1000,
        })
        self.wait_statustext("DCM Active", check_context=True, timeout=60)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.wait_statustext("DCM Active", check_context=True)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.wait_statustext("DCM Active", check_context=True)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.context_stop_collecting('STATUSTEXT')

        self.fly_home_land_and_disarm()
        self.context_pop()
        self.reboot_sitl()

    def ForcedDCM(self):
        '''Switch to DCM mid-flight'''
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.takeoff(50)
        self.context_collect('STATUSTEXT')
        self.set_parameter("AHRS_EKF_TYPE", 0)
        self.wait_statustext("DCM Active", check_context=True)
        self.context_stop_collecting('STATUSTEXT')

        self.change_mode('AUTOLAND')
        self.wait_disarmed(timeout=180)

    def EFITest(self, efi_type, name, sim_name, check_fuel_flow=True):
        '''method to be called by EFI tests'''
        self.start_subtest("EFI Test for (%s)" % name)
        self.assert_not_receiving_message('EFI_STATUS')
        self.set_parameters({
            'SIM_EFI_TYPE': efi_type,
            'EFI_TYPE': efi_type,
            'SERIAL5_PROTOCOL': 24,
            'RPM1_TYPE': 10,
        })

        self.customise_SITL_commandline(
            ["--serial5=sim:%s" % sim_name,
             ],
        )
        self.wait_ready_to_arm()

        baro_m = self.assert_receive_message("SCALED_PRESSURE")
        self.progress(self.dump_message_verbose(baro_m))
        baro_temperature = baro_m.temperature / 100.0  # cDeg->deg
        m = self.assert_received_message_field_values("EFI_STATUS", {
            "throttle_out": 0,
            "health": 1,
        }, very_verbose=1)

        if abs(baro_temperature - m.intake_manifold_temperature) > 1:
            raise NotAchievedException(
                "Bad intake manifold temperature (want=%f got=%f)" %
                (baro_temperature, m.intake_manifold_temperature))

        self.arm_vehicle()

        self.set_rc(3, 1300)

        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                raise NotAchievedException("RPM1 and EFI_STATUS.rpm did not match")
            rpm_m = self.assert_receive_message("RPM", verbose=1)
            want_rpm = 1000
            if rpm_m.rpm1 < want_rpm:
                continue

            m = self.assert_receive_message("EFI_STATUS", verbose=1)
            if abs(m.rpm - rpm_m.rpm1) > 100:
                continue

            break

        self.progress("now we're started, check a few more values")
        # note that megasquirt drver doesn't send throttle, so throttle_out is zero!
        m = self.assert_received_message_field_values("EFI_STATUS", {
            "health": 1,
        }, very_verbose=1)
        m = self.wait_message_field_values("EFI_STATUS", {
            "throttle_position": 31,
            "intake_manifold_temperature": 28,
        }, very_verbose=1, epsilon=2)

        if check_fuel_flow:
            if abs(m.fuel_flow - 0.2) < 0.0001:
                raise NotAchievedException("Expected fuel flow")

        self.set_rc(3, 1000)

        # need to force disarm as the is_flying flag can trigger with the engine running
        self.disarm_vehicle(force=True)

    def MegaSquirt(self):
        '''test MegaSquirt driver'''
        self.EFITest(
            1, "MegaSquirt", "megasquirt",
            check_fuel_flow=False,
        )

    def Hirth(self):
        '''Test Hirth EFI'''
        self.EFITest(8, "Hirth", "hirth")

    def AltitudeSlopeMaxHeight(self):
        '''Test rebuild altitude slope if above and climbing'''

        # Test that ALT_SLOPE_MAXHGT correctly controls re-planning altitude slope
        # in the scenario that aircraft is above planned slope and slope is positive (climbing).
        #
        #
        #  Behaviour with ALT_SLOPE_MAXHGT = 0 (no slope replanning)
        #       (2)..      __(4)
        #         |  \..__/
        #         |  __/
        #         (3)
        #
        # Behaviour with ALT_SLOPE_MAXHGT = 5 (slope replanning when >5m error)
        #       (2)........__(4)
        #         |     __/
        #         |  __/
        #         (3)
        # Solid is plan, dots are actual flightpath.

        self.load_mission('rapid-descent-then-climb.txt', strict=False)

        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        #
        # Initial run with ALT_SLOPE_MAXHGT = 5 (default).
        #
        self.set_parameter("ALT_SLOPE_MAXHGT", 5)

        # Wait for waypoint commanding rapid descent, followed by climb.
        self.wait_current_waypoint(5, timeout=1200)

        # Altitude should not descend significantly below the initial altitude
        init_altitude = self.get_altitude(relative=True, timeout=2)
        timeout = 600
        wpnum = 7
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get wanted current waypoint")

            if (self.get_altitude(relative=True, timeout=2) - init_altitude) < -10:
                raise NotAchievedException("Descended >10m before reaching desired waypoint,\
  indicating slope was not replanned")

            seq = self.mav.waypoint_current()
            self.progress("Waiting for wp=%u current=%u" % (wpnum, seq))
            if seq == wpnum:
                break

        self.set_current_waypoint(2)

        #
        # Second run with ALT_SLOPE_MAXHGT = 0 (no re-plan).
        #
        self.set_parameter("ALT_SLOPE_MAXHGT", 0)

        # Wait for waypoint commanding rapid descent, followed by climb.
        self.wait_current_waypoint(5, timeout=1200)

        # This time altitude should descend significantly below the initial altitude
        init_altitude = self.get_altitude(relative=True, timeout=2)
        timeout = 600
        wpnum = 7
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get wanted altitude")

            seq = self.mav.waypoint_current()
            self.progress("Waiting for wp=%u current=%u" % (wpnum, seq))
            if seq == wpnum:
                raise NotAchievedException("Reached desired waypoint without first descending 10m,\
 indicating slope was replanned unexpectedly")

            if (self.get_altitude(relative=True, timeout=2) - init_altitude) < -10:
                break

        # Disarm
        self.wait_disarmed(timeout=600)

        self.progress("Mission OK")

    def MAV_CMD_NAV_LOITER_TURNS(self, target_system=1, target_component=1):
        '''test MAV_CMD_NAV_LOITER_TURNS mission item'''
        alt = 100
        seq = 0
        items = []
        tests = [
            (self.home_relative_loc_ne(50, -50), 100, 0.3),
            (self.home_relative_loc_ne(100, 50), 1005, 3),
        ]
        # add a home position:
        items.append(self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            seq, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
        seq += 1

        # add takeoff
        items.append(self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            seq, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            alt, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
        seq += 1

        # add circles
        for (loc, radius, turn) in tests:
            items.append(self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                seq, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                0, # current
                0, # autocontinue
                turn, # p1
                0, # p2
                radius, # p3
                0, # p4
                int(loc.lat*1e7), # latitude
                int(loc.lng*1e7), # longitude
                alt, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
            seq += 1

        # add an RTL
        items.append(self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            seq, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
        seq += 1

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, items)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        ofs = 2
        self.progress("Checking downloaded mission is as expected")
        for (loc, radius, turn) in tests:
            downloaded = downloaded_items[ofs]
            if radius > 255:
                # ArduPilot only stores % 10
                radius = radius - radius % 10
            if downloaded.param3 != radius:
                raise NotAchievedException(
                    "Did not get expected radius for item %u; want=%f got=%f" %
                    (ofs, radius, downloaded.param3))
            if turn > 0 and turn < 1:
                # ArduPilot stores fractions in 8 bits (*256) and unpacks it (/256)
                turn = int(turn*256) / 256.0
            if downloaded.param1 != turn:
                raise NotAchievedException(
                    "Did not get expected turn for item %u; want=%f got=%f" %
                    (ofs, turn, downloaded.param1))
            ofs += 1

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("NAVL1_LIM_BANK", 50)

        self.wait_current_waypoint(2)

        for (loc, expected_radius, _) in tests:
            self.wait_circling_point_with_radius(
                loc,
                expected_radius,
                epsilon=20.0,
                timeout=240,
            )
            self.set_current_waypoint(self.current_waypoint()+1)

        self.fly_home_land_and_disarm(timeout=180)

    def MidAirDisarmDisallowed(self):
        '''Ensure mid-air disarm is not possible'''
        self.takeoff(50)
        disarmed = False
        try:
            self.disarm_vehicle()
            disarmed = True
        except ValueError as e:
            self.progress("Got %s" % repr(e))
            if "Expected MAV_RESULT_ACCEPTED got MAV_RESULT_FAILED" not in str(e):
                raise e
        if disarmed:
            raise NotAchievedException("Disarmed when we shouldn't have")
        # should still be able to force-disarm:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def AerobaticsScripting(self):
        '''Fixed Wing Aerobatics'''
        applet_script = "Aerobatics/FixedWing/plane_aerobatics.lua"
        airshow = "Aerobatics/FixedWing/Schedules/AirShow.txt"
        trick72 = "trick72.txt"

        model = "plane-3d"

        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath="Tools/autotest/models/plane-3d.parm",
            wipe=True)

        self.context_push()
        self.install_applet_script_context(applet_script)
        self.install_applet_script_context(airshow, install_name=trick72)
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        self.set_parameter("TRIK_ENABLE", 1)
        self.set_rc(7, 1000) # disable tricks

        self.scripting_restart()
        self.wait_text("Enabled 3 aerobatic tricks", check_context=True)
        self.set_parameters({
            "TRIK1_ID": 72,
            "RC7_OPTION" : 300, # activation switch
            "RC9_OPTION" : 301, # selection switch
            "SIM_SPEEDUP": 5, # need to give some cycles to lua
        })

        self.wait_ready_to_arm()
        self.change_mode("TAKEOFF")
        self.arm_vehicle()
        self.wait_altitude(30, 40, timeout=30, relative=True)
        self.change_mode("CRUISE")

        self.set_rc(9, 1000) # select first trick
        self.delay_sim_time(1)
        self.set_rc(7, 1500) # show selected trick

        self.wait_text("Trick 1 selected (SuperAirShow)", check_context=True)
        self.set_rc(7, 2000) # activate trick
        self.wait_text("Trick 1 started (SuperAirShow)", check_context=True)

        highest_error = 0
        while True:
            m = self.mav.recv_match(type='NAMED_VALUE_FLOAT', timeout=2, blocking=True)
            if not m:
                break
            if m.name != 'PERR':
                continue
            highest_error = max(highest_error, m.value)
            if highest_error > 15:
                raise NotAchievedException("path error %.1f" % highest_error)

        if highest_error == 0:
            raise NotAchievedException("path error not reported")
        self.progress("Finished trick, max error=%.1fm" % highest_error)
        self.disarm_vehicle(force=True)

        messages = self.context_collection('STATUSTEXT')
        self.context_pop()
        self.reboot_sitl()

        # check all messages to see if we got all tricks
        tricks = ["Loop", "HalfReverseCubanEight", "ScaleFigureEight", "Immelmann",
                  "Split-S", "RollingCircle", "HumptyBump", "HalfCubanEight",
                  "BarrelRoll", "CrossBoxTopHat", "TriangularLoop",
                  "Finishing SuperAirShow!"]
        texts = [m.text for m in messages]
        for t in tricks:
            if t in texts:
                self.progress("Completed trick %s" % t)
            else:
                raise NotAchievedException("Missing trick %s" % t)

    def UniversalAutoLandScript(self):
        '''Test UniversalAutoLandScript'''
        applet_script = "UniversalAutoLand.lua"
        self.customise_SITL_commandline(["--home", "-35.362938,149.165085,585,173"])

        self.install_applet_script_context(applet_script)
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SCR_ENABLE" : 1,
            "SCR_VM_I_COUNT" : 1000000,
            "RTL_AUTOLAND" : 2
            })
        self.reboot_sitl()
        self.wait_text("Loaded UniversalAutoLand.lua", check_context=True)
        self.set_parameters({
             "ALAND_ENABLE" : 1,
             "ALAND_WP_ALT" : 55,
             "ALAND_WP_DIST" : 400
            })
        self.wait_ready_to_arm()
        self.scripting_restart()
        self.wait_text("Scripting: restarted", check_context=True)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("AUTO")
        self.wait_text("Captured initial takeoff direction", check_context=True)

        self.wait_disarmed(120)
        self.progress("Check the landed heading matches takeoff")
        self.wait_heading(173, accuracy=5, timeout=1)
        loc = mavutil.location(-35.362938, 149.165085, 585, 173)
        if self.get_distance(loc, self.mav.location()) > 35:
            raise NotAchievedException("Did not land close to home")

    def SDCardWPTest(self):
        '''test BRD_SD_MISSION support'''
        spiral_script = "mission_spiral.lua"

        self.context_push()
        self.install_example_script(spiral_script)
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "BRD_SD_MISSION" : 64,
            "SCR_ENABLE" : 1,
            "SCR_VM_I_COUNT" : 1000000
            })

        self.wait_ready_to_arm()
        self.reboot_sitl()

        self.wait_text("Loaded spiral mission creator", check_context=True)
        self.set_parameters({
            "SCR_USER2": 19, # radius
            "SCR_USER3": -35.36322, # lat
            "SCR_USER4": 149.16525, # lon
            "SCR_USER5": 684.13, # alt
        })

        count = (65536 // 15) - 1

        self.progress("Creating spiral mission of size %s" % count)
        self.set_parameter("SCR_USER1", count)

        self.wait_text("Created spiral of size %u" % count, check_context=True)

        self.progress("Checking spiral before reboot")
        self.set_parameter("SCR_USER6", count)
        self.wait_text("Compared spiral of size %u OK" % count, check_context=True)
        self.set_parameter("SCR_USER6", 0)

        self.wait_ready_to_arm()
        self.reboot_sitl()
        self.progress("Checking spiral after reboot")
        self.set_parameter("SCR_USER6", count)
        self.wait_text("Compared spiral of size %u OK" % count, check_context=True)

        self.remove_installed_script(spiral_script)

        self.context_pop()
        self.wait_ready_to_arm()
        self.reboot_sitl()

    def MANUAL_CONTROL(self):
        '''test MANUAL_CONTROL mavlink message'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)

        self.progress("Takeoff")
        self.takeoff(alt=50)

        self.change_mode('FBWA')

        tstart = self.get_sim_time_cached()
        roll_input = -500
        want_roll_degrees = -12
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach roll")
            self.progress("Sending roll-left")
            self.mav.mav.manual_control_send(
                1, # target system
                32767, # x (pitch)
                roll_input, # y (roll)
                32767, # z (thrust)
                32767, # r (yaw)
                0) # button mask
            m = self.assert_receive_message('ATTITUDE', verbose=True)
            p = math.degrees(m.roll)
            self.progress("roll=%f want<=%f" % (p, want_roll_degrees))
            if p <= want_roll_degrees:
                break
        self.mav.mav.manual_control_send(
            1, # target system
            32767, # x (pitch)
            32767, # y (roll)
            32767, # z (thrust)
            32767, # r (yaw)
            0) # button mask
        self.fly_home_land_and_disarm()

    def mission_home_point(self, target_system=1, target_component=1):
        '''just an empty-ish item-int to store home'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_jump_tag(self, tag, target_system=1, target_component=1):
        '''create a jump tag mission item'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_JUMP_TAG,
            0, # current
            0, # autocontinue
            tag, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_do_jump_tag(self, tag, target_system=1, target_component=1):
        '''create a jump tag mission item'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            0, # current
            0, # autocontinue
            tag, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_anonymous_waypoint(self, target_system=1, target_component=1):
        '''just a boring waypoint'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            1, # latitude
            1, # longitude
            1, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def renumber_mission_items(self, mission):
        count = 0
        for item in mission:
            item.seq = count
            count += 1

    def MissionJumpTags_missing_jump_target(self, target_system=1, target_component=1):
        self.start_subtest("Check missing-jump-tag behaviour")
        jump_target = 2
        mission = [
            self.mission_home_point(),
            self.mission_anonymous_waypoint(),
            self.mission_anonymous_waypoint(),
            self.mission_jump_tag(jump_target),
            self.mission_anonymous_waypoint(),
            self.mission_anonymous_waypoint(),
        ]
        self.renumber_mission_items(mission)
        self.check_mission_upload_download(mission)
        self.progress("Checking incorrect tag behaviour")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            p1=jump_target + 1,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED
        )
        self.progress("Checking correct tag behaviour")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            p1=jump_target,
        )
        self.assert_current_waypoint(4)

    def MissionJumpTags_do_jump_to_bad_tag(self, target_system=1, target_component=1):
        mission = [
            self.mission_home_point(),
            self.mission_anonymous_waypoint(),
            self.mission_do_jump_tag(17),
            self.mission_anonymous_waypoint(),
        ]
        self.renumber_mission_items(mission)
        self.check_mission_upload_download(mission)
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.set_current_waypoint(2, check_afterwards=False)
        self.assert_mode('RTL')
        self.disarm_vehicle()

    def MissionJumpTags_jump_tag_at_end_of_mission(self, target_system=1, target_component=1):
        mission = [
            self.mission_home_point(),
            self.mission_anonymous_waypoint(),
            self.mission_jump_tag(17),
        ]
        # Jumping to an end of a mission, either DO_JUMP or DO_JUMP_TAG will result in a failed attempt.
        # The failure is from mission::set_current_cmd() returning false if it can not find any NAV
        # commands on or after the index. Two scenarios:
        # 1) AUTO mission triggered: The the set_command will fail and it will cause an RTL event
        #       (Harder to test, need vehicle to actually reach the waypoint)
        # 2) GCS/MAVLink: It will return MAV_RESULT_FAILED and there's on change to the mission. (Easy to test)
        self.renumber_mission_items(mission)
        self.check_mission_upload_download(mission)
        self.progress("Checking correct tag behaviour")
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            p1=17,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED
        )
        self.disarm_vehicle()

    def MissionJumpTags(self):
        '''test MAV_CMD_JUMP_TAG'''
        self.wait_ready_to_arm()
        self.MissionJumpTags_missing_jump_target()
        self.MissionJumpTags_do_jump_to_bad_tag()
        self.MissionJumpTags_jump_tag_at_end_of_mission()

    def AltResetBadGPS(self):
        '''Tests the handling of poor GPS lock pre-arm alt resets'''
        self.set_parameters({
            "SIM_GPS1_GLTCH_Z": 0,
            "SIM_GPS1_ACC": 0.3,
        })
        self.wait_ready_to_arm()

        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        relalt = m.relative_alt*0.001
        if abs(relalt) > 3:
            raise NotAchievedException("Bad relative alt %.1f" % relalt)

        self.progress("Setting low accuracy, glitching GPS")
        self.set_parameter("SIM_GPS1_ACC", 40)
        self.set_parameter("SIM_GPS1_GLTCH_Z", -47)

        self.progress("Waiting 10s for height update")
        self.delay_sim_time(10)

        self.wait_ready_to_arm()
        self.arm_vehicle()

        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        relalt = m.relative_alt*0.001
        if abs(relalt) > 3:
            raise NotAchievedException("Bad glitching relative alt %.1f" % relalt)

        self.disarm_vehicle()
        # reboot to clear potentially bad state

    def trigger_airspeed_cal(self):
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p3=1,
        )

    def AirspeedCal(self):
        '''test Airspeed calibration'''

        self.start_subtest('1 airspeed sensor')
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.trigger_airspeed_cal()
        self.wait_statustext('Airspeed 1 calibrated', check_context=True)
        self.context_pop()

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.start_subtest('0 airspeed sensors')
        self.set_parameter('ARSPD_TYPE', 0)
        self.reboot_sitl()
        self.wait_statustext('No airspeed sensor', check_context=True)
        self.trigger_airspeed_cal()
        self.delay_sim_time(5)
        if self.statustext_in_collections('Airspeed 1 calibrated'):
            raise NotAchievedException("Did not disable airspeed sensor?!")
        self.context_pop()

        self.start_subtest('2 airspeed sensors')
        self.set_parameter('ARSPD_TYPE', 100)
        self.set_parameter('ARSPD2_TYPE', 100)
        self.reboot_sitl()
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.trigger_airspeed_cal()
        self.wait_statustext('Airspeed 1 calibrated', check_context=True)
        self.wait_statustext('Airspeed 2 calibrated', check_context=True)
        self.context_pop()

        self.reboot_sitl()

    def RunMissionScript(self):
        '''Test run_mission.py script'''
        script = os.path.join('Tools', 'autotest', 'run_mission.py')
        self.stop_SITL()
        util.run_cmd([
            util.reltopdir(script),
            self.binary,
            'plane',
            self.generic_mission_filepath_for_filename("flaps.txt"),
        ], checkfail=True)
        self.start_SITL()

    def MAV_CMD_GUIDED_CHANGE_ALTITUDE(self):
        '''test handling of MAV_CMD_GUIDED_CHANGE_ALTITUDE'''
        target_alt = 750
        # this location is chosen to be fairly flat, but at a different terrain height to home
        higher_ground = mavutil.location(-35.35465024, 149.13974996, target_alt, 0)
        self.install_terrain_handlers_context()
        self.start_subtest("set home relative altitude")
        self.takeoff(30, relative=True)
        self.change_mode('GUIDED')

        # remember home
        home_loc = self.home_position_as_mav_location()
        height_diff = target_alt - home_loc.alt

        # fly to higher ground
        self.send_do_reposition(higher_ground, frame=mavutil.mavlink.MAV_FRAME_GLOBAL)
        self.wait_location(
            higher_ground,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )

        for alt in 50, 70:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                p7=alt+height_diff,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.wait_altitude(target_alt+alt-1, target_alt+alt+1, timeout=30, minimum_duration=10, relative=False)

        # test for #24535
        self.start_subtest("switch to loiter and resume guided maintains home relative altitude")
        self.change_mode('LOITER')
        self.delay_sim_time(1)
        self.change_mode('GUIDED')
        self.wait_altitude(
            height_diff+alt-3,  # NOTE: reuse of alt from above loop!
            height_diff+alt+3,
            minimum_duration=10,
            timeout=30,
            relative=True,
        )
        # test that this works if switching between RELATIVE (HOME) and GLOBAL(AMSL)
        self.start_subtest("set global/AMSL altitude, switch to loiter and resume guided")
        alt = target_alt+30
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
        )
        self.wait_altitude(alt-3, alt+3, timeout=30, relative=False, minimum_duration=10)

        # let it settle so LOITER captures a constant altitude
        self.delay_sim_time(10)

        # now switch to LOITER then back to GUIDED
        self.change_mode('LOITER')
        self.delay_sim_time(5)
        self.change_mode('GUIDED')
        self.wait_altitude(
            alt-5,  # NOTE: reuse of alt from above CHANGE_ALTITUDE
            alt+5,
            minimum_duration=10,
            timeout=30,
            relative=False,
        )

        # test that this works if switching between RELATIVE (HOME) and terrain
        self.start_subtest("set terrain altitude, switch to loiter and resume guided")
        self.change_mode('GUIDED')
        alt = 100
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )
        self.wait_altitude(
            alt-10,  # NOTE: reuse of alt from abovE
            alt+10,  # use a 10m buffer as the plane needs to go up and down a bit to maintain terrain distance
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )

        alt = 150
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )
        self.wait_altitude(
            alt-10,  # NOTE: reuse of alt from abovE
            alt+10,  # use a 10m buffer as the plane needs to go up and down a bit to maintain terrain distance
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )
        self.delay_sim_time(30)

        self.change_mode('LOITER')
        self.delay_sim_time(1)
        self.change_mode('GUIDED')
        self.wait_altitude(
            alt-5,  # NOTE: reuse of alt from abovE
            alt+5,  # use a 5m buffer as the plane needs to go up and down a bit to maintain terrain distance
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )

        self.start_subtest("test flyto with relative alt")
        dest = copy.copy(higher_ground)
        dest.alt = higher_ground.alt + 100 - home_loc.alt
        self.progress("dest.alt=%.1f" % dest.alt)
        self.send_do_reposition(dest, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.wait_location(
            dest,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )
        self.wait_altitude(
            dest.alt-5,
            dest.alt+5,
            minimum_duration=10,
            timeout=30,
            relative=True
        )

        self.start_subtest("test flyto with absolute alt")
        dest = copy.copy(higher_ground)
        dest.alt = higher_ground.alt + 60
        self.progress("dest.alt=%.1f" % dest.alt)
        self.send_do_reposition(dest, frame=mavutil.mavlink.MAV_FRAME_GLOBAL)
        self.wait_location(
            dest,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )
        self.wait_altitude(
            dest.alt-5,
            dest.alt+5,
            minimum_duration=10,
            timeout=30,
            relative=False
        )

        self.start_subtest("test flyto with terrain alt")
        dest = copy.copy(higher_ground)
        dest.alt = 130
        self.progress("dest.alt=%.1f" % dest.alt)
        self.send_do_reposition(dest, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
        self.wait_location(
            dest,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )
        self.wait_altitude(
            dest.alt-10,
            dest.alt+10,
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )

        self.delay_sim_time(5)
        self.fly_home_land_and_disarm()

    def _MAV_CMD_PREFLIGHT_CALIBRATION(self, command):
        self.context_push()
        self.start_subtest("Denied when armed")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p1=1,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.disarm_vehicle()

        self.context_collect('STATUSTEXT')

        self.start_subtest("gyro cal")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p1=1,
        )

        self.start_subtest("baro cal")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p3=1,
        )
        self.wait_statustext('Barometer calibration complete', check_context=True)

        # accelcal skipped here, it is checked elsewhere

        self.start_subtest("ins trim")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=2,
        )

        # enforced delay between cals:
        self.delay_sim_time(5)

        self.start_subtest("simple accel cal")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
        )
        # simple gyro cal makes the GPS units go unhealthy as they are
        # not maintaining their update rate (gyro cal is synchronous
        # in the main loop).  Usually ~30 seconds to recover...
        self.wait_gps_sys_status_not_present_or_enabled_and_healthy(timeout=60)

        self.start_subtest("force save accels")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=76,
        )

        self.start_subtest("force save compasses")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p2=76,
        )

        self.context_pop()

    def MAV_CMD_PREFLIGHT_CALIBRATION(self):
        '''test MAV_CMD_PREFLIGHT_CALIBRATION mavlink handling'''
        self._MAV_CMD_PREFLIGHT_CALIBRATION(self.run_cmd)
        self._MAV_CMD_PREFLIGHT_CALIBRATION(self.run_cmd_int)

    def MAV_CMD_DO_INVERTED_FLIGHT(self):
        '''fly upside-down mission item'''
        alt = 30
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 400, 0, alt),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_INVERTED_FLIGHT,
                p1=1,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, alt),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_INVERTED_FLIGHT,
                p1=0,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1200, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.wait_current_waypoint(2)  # upright flight
        self.wait_message_field_values("NAV_CONTROLLER_OUTPUT", {
            "nav_roll": 0,
            "nav_pitch": 0,
        }, epsilon=10)

        def check_altitude(mav, m):
            m_type = m.get_type()
            if m_type != 'GLOBAL_POSITION_INT':
                return
            if abs(30 - m.relative_alt * 0.001) > 15:
                raise NotAchievedException("Bad altitude while flying inverted")

        self.context_push()
        self.install_message_hook_context(check_altitude)

        self.wait_current_waypoint(4)  # inverted flight
        self.wait_message_field_values("NAV_CONTROLLER_OUTPUT", {
            "nav_roll": 180,
            "nav_pitch": 9,
        }, epsilon=10,)

        self.wait_current_waypoint(6)  # upright flight
        self.wait_message_field_values("NAV_CONTROLLER_OUTPUT", {
            "nav_roll": 0,
            "nav_pitch": 0,
        }, epsilon=10)

        self.context_pop()  # remove the check_altitude call

        self.wait_current_waypoint(7)

        self.fly_home_land_and_disarm()

    def MAV_CMD_DO_AUTOTUNE_ENABLE(self):
        '''test enabling autotune via mavlink'''
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE, p1=1)
        self.wait_statustext('Started autotune', check_context=True)
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE, p1=0)
        self.wait_statustext('Stopped autotune', check_context=True)

    def DO_PARACHUTE(self):
        '''test triggering parachute via mavlink'''
        self.set_parameters({
            "CHUTE_ENABLED": 1,
            "CHUTE_TYPE": 10,
            "SERVO9_FUNCTION": 27,
            "SIM_PARA_ENABLE": 1,
            "SIM_PARA_PIN": 9,
            "FS_LONG_ACTN": 3,
        })
        for command in self.run_cmd, self.run_cmd_int:
            self.wait_servo_channel_value(9, 1100)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            command(
                mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
                p1=mavutil.mavlink.PARACHUTE_RELEASE,
            )
            self.wait_servo_channel_value(9, 1300)
            self.disarm_vehicle()
            self.reboot_sitl()

    def _MAV_CMD_DO_GO_AROUND(self, command):
        self.load_mission("mission.txt")
        self.set_parameter("RTL_AUTOLAND", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(6)
        command(mavutil.mavlink.MAV_CMD_DO_GO_AROUND, p1=150)
        self.wait_current_waypoint(5)
        self.wait_altitude(135, 165, relative=True)
        self.wait_disarmed(timeout=300)

    def MAV_CMD_DO_GO_AROUND(self):
        '''test MAV_CMD_DO_GO_AROUND as a mavlink command'''
        self._MAV_CMD_DO_GO_AROUND(self.run_cmd)
        self._MAV_CMD_DO_GO_AROUND(self.run_cmd_int)

    def _MAV_CMD_DO_FLIGHTTERMINATION(self, command):
        self.set_parameters({
            "AFS_ENABLE": 1,
            "MAV_GCS_SYSID": self.mav.source_system,
            "AFS_TERM_ACTION": 42,
        })
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        command(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, p1=1)
        self.wait_disarmed()
        self.wait_text('Terminating due to GCS request', check_context=True)
        self.reboot_sitl()

    def MAV_CMD_DO_FLIGHTTERMINATION(self):
        '''test MAV_CMD_DO_FLIGHTTERMINATION works on Plane'''
        self._MAV_CMD_DO_FLIGHTTERMINATION(self.run_cmd)
        self._MAV_CMD_DO_FLIGHTTERMINATION(self.run_cmd_int)

    def MAV_CMD_DO_LAND_START(self):
        '''test MAV_CMD_DO_LAND_START as mavlink command'''
        self.set_parameters({
            "RTL_AUTOLAND": 3,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 30),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.start_subtest("DO_LAND_START as COMMAND_LONG")
        self.wait_current_waypoint(2)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START)
        self.wait_current_waypoint(4)

        self.start_subtest("DO_LAND_START as COMMAND_INT")
        self.set_current_waypoint(2)
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_LAND_START)
        self.wait_current_waypoint(4)

        self.fly_home_land_and_disarm()

    def MAV_CMD_NAV_ALTITUDE_WAIT(self):
        '''test MAV_CMD_NAV_ALTITUDE_WAIT mission item, wiggling only'''

        # Load a single waypoint
        self.upload_simple_relhome_mission([
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_ALTITUDE_WAIT,
                p1=1000, # 1000m alt threshold, this should not trigger
                p2=10, # 10m/s descent rate, this should not trigger
                p3=10 # servo wiggle every 10 seconds
            )
        ])

        # Set initial conditions for servo wiggle testing
        servo_wiggled = {1: False, 2: False, 4: False}

        def look_for_wiggle(mav, m):
            if m.get_type() == 'SERVO_OUTPUT_RAW':
                # Throttle must be zero
                if m.servo3_raw != 1000:
                    raise NotAchievedException(
                        "Throttle must be 0 in altitude wait, got %f" % m.servo3_raw)

                # Check if all servos wiggle
                if m.servo1_raw != 1500:
                    servo_wiggled[1] = True
                if m.servo2_raw != 1500:
                    servo_wiggled[2] = True
                if m.servo4_raw != 1500:
                    servo_wiggled[4] = True

        # Start mission
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Check outputs
        self.context_push()
        self.install_message_hook_context(look_for_wiggle)

        # Wait for a bit to let message hook sample
        self.delay_sim_time(60)

        self.context_pop()

        # If the mission item completes as there is no other waypoints we will end up in RTL
        if not self.mode_is('AUTO'):
            raise NotAchievedException("Must still be in AUTO")

        # Raise error if not all servos have wiggled
        if not all(servo_wiggled.values()):
            raise NotAchievedException("Not all servos have moved within the test frame")

        self.disarm_vehicle()

    def InteractTest(self):
        '''just takeoff'''

        if self.mavproxy is None:
            raise NotAchievedException("Must be started with --map")

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 800, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 400, 0),
        ])

        self.wait_current_waypoint(4)

        self.set_parameter('SIM_SPEEDUP', 1)

        self.mavproxy.interact()

    def MAV_CMD_MISSION_START(self):
        '''test MAV_CMD_MISSION_START starts AUTO'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])
        for run_cmd in self.run_cmd, self.run_cmd_int:
            self.change_mode('LOITER')
            run_cmd(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_mode('AUTO')

    def MAV_CMD_NAV_LOITER_UNLIM(self):
        '''test receiving MAV_CMD_NAV_LOITER_UNLIM from GCS'''
        self.takeoff(10)
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
        self.wait_mode('LOITER')
        self.change_mode('GUIDED')
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
        self.wait_mode('LOITER')
        self.fly_home_land_and_disarm()

    def MAV_CMD_NAV_RETURN_TO_LAUNCH(self):
        '''test receiving MAV_CMD_NAV_RETURN_TO_LAUNCH from GCS'''
        self.set_parameter('RTL_AUTOLAND', 1)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 30),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])

        for i in self.run_cmd, self.run_cmd_int:
            self.wait_current_waypoint(2)
            self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            self.wait_current_waypoint(4)
            self.set_current_waypoint(2)
        self.fly_home_land_and_disarm()

    def location_from_ADSB_VEHICLE(self, m):
        '''return a mavutil.location extracted from an ADSB_VEHICLE mavlink
        message'''
        if m.altitude_type != mavutil.mavlink.ADSB_ALTITUDE_TYPE_GEOMETRIC:
            raise ValueError("Expected geometric alt")
        return mavutil.location(
            m.lat*1e-7,
            m.lon*1e-7,
            m.altitude/1000.0585,  # mm -> m
            m.heading * 0.01  # centidegrees -> degrees
        )

    def SagetechMXS(self):
        '''test Sagetech MXS ADSB device driver'''
        sim_name = "sagetech_mxs"
        self.set_parameters({
            "SERIAL5_PROTOCOL": 35,
            "ADSB_TYPE": 4,  # Sagetech-MXS
            "SIM_ADSB_TYPES": 8,  # Sagetech-MXS
            "SIM_ADSB_COUNT": 5,
        })
        self.customise_SITL_commandline(["--serial5=sim:%s" % sim_name])
        m = self.assert_receive_message("ADSB_VEHICLE")
        adsb_vehicle_loc = self.location_from_ADSB_VEHICLE(m)
        self.progress("ADSB Vehicle at loc %s" % str(adsb_vehicle_loc))
        home = self.home_position_as_mav_location()
        self.assert_distance(home, adsb_vehicle_loc, 0, 10000)

    def MinThrottle(self):
        '''Make sure min throttle does not apply in manual mode and does in FBWA'''

        servo_min = self.get_parameter("RC3_MIN")
        servo_max = self.get_parameter("RC3_MAX")
        min_throttle = 10
        servo_min_throttle = servo_min + (servo_max - servo_min) * (min_throttle / 100)

        # Set min throttle
        self.set_parameter('THR_MIN', min_throttle)

        # Should be 0 throttle while disarmed
        self.change_mode("MANUAL")
        self.drain_mav() # make sure we have the latest data before checking throttle output
        self.assert_servo_channel_value(3, servo_min)

        # Arm and check throttle is still 0 in manual
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.drain_mav()
        self.assert_servo_channel_value(3, servo_min)

        # FBWA should apply throttle min
        self.change_mode("FBWA")
        self.drain_mav()
        self.assert_servo_channel_value(3, servo_min_throttle)

        # But not when disarmed
        self.disarm_vehicle()
        self.drain_mav()
        self.assert_servo_channel_value(3, servo_min)

    def ClimbThrottleSaturation(self):
        '''check what happens when throttle is saturated in GUIDED'''
        self.set_parameters({
            "TECS_CLMB_MAX": 30,
            "TKOFF_ALT": 1000,
        })

        self.change_mode("TAKEOFF")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_message_field_values('VFR_HUD', {
            "throttle": 100,
        }, minimum_duration=30, timeout=90)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def GuidedAttitudeNoGPS(self):
        '''test that guided-attitude still works with no GPS'''
        self.takeoff(50)
        self.change_mode('GUIDED')
        self.context_push()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(30)
        self.set_attitude_target()
        self.context_pop()
        self.fly_home_land_and_disarm()

    def ScriptStats(self):
        '''test script stats logging'''
        self.context_push()
        self.set_parameters({
            'SCR_ENABLE': 1,
            'SCR_DEBUG_OPTS': 8,  # runtime memory usage and time
        })
        self.install_test_scripts_context([
            "math.lua",
            "strings.lua",
        ])
        self.install_example_script_context('simple_loop.lua')
        self.context_collect('STATUSTEXT')

        self.reboot_sitl()

        self.wait_statustext('hello, world')
        delay = 20
        self.delay_sim_time(delay, reason='gather some stats')
        self.wait_statustext("math.lua exceeded time limit", check_context=True, timeout=0)

        dfreader = self.dfreader_for_current_onboard_log()
        seen_hello_world = False
#        runtime = None
        while True:
            m = dfreader.recv_match(type=['SCR'])
            if m is None:
                break
            if m.Name == "simple_loop.lua":
                seen_hello_world = True
#            if m.Name == "math.lua":
#                runtime = m.Runtime

        if not seen_hello_world:
            raise NotAchievedException("Did not see simple_loop.lua script")

#        self.progress(f"math took {runtime} seconds to run over {delay} seconds")
#        if runtime == 0:
#            raise NotAchievedException("Expected non-zero runtime for math")

        self.context_pop()
        self.reboot_sitl()

    def GPSPreArms(self):
        '''ensure GPS prearm checks work'''
        self.wait_ready_to_arm()
        self.start_subtest('DroneCAN sanity checks')
        self.set_parameter('GPS1_TYPE', 9)
        self.set_parameter('GPS2_TYPE', 9)
        self.set_parameter('GPS1_CAN_OVRIDE', 130)
        self.set_parameter('GPS2_CAN_OVRIDE', 130)
        self.assert_prearm_failure(
            "set for multiple GPS",
            other_prearm_failures_fatal=False,
        )

    def SetHomeAltChange(self):
        '''check modes retain altitude when home alt changed'''
        for mode in 'FBWB', 'CRUISE', 'LOITER':
            self.set_rc(3, 1000)
            self.wait_ready_to_arm()
            home = self.home_position_as_mav_location()
            target_alt = 20
            self.takeoff(target_alt, mode="TAKEOFF")
            self.delay_sim_time(20)  # Give some time to altitude to stabilize.
            self.set_rc(3, 1500)
            self.change_mode(mode)
            higher_home = copy.copy(home)
            higher_home.alt += 40
            self.set_home(higher_home)
            self.wait_altitude(home.alt+target_alt-5, home.alt+target_alt+5, relative=False, minimum_duration=10, timeout=12)
            self.disarm_vehicle(force=True)
            self.reboot_sitl()

    def SetHomeAltChange2(self):
        '''ensure TECS operates predictably as home altitude changes continuously'''
        '''
        This can happen when performing a ship landing, where the home
        coordinates are continuously set by the ship GNSS RX.
        '''
        self.set_parameter('TRIM_THROTTLE', 70)
        self.wait_ready_to_arm()
        home = self.home_position_as_mav_location()
        target_alt = 20
        self.takeoff(target_alt, mode="TAKEOFF")
        self.change_mode("LOITER")
        self.delay_sim_time(20) # Let the plane settle.

        tstart = self.get_sim_time()
        test_time = 10 # Run the test for 10s.
        pub_freq = 10
        for i in range(test_time*pub_freq):
            tnow = self.get_sim_time()
            higher_home = copy.copy(home)
            # Produce 1Hz sine waves in home altitude change.
            higher_home.alt += 40*math.sin((tnow-tstart)*(2*math.pi))
            self.set_home(higher_home)
            if tnow-tstart > test_time:
                break
            self.delay_sim_time(1.0/pub_freq)

        # Test if the altitude is still within bounds.
        self.wait_altitude(home.alt+target_alt-5, home.alt+target_alt+5, relative=False, minimum_duration=1, timeout=2)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def SetHomeAltChange3(self):
        '''same as SetHomeAltChange, but the home alt change occurs during TECS operation'''
        self.wait_ready_to_arm()
        home = self.home_position_as_mav_location()
        target_alt = 20
        self.takeoff(target_alt, mode="TAKEOFF")
        self.change_mode("LOITER")
        self.delay_sim_time(20) # Let the plane settle.

        higher_home = copy.copy(home)
        higher_home.alt += 40
        self.set_home(higher_home)
        self.wait_altitude(home.alt+target_alt-5, home.alt+target_alt+5, relative=False, minimum_duration=10, timeout=10.1)

        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def ForceArm(self):
        '''check force-arming functionality'''
        self.set_parameter("SIM_GPS1_ENABLE", 0)
        # 21196 is the mavlink standard, 2989 is legacy
        for magic_value in 21196, 2989:
            self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK,
                                   present=True,
                                   enabled=True,
                                   healthy=False,
                                   )
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1,  # ARM
                p2=0,
                want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            )
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1,  # ARM
                p2=magic_value,
                want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )
            self.disarm_vehicle()

    def CompassLearnInFlight(self):
        '''check we can learn compass offsets in flight'''
        self.context_push()
        self.set_parameters({
            "COMPASS_OFS_X": 1100,
        })
        self.assert_prearm_failure("Check mag field", other_prearm_failures_fatal=False)
        self.context_pop()
        self.wait_ready_to_arm()
        self.takeoff(30, mode='TAKEOFF')
        self.assert_parameter_value("COMPASS_OFS_X", 20, epsilon=30)
        # fly straight and level for a bit to let GSF converge for accurate learning
        self.change_mode("FBWB") # not "CRUISE" to avoid heading track with bad compass
        self.wait_distance(200, accuracy=20)
        old_compass_ofs_x = self.get_parameter('COMPASS_OFS_X')
        self.set_parameters({
            "COMPASS_OFS_X": 1100,
        })
        self.send_set_parameter("COMPASS_LEARN", 3)  # 3 is in-flight learning
        self.wait_parameter_value("COMPASS_LEARN", 0)
        self.assert_parameter_value("COMPASS_OFS_X", old_compass_ofs_x, epsilon=30)
        self.fly_home_land_and_disarm()
        self.reboot_sitl()
        self.assert_parameter_value("COMPASS_OFS_X", old_compass_ofs_x, epsilon=30)

    def _MAV_CMD_EXTERNAL_WIND_ESTIMATE(self, command):
        self.reboot_sitl()

        def cmp_with_variance(a, b, p):
            return abs(a - b) < p

        def check_eq(speed, direction, ret_dir, timeout=1):
            command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=speed, p3=direction)

            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > timeout:
                    raise NotAchievedException(
                        f"Failed to set wind speed or/and direction: speed != {speed} or direction != {direction}")

                m = self.assert_receive_message("WIND")
                if cmp_with_variance(m.speed, speed, 0.5) and cmp_with_variance(m.direction, ret_dir, 5):
                    return True

        check_eq(1, 45, 45)
        check_eq(2, 90, 90)
        check_eq(3, 120, 120)
        check_eq(4, 180, -180)
        check_eq(5, 240, -120)
        check_eq(6, 320, -40)
        check_eq(7, 360, 0)

        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=-2, p3=90, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=2, p3=-90, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=-2, p3=-90, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
        command(mavutil.mavlink.MAV_CMD_EXTERNAL_WIND_ESTIMATE, p1=2, p3=370, want_result=mavutil.mavlink.MAV_RESULT_DENIED)

    def FenceDoubleBreach(self):
        '''test breaching the fence twice'''
        self.wait_ready_to_arm()

        fence_centre_ne = (0, -500)

        fence_centre = self.mav.location()
        fence_centre = self.offset_location_ne(fence_centre, fence_centre_ne[0], fence_centre_ne[1])

        self.set_parameters({
            "RTL_AUTOLAND": 2,
        })

        alt = 50
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, fence_centre_ne[0], fence_centre_ne[1], alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -750, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1000, alt),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 800, alt),
        ])

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                self.offset_location_ne(fence_centre, -200, -200), # bl
                self.offset_location_ne(fence_centre, -200, 200), # br
                self.offset_location_ne(fence_centre, 200, 200), # tr
                self.offset_location_ne(fence_centre, 200, -200), # tl,
            ]),
        ])

        self.do_fence_enable()

        self.takeoff(mode='FBWA')
        self.set_rc(3, 1500)

        self.change_mode('AUTO')

        self.context_collect('STATUSTEXT')

        self.wait_statustext('Polygon fence breached', timeout=300)
        self.wait_current_waypoint(6)
        self.wait_distance_to_location(fence_centre, 350, 20000)

        self.set_current_waypoint(2)

        self.wait_statustext('Polygon fence breached', timeout=300)
        self.wait_current_waypoint(6, timeout=5)
        self.fly_home_land_and_disarm()

    def MAV_CMD_EXTERNAL_WIND_ESTIMATE(self):
        '''test MAV_CMD_EXTERNAL_WIND_ESTIMATE as a mavlink command'''
        self._MAV_CMD_EXTERNAL_WIND_ESTIMATE(self.run_cmd)
        self._MAV_CMD_EXTERNAL_WIND_ESTIMATE(self.run_cmd_int)

    def LoggedNamedValueFloat(self):
        '''ensure that sent named value floats are logged'''
        self.context_push()
        self.install_example_script_context('simple_loop.lua')
        self.set_parameters({
            'SCR_ENABLE': 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.wait_statustext('hello, world')
        m = self.assert_received_message_field_values('NAMED_VALUE_FLOAT', {
            "name": "Lua Float",
        })
        dfreader = self.dfreader_for_current_onboard_log()
        self.context_pop()

        m = dfreader.recv_match(type='NVF')
        if m is None:
            raise NotAchievedException("Did not find NVF message")
        self.progress(f"Received NVF with value {m.Value}")

    def LoggedNamedValueString(self):
        '''ensure that sent named value strings are logged'''
        self.context_push()
        self.install_example_script_context('simple_named_string.lua')
        self.set_parameters({
            'SCR_ENABLE': 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        m = self.assert_received_message_field_values('NAMED_VALUE_STRING', {
            "name": "Lua String",
            "value": "Lua String Value",
        })
        dfreader = self.dfreader_for_current_onboard_log()
        self.context_pop()

        m = dfreader.recv_match(type='NVS')
        if m is None:
            raise NotAchievedException("Did not find NVS message")
        self.progress(f"Received NVS with value {m.Value}")
        if m.Name != 'Lua String':
            raise NotAchievedException("Unexpected name in NVS")
        if m.Value != 'Lua String Value':
            raise NotAchievedException("Unexpected value in NVS")

    def GliderPullup(self):
        '''test pullup of glider after ALTITUDE_WAIT'''
        self.start_subtest("test glider pullup")

        self.customise_SITL_commandline(
            [],
            model="glider",
            defaults_filepath="Tools/autotest/default_params/glider.parm",
            wipe=True)

        self.set_parameter('LOG_DISARMED', 1)

        self.set_parameters({
            "PUP_ENABLE": 1,
            "SERVO6_FUNCTION": 0, # balloon lift
            "SERVO10_FUNCTION": 156, # lift release
            "EK3_IMU_MASK": 1, # lane switches just make log harder to read
            "AHRS_OPTIONS": 4, # don't disable airspeed based on EKF checks
            "ARSPD_OPTIONS": 0, # don't disable airspeed
            "ARSPD_WIND_GATE": 0,
        })

        self.set_servo(6, 1000)

        self.load_mission("glider-pullup-mission.txt")
        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')

        self.progress("Start balloon lift")
        self.set_servo(6, 2000)

        self.wait_text("Reached altitude", check_context=True, timeout=1000)
        self.wait_text("Start pullup airspeed", check_context=True)
        self.wait_text("Pullup airspeed", check_context=True)
        self.wait_text("Pullup pitch", check_context=True)
        self.wait_text("Pullup level", check_context=True)
        self.wait_text("Loiter to alt complete", check_context=True, timeout=1000)
        self.wait_text("Flare", check_context=True, timeout=400)
        self.wait_text("Auto disarmed", check_context=True, timeout=200)

    def BadRollChannelDefined(self):
        '''ensure we don't die with a  bad Roll channel defined'''
        self.set_parameter("RCMAP_ROLL", 17)

    def MAV_CMD_NAV_LOITER_TO_ALT(self):
        '''test loiter to alt mission item'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 0, 0, 500),
            (mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_altitude(450, 475, relative=True, timeout=600)
        self.wait_altitude(75, 125, relative=True, timeout=600)
        self.wait_current_waypoint(4)
        self.fly_home_land_and_disarm()

    def RudderArmedTakeoffRequiresNeutralThrottle(self):
        '''auto-takeoff should not occur while rudder continues to be held over'''
        self.change_mode('TAKEOFF')
        self.wait_ready_to_arm()
        self.set_rc(4, 1000)
        self.wait_armed()
        self.wait_groundspeed(0, 1, minimum_duration=10)
        self.set_rc(4, 1500)
        self.wait_groundspeed(5, 100)
        self.fly_home_land_and_disarm()

    def VolzMission(self):
        '''test Volz serially-connected servos in a mission'''
        volz_motor_mask = ((1 << 0) | (1 << 1) | (1 << 3) | (1 << 8) | (1 << 9) | (1 << 11))
        self.set_parameters({
            'SERIAL5_PROTOCOL': 14,
            'SERVO_VOLZ_MASK': volz_motor_mask,
            'RTL_AUTOLAND': 2,

            'SIM_VOLZ_ENA': 1,
            'SIM_VOLZ_MASK': volz_motor_mask,
        })
        # defaults file not working?
        self.set_parameters({
            "SERVO2_REVERSED":  0,  # elevator

            "SERVO9_FUNCTION": 4,

            "SERVO10_FUNCTION": 19,  # elevator

            "SERVO12_FUNCTION": 21,  # rudder
            "SERVO12_REVERSED":  1,  # rudder

        })
        self.customise_SITL_commandline([
            "--serial5=sim:volz",
        ], model="plane-redundant",
                                        )
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff()
        self.fly_home_land_and_disarm()

    def DO_CHANGE_ALTITUDE(self):
        '''test DO_CHANGE_ALTITUDE mavlink command'''
        takeoff_alt = 30
        self.takeoff(alt=takeoff_alt, mode='TAKEOFF')
        self.wait_altitude(takeoff_alt-1, takeoff_alt+1, minimum_duration=10, relative=True, timeout=60)

        self.start_subtest("Home-relative altitude")
        target_rel_alt = 40
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_rel_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            target_rel_alt-1,
            target_rel_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )

        self.start_subtest("Absolute altitude")
        current_abs_alt = self.get_altitude()
        target_abs_alt = current_abs_alt + 30
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_abs_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL,
        )
        self.wait_altitude(
            target_abs_alt-1,
            target_abs_alt+1,
            minimum_duration=10,
            timeout=60,
        )

        self.start_subtest("Terrain altitude")
        current_relative_alt = self.get_altitude(relative=True)
        target_terr_alt = current_relative_alt + 10
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_terr_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )
        self.wait_altitude(
            target_terr_alt-1,
            target_terr_alt+1,
            minimum_duration=10,
            relative=True,
            altitude_source="TERRAIN_REPORT.current_height",
            timeout=120,
        )

        self.start_subtest("Change alt in loiter")
        self.change_mode('LOITER')
        current_relative_alt = self.get_altitude(relative=True)
        self.wait_altitude(
            current_relative_alt-1,
            current_relative_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )
        target_loiter_alt = current_relative_alt+5
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_loiter_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            target_loiter_alt-1,
            target_loiter_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )

        self.start_subtest("Change alt in circle")
        self.change_mode('CIRCLE')
        current_relative_alt = self.get_altitude(relative=True)
        self.wait_altitude(
            current_relative_alt-1,
            current_relative_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )
        target_circle_alt = current_relative_alt-5
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_circle_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            target_circle_alt-1,
            target_circle_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )

        self.start_subtest("Immediately respond to DO_CHANGE_ALTITUDE in a mission")
        current_relative_alt = self.get_altitude(relative=True)
        mission_alt = current_relative_alt - 10
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -2000, 0, mission_alt)
        ])
        self.change_mode('AUTO')
        self.wait_altitude(
            mission_alt-1,
            mission_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=240,
        )
        self.fly_home_land_and_disarm()

    def SET_POSITION_TARGET_GLOBAL_INT_for_altitude(self):
        '''test changing altitude using SET_POSITION_TARGET_GLOBAL_INT_for_altitude in guided mode'''
        self.takeoff(30, mode='TAKEOFF')
        self.change_mode('GUIDED')
        target_alt = 40
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            self.mav.target_system, # target system_id
            self.mav.target_component, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.ALT_ONLY,
            0, # lat
            0, # lon
            target_alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        self.wait_altitude(
            target_alt-1,
            target_alt+1,
            minimum_duration=10,
            timeout=120,
            relative=True,
        )

        self.progress("Ensure ignore bit is honoured")
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            self.mav.target_system, # target system_id
            self.mav.target_component, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.IGNORE_ALL, # mask specifying use-only-alt
            0, # lat
            0, # lon
            target_alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        self.wait_altitude(
            target_alt-1,
            target_alt+1,
            timeout=60,
            minimum_duration=10,
            relative=True,
        )
        self.fly_home_land_and_disarm()

    def mavlink_AIRSPEED(self):
        '''check receiving of two airspeed sensors'''
        self.set_parameters({
            "ARSPD_PIN": 2,
            "ARSPD_RATIO": 0,
            "ARSPD2_RATIO": 4,
            "ARSPD2_TYPE": 3,  # MS5525
            "ARSPD2_BUS": 1,
            "ARSPD2_AUTOCAL": 1,
        })
        self.reboot_sitl()

        self.start_subtest('Ensure we get both instances')
        self.wait_message_field_values('AIRSPEED', {
            "id": 0,
            "flags": mavutil.mavlink.AIRSPEED_SENSOR_USING,
        })
        self.wait_message_field_values('AIRSPEED', {
            "id": 1,
            "flags": 0,
        })

        self.wait_ready_to_arm()
        self.takeoff()

        self.start_subtest("Now testing failure of sensor 1 - fail to many m/s")
        self.set_parameter("SIM_ARSPD_FAIL", 60)

        # airspeed sensor never becomes unhealthy - we just stop using
        # it as EKF3 starts to reject:
        self.wait_message_field_values('AIRSPEED', {
            "id": 0,
            "flags": 0,
        })
        # ArduPilot's airspeed redundancy is only available through
        # EKF3 affinity:
        self.progress("Checking we're not using second airspeed sensor")
        self.wait_message_field_values('AIRSPEED', {
            "id": 1,
            "flags": 0,
        })
        self.set_parameter("SIM_ARSPD_FAIL", 0)
        self.fly_home_land_and_disarm()

    def RudderArmingWithArmingChecksSkipped(self):
        '''check we can't arm with rudder even if all checks are skipped'''
        self.set_parameters({
            "ARMING_RUDDER": 0,
            "ARMING_SKIPCHK": -1,
            "RC4_REVERSED": 0,
        })
        self.reboot_sitl()
        self.delay_sim_time(5)
        self.set_rc(4, 2000)
        w = vehicle_test_suite.WaitAndMaintainDisarmed(
            self,
            minimum_duration=30,
            timeout=60,
        )
        w.run()

    def Volz(self):
        '''test Volz serially-connected'''
        volz_motor_mask = ((1 << 0) | (1 << 1) | (1 << 3) | (1 << 8) | (1 << 9) | (1 << 11))
        self.set_parameters({
            'SERIAL5_PROTOCOL': 14,
            'SERVO_VOLZ_MASK': volz_motor_mask,
            'RTL_AUTOLAND': 2,

            'SIM_VOLZ_ENA': 1,
            'SIM_VOLZ_MASK': volz_motor_mask,
        })
        # defaults file not working?
        self.set_parameters({
            "SERVO2_REVERSED":  0,  # elevator

            "SERVO9_FUNCTION": 4,

            "SERVO10_FUNCTION": 19,  # elevator

            "SERVO12_FUNCTION": 21,  # rudder
            "SERVO12_REVERSED":  1,  # rudder

        })
        self.customise_SITL_commandline([
            "--serial5=sim:volz",
        ], model="plane-redundant",
                                        )
        self.wait_ready_to_arm()
        self.takeoff()
        self.change_mode('FBWA')
        straight_and_level_text = "straight-and-level"
        self.send_statustext(straight_and_level_text)
        self.delay_sim_time(2)
        self.progress("sticking servo with constant deflection")
        self.set_rc(1, 1400)
        self.change_mode('MANUAL')
        self.delay_sim_time(0.5)
        self.progress("Failing servo")
        self.set_parameter('SIM_VOLZ_FMASK', 1)
        self.set_rc(1, 1500)
        self.change_mode('FBWA')
        aileron_failed_text = "aileron has been failed"
        self.send_statustext(aileron_failed_text)
        self.delay_sim_time(15)
        self.set_parameter('SIM_VOLZ_FMASK', 0)

        log_filepath = self.current_onboard_log_filepath()
        # terminate vehicle in-flight so our tests aren't fooled by the
        # "flying home" data:
        self.reboot_sitl(force=True)

        self.progress("Inspecting DFReader to ensure servo failure is recorded in the log")
        dfreader = self.dfreader_for_path(log_filepath)
        while True:
            m = dfreader.recv_match(type=['MSG'])
            if m is None:
                raise NotAchievedException("Did not see straight_and_level_text")
            if m.Message == "SRC=250/250:" + straight_and_level_text:
                break

        self.progress("Ensuring deflections are close to zero in straight-and-level flight")
        chan1_good = False
        chan9_good = False
        while not (chan1_good and chan9_good):
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see chan1 and chan9 as close-to-0")
            if m.get_type() != 'CSRV':
                continue
            if m.Id == 0 and abs(m.Pos) < 3:
                chan1_good = True
            elif m.Id == 8 and abs(m.Pos) < 3:
                chan9_good = True

        while True:
            m = dfreader.recv_match(type=['MSG'])
            if m is None:
                raise NotAchievedException("Did not see aileron_failed_text")
            if m.Message == "SRC=250/250:" + aileron_failed_text:
                break

        self.progress("Checking servo9 is deflected")
        while True:
            # m = dfreader.recv_match(type=['CSRV'])
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see chan9 deflection")
            if m.get_type() != 'CSRV':
                continue
            if m.Id != 8:
                continue
            if m.Pos < 20:
                continue
            self.progress(f"Chan9 is deflected ({m})")
            break

        self.progress("Ensuring the vehicle stabilised with a single aileron")
        attitude_good_count = 0
        while attitude_good_count < 5:
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see good attitude")
            if m.get_type() != 'ATT':
                continue
            if abs(m.Roll) >= 5:
                attitude_good_count = 0
                continue
            attitude_good_count += 1
        self.progress(f"Attitude is stabilised ({m})")

        self.progress("Ensure the roll integrator is wound up")
        while True:
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see wound-up roll integrator")
            if m.get_type() != 'PIDR':
                continue
            if m.I > 5:
                self.progress(f"Roll integrator is wound up ({m})")
                break

        self.progress("Checking that aileron is stuck at some deflection")
        good_count = 0
        while good_count < 5:
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see csrv Pos/PosCmd discrepancy")
            if m.get_type() != 'CSRV':
                continue
            if m.Id != 0:
                continue
            delta = abs(m.Pos - m.PosCmd)
            if delta <= 20:
                self.progress(f"CSRV Pos/PosCmd {delta=:.2f} BAD {m}")
                good_count = 0
                continue
            self.progress(f"CSRV Pos/PosCmd {delta=:.2f} OK {m}")
            good_count += 1

    def MAV_CMD_NAV_LOITER_TURNS_zero_turn(self):
        '''Ensure air vehicle achieves loiter target before exiting'''
        offset = 500
        alt = self.get_parameter("RTL_ALTITUDE")
        waypoint_radius = 100

        loiter_turns_loc_ccw = self.home_relative_loc_ne(offset, offset)
        loiter_turns_loc_cw = self.home_relative_loc_ne(offset, -offset)

        # upload a mission plan containing zero-turn loiters
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 10),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=0,                # Turns
                p3=-waypoint_radius, # Radius (If positive loiter clockwise, else counter-clockwise)
                x=int(loiter_turns_loc_ccw.lat*1e7),
                y=int(loiter_turns_loc_ccw.lng*1e7),
                z=alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=0,               # Turns
                p3=waypoint_radius, # Radius (If positive loiter clockwise, else counter-clockwise)
                x=int(loiter_turns_loc_cw.lat*1e7),
                y=int(loiter_turns_loc_cw.lng*1e7),
                z=alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_JUMP,
                p1=2, # waypoint to jump to
                p2=1  # number of jumps (-1: infinite)
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -offset, -offset, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -offset, offset, alt),
        ])

        self.change_mode("AUTO")
        self.wait_ready_to_arm()

        self.arm_vehicle()

        # check the vehicle is flying to each waypoint as expected
        self.wait_distance_to_waypoint(2, distance_min=90, distance_max=110, timeout=90) # North East Loiter
        self.wait_distance_to_waypoint(3, distance_min=90, distance_max=110, timeout=90) # North West Loiter

        # Jump to first 0-Turn Loiter
        self.wait_distance_to_waypoint(2, distance_min=90, distance_max=110, timeout=90) # North East Loiter

        # Ensure we go back through this loiter point
        self.wait_distance_to_waypoint(3, distance_min=90, distance_max=110, timeout=90) # North West Loiter

        self.wait_distance_to_waypoint(5, distance_min=10, distance_max=20, timeout=90)  # South West Waypoint
        self.wait_distance_to_waypoint(6, distance_min=10, distance_max=20, timeout=90)  # South East Waypoint

        self.fly_home_land_and_disarm()

    class ValidateVFRHudClimbAgainstSimState(vehicle_test_suite.TestSuite.MessageHook):
        '''monitors VFR_HUD to make sure reported climbrate is in-line with SIM_STATE.vd'''
        def __init__(self, suite, max_allowed_divergence=5):
            super(AutoTestPlane.ValidateVFRHudClimbAgainstSimState, self).__init__(suite)
            self.max_allowed_divergence = max_allowed_divergence  # m/s
            self.max_divergence = 0
            self.vfr_hud = None
            self.sim_state = None
            self.last_print = 0
            self.min_print_interval = 1  # seconds
            self.instafail = True
            self.failed = False

        def progress_prefix(self):
            return "VVHCASS: "

        def process(self, mav, m):
            if m.get_type() == 'VFR_HUD':
                self.vfr_hud = m
            elif m.get_type() == 'SIM_STATE':
                self.sim_state = m
            if self.vfr_hud is None:
                return
            if self.sim_state is None:
                return

            vfr_hud_climb = self.vfr_hud.climb
            sim_state_climb = -self.sim_state.vd
            divergence = abs(vfr_hud_climb - sim_state_climb)
            if (time.time() - self.last_print > self.min_print_interval or
                    divergence > self.max_divergence):
                self.progress(f"climb delta is {divergence}")
                self.last_print = time.time()
            if divergence > self.max_divergence:
                self.max_divergence = divergence
            if divergence > self.max_allowed_divergence:
                msg = f"VFR_HUD.climb diverged from SIM_STATE.vd by {divergence}m/s (max={self.max_allowed_divergence}m/s"
                if self.instafail:
                    raise NotAchievedException(msg)
                else:
                    self.failed = True
                    self.progress(msg)

        def hook_removed(self):
            if self.vfr_hud is None:
                raise ValueError("Did not receive VFR_HUD")
            if self.sim_state is None:
                raise ValueError("Did not receive SIM_STATE")
            msg = f"Maximum divergence was {self.max_divergence}m/s (max={self.max_allowed_divergence}m/s)"
            if self.failed:
                raise NotAchievedException(msg)

            self.progress(msg)

    def SoaringClimbRate(self):
        '''test displayed climb rate when soaring'''
        self.set_parameters({
            'RC16_OPTION': 88,  # soaring enable
            'SOAR_ENABLE': 1,
        })
        self.set_rc(16, 1000)  # disable soaring
        self.reboot_sitl()
        self.set_message_rate_hz('SIM_STATE', 10)
        self.install_message_hook_context(AutoTestPlane.ValidateVFRHudClimbAgainstSimState(self))
        self.takeoff(20)
        self.change_mode('FBWB')
        self.set_rc(2, 1000)  # full climb
        self.delay_sim_time(10)
        self.set_rc(16, 2000)  # enable soaring
        self.delay_sim_time(10)

        self.set_rc(2, 1500)
        self.fly_home_land_and_disarm()

    def ScriptedArmingChecksApplet(self):
        """ Applet for Arming Checks will prevent a vehicle from arming based on scripted checks
            """
        self.start_subtest("Scripted Arming Checks Applet validation")
        self.context_collect("STATUSTEXT")

        """Initialize the FC"""
        self.set_parameter("SCR_ENABLE", 1)
        self.install_applet_script_context("arming-checks.lua")
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)

        self.start_subsubtest("ArmCk: MAV_SYSID not set")
        self.progress("Currently SYSID is %f" % self.get_parameter('MAV_SYSID'))
        self.wait_text("ArmCk: warn: MAV_SYSID not set", timeout=30, check_context=True, regex=True)
        """ disable the SYSID check, since autotest doesn't like changing the sysid"""
        self.set_parameter("ARM_SYSID", -1)

        ''' This check comes first since its part of the standard parameters - an invalid scaling speed'''
        self.start_subsubtest("ArmCk: SCALING_SPEED close to AIRSPEED_CRUISE")
        self.assert_prearm_failure("fail: Scaling spd", other_prearm_failures_fatal=False)
        self.set_parameter("SCALING_SPEED", 22)
        self.wait_text("clear: Scaling spd", check_context=True)

        self.start_subsubtest("ArmCk: FOLL_SYSID must be set if FOLL_ENABLE = 1")
        self.set_parameter("FOLL_ENABLE", 1)
        self.set_parameter("FOLL_OFS_X", 10)
        self.assert_prearm_failure("FOLL_SYSID not set", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_SYSID", 3)
        self.wait_text("clear: FOLL_SYSID not set", check_context=True)
        self.set_parameter("FOLL_SYSID", -1)

        self.start_subsubtest("ArmCk: FOLL_OFS_[XYZ] must be set if FOLL_ENABLE = 1")
        self.set_parameter("FOLL_OFS_X", 0)
        self.assert_prearm_failure("FOLL_OFS_[XYZ] = 0", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_OFS_X", 10)
        self.wait_text("clear: FOLL_OFS_[XYZ] = 0", check_context=True)
        self.set_parameter("FOLL_OFS_X", 0)
        self.assert_prearm_failure("FOLL_OFS_[XYZ] = 0", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_OFS_Y", 10)
        self.wait_text("clear: FOLL_OFS_[XYZ] = 0", check_context=True)
        self.set_parameter("FOLL_OFS_Y", 0)
        self.assert_prearm_failure("FOLL_OFS_[XYZ] = 0", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_OFS_Z", 10)
        self.wait_text("clear: FOLL_OFS_[XYZ] = 0", check_context=True)
        """ clear these checks to make the context cleaner for subsequent tests """
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
        })

        self.start_subsubtest("ArmCk: RTL_ALTITUDE must be legal")
        self.progress("Currently ARM_P_RTL_ALT is %f" % self.get_parameter('ARM_P_RTL_ALT'))
        self.set_parameter("RTL_ALTITUDE", 150)
        self.assert_prearm_failure("ArmCk: fail: RTL_ALTITUDE too high", other_prearm_failures_fatal=False)
        self.set_parameter("RTL_ALTITUDE", 120)
        self.wait_text("clear: RTL_ALT", check_context=True)

        self.start_subsubtest("ArmCk: RTL_CLIMB_MIN must be legal")
        self.set_parameter("RTL_CLIMB_MIN", 150)
        self.wait_text("ArmCk: warn: RTL_CLIMB_MIN too high", check_context=True)
        self.set_parameter("RTL_CLIMB_MIN", 120)
        self.wait_text("clear: RTL_CLIMB_MIN too high", check_context=True)

        self.start_subsubtest("ArmCk: AIRSPEED stall < min < cruise < max")
        ''' Airspeed parameter start out as
        AIRSPEED_STALL = 0
        AIRSPEED_MIN = 10
        AIRSPEED_CRUISE = 22
        AIRSPEED_MAX = 30
        '''
        self.start_subsubtest("ArmCk: AIRSPEED max < others")
        self.set_parameter("AIRSPEED_MAX", 5)
        self.assert_prearm_failure("ArmCk: fail: stall < min", other_prearm_failures_fatal=False)
        self.set_parameter("AIRSPEED_MAX", 30)
        self.wait_text("clear: stall < min", check_context=True)
        self.start_subsubtest("ArmCk: AIRSPEED cruise < min")
        self.set_parameter("AIRSPEED_MIN", 25)
        self.assert_prearm_failure("ArmCk: fail: stall < min", other_prearm_failures_fatal=False)
        self.set_parameter("AIRSPEED_MIN", 10)
        self.wait_text("clear: stall < min", check_context=True)
        self.start_subsubtest("ArmCk: AIRSPEED cruise > max")
        self.set_parameter("AIRSPEED_CRUISE", 40)
        self.set_parameter("SCALING_SPEED", 40)
        self.assert_prearm_failure("ArmCk: fail: stall < min", other_prearm_failures_fatal=False)
        self.set_parameter("AIRSPEED_CRUISE", 22)
        self.set_parameter("SCALING_SPEED", 22)
        self.wait_text("clear: stall < min", check_context=True)

        self.start_subsubtest("ArmCk: AIRSPEED_MIN must be > AIRSPEED_STALL")
        ''' only applies if AIRSPEED_STALL is non zero'''
        self.set_parameter("AIRSPEED_STALL", 2)
        self.wait_text("ArmCk: info: Min Speed not", check_context=True)
        self.set_parameter("AIRSPEED_STALL", 8)

        self.start_subsubtest("ArmCk: Mount SYSID must not match FOLL_SYSID")
        self.set_parameter("ARM_SYSID", -1)
        ''' to fail the check MNTx_SYSID_DFLT must be non zero but not= FOLL_SYSID'''
        self.set_parameter("MNT1_SYSID_DFLT", 1)
        ''' Need a healthy camera mount defined for this to work '''
        self.setup_servo_mount()
        self.progress("rebooting to enable MNT1")
        self.reboot_sitl() # to handle MNT_TYPE changing
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.progress("Currently FOLL_SYSID is %f" % self.get_parameter('FOLL_SYSID'))
        self.progress("Currently MNT1_SYSID_DFLT is %f" % self.get_parameter('MNT1_SYSID_DFLT'))
        self.set_parameter("ARM_SYSID", -1)
        self.progress("Currently FOLL_SYSID is %f" % self.get_parameter('FOLL_SYSID'))
        self.progress("Currently MNT1_SYSID_DFLT is %f" % self.get_parameter('MNT1_SYSID_DFLT'))
        self.wait_text("warn: MNTx_SYSID != FOLL", check_context=True)

        self.start_subsubtest("ArmCk: Fence must be enabled or autoenabled (warning)")
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.load_fence("CMAC-fence.txt")
        self.wait_statustext("warn: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_ENABLE", 1)
        self.wait_statustext("clear: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_ENABLE", 0)
        self.wait_statustext("warn: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_AUTOENABLE", 1)
        self.wait_text("clear: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_AUTOENABLE", 0)
        self.wait_text("warn: Fence not enabled", check_context=True)

    def ScriptedArmingChecksAppletEStop(self):
        """ Applet for Arming Checks will prevent a vehicle from arming based on scripted checks
            """
        self.start_subtest("Scripted Arming Checks Applet validation")
        self.context_collect("STATUSTEXT")

        """Initialize the FC"""
        self.set_parameter("SCR_ENABLE", 1)
        self.install_applet_script_context("arming-checks.lua")
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
            "ARM_P_SCALING": -1,
        })

        self.start_subsubtest("ArmCk: Cannot arm while motors estopped")
        self.set_parameter("RC6_OPTION", 165)
        self.progress("rebooting to enable RC channel")
        self.reboot_sitl() # to handle RC option changing
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
            "ARM_P_SCALING": -1,
        })
        self.set_rc(6, 1000)
        self.assert_prearm_failure("ArmCk: fail: Motors EStopped", other_prearm_failures_fatal=False)
        self.set_rc(6, 2000)
        self.wait_text("clear: Motors EStopped", timeout=30, check_context=True, regex=True)
        self.wait_ready_to_arm()

    def ScriptedArmingChecksAppletRally(self):
        """ Applet for Arming Checks will prevent a vehicle from arming based on scripted checks
            """
        self.start_subtest("Scripted Arming Checks Applet validation")
        self.context_collect("STATUSTEXT")

        """Initialize the FC"""
        self.set_parameter("SCR_ENABLE", 1)
        self.install_applet_script_context("arming-checks.lua")
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)

        self.start_subsubtest("ArmCk: MAV_SYSID not set")
        self.progress("Currently SYSID is %f" % self.get_parameter('MAV_SYSID'))
        self.wait_text("ArmCk: warn: MAV_SYSID not set", timeout=30, check_context=True, regex=True)
        """ disable the SYSID check, since autotest doesn't like changing the sysid"""
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
            "ARM_P_SCALING": -1,
        })

        self.start_subsubtest("ArmCk: Rally Point must be < ARM_V_RALLY_MAX meters away")
        self.progress("Currently RALLY_LIMIT_KM is %f" % self.get_parameter('RALLY_LIMIT_KM'))
        loc = self.home_relative_loc_ne(6500, -50)
        self.upload_rally_points_from_locations([loc])
        self.wait_text("warn: Rally too far", check_context=True)
        self.set_parameter("RALLY_LIMIT_KM", 7)
        self.wait_text("clear: Rally too far", check_context=True)

    def PlaneFollowAppletSanity(self):
        '''PLane Follow Sanity Check, not a detailed test'''
        self.start_subtest("Plane Follow Script Load and Start")

        self.install_applet_script_context("plane_follow.lua")
        self.install_script_module(self.script_modules_source_path("pid.lua"), "pid.lua")
        self.install_script_module(self.script_modules_source_path("mavlink_attitude.lua"), "mavlink_attitude.lua")
        self.install_mavlink_module()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_SPEEDUP": 20, # need to give some cycles to lua
            "RC7_OPTION": 301,
        })

        self.context_collect("STATUSTEXT")

        self.reboot_sitl()

        self.wait_text("Plane Follow .* script loaded", timeout=30, regex=True, check_context=True)

        self.wait_ready_to_arm()
        self.set_rc(7, 2000)
        self.wait_text("PFollow: must be armed", check_context=True)
        self.set_rc(7, 1000)
        self.arm_vehicle()
        self.set_rc(7, 2000)
        self.wait_text("PFollow: enabled", check_context=True)
        self.set_rc(7, 1000)
        self.wait_text("PFollow: disabled", check_context=True)
        self.disarm_vehicle()

        self.reboot_sitl()
        # remove the installed modules.
        self.remove_installed_script_module("pid.lua")
        self.remove_installed_script_module("mavlink_attitude.lua")

    def PreflightRebootComponent(self):
        '''Ensure that PREFLIGHT_REBOOT commands sent to components don't reboot Autopilot'''
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            p1=1, # Reboot autopilot
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
            target_compid=mavutil.mavlink.MAV_COMP_ID_GIMBAL
        )

    def tests(self):
        '''return list of all tests'''
        ret = []
        ret.extend(self.tests1a())
        ret.extend(self.tests1b())
        ret.extend(self.tests1c())
        return ret

    def tests1a(self):
        ret = []
        ret = super(AutoTestPlane, self).tests()
        ret.extend([
            self.AuxModeSwitch,
            self.TestRCCamera,
            self.TestRCRelay,
            self.ThrottleFailsafe,
            self.NeedEKFToArm,
            self.ThrottleFailsafeFence,
            self.SoaringClimbRate,
            self.TestFlaps,
            self.DO_CHANGE_SPEED,
            self.DO_REPOSITION,
            self.GuidedRequest,
            self.MainFlight,
            self.TestGripperMission,
            self.Parachute,
            self.ParachuteSinkRate,
            self.DO_PARACHUTE,
            self.PitotBlockage,
            self.AIRSPEED_AUTOCAL,
            self.RangeFinder,
            self.FenceStatic,
            self.FenceRTL,
            self.FenceRTLRally,
            self.FenceRetRally,
            self.FenceAltCeilFloor,
            self.FenceMinAltAutoEnable,
            self.FenceMinAltEnableAutoland,
            self.FenceMinAltAutoEnableAbort,
            self.FenceAutoEnableDisableSwitch,
            Test(self.FenceCircleExclusionAutoEnable, speedup=20),
            self.FenceEnableDisableSwitch,
            self.FenceEnableDisableAux,
            self.FenceBreachedChangeMode,
            self.FenceNoFenceReturnPoint,
            self.FenceNoFenceReturnPointInclusion,
            self.FenceDisableUnderAction,
            self.ADSBFailActionRTL,
            self.ADSBResumeActionResumeLoiter,
            self.SimADSB,
            self.Button,
            self.FRSkySPort,
            self.FRSkyPassThroughStatustext,
            self.FRSkyPassThroughSensorIDs,
            self.FRSkyMAVlite,
            self.FRSkyD,
            self.LTM,
            self.DEVO,
            self.AdvancedFailsafe,
            self.LOITER,
            self.MAV_CMD_NAV_LOITER_TURNS,
            self.MAV_CMD_NAV_LOITER_TO_ALT,
            self.DeepStall,
            self.WatchdogHome,
            self.LargeMissions,
            self.Soaring,
            self.Terrain,
            self.TerrainMission,
            self.TerrainMissionInterrupt,
            self.UniversalAutoLandScript,
        ])
        return ret

    def tests1b(self):
        return [
            self.TerrainLoiter,
            self.VectorNavEAHRS,
            self.MicroStrainEAHRS5,
            self.MicroStrainEAHRS7,
            self.InertialLabsEAHRS,
            self.GpsSensorPreArmEAHRS,
            self.Deadreckoning,
            self.EKFlaneswitch,
            self.AirspeedDrivers,
            self.RTL_CLIMB_MIN,
            self.ClimbBeforeTurn,
            self.IMUTempCal,
            self.MAV_CMD_DO_AUX_FUNCTION,
            self.SmartBattery,
            self.FlyEachFrame,
            self.AutoLandMode,
            self.RCDisableAirspeedUse,
            self.AHRS_ORIENTATION,
            self.AHRSTrim,
            self.LandingDrift,
            self.TakeoffAuto1,
            self.TakeoffAuto2,
            self.TakeoffAuto3,
            self.TakeoffAuto4,
            self.TakeoffTakeoff1,
            self.TakeoffTakeoff2,
            self.TakeoffTakeoff3,
            self.TakeoffTakeoff4,
            self.TakeoffTakeoff5,
            self.TakeoffGround,
            self.TakeoffIdleThrottle,
            self.TakeoffBadLevelOff,
            self.TakeoffLevelOffWind,
            self.ForcedDCM,
            self.DCMFallback,
            self.MAVFTP,
            self.AUTOTUNE,
            self.AutotuneFiltering,
            self.MegaSquirt,
            self.Hirth,
            self.MSP_DJI,
            self.SpeedToFly,
            self.AltitudeSlopeMaxHeight,
            self.HIGH_LATENCY2,
            self.MidAirDisarmDisallowed,
            self.AerobaticsScripting,
            self.MANUAL_CONTROL,
            self.RunMissionScript,
            self.WindEstimates,
            self.AltResetBadGPS,
            self.AirspeedCal,
            self.MissionJumpTags,
            Test(self.GCSFailsafe, speedup=8),
            self.SDCardWPTest,
            self.NoArmWithoutMissionItems,
            self.RudderArmedTakeoffRequiresNeutralThrottle,
            self.MODE_SWITCH_RESET,
            self.ExternalPositionEstimate,
            self.SagetechMXS,
            self.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            self.MAV_CMD_PREFLIGHT_CALIBRATION,
            self.MAV_CMD_DO_INVERTED_FLIGHT,
            self.MAV_CMD_DO_AUTOTUNE_ENABLE,
            self.MAV_CMD_DO_GO_AROUND,
            self.MAV_CMD_DO_FLIGHTTERMINATION,
            self.MAV_CMD_DO_LAND_START,
            self.MAV_CMD_NAV_ALTITUDE_WAIT,
            self.InteractTest,
            self.CompassLearnInFlight,
            self.MAV_CMD_MISSION_START,
            self.TerrainRally,
            self.MAV_CMD_NAV_LOITER_UNLIM,
            self.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            self.MinThrottle,
            self.ClimbThrottleSaturation,
            self.GuidedAttitudeNoGPS,
            self.ScriptStats,
            self.GPSPreArms,
            self.SetHomeAltChange,
            self.SetHomeAltChange2,
            self.SetHomeAltChange3,
            self.ForceArm,
            self.MAV_CMD_EXTERNAL_WIND_ESTIMATE,
            self.GliderPullup,
            self.BadRollChannelDefined,
            self.VolzMission,
            self.mavlink_AIRSPEED,
            self.Volz,
            self.LoggedNamedValueFloat,
            self.LoggedNamedValueString,
            self.AdvancedFailsafeBadBaro,
            self.DO_CHANGE_ALTITUDE,
            self.SET_POSITION_TARGET_GLOBAL_INT_for_altitude,
            self.MAV_CMD_NAV_LOITER_TURNS_zero_turn,
            self.RudderArmingWithArmingChecksSkipped,
            self.TerrainLoiterToCircle,
            self.FenceDoubleBreach,
            self.ScriptedArmingChecksApplet,
            self.ScriptedArmingChecksAppletEStop,
            self.ScriptedArmingChecksAppletRally,
            self.PlaneFollowAppletSanity,
            self.PreflightRebootComponent,
        ]

    def tests1c(self):
        '''kind of reserved for flapping tests which we still have hopes for'''
        return [
            self.DeadreckoningNoAirSpeed,
        ]

    def disabled_tests(self):
        return {
            "LandingDrift": "Flapping test. See https://github.com/ArduPilot/ardupilot/issues/20054",
            "InteractTest": "requires user interaction",
            "ClimbThrottleSaturation": "requires https://github.com/ArduPilot/ardupilot/pull/27106 to pass",
            "SoaringClimbRate": "very bad sink rate",
        }


class AutoTestPlaneTests1a(AutoTestPlane):
    def tests(self):
        return self.tests1a()


class AutoTestPlaneTests1b(AutoTestPlane):
    def tests(self):
        return self.tests1b()


class AutoTestPlaneTests1c(AutoTestPlane):
    def tests(self):
        return self.tests1c()
