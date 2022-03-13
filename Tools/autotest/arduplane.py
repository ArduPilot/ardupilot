#!/usr/bin/env python

'''
Fly ArduPlane in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
import math
import os
import signal
import sys
import time

from pymavlink import quaternion
from pymavlink import mavextra
from pymavlink import mavutil

from common import AutoTest
from common import AutoTestTimeoutException
from common import NotAchievedException
from common import PreconditionFailedException
from pymavlink.rotmat import Vector3
from pysim import vehicleinfo

import operator

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 585, 354)
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestPlane(AutoTest):
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

    def takeoff(self, alt=150, alt_max=None, relative=True):
        """Takeoff to altitude."""

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
        self.wait_altitude(alt, alt_max, timeout=30, relative=relative)

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
        self.change_mode('RTL')
        self.wait_location(self.homeloc,
                           accuracy=120,
                           target_altitude=self.homeloc.alt+100,
                           height_accuracy=20,
                           timeout=180)
        self.progress("RTL Complete")

    def test_need_ekf_to_arm(self):
        """Loiter where we are."""
        self.progress("Ensuring we need EKF to be healthy to arm")
        self.reboot_sitl()
        self.context_collect("STATUSTEXT")
        tstart = self.get_sim_time()
        success = False
        while not success:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("Did not get correct failure reason")
            self.send_mavlink_arm_command()
            try:
                self.wait_statustext(".*(AHRS not healthy|AHRS: Not healthy).*", timeout=1, check_context=True, regex=True)
                success = True
                continue
            except AutoTestTimeoutException:
                pass
            if self.armed():
                raise NotAchievedException("Armed unexpectedly")

    def fly_LOITER(self, num_circles=4):
        """Loiter where we are."""
        self.progress("Testing LOITER for %u turns" % num_circles)
        self.change_mode('LOITER')

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        initial_alt = m.alt
        self.progress("Initial altitude %u\n" % initial_alt)

        while num_circles > 0:
            self.wait_heading(0, accuracy=10, timeout=60)
            self.wait_heading(180, accuracy=10, timeout=60)
            num_circles -= 1
            self.progress("Loiter %u circles left" % num_circles)

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
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

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        initial_alt = m.alt
        self.progress("Initial altitude %u\n" % initial_alt)

        while num_circles > 0:
            self.wait_heading(0, accuracy=10, timeout=60)
            self.wait_heading(180, accuracy=10, timeout=60)
            num_circles -= 1
            self.progress("CIRCLE %u circles left" % num_circles)

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
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
            m = self.mav.recv_match(type='ATTITUDE', blocking=True)
            roll = math.degrees(m.roll)
            pitch = math.degrees(m.pitch)
            self.progress("Roll=%.1f Pitch=%.1f" % (roll, pitch))
            if math.fabs(roll) <= accuracy and math.fabs(pitch) <= accuracy:
                self.progress("Attained level flight")
                return
        raise NotAchievedException("Failed to attain level flight")

    def change_altitude(self, altitude, accuracy=30):
        """Get to a given altitude."""
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
        self.change_altitude(self.homeloc.alt+300)

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
        self.change_altitude(self.homeloc.alt+300)
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
#        self.set_parameter("STALL_PREVENTION", 0)

        state_roll_over = "roll-over"
        state_stabilize_roll = "stabilize-roll"
        state_hold = "hold"
        state_roll_back = "roll-back"
        state_done = "done"

        tstart = self.get_sim_time()

        try:
            state = state_roll_over
            while state != state_done:

                m = self.mav.recv_match(type='ATTITUDE',
                                        blocking=True,
                                        timeout=0.1)
                now = self.get_sim_time_cached()
                if now - tstart > 20:
                    raise AutoTestTimeoutException("Manuevers not completed")
                if m is None:
                    continue

                r = math.degrees(m.roll)
                if state == state_roll_over:
                    target_roll_degrees = 60
                    if abs(r - target_roll_degrees) < tolerance:
                        state = state_stabilize_roll
                        stabilize_start = now
                elif state == state_stabilize_roll:
                    # just give it a little time to sort it self out
                    if now - stabilize_start > 2:
                        state = state_hold
                        hold_start = now
                elif state == state_hold:
                    target_roll_degrees = 60
                    if now - hold_start > tolerance:
                        state = state_roll_back
                    if abs(r - target_roll_degrees) > tolerance:
                        raise NotAchievedException("Failed to hold attitude")
                elif state == state_roll_back:
                    target_roll_degrees = 0
                    if abs(r - target_roll_degrees) < tolerance:
                        state = state_done
                else:
                    raise ValueError("Unknown state %s" % str(state))

                m_nav = self.mav.messages['NAV_CONTROLLER_OUTPUT']
                self.progress("%s Roll: %f desired=%f set=%f" %
                              (state, r, m_nav.nav_roll, target_roll_degrees))

                time_boot_millis = 0 # FIXME
                target_system = 1 # FIXME
                target_component = 1 # FIXME
                type_mask = 0b10000001 ^ 0xFF # FIXME
                # attitude in radians:
                q = quaternion.Quaternion([math.radians(target_roll_degrees),
                                           0,
                                           0])
                roll_rate_radians = 0.5
                pitch_rate_radians = 0
                yaw_rate_radians = 0
                thrust = 1.0
                self.mav.mav.set_attitude_target_send(time_boot_millis,
                                                      target_system,
                                                      target_component,
                                                      type_mask,
                                                      q,
                                                      roll_rate_radians,
                                                      pitch_rate_radians,
                                                      yaw_rate_radians,
                                                      thrust)
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
        self.change_altitude(self.homeloc.alt+300)
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
        self.change_altitude(self.homeloc.alt+300)
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

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
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

        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
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
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode('AUTO')
        self.wait_waypoint(1, num_wp, max_dist=60, timeout=mission_timeout)
        self.wait_groundspeed(0, 0.5, timeout=mission_timeout)
        if quadplane:
            self.wait_statustext("Throttle disarmed", timeout=200)
        else:
            self.wait_statustext("Auto disarmed", timeout=60)
        self.progress("Mission OK")

    def fly_do_reposition(self):
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
            0,
            0,
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            new_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_altitude(new_alt-10, new_alt, timeout=30, relative=True)

        self.fly_home_land_and_disarm()

    def fly_deepstall(self):
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

        self.wait_text("Deepstall: Entry: ", timeout=240)

        # assume elevator is on channel 2:
        self.wait_servo_channel_value(2, deepstall_elevator_pwm)

        self.disarm_wait(timeout=120)

        self.progress("Flying home")
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
        self.progress("Waiting for deepstall messages")

        self.wait_text("Deepstall: Entry: ", timeout=240)

        # assume elevator is on channel 2:
        self.wait_servo_channel_value(2, deepstall_elevator_pwm)

        self.disarm_wait(timeout=120)

        self.progress("Flying home")
        self.takeoff(100)
        self.set_parameter("LAND_TYPE", 0)
        self.fly_home_land_and_disarm(timeout=240)

    def SmartBattery(self):
        self.set_parameters({
            "BATT_MONITOR": 16, # Maxell battery monitor
        })

        # Must reboot sitl after setting montior type for SMBus parameters to be set due to dynamic group
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

    def fly_do_change_speed(self):
        # the following lines ensure we revert these parameter values
        # - DO_CHANGE_AIRSPEED is a permanent vehicle change!
        self.set_parameters({
            "TRIM_ARSPD_CM": self.get_parameter("TRIM_ARSPD_CM"),
            "MIN_GNDSPD_CM": self.get_parameter("MIN_GNDSPD_CM"),
        })

        self.progress("Takeoff")
        self.takeoff(alt=100)
        self.set_rc(3, 1500)
        # ensure we know what the airspeed is:
        self.progress("Entering guided and flying somewhere constant")
        self.change_mode("GUIDED")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            0,
            0,
            0,
            12345, # lat* 1e7
            12345, # lon* 1e7
            100    # alt
        )
        self.delay_sim_time(10)
        self.progress("Ensuring initial speed is known and relatively constant")
        initial_speed = 21.5
        timeout = 10
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("GroundSpeed: %f want=%f" %
                          (m.groundspeed, initial_speed))
            if abs(initial_speed - m.groundspeed) > 1:
                raise NotAchievedException("Initial speed not as expected (want=%f got=%f" % (initial_speed, m.groundspeed))

        self.progress("Setting groundspeed")
        new_target_groundspeed = initial_speed + 5
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            1, # groundspeed
            new_target_groundspeed,
            -1, # throttle / no change
            0, # absolute values
            0,
            0,
            0)
        self.wait_groundspeed(new_target_groundspeed-0.5, new_target_groundspeed+0.5, timeout=40)
        self.progress("Adding some wind, ensuring groundspeed holds")
        self.set_parameter("SIM_WIND_SPD", 5)
        self.delay_sim_time(5)
        self.wait_groundspeed(new_target_groundspeed-0.5, new_target_groundspeed+0.5, timeout=40)
        self.set_parameter("SIM_WIND_SPD", 0)

        self.progress("Setting airspeed")
        new_target_airspeed = initial_speed + 5
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0, # airspeed
            new_target_airspeed,
            -1, # throttle / no change
            0, # absolute values
            0,
            0,
            0)
        self.wait_groundspeed(new_target_airspeed-0.5, new_target_airspeed+0.5)
        self.progress("Adding some wind, hoping groundspeed increases/decreases")
        self.set_parameters({
            "SIM_WIND_SPD": 5,
            "SIM_WIND_DIR": 270,
        })
        self.delay_sim_time(5)
        timeout = 10
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not achieve groundspeed delta")
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            delta = abs(m.airspeed - m.groundspeed)
            want_delta = 4
            self.progress("groundspeed and airspeed should be different (have=%f want=%f)" % (delta, want_delta))
            if delta > want_delta:
                break
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
        self.drain_mav()
        # TODO: reflect on file to find this magic waypoint number?
        #        self.wait_waypoint(7, num_wp-1, timeout=500) # we
        #        tend to miss the final waypoint by a fair bit, and
        #        this is probably too noisy anyway?
        self.wait_disarmed(timeout=timeout)

    def fly_flaps(self):
        """Test flaps functionality."""
        filename = "flaps.txt"
        self.context_push()
        ex = None
        try:

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
            self.set_current_waypoint(1)
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            last_mission_current_msg = 0
            last_seq = None
            while self.armed():
                m = self.mav.recv_match(type='MISSION_CURRENT', blocking=True)
                time_delta = (self.get_sim_time_cached() -
                              last_mission_current_msg)
                if (time_delta > 1 or m.seq != last_seq):
                    dist = None
                    x = self.mav.messages.get("NAV_CONTROLLER_OUTPUT", None)
                    if x is not None:
                        dist = x.wp_dist
                    self.progress("MISSION_CURRENT.seq=%u (dist=%s)" %
                                  (m.seq, str(dist)))
                    last_mission_current_msg = self.get_sim_time_cached()
                    last_seq = m.seq
            # flaps should undeploy at the end
            self.wait_servo_channel_value(servo_ch, servo_ch_min, timeout=30)

            # do a short flight in FBWA, watching for flaps
            # self.mavproxy.send('switch 4\n')
            # self.wait_mode('FBWA')
            # self.delay_sim_time(10)
            # self.mavproxy.send('switch 6\n')
            # self.wait_mode('MANUAL')
            # self.delay_sim_time(10)

            self.progress("Flaps OK")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        if ex:
            if self.armed():
                self.disarm_vehicle()
            raise ex

    def test_rc_relay(self):
        '''test toggling channel 12 toggles relay'''
        self.set_parameter("RC12_OPTION", 28) # Relay On/Off
        self.set_rc(12, 1000)
        self.reboot_sitl() # needed for RC12_OPTION to take effect

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

    def test_rc_option_camera_trigger(self):
        '''test toggling channel 12 takes picture'''
        self.set_parameter("RC12_OPTION", 9) # CameraTrigger
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
        self.fly_home_land_and_disarm()

    def ThrottleFailsafe(self):
        self.change_mode('MANUAL')
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
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
        if (not self.get_mode_from_mode_mapping("CIRCLE") in
                [x.custom_mode for x in self.context_stop_collecting("HEARTBEAT")]):
            raise NotAchievedException("Did not go via circle mode")
        self.progress("Ensure we've had our throttle squashed to 950")
        self.wait_rc_channel_value(3, 950)
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
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
        self.drain_mav_unparsed()
        # have to allow time for RC to be fetched from SITL
        self.delay_sim_time(0.5)
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
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
        if (not self.get_mode_from_mode_mapping("CIRCLE") in
                [x.custom_mode for x in self.context_stop_collecting("HEARTBEAT")]):
            raise NotAchievedException("Did not go via circle mode")
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
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
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
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
        ex = None
        try:
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
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

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

    def test_throttle_failsafe_fence(self):
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.progress("Checking fence is not present before being configured")
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
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

        m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=2)
        self.progress("Got (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Got FENCE_STATUS unexpectedly")

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
        self.drain_mav_unparsed()

        self.progress("Checking fence is OK after receiver failure (bind-values)")
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        self.progress("Got (%s)" % str(m))
        if (not (m.onboard_control_sensors_enabled & fence_bit)):
            raise NotAchievedException("Fence not enabled after RC fail")
        self.do_fence_disable() # Ensure the fence is disabled after test

    def test_gripper_mission(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("RTL_AUTOLAND", 1)
            self.load_mission("plane-gripper-mission.txt")
            self.set_current_waypoint(1)
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.wait_statustext("Gripper Grabbed", timeout=60)
            self.wait_statustext("Gripper Released", timeout=60)
            self.wait_statustext("Auto disarmed", timeout=60)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def assert_fence_sys_status(self, present, enabled, health):
        self.delay_sim_time(1)
        self.drain_mav_unparsed()
        m = self.assert_receive_message('SYS_STATUS', timeout=1)
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

    def wait_circling_point_with_radius(self, loc, want_radius, epsilon=5.0, min_circle_time=5, timeout=120):
        on_radius_start_heading = None
        average_radius = 0.0
        circle_time_start = 0
        done_time = False
        done_angle = False
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get onto circle")
            here = self.mav.location()
            got_radius = self.get_distance(loc, here)
            average_radius = 0.95*average_radius + 0.05*got_radius
            on_radius = abs(got_radius - want_radius) < epsilon
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            heading = m.heading
            on_string = "off"
            got_angle = ""
            if on_radius_start_heading is not None:
                got_angle = "%0.2f" % abs(on_radius_start_heading - heading) # FIXME
                on_string = "on"

            want_angle = 180 # we don't actually get this (angle-substraction issue.  But we get enough...
            self.progress("wait-circling: got-r=%0.2f want-r=%f avg-r=%f %s want-a=%0.1f got-a=%s" %
                          (got_radius, want_radius, average_radius, on_string, want_angle, got_angle))
            if on_radius:
                if on_radius_start_heading is None:
                    on_radius_start_heading = heading
                    average_radius = got_radius
                    circle_time_start = self.get_sim_time()
                    continue
                if abs(on_radius_start_heading - heading) > want_angle: # FIXME
                    done_angle = True
                if self.get_sim_time() - circle_time_start > min_circle_time:
                    done_time = True
                if done_time and done_angle:
                    return
                continue
            if on_radius_start_heading is not None:
                average_radius = 0.0
            on_radius_start_heading = None
            circle_time_start = 0

    def test_fence_static(self):
        ex = None
        try:
            self.progress("Checking for bizarre healthy-when-not-present-or-enabled")
            self.set_parameter("FENCE_TYPE", 4) # Start by only setting polygon fences, otherwise fence will report present
            self.assert_fence_sys_status(False, False, True)
            self.load_fence("CMAC-fence.txt")
            m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=2)
            if m is not None:
                raise NotAchievedException("Got FENCE_STATUS unexpectedly")
            self.drain_mav_unparsed()
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

            self.try_arm(False, "vehicle outside fence")
            self.do_fence_disable()
            self.set_parameter("FENCE_ALT_MIN", default_fence_alt_min)

            # Test arming outside inclusion zone
            self.progress("Test arming while vehicle outside of inclusion zone")
            self.set_parameter("FENCE_TYPE", 4) # Enables polygon fence types
            locs = [
                mavutil.location(1.000, 1.000, 0, 0),
                mavutil.location(1.000, 1.001, 0, 0),
                mavutil.location(1.001, 1.001, 0, 0),
                mavutil.location(1.001, 1.000, 0, 0)
            ]
            self.upload_fences_from_locations(
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                [
                    locs
                ]
            )
            self.delay_sim_time(10) # let fence check run so it loads-from-eeprom
            self.do_fence_enable()
            self.assert_fence_enabled()
            self.delay_sim_time(2) # Allow breach to propagate
            self.try_arm(False, "vehicle outside fence")
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
            self.upload_fences_from_locations(
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                [
                    locs
                ]
            )
            self.delay_sim_time(10) # let fence check run so it loads-from-eeprom
            self.do_fence_enable()
            self.assert_fence_enabled()
            self.delay_sim_time(2) # Allow breach to propagate
            self.try_arm(False, "vehicle outside fence")
            self.do_fence_disable()
            self.clear_fence()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.clear_fence()
        if ex is not None:
            raise ex

    def test_fence_breach_circle_at(self, loc, disable_on_breach=False):
        ex = None
        try:
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

            if disable_on_breach:
                self.do_fence_disable()

            self.wait_circling_point_with_radius(loc, expected_radius)

            self.disarm_vehicle(force=True)
            self.reboot_sitl()

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.clear_fence()
        if ex is not None:
            raise ex

    def test_fence_rtl(self):
        self.progress("Testing FENCE_ACTION_RTL no rally point")
        # have to disable the fence once we've breached or we breach
        # it as part of the loiter-at-home!
        self.test_fence_breach_circle_at(self.home_position_as_mav_location(),
                                         disable_on_breach=True)

    def test_fence_rtl_rally(self):
        ex = None
        target_system = 1
        target_component = 1
        try:
            self.progress("Testing FENCE_ACTION_RTL with rally point")

            self.wait_ready_to_arm()
            loc = self.home_relative_loc_ne(50, -50)

            self.set_parameter("RALLY_TOTAL", 1)
            self.mav.mav.rally_point_send(target_system,
                                          target_component,
                                          0, # sequence number
                                          1, # total count
                                          int(loc.lat * 1e7),
                                          int(loc.lng * 1e7),
                                          15,
                                          0, # "break" alt?!
                                          0, # "land dir"
                                          0) # flags
            self.delay_sim_time(1)
            if self.mavproxy is not None:
                self.mavproxy.send("rally list\n")
            self.test_fence_breach_circle_at(loc)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if ex is not None:
            raise ex

    def test_fence_ret_rally(self):
        """ Tests the FENCE_RET_RALLY flag, either returning to fence return point,
            or rally point """
        target_system = 1
        target_component = 1
        self.progress("Testing FENCE_ACTION_RTL with fence rally point")

        self.wait_ready_to_arm()
        self.homeloc = self.mav.location()

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
        self.set_parameter("RALLY_TOTAL", 1)
        self.mav.mav.rally_point_send(target_system,
                                      target_component,
                                      0, # sequence number
                                      1, # total count
                                      int(rally_loc.lat * 1e7),
                                      int(rally_loc.lng * 1e7),
                                      15,
                                      0, # "break" alt?!
                                      0, # "land dir"
                                      0) # flags
        self.delay_sim_time(1)

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

        # Fly up before re-triggering fence breach. Fly to fence return point
        self.change_altitude(self.homeloc.alt+30)
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

    def test_parachute(self):
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

    def test_parachute_sinkrate(self):
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

    def check_attitudes_match(self, a, b):
        '''make sure ahrs2 and simstate and ATTTIUDE_QUATERNION all match'''

        # these are ordered to bookend the list with timestamps (which
        # both attitude messages have):
        get_names = ['ATTITUDE', 'SIMSTATE', 'AHRS2', 'ATTITUDE_QUATERNION']
        msgs = self.get_messages_frame(get_names)

        for get_name in get_names:
            self.progress("%s: %s" % (get_name, msgs[get_name]))

        simstate = msgs['SIMSTATE']
        attitude = msgs['ATTITUDE']
        ahrs2 = msgs['AHRS2']
        attitude_quaternion = msgs['ATTITUDE_QUATERNION']

        # check ATTITUDE
        want = math.degrees(simstate.roll)
        got = math.degrees(attitude.roll)
        if abs(mavextra.angle_diff(want, got)) > 20:
            raise NotAchievedException("ATTITUDE.Roll looks bad (want=%f got=%f)" %
                                       (want, got))
        want = math.degrees(simstate.pitch)
        got = math.degrees(attitude.pitch)
        if abs(mavextra.angle_diff(want, got)) > 20:
            raise NotAchievedException("ATTITUDE.Pitch looks bad (want=%f got=%f)" %
                                       (want, got))

        # check AHRS2
        want = math.degrees(simstate.roll)
        got = math.degrees(ahrs2.roll)
        if abs(mavextra.angle_diff(want, got)) > 20:
            raise NotAchievedException("AHRS2.Roll looks bad (want=%f got=%f)" %
                                       (want, got))

        want = math.degrees(simstate.pitch)
        got = math.degrees(ahrs2.pitch)
        if abs(mavextra.angle_diff(want, got)) > 20:
            raise NotAchievedException("AHRS2.Pitch looks bad (want=%f got=%f)" %
                                       (want, got))

        # check ATTITUDE_QUATERNION
        q = quaternion.Quaternion([
            attitude_quaternion.q1,
            attitude_quaternion.q2,
            attitude_quaternion.q3,
            attitude_quaternion.q4
        ])
        euler = q.euler
        self.progress("attquat:%s q:%s euler:%s" % (
            str(attitude_quaternion), q, euler))

        want = math.degrees(simstate.roll)
        got = math.degrees(euler[0])
        if mavextra.angle_diff(want, got) > 20:
            raise NotAchievedException("quat roll differs from attitude roll; want=%f got=%f" %
                                       (want, got))

        want = math.degrees(simstate.pitch)
        got = math.degrees(euler[1])
        if mavextra.angle_diff(want, got) > 20:
            raise NotAchievedException("quat pitch differs from attitude pitch; want=%f got=%f" %
                                       (want, got))

    def fly_ahrs2_test(self):
        '''check secondary estimator is looking OK'''

        ahrs2 = self.mav.recv_match(type='AHRS2', blocking=True, timeout=1)
        if ahrs2 is None:
            raise NotAchievedException("Did not receive AHRS2 message")
        self.progress("AHRS2: %s" % str(ahrs2))

        # check location
        gpi = self.mav.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True,
            timeout=5
        )
        if gpi is None:
            raise NotAchievedException("Did not receive GLOBAL_POSITION_INT message")
        self.progress("GPI: %s" % str(gpi))
        if self.get_distance_int(gpi, ahrs2) > 10:
            raise NotAchievedException("Secondary location looks bad")

        self.check_attitudes_match(1, 2)

    def test_main_flight(self):

        self.change_mode('MANUAL')

        self.progress("Asserting we do support transfer of fence via mission item protocol")
        self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)

        # grab home position:
        self.mav.recv_match(type='HOME_POSITION', blocking=True)
        self.homeloc = self.mav.location()

        self.run_subtest("Takeoff", self.takeoff)

        self.run_subtest("Set Attitude Target", self.set_attitude_target)

        self.run_subtest("Fly left circuit", self.fly_left_circuit)

        self.run_subtest("Left roll", lambda: self.axial_left_roll(1))

        self.run_subtest("Inside loop", self.inside_loop)

        self.run_subtest("Stablize test", self.test_stabilize)

        self.run_subtest("ACRO test", self.test_acro)

        self.run_subtest("FBWB test", self.test_FBWB)

        self.run_subtest("CRUISE test", lambda: self.test_FBWB(mode='CRUISE'))

        self.run_subtest("RTL test", self.fly_RTL)

        self.run_subtest("LOITER test", self.fly_LOITER)

        self.run_subtest("CIRCLE test", self.fly_CIRCLE)

        self.run_subtest("AHRS2 test", self.fly_ahrs2_test)

        self.run_subtest("Mission test",
                         lambda: self.fly_mission("ap1.txt", strict=False))

    def airspeed_autocal(self):
        self.progress("Ensure no AIRSPEED_AUTOCAL on ground")
        self.set_parameters({
            "ARSPD_AUTOCAL": 1,
            "RTL_AUTOLAND": 1,
        })
        m = self.mav.recv_match(type='AIRSPEED_AUTOCAL',
                                blocking=True,
                                timeout=5)
        if m is not None:
            raise NotAchievedException("Got autocal on ground")
        mission_filepath = "flaps.txt"
        num_wp = self.load_mission(mission_filepath)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("AUTO")
        self.progress("Ensure AIRSPEED_AUTOCAL in air")
        m = self.mav.recv_match(type='AIRSPEED_AUTOCAL',
                                blocking=True,
                                timeout=5)
        self.wait_waypoint(7, num_wp-1, max_dist=5, timeout=500)
        self.wait_disarmed(timeout=120)

    def deadreckoning_main(self, disable_airspeed_sensor=False):
        self.wait_ready_to_arm()
        self.gpi = None
        self.simstate = None
        self.last_print = 0
        self.max_divergence = 0

        def validate_global_position_int_against_simstate(mav, m):
            if m.get_type() == 'GLOBAL_POSITION_INT':
                self.gpi = m
            elif m.get_type() == 'SIMSTATE':
                self.simstate = m
            if self.gpi is None:
                return
            if self.simstate is None:
                return
            divergence = self.get_distance_int(self.gpi, self.simstate)
            max_allowed_divergence = 200
            if (time.time() - self.last_print > 1 or
                    divergence > self.max_divergence):
                self.progress("position-estimate-divergence=%fm" % (divergence,))
                self.last_print = time.time()
            if divergence > self.max_divergence:
                self.max_divergence = divergence
            if divergence > max_allowed_divergence:
                raise NotAchievedException(
                    "global-position-int diverged from simstate by %fm (max=%fm" %
                    (divergence, max_allowed_divergence,))

        self.install_message_hook(validate_global_position_int_against_simstate)

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
                0,
                mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                0,
                0,
                int(loc.lat * 1e7),
                int(loc.lng * 1e7),
                100,    # alt
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            )
            self.wait_location(loc, accuracy=100)
            self.progress("Stewing")
            self.delay_sim_time(20)
            self.set_parameter("SIM_GPS_DISABLE", 1)
            self.progress("Roasting")
            self.delay_sim_time(20)
            self.change_mode("RTL")
            self.wait_distance_to_home(100, 200, timeout=200)
            self.set_parameter("SIM_GPS_DISABLE", 0)
            self.delay_sim_time(10)
            self.set_rc(3, 1000)
            self.fly_home_land_and_disarm()
            self.progress("max-divergence: %fm" % (self.max_divergence,))
        finally:
            self.remove_message_hook(validate_global_position_int_against_simstate)

    def deadreckoning(self):
        self.deadreckoning_main()

    def deadreckoning_no_airspeed_sensor(self):
        self.deadreckoning_main(disable_airspeed_sensor=True)

    def climb_before_turn(self):
        self.wait_ready_to_arm()
        self.set_parameters({
            "FLIGHT_OPTIONS": 0,
            "ALT_HOLD_RTL": 8000,
            "RTL_AUTOLAND": 1,
        })
        takeoff_alt = 10
        self.takeoff(alt=takeoff_alt)
        self.change_mode("CRUISE")
        self.wait_distance_to_home(500, 1000, timeout=60)
        self.change_mode("RTL")
        expected_alt = self.get_parameter("ALT_HOLD_RTL") / 100.0

        home = self.home_position_as_mav_location()
        distance = self.get_distance(home, self.mav.location())

        self.wait_altitude(expected_alt - 10, expected_alt + 10, relative=True)

        new_distance = self.get_distance(home, self.mav.location())
        # We should be closer to home.
        if new_distance > distance:
            raise NotAchievedException(
                "Expected to be closer to  home (was %fm, now %fm)."
                % (distance, new_distance)
            )

        self.fly_home_land_and_disarm()
        self.change_mode("MANUAL")
        self.set_rc(3, 1000)

        self.wait_ready_to_arm()
        self.set_parameters({
            "FLIGHT_OPTIONS": 16,
            "ALT_HOLD_RTL": 10000,
        })
        self.takeoff(alt=takeoff_alt)
        self.change_mode("CRUISE")
        self.wait_distance_to_home(500, 1000, timeout=60)
        self.change_mode("RTL")

        home = self.home_position_as_mav_location()
        distance = self.get_distance(home, self.mav.location())

        self.wait_altitude(expected_alt - 10, expected_alt + 10, relative=True)

        new_distance = self.get_distance(home, self.mav.location())
        # We should be farther from to home.
        if new_distance < distance:
            raise NotAchievedException(
                "Expected to be farther from home (was %fm, now %fm)."
                % (distance, new_distance)
            )

        self.fly_home_land_and_disarm(timeout=240)

    def rtl_climb_min(self):
        self.wait_ready_to_arm()
        rtl_climb_min = 100
        self.set_parameter("RTL_CLIMB_MIN", rtl_climb_min)
        takeoff_alt = 50
        self.takeoff(alt=takeoff_alt)
        self.change_mode('CRUISE')
        self.wait_distance_to_home(1000, 1500, timeout=60)
        post_cruise_alt = self.get_altitude(relative=True)
        self.change_mode('RTL')
        expected_alt = self.get_parameter("ALT_HOLD_RTL")/100.0
        if expected_alt == -1:
            expected_alt = self.get_altitude(relative=True)

        # ensure we're about half-way-down at the half-way-home stage:
        self.wait_distance_to_nav_target(
            0,
            500,
            timeout=120,
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

    def test_rangefinder(self):
        ex = None
        self.context_push()
        self.progress("Making sure we don't ordinarily get RANGEFINDER")
        m = None
        try:
            m = self.mav.recv_match(type='RANGEFINDER',
                                    blocking=True,
                                    timeout=5)
        except Exception as e:
            self.print_exception_caught(e)

        if m is not None:
            raise NotAchievedException("Received unexpected RANGEFINDER msg")

        try:
            self.set_analog_rangefinder_parameters()

            self.reboot_sitl()

            '''ensure rangefinder gives height-above-ground'''
            self.load_mission("plane-gripper-mission.txt") # borrow this
            self.set_parameter("RTL_AUTOLAND", 1)
            self.set_current_waypoint(1)
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.wait_waypoint(5, 5, max_dist=100)
            rf = self.mav.recv_match(type="RANGEFINDER", timeout=1, blocking=True)
            if rf is None:
                raise NotAchievedException("Did not receive rangefinder message")
            gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if gpi is None:
                raise NotAchievedException("Did not receive GLOBAL_POSITION_INT message")
            if abs(rf.distance - gpi.relative_alt/1000.0) > 3:
                raise NotAchievedException(
                    "rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                    (rf.distance, gpi.relative_alt/1000.0))
            self.wait_statustext("Auto disarmed", timeout=60)

            self.progress("Ensure RFND messages in log")
            if not self.current_onboard_log_contains_message("RFND"):
                raise NotAchievedException("No RFND messages in log")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def rc_defaults(self):
        ret = super(AutoTestPlane, self).rc_defaults()
        ret[3] = 1000
        ret[8] = 1800
        return ret

    def initial_mode_switch_mode(self):
        return "MANUAL"

    def default_mode(self):
        return "MANUAL"

    def test_pid_tuning(self):
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestPlane, self).test_pid_tuning()

    def test_setting_modes_via_auxswitches(self):
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
        '''trivial tests to ensure simulated ADSB sensor continues to
function'''
        self.set_parameters({
            "SIM_ADSB_COUNT": 1,
            "ADSB_TYPE": 1,
        })
        self.reboot_sitl()
        self.assert_receive_message('ADSB_VEHICLE', timeout=30)

    def test_adsb(self):
        self.context_push()
        ex = None
        try:
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
            m = self.mav.recv_match(type='COLLISION', blocking=True, timeout=4)
            self.progress("Got (%s)" % str(m))
            if m is not None:
                raise NotAchievedException("Got collision message when I shouldn't have")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def fly_do_guided_request(self, target_system=1, target_component=1):
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
        self.takeoff(alt=200)
        self.set_rc(3, 1500)
        self.change_mode("LOITER")
        self.progress("Doing a bit of loitering to start with")
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=5)
            if m is None:
                raise NotAchievedException("Did not get VFR_HUD")
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
        self.fly_home_land_and_disarm()

    def CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        self.plane_CPUFailsafe()

    def test_large_missions(self):
        self.load_mission("Kingaroy-vlarge.txt", strict=False)
        self.load_mission("Kingaroy-vlarge2.txt", strict=False)

    def fly_soaring(self):

        model = "plane-soaring"

        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath=self.model_defaults_filepath(model),
            wipe=True)

        self.load_mission('CMAC-soar.txt', strict=False)

        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

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
            3: 1500, # Use trim airspeed.
        })

        # Wait to detect thermal
        self.progress("Waiting for thermal")
        self.wait_mode('THERMAL', timeout=600)

        self.set_parameter("SOAR_VSPEED", 0.6)

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
        self.wait_servo_channel_value(3, 1200, timeout=2, comparator=operator.gt)

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

        # And reenable. This should force throttle-down
        self.set_parameter("SOAR_ENABLE", 1)
        self.delay_sim_time(10)

        # Now wait for descent and check throttle up
        self.wait_altitude(alt_min-10, alt_min, timeout=600, relative=True)

        self.progress("Waiting for climb")
        self.wait_altitude(alt_ctf-10, alt_ctf, timeout=600, relative=True)

        # Back to auto
        self.change_mode('AUTO')

        # Reenable thermals
        self.set_parameter("SIM_THML_SCENARI", 1)

        # Disable soaring using RC channel.
        self.set_rc(rc_chan, 1100)

        # Wait to get back to waypoint before thermal.
        self.progress("Waiting to get back to position")
        self.wait_current_waypoint(3, timeout=1200)

        # Enable soaring with mode changes suppressed)
        self.set_rc(rc_chan, 1500)

        # Make sure this causes throttle down.
        self.wait_servo_channel_value(3, 1200, timeout=2, comparator=operator.lt)

        self.progress("Waiting for next WP with no thermalling")
        self.wait_waypoint(4, 4, timeout=1200, max_dist=120)

        # Disarm
        self.disarm_vehicle_expect_fail()

        self.progress("Mission OK")

    def fly_soaring_speed_to_fly(self):

        model = "plane-soaring"

        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath=self.model_defaults_filepath(model),
            wipe=True)

        self.load_mission('CMAC-soar.txt', strict=False)

        # Turn of environmental thermals.
        self.set_parameter("SIM_THML_SCENARI", 0)

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

        # Enable speed to fly.
        self.set_parameter("SOAR_CRSE_ARSPD", -1)

        # Set appropriate McCready.
        self.set_parameter("SOAR_VSPEED", 1)
        self.set_parameter("SIM_WIND_SPD", 0)

        # Wait a few seconds before determining the "trim" airspeed.
        self.delay_sim_time(20)
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)
        trim_airspeed = m.airspeed

        min_airspeed = self.get_parameter("ARSPD_FBW_MIN")
        max_airspeed = self.get_parameter("ARSPD_FBW_MAX")

        # Add updraft
        self.set_parameter("SIM_WIND_SPD", 1)
        self.set_parameter('SIM_WIND_DIR_Z', 90)
        self.delay_sim_time(20)
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)

        if not m.airspeed < trim_airspeed and trim_airspeed > min_airspeed:
            raise NotAchievedException("Airspeed did not reduce in updraft")

        # Add downdraft
        self.set_parameter('SIM_WIND_DIR_Z', -90)
        self.delay_sim_time(20)
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)

        if not m.airspeed > trim_airspeed and trim_airspeed < max_airspeed:
            raise NotAchievedException("Airspeed did not increase in downdraft")

        # Zero the wind and increase McCready.
        self.set_parameter("SIM_WIND_SPD", 0)
        self.set_parameter("SOAR_VSPEED", 2)
        self.delay_sim_time(20)
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)

        if not m.airspeed > trim_airspeed and trim_airspeed < max_airspeed:
            raise NotAchievedException("Airspeed did not increase with higher SOAR_VSPEED")

        # Reduce McCready.
        self.set_parameter("SOAR_VSPEED", 0)
        self.delay_sim_time(20)
        m = self.mav.recv_match(type='VFR_HUD', blocking=True)

        if not m.airspeed < trim_airspeed and trim_airspeed > min_airspeed:
            raise NotAchievedException("Airspeed did not reduce with lower SOAR_VSPEED")

        # Disarm
        self.disarm_vehicle_expect_fail()

        self.progress("Mission OK")

    def test_airspeed_drivers(self):
        airspeed_sensors = [
            ("MS5525", 3, 1),
            ("DLVR", 7, 2),
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

            def check_airspeeds(mav, m):
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

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.fly_mission("ap-terrain.txt", mission_timeout=600)

    def Terrain(self):
        '''test AP_Terrain'''
        self.reboot_sitl()  # we know the terrain height at CMAC

        mavproxy = self.start_mavproxy()

        self.wait_ready_to_arm()
        loc = self.mav.location()

        lng_int = int(loc.lng * 1e7)
        lat_int = int(loc.lat * 1e7)

        # FIXME: once we have a pre-populated terrain cache this
        # should require an instantly correct report to pass
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("Did not get correct terrain report")

            self.mav.mav.terrain_check_send(lat_int, lng_int)

            report = self.mav.recv_match(type='TERRAIN_REPORT', blocking=True, timeout=60)
            self.progress(self.dump_message_verbose(report))
            if report.spacing != 0:
                break

            self.delay_sim_time(1)

        self.progress(self.dump_message_verbose(report))

        expected_terrain_height = 583.5
        if abs(report.terrain_height - expected_terrain_height) > 0.5:
            raise NotAchievedException("Expected terrain height=%f got=%f" %
                                       (expected_terrain_height, report.terrain_height))

        self.stop_mavproxy(mavproxy)

    def test_loiter_terrain(self):
        default_rad = self.get_parameter("WP_LOITER_RAD")
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
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60*15: # enough time to do one and a bit circles
                break
            terrain = self.mav.recv_match(
                type='TERRAIN_REPORT',
                blocking=True,
                timeout=1
            )
            if terrain is None:
                raise NotAchievedException("Did not get TERRAIN_REPORT message")
            rel_alt = terrain.current_height
            self.progress("%um above terrain" % rel_alt)
            if rel_alt > alt*1.2 or rel_alt < alt * 0.8:
                raise NotAchievedException("Not terrain following")
        self.progress("Returning home")
        self.set_parameters({
            "TERRAIN_FOLLOW": 0,
            "WP_LOITER_RAD": default_rad,
        })
        self.fly_home_land_and_disarm(240)

    def fly_external_AHRS(self, sim, eahrs_type, mission):
        """Fly with external AHRS (VectorNav)"""
        self.customise_SITL_commandline(["--uartE=sim:%s" % sim])

        self.set_parameters({
            "EAHRS_TYPE": eahrs_type,
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS_TYPE": 21,
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
        })
        self.reboot_sitl()
        self.progress("Running accelcal")
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                     0, 0, 0, 0, 4, 0, 0,
                     timeout=5)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.fly_mission(mission)

    def test_vectornav(self):
        self.fly_external_AHRS("VectorNav", 1, "ap1.txt")

    def test_lord(self):
        self.fly_external_AHRS("LORD", 2, "ap1.txt")

    def get_accelvec(self, m):
        return Vector3(m.xacc, m.yacc, m.zacc) * 0.001 * 9.81

    def get_gyrovec(self, m):
        return Vector3(m.xgyro, m.ygyro, m.zgyro) * 0.001 * math.degrees(1)

    def test_imu_tempcal(self):
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
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                     0, 0, 0, 0, 4, 0, 0,
                     timeout=5)
        self.progress("Running gyro cal")
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                     0, 0, 0, 0, 1, 0, 0,
                     timeout=5)
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

    def ekf_lane_switch(self):

        self.context_push()
        ex = None

        # new lane swtich available only with EK3
        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "EK3_AFFINITY": 15, # enable affinity for all sensors
            "EK3_IMU_MASK": 3, # use only 2 IMUs
            "GPS_TYPE2": 1,
            "SIM_GPS2_DISABLE": 0,
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
        self.install_message_hook(statustext_hook)

        # get flying
        self.takeoff(alt=50)
        self.change_mode('CIRCLE')

        try:
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
                    "SIM_GPS_VERR_X": self.get_parameter("SIM_GPS_VERR_X") + 2,
                    "SIM_GPS_VERR_Y": self.get_parameter("SIM_GPS_VERR_Y") + 2,
                    "SIM_GPS_VERR_Z": self.get_parameter("SIM_GPS_VERR_Z") + 2,
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
                    0,
                    0,
                    0,
                    0,
                    int(loc.lat * 1e7),
                    int(loc.lng * 1e7),
                    50    # alt
                )
                self.delay_sim_time(5)
                # create an airspeed sensor error by freezing to the
                # current airspeed then changing the airspeed demand
                # to a higher value and waiting for the TECS speed
                # loop to diverge
                m = self.mav.recv_match(type='VFR_HUD', blocking=True)
                self.set_parameter("SIM_ARSPD_FAIL", m.airspeed)
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    0, # airspeed
                    30,
                    -1, # throttle / no change
                    0, # absolute values
                    0,
                    0,
                    0
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

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.remove_message_hook(statustext_hook)

        self.context_pop()

        # some parameters need reboot to take effect
        self.reboot_sitl()

        if ex is not None:
            raise ex

    def test_fence_alt_ceil_floor(self):
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        self.set_parameters({
            "FENCE_TYPE": 9,     # Set fence type to max and min alt
            "FENCE_ACTION": 0,   # Set action to report
            "FENCE_ALT_MAX": 200,
            "FENCE_ALT_MIN": 100,
        })

        # Grab Home Position
        self.mav.recv_match(type='HOME_POSITION', blocking=True)
        self.homeloc = self.mav.location()

        cruise_alt = 150
        self.takeoff(cruise_alt)

        self.do_fence_enable()

        self.progress("Fly above ceiling and check for breach")
        self.change_altitude(self.homeloc.alt + cruise_alt + 80)
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        self.progress("Got (%s)" % str(m))
        if ((m.onboard_control_sensors_health & fence_bit)):
            raise NotAchievedException("Fence Ceiling did not breach")

        self.progress("Return to cruise alt and check for breach clear")
        self.change_altitude(self.homeloc.alt + cruise_alt)

        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        self.progress("Got (%s)" % str(m))
        if (not (m.onboard_control_sensors_health & fence_bit)):
            raise NotAchievedException("Fence breach did not clear")

        self.progress("Fly below floor and check for breach")
        self.change_altitude(self.homeloc.alt + cruise_alt - 80)

        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        self.progress("Got (%s)" % str(m))
        if ((m.onboard_control_sensors_health & fence_bit)):
            raise NotAchievedException("Fence Floor did not breach")

        self.do_fence_disable()

        self.fly_home_land_and_disarm(timeout=150)

    def test_fence_breached_change_mode(self):
        """ Attempts to change mode while a fence is breached.
            This should revert to the mode specified by the fence action. """
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
        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            [
                locs
            ]
        )
        self.delay_sim_time(1)
        self.wait_ready_to_arm()
        self.takeoff(alt=50)
        self.change_mode("CRUISE")
        self.wait_distance(90, accuracy=15)

        self.progress("Enable fence and initiate fence action")
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.wait_mode("RTL") # We should RTL because of fence breach

        self.progress("User mode change to cruise should retrigger fence action")
        self.change_mode("CRUISE")
        self.wait_mode("RTL", timeout=5)

        self.progress("Test complete, disable fence and come home")
        self.do_fence_disable()
        self.fly_home_land_and_disarm()

    def test_fence_breach_no_return_point(self):
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
        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            [
                locs
            ]
        )
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

    def test_fence_breach_no_return_point_no_inclusion(self):
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

    def test_fence_disable_under_breach_action(self):
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

    def run_auxfunc(self,
                    function,
                    level,
                    want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
            function,  # p1
            level,  # p2
            0,  # p3
            0,  # p4
            0,  # p5
            0,  # p6
            0,  # p7
            want_result=want_result
        )

    def fly_aux_function(self):
        self.context_collect('STATUSTEXT')
        self.run_auxfunc(64, 2)  # 64 == reverse throttle
        self.wait_statustext("RevThrottle: ENABLE", check_context=True)
        self.run_auxfunc(64, 0)
        self.wait_statustext("RevThrottle: DISABLE", check_context=True)
        self.run_auxfunc(65, 2)  # 65 == GPS_DISABLE

        self.start_subtest("Bad auxfunc")
        self.run_auxfunc(
            65231,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED
        )

        self.start_subtest("Bad switchpos")
        self.run_auxfunc(
            62,
            17,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED
        )

    def fly_each_frame(self):
        vinfo = vehicleinfo.VehicleInfo()
        vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
            "firefly": "falls out of sky after transition",
            "plane-tailsitter": "does not take off; immediately emits 'AP: Transition VTOL done' while on ground",
            "quadplane-cl84": "falls out of sky instead of transitioning",
            "quadplane-tilttri": "falls out of sky instead of transitioning",
            "quadplane-tilttrivec": "loses attitude control and crashes",
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
            # the model string for Callisto has crap in it.... we
            # should really have another entry in the vehicleinfo data
            # to carry the path to the JSON.
            actual_model = model.split(":")[0]
            defaults = self.model_defaults_filepath(actual_model)
            if type(defaults) != list:
                defaults = [defaults]
            self.customise_SITL_commandline(
                ["--defaults", ','.join(defaults), ],
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

    def RCDisableAirspeedUse(self):
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
                0, # p1,
                0, # p2,
                0, # p3,
                0, # p4,
                new_home.latitude,
                new_home.longitude,
                new_home.altitude/1000.0, # mm => m
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

    def fly_landing_baro_drift(self):

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

    def DCMFallback(self):
        self.reboot_sitl()
        self.delay_sim_time(30)
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.takeoff(50)
        self.change_mode('CIRCLE')
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "EK3_POS_I_GATE": 0,
            "SIM_GPS_HZ": 1,
            "SIM_GPS_LAG_MS": 1000,
        })
        self.wait_statustext("DCM Active", check_context=True, timeout=60)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.wait_statustext("DCM Active", check_context=True)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.wait_statustext("DCM Active", check_context=True)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.context_stop_collecting('STATUSTEXT')

        self.fly_home_land_and_disarm()

    def ForcedDCM(self):

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.takeoff(50)
        self.context_collect('STATUSTEXT')
        self.set_parameter("AHRS_EKF_TYPE", 0)
        self.wait_statustext("DCM Active", check_context=True)
        self.context_stop_collecting('STATUSTEXT')

        self.fly_home_land_and_disarm()

    def MegaSquirt(self):
        self.assert_not_receiving_message('EFI_STATUS')
        self.set_parameters({
            'SIM_EFI_TYPE': 1,
            'EFI_TYPE': 1,
            'SERIAL5_PROTOCOL': 24,
        })
        self.customise_SITL_commandline(["--uartF=sim:megasquirt"])
        self.delay_sim_time(5)
        m = self.assert_receive_message('EFI_STATUS')
        mavutil.dump_message_verbose(sys.stdout, m)
        if m.throttle_out != 0:
            raise NotAchievedException("Expected zero throttle")
        if m.health != 1:
            raise NotAchievedException("Not healthy")
        if m.intake_manifold_temperature < 20:
            raise NotAchievedException("Bad intake manifold temperature")

    def test_glide_slope_threshold(self):

        # Test that GLIDE_SLOPE_THRESHOLD correctly controls re-planning glide slope
        # in the scenario that aircraft is above planned slope and slope is positive (climbing).
        #
        #
        #  Behaviour with GLIDE_SLOPE_THRESH = 0 (no slope replanning)
        #       (2)..      __(4)
        #         |  \..__/
        #         |  __/
        #         (3)
        #
        # Behaviour with GLIDE_SLOPE_THRESH = 5 (slope replanning when >5m error)
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
        # Initial run with GLIDE_SLOPE_THR = 5 (default).
        #
        self.set_parameter("GLIDE_SLOPE_THR", 5)

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
        # Second run with GLIDE_SLOPE_THR = 0 (no re-plan).
        #
        self.set_parameter("GLIDE_SLOPE_THR", 0)

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
                raise NotAchievedException("Reached desired waypoint without first decending 10m,\
 indicating slope was replanned unexpectedly")

            if (self.get_altitude(relative=True, timeout=2) - init_altitude) < -10:
                break

        # Disarm
        self.wait_disarmed(timeout=600)

        self.progress("Mission OK")

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestPlane, self).tests()
        ret.extend([

            ("AuxModeSwitch",
             "Set modes via auxswitches",
             self.test_setting_modes_via_auxswitches),

            ("TestRCCamera",
             "Test RC Option - Camera Trigger",
             self.test_rc_option_camera_trigger),

            ("TestRCRelay", "Test Relay RC Channel Option", self.test_rc_relay),

            ("ThrottleFailsafe",
             "Fly throttle failsafe",
             self.ThrottleFailsafe),

            ("NeedEKFToArm",
             "Ensure we need EKF to be healthy to arm",
             self.test_need_ekf_to_arm),

            ("ThrottleFailsafeFence",
             "Fly fence survives throttle failsafe",
             self.test_throttle_failsafe_fence),

            ("TestFlaps", "Flaps", self.fly_flaps),

            ("DO_CHANGE_SPEED", "Test mavlink DO_CHANGE_SPEED command", self.fly_do_change_speed),

            ("DO_REPOSITION",
             "Test mavlink DO_REPOSITION command",
             self.fly_do_reposition),

            ("GuidedRequest",
             "Test handling of MISSION_ITEM in guided mode",
             self.fly_do_guided_request),

            ("MainFlight",
             "Lots of things in one flight",
             self.test_main_flight),

            ("TestGripperMission",
             "Test Gripper mission items",
             self.test_gripper_mission),

            ("Parachute", "Test Parachute", self.test_parachute),

            ("ParachuteSinkRate", "Test Parachute (SinkRate triggering)", self.test_parachute_sinkrate),

            ("AIRSPEED_AUTOCAL", "Test AIRSPEED_AUTOCAL", self.airspeed_autocal),

            ("RangeFinder",
             "Test RangeFinder Basic Functionality",
             self.test_rangefinder),

            ("FenceStatic",
             "Test Basic Fence Functionality",
             self.test_fence_static),

            ("FenceRTL",
             "Test Fence RTL",
             self.test_fence_rtl),

            ("FenceRTLRally",
             "Test Fence RTL Rally",
             self.test_fence_rtl_rally),

            ("FenceRetRally",
             "Test Fence Ret_Rally",
             self.test_fence_ret_rally),

            ("FenceAltCeilFloor",
             "Tests the fence ceiling and floor",
             self.test_fence_alt_ceil_floor),

            ("FenceBreachedChangeMode",
             "Tests retrigger of fence action when changing of mode while fence is breached",
             self.test_fence_breached_change_mode),

            ("FenceNoFenceReturnPoint",
             "Tests calculated return point during fence breach when no fence return point present",
             self.test_fence_breach_no_return_point),

            ("FenceNoFenceReturnPointInclusion",
             "Tests using home as fence return point when none is present, and no inclusion fence is uploaded",
             self.test_fence_breach_no_return_point_no_inclusion),

            ("FenceDisableUnderAction",
             "Tests Disabling fence while undergoing action caused by breach",
             self.test_fence_disable_under_breach_action),

            ("ADSB",
             "Test ADSB",
             self.test_adsb),

            ("SimADSB",
             "Test SIM_ADSB",
             self.SimADSB),

            ("Button",
             "Test Buttons",
             self.test_button),

            ("FRSkySPort",
             "Test FrSky SPort mode",
             self.test_frsky_sport),

            ("FRSkyPassThrough",
             "Test FrSky PassThrough serial output",
             self.test_frsky_passthrough),

            ("FRSkyMAVlite",
             "Test FrSky MAVlite serial output",
             self.test_frsky_mavlite),

            ("FRSkyD",
             "Test FrSkyD serial output",
             self.test_frsky_d),

            ("LTM",
             "Test LTM serial output",
             self.test_ltm),

            ("DEVO",
             "Test DEVO serial output",
             self.DEVO),

            ("AdvancedFailsafe",
             "Test Advanced Failsafe",
             self.test_advanced_failsafe),

            ("LOITER",
             "Test Loiter mode",
             self.LOITER),

            ("DeepStall",
             "Test DeepStall Landing",
             self.fly_deepstall),

            ("WatchdogHome",
             "Ensure home is restored after watchdog reset",
             self.WatchdogHome),

            ("LargeMissions",
             "Test Manipulation of Large missions",
             self.test_large_missions),

            ("Soaring",
             "Test Soaring feature",
             self.fly_soaring),

            ("Terrain",
             "Test AP_Terrain",
             self.Terrain),

            ("TerrainMission",
             "Test terrain following in mission",
             self.TerrainMission),

            ("Terrain-loiter",
             "Test terrain following in loiter",
             self.test_loiter_terrain),

            ("VectorNavEAHRS",
             "Test VectorNav EAHRS support",
             self.test_vectornav),

            ("LordEAHRS",
             "Test LORD Microstrain EAHRS support",
             self.test_lord),

            ("Deadreckoning",
             "Test deadreckoning support",
             self.deadreckoning),

            ("DeadreckoningNoAirSpeed",
             "Test deadreckoning support with no airspeed sensor",
             self.deadreckoning_no_airspeed_sensor),

            ("EKFlaneswitch",
             "Test EKF3 Affinity and Lane Switching",
             self.ekf_lane_switch),

            ("AirspeedDrivers",
             "Test AirSpeed drivers",
             self.test_airspeed_drivers),

            ("RTL_CLIMB_MIN",
             "Test RTL_CLIMB_MIN",
             self.rtl_climb_min),

            ("ClimbBeforeTurn",
             "Test climb-before-turn",
             self.climb_before_turn),

            ("IMUTempCal",
             "Test IMU temperature calibration",
             self.test_imu_tempcal),

            ("MAV_DO_AUX_FUNCTION",
             "Test triggering Auxillary Functions via mavlink",
             self.fly_aux_function),

            ("SmartBattery",
             "Test smart battery logging etc",
             self.SmartBattery),

            ("FlyEachFrame",
             "Fly each supported internal frame",
             self.fly_each_frame),

            ("RCDisableAirspeedUse",
             "Test RC DisableAirspeedUse option",
             self.RCDisableAirspeedUse),

            ("AHRS_ORIENTATION",
             "Test AHRS_ORIENTATION parameter",
             self.AHRS_ORIENTATION),

            ("AHRSTrim",
             "AHRS trim testing",
             self.ahrstrim),

            ("Landing-Drift",
             "Circuit with baro drift",
             self.fly_landing_baro_drift),

            ("ForcedDCM",
             "Switch to DCM mid-flight",
             self.ForcedDCM),

            ("DCMFallback",
             "Really annoy the EKF and force fallback",
             self.DCMFallback),

            ("MAVFTP",
             "Test MAVProxy can talk FTP to autopilot",
             self.MAVFTP),

            ("AUTOTUNE",
             "Test AutoTune mode",
             self.AUTOTUNE),

            ("MegaSquirt",
             "Test MegaSquirt EFI",
             self.MegaSquirt),

            ("MSP_DJI",
             "Test MSP DJI serial output",
             self.test_msp_dji),

            ("SpeedToFly",
             "Test soaring speed-to-fly",
             self.fly_soaring_speed_to_fly),

            ("GlideSlopeThresh",
             "Test rebuild glide slope if above and climbing",
             self.test_glide_slope_threshold),

            ("LogUpload",
             "Log upload",
             self.log_upload),

            ("HIGH_LATENCY2",
             "Set sending of HIGH_LATENCY2",
             self.HIGH_LATENCY2),
        ])
        return ret

    def disabled_tests(self):
        return {
            "Terrain-loiter": "Loading of terrain data is not reliable",
            "Landing-Drift": "Flapping test. See https://github.com/ArduPilot/ardupilot/issues/20054",
        }
