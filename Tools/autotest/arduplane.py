#!/usr/bin/env python

# Fly ArduPlane in SITL
from __future__ import print_function
import math
import os
import time

from pymavlink import quaternion
from pymavlink import mavutil

from common import AutoTest
from common import AutoTestTimeoutException
from common import NotAchievedException
from common import PreconditionFailedException

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
        self.set_rc(3, 1600)
        self.set_rc(4, 1500)
        self.wait_groundspeed(12, 100)

        # hit the gas harder now, and give it some more elevator
        self.set_rc(2, 1100)
        self.set_rc(3, 2000)

        # gain a bit of altitude
        self.wait_altitude(alt, alt_max, timeout=30, relative=relative)

        # level off
        self.set_rc(2, 1500)

        self.progress("TAKEOFF COMPLETE")

    def fly_left_circuit(self):
        """Fly a left circuit, 200m on a side."""
        self.mavproxy.send('switch 4\n')
        self.wait_mode('FBWA')
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
        self.mavproxy.send('switch 2\n')
        self.wait_mode('RTL')
        self.wait_location(self.homeloc,
                           accuracy=120,
                           target_altitude=self.homeloc.alt+100,
                           height_accuracy=20,
                           timeout=180)
        self.progress("RTL Complete")

    def fly_LOITER(self, num_circles=4):
        """Loiter where we are."""
        self.progress("Testing LOITER for %u turns" % num_circles)
        self.mavproxy.send('loiter\n')
        self.wait_mode('LOITER')

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

        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')

        if abs(final_alt - initial_alt) > 20:
            raise NotAchievedException("Failed to maintain altitude")

        self.progress("Completed Loiter OK")

    def fly_CIRCLE(self, num_circles=1):
        """Circle where we are."""
        self.progress("Testing CIRCLE for %u turns" % num_circles)
        self.mavproxy.send('mode CIRCLE\n')
        self.wait_mode('CIRCLE')

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

        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')

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
        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')
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
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')

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
        self.mavproxy.send('switch 4\n')
        self.wait_mode('FBWA')
        self.set_rc(3, 1700)
        return self.wait_level_flight()

    def inside_loop(self, count=1):
        """Fly a inside loop."""
        # full throttle!
        self.set_rc(3, 2000)
        self.change_altitude(self.homeloc.alt+300)
        # fly the loop in manual
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')

        while count > 0:
            self.progress("Starting loop")
            self.set_rc(2, 1000)
            self.wait_pitch(-60, accuracy=20)
            self.wait_pitch(0, accuracy=20)
            count -= 1

        # back to FBWA
        self.set_rc(2, 1500)
        self.mavproxy.send('switch 4\n')
        self.wait_mode('FBWA')
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
            self.mavproxy.send('mode FBWA\n')
            self.wait_mode('FBWA')
            self.set_rc(3, 1700)
            raise e

        # back to FBWA
        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')
        self.set_rc(3, 1700)
        self.wait_level_flight()

    def test_stabilize(self, count=1):
        """Fly stabilize mode."""
        # full throttle!
        self.set_rc(3, 2000)
        self.set_rc(2, 1300)
        self.change_altitude(self.homeloc.alt+300)
        self.set_rc(2, 1500)

        self.mavproxy.send("mode STABILIZE\n")
        self.wait_mode('STABILIZE')

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
        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')
        self.set_rc(3, 1700)
        return self.wait_level_flight()

    def test_acro(self, count=1):
        """Fly ACRO mode."""
        # full throttle!
        self.set_rc(3, 2000)
        self.set_rc(2, 1300)
        self.change_altitude(self.homeloc.alt+300)
        self.set_rc(2, 1500)

        self.mavproxy.send("mode ACRO\n")
        self.wait_mode('ACRO')

        while count > 0:
            self.progress("Starting roll")
            self.set_rc(1, 1000)
            self.wait_roll(-150, accuracy=90)
            self.wait_roll(150, accuracy=90)
            self.wait_roll(0, accuracy=90)
            count -= 1
        self.set_rc(1, 1500)

        # back to FBWA
        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')

        self.wait_level_flight()

        self.mavproxy.send("mode ACRO\n")
        self.wait_mode('ACRO')

        count = 2
        while count > 0:
            self.progress("Starting loop")
            self.set_rc(2, 1000)
            self.wait_pitch(-60, accuracy=20)
            self.wait_pitch(0, accuracy=20)
            count -= 1

        self.set_rc(2, 1500)

        # back to FBWA
        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')
        self.set_rc(3, 1700)
        return self.wait_level_flight()

    def test_FBWB(self, mode='FBWB'):
        """Fly FBWB or CRUISE mode."""
        self.mavproxy.send("mode %s\n" % mode)
        self.wait_mode(mode)
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
        self.mavproxy.send('mode FBWA\n')
        self.wait_mode('FBWA')

        if abs(final_alt - initial_alt) > 20:
            raise NotAchievedException("Failed to maintain altitude")

        return self.wait_level_flight()

    def fly_mission(self, filename, mission_timeout=60.0):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        self.load_mission(filename)
        self.mavproxy.send('switch 1\n')  # auto mode
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 7, max_dist=60)
        self.wait_groundspeed(0, 0.5, timeout=mission_timeout)
        self.mavproxy.expect("Auto disarmed")
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
            int(loc.lat*1e7),
            int(loc.lng*1e7),
            new_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_altitude(new_alt-10, new_alt, timeout=30, relative=True)

        self.fly_home_land_and_disarm()

    def fly_deepstall(self):
#        self.fly_deepstall_absolute()
        self.fly_deepstall_relative()

    def fly_deepstall_absolute(self):
        self.start_subtest("DeepStall Relative Absolute")
        self.set_parameter("LAND_TYPE", 1)
        deepstall_elevator_pwm = 1661
        self.set_parameter("LAND_DS_ELEV_PWM", deepstall_elevator_pwm)
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
        self.set_parameter("LAND_TYPE", 1)
        deepstall_elevator_pwm = 1661
        self.set_parameter("LAND_DS_ELEV_PWM", deepstall_elevator_pwm)
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

    def fly_do_change_speed(self):
        # the following lines ensure we revert these parameter values
        # - DO_CHANGE_AIRSPEED is a permanent vehicle change!
        self.set_parameter("TRIM_ARSPD_CM", self.get_parameter("TRIM_ARSPD_CM"))
        self.set_parameter("MIN_GNDSPD_CM", self.get_parameter("MIN_GNDSPD_CM"))

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
            12345, # lat*1e7
            12345, # lon*1e7
            100    # alt
        )
        self.delay_sim_time(10)
        self.progress("Ensuring initial speed is known and relatively constant")
        initial_speed = 21.5;
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
        self.set_parameter("SIM_WIND_SPD", 5)
        self.set_parameter("SIM_WIND_DIR", 270)
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
        self.fly_home_land_and_disarm()

    def fly_home_land_and_disarm(self, timeout=120):
        filename = "flaps.txt"
        self.progress("Using %s to fly home" % filename)
        num_wp = self.load_mission(filename)
        self.change_mode("AUTO")
        self.mavproxy.send('wp set 7\n')
        self.drain_mav()
        # TODO: reflect on file to find this magic waypoint number?
#        self.wait_waypoint(7, num_wp-1, timeout=500) # we tend to miss the final waypoint by a fair bit, and this is probably too noisy anyway?
        self.wait_disarmed(timeout=timeout)

    def fly_flaps(self):
        """Test flaps functionality."""
        filename = "flaps.txt"
        self.context_push()
        ex = None
        try:
            flaps_ch = 5
            servo_ch = 5
            self.set_parameter("SERVO%u_FUNCTION" % servo_ch, 3) # flapsauto
            self.set_parameter("RC%u_OPTION" % flaps_ch, 208) # Flaps RCx_OPTION
            self.set_parameter("LAND_FLAP_PERCNT", 50)
            self.set_parameter("LOG_DISARMED", 1)
            flaps_ch_min = 1000
            flaps_ch_trim = 1500
            flaps_ch_max = 2000
            self.set_parameter("RC%u_MIN" % flaps_ch, flaps_ch_min)
            self.set_parameter("RC%u_MAX" % flaps_ch, flaps_ch_max)
            self.set_parameter("RC%u_TRIM" % flaps_ch, flaps_ch_trim)

            servo_ch_min = 1200
            servo_ch_trim = 1300
            servo_ch_max = 1800
            self.set_parameter("SERVO%u_MIN" % servo_ch, servo_ch_min)
            self.set_parameter("SERVO%u_MAX" % servo_ch, servo_ch_max)
            self.set_parameter("SERVO%u_TRIM" % servo_ch, servo_ch_trim)

            self.progress("check flaps are not deployed")
            self.set_rc(flaps_ch, flaps_ch_min)
            self.wait_servo_channel_value(servo_ch, servo_ch_min)
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
            self.mavproxy.send('wp set 1\n')
            self.mavproxy.send('switch 1\n')  # auto mode
            self.wait_mode('AUTO')
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

    def test_throttle_failsafe(self):
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
        self.set_parameter("SIM_RC_FAIL", 2) # throttle-to-950
        self.wait_mode('CIRCLE') # short failsafe
        self.wait_mode('RTL') # long failsafe
        self.progress("Ensure we've had our throttle squashed to 950")
        self.wait_rc_channel_value(3, 950)
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        print("%s" % str(m))
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
        self.set_parameter("SIM_RC_FAIL", 1) # no-pulses
        self.wait_mode('CIRCLE') # short failsafe
        self.wait_mode('RTL') # long failsafe
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        print("%s" % str(m))
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
        ex = None
        try:
            self.set_parameter("FS_SHORT_ACTN", 3) # 3 means disabled
            self.set_parameter("SIM_RC_FAIL", 1)
            self.wait_statustext("Long event on")
            self.wait_mode("RTL")
            self.set_parameter("SIM_RC_FAIL", 0)
            self.wait_text("Long event off")
            self.change_mode("MANUAL")

            self.progress("Trying again with THR_FS_VALUE")
            self.set_parameter("THR_FS_VALUE", 960)
            self.set_parameter("SIM_RC_FAIL", 2)
            self.wait_statustext("Long event on")
            self.wait_mode("RTL")
        except Exception as e:
            self.progress("Exception caught:")
            self.progress(self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def test_throttle_failsafe_fence(self):
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.progress("Checking fence is not present before being configured")
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        print("%s" % str(m))
        if (m.onboard_control_sensors_enabled & fence_bit):
            raise NotAchievedException("Fence enabled before being configured")

        self.change_mode('MANUAL')
        self.wait_ready_to_arm()

        self.load_fence("CMAC-fence.txt")

        self.set_parameter("FENCE_CHANNEL", 7)
        self.set_parameter("FENCE_ACTION", 4)
        self.set_rc(3, 1000)
        self.set_rc(7, 2000)

        self.progress("Checking fence is initially OK")
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        print("%s" % str(m))
        if (not (m.onboard_control_sensors_enabled & fence_bit)):
            raise NotAchievedException("Fence not initially enabled")

        self.set_parameter("THR_FS_VALUE", 960)
        self.progress("Failing receiver (throttle-to-950)")
        self.set_parameter("SIM_RC_FAIL", 2) # throttle-to-950
        self.wait_mode("CIRCLE")
        self.delay_sim_time(1) # give
        self.drain_mav_unparsed()

        self.progress("Checking fence is OK after receiver failure (bind-values)")
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True)
        print("%s" % str(m))
        if (not (m.onboard_control_sensors_enabled & fence_bit)):
            raise NotAchievedException("Fence not enabled after RC fail")

    def test_gripper_mission(self):
        self.context_push()
        ex = None
        try:
            self.load_mission("plane-gripper-mission.txt")
            self.mavproxy.send("wp set 1\n")
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.mavproxy.expect("Gripper Grabbed")
            self.mavproxy.expect("Gripper Released")
            self.mavproxy.expect("Auto disarmed")
        except Exception as e:
            self.progress("Exception caught:")
            self.progress(self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def assert_fence_sys_status(self, present, enabled, health):
        self.delay_sim_time(1)
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if m is None:
            raise NotAchievedException("Did not receive SYS_STATUS")
        tests = [ ( "present", present, m.onboard_control_sensors_present ),
                  ( "enabled", enabled, m.onboard_control_sensors_enabled ),
                  ( "health", health, m.onboard_control_sensors_health ),
                  ]
        bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        for test in tests:
            (name, want, field) = test
            got = (field & bit) != 0
            if want != got:
                raise NotAchievedException("fence status incorrect; %s want=%u got=%u" %
                                           (name, want, got))

    def do_fence_en_or_dis_able(self, value, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        if value:
            p1 = 1
        else:
            p1 = 0
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
                     p1, # param1
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     0, # param7
                     want_result=want_result)

    def do_fence_enable(self, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        self.do_fence_en_or_dis_able(True, want_result=want_result)

    def do_fence_disable(self, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        self.do_fence_en_or_dis_able(False, want_result=want_result)

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
            self.assert_fence_sys_status(False, False, True)
            self.load_fence("CMAC-fence.txt")
            m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=2)
            if m is not None:
                raise NotAchievedException("Got FENCE_STATUS unexpectedly");
            self.drain_mav_unparsed()
            self.set_parameter("FENCE_ACTION", mavutil.mavlink.FENCE_ACTION_NONE) # report only
            self.assert_fence_sys_status(False, False, True)
            self.set_parameter("FENCE_ACTION", mavutil.mavlink.FENCE_ACTION_RTL) # report only
            self.assert_fence_sys_status(True, False, True)
            self.mavproxy.send('fence enable\n')
            self.mavproxy.expect("fence enabled")
            self.assert_fence_sys_status(True, True, True)
            m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=2)
            if m is None:
                raise NotAchievedException("Did not get FENCE_STATUS");
            if m.breach_status:
                raise NotAchievedException("Breached fence unexpectedly (%u)" %
                                           (m.breach_status))
            self.mavproxy.send('fence disable\n')
            self.mavproxy.expect("fence disabled")
            self.assert_fence_sys_status(True, False, True)
            self.set_parameter("FENCE_ACTION", mavutil.mavlink.FENCE_ACTION_NONE)
            self.assert_fence_sys_status(False, False, True)
            self.set_parameter("FENCE_ACTION", mavutil.mavlink.FENCE_ACTION_RTL)
            self.assert_fence_sys_status(True, False, True)
            self.mavproxy.send("fence clear\n")
            self.mavproxy.expect("fence removed")
            if self.get_parameter("FENCE_TOTAL") != 0:
                raise NotAchievedException("Expected zero points remaining")
            self.assert_fence_sys_status(False, False, True)
            self.progress("Trying to enable fence with no points")
            self.do_fence_enable(want_result=mavutil.mavlink.MAV_RESULT_FAILED)

            # test a rather unfortunate behaviour:
            self.progress("Killing a live fence with fence-clear")
            self.load_fence("CMAC-fence.txt")
            self.set_parameter("FENCE_ACTION", mavutil.mavlink.FENCE_ACTION_RTL)
            self.do_fence_enable()
            self.assert_fence_sys_status(True, True, True)
            self.mavproxy.send("fence clear\n")
            self.mavproxy.expect("fence removed")
            if self.get_parameter("FENCE_TOTAL") != 0:
                raise NotAchievedException("Expected zero points remaining")
            self.assert_fence_sys_status(False, False, True)

        except Exception as e:
            self.progress("Exception caught:")
            self.progress(self.get_exception_stacktrace(e))
            ex = e
        self.mavproxy.send('fence clear\n')
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
            self.set_parameter("RTL_RADIUS", want_radius)
            self.set_parameter("NAVL1_LIM_BANK", 60)
            self.set_parameter("FENCE_ACTION", mavutil.mavlink.FENCE_ACTION_RTL)

            self.do_fence_enable()
            self.assert_fence_sys_status(True, True, True)

            self.takeoff(alt=45, alt_max=300)

            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 30:
                    raise NotAchievedException("Did not breach fence")
                m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=2)
                if m is None:
                    raise NotAchievedException("Did not get FENCE_STATUS");
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
            self.progress("Exception caught:")
            self.progress(self.get_exception_stacktrace(e))
            ex = e
        self.mavproxy.send('fence clear\n')
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
            loc = self.home_position_as_mav_location()
            self.location_offset_ne(loc, 50, -50)

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
            self.mavproxy.send("rally list\n")
            self.test_fence_breach_circle_at(loc)
        except Exception as e:
            self.progress("Exception caught:")
            self.progress(self.get_exception_stacktrace(e))
            ex = e
        self.mavproxy.send('rally clear\n')
        if ex is not None:
            raise ex

    def test_parachute(self):
        self.set_rc(9, 1000)
        self.set_parameter("CHUTE_ENABLED", 1)
        self.set_parameter("CHUTE_TYPE", 10)
        self.set_parameter("SERVO9_FUNCTION", 27)
        self.set_parameter("SIM_PARA_ENABLE", 1)
        self.set_parameter("SIM_PARA_PIN", 9)

        self.load_mission("plane-parachute-mission.txt")
        self.mavproxy.send("wp set 1\n")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mavproxy.expect("BANG")
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def test_parachute_sinkrate(self):
        self.set_rc(9, 1000)
        self.set_parameter("CHUTE_ENABLED", 1)
        self.set_parameter("CHUTE_TYPE", 10)
        self.set_parameter("SERVO9_FUNCTION", 27)
        self.set_parameter("SIM_PARA_ENABLE", 1)
        self.set_parameter("SIM_PARA_PIN", 9)

        self.set_parameter("CHUTE_CRT_SINK", 9)

        self.progress("Takeoff")
        self.takeoff(alt=300)

        self.progress("Diving")
        self.set_rc(2, 2000)
        self.mavproxy.expect("BANG")

        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def run_subtest(self, desc, func):
        self.start_subtest(desc)
        func()

    def test_main_flight(self):

        self.change_mode('MANUAL')

        self.progress("Asserting we don't support transfer of fence via mission item protocol")
        self.assert_no_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)

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

        self.run_subtest("Mission test",
                         lambda: self.fly_mission("ap1.txt"))

    def airspeed_autocal(self):
        self.progress("Ensure no AIRSPEED_AUTOCAL on ground")
        self.set_parameter("ARSPD_AUTOCAL", 1)
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
        self.wait_waypoint(7, num_wp-1, timeout=500)
        self.wait_disarmed(timeout=120)

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
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))

        if m is not None:
            raise NotAchievedException("Received unexpected RANGEFINDER msg")

        try:
            self.set_analog_rangefinder_parameters()

            self.reboot_sitl()

            '''ensure rangefinder gives height-above-ground'''
            self.load_mission("plane-gripper-mission.txt") # borrow this
            self.mavproxy.send("wp set 1\n")
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
                raise NotAchievedException("rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" % (rf.distance, gpi.relative_alt/1000.0))
            self.mavproxy.expect("Auto disarmed")

            self.progress("Ensure RFND messages in log")
            if not self.current_onboard_log_contains_message("RFND"):
                raise NotAchievedException("No RFND messages in log")

        except Exception as e:
            self.progress("Exception caught:")
            self.progress(self.get_exception_stacktrace(e))
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

    def default_mode(self):
        return "MANUAL"

    def test_pid_tuning(self):
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestPlane, self).test_pid_tuning()


    def test_setting_modes_via_auxswitches(self):
        self.set_parameter("FLTMODE5", 1)
        self.mavproxy.send('switch 1\n')  # random mode
        self.wait_heartbeat()
        self.change_mode('MANUAL')
        self.mavproxy.send('switch 5\n')  # acro mode
        self.wait_mode("CIRCLE")
        self.set_rc(9, 1000)
        self.set_rc(10, 1000)
        self.set_parameter("RC9_OPTION", 4) # RTL
        self.set_parameter("RC10_OPTION", 55) # guided
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

    def test_adsb_send_threatening_adsb_message(self, here):
        self.progress("Sending ABSD_VEHICLE message")
        self.mav.mav.adsb_vehicle_send(37, # ICAO address
                                       int(here.lat * 1e7),
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

    def test_adsb(self):
        self.context_push()
        ex = None
        try:
            # message ADSB_VEHICLE 37 -353632614 1491652305 0 584070 0 0 0 "bob" 3 1 255 17
            self.set_parameter("RC12_OPTION", 38) # avoid-adsb
            self.set_rc(12, 2000)
            self.set_parameter("ADSB_ENABLE", 1)
            self.set_parameter("AVD_ENABLE", 1)
            self.set_parameter("AVD_F_ACTION", mavutil.mavlink.MAV_COLLISION_ACTION_RTL)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            here = self.mav.location()
            self.change_mode("FBWA")
            self.delay_sim_time(2) # TODO: work out why this is required...
            self.test_adsb_send_threatening_adsb_message(here)
            self.progress("Waiting for collision message")
            m = self.mav.recv_match(type='COLLISION', blocking=True, timeout=4)
            if m is None:
                raise NotAchievedException("Did not get collision message")
            if m.threat_level != 2:
                raise NotAchievedException("Expected some threat at least")
            if m.action != mavutil.mavlink.MAV_COLLISION_ACTION_RTL:
                raise NotAchievedException("Incorrect action; want=%u got=%u" %
                                           (mavutil.mavlink.MAV_COLLISION_ACTION_RTL, m.action))
            self.wait_mode("RTL")

            self.progress("Sending far-away ABSD_VEHICLE message")
            self.mav.mav.adsb_vehicle_send(37, # ICAO address
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
            print("Got (%s)" % str(m))
            if m is not None:
                raise NotAchievedException("Got collision message when I shouldn't have")

        except Exception as e:
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
            int(loc.lat *1e7), # latitude
            int(loc.lng *1e7), # longitude
            loc.alt, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        m = self.mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if m is None:
            raise NotAchievedException("Did not get MISSION_ACK")
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
            int(loc.lat *1e7), # latitude
            int(loc.lng *1e7), # longitude
            desired_relative_alt, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        m = self.mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if m is None:
            raise NotAchievedException("Did not get MISSION_ACK")
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Did not get accepted response")
        self.wait_location(loc, accuracy=100) # based on loiter radius
        self.delay_sim_time(20)
        self.wait_altitude(altitude_min=desired_relative_alt-3,
                           altitude_max=desired_relative_alt+3,
                           relative=True)

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
            m = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=5)
            if m is None:
                raise NotAchievedException("Did not get ATTITUDE")
            pitch = math.degrees(m.pitch)
            self.progress("Pitch:%f throttle:%u alt:%f" % (pitch, new_throttle, alt))
        m = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=5)
        if m is None:
            raise NotAchievedException("Did not get VFR_HUD")
        initial_throttle = m.throttle
        initial_alt = m.alt
        self.progress("Initial throttle: %u" % initial_throttle)
        # pitch down, ensure throttle decreases:
        rc2_max = self.get_parameter("RC2_MAX")
        self.set_rc(2, rc2_max)
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            '''stick-mixing is pushing the aircraft down.  It doesn't want to go
            down (the target loiter altitude hasn't changed), so it
            tries to add energy by increasing the throttle.
            '''
            if now - tstart > 60:
                raise NotAchievedException("Did not see increase in throttle")
            m = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=5)
            if m is None:
                raise NotAchievedException("Did not get VFR_HUD")
            new_throttle = m.throttle
            alt = m.alt
            m = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=5)
            if m is None:
                raise NotAchievedException("Did not get ATTITUDE")
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
        self.load_mission("Kingaroy-vlarge.txt")
        self.load_mission("Kingaroy-vlarge2.txt")

    def fly_soaring(self):

        model="plane-soaring"

        self.customise_SITL_commandline([],
                                        model=model,
                                        defaults_filepath=self.model_defaults_filepath("ArduPlane",model),
                                        wipe=True)

        self.load_mission('CMAC-soar.txt')


        self.mavproxy.send("wp set 1\n")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Enable thermalling RC
        rc_chan = 0
        for i in range(8):
            rcx_option = self.get_parameter('RC{0}_OPTION'.format(i+1))
            if rcx_option==88:
                rc_chan = i+1;
                break

        if rc_chan==0:
            raise NotAchievedException("Did not find soaring enable channel option.")

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
        self.delay_sim_time(5)

        # Now disable soaring (should hold altitude)
        self.set_parameter("SOAR_ENABLE", 0)
        self.delay_sim_time(10)

        #And reenable. This should force throttle-down
        self.set_parameter("SOAR_ENABLE", 1)
        self.delay_sim_time(10)

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

        self.progress("Mission OK")

    def fly_terrain_mission(self):

        self.customise_SITL_commandline([], wipe=True)

        self.mavproxy.send("wp set 1\n")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.fly_mission("ap-terrain.txt", mission_timeout=600)

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
             self.test_throttle_failsafe),

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

            ("ADSB",
             "Test ADSB",
             self.test_adsb),

            ("Button",
             "Test Buttons",
             self.test_button),

            ("FRSkySPort",
             "Test FrSky SPort mode",
             self.test_frsky_sport),

            ("FRSkyPassThrough",
             "Test FrSky PassThrough serial output",
             self.test_frsky_passthrough),

            ("FRSkyD",
             "Test FrSkyD serial output",
             self.test_frsky_d),

            ("LTM",
             "Test LTM serial output",
             self.test_ltm),

            ("AdvancedFailsafe",
             "Test Advanced Failsafe",
             self.test_advanced_failsafe),

            ("LOITER",
             "Test Loiter mode",
             self.LOITER),

            ("DeepStall",
             "Test DeepStall Landing",
             self.fly_deepstall),

            ("LargeMissions",
             "Test Manipulation of Large missions",
             self.test_large_missions),

            ("Soaring",
            "Test Soaring feature",
            self.fly_soaring),

            ("Terrain",
             "Test terrain following in mission",
             self.fly_terrain_mission),

            ("LogUpload",
             "Log upload",
             self.log_upload),
        ])
        return ret
