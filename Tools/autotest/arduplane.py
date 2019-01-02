#!/usr/bin/env python

# Fly ArduPlane in SITL
from __future__ import print_function
import math
import os

import pexpect
from pymavlink import quaternion
from pymavlink import mavutil

from pysim import util

from common import AutoTest
from common import AutoTestTimeoutException
from common import NotAchievedException
from common import PreconditionFailedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-35.362938, 149.165085, 585, 354)
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestPlane(AutoTest):
    def __init__(self,
                 binary,
                 valgrind=False,
                 gdb=False,
                 speedup=10,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 breakpoints=[],
                 **kwargs):
        super(AutoTestPlane, self).__init__(**kwargs)
        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver
        self.breakpoints = breakpoints

        self.home = "%f,%f,%u,%u" % (HOME.lat,
                                     HOME.lng,
                                     HOME.alt,
                                     HOME.heading)
        self.homeloc = None
        self.speedup = speedup

        self.sitl = None
        self.hasInit = False

        self.log_name = "ArduPlane"

    def init(self):
        if self.frame is None:
            self.frame = 'plane-elevrev'

        self.mavproxy_logfile = self.open_mavproxy_logfile()

        defaults_file = os.path.join(testdir,
                                     'default_params/plane-jsbsim.parm')
        self.sitl = util.start_SITL(self.binary,
                                    wipe=True,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    defaults_file=defaults_file,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver,
                                    breakpoints=self.breakpoints)
        self.mavproxy = util.start_MAVProxy_SITL(
            'ArduPlane', options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        self.logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)
        self.try_symlink_tlog()

        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Started simulator")

        self.get_mavlink_connection_going()

        self.hasInit = True
        self.progress("Ready to start testing!")

    def takeoff(self):
        """Takeoff get to 30m altitude."""

        self.mavproxy.send('switch 4\n')
        self.wait_mode('FBWA')

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # some rudder to counteract the prop torque
        self.set_rc(4, 1700)

        # some up elevator to keep the tail down
        self.set_rc(2, 1200)

        # get it moving a bit first
        self.set_rc(3, 1300)
        self.mav.recv_match(condition='VFR_HUD.groundspeed>6', blocking=True)

        # a bit faster again, straighten rudder
        self.set_rc(3, 1600)
        self.set_rc(4, 1500)
        self.mav.recv_match(condition='VFR_HUD.groundspeed>12', blocking=True)

        # hit the gas harder now, and give it some more elevator
        self.set_rc(2, 1100)
        self.set_rc(3, 2000)

        # gain a bit of altitude
        self.wait_altitude(self.homeloc.alt+150,
                           self.homeloc.alt+180,
                           timeout=30)

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
        while self.get_sim_time() < tstart + timeout:
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

    def set_attitude_target(self):
        """Test setting of attitude target in guided mode."""
        # mode guided:
        self.mavproxy.send('mode GUIDED\n')
        self.wait_mode('GUIDED')

        target_roll_degrees = 70
        state_roll_over = "roll-over"
        state_stabilize_roll = "stabilize-roll"
        state_hold = "hold"
        state_roll_back = "roll-back"
        state_done = "done"

        tstart = self.get_sim_time()

        try:
            state = state_roll_over
            while state != state_done:
                if self.get_sim_time() - tstart > 20:
                    raise AutoTestTimeoutException("Manuevers not completed")

                m = self.mav.recv_match(type='ATTITUDE',
                                        blocking=True,
                                        timeout=0.1)
                if m is None:
                    continue

                r = math.degrees(m.roll)
                if state == state_roll_over:
                    target_roll_degrees = 70
                    if abs(r - target_roll_degrees) < 10:
                        state = state_stabilize_roll
                        stabilize_start = self.get_sim_time()
                elif state == state_stabilize_roll:
                    # just give it a little time to sort it self out
                    if self.get_sim_time() - stabilize_start > 2:
                        state = state_hold
                        hold_start = self.get_sim_time()
                elif state == state_hold:
                    target_roll_degrees = 70
                    if self.get_sim_time() - hold_start > 10:
                        state = state_roll_back
                    if abs(r - target_roll_degrees) > 10:
                        raise NotAchievedException("Failed to hold attitude")
                elif state == state_roll_back:
                    target_roll_degrees = 0
                    if abs(r - target_roll_degrees) < 10:
                        state = state_done
                else:
                    raise ValueError("Unknown state %s" % str(state))

                self.progress("%s Roll: %f desired=%f" %
                              (state, r, target_roll_degrees))

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

    def fly_mission(self, filename):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        self.load_mission(filename)
        self.mavproxy.send('switch 1\n')  # auto mode
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 7, max_dist=60)
        self.wait_groundspeed(0, 0.5, timeout=60)
        self.mavproxy.expect("Auto disarmed")
        self.progress("Mission OK")

    def fly_flaps(self):
        """Test flaps functionality."""
        filename = os.path.join(testdir, "flaps.txt")
        self.context_push()
        ex = None
        try:
            flaps_ch = 5
            servo_ch = 5
            self.set_parameter("SERVO%u_FUNCTION" % servo_ch, 3) # flapsauto
            self.set_parameter("FLAP_IN_CHANNEL", flaps_ch)
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
            tstart = self.get_sim_time_cached()
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
            # self.wait_seconds(10)
            # self.mavproxy.send('switch 6\n')
            # self.wait_mode('MANUAL')
            # self.wait_seconds(10)

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
        while self.get_sim_time() - tstart < 10:
            x = self.mav.messages.get("CAMERA_FEEDBACK", None)
            if x is not None:
                break
            self.wait_heartbeat()
        self.set_rc(12, 1000)
        if x is None:
            raise NotAchievedException("No CAMERA_FEEDBACK message received")

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
            self.progress("Exception caught")
            ex = e
        self.context_pop()
        if ex is not None:
            raise ex

    def start_subtest(self, description):
        self.progress("-")
        self.progress("---------- %s  ----------" % description)
        self.progress("-")

    def run_subtest(self, desc, func):
        self.start_subtest(desc)
        func()

    def test_main_flight(self):

        self.change_mode('MANUAL')

        # grab home position:
        m = self.mav.recv_match(type='HOME_POSITION', blocking=True)
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
                         lambda: self.fly_mission(
                             os.path.join(testdir, "ap1.txt")))

    def set_rc_default(self):
        super(AutoTestPlane, self).set_rc_default()
        self.set_rc(3, 1000)
        self.set_rc(8, 1800)

    def default_mode(self):
        return "MANUAL"

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestPlane, self).tests()
        ret.extend([

            ("TestRCCamera",
             "Test RC Option - Camera Trigger",
             self.test_rc_option_camera_trigger),

            ("TestRCRelay", "Test Relay RC Channel Option", self.test_rc_relay),

            ("TestFlaps", "Flaps", self.fly_flaps),

            ("MainFlight",
             "Lots of things in one flight",
             self.test_main_flight),

            ("TestGripperMission",
             "Test Gripper mission items",
             self.test_gripper_mission),

            ("LogDownLoad",
             "Log download",
             lambda: self.log_download(
                 self.buildlogs_path("ArduPlane-log.bin"),
                 upload_logs=True))
        ])
        return ret
