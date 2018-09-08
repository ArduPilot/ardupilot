#!/usr/bin/env python

# Fly ArduPlane in SITL
from __future__ import print_function
import math
import os

import pexpect
from pymavlink import mavutil

from pysim import util

from common import AutoTest
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
            self.progress("Failed to maintain altitude")
            raise NotAchievedException()

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
            self.progress("Failed to maintain altitude")
            raise NotAchievedException()

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
        self.progress("Failed to attain level flight")
        raise NotAchievedException()

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
            self.progress("Failed to maintain altitude")
            raise NotAchievedException()

        return self.wait_level_flight()

    def wp_load(self, filename):
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')

    def fly_mission(self, filename):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        self.wp_load(filename)
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

            # check flaps are not deployed:
            self.set_rc(flaps_ch, flaps_ch_min)
            self.wait_servo_channel_value(servo_ch, servo_ch_min)
            # deploy the flaps:
            self.set_rc(flaps_ch, flaps_ch_max)
            tstart = self.get_sim_time()
            self.wait_servo_channel_value(servo_ch, servo_ch_max)
            tstop = self.get_sim_time_cached()
            delta_time = tstop - tstart
            delta_time_min = 0.5
            delta_time_max = 1.5
            if delta_time < delta_time_min or delta_time > delta_time_max:
                self.progress("Flaps Slew not working (%f seconds)" %
                              (delta_time,))
                raise NotAchievedException()
            # undeploy flaps:
            self.set_rc(flaps_ch, flaps_ch_min)
            self.wait_servo_channel_value(servo_ch, servo_ch_min)

            self.progress("Flying mission %s" % filename)
            self.wp_load(filename)
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
                if (time_delta >1 or
                    m.seq != last_seq):
                    dist = None
                    x = self.mav.messages.get("NAV_CONTROLLER_OUTPUT", None)
                    if x is not None:
                        dist = x.wp_dist
                    self.progress("MISSION_CURRENT.seq=%u (dist=%s)" %
                                  (m.seq,str(dist)))
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
        off = self.get_parameter("SIM_PIN_MASK")
        if off:
            raise PreconditionFailedException()
        self.set_rc(12, 2000)
        self.mav.wait_heartbeat()
        self.mav.wait_heartbeat()
        on = self.get_parameter("SIM_PIN_MASK")
        if not on:
            raise NotAchievedException()
        self.set_rc(12, 1000)
        self.mav.wait_heartbeat()
        self.mav.wait_heartbeat()
        off = self.get_parameter("SIM_PIN_MASK")
        if off:
            raise NotAchievedException()

    def test_rc_option_camera_trigger(self):
        '''test toggling channel 12 takes picture'''
        x = self.mav.messages.get("CAMERA_FEEDBACK", None)
        if x is not None:
            raise PreconditionFailedException()
        self.set_rc(12, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 10:
            x = self.mav.messages.get("CAMERA_FEEDBACK", None)
            if x is not None:
                break
            self.mav.wait_heartbeat()
        self.set_rc(12, 1000)
        if x is None:
            raise NotAchievedException()

    def autotest(self):
        """Autotest ArduPlane in SITL."""
        self.check_test_syntax(test_file=os.path.realpath(__file__))
        if not self.hasInit:
            self.init()

        self.fail_list = []
        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(3, 1000)
            self.set_rc(8, 1800)

            self.set_parameter("RC12_OPTION", 9)
            self.reboot_sitl() # needed for RC12_OPTION to take effect

            self.run_test("Test RC Option - Camera Trigger",
                          self.test_rc_option_camera_trigger)

            self.set_parameter("RC12_OPTION", 28)
            self.reboot_sitl() # needed for RC12_OPTION to take effect

            self.run_test("Test Relay RC Channel Option",
                          self.test_rc_relay)

            self.progress("Waiting for GPS fix")
            self.mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
            self.mav.wait_gps_fix()
            while self.mav.location().alt < 10:
                self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)
            self.wait_ready_to_arm()
            self.run_test("Arm features", self.test_arm_feature)

            self.run_test("Flaps", self.fly_flaps)

            self.mavproxy.send('switch 6\n')
            self.wait_mode('MANUAL')

            self.run_test("Takeoff", self.takeoff)

            self.run_test("Fly left circuit", self.fly_left_circuit)

            self.run_test("Left roll", lambda: self.axial_left_roll(1))

            self.run_test("Inside loop", self.inside_loop)

            self.run_test("Stablize test", self.test_stabilize)

            self.run_test("ACRO test", self.test_acro)

            self.run_test("FBWB test", self.test_FBWB)

            self.run_test("CRUISE test", lambda: self.test_FBWB(mode='CRUISE'))

            self.run_test("RTL test", self.fly_RTL)

            self.run_test("LOITER test", self.fly_LOITER)

            self.run_test("CIRCLE test", self.fly_CIRCLE)

            self.run_test("Mission test",
                          lambda: self.fly_mission(
                              os.path.join(testdir, "ap1.txt")))

            self.run_test("Log download",
                          lambda: self.log_download(
                              self.buildlogs_path("ArduPlane-log.bin")))

        except pexpect.TIMEOUT:
            self.progress("Failed with timeout")
            self.fail_list.append("timeout")

        self.close()

        if len(self.fail_list):
            self.progress("FAILED: %s" % self.fail_list)
            return False

        self.progress("Max set_rc_timeout=%s" % self.max_set_rc_timeout);

        return True
