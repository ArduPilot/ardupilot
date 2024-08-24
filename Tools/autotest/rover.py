'''
Drive Rover in SITL

AP_FLAKE8_CLEAN

'''

from __future__ import print_function

import copy
import math
import operator
import os
import sys
import time

import vehicle_test_suite

from pysim import util

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from pymavlink import mavextra
from pymavlink import mavutil

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

SITL_START_LOCATION = mavutil.location(40.071374969556928,
                                       -105.22978898137808,
                                       1583.702759,
                                       246)


class AutoTestRover(vehicle_test_suite.TestSuite):
    @staticmethod
    def get_not_armable_mode_list():
        return ["RTL", "SMART_RTL"]

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return ["FOLLOW"]

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return ["GUIDED", "LOITER", "STEERING", "AUTO"]

    @staticmethod
    def get_normal_armable_modes_list():
        return ["ACRO", "HOLD", "MANUAL"]

    def log_name(self):
        return "Rover"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduRover_Tests/" + name + "/"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_frame(self):
        return "rover"

    def default_speedup(self):
        return 30

    def is_rover(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_ROLL"))

    ##########################################################
    #   TESTS DRIVE
    ##########################################################
    # Drive a square in manual mode
    def DriveSquare(self, side=50):
        """Learn/Drive Square with Ch7 option"""

        self.progress("TEST SQUARE")
        self.set_parameters({
            "RC7_OPTION": 7,
            "RC9_OPTION": 58,
        })

        self.change_mode('MANUAL')

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.clear_wp(9)

        # first aim north
        self.progress("\nTurn right towards north")
        self.reach_heading_manual(10)
        # save bottom left corner of box as home AND waypoint
        self.progress("Save HOME")
        self.save_wp()

        self.progress("Save WP")
        self.save_wp()

        # pitch forward to fly north
        self.progress("\nGoing north %u meters" % side)
        self.reach_distance_manual(side)
        # save top left corner of square as waypoint
        self.progress("Save WP")
        self.save_wp()

        # roll right to fly east
        self.progress("\nGoing east %u meters" % side)
        self.reach_heading_manual(100)
        self.reach_distance_manual(side)
        # save top right corner of square as waypoint
        self.progress("Save WP")
        self.save_wp()

        # pitch back to fly south
        self.progress("\nGoing south %u meters" % side)
        self.reach_heading_manual(190)
        self.reach_distance_manual(side)
        # save bottom right corner of square as waypoint
        self.progress("Save WP")
        self.save_wp()

        # roll left to fly west
        self.progress("\nGoing west %u meters" % side)
        self.reach_heading_manual(280)
        self.reach_distance_manual(side)
        # save bottom left corner of square (should be near home) as waypoint
        self.progress("Save WP")
        self.save_wp()

        self.progress("Checking number of saved waypoints")
        mavproxy = self.start_mavproxy()
        num_wp = self.save_mission_to_file_using_mavproxy(
            mavproxy,
            os.path.join(testdir, "ch7_mission.txt"))
        self.stop_mavproxy(mavproxy)
        expected = 7 # home + 6 toggled in
        if num_wp != expected:
            raise NotAchievedException("Did not get %u waypoints; got %u" %
                                       (expected, num_wp))

        # TODO: actually drive the mission

        self.clear_wp(9)

        self.disarm_vehicle()

    def drive_left_circuit(self):
        """Drive a left circuit, 50m on a side."""
        self.change_mode('MANUAL')
        self.set_rc(3, 2000)

        self.progress("Driving left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(1, 1000)
            self.wait_heading(270 - (90*i), accuracy=10)
            self.set_rc(1, 1500)
            self.progress("Starting leg %u" % i)
            self.wait_distance(50, accuracy=7)
        self.set_rc(3, 1500)
        self.progress("Circuit complete")

    # def test_throttle_failsafe(self, home, distance_min=10, side=60,
    #                            timeout=300):
    #     """Fly east, Failsafe, return, land."""
    #
    #     self.mavproxy.send('switch 6\n')  # manual mode
    #     self.wait_mode('MANUAL')
    #     self.mavproxy.send("param set FS_ACTION 1\n")
    #
    #     # first aim east
    #     self.progress("turn east")
    #     if not self.reach_heading_manual(135):
    #         return False
    #
    #     # fly east 60 meters
    #     self.progress("# Going forward %u meters" % side)
    #     if not self.reach_distance_manual(side):
    #         return False
    #
    #     # pull throttle low
    #     self.progress("# Enter Failsafe")
    #     self.mavproxy.send('rc 3 900\n')
    #
    #     tstart = self.get_sim_time()
    #     success = False
    #     while self.get_sim_time() < tstart + timeout and not success:
    #         m = self.mav.recv_match(type='VFR_HUD', blocking=True)
    #         pos = self.mav.location()
    #         home_distance = self.get_distance(home, pos)
    #         self.progress("Alt: %u  HomeDistance: %.0f" %
    #                       (m.alt, home_distance))
    #         # check if we've reached home
    #         if home_distance <= distance_min:
    #             self.progress("RTL Complete")
    #             success = True
    #
    #     # reduce throttle
    #     self.mavproxy.send('rc 3 1500\n')
    #     self.mavproxy.expect('AP: Failsafe ended')
    #     self.mavproxy.send('switch 2\n')  # manual mode
    #     self.wait_heartbeat()
    #     self.wait_mode('MANUAL')
    #
    #     if success:
    #         self.progress("Reached failsafe home OK")
    #         return True
    #     else:
    #         self.progress("Failed to reach Home on failsafe RTL - "
    #         "timed out after %u seconds" % timeout)
    #         return False

    def Sprayer(self):
        """Test sprayer functionality."""
        rc_ch = 5
        pump_ch = 5
        spinner_ch = 6
        pump_ch_min = 1050
        pump_ch_trim = 1520
        pump_ch_max = 1950
        spinner_ch_min = 975
        spinner_ch_trim = 1510
        spinner_ch_max = 1975

        self.set_parameters({
            "SPRAY_ENABLE": 1,

            "SERVO%u_FUNCTION" % pump_ch: 22,
            "SERVO%u_MIN" % pump_ch: pump_ch_min,
            "SERVO%u_TRIM" % pump_ch: pump_ch_trim,
            "SERVO%u_MAX" % pump_ch: pump_ch_max,

            "SERVO%u_FUNCTION" % spinner_ch: 23,
            "SERVO%u_MIN" % spinner_ch: spinner_ch_min,
            "SERVO%u_TRIM" % spinner_ch: spinner_ch_trim,
            "SERVO%u_MAX" % spinner_ch: spinner_ch_max,

            "SIM_SPR_ENABLE": 1,
            "SIM_SPR_PUMP": pump_ch,
            "SIM_SPR_SPIN": spinner_ch,

            "RC%u_OPTION" % rc_ch: 15,
            "LOG_DISARMED": 1,
        })

        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.progress("test bootup state - it's zero-output!")
        self.wait_servo_channel_value(spinner_ch, 0)
        self.wait_servo_channel_value(pump_ch, 0)

        self.progress("Enable sprayer")
        self.set_rc(rc_ch, 2000)

        self.progress("Testing zero-speed state")
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing turning it off")
        self.set_rc(rc_ch, 1000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing turning it back on")
        self.set_rc(rc_ch, 2000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing speed-ramping")
        self.set_rc(3, 1700) # start driving forward

        # this is somewhat empirical...
        self.wait_servo_channel_value(pump_ch, 1695, timeout=60)

        self.progress("Turning it off again")
        self.set_rc(rc_ch, 1000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.start_subtest("Sprayer Mission")
        self.load_mission("sprayer-mission.txt")
        self.change_mode("AUTO")
#            self.send_debug_trap()
        self.progress("Waiting for sprayer to start")
        self.wait_servo_channel_value(pump_ch, 1250, timeout=60, comparator=operator.gt)
        self.progress("Waiting for sprayer to stop")
        self.wait_servo_channel_value(pump_ch, pump_ch_min, timeout=120)

        self.start_subtest("Checking mavlink commands")
        self.change_mode("MANUAL")
        self.progress("Starting Sprayer")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SPRAYER, p1=1)

        self.progress("Testing speed-ramping")
        self.set_rc(3, 1700) # start driving forward
        self.wait_servo_channel_value(pump_ch, 1690, timeout=60, comparator=operator.gt)
        self.start_subtest("Stopping Sprayer")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SPRAYER, p1=0)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)
        self.set_rc(3, 1000) # stop driving forward

        self.progress("Sprayer OK")
        self.disarm_vehicle()

    def DriveMaxRCIN(self, timeout=30):
        """Drive rover at max RC inputs"""
        self.progress("Testing max RC inputs")
        self.change_mode("MANUAL")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.set_rc(3, 2000)
        self.set_rc(1, 1000)

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            m = self.assert_receive_message('VFR_HUD')
            self.progress("Current speed: %f" % m.groundspeed)

        self.disarm_vehicle()

    #################################################
    # AUTOTEST ALL
    #################################################
    def drive_mission(self, filename, strict=True):
        """Drive a mission from a file."""
        self.progress("Driving mission %s" % filename)
        wp_count = self.load_mission(filename, strict=strict)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, wp_count-1, max_dist=5)
        self.wait_statustext("Mission Complete", timeout=600)
        self.disarm_vehicle()
        self.progress("Mission OK")

    def DriveMission(self):
        '''Drive Mission rover1.txt'''
        self.drive_mission("rover1.txt", strict=False)

    def GripperMission(self):
        '''Test Gripper Mission Items'''
        self.load_mission("rover-gripper-mission.txt")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        self.wait_statustext("Gripper Grabbed", timeout=60, check_context=True)
        self.wait_statustext("Gripper Released", timeout=60, check_context=True)
        self.wait_statustext("Mission Complete", timeout=60, check_context=True)
        self.disarm_vehicle()

    def _MAV_CMD_DO_SEND_BANNER(self, run_cmd):
        '''Get Banner'''
        self.context_push()
        self.context_collect('STATUSTEXT')
        run_cmd(mavutil.mavlink.MAV_CMD_DO_SEND_BANNER)
        self.wait_statustext("ArduRover", timeout=1, check_context=True)
        self.context_pop()

    def MAV_CMD_DO_SEND_BANNER(self):
        '''test MAV_CMD_DO_SEND_BANNER'''
        self._MAV_CMD_DO_SEND_BANNER(self.run_cmd)
        self._MAV_CMD_DO_SEND_BANNER(self.run_cmd_int)

    def drive_brake_get_stopping_distance(self, speed):
        '''measure our stopping distance'''

        self.context_push()

        # controller tends not to meet cruise speed (max of ~14 when 15
        # set), thus *1.2
        # at time of writing, the vehicle is only capable of 10m/s/s accel
        self.set_parameters({
            'CRUISE_SPEED': speed*1.2,
            'ATC_ACCEL_MAX': 15,
        })
        self.change_mode("STEERING")
        self.set_rc(3, 2000)
        self.wait_groundspeed(15, 100)
        initial = self.mav.location()
        initial_time = time.time()
        while time.time() - initial_time < 2:
            # wait for a position update from the autopilot
            start = self.mav.location()
            if start != initial:
                break
        self.set_rc(3, 1500)
        self.wait_groundspeed(0, 0.2)  # why do we not stop?!
        initial = self.mav.location()
        initial_time = time.time()
        while time.time() - initial_time < 2:
            # wait for a position update from the autopilot
            stop = self.mav.location()
            if stop != initial:
                break
        delta = self.get_distance(start, stop)

        self.context_pop()

        return delta

    def DriveBrake(self):
        '''Test braking'''
        self.set_parameters({
            'CRUISE_SPEED': 15,
            'ATC_BRAKE': 0,
        })

        self.arm_vehicle()

        distance_without_brakes = self.drive_brake_get_stopping_distance(15)

        # brakes on:
        self.set_parameter('ATC_BRAKE', 1)
        distance_with_brakes = self.drive_brake_get_stopping_distance(15)

        delta = distance_without_brakes - distance_with_brakes

        self.disarm_vehicle()

        if delta < distance_without_brakes * 0.05:  # 5% isn't asking for much
            raise NotAchievedException("""
Brakes have negligible effect (with=%0.2fm without=%0.2fm delta=%0.2fm)
""" %
                                       (distance_with_brakes,
                                        distance_without_brakes,
                                        delta))

        self.progress(
            "Brakes work (with=%0.2fm without=%0.2fm delta=%0.2fm)" %
            (distance_with_brakes, distance_without_brakes, delta))

    def drive_rtl_mission_max_distance_from_home(self):
        '''maximum distance allowed from home at end'''
        return 6.5

    def DriveRTL(self, timeout=120):
        '''Drive an RTL Mission'''
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.load_mission("rtl.txt")
        self.change_mode("AUTO")

        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("Didn't see wp 3")
            m = self.mav.recv_match(type='MISSION_CURRENT',
                                    blocking=True,
                                    timeout=1)
            self.progress("MISSION_CURRENT: %s" % str(m))
            if m.seq == 3:
                break

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

        # the EKF doesn't pull us down to 0 speed:
        self.wait_groundspeed(0, 0.5, timeout=600)

        # current Rover blows straight past the home position and ends
        # up ~6m past the home point.
        home_distance = self.distance_to_home()
        home_distance_min = 5.5
        home_distance_max = self.drive_rtl_mission_max_distance_from_home()
        if home_distance > home_distance_max:
            raise NotAchievedException(
                "Did not stop near home (%f metres distant (%f > want > %f))" %
                (home_distance, home_distance_min, home_distance_max))
        self.disarm_vehicle()
        self.progress("RTL Mission OK (%fm)" % home_distance)

    def RTL_SPEED(self, timeout=120):
        '''Test RTL_SPEED is honoured'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 300, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 0),
        ])

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.change_mode('AUTO')
        self.wait_current_waypoint(2, timeout=120)
        for speed in 1, 5.5, 1.5, 7.5:
            self.set_parameter("RTL_SPEED", speed)
            self.change_mode('RTL')
            self.wait_groundspeed(speed-0.1, speed+0.1, minimum_duration=10)
            self.change_mode('HOLD')
        self.do_RTL()
        self.disarm_vehicle()

    def AC_Avoidance(self):
        '''Test AC Avoidance switch'''
        self.load_fence("rover-fence-ac-avoid.txt")
        self.set_parameters({
            "FENCE_ENABLE": 0,
            "PRX1_TYPE": 10,
            "RC10_OPTION": 40, # proximity-enable
        })
        self.reboot_sitl()
        # start = self.mav.location()
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # first make sure we can breach the fence:
        self.set_rc(10, 1000)
        self.change_mode("ACRO")
        self.set_rc(3, 1550)
        self.wait_distance_to_home(25, 100000, timeout=60)
        self.change_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        # now enable avoidance and make sure we can't:
        self.set_rc(10, 2000)
        self.change_mode("ACRO")
        self.wait_groundspeed(0, 0.7, timeout=60)
        # watch for speed zero
        self.wait_groundspeed(0, 0.2, timeout=120)
        self.disarm_vehicle()

    def ServoRelayEvents(self):
        '''Test ServoRelayEvents'''
        for method in self.run_cmd, self.run_cmd_int:
            self.context_push()

            self.set_parameters({
                "RELAY1_FUNCTION": 1, # Enable relay 1 as a standard relay pin
                "RELAY2_FUNCTION": 1, # Enable relay 2 as a standard relay pin
            })
            self.reboot_sitl() # Needed for relay functions to take effect

            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=0)
            off = self.get_parameter("SIM_PIN_MASK")
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=1)
            on = self.get_parameter("SIM_PIN_MASK")
            if on == off:
                raise NotAchievedException(
                    "Pin mask unchanged after relay cmd")
            self.progress("Pin mask changed after relay command")
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=0)

            self.set_message_rate_hz("RELAY_STATUS", 10)

            # default configuration for relays in sim have one relay:
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 0,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 1,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=1, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 3,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=0)
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=1, p2=0)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 0,
            })

            # add another relay and ensure that it changes the "present field"
            self.set_parameters({
                "RELAY6_FUNCTION": 1, # Enable relay 6 as a standard relay pin
                "RELAY6_PIN": 14, # Set pin number
            })
            self.reboot_sitl() # Needed for relay function to take effect
            self.set_message_rate_hz("RELAY_STATUS", 10) # Need to re-request the message since reboot

            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 0,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=5, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 32,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 33,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=5, p2=0)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 1,
            })

            self.start_subtest("test MAV_CMD_DO_REPEAT_RELAY")
            self.context_push()
            self.set_parameter("SIM_SPEEDUP", 1)
            method(
                mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY,
                p1=0,  # servo 1
                p2=5,  # 5 times
                p3=0.5,  # 1 second between being on
            )
            for value in 0, 1, 0, 1, 0, 1, 0, 1:
                self.wait_message_field_values('RELAY_STATUS', {
                    "on": value,
                })
            self.context_pop()
            self.delay_sim_time(3)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "on": 1,  # back to initial state
            })
            self.context_pop()

            self.start_subtest("test MAV_CMD_DO_SET_SERVO")
            for value in 1678, 2300, 0:
                method(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=13, p2=value)
                self.wait_servo_channel_value(13, value)

            self.start_subtest("test MAV_CMD_DO_REPEAT_SERVO")

            self.context_push()
            self.set_parameter("SIM_SPEEDUP", 1)
            trim = self.get_parameter("SERVO13_TRIM")
            value = 2000
            method(
                mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO,
                p1=12,  # servo12
                p2=value,  # pwm
                p3=5,  # count
                p4=0.5,  # cycle time (1 second between high and high)
            )
            for value in trim, value, trim, value, trim, value, trim, value:
                self.wait_servo_channel_value(12, value)
            self.context_pop()

        self.set_message_rate_hz("RELAY_STATUS", 0)

    def MAVProxy_SetModeUsingSwitch(self):
        """Set modes via mavproxy switch"""
        port = self.sitl_rcin_port(offset=1)
        self.customise_SITL_commandline([
            "--rc-in-port", str(port),
        ])
        ex = None
        try:
            self.load_mission(self.arming_test_mission())
            self.wait_ready_to_arm()
            fnoo = [(1, 'MANUAL'),
                    (2, 'MANUAL'),
                    (3, 'RTL'),
                    (4, 'AUTO'),
                    (5, 'AUTO'),  # non-existant mode, should stay in RTL
                    (6, 'MANUAL')]
            mavproxy = self.start_mavproxy(sitl_rcin_port=port)
            for (num, expected) in fnoo:
                mavproxy.send('switch %u\n' % num)
                self.wait_mode(expected)
            self.stop_mavproxy(mavproxy)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        # if we don't put things back ourselves then the test cleanup
        # doesn't go well as we can't set the RC defaults correctly:
        self.customise_SITL_commandline([
        ])

        if ex is not None:
            raise ex

    def MAVProxy_SetModeUsingMode(self):
        '''Set modes via mavproxy mode command'''
        fnoo = [(1, 'ACRO'),
                (3, 'STEERING'),
                (4, 'HOLD'),
                ]
        mavproxy = self.start_mavproxy()
        for (num, expected) in fnoo:
            mavproxy.send('mode manual\n')
            self.wait_mode("MANUAL")
            mavproxy.send('mode %u\n' % num)
            self.wait_mode(expected)
            mavproxy.send('mode manual\n')
            self.wait_mode("MANUAL")
            mavproxy.send('mode %s\n' % expected)
            self.wait_mode(expected)
        self.stop_mavproxy(mavproxy)

    def ModeSwitch(self):
        ''''Set modes via modeswitch'''
        self.set_parameter("MODE_CH", 8)
        self.set_rc(8, 1000)
        # mavutil.mavlink.ROVER_MODE_HOLD:
        self.set_parameter("MODE6", 4)
        # mavutil.mavlink.ROVER_MODE_ACRO
        self.set_parameter("MODE5", 1)
        self.set_rc(8, 1800) # PWM for mode6
        self.wait_mode("HOLD")
        self.set_rc(8, 1700) # PWM for mode5
        self.wait_mode("ACRO")
        self.set_rc(8, 1800) # PWM for mode6
        self.wait_mode("HOLD")
        self.set_rc(8, 1700) # PWM for mode5
        self.wait_mode("ACRO")

    def AuxModeSwitch(self):
        '''Set modes via auxswitches'''
        # from mavproxy_rc.py
        mapping = [0, 1165, 1295, 1425, 1555, 1685, 1815]
        self.set_parameter("MODE1", 1)  # acro
        self.set_rc(8, mapping[1])
        self.wait_mode('ACRO')

        self.set_rc(9, 1000)
        self.set_rc(10, 1000)
        self.set_parameters({
            "RC9_OPTION": 53, # steering
            "RC10_OPTION": 54, # hold
        })
        self.set_rc(9, 1900)
        self.wait_mode("STEERING")
        self.set_rc(10, 1900)
        self.wait_mode("HOLD")

        # reset both switches - should go back to ACRO
        self.set_rc(9, 1000)
        self.set_rc(10, 1000)
        self.wait_mode("ACRO")

        self.set_rc(9, 1900)
        self.wait_mode("STEERING")
        self.set_rc(10, 1900)
        self.wait_mode("HOLD")

        self.set_rc(10, 1000) # this re-polls the mode switch
        self.wait_mode("ACRO")
        self.set_rc(9, 1000)

    def RCOverridesCancel(self):
        '''Test RC overrides Cancel'''
        self.set_parameter("SYSID_MYGCS", self.mav.source_system)
        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.zero_throttle()
        self.arm_vehicle()
        # start moving forward a little:
        normal_rc_throttle = 1700
        throttle_override = 1900

        self.progress("Establishing baseline RC input")
        self.set_rc(3, normal_rc_throttle)
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not get rc change")
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            if m.chan3_raw == normal_rc_throttle:
                break

        self.progress("Set override with RC_CHANNELS_OVERRIDE")
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not override")
            self.progress("Sending throttle of %u" % (throttle_override,))
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                throttle_override, # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw

            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            self.progress("chan3=%f want=%f" % (m.chan3_raw, throttle_override))
            if m.chan3_raw == throttle_override:
                break

        self.progress("disabling override and making sure we revert to RC input in good time")
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 0.5:
                raise AutoTestTimeoutException("Did not cancel override")
            self.progress("Sending cancel of throttle override")
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                0,     # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw
            self.do_timesync_roundtrip()
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            self.progress("chan3=%f want=%f" % (m.chan3_raw, normal_rc_throttle))
            if m.chan3_raw == normal_rc_throttle:
                break
        self.disarm_vehicle()

    def RCOverrides(self):
        '''Test RC overrides'''
        self.set_parameter("SYSID_MYGCS", self.mav.source_system)
        self.set_parameter("RC12_OPTION", 46)
        self.reboot_sitl()

        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.set_rc(3, 1500)  # throttle at zero
        self.arm_vehicle()
        # start moving forward a little:
        normal_rc_throttle = 1700
        self.set_rc(3, normal_rc_throttle)
        self.wait_groundspeed(5, 100)

        # allow overrides:
        self.set_rc(12, 2000)

        # now override to stop:
        throttle_override = 1500

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach speed")
            self.progress("Sending throttle of %u" % (throttle_override,))
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                throttle_override, # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw

            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            want_speed = 2.0
            self.progress("Speed=%f want=<%f" % (m.groundspeed, want_speed))
            if m.groundspeed < want_speed:
                break

        # now override to stop - but set the switch on the RC
        # transmitter to deny overrides; this should send the
        # speed back up to 5 metres/second:
        self.set_rc(12, 1000)

        throttle_override = 1500
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not speed back up")
            self.progress("Sending throttle of %u" % (throttle_override,))
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                throttle_override, # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw

            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            want_speed = 5.0
            self.progress("Speed=%f want=>%f" % (m.groundspeed, want_speed))

            if m.groundspeed > want_speed:
                break

        # re-enable RC overrides
        self.set_rc(12, 2000)

        # check we revert to normal RC inputs when gcs overrides cease:
        self.progress("Waiting for RC to revert to normal RC input")
        self.wait_rc_channel_value(3, normal_rc_throttle, timeout=10)

        self.start_subtest("Check override time of zero disables overrides")
        old = self.get_parameter("RC_OVERRIDE_TIME")
        ch = 2
        self.set_rc(ch, 1000)
        channels = [65535] * 18
        ch_override_value = 1700
        channels[ch-1] = ch_override_value
        channels[7] = 1234 # that's channel 8!
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        # long timeout required here as we may have sent a lot of
        # things via MAVProxy...
        self.wait_rc_channel_value(ch, ch_override_value, timeout=30)
        self.set_parameter("RC_OVERRIDE_TIME", 0)
        self.wait_rc_channel_value(ch, 1000)
        self.set_parameter("RC_OVERRIDE_TIME", old)
        self.wait_rc_channel_value(ch, ch_override_value)

        ch_override_value = 1720
        channels[ch-1] = ch_override_value
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
        self.set_parameter("RC_OVERRIDE_TIME", 0)
        self.wait_rc_channel_value(ch, 1000)
        self.set_parameter("RC_OVERRIDE_TIME", old)

        self.progress("Ensuring timeout works")
        self.wait_rc_channel_value(ch, 1000, timeout=5)
        self.delay_sim_time(10)

        self.set_parameter("RC_OVERRIDE_TIME", 10)
        self.progress("Sending override message")

        ch_override_value = 1730
        channels[ch-1] = ch_override_value
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
        tstart = self.get_sim_time()
        self.progress("Waiting for channel to revert to 1000 in ~10s")
        self.wait_rc_channel_value(ch, 1000, timeout=15)
        delta = self.get_sim_time() - tstart
        if delta > 12:
            raise NotAchievedException("Took too long to revert RC channel value (delta=%f)" % delta)
        min_delta = 9
        if delta < min_delta:
            raise NotAchievedException("Didn't take long enough to revert RC channel value (delta=%f want>=%f)" %
                                       (delta, min_delta))
        self.progress("Disabling RC override timeout")
        self.set_parameter("RC_OVERRIDE_TIME", -1)
        ch_override_value = 1740
        channels[ch-1] = ch_override_value
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
        tstart = self.get_sim_time()
        while True:
            # warning: this is get_sim_time() and can slurp messages on you!
            delta = self.get_sim_time() - tstart
            if delta > 20:
                break
            m = self.assert_receive_message('RC_CHANNELS', timeout=1)
            channel_field = "chan%u_raw" % ch
            m_value = getattr(m, channel_field)
            if m_value != ch_override_value:
                raise NotAchievedException("Value reverted after %f seconds when it should not have (got=%u) (want=%u)" % (delta, m_value, ch_override_value))  # noqa
        self.set_parameter("RC_OVERRIDE_TIME", old)

        self.delay_sim_time(10)

        self.start_subtest("Checking higher-channel semantics")
        self.context_push()
        self.set_parameter("RC_OVERRIDE_TIME", 30)

        ch = 11
        rc_value = 1010
        self.set_rc(ch, rc_value)

        channels = [65535] * 18
        ch_override_value = 1234
        channels[ch-1] = ch_override_value
        self.progress("Sending override message ch%u=%u" % (ch, ch_override_value))
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.progress("Wait for override value")
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)

        self.progress("Sending return-to-RC-input value")
        channels[ch-1] = 65534
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, rc_value, timeout=10)

        channels[ch-1] = ch_override_value
        self.progress("Sending override message ch%u=%u" % (ch, ch_override_value))
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.progress("Wait for override value")
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)

        # make we keep the override vaue for at least 10 seconds:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                break
            # try both ignore values:
            ignore_value = 0
            if self.get_sim_time_cached() - tstart > 5:
                ignore_value = 65535
            self.progress("Sending ignore value %u" % ignore_value)
            channels[ch-1] = ignore_value
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                *channels
            )
            if self.get_rc_channel_value(ch) != ch_override_value:
                raise NotAchievedException("Did not maintain value")

        self.context_pop()

        self.end_subtest("Checking higher-channel semantics")

        self.disarm_vehicle()

    def MANUAL_CONTROL(self):
        '''Test mavlink MANUAL_CONTROL'''
        self.set_parameters({
            "SYSID_MYGCS": self.mav.source_system,
            "RC12_OPTION": 46, # enable/disable rc overrides
        })
        self.reboot_sitl()

        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("start moving forward a little")
        normal_rc_throttle = 1700
        self.set_rc(3, normal_rc_throttle)
        self.wait_groundspeed(5, 100)

        self.progress("allow overrides")
        self.set_rc(12, 2000)

        self.progress("now override to stop")
        throttle_override_normalized = 0
        expected_throttle = 0 # in VFR_HUD

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach speed")
            self.progress("Sending normalized throttle of %d" % (throttle_override_normalized,))
            self.mav.mav.manual_control_send(
                1, # target system
                32767, # x (pitch)
                32767, # y (roll)
                throttle_override_normalized, # z (thrust)
                32767, # r (yaw)
                0) # button mask

            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            want_speed = 2.0
            self.progress("Speed=%f want=<%f  throttle=%u want=%u" %
                          (m.groundspeed, want_speed, m.throttle, expected_throttle))
            if m.groundspeed < want_speed and m.throttle == expected_throttle:
                break

        self.progress("now override to stop - but set the switch on the RC transmitter to deny overrides; this should send the speed back up to 5 metres/second")  # noqa
        self.set_rc(12, 1000)

        throttle_override_normalized = 500
        expected_throttle = 36 # in VFR_HUD, corresponding to normal_rc_throttle adjusted for channel min/max

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not stop")
            self.progress("Sending normalized throttle of %u" % (throttle_override_normalized,))
            self.mav.mav.manual_control_send(
                1, # target system
                32767, # x (pitch)
                32767, # y (roll)
                throttle_override_normalized, # z (thrust)
                32767, # r (yaw)
                0) # button mask

            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            want_speed = 5.0

            self.progress("Speed=%f want=>%f  throttle=%u want=%u" %
                          (m.groundspeed, want_speed, m.throttle, expected_throttle))
            if m.groundspeed > want_speed and m.throttle == expected_throttle:
                break

        # re-enable RC overrides
        self.set_rc(12, 2000)

        # check we revert to normal RC inputs when gcs overrides cease:
        self.progress("Waiting for RC to revert to normal RC input")
        self.wait_rc_channel_value(3, normal_rc_throttle, timeout=10)

        self.disarm_vehicle()

    def CameraMission(self):
        '''Test Camera Mission Items'''
        self.set_parameter("CAM1_TYPE", 1) # Camera with servo trigger
        self.reboot_sitl() # needed for CAM1_TYPE to take effect
        self.load_mission("rover-camera-mission.txt")
        self.wait_ready_to_arm()
        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        prev_cf = None
        while True:
            cf = self.mav.recv_match(type='CAMERA_FEEDBACK', blocking=True)
            if prev_cf is None:
                prev_cf = cf
                continue
            dist_travelled = self.get_distance_int(prev_cf, cf)
            prev_cf = cf
            mc = self.mav.messages.get("MISSION_CURRENT", None)
            if mc is None:
                continue
            elif mc.seq == 2:
                expected_distance = 2
            elif mc.seq == 4:
                expected_distance = 5
            elif mc.seq == 5:
                break
            else:
                continue
            self.progress("Expected distance %f got %f" %
                          (expected_distance, dist_travelled))
            error = abs(expected_distance - dist_travelled)
            # Rover moves at ~5m/s; we appear to do something at
            # 5Hz, so we do see over a meter of error!
            max_error = 1.5
            if error > max_error:
                raise NotAchievedException("Camera distance error: %f (%f)" %
                                           (error, max_error))

        self.disarm_vehicle()

    def DO_SET_MODE(self):
        '''Set mode via MAV_COMMAND_DO_SET_MODE'''
        self.do_set_mode_via_command_long("HOLD")
        self.do_set_mode_via_command_long("MANUAL")
        self.do_set_mode_via_command_int("HOLD")
        self.do_set_mode_via_command_int("MANUAL")

    def RoverInitialMode(self):
        '''test INITIAL_MODE parameter works'''
        # from mavproxy_rc.py
        self.wait_ready_to_arm()
        mapping = [0, 1165, 1295, 1425, 1555, 1685, 1815]
        mode_ch = 8
        throttle_ch = 3
        self.set_parameter('MODE5', 3)
        self.set_rc(mode_ch, mapping[5])
        self.wait_mode('STEERING')
        self.set_rc(mode_ch, mapping[6])
        self.wait_mode('MANUAL')
        self.set_parameter("INITIAL_MODE", 1) # acro
        # stop the vehicle polling the mode switch at boot:
        self.set_parameter('FS_ACTION', 0)  # do nothing when radio fails
        self.set_rc(throttle_ch, 900)  # RC fail
        self.reboot_sitl()
        self.wait_mode(1)
        self.progress("Make sure we stay in this mode")
        self.delay_sim_time(5)
        self.wait_mode(1)
        # now change modes with a switch:
        self.set_rc(throttle_ch, 1100)
        self.delay_sim_time(3)
        self.set_rc(mode_ch, mapping[5])
        self.wait_mode('STEERING')

    def MAVProxy_DO_SET_MODE(self):
        '''Set mode using MAVProxy commandline DO_SET_MODE'''
        mavproxy = self.start_mavproxy()
        self.mavproxy_do_set_mode_via_command_long(mavproxy, "HOLD")
        self.mavproxy_do_set_mode_via_command_long(mavproxy, "MANUAL")
        self.stop_mavproxy(mavproxy)

    def SYSID_ENFORCE(self):
        '''Test enforcement of SYSID_MYGCS'''
        '''Run the same arming code with correct then incorrect SYSID'''

        if self.mav.source_system != self.mav.mav.srcSystem:
            raise PreconditionFailedException("Expected mav.source_system and mav.srcSystem to match")

        self.context_push()
        old_srcSystem = self.mav.mav.srcSystem
        ex = None
        try:
            self.set_parameter("SYSID_MYGCS", self.mav.source_system)
            self.set_parameter("SYSID_ENFORCE", 1, add_to_context=False)

            self.change_mode('MANUAL')

            self.progress("make sure I can arm ATM")
            self.wait_ready_to_arm()
            self.arm_vehicle(timeout=5)
            self.disarm_vehicle()

            self.do_timesync_roundtrip()

            # should not be able to arm from a system id which is not MY_SYSID
            self.progress("Attempting to arm vehicle from bad system-id")
            success = None
            try:
                # temporarily set a different system ID than normal:
                self.mav.mav.srcSystem = 72
                self.arm_vehicle(timeout=5)
                self.disarm_vehicle()
                success = False
            except AutoTestTimeoutException:
                success = True
            self.mav.mav.srcSystem = old_srcSystem
            if not success:
                raise NotAchievedException("Managed to arm with SYSID_ENFORCE set")

            # should be able to arm from the vehicle's own components:
            self.progress("Attempting to arm vehicle from vehicle component")
            comp_arm_exception = None
            try:
                self.mav.mav.srcSystem = 1
                self.arm_vehicle(timeout=5)
                self.disarm_vehicle()
            except Exception as e:
                comp_arm_exception = e
            self.mav.mav.srcSystem = old_srcSystem
            if comp_arm_exception is not None:
                raise comp_arm_exception

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.mav.mav.srcSystem = old_srcSystem
        self.set_parameter("SYSID_ENFORCE", 0, add_to_context=False)
        self.context_pop()
        if ex is not None:
            raise ex

    def Rally(self):
        '''Test Rally Points'''
        self.load_rally_using_mavproxy("rover-test-rally.txt")
        self.assert_parameter_value('RALLY_TOTAL', 2)

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # calculate early to avoid round-trips while vehicle is moving:
        accuracy = self.get_parameter("WP_RADIUS")

        self.reach_heading_manual(10)
        self.reach_distance_manual(50)

        self.change_mode("RTL")

        # location copied in from rover-test-rally.txt:
        loc = mavutil.location(40.071553,
                               -105.229401,
                               1583,
                               0)

        self.wait_location(loc, accuracy=accuracy, minimum_duration=10, timeout=45)
        self.disarm_vehicle()

    def fence_with_bad_frame(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_zero_vertex_count(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_wrong_vertex_count(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                2, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_multiple_return_points(self, target_system=1, target_component=1):
        return [
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
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_invalid_latlon(self, target_system=1, target_component=1):
        return [
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
                int(100 * 1e7), # bad latitude. bad.
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_multiple_return_points_with_bad_sequence_numbers(self, target_system=1, target_component=1):
        return [
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
                int(1.0 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
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
                int(2.0 * 1e7), # latitude
                int(2.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_which_exceeds_storage_space(self, target_system=1, target_component=1):
        ret = []
        for i in range(0, 60):
            ret.append(self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                i, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                10, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            )
        return ret

    def fences_which_should_not_upload(self, target_system=1, target_component=1):
        return [
            ("Bad Frame", self.fence_with_bad_frame(
                target_system=target_system,
                target_component=target_component)),
            ("Zero Vertex Count", self.fence_with_zero_vertex_count(
                target_system=target_system,
                target_component=target_component)),
            ("Wrong Vertex Count", self.fence_with_wrong_vertex_count(
                target_system=target_system,
                target_component=target_component)),
            ("Multiple return points", self.fence_with_multiple_return_points(
                target_system=target_system,
                target_component=target_component)),
            ("Invalid lat/lon", self.fence_with_invalid_latlon(
                target_system=target_system,
                target_component=target_component)),
            ("Multiple Return points with bad sequence numbers",
                 self.fence_with_multiple_return_points_with_bad_sequence_numbers(  # noqa
                     target_system=target_system,
                     target_component=target_component)),
            ("Fence which exceeds storage space",
             self.fence_which_exceeds_storage_space(
                 target_system=target_system,
                 target_component=target_component)),
        ]

    def fence_with_single_return_point(self, target_system=1, target_component=1):
        return [
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
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_single_return_point_and_5_vertex_inclusion(self,
                                                              target_system=1,
                                                              target_component=1):
        return [
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
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                4, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0002 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                5, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0003 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_many_exclusion_circles(self, count=50, target_system=1, target_component=1):
        ret = []
        for i in range(0, count):
            lat_deg = 1.0003 + count/10
            lng_deg = 1.0002 + count/10
            item = self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                i, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                count, # p1
                0, # p2
                0, # p3
                0, # p4
                int(lat_deg * 1e7), # latitude
                int(lng_deg * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
            ret.append(item)
        return ret

    def fence_with_many_exclusion_polyfences(self, target_system=1, target_component=1):
        ret = []
        seq = 0
        for fencenum in range(0, 4):
            pointcount = fencenum + 6
            for p in range(0, pointcount):
                lat_deg = 1.0003 + p/10 + fencenum/100
                lng_deg = 1.0002 + p/10 + fencenum/100
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    0, # current
                    0, # autocontinue
                    pointcount, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(lat_deg * 1e7), # latitude
                    int(lng_deg * 1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                ret.append(item)
                seq += 1
        return ret

    def fences_which_should_upload(self, target_system=1, target_component=1):
        return [
            ("Single Return Point",
             self.fence_with_single_return_point(
                 target_system=target_system,
                 target_component=target_component)),
            ("Return and 5-vertex-inclusion",
             self.fence_with_single_return_point_and_5_vertex_inclusion(
                 target_system=target_system,
                 target_component=target_component)),
            ("Many exclusion circles",
             self.fence_with_many_exclusion_circles(
                 target_system=target_system,
                 target_component=target_component)),
            ("Many exclusion polyfences",
             self.fence_with_many_exclusion_polyfences(
                 target_system=target_system,
                 target_component=target_component)),
            ("Empty fence", []),
        ]

    def assert_fence_does_not_upload(self, fence, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        # upload single item using mission item protocol:
        upload_failed = False
        try:
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                               fence)
        except NotAchievedException:
            # TODO: make sure we failed for correct reason
            upload_failed = True
        if not upload_failed:
            raise NotAchievedException("Uploaded fence when should not be possible")
        self.progress("Fence rightfully bounced")

    def send_fencepoint_expect_statustext(self,
                                          offset,
                                          count,
                                          lat,
                                          lng,
                                          statustext_fragment,
                                          target_system=1,
                                          target_component=1,
                                          timeout=10):
        self.mav.mav.fence_point_send(target_system,
                                      target_component,
                                      offset,
                                      count,
                                      lat,
                                      lng)

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get error message back")
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            self.progress("statustext: %s (want='%s')" %
                          (str(m), statustext_fragment))
            if m is None:
                continue
            if statustext_fragment in m.text:
                break

    def GCSFailsafe(self, side=60, timeout=360):
        """Test GCS Failsafe"""
        try:
            self.test_gcs_failsafe(side=side, timeout=timeout)
        except Exception as ex:
            self.setGCSfailsafe(0)
            self.set_parameter('FS_ACTION', 0)
            self.disarm_vehicle(force=True)
            self.reboot_sitl()
            raise ex

    def test_gcs_failsafe(self, side=60, timeout=360):
        self.set_parameter("SYSID_MYGCS", self.mav.source_system)
        self.set_parameter("FS_ACTION", 1)
        self.set_parameter("FS_THR_ENABLE", 0)  # disable radio FS as it inhibt GCS one's

        def go_somewhere():
            self.change_mode("MANUAL")
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_rc(3, 2000)
            self.delay_sim_time(5)
            self.set_rc(3, 1500)
        # Trigger telemetry loss with failsafe disabled. Verify no action taken.
        self.start_subtest("GCS failsafe disabled test: FS_GCS_ENABLE=0 should take no failsafe action")
        self.setGCSfailsafe(0)
        go_somewhere()
        self.set_heartbeat_rate(0)
        self.delay_sim_time(5)
        self.wait_mode("MANUAL")
        self.set_heartbeat_rate(self.speedup)
        self.delay_sim_time(5)
        self.wait_mode("MANUAL")
        self.end_subtest("Completed GCS failsafe disabled test")

        # Trigger telemetry loss with failsafe enabled. Verify
        # failsafe triggers to RTL. Restore telemetry, verify failsafe
        # clears, and change modes.
        # TODO not implemented
        # self.start_subtest("GCS failsafe recovery test: FS_GCS_ENABLE=1")
        # self.setGCSfailsafe(1)
        # self.set_heartbeat_rate(0)
        # self.wait_mode("RTL")
        # self.set_heartbeat_rate(self.speedup)
        # self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        # self.change_mode("MANUAL")
        # self.end_subtest("Completed GCS failsafe recovery test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe RTL with no options test: FS_GCS_ENABLE=1")
        self.setGCSfailsafe(1)
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe RTL")

        # Trigger telemetry loss with an invalid failsafe value. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe invalid value with no options test: FS_GCS_ENABLE=99")
        self.setGCSfailsafe(99)
        go_somewhere()
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe invalid value")

        self.start_subtest("Testing continue in auto mission")
        self.disarm_vehicle()
        self.setGCSfailsafe(2)
        self.load_mission("test_arming.txt")
        self.change_mode("AUTO")
        self.delay_sim_time(5)
        self.set_heartbeat_rate(0)
        self.wait_statustext("Failsafe - Continuing Auto Mode", timeout=60)
        self.delay_sim_time(5)
        self.wait_mode("AUTO")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.start_subtest("GCS failsafe RTL with no options test: FS_GCS_ENABLE=1 and FS_GCS_TIMEOUT=10")
        self.setGCSfailsafe(1)
        old_gcs_timeout = self.get_parameter("FS_GCS_TIMEOUT")
        new_gcs_timeout = old_gcs_timeout * 2
        self.set_parameter("FS_GCS_TIMEOUT", new_gcs_timeout)
        go_somewhere()
        self.set_heartbeat_rate(0)
        self.delay_sim_time(old_gcs_timeout + (new_gcs_timeout - old_gcs_timeout) / 2)
        self.assert_mode("MANUAL")
        self.wait_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.disarm_vehicle()
        self.end_subtest("Completed GCS failsafe RTL")

        self.setGCSfailsafe(0)
        self.progress("All GCS failsafe tests complete")

    def test_gcs_fence_centroid(self, target_system=1, target_component=1):
        self.start_subtest("Ensuring if we don't have a centroid it gets calculated")
        items = self.test_gcs_fence_need_centroid(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        centroid = self.get_fence_point(0)
        want_lat = 1.0001
        want_lng = 1.00005
        if abs(centroid.lat - want_lat) > 0.000001:
            raise NotAchievedException("Centroid lat not as expected (want=%f got=%f)" % (want_lat, centroid.lat))
        if abs(centroid.lng - want_lng) > 0.000001:
            raise NotAchievedException("Centroid lng not as expected (want=%f got=%f)" % (want_lng, centroid.lng))

    def test_gcs_fence_update_fencepoint(self, target_system=1, target_component=1):
        self.start_subtest("Ensuring we can move a fencepoint")
        items = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
#        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        item_seq = 2
        item = items[item_seq]
        print("item is (%s)" % str(item))
        self.progress("original x=%d" % item.x)
        item.x += int(0.1 * 1e7)
        self.progress("new x=%d" % item.x)
        self.progress("try to overwrite item %u" % item_seq)
        self.mav.mav.mission_write_partial_list_send(
            target_system,
            target_component,
            item_seq,
            item_seq,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, item_seq)
        item.pack(self.mav.mav)
        self.mav.mav.send(item)
        self.progress("Answered request for fence point %u" % item_seq)

        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        downloaded_items2 = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if downloaded_items2[item_seq].x != item.x:
            raise NotAchievedException("Item did not update")
        self.check_fence_items_same([items[0], items[1], item, items[3]], downloaded_items2)

    def test_gcs_fence_boring_triangle(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.00015 * 1e7), # latitude
                int(1.00015 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ])

    def test_gcs_fence_need_centroid(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ])

    def click_location_from_item(self, mavproxy, item):
        mavproxy.send("click %f %f\n" % (item.x*1e-7, item.y*1e-7))

    def test_gcs_fence_via_mavproxy(self, target_system=1, target_component=1):
        self.start_subtest("Fence via MAVProxy")
        if not self.mavproxy_can_do_mision_item_protocols():
            return
        mavproxy = self.start_mavproxy()
        self.start_subsubtest("fence addcircle")
        self.clear_fence_using_mavproxy(mavproxy)
        self.delay_sim_time(1)
        radius = 20
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            0, # current
            0, # autocontinue
            radius, # p1
            0, # p2
            0, # p3
            0, # p4
            int(1.0017 * 1e7), # latitude
            int(1.0017 * 1e7), # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("item is (%s)" % str(item))
        self.click_location_from_item(mavproxy, item)
        mavproxy.send("fence addcircle inc %u\n" % radius)
        self.delay_sim_time(1)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("downloaded items: %s" % str(downloaded_items))
        self.check_fence_items_same([item], downloaded_items)

        radius_exc = 57.3
        item2 = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            radius_exc, # p1
            0, # p2
            0, # p3
            0, # p4
            int(1.0017 * 1e7), # latitude
            int(1.0017 * 1e7), # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.click_location_from_item(mavproxy, item2)
        mavproxy.send("fence addcircle exc %f\n" % radius_exc)
        self.delay_sim_time(1)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("downloaded items: %s" % str(downloaded_items))
        self.check_fence_items_same([item, item2], downloaded_items)
        self.end_subsubtest("fence addcircle")

        self.start_subsubtest("fence addpoly")
        self.clear_fence_using_mavproxy(mavproxy)
        self.delay_sim_time(1)
        pointcount = 7
        mavproxy.send("fence addpoly inc 20 %u 37.2\n" % pointcount) # radius, pointcount, rotaiton
        self.delay_sim_time(5)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != pointcount:
            raise NotAchievedException("Did not get expected number of points returned (want=%u got=%u)" %
                                       (pointcount, len(downloaded_items)))
        self.end_subsubtest("fence addpoly")

        self.start_subsubtest("fence movepolypoint")
        self.clear_fence_using_mavproxy(mavproxy)
        self.delay_sim_time(1)
        triangle = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           triangle)
        mavproxy.send("fence list\n")
        self.delay_sim_time(1)
        triangle[2].x += 500
        triangle[2].y += 700
        self.click_location_from_item(mavproxy, triangle[2])
        mavproxy.send("fence movepolypoint 0 2\n")
        self.delay_sim_time(10)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.check_fence_items_same(triangle, downloaded_items)
        self.end_subsubtest("fence movepolypoint")

        self.start_subsubtest("fence enable and disable")
        mavproxy.send("fence enable\n")
        mavproxy.expect("fence enabled")
        mavproxy.send("fence disable\n")
        mavproxy.expect("fence disabled")
        self.end_subsubtest("fence enable and disable")

        self.stop_mavproxy(mavproxy)

#        MANUAL> usage: fence <addcircle|addpoly|changealt|clear|disable|draw|enable|list|load|move|movemulti|movepolypoint|param|remove|save|savecsv|savelocal|show|status|undo|update>  # noqa

    def GCSFence(self):
        '''Upload and download of fence'''
        target_system = 1
        target_component = 1

        self.progress("Testing FENCE_POINT protocol")

        self.start_subtest("FENCE_TOTAL manipulation")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.assert_parameter_value("FENCE_TOTAL", 0)

        self.set_parameter("FENCE_TOTAL", 5)
        self.assert_parameter_value("FENCE_TOTAL", 5)

        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.assert_parameter_value("FENCE_TOTAL", 0)

        self.progress("sending out-of-range fencepoint")
        self.send_fencepoint_expect_statustext(0,
                                               0,
                                               1.2345,
                                               5.4321,
                                               "index past total",
                                               target_system=target_component,
                                               target_component=target_component)

        self.progress("sending another out-of-range fencepoint")
        self.send_fencepoint_expect_statustext(0,
                                               1,
                                               1.2345,
                                               5.4321,
                                               "bad count",
                                               target_system=target_component,
                                               target_component=target_component)

        self.set_parameter("FENCE_TOTAL", 1)
        self.assert_parameter_value("FENCE_TOTAL", 1)

        self.send_fencepoint_expect_statustext(0,
                                               1,
                                               1.2345,
                                               5.4321,
                                               "Invalid FENCE_TOTAL",
                                               target_system=target_component,
                                               target_component=target_component)

        self.set_parameter("FENCE_TOTAL", 5)
        self.progress("Checking default points")
        for i in range(5):
            m = self.get_fence_point(i)
            if m.count != 5:
                raise NotAchievedException("Unexpected count in fence point (want=%u got=%u" %
                                           (5, m.count))
            if m.lat != 0 or m.lng != 0:
                raise NotAchievedException("Unexpected lat/lon in fencepoint")

        self.progress("Storing a return point")
        self.roundtrip_fencepoint_protocol(0,
                                           5,
                                           1.2345,
                                           5.4321,
                                           target_system=target_system,
                                           target_component=target_component)

        lat = 2.345
        lng = 4.321
        self.roundtrip_fencepoint_protocol(0,
                                           5,
                                           lat,
                                           lng,
                                           target_system=target_system,
                                           target_component=target_component)

        if not self.mavproxy_can_do_mision_item_protocols():
            self.progress("MAVProxy too old to do fence point protocols")
            return

        self.progress("Download with new protocol")
        items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(items) != 1:
            raise NotAchievedException("Unexpected fencepoint count (want=%u got=%u)" % (1, len(items)))
        if items[0].command != mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
            raise NotAchievedException(
                "Fence return point not of correct type expected (%u) got %u" %
                (items[0].command,
                 mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT))
        if items[0].frame != mavutil.mavlink.MAV_FRAME_GLOBAL:
            raise NotAchievedException(
                "Unexpected frame want=%s got=%s," %
                (self.string_for_frame(mavutil.mavlink.MAV_FRAME_GLOBAL),
                 self.string_for_frame(items[0].frame)))
        got_lat = items[0].x
        want_lat = lat * 1e7
        if abs(got_lat - want_lat) > 1:
            raise NotAchievedException("Disagree in lat (got=%f want=%f)" % (got_lat, want_lat))
        if abs(items[0].y - lng * 1e7) > 1:
            raise NotAchievedException("Disagree in lng")
        if items[0].seq != 0:
            raise NotAchievedException("Disagree in offset")
        self.progress("Downloaded with new protocol OK")

        # upload using mission protocol:
        items = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)

        self.progress("Download with new protocol")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != len(items):
            raise NotAchievedException("Did not download expected number of items (wanted=%u got=%u)" %
                                       (len(items), len(downloaded_items)))
        self.assert_parameter_value("FENCE_TOTAL", len(items) + 1)  # +1 for closing
        self.progress("Ensuring fence items match what we sent up")
        self.check_fence_items_same(items, downloaded_items)

        # now check centroid
        self.progress("Requesting fence return point")
        self.mav.mav.fence_fetch_point_send(target_system,
                                            target_component,
                                            0)
        m = self.mav.recv_match(type="FENCE_POINT", blocking=True, timeout=1)
        print("m: %s" % str(m))

        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.progress("Checking count post-nuke")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.assert_mission_count_on_link(self.mav,
                                          0,
                                          target_system,
                                          target_component,
                                          mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        self.start_subtest("Ensuring bad fences get bounced")
        for fence in self.fences_which_should_not_upload(target_system=target_system, target_component=target_component):
            (name, items) = fence
            self.progress("Ensuring (%s) gets bounced" % (name,))
            self.assert_fence_does_not_upload(items)

        self.start_subtest("Ensuring good fences don't get bounced")
        for fence in self.fences_which_should_upload(target_system=target_system, target_component=target_component):
            (name, items) = fence
            self.progress("Ensuring (%s) gets uploaded" % (name,))
            self.check_fence_upload_download(items)
            self.progress("(%s) uploaded just fine" % (name,))

        self.test_gcs_fence_update_fencepoint(target_system=target_system,
                                              target_component=target_component)

        self.test_gcs_fence_centroid(target_system=target_system,
                                     target_component=target_component)

        self.test_gcs_fence_via_mavproxy(target_system=target_system,
                                         target_component=target_component)

    # explode the write_type_to_storage method
    # FIXME: test converting invalid fences / minimally valid fences / normal fences
    # FIXME: show that uploading smaller items take up less space
    # FIXME: add test for consecutive breaches within the manual recovery period
    # FIXME: ensure truncation does the right thing by fence_total

    # FIXME: test vehicle escape from outside inclusion zones to
    # inside inclusion zones (and inside exclusion zones to outside
    # exclusion zones)
    # FIXME: add test that a fence with edges that cross can't be uploaded
    # FIXME: add a test that fences enclose an area (e.g. all the points aren't the same value!
    def Offboard(self, timeout=90):
        '''Test Offboard Control'''
        self.load_mission("rover-guided-mission.txt")
        self.wait_ready_to_arm(require_absolute=True)
        self.arm_vehicle()
        self.change_mode("AUTO")

        offboard_expected_duration = 10 # see mission file

        if self.mav.messages.get("SET_POSITION_TARGET_GLOBAL_INT", None):
            raise PreconditionFailedException("Already have SET_POSITION_TARGET_GLOBAL_INT")

        tstart = self.get_sim_time_cached()
        last_heartbeat_sent = 0
        got_ptgi = False
        magic_waypoint_tstart = 0
        magic_waypoint_tstop = 0
        while True:
            now = self.get_sim_time_cached()
            if now - last_heartbeat_sent > 1:
                last_heartbeat_sent = now
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                            0,
                                            0,
                                            0)

            if now - tstart > timeout:
                raise AutoTestTimeoutException("Didn't complete")
            magic_waypoint = 3
            mc = self.mav.recv_match(type=["MISSION_CURRENT", "STATUSTEXT"],
                                     blocking=False)
            if mc is not None:
                print("%s" % str(mc))
                if mc.get_type() == "STATUSTEXT":
                    if "Mission Complete" in mc.text:
                        break
                    continue
                if mc.seq == magic_waypoint:
                    print("At magic waypoint")
                    if magic_waypoint_tstart == 0:
                        magic_waypoint_tstart = self.get_sim_time_cached()
                    ptgi = self.mav.messages.get("POSITION_TARGET_GLOBAL_INT", None)
                    if ptgi is not None:
                        got_ptgi = True
                elif mc.seq > magic_waypoint:
                    if magic_waypoint_tstop == 0:
                        magic_waypoint_tstop = self.get_sim_time_cached()

        self.disarm_vehicle()
        offboard_duration = magic_waypoint_tstop - magic_waypoint_tstart
        if abs(offboard_duration - offboard_expected_duration) > 1:
            raise NotAchievedException("Did not stay in offboard control for correct time (want=%f got=%f)" %
                                       (offboard_expected_duration, offboard_duration))

        if not got_ptgi:
            raise NotAchievedException("Did not get ptgi message")
        print("pgti: %s" % str(ptgi))

    def assert_mission_count_on_link(self, mav, expected_count, target_system, target_component, mission_type):
        self.drain_mav_unparsed(mav=mav, freshen_sim_time=True)
        self.progress("waiting for a message - any message....")
        m = mav.recv_match(blocking=True, timeout=1)
        self.progress("Received (%s)" % str(m))

        if not mav.mavlink20():
            raise NotAchievedException("Not doing mavlink2")
        mav.mav.mission_request_list_send(target_system,
                                          target_component,
                                          mission_type)
        self.assert_receive_mission_count_on_link(mav,
                                                  expected_count,
                                                  target_system,
                                                  target_component,
                                                  mission_type)

    def assert_receive_mission_count_on_link(self,
                                             mav,
                                             expected_count,
                                             target_system,
                                             target_component,
                                             mission_type,
                                             expected_target_system=None,
                                             expected_target_component=None,
                                             timeout=120):
        if expected_target_system is None:
            expected_target_system = mav.mav.srcSystem
        if expected_target_component is None:
            expected_target_component = mav.mav.srcComponent
        self.progress("Waiting for mission count of (%u) from (%u:%u) to (%u:%u)" %
                      (expected_count, target_system, target_component, expected_target_system, expected_target_component))

        tstart = self.get_sim_time_cached()
        while True:
            delta = self.get_sim_time_cached() - tstart
            if delta > timeout:
                raise NotAchievedException("Did not receive MISSION_COUNT on link after %fs" % delta)
            m = mav.recv_match(blocking=True, timeout=1)
            if m is None:
                self.progress("No messages")
                continue
#            self.progress("Received (%s)" % str(m))
            if m.get_type() == "MISSION_ACK":
                if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise NotAchievedException("Expected MAV_MISSION_ACCEPTED, got (%s)" % m)
            if m.get_type() == "MISSION_COUNT":
                break
        if m.target_system != expected_target_system:
            raise NotAchievedException("Incorrect target system in MISSION_COUNT (want=%u got=%u)" %
                                       (expected_target_system, m.target_system))
        if m.target_component != expected_target_component:
            raise NotAchievedException("Incorrect target component in MISSION_COUNT")
        if m.mission_type != mission_type:
            raise NotAchievedException("Did not get expected mission type (want=%u got=%u)" % (mission_type, m.mission_type))
        if m.count != expected_count:
            raise NotAchievedException("Bad count received (want=%u got=%u)" %
                                       (expected_count, m.count))
        self.progress("Asserted mission count (type=%u) is %u after %fs" % (
            (mission_type, m.count, delta)))

    def get_mission_item_int_on_link(self, item, mav, target_system, target_component, mission_type, delay_fn=None):
        self.drain_mav(mav=mav, unparsed=True)
        mav.mav.mission_request_int_send(target_system,
                                         target_component,
                                         item,
                                         mission_type)
        m = self.assert_receive_message(
            'MISSION_ITEM_INT',
            timeout=60,
            condition='MISSION_ITEM_INT.mission_type==%u' % mission_type,
            delay_fn=delay_fn)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_ITEM_INT")
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission item of incorrect type")
        if m.target_system != mav.mav.srcSystem:
            raise NotAchievedException("Unexpected target system %u want=%u" %
                                       (m.target_system, mav.mav.srcSystem))
        if m.seq != item:
            raise NotAchievedException(
                "Incorrect sequence number on received item got=%u want=%u" %
                (m.seq, item))
        if m.mission_type != mission_type:
            raise NotAchievedException(
                "Mission type incorrect on received item (want=%u got=%u)" %
                (mission_type, m.mission_type))
        if m.target_component != mav.mav.srcComponent:
            raise NotAchievedException(
                "Unexpected target component %u want=%u" %
                (m.target_component, mav.mav.srcComponent))
        return m

    def get_mission_item_on_link(self, item, mav, target_system, target_component, mission_type):
        self.drain_mav(mav=mav, unparsed=True)
        mav.mav.mission_request_send(target_system,
                                     target_component,
                                     item,
                                     mission_type)
        m = mav.recv_match(type='MISSION_ITEM',
                           blocking=True,
                           timeout=60)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_ITEM")
        if m.target_system != mav.mav.srcSystem:
            raise NotAchievedException("Unexpected target system %u want=%u" %
                                       (m.target_system, mav.mav.srcSystem))
        if m.seq != item:
            raise NotAchievedException("Incorrect sequence number on received item_int got=%u want=%u" %
                                       (m.seq, item))
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission type incorrect on received item_int (want=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.target_component != mav.mav.srcComponent:
            raise NotAchievedException("Unexpected target component %u want=%u" %
                                       (m.target_component, mav.mav.srcComponent))
        return m

    def assert_receive_mission_item_request(self, mission_type, seq):
        self.progress("Expecting request for item %u" % seq)
        m = self.assert_receive_message('MISSION_REQUEST', timeout=1)
        if m.mission_type != mission_type:
            raise NotAchievedException("Incorrect mission type (wanted=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.seq != seq:
            raise NotAchievedException("Unexpected sequence number (want=%u got=%u)" % (seq, m.seq))
        self.progress("Received item request OK")

    def assert_receive_mission_ack(self, mission_type,
                                   want_type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
                                   target_system=None,
                                   target_component=None,
                                   mav=None):
        if mav is None:
            mav = self.mav
        if target_system is None:
            target_system = mav.mav.srcSystem
        if target_component is None:
            target_component = mav.mav.srcComponent
        self.progress("Expecting mission ack")
        m = mav.recv_match(type='MISSION_ACK',
                           blocking=True,
                           timeout=5)
        self.progress("Received ACK (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Expected mission ACK")
        if m.target_system != target_system:
            raise NotAchievedException("ACK not targetted at correct system want=%u got=%u" %
                                       (target_system, m.target_system))
        if m.target_component != target_component:
            raise NotAchievedException("ACK not targetted at correct component")
        if m.mission_type != mission_type:
            raise NotAchievedException("Unexpected mission type %u want=%u" %
                                       (m.mission_type, mission_type))
        if m.type != want_type:
            raise NotAchievedException("Expected ack type %u got %u" %
                                       (want_type, m.type))

    def assert_filepath_content(self, filepath, want):
        with open(filepath) as f:
            got = f.read()
        if want != got:
            raise NotAchievedException("Did not get expected file content (want=%s) (got=%s)" % (want, got))

    def mavproxy_can_do_mision_item_protocols(self):
        return False
        if not self.mavproxy_version_gt(1, 8, 69):
            self.progress("MAVProxy is too old; skipping tests")
            return False
        return True

    def check_rally_items_same(self, want, got, epsilon=None):
        check_atts = ['mission_type', 'command', 'x', 'y', 'z', 'seq', 'param1']
        return self.check_mission_items_same(check_atts, want, got, epsilon=epsilon)

    def click_three_in(self, mavproxy, target_system=1, target_component=1):
        mavproxy.send('rally clear\n')
        self.drain_mav()
        # there are race conditions in MAVProxy.  Beware.
        mavproxy.send("click 1.0 1.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(1)
        mavproxy.send("click 2.0 2.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(1)
        mavproxy.send("click 3.0 3.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            3,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )

    def GCSRally(self, target_system=1, target_component=1):
        '''Upload and download of rally using MAVProxy'''
        self.start_subtest("Testing mavproxy CLI for rally points")
        if not self.mavproxy_can_do_mision_item_protocols():
            return

        mavproxy = self.start_mavproxy()

        mavproxy.send('rally clear\n')

        self.start_subsubtest("rally add")
        mavproxy.send('rally clear\n')
        lat_s = "-5.6789"
        lng_s = "98.2341"
        lat = float(lat_s)
        lng = float(lng_s)
        mavproxy.send('click %s %s\n' % (lat_s, lng_s))
        self.drain_mav()
        mavproxy.send('rally add\n')
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        self.delay_sim_time(5)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 1:
            raise NotAchievedException("Unexpected count (got=%u want=1)" %
                                       (len(downloaded_items), ))
        if (downloaded_items[0].x - int(lat * 1e7)) > 1:
            raise NotAchievedException("Bad rally lat.  Want=%d got=%d" %
                                       (int(lat * 1e7), downloaded_items[0].x))
        if (downloaded_items[0].y - int(lng * 1e7)) > 1:
            raise NotAchievedException("Bad rally lng.  Want=%d got=%d" %
                                       (int(lng * 1e7), downloaded_items[0].y))
        if (downloaded_items[0].z - int(90)) > 1:
            raise NotAchievedException("Bad rally alt.  Want=90 got=%d" %
                                       (downloaded_items[0].y))
        self.end_subsubtest("rally add")

        self.start_subsubtest("rally list")
        util.pexpect_drain(mavproxy)
        mavproxy.send('rally list\n')
        mavproxy.expect(r"Saved 1 rally items to ([^\s]*)\s")
        filename = mavproxy.match.group(1)
        self.assert_rally_filepath_content(filename, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	90.000000	0
''')
        self.end_subsubtest("rally list")

        self.start_subsubtest("rally save")
        util.pexpect_drain(mavproxy)
        save_tmppath = self.buildlogs_path("rally-testing-tmp.txt")
        mavproxy.send('rally save %s\n' % save_tmppath)
        mavproxy.expect(r"Saved 1 rally items to ([^\s]*)\s")
        filename = mavproxy.match.group(1)
        if filename != save_tmppath:
            raise NotAchievedException("Bad save filepath; want=%s got=%s" % (save_tmppath, filename))
        self.assert_rally_filepath_content(filename, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	90.000000	0
''')
        self.end_subsubtest("rally save")

        self.start_subsubtest("rally savecsv")
        util.pexpect_drain(mavproxy)
        csvpath = self.buildlogs_path("rally-testing-tmp.csv")
        mavproxy.send('rally savecsv %s\n' % csvpath)
        mavproxy.expect('"Seq","Frame"')
        expected_content = '''"Seq","Frame","Cmd","P1","P2","P3","P4","X","Y","Z"
"0","Rel","NAV_RALLY_POINT","0.0","0.0","0.0","0.0","-5.67890024185","98.2341003418","90.0"
'''
        if sys.version_info[0] >= 3:
            # greater precision output by default
            expected_content = '''"Seq","Frame","Cmd","P1","P2","P3","P4","X","Y","Z"
"0","Rel","NAV_RALLY_POINT","0.0","0.0","0.0","0.0","-5.678900241851807","98.23410034179688","90.0"
'''
        self.assert_filepath_content(csvpath, expected_content)

        self.end_subsubtest("rally savecsv")

        self.start_subsubtest("rally load")
        self.drain_mav()
        mavproxy.send('rally clear\n')
        self.assert_mission_count_on_link(self.mav,
                                          0,
                                          target_system,
                                          target_component,
                                          mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

        # warning: uses file saved from previous test
        self.start_subtest("Check rally load from filepath")
        mavproxy.send('rally load %s\n' % save_tmppath)
        mavproxy.expect(r"Loaded 1 rally items from ([^\s]*)\s")
        mavproxy.expect("Sent all .* rally items") # notional race condition here
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 1:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if abs(int(downloaded_items[0].x) - int(lat * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" %
                                       (lat * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(lng * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" %
                                       (lng * 1e7, downloaded_items[0].y))
        self.end_subsubtest("rally load")

        self.start_subsubtest("rally changealt")
        mavproxy.send('rally clear\n')
        mavproxy.send("click 1.0 1.0\n")
        mavproxy.send("rally add\n")
        mavproxy.send("click 2.0 2.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.drain_mav()
        mavproxy.send("rally changealt 1 17.6\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        self.delay_sim_time(10)
        mavproxy.send("rally changealt 2 19.1\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        self.delay_sim_time(10)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if abs(int(downloaded_items[0].x) - int(1 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (1 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(1 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (1 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[0].z) - int(17.6)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.6, downloaded_items[0].z))

        if abs(int(downloaded_items[1].x) - int(2 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (2 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[1].y) - int(2 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (2 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[1].z) - int(19.1)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (19.1, downloaded_items[1].z))

        self.progress("Now change two at once")
        mavproxy.send("rally changealt 1 17.3 2\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))

        if abs(int(downloaded_items[0].x) - int(1 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (1 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(1 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (1 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[0].z) - int(17.3)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.3, downloaded_items[0].z))

        if abs(int(downloaded_items[1].x) - int(2 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (2 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[1].y) - int(2 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (2 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[1].z) - int(17.3)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.3, downloaded_items[0].z))

        self.end_subsubtest("rally changealt")

        self.start_subsubtest("rally move")
        mavproxy.send('rally clear\n')
        mavproxy.send("click 1.0 1.0\n")
        mavproxy.send("rally add\n")
        mavproxy.send("click 2.0 2.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(5)
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        mavproxy.send("click 3.0 3.0\n")
        mavproxy.send("rally move 2\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)
        mavproxy.send("click 4.12345 4.987654\n")
        mavproxy.send("rally move 1\n")
        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                        target_system=255,
                                        target_component=0)

        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if downloaded_items[0].x != 41234500:
            raise NotAchievedException("Bad latitude")
        if downloaded_items[0].y != 49876540:
            raise NotAchievedException("Bad longitude")
        if downloaded_items[0].z != 90:
            raise NotAchievedException("Bad altitude (want=%u got=%u)" %
                                       (90, downloaded_items[0].z))

        if downloaded_items[1].x != 30000000:
            raise NotAchievedException("Bad latitude")
        if downloaded_items[1].y != 30000000:
            raise NotAchievedException("Bad longitude")
        if downloaded_items[1].z != 90:
            raise NotAchievedException("Bad altitude (want=%u got=%u)" %
                                       (90, downloaded_items[1].z))
        self.end_subsubtest("rally move")

        self.start_subsubtest("rally movemulti")
        self.drain_mav()
        mavproxy.send('rally clear\n')
        self.drain_mav()
        # there are race conditions in MAVProxy.  Beware.
        mavproxy.send("click 1.0 1.0\n")
        mavproxy.send("rally add\n")
        mavproxy.send("click 2.0 2.0\n")
        mavproxy.send("rally add\n")
        mavproxy.send("click 3.0 3.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            3,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        click_lat = 2.0
        click_lon = 3.0
        unmoved_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(unmoved_items) != 3:
            raise NotAchievedException("Unexpected item count")
        mavproxy.send("click %f %f\n" % (click_lat, click_lon))
        mavproxy.send("rally movemulti 2 1 3\n")
        # MAVProxy currently sends three separate items up.  That's
        # not great and I don't want to lock that behaviour in here.
        self.delay_sim_time(10)
        expected_moved_items = copy.copy(unmoved_items)
        expected_moved_items[0].x = 1.0 * 1e7
        expected_moved_items[0].y = 2.0 * 1e7
        expected_moved_items[1].x = 2.0 * 1e7
        expected_moved_items[1].y = 3.0 * 1e7
        expected_moved_items[2].x = 3.0 * 1e7
        expected_moved_items[2].y = 4.0 * 1e7
        moved_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        # we're moving an entire degree in latitude; quite an epsilon required...
        self.check_rally_items_same(expected_moved_items, moved_items, epsilon=10000)

        self.progress("now move back and rotate through 90 degrees")
        mavproxy.send("click %f %f\n" % (2, 2))
        mavproxy.send("rally movemulti 2 1 3 90\n")

        # MAVProxy currently sends three separate items up.  That's
        # not great and I don't want to lock that behaviour in here.
        self.delay_sim_time(10)
        expected_moved_items = copy.copy(unmoved_items)
        expected_moved_items[0].x = 3.0 * 1e7
        expected_moved_items[0].y = 1.0 * 1e7
        expected_moved_items[1].x = 2.0 * 1e7
        expected_moved_items[1].y = 2.0 * 1e7
        expected_moved_items[2].x = 1.0 * 1e7
        expected_moved_items[2].y = 3.0 * 1e7
        moved_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        # we're moving an entire degree in latitude; quite an epsilon required...
        self.check_rally_items_same(expected_moved_items, moved_items, epsilon=12000)
        self.end_subsubtest("rally movemulti")

        self.start_subsubtest("rally param")
        mavproxy.send("rally param 3 2 5\n")
        mavproxy.expect("Set param 2 for 3 to 5.000000")
        self.end_subsubtest("rally param")

        self.start_subsubtest("rally remove")
        self.click_three_in(target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.progress("Removing last in list")
        mavproxy.send("rally remove 3\n")
        self.delay_sim_time(10)
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        fewer_downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(fewer_downloaded_items) != 2:
            raise NotAchievedException("Unexpected download list length")
        shorter_items = copy.copy(pure_items)
        shorter_items = shorter_items[0:2]
        self.check_rally_items_same(shorter_items, fewer_downloaded_items)

        self.progress("Removing first in list")
        mavproxy.send("rally remove 1\n")
        self.delay_sim_time(5)
        self.assert_mission_count_on_link(
            self.mav,
            1,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        fewer_downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(fewer_downloaded_items) != 1:
            raise NotAchievedException("Unexpected download list length")
        shorter_items = shorter_items[1:]
        self.check_rally_items_same(shorter_items, fewer_downloaded_items)

        self.progress("Removing remaining item")
        mavproxy.send("rally remove 1\n")
        self.delay_sim_time(5)
        self.assert_mission_count_on_link(
            self.mav,
            0,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.end_subsubtest("rally remove")

        self.start_subsubtest("rally show")
        # what can we test here?
        mavproxy.send("rally show %s\n" % save_tmppath)
        self.end_subsubtest("rally show")

        # savelocal must be run immediately after show!
        self.start_subsubtest("rally savelocal")
        util.pexpect_drain(mavproxy)
        savelocal_path = self.buildlogs_path("rally-testing-tmp-local.txt")
        mavproxy.send('rally savelocal %s\n' % savelocal_path)
        self.delay_sim_time(5)
        self.assert_rally_filepath_content(savelocal_path, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	90.000000	0
''')
        self.end_subsubtest("rally savelocal")

        self.start_subsubtest("rally status")
        self.click_three_in(target_system=target_system, target_component=target_component)
        mavproxy.send("rally status\n")
        mavproxy.expect("Have 3 of 3 rally items")
        mavproxy.send("rally clear\n")
        mavproxy.send("rally status\n")
        mavproxy.expect("Have 0 of 0 rally items")
        self.end_subsubtest("rally status")

        self.start_subsubtest("rally undo")
        self.progress("Testing undo-remove")
        self.click_three_in(target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.progress("Removing first in list")
        mavproxy.send("rally remove 1\n")
        self.delay_sim_time(5)
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        mavproxy.send("rally undo\n")
        self.delay_sim_time(5)
        undone_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, undone_items)

        self.progress("Testing undo-move")

        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        mavproxy.send("click 4.12345 4.987654\n")
        mavproxy.send("rally move 1\n")
        # move has already been tested, assume it works...
        self.delay_sim_time(5)
        mavproxy.send("rally undo\n")
        self.delay_sim_time(5)
        undone_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, undone_items)

        self.end_subsubtest("rally undo")

        self.start_subsubtest("rally update")
        self.click_three_in(target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        rally_update_tmpfilepath = self.buildlogs_path("rally-tmp-update.txt")
        mavproxy.send("rally save %s\n" % rally_update_tmpfilepath)
        self.delay_sim_time(5)
        self.progress("Moving waypoint")
        mavproxy.send("click 13.0 13.0\n")
        mavproxy.send("rally move 1\n")
        self.delay_sim_time(5)
        self.progress("Reverting to original")
        mavproxy.send("rally update %s\n" % rally_update_tmpfilepath)
        self.delay_sim_time(5)
        reverted_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, reverted_items)

        self.progress("Making sure specifying a waypoint to be updated works")
        mavproxy.send("click 13.0 13.0\n")
        mavproxy.send("rally move 1\n")
        self.delay_sim_time(5)
        mavproxy.send("click 17.0 17.0\n")
        mavproxy.send("rally move 2\n")
        self.delay_sim_time(5)
        self.progress("Reverting to original item 2")
        mavproxy.send("rally update %s 2\n" % rally_update_tmpfilepath)
        self.delay_sim_time(5)
        reverted_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if reverted_items[0].x != 130000000:
            raise NotAchievedException("Expected item1 x to stay changed (got=%u want=%u)" % (reverted_items[0].x, 130000000))
        if reverted_items[1].x == 170000000:
            raise NotAchievedException("Expected item2 x to revert")

        self.end_subsubtest("rally update")
        self.delay_sim_time(1)
        if self.get_parameter("RALLY_TOTAL") != 0:
            raise NotAchievedException("Failed to clear rally points")

        self.stop_mavproxy(mavproxy)

# MANUAL> usage: rally <add|alt|changealt|clear|list|load|move|movemulti|param|remove|save|savecsv|savelocal|show|status|undo|update>  # noqa

    def RallyUploadDownload(self, target_system=1, target_component=1):
        '''Upload and download of rally'''
        old_srcSystem = self.mav.mav.srcSystem

        self.drain_mav()
        try:
            item1_lat = int(2.0000 * 1e7)
            items = [
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    0, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(1.0000 * 1e7), # latitude
                    int(1.0000 * 1e7), # longitude
                    31.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    1, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    item1_lat, # latitude
                    int(2.0000 * 1e7), # longitude
                    32.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    2, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(3.0000 * 1e7), # latitude
                    int(3.0000 * 1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            ]
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            print("Got items (%s)" % str(items))
            if len(downloaded) != len(items):
                raise NotAchievedException(
                    "Did not download correct number of items want=%u got=%u" %
                    (len(downloaded), len(items)))

            rally_total = self.get_parameter("RALLY_TOTAL")
            if rally_total != len(downloaded):
                raise NotAchievedException(
                    "Unexpected rally point count: want=%u got=%u" %
                    (len(items), rally_total))

            self.progress("Pruning count by setting parameter (urgh)")
            self.set_parameter("RALLY_TOTAL", 2)
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if len(downloaded) != 2:
                raise NotAchievedException(
                    "Failed to prune rally points by setting parameter.  want=%u got=%u" %
                    (2, len(downloaded)))

            self.progress("Uploading a third item using old protocol")
            new_item2_lat = int(6.0 * 1e7)
            self.set_parameter("RALLY_TOTAL", 3)
            self.mav.mav.rally_point_send(target_system,
                                          target_component,
                                          2, # sequence number
                                          3, # total count
                                          new_item2_lat,
                                          int(7.0 * 1e7),
                                          15,
                                          0, # "break" alt?!
                                          0, # "land dir"
                                          0) # flags
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if len(downloaded) != 3:
                raise NotAchievedException(
                    "resetting rally point count didn't change items returned")
            if downloaded[2].x != new_item2_lat:
                raise NotAchievedException(
                    "Bad lattitude in downloaded item: want=%u got=%u" %
                    (new_item2_lat, downloaded[2].x))

            self.progress("Grabbing original item 1 using original protocol")
            self.mav.mav.rally_fetch_point_send(target_system,
                                                target_component,
                                                1)
            m = self.mav.recv_match(type="RALLY_POINT", blocking=True, timeout=1)
            if m.target_system != self.mav.source_system:
                raise NotAchievedException(
                    "Bad target_system on received rally point (want=%u got=%u)" %
                    (255, m.target_system))
            if m.target_component != self.mav.source_component: # autotest's component ID
                raise NotAchievedException("Bad target_component on received rally point")
            if m.lat != item1_lat:
                raise NotAchievedException("Bad latitude on received rally point")

            self.start_subtest("Test upload lockout and timeout")
            self.progress("Starting upload from normal sysid")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            len(items),
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.drain_mav() # throw away requests for items
            self.mav.mav.srcSystem = 243

            self.progress("Attempting upload from sysid=%u" %
                          (self.mav.mav.srcSystem,))
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            len(items),
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED)

            self.progress("Attempting download from sysid=%u" %
                          (self.mav.mav.srcSystem,))
            self.mav.mav.mission_request_list_send(target_system,
                                                   target_component,
                                                   mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED)

            # wait for the upload from sysid=1 to time out:
            tstart = self.get_sim_time()
            got_statustext = False
            got_ack = False
            while True:
                if got_statustext and got_ack:
                    self.progress("Got both ack and statustext")
                    break
                if self.get_sim_time_cached() - tstart > 100:
                    raise NotAchievedException("Did not get both ack and statustext")
                m = self.mav.recv_match(type=['STATUSTEXT', 'MISSION_ACK'],
                                        blocking=True,
                                        timeout=1)
                if m is None:
                    continue
                self.progress("Got (%s)" % str(m))
                if m.get_type() == 'STATUSTEXT':
                    if "upload timeout" in m.text:
                        got_statustext = True
                        self.progress("Received desired statustext")
                    continue
                if m.get_type() == 'MISSION_ACK':
                    if m.target_system != old_srcSystem:
                        raise NotAchievedException("Incorrect sourcesystem")
                    if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                        raise NotAchievedException("Incorrect result")
                    if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_RALLY:
                        raise NotAchievedException("Incorrect mission_type")
                    got_ack = True
                    self.progress("Received desired ACK")
                    continue
                raise NotAchievedException("Huh?")

            self.progress("Now trying to upload empty mission after timeout")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            0,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.start_subtest("Check rally upload/download across separate links")
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            self.progress("ensure a mavlink1 connection can't do anything useful with new item types")
            self.set_parameter("SERIAL2_PROTOCOL", 1)
            self.reboot_sitl()
            mav2 = mavutil.mavlink_connection("tcp:localhost:5763",
                                              robust_parsing=True,
                                              source_system=7,
                                              source_component=7)
            mav2.mav.mission_request_list_send(target_system,
                                               target_component,
                                               mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            # so this looks a bit odd; the other end isn't sending
            # mavlink2 so can't fill in the extension here.
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                want_type=mavutil.mavlink.MAV_MISSION_UNSUPPORTED,
                mav=mav2,
            )
            # this relies on magic upgrade to serial2:
            self.set_parameter("SERIAL2_PROTOCOL", 2)
            expected_count = 3
            self.progress("Assert mission count on new link")
            self.assert_mission_count_on_link(
                mav2,
                expected_count,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Assert mission count on original link")
            self.assert_mission_count_on_link(
                self.mav,
                expected_count,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Get first item on new link")

            def drain_self_mav_fn():
                self.drain_mav(self.mav)
            m2 = self.get_mission_item_int_on_link(
                2,
                mav2,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                delay_fn=drain_self_mav_fn)
            self.progress("Get first item on original link")
            m = self.get_mission_item_int_on_link(
                2,
                self.mav,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if m2.x != m.x:
                raise NotAchievedException("mission items do not match (%d vs %d)" % (m2.x, m.x))
            self.get_mission_item_on_link(2, self.mav, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            # ensure we get nacks for bad mission item requests:
            self.mav.mav.mission_request_send(target_system,
                                              target_component,
                                              65,
                                              mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE,
            )
            self.mav.mav.mission_request_int_send(target_system,
                                                  target_component,
                                                  65,
                                                  mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE,
            )

            self.start_subtest("Should enforce items come from correct GCS")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)
            self.progress("Attempting to upload from bad sysid")
            old_sysid = self.mav.mav.srcSystem
            self.mav.mav.srcSystem = 17
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.mav.mav.srcSystem = old_sysid
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED,
                                            target_system=17)
            self.progress("Sending from correct sysid")
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Attempt to send item on different link to that which we are sending requests on")
            self.progress("Sending count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            2,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)
            old_mav2_system = mav2.mav.srcSystem
            old_mav2_component = mav2.mav.srcComponent
            mav2.mav.srcSystem = self.mav.mav.srcSystem
            mav2.mav.srcComponent = self.mav.mav.srcComponent
            self.progress("Sending item on second link")
            # note that the routing table in ArduPilot now will say
            # this sysid/compid is on both links which may cause
            # weirdness...
            items[0].pack(mav2.mav)
            self.drain_mav(mav=self.mav, unparsed=True)
            mav2.mav.send(items[0])
            mav2.mav.srcSystem = old_mav2_system
            mav2.mav.srcComponent = old_mav2_component
            # we continue to receive requests on the original link:
            m = self.assert_receive_message('MISSION_REQUEST', timeout=1)
            if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_RALLY:
                raise NotAchievedException("Mission request of incorrect type")
            if m.seq != 1:
                raise NotAchievedException("Unexpected sequence number (expected=%u got=%u)" % (1, m.seq))
            items[1].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[1])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Upload mission and rally points at same time")
            self.progress("Sending rally count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            3,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)

            self.progress("Sending wp count")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            3,
                                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 0)

            self.progress("Answering request for mission item 0")
            self.drain_mav(mav=self.mav, unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 1)

            self.progress("Answering request for rally point 0")
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.progress("Expecting request for rally item 1")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Answering request for rally point 1")
            items[1].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[1])
            self.progress("Expecting request for rally item 2")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)

            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[2])
            self.progress("Expecting mission ack for rally")
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.progress("Answering request for waypoints item 1")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 2)

            self.progress("Answering request for waypoints item 2")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            self.start_subtest("Test write-partial-list")
            self.progress("Clearing rally points using count-send")
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                               target_system=target_system,
                               target_component=target_component)
            self.progress("Should not be able to set items completely past the waypoint count")
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                17,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("Should not be able to set items overlapping the waypoint count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                0,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("try to overwrite items 1 and 2")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                1,
                2,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Try shoving up an incorrectly sequenced item")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Try shoving up an incorrectly sequenced item (but within band)")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Now provide correct item")
            item1_latitude = int(1.2345 * 1e7)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                item1_latitude, # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)
            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[2])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("TODO: ensure partial mission write was good")

            self.start_subtest("clear mission types")
            self.assert_mission_count_on_link(
                self.mav,
                3,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                3,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                0,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                3,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_mission_count_on_link(
                self.mav,
                0,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                0,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            self.start_subtest("try sending out-of-range counts")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            230)
            self.assert_receive_mission_ack(230,
                                            want_type=mavutil.mavlink.MAV_MISSION_UNSUPPORTED)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            16000,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_NO_SPACE)

        except Exception as e:
            self.progress("Received exception (%s)" % self.get_exception_stacktrace(e))
            self.mav.mav.srcSystem = old_srcSystem
            raise e
        self.reboot_sitl()

    def ClearMission(self, target_system=1, target_component=1):
        '''check mission clearing'''

        self.start_subtest("Clear via mission_clear_all message")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
        ])
        self.set_current_waypoint(3)

        self.mav.mav.mission_clear_all_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        self.assert_current_waypoint(0)

        self.drain_mav()

        self.start_subtest("No clear mission while it is being uploaded by a different node")
        mav2 = mavutil.mavlink_connection("tcp:localhost:5763",
                                          robust_parsing=True,
                                          source_system=7,
                                          source_component=7)
        self.context_push()
        self.context_collect("MISSION_REQUEST")
        mav2.mav.mission_count_send(target_system,
                                    target_component,
                                    17,
                                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        ack = self.assert_receive_message('MISSION_REQUEST', check_context=True, mav=mav2)
        self.context_pop()

        self.context_push()
        self.context_collect("MISSION_ACK")
        self.mav.mav.mission_clear_all_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        ack = self.assert_receive_message('MISSION_ACK', check_context=True)
        self.assert_message_field_values(ack, {
            "type": mavutil.mavlink.MAV_MISSION_DENIED,
        })
        self.context_pop()

        self.progress("Test cancel upload from second connection")
        self.context_push()
        self.context_collect("MISSION_ACK")
        mav2.mav.mission_clear_all_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        ack = self.assert_receive_message('MISSION_ACK', mav=mav2, check_context=True)
        self.assert_message_field_values(ack, {
            "type": mavutil.mavlink.MAV_MISSION_ACCEPTED,
        })
        self.context_pop()
        mav2.close()
        del mav2

    def GCSMission(self):
        '''check MAVProxy's waypoint handling of missions'''

        target_system = 1
        target_component = 1
        mavproxy = self.start_mavproxy()
        mavproxy.send('wp clear\n')
        self.delay_sim_time(1)
        if self.get_parameter("MIS_TOTAL") != 0:
            raise NotAchievedException("Failed to clear mission")
        m = self.assert_receive_message('MISSION_CURRENT', timeout=5, verbose=True)
        if m.seq != 0:
            raise NotAchievedException("Bad mission current")
        self.load_mission_using_mavproxy(mavproxy, "rover-gripper-mission.txt")
        set_wp = 1
        mavproxy.send('wp set %u\n' % set_wp)
        self.wait_current_waypoint(set_wp)

        self.start_subsubtest("wp changealt")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        changealt_item = 1
#        oldalt = downloaded_items[changealt_item].z
        want_newalt = 37.2
        mavproxy.send('wp changealt %u %f\n' % (changealt_item, want_newalt))
        self.delay_sim_time(15)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        if abs(downloaded_items[changealt_item].z - want_newalt) > 0.0001:
            raise NotAchievedException(
                "changealt didn't (want=%f got=%f)" %
                (want_newalt, downloaded_items[changealt_item].z))
        self.end_subsubtest("wp changealt")

        self.start_subsubtest("wp sethome")
        new_home_lat = 3.14159
        new_home_lng = 2.71828
        mavproxy.send('click %f %f\n' % (new_home_lat, new_home_lng))
        mavproxy.send('wp sethome\n')
        self.delay_sim_time(5)
        # any way to close the loop on this one?
        # downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        # if abs(downloaded_items[0].x - new_home_lat) > 0.0001:
        #     raise NotAchievedException("wp sethome didn't work")
        # if abs(downloaded_items[0].y - new_home_lng) > 0.0001:
        #     raise NotAchievedException("wp sethome didn't work")
        self.end_subsubtest("wp sethome")

        self.start_subsubtest("wp slope")
        mavproxy.send('wp slope\n')
        mavproxy.expect("WP3: slope 0.1")
        self.delay_sim_time(5)
        self.end_subsubtest("wp slope")

        if not self.mavproxy_can_do_mision_item_protocols():
            # adding based on click location yet to be merged into MAVProxy
            return

        self.start_subsubtest("wp split")
        mavproxy.send("wp clear\n")
        self.delay_sim_time(5)
        mavproxy.send("wp list\n")
        self.delay_sim_time(5)
        items = [
            None,
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 * 1e7), # latitude
                int(1.0 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(2.0 * 1e7), # latitude
                int(2.0 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        ]
        mavproxy.send("click 5 5\n") # space for home position
        mavproxy.send("wp add\n")
        self.delay_sim_time(5)
        self.click_location_from_item(mavproxy, items[1])
        mavproxy.send("wp add\n")
        self.delay_sim_time(5)
        self.click_location_from_item(mavproxy, items[2])
        mavproxy.send("wp add\n")
        self.delay_sim_time(5)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.check_mission_waypoint_items_same(items, downloaded_items)
        mavproxy.send("wp split 2\n")
        self.delay_sim_time(5)
        items_with_split_in = [
            items[0],
            items[1],
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.5 * 1e7), # latitude
                int(1.5 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            items[2],
        ]
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.check_mission_waypoint_items_same(items_with_split_in,
                                               downloaded_items)

        self.stop_mavproxy(mavproxy)

        # MANUAL> usage: wp <changealt|clear|draw|editor|list|load|loop|move|movemulti|noflyadd|param|remove|save|savecsv|savelocal|set|sethome|show|slope|split|status|undo|update>  # noqa

    def wait_location_sending_target(self, loc, target_system=1, target_component=1, timeout=60, max_delta=2):
        tstart = self.get_sim_time()
        last_sent = 0

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("Did not get to location")
            if now - last_sent > 10:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat * 1.0e7),
                    int(loc.lng * 1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                self.progress("Target: (%s)" % str(m), send_statustext=False)
            elif t == "GLOBAL_POSITION_INT":
                self.progress("Position: (%s)" % str(m), send_statustext=False)
                delta = self.get_distance(
                    mavutil.location(m.lat * 1e-7, m.lon * 1e-7, 0, 0),
                    loc)
                self.progress("delta: %s" % str(delta), send_statustext=False)
                if delta < max_delta:
                    self.progress("Reached destination")

    def drive_to_location(self, loc, tolerance=1, timeout=30, target_system=1, target_component=1):
        self.assert_mode('GUIDED')

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not get to location")
            if now - last_sent > 10:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat * 1.0e7),
                    int(loc.lng * 1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            if self.get_distance(self.mav.location(), loc) > tolerance:
                continue
            return

    def drive_somewhere_breach_boundary_and_rtl(self, loc, target_system=1, target_component=1, timeout=60):
        tstart = self.get_sim_time()
        last_sent = 0
        seen_fence_breach = False

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not breach boundary + RTL")
            if now - last_sent > 10:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat * 1.0e7),
                    int(loc.lng * 1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                self.progress("Target: (%s)" % str(m))
            elif t == "GLOBAL_POSITION_INT":
                self.progress("Position: (%s)" % str(m))
            elif t == "FENCE_STATUS":
                self.progress("Fence: %s" % str(m))
                if m.breach_status != 0:
                    seen_fence_breach = True
                    self.progress("Fence breach detected!")
                    if m.breach_type != mavutil.mavlink.FENCE_BREACH_BOUNDARY:
                        raise NotAchievedException("Breach of unexpected type")
            if self.mode_is("RTL", cached=True) and seen_fence_breach:
                break
        self.wait_distance_to_home(3, 7, timeout=30)

    def drive_somewhere_stop_at_boundary(self,
                                         loc,
                                         expected_stopping_point,
                                         expected_distance_epsilon=1.0,
                                         target_system=1,
                                         target_component=1,
                                         timeout=120):
        tstart = self.get_sim_time()
        last_sent = 0

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')
        at_stopping_point = False
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not arrive and stop at boundary")
            if now - last_sent > 10:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat * 1.0e7),
                    int(loc.lng * 1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                print("Target: (%s)" % str(m))
            elif t == "GLOBAL_POSITION_INT":
                print("Position: (%s)" % str(m))
                delta = self.get_distance(
                    mavutil.location(m.lat * 1e-7, m.lon * 1e-7, 0, 0),
                    mavutil.location(expected_stopping_point.lat,
                                     expected_stopping_point.lng,
                                     0,
                                     0))
                print("delta: %s want_delta<%f" % (str(delta), expected_distance_epsilon))
                at_stopping_point = delta < expected_distance_epsilon
            elif t == "VFR_HUD":
                print("groundspeed: %f" % m.groundspeed)
                if at_stopping_point:
                    if m.groundspeed < 1:
                        self.progress("Seemed to have stopped at stopping point")
                        return

    def assert_fence_breached(self):
        m = self.assert_receive_message('FENCE_STATUS', timeout=10)
        if m.breach_status != 1:
            raise NotAchievedException("Expected to be breached")

    def wait_fence_not_breached(self, timeout=5):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("Fence remains breached")
            m = self.mav.recv_match(type='FENCE_STATUS',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                self.progress("No FENCE_STATUS received")
                continue
            self.progress("STATUS: %s" % str(m))
            if m.breach_status == 0:
                break

    def test_poly_fence_noarms(self, target_system=1, target_component=1):
        '''various tests to ensure we can't arm when in breach of a polyfence'''
        self.start_subtest("Ensure PolyFence arming checks work")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.set_parameters({
            "FENCE_TYPE": 2,    # circle only
        })
        self.delay_sim_time(5) # let breaches clear
        # FIXME: should we allow this?
        self.progress("Ensure we can arm with no poly in place")
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()
        self.set_parameters({
            "FENCE_TYPE": 6,    # polyfence + circle
        })

        self.test_poly_fence_noarms_exclusion_circle(target_system=target_system,
                                                     target_component=target_component)
        self.test_poly_fence_noarms_inclusion_circle(target_system=target_system,
                                                     target_component=target_component)
        self.test_poly_fence_noarms_exclusion_polyfence(target_system=target_system,
                                                        target_component=target_component)
        self.test_poly_fence_noarms_inclusion_polyfence(target_system=target_system,
                                                        target_component=target_component)

    def test_poly_fence_noarms_exclusion_circle(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when within an exclusion circle")

        here = self.mav.location()

        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(here.lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(self.offset_location_ne(here, 100, 100).lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when within exclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_inclusion_circle(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when outside an inclusion circle (but within another")

        here = self.mav.location()

        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(here.lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(self.offset_location_ne(here, 100, 100).lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when outside an inclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_exclusion_polyfence(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when inside an exclusion polyfence (but outside another")

        here = self.mav.location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when within polygon exclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_inclusion_polyfence(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when outside an inclusion polyfence (but within another")

        here = self.mav.location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.delay_sim_time(5) # ArduPilot only checks for breaches @1Hz
        self.drain_mav()
        self.assert_fence_breached()
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when outside polygon inclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_fence_upload_timeouts_1(self, target_system=1, target_component=1):
        self.start_subtest("fence_upload timeouts 1")
        self.progress("Telling victim there will be one item coming")
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        1,
                                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'],
                                blocking=True,
                                timeout=1)
        self.progress("Got (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Did not get ACK or mission request")

        if m.get_type() == "MISSION_ACK":
            raise NotAchievedException("Expected MISSION_REQUEST")

        if m.seq != 0:
            raise NotAchievedException("Expected request for seq=0")

        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
        tstart = self.get_sim_time()
        rerequest_count = 0
        received_text = False
        received_ack = False
        while True:
            if received_ack and received_text:
                break
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not get expected ack and statustext")
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK', 'STATUSTEXT'],
                                    blocking=True,
                                    timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.progress("Did not receive any messages")
                continue
            if m.get_type() == "MISSION_REQUEST":
                if m.seq != 0:
                    raise NotAchievedException("Received request for invalid seq")
                if m.target_system != self.mav.mav.srcSystem:
                    raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
                if m.target_component != self.mav.mav.srcComponent:
                    raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
                rerequest_count += 1
                self.progress("Valid re-request received.")
                continue
            if m.get_type() == "MISSION_ACK":
                if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    raise NotAchievedException("Wrong mission type")
                if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                    raise NotAchievedException("Wrong result")
                received_ack = True
                continue
            if m.get_type() == "STATUSTEXT":
                if "upload time" in m.text:
                    received_text = True
                continue
        if rerequest_count < 3:
            raise NotAchievedException("Expected several re-requests of mission item")
        self.end_subtest("fence upload timeouts 1")

    def expect_request_for_item(self, item):
        m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'],
                                blocking=True,
                                timeout=1)
        self.progress("Got (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Did not get ACK or mission request")

        if m.get_type() == "MISSION_ACK":
            raise NotAchievedException("Expected MISSION_REQUEST")

        if m.seq != item.seq:
            raise NotAchievedException("Expected request for seq=%u" % item.seq)

        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("Incorrect target component in MISSION_REQUEST")

    def test_fence_upload_timeouts_2(self, target_system=1, target_component=1):
        self.start_subtest("fence upload timeouts 2")
        self.progress("Telling victim there will be two items coming")
        # avoid a timeout race condition where ArduPilot re-requests a
        # fence point before we receive and respond to the first one.
        # Since ArduPilot has a 1s timeout on re-requesting, This only
        # requires a round-trip delay of 1/speedup seconds to trigger
        # - and that has been seen in practise on Travis
        old_speedup = self.get_parameter("SIM_SPEEDUP")
        self.set_parameter("SIM_SPEEDUP", 1)
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        2,
                                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.progress("Sending item with seq=0")
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            1, # p1 radius
            0, # p2
            0, # p3
            0, # p4
            int(1.1 * 1e7), # latitude
            int(1.2 * 1e7), # longitude
            33.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.expect_request_for_item(item)
        item.pack(self.mav.mav)
        self.mav.mav.send(item)

        self.progress("Sending item with seq=1")
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            1, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            1, # p1 radius
            0, # p2
            0, # p3
            0, # p4
            int(1.1 * 1e7), # latitude
            int(1.2 * 1e7), # longitude
            33.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        self.expect_request_for_item(item)

        self.set_parameter("SIM_SPEEDUP", old_speedup)

        self.progress("Now waiting for a timeout")
        tstart = self.get_sim_time()
        rerequest_count = 0
        received_text = False
        received_ack = False
        while True:
            if received_ack and received_text:
                break
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not get expected ack and statustext")
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK', 'STATUSTEXT'],
                                    blocking=True,
                                    timeout=0.1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.progress("Did not receive any messages")
                continue
            if m.get_type() == "MISSION_REQUEST":
                if m.seq != 1:
                    raise NotAchievedException("Received request for invalid seq")
                if m.target_system != self.mav.mav.srcSystem:
                    raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
                if m.target_component != self.mav.mav.srcComponent:
                    raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
                rerequest_count += 1
                self.progress("Valid re-request received.")
                continue
            if m.get_type() == "MISSION_ACK":
                if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    raise NotAchievedException("Wrong mission type")
                if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                    raise NotAchievedException("Wrong result")
                received_ack = True
                continue
            if m.get_type() == "STATUSTEXT":
                if "upload time" in m.text:
                    received_text = True
                continue
        if rerequest_count < 3:
            raise NotAchievedException("Expected several re-requests of mission item")
        self.end_subtest("fence upload timeouts 2")

    def test_fence_upload_timeouts(self, target_system=1, target_component=1):
        self.test_fence_upload_timeouts_1(target_system=target_system,
                                          target_component=target_component)
        self.test_fence_upload_timeouts_2(target_system=target_system,
                                          target_component=target_component)

    def test_poly_fence_compatability_ordering(self, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        here = self.mav.location()
        self.progress("try uploading return point last")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[1, 2, 3, 4, 5, 0])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        self.progress("try uploading return point in middle")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[1, 2, 3, 0, 4, 5])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        self.progress("try closing point in middle")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[0, 1, 2, 5, 3, 4])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        # this is expected to fail as we don't return the closing
        # point correctly until the first is uploaded
        # self.progress("try closing point first")
        # failed = False
        # try:
        #     self.roundtrip_fence_using_fencepoint_protocol([
        #         self.offset_location_ne(here, 0, 0), # bl // return point
        #         self.offset_location_ne(here, -50, 20), # bl
        #         self.offset_location_ne(here, 50, 20), # br
        #         self.offset_location_ne(here, 50, 40), # tr
        #         self.offset_location_ne(here, -50, 40), # tl,
        #         self.offset_location_ne(here, -50, 20), # closing point
        #     ], ordering=[5, 0, 1, 2, 3, 4])
        # except NotAchievedException as e:
        #     failed = "got=0.000000 want=" in str(e)
        # if not failed:
        #     raise NotAchievedException("Expected failure, did not get it")
        # self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        #                    target_system=target_system,
        #                    target_component=target_component)

        self.progress("try (almost) reverse order")
        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[4, 3, 2, 1, 0, 5])
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

    def test_poly_fence_big_then_small(self, target_system=1, target_component=1):
        here = self.mav.location()

        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 20), # br
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[1, 2, 3, 4, 5, 0])
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != 5:
            # that's one return point and then bl, br, tr, then tl
            raise NotAchievedException("Bad number of downloaded items in original download")

        self.roundtrip_fence_using_fencepoint_protocol([
            self.offset_location_ne(here, 0, 0), # bl // return point
            self.offset_location_ne(here, -50, 20), # bl
            self.offset_location_ne(here, 50, 40), # tr
            self.offset_location_ne(here, -50, 40), # tl,
            self.offset_location_ne(here, -50, 20), # closing point
        ], ordering=[1, 2, 3, 4, 0])

        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        want_count = 4
        if len(downloaded_items) != want_count:
            # that's one return point and then bl, tr, then tl
            raise NotAchievedException("Bad number of downloaded items in second download got=%u wanted=%u" %
                                       (len(downloaded_items), want_count))
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != 4:
            # that's one return point and then bl, tr, then tl
            raise NotAchievedException("Bad number of downloaded items in second download (second time) got=%u want=%u" %
                                       (len(downloaded_items), want_count))

    def test_poly_fence_compatability(self, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)

        self.test_poly_fence_compatability_ordering(target_system=target_system, target_component=target_component)

        here = self.mav.location()

        self.progress("Playing with changing point count")
        self.roundtrip_fence_using_fencepoint_protocol(
            [
                self.offset_location_ne(here, 0, 0), # bl // return point
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
                self.offset_location_ne(here, -50, 20), # closing point
            ])
        self.roundtrip_fence_using_fencepoint_protocol(
            [
                self.offset_location_ne(here, 0, 0), # bl // return point
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, -50, 40), # tl,
                self.offset_location_ne(here, -50, 20), # closing point
            ])
        self.roundtrip_fence_using_fencepoint_protocol(
            [
                self.offset_location_ne(here, 0, 0), # bl // return point
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
                self.offset_location_ne(here, -50, 20), # closing point
            ])

    def test_poly_fence_reboot_survivability(self):
        here = self.mav.location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.reboot_sitl()
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        downloaded_len = len(downloaded_items)
        if downloaded_len != 8:
            raise NotAchievedException("Items did not survive reboot (want=%u got=%u)" %
                                       (8, downloaded_len))

    def PolyFence(self):
        '''test fence-related functions'''
        target_system = 1
        target_component = 1

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        here = self.mav.location()
        self.progress("here: %f %f" % (here.lat, here.lng))
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 0,
        })

#        self.set_parameter("SIM_SPEEDUP", 1)

        self.test_poly_fence_big_then_small()

        self.test_poly_fence_compatability()

        self.test_fence_upload_timeouts()

        self.test_poly_fence_noarms(target_system=target_system, target_component=target_component)

        self.arm_vehicle()

        self.test_poly_fence_inclusion(here, target_system=target_system, target_component=target_component)
        self.test_poly_fence_exclusion(here, target_system=target_system, target_component=target_component)

        self.disarm_vehicle()

        self.test_poly_fence_reboot_survivability()

    def test_poly_fence_inclusion_overlapping_inclusion_circles(self, here, target_system=1, target_component=1):
        self.start_subtest("Overlapping circular inclusion")
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -20, 0),
            }),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, 20, 0),
            }),
        ])
        if self.mavproxy is not None:
            # handy for getting pretty pictures
            self.mavproxy.send("fence list\n")

        self.delay_sim_time(5)
        self.progress("Drive outside top circle")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.delay_sim_time(5)
        self.progress("Drive outside bottom circle")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_inclusion(self, here, target_system=1, target_component=1):
        self.progress("Circle and Polygon inclusion")
        self.test_poly_fence_inclusion_overlapping_inclusion_circles(
            here,
            target_system=target_system,
            target_component=target_component)

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(here, -40, -20), # tl
                self.offset_location_ne(here, 50, -20), # tr
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, -40, 20), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -20, 0),
            }),
        ])

        self.delay_sim_time(5)
        if self.mavproxy is not None:
            self.mavproxy.send("fence list\n")
        self.progress("Drive outside polygon")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.delay_sim_time(5)
        self.progress("Drive outside circle")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(here, -20, -25), # tl
                self.offset_location_ne(here, 50, -25), # tr
                self.offset_location_ne(here, 50, 15), # br
                self.offset_location_ne(here, -20, 15), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(here, 20, -20), # tl
                self.offset_location_ne(here, -50, -20), # tr
                self.offset_location_ne(here, -50, 20), # br
                self.offset_location_ne(here, 20, 20), # bl,
            ]),
        ])

        self.delay_sim_time(5)
        if self.mavproxy is not None:
            self.mavproxy.send("fence list\n")
        self.progress("Drive outside top polygon")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.delay_sim_time(5)
        self.progress("Drive outside bottom polygon")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_exclusion(self, here, target_system=1, target_component=1):

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # west
                self.offset_location_ne(here, -50, -20), # tl
                self.offset_location_ne(here, 50, -20), # tr
                self.offset_location_ne(here, 50, -40), # br
                self.offset_location_ne(here, -50, -40), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -60, 0),
            }),
        ])
        self.delay_sim_time(5)
        if self.mavproxy is not None:
            self.mavproxy.send("fence list\n")

        self.progress("Breach eastern boundary")
        fence_middle = self.offset_location_ne(here, 0, 30)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)

        self.progress("delaying - hack to work around manual recovery bug")
        self.delay_sim_time(5)

        self.progress("Breach western boundary")
        fence_middle = self.offset_location_ne(here, 0, -30)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)

        self.progress("delaying - hack to work around manual recovery bug")
        self.delay_sim_time(5)

        self.progress("Breach southern circle")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)

    def SmartRTL(self):
        '''Test SmartRTL'''
        self.change_mode("STEERING")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # drive two sides of a square, make sure we don't go back through
        # the middle of the square
        self.progress("Driving North")
        self.reach_heading_manual(0)
        self.set_rc(3, 2000)
        self.delay_sim_time(5)
        self.set_rc(3, 1000)
        self.wait_groundspeed(0, 1)
        loc = self.mav.location()
        self.progress("Driving East")
        self.set_rc(3, 2000)
        self.reach_heading_manual(90)
        self.set_rc(3, 2000)
        self.delay_sim_time(5)
        self.set_rc(3, 1000)

        self.progress("Entering smartrtl")
        self.change_mode("SMART_RTL")

        self.progress("Ensure we go via intermediate point")
        self.wait_distance_to_location(loc, 0, 5, timeout=60)

        self.progress("Ensure we get home")
        self.wait_distance_to_home(3, 7, timeout=30)

        self.disarm_vehicle()

    def MotorTest(self):
        '''Motor Test triggered via mavlink'''
        magic_throttle_value = 1812
        self.wait_ready_to_arm()
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            p1=1, # motor instance
            p2=mavutil.mavlink.MOTOR_TEST_THROTTLE_PWM, # throttle type
            p3=magic_throttle_value, # throttle
            p4=5, # timeout
            p5=1, # motor count
            p6=0, # test order (see MOTOR_TEST_ORDER)
        )
        self.wait_armed()
        self.progress("Waiting for magic throttle value")
        self.wait_servo_channel_value(3, magic_throttle_value)
        self.wait_servo_channel_value(3, self.get_parameter("RC3_TRIM", 5), timeout=10)
        self.wait_disarmed()

    def PolyFenceObjectAvoidanceGuided(self, target_system=1, target_component=1):
        '''PolyFence object avoidance tests - guided mode'''
        if not self.mavproxy_can_do_mision_item_protocols():
            return

        self.test_poly_fence_object_avoidance_guided_pathfinding(
            target_system=target_system,
            target_component=target_component)
        self.test_poly_fence_object_avoidance_guided_two_squares(
            target_system=target_system,
            target_component=target_component)

    def PolyFenceObjectAvoidanceAuto(self, target_system=1, target_component=1):
        '''PolyFence object avoidance tests - auto mode'''
        mavproxy = self.start_mavproxy()
        self.load_fence_using_mavproxy(mavproxy, "rover-path-planning-fence.txt")
        self.stop_mavproxy(mavproxy)
        # self.load_fence("rover-path-planning-fence.txt")
        self.load_mission("rover-path-planning-mission.txt")
        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 2,
            "FENCE_MARGIN": 0, # FIXME: https://github.com/ArduPilot/ardupilot/issues/11601
        })
        self.reboot_sitl()
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("FENCE_ENABLE", 1)
        # target_loc is copied from the mission file
        target_loc = mavutil.location(40.073799, -105.229156)
        self.wait_location(target_loc, height_accuracy=None, timeout=300)
        # mission has RTL as last item
        self.wait_distance_to_home(3, 7, timeout=300)
        self.disarm_vehicle()

    def send_guided_mission_item(self, loc, target_system=1, target_component=1):
        self.mav.mav.mission_item_send(
            target_system,
            target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, # current
            0, # autocontinue
            0, # param1
            0, # param2
            0, # param3
            0, # param4
            loc.lat, # x
            loc.lng, # y
            0 # z
        )

    def test_poly_fence_object_avoidance_guided_pathfinding(self, target_system=1, target_component=1):
        self.load_fence("rover-path-planning-fence.txt")
        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 2,
            "FENCE_MARGIN": 0, # FIXME: https://github.com/ArduPilot/ardupilot/issues/11601
        })
        self.reboot_sitl()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("FENCE_ENABLE", 1)
        target_loc = mavutil.location(40.073800, -105.229172)
        self.send_guided_mission_item(target_loc,
                                      target_system=target_system,
                                      target_component=target_component)
        self.wait_location(target_loc, timeout=300)
        self.do_RTL(timeout=300)
        self.disarm_vehicle()

    def WheelEncoders(self):
        '''make sure wheel encoders are generally working'''
        self.set_parameters({
            "WENC_TYPE": 10,
            "EK3_ENABLE": 1,
            "AHRS_EKF_TYPE": 3,
        })
        self.reboot_sitl()
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.change_mode("MANUAL")
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

    def test_poly_fence_object_avoidance_guided_two_squares(self, target_system=1, target_component=1):
        self.start_subtest("Ensure we can steer around obstacles in guided mode")
        here = self.mav.location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 10), # tl
                self.offset_location_ne(here, 50, 30), # tr
                self.offset_location_ne(here, -50, 40), # br,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # further east (and south
                self.offset_location_ne(here, -60, 60), # bl
                self.offset_location_ne(here, 40, 70), # tl
                self.offset_location_ne(here, 40, 90), # tr
                self.offset_location_ne(here, -60, 80), # br,
            ]),
        ])
        if self.mavproxy is not None:
            self.mavproxy.send("fence list\n")
        self.context_push()
        ex = None
        try:
            self.set_parameters({
                "AVOID_ENABLE": 3,
                "OA_TYPE": 2,
            })
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.set_parameter("FENCE_ENABLE", 1)
            if self.mavproxy is not None:
                self.mavproxy.send("fence list\n")
            self.arm_vehicle()

            self.change_mode("GUIDED")
            target = mavutil.location(40.071382, -105.228340, 0, 0)
            self.send_guided_mission_item(target,
                                          target_system=target_system,
                                          target_component=target_component)
            self.wait_location(target, timeout=300)
            self.do_RTL()
            self.disarm_vehicle()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_poly_fence_avoidance_dont_breach_exclusion(self, target_system=1, target_component=1):
        self.start_subtest("Ensure we stop before breaching an exclusion fence")
        here = self.mav.location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # west
                self.offset_location_ne(here, -50, -20), # tl
                self.offset_location_ne(here, 50, -20), # tr
                self.offset_location_ne(here, 50, -40), # br
                self.offset_location_ne(here, -50, -40), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -60, 0),
            }),
        ])
        if self.mavproxy is not None:
            self.mavproxy.send("fence list\n")
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 3,
        })
        fence_middle = self.offset_location_ne(here, 0, 30)
        # FIXME: this might be nowhere near "here"!
        expected_stopping_point = mavutil.location(40.0713376, -105.2295738, 0, 0)
        self.drive_somewhere_stop_at_boundary(
            fence_middle,
            expected_stopping_point,
            target_system=target_system,
            target_component=target_component,
            expected_distance_epsilon=3)
        self.set_parameter("AVOID_ENABLE", 0)
        self.do_RTL()

    def do_RTL(self, distance_min=3, distance_max=7, timeout=60):
        self.change_mode("RTL")
        self.wait_distance_to_home(distance_min, distance_max, timeout=timeout)

    def PolyFenceAvoidance(self, target_system=1, target_component=1):
        '''PolyFence avoidance tests'''
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("MANUAL")
        self.reach_heading_manual(180, turn_right=False)
        self.change_mode("GUIDED")

        self.test_poly_fence_avoidance_dont_breach_exclusion(target_system=target_system, target_component=target_component)

        self.disarm_vehicle()

    def PolyFenceObjectAvoidanceBendyRuler(self, target_system=1, target_component=1):
        '''PolyFence object avoidance tests - bendy ruler'''
        self.load_fence_using_mavwp("rover-path-bendyruler-fence.txt")
        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 1,
            "FENCE_ENABLE": 1,
            "WP_RADIUS": 5,
        })
        self.reboot_sitl()
        self.set_parameters({
            "OA_BR_LOOKAHEAD": 50,
        })
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        target_loc = mavutil.location(40.071060, -105.227734, 1584, 0)
        self.send_guided_mission_item(target_loc,
                                      target_system=target_system,
                                      target_component=target_component)
        # FIXME: we don't get within WP_RADIUS of our target?!
        self.wait_location(target_loc, timeout=300, accuracy=15)
        self.do_RTL(timeout=300)
        self.disarm_vehicle()

    def PolyFenceObjectAvoidanceBendyRulerEasierGuided(self, target_system=1, target_component=1):
        '''finish-line issue means we can't complete the harder one.  This
        test can go away once we've nailed that one.  The only
        difference here is the target point.
        '''
        self.load_fence_using_mavwp("rover-path-bendyruler-fence.txt")
        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 1,
            "FENCE_ENABLE": 1,
            "WP_RADIUS": 5,
        })
        self.reboot_sitl()
        self.set_parameters({
            "OA_BR_LOOKAHEAD": 60,
        })
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        target_loc = mavutil.location(40.071260, -105.227000, 1584, 0)
        self.send_guided_mission_item(target_loc,
                                      target_system=target_system,
                                      target_component=target_component)
        # FIXME: we don't get within WP_RADIUS of our target?!
        self.wait_location(target_loc, timeout=300, accuracy=15)
        self.do_RTL(timeout=300)
        self.disarm_vehicle()

    def PolyFenceObjectAvoidanceBendyRulerEasierAuto(self, target_system=1, target_component=1):
        '''finish-line issue means we can't complete the harder one.  This
        test can go away once we've nailed that one.  The only
        difference here is the target point.
        '''
        self.load_fence_using_mavwp("rover-path-bendyruler-fence.txt")
        self.load_mission("rover-path-bendyruler-mission-easier.txt")

        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 1,  # BendyRuler
            "FENCE_ENABLE": 1,
            "WP_RADIUS": 5,
        })
        self.reboot_sitl()
        self.set_parameters({
            "OA_BR_LOOKAHEAD": 60,
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        target_loc = mavutil.location(40.071260, -105.227000, 1584, 0)
        # target_loc is copied from the mission file
        self.wait_location(target_loc, timeout=300)
        # mission has RTL as last item
        self.wait_distance_to_home(3, 7, timeout=300)
        self.disarm_vehicle()

    def test_scripting_simple_loop(self):
        self.start_subtest("Scripting simple loop")

        self.context_push()

        messages = []

        def my_message_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            messages.append(message)

        self.install_message_hook_context(my_message_hook)

        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("simple_loop.lua")
        self.reboot_sitl()
        self.delay_sim_time(10)

        self.context_pop()
        self.reboot_sitl()

        # check all messages to see if we got our message
        count = 0
        for m in messages:
            if "hello, world" in m.text:
                count += 1
        self.progress("Got %u hellos" % count)
        if count < 3:
            raise NotAchievedException("Expected at least three hellos")

    def test_scripting_internal_test(self):
        self.start_subtest("Scripting internal test")

        self.context_push()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_HEAP_SIZE": 1024000,
            "SCR_VM_I_COUNT": 1000000,
        })
        self.install_test_modules_context()
        self.install_mavlink_module_context()

        self.install_test_scripts_context([
            "scripting_test.lua",
            "scripting_require_test_2.lua",
            "math.lua",
            "strings.lua",
            "mavlink_test.lua",
        ])

        self.context_collect('STATUSTEXT')
        self.context_collect('NAMED_VALUE_FLOAT')

        self.reboot_sitl()

        for success_text in [
                "Internal tests passed",
                "Require test 2 passed",
                "Math tests passed",
                "String tests passed",
                "Received heartbeat from"
        ]:
            self.wait_statustext(success_text, check_context=True)

        for success_nvf in [
                "test",
        ]:
            self.assert_received_message_field_values("NAMED_VALUE_FLOAT", {
                "name": success_nvf,
            }, check_context=True)

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_hello_world(self):
        self.start_subtest("Scripting hello world")

        self.context_push()
        self.context_collect("STATUSTEXT")
        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("hello_world.lua")
        self.reboot_sitl()

        self.wait_statustext('hello, world', check_context=True, timeout=30)

        self.context_pop()
        self.reboot_sitl()

    def ScriptingSteeringAndThrottle(self):
        '''Scripting test - steering and throttle'''
        self.start_subtest("Scripting square")

        self.context_push()
        self.install_example_script_context("rover-set-steering-and-throttle.lua")
        self.set_parameter("SCR_ENABLE", 1)
        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while not self.mode_is("HOLD"):
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("Did not move to hold")
            m = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            if m is not None:
                self.progress("Current speed: %f" % m.groundspeed)
        self.disarm_vehicle()

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_auxfunc(self):
        self.start_subtest("Scripting aufunc triggering")

        self.context_push()
        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "SCR_ENABLE": 1,
            "RELAY1_FUNCTION": 1,
            "RELAY1_PIN": 1
        })
        self.install_example_script_context("RCIN_test.lua")
        self.reboot_sitl()

        self.wait_parameter_value("SIM_PIN_MASK", 121)
        self.wait_parameter_value("SIM_PIN_MASK", 123)
        self.wait_parameter_value("SIM_PIN_MASK", 121)

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_print_home_and_origin(self):
        self.start_subtest("Scripting print home and origin")

        self.context_push()

        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("ahrs-print-home-and-origin.lua")
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.wait_statustext("Home - ")
        self.wait_statustext("Origin - ")

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_set_home_to_vehicle_location(self):
        self.start_subtest("Scripting set home to vehicle location")

        self.context_push()
        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("ahrs-set-home-to-vehicle-location.lua")
        self.reboot_sitl()

        self.wait_statustext("Home position reset")

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_serial_loopback(self):
        self.start_subtest("Scripting serial loopback test")

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_SDEV_EN": 1,
            "SCR_SDEV1_PROTO": 28,
        })
        self.install_test_script_context("serial_loopback.lua")
        self.reboot_sitl()

        for success_text in [
                "driver -> device good",
                "device -> driver good",
        ]:
            self.wait_statustext(success_text, check_context=True)

        self.context_pop()
        self.reboot_sitl()

    def Scripting(self):
        '''Scripting test'''
        self.test_scripting_set_home_to_vehicle_location()
        self.test_scripting_print_home_and_origin()
        self.test_scripting_hello_world()
        self.test_scripting_simple_loop()
        self.test_scripting_internal_test()
        self.test_scripting_auxfunc()
        self.test_scripting_serial_loopback()

    def test_mission_frame(self, frame, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                           target_system=target_system,
                           target_component=target_component)
        items = [
            # first item is ignored for missions
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        ]

        self.check_mission_upload_download(items)

    def MissionFrames(self, target_system=1, target_component=1):
        '''Upload/Download of items in different frames'''
        for frame in (mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL):
            self.test_mission_frame(frame,
                                    target_system=1,
                                    target_component=1)

    def mavlink_time_boot_ms(self):
        '''returns a time suitable for putting into the time_boot_ms entry in mavlink packets'''
        return int(time.time() * 1000000)

    def mavlink_time_boot_us(self):
        '''returns a time suitable for putting into the time_boot_ms entry in mavlink packets'''
        return int(time.time() * 1000000000)

    def ap_proximity_mav_obstacle_distance_send(self, data):
        increment = data.get("increment", 0)
        increment_f = data.get("increment_f", 0.0)
        max_distance = data["max_distance"]
        invalid_distance = max_distance + 1  # per spec
        distances = data["distances"][:]
        distances.extend([invalid_distance] * (72-len(distances)))
        self.mav.mav.obstacle_distance_send(
            self.mavlink_time_boot_us(),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            distances,
            increment,
            data["min_distance"],
            data["max_distance"],
            increment_f,
            data["angle_offset"],
            mavutil.mavlink.MAV_FRAME_BODY_FRD
        )

    def send_obstacle_distances_expect_distance_sensor_messages(self, obstacle_distances_in, expect_distance_sensor_messages):
        self.delay_sim_time(11)  # allow obstacles to time out
        self.do_timesync_roundtrip()
        expect_distance_sensor_messages_copy = expect_distance_sensor_messages[:]
        last_sent = 0
        while True:
            now = self.get_sim_time_cached()
            if now - last_sent > 1:
                self.progress("Sending")
                self.ap_proximity_mav_obstacle_distance_send(obstacle_distances_in)
                last_sent = now
            m = self.mav.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.delay_sim_time(1)
                continue
            orientation = m.orientation
            found = False
            if m.current_distance == m.max_distance:
                # ignored
                continue
            for expected_distance_sensor_message in expect_distance_sensor_messages_copy:
                if expected_distance_sensor_message["orientation"] != orientation:
                    continue
                found = True
                if not expected_distance_sensor_message.get("__found__", False):
                    self.progress("Marking message as found")
                    expected_distance_sensor_message["__found__"] = True
                if (m.current_distance - expected_distance_sensor_message["distance"] > 1):
                    raise NotAchievedException(
                        "Bad distance for orient=%u want=%u got=%u" %
                        (orientation, expected_distance_sensor_message["distance"], m.current_distance))
                break
            if not found:
                raise NotAchievedException("Got unexpected DISTANCE_SENSOR message")
            all_found = True
            for expected_distance_sensor_message in expect_distance_sensor_messages_copy:
                if not expected_distance_sensor_message.get("__found__", False):
                    self.progress("message still not found (orient=%u" % expected_distance_sensor_message["orientation"])
                    all_found = False
                    break
            if all_found:
                self.progress("Have now seen all expected messages")
                break

    def AP_Proximity_MAV(self):
        '''Test MAV proximity backend'''

        self.set_parameters({
            "PRX1_TYPE": 2,  # AP_Proximity_MAV
            "OA_TYPE": 2,  # dijkstra
            "OA_DB_OUTPUT": 3,  # send all items
        })
        self.reboot_sitl()

        # 1 laser pointing straight forward:
        self.send_obstacle_distances_expect_distance_sensor_messages(
            {
                "distances": [234],
                "increment_f": 10,
                "angle_offset": 0.0,
                "min_distance": 0,
                "max_distance": 1000, # cm
            }, [
                {"orientation": 0, "distance": 234},
            ])

        # 5 lasers at front of vehicle, spread over 40 degrees:
        self.send_obstacle_distances_expect_distance_sensor_messages(
            {
                "distances": [111, 222, 333, 444, 555],
                "increment_f": 10,
                "angle_offset": -20.0,
                "min_distance": 0,
                "max_distance": 1000, # cm
            }, [
                {"orientation": 0, "distance": 111},
            ])

        # lots of dense readings (e.g. vision camera:
        distances = [0] * 72
        for i in range(0, 72):
            distances[i] = 1000 + 10*abs(36-i)

        self.send_obstacle_distances_expect_distance_sensor_messages(
            {
                "distances": distances,
                "increment_f": 90/72.0,
                "angle_offset": -45.0,
                "min_distance": 0,
                "max_distance": 2000, # cm
            }, [
                {"orientation": 0, "distance": 1000},
                {"orientation": 1, "distance": 1190},
                {"orientation": 7, "distance": 1190},
            ])

    def SendToComponents(self):
        '''Test ArduPilot send_to_components function'''
        self.set_parameter("CAM1_TYPE", 5) # Camera with MAVlink trigger
        self.reboot_sitl() # needed for CAM1_TYPE to take effect
        self.progress("Introducing ourselves to the autopilot as a component")
        old_srcSystem = self.mav.mav.srcSystem
        self.mav.mav.srcSystem = 1
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0)

        self.progress("Sending control message")
        self.context_push()
        self.context_collect('COMMAND_LONG')
        self.mav.mav.digicam_control_send(
            1, # target_system
            1, # target_component
            1, # start or keep it up
            1, # zoom_pos
            0, # zoom_step
            0, # focus_lock
            0, # 1 shot or start filming
            17, # command id (de-dupe field)
            0, # extra_param
            0.0, # extra_value
        )
        self.mav.mav.srcSystem = old_srcSystem

        self.assert_received_message_field_values('COMMAND_LONG', {
            'command': mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
            'param6': 17,
        }, timeout=2, check_context=True)
        self.context_pop()

        # test sending via commands:
        for run_cmd in self.run_cmd, self.run_cmd_int:
            self.progress("Sending control command")
            self.context_push()
            self.context_collect('COMMAND_LONG')
            run_cmd(mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                    p1=1, # start or keep it up
                    p2=1, # zoom_pos
                    p3=0, # zoom_step
                    p4=0, # focus_lock
                    p5=0, # 1 shot or start filming
                    p6=37, # command id (de-dupe field)
                    )

            self.assert_received_message_field_values('COMMAND_LONG', {
                'command': mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                'param6': 37,
            }, timeout=2, check_context=True)

            self.context_pop()

        # test sending via commands:
        for run_cmd in self.run_cmd, self.run_cmd_int:
            self.progress("Sending configure command")
            self.context_push()
            self.context_collect('COMMAND_LONG')
            run_cmd(mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE,
                    p1=1,
                    p2=1,
                    p3=0,
                    p4=0,
                    p5=12,
                    p6=37
                    )

            self.assert_received_message_field_values('COMMAND_LONG', {
                'command': mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE,
                'param5': 12,
                'param6': 37,
            }, timeout=2, check_context=True)

            self.context_pop()

        self.mav.mav.srcSystem = old_srcSystem

    def SkidSteer(self):
        '''Check skid-steering'''
        model = "rover-skid"

        self.customise_SITL_commandline([],
                                        model=model,
                                        defaults_filepath=self.model_defaults_filepath(model))

        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("get a known heading to avoid worrying about wrap")
        # this is steering-type-two-paddles
        self.set_rc(1, 1400)
        self.set_rc(3, 1500)
        self.wait_heading(90)
        self.progress("straighten up")
        self.set_rc(1, 1500)
        self.set_rc(3, 1500)
        self.progress("steer one way")
        self.set_rc(1, 1600)
        self.set_rc(3, 1400)
        self.wait_heading(120)
        self.progress("steer the other")
        self.set_rc(1, 1400)
        self.set_rc(3, 1600)
        self.wait_heading(60)
        self.zero_throttle()
        self.disarm_vehicle()

    def SlewRate(self):
        """Test Motor Slew Rate feature."""
        self.context_push()
        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.start_subtest("Test no slew behavior")
        throttle_channel = 3
        throttle_max = 2000
        self.set_parameter("MOT_SLEWRATE", 0)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time > 0.5:
            raise NotAchievedException("Output response should be instant, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 100% slew rate")
        self.set_parameter("MOT_SLEWRATE", 100)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 0.9 or achieved_time > 1.1:
            raise NotAchievedException("Output response should be 1s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 50% slew rate")
        self.set_parameter("MOT_SLEWRATE", 50)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max, timeout=10)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 1.8 or achieved_time > 2.2:
            raise NotAchievedException("Output response should be 2s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 25% slew rate")
        self.set_parameter("MOT_SLEWRATE", 25)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max, timeout=10)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 3.6 or achieved_time > 4.4:
            raise NotAchievedException("Output response should be 4s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 10% slew rate")
        self.set_parameter("MOT_SLEWRATE", 10)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max, timeout=20)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 9 or achieved_time > 11:
            raise NotAchievedException("Output response should be 10s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!
        self.disarm_vehicle()
        self.context_pop()

    def SET_ATTITUDE_TARGET(self, target_sysid=None, target_compid=1):
        '''Test handling of SET_ATTITUDE_TARGET'''
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                raise AutoTestTimeoutException("Didn't get to speed")
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                target_sysid,
                target_compid,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
                mavextra.euler_to_quat([0,
                                        math.radians(0),
                                        math.radians(0)]), # att
                0, # yaw rate (rad/s)
                0, # pitch rate
                0, # yaw rate
                1) # thrust

            msg = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            if msg is None:
                raise NotAchievedException("No VFR_HUD message")
            if msg.groundspeed > 5:
                break
        self.disarm_vehicle()

    def SET_ATTITUDE_TARGET_heading(self, target_sysid=None, target_compid=1):
        '''Test handling of SET_ATTITUDE_TARGET'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        for angle in 0, 290, 70, 180, 0:
            self.SET_ATTITUDE_TARGET_heading_test_target(angle, target_sysid, target_compid)
        self.disarm_vehicle()

    def SET_ATTITUDE_TARGET_heading_test_target(self, angle, target_sysid, target_compid):
        if target_sysid is None:
            target_sysid = self.sysid_thismav()

        def poke_set_attitude(value, target):
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                target_sysid,
                target_compid,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
                mavextra.euler_to_quat([
                    math.radians(0),
                    math.radians(0),
                    math.radians(angle)
                ]), # att
                0, # roll rate (rad/s)
                0, # pitch rate
                0, # yaw rate
                1) # thrust

        self.wait_heading(angle, called_function=poke_set_attitude, minimum_duration=5)

    def SET_POSITION_TARGET_LOCAL_NED(self, target_sysid=None, target_compid=1):
        '''Test handling of SET_POSITION_TARGET_LOCAL_NED'''
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        ofs_x = 30.0
        ofs_y = 30.0

        def send_target():
            self.mav.mav.set_position_target_local_ned_send(
                0, # time_boot_ms
                target_sysid,
                target_compid,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
                ofs_x,  # pos-x
                ofs_y,  # pos-y
                0,     # pos-z
                0,     # vel-x
                0,     # vel-y
                0,     # vel-z
                0,     # acc-x
                0,     # acc-y
                0,     # acc-z
                0,     # yaw
                0,     # yaw rate
            )

        self.wait_distance_to_local_position(
            (ofs_x, ofs_y, 0),
            distance_min=0,
            distance_max=3,
            timeout=60,
            called_function=lambda last_value, target : send_target(),
            minimum_duration=5,  # make sure we stop!
        )

        self.do_RTL()
        self.disarm_vehicle()

    def EndMissionBehavior(self, timeout=60):
        '''Test end mission behavior'''
        self.context_push()

        self.load_mission("end-mission.txt")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.start_subtest("Test End Mission Behavior HOLD")
        self.context_collect("STATUSTEXT")
        self.change_mode("AUTO")
        self.wait_text("Mission Complete", check_context=True, wallclock_timeout=2)
        # On Hold we should just stop and don't update the navigation target anymore
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 15:
                raise AutoTestTimeoutException("Still getting POSITION_TARGET_GLOBAL_INT")
            m = self.mav.recv_match(type="POSITION_TARGET_GLOBAL_INT",
                                    blocking=True,
                                    timeout=10)
            if m is None:
                self.progress("No POSITION_TARGET_GLOBAL_INT received, all good !")
                break
        self.context_clear_collection("STATUSTEXT")
        self.change_mode("GUIDED")
        self.context_collect("STATUSTEXT")

        self.start_subtest("Test End Mission Behavior LOITER")
        self.set_parameter("MIS_DONE_BEHAVE", 1)
        self.change_mode("AUTO")
        self.wait_text("Mission Complete", check_context=True, wallclock_timeout=2)
        # On LOITER we should update the navigation target
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 15:
                raise AutoTestTimeoutException("Not getting POSITION_TARGET_GLOBAL_INT")
            m = self.mav.recv_match(type="POSITION_TARGET_GLOBAL_INT",
                                    blocking=True,
                                    timeout=5)
            if m is None:
                self.progress("No POSITION_TARGET_GLOBAL_INT received")
                continue
            else:
                if self.get_sim_time_cached() - tstart > 15:
                    self.progress("Got POSITION_TARGET_GLOBAL_INT, all good !")
                    break

        self.start_subtest("Test End Mission Behavior ACRO")
        self.set_parameter("MIS_DONE_BEHAVE", 2)
        # race conditions here to do with get_sim_time()
        # swallowing heartbeats means we have to be a little
        # circuitous when testing here:
        self.change_mode("GUIDED")
        self.send_cmd_do_set_mode('AUTO')
        self.wait_mode("ACRO")

        self.start_subtest("Test End Mission Behavior MANUAL")
        self.set_parameter("MIS_DONE_BEHAVE", 3)
        # race conditions here to do with get_sim_time()
        # swallowing heartbeats means we have to be a little
        # circuitous when testing here:
        self.change_mode("GUIDED")
        self.send_cmd_do_set_mode("AUTO")
        self.wait_mode("MANUAL")
        self.disarm_vehicle()

        self.context_pop()
        self.reboot_sitl()

    def MAVProxyParam(self):
        '''Test MAVProxy parameter handling'''
        mavproxy = self.start_mavproxy()
        mavproxy.send("param fetch\n")
        mavproxy.expect("Received [0-9]+ parameters")
        self.stop_mavproxy(mavproxy)

    def MAV_CMD_DO_SET_MISSION_CURRENT_mission(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        ])

    def MAV_CMD_DO_SET_MISSION_CURRENT(self, target_sysid=None, target_compid=1):
        '''Test handling of CMD_DO_SET_MISSION_CURRENT'''
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        self.check_mission_upload_download(self.MAV_CMD_DO_SET_MISSION_CURRENT_mission())

        self.set_current_waypoint(2)

        self.set_current_waypoint_using_mav_cmd_do_set_mission_current(2)

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            p1=17,
            timeout=1,
            target_sysid=target_sysid,
            target_compid=target_compid,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

    def FlashStorage(self):
        '''Test flash storage (for parameters etc)'''
        self.set_parameter("LOG_BITMASK", 1)
        self.reboot_sitl()

        self.customise_SITL_commandline([
            "--set-storage-posix-enabled", "0",
            "--set-storage-flash-enabled", "1",
        ])
        if self.get_parameter("LOG_BITMASK") == 1:
            raise NotAchievedException("not using flash storage?")
        self.set_parameter("LOG_BITMASK", 2)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 2)
        self.set_parameter("LOG_BITMASK", 3)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 3)

        self.customise_SITL_commandline([])
        # make sure we're back at our original value:
        self.assert_parameter_value("LOG_BITMASK", 1)

    def FRAMStorage(self):
        '''Test FRAM storage (for parameters etc)'''
        self.set_parameter("LOG_BITMASK", 1)
        self.reboot_sitl()

        self.customise_SITL_commandline([
            "--set-storage-posix-enabled", "0",
            "--set-storage-fram-enabled", "1",
        ])
        # TODO: ensure w'ere actually taking stuff from flash storage:
#        if self.get_parameter("LOG_BITMASK") == 1:
#            raise NotAchievedException("not using flash storage?")
        self.set_parameter("LOG_BITMASK", 2)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 2)
        self.set_parameter("LOG_BITMASK", 3)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 3)

        self.customise_SITL_commandline([])
        # make sure we're back at our original value:
        self.assert_parameter_value("LOG_BITMASK", 1)

    def RangeFinder(self):
        '''Test RangeFinder'''
        # the following magic numbers correspond to the post locations in SITL
        home_string = "%s,%s,%s,%s" % (51.8752066, 14.6487840, 54.15, 231)

        rangefinder_params = {
            "SIM_SONAR_ROT": 0,
        }
        rangefinder_params.update(self.analog_rangefinder_parameters())

        self.set_parameters(rangefinder_params)
        self.customise_SITL_commandline([
            "--home", home_string,
        ])
        self.wait_ready_to_arm()
        if self.mavproxy is not None:
            self.mavproxy.send('script /tmp/post-locations.scr\n')
        m = self.assert_receive_message('RANGEFINDER', very_verbose=True)
        if m.voltage == 0:
            raise NotAchievedException("Did not get non-zero voltage")
        want_range = 10
        if abs(m.distance - want_range) > 0.5:
            raise NotAchievedException("Expected %fm got %fm" % (want_range, m.distance))

    def DepthFinder(self):
        '''Test mulitple depthfinders for boats'''
        # Setup rangefinders
        self.customise_SITL_commandline([
            "--serial7=sim:nmea", # NMEA Rangefinder
        ])

        # RANGEFINDER_INSTANCES = [0, 2, 5]
        self.set_parameters({
            "RNGFND1_TYPE" : 17,     # NMEA must attach uart to SITL
            "RNGFND1_ORIENT" : 25,   # Set to downward facing
            "SERIAL7_PROTOCOL" : 9,  # Rangefinder on serial7
            "SERIAL7_BAUD" : 9600,   # Rangefinder specific baudrate

            "RNGFND3_TYPE" : 2,      # MaxbotixI2C
            "RNGFND3_ADDR" : 112,    # 0x70 address from SIM_I2C.cpp
            "RNGFND3_ORIENT" : 0,    # Set to forward facing, thus we should not receive DPTH messages from this one

            "RNGFND6_ADDR" : 113,    # 0x71 address from SIM_I2C.cpp
            "RNGFND6_ORIENT" : 25,   # Set to downward facing
            "RNGFND6_TYPE" : 2,      # MaxbotixI2C
        })

        self.reboot_sitl()
        self.wait_ready_to_arm()

        # should not get WATER_DEPTH messages or DPTH logs when the FRAME_CLASS is not a boat
        m = self.mav.recv_match(type="WATER_DEPTH", blocking=True, timeout=2)
        if m is not None:
            raise NotAchievedException("WATER_DEPTH: received message when FRAME_CLASS not a Boat")

        # Set FRAME_CLASS to start receiving WATER_DEPTH messages & logging DPTH
        self.set_parameters({
            "FRAME_CLASS": 2,       # Boat
        })

        # Check each rangefinder instance is in collection
        rangefinder = [None, None, None, None, None, None] # Be lazy FIXME only need [3]

        def check_rangefinder(mav, m):
            if m.get_type() != 'WATER_DEPTH':
                return

            id = m.id

            # Should not find instance 3 as it is forward facing
            if id == 2:
                raise NotAchievedException("Depthfinder Instance %i with non-downward orientation found" % (id))

            rangefinder[id] = True

            if id == 0:
                if float(m.temperature) == 0.0:
                    raise NotAchievedException("Depthfinder Instance %i NMEA with temperature not found" % (id))
            elif id == 5:
                if float(m.temperature) != 0.0:
                    raise NotAchievedException("Depthfinder Instance %i should not have temperature" % (id))

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.install_message_hook_context(check_rangefinder)
        self.drive_mission("rover1.txt", strict=False)

        if rangefinder[0] is None:
            raise NotAchievedException("Never saw Depthfinder 1")
        if rangefinder[2] is not None:
            raise NotAchievedException("Should not have found a Depthfinder 3")
        if rangefinder[5] is None:
            raise NotAchievedException("Never saw Depthfinder 6")
        if not self.current_onboard_log_contains_message("DPTH"):
            raise NotAchievedException("Expected DPTH log message")

        # self.context_pop()

    def EStopAtBoot(self):
        '''Ensure EStop prevents arming when asserted at boot time'''
        self.context_push()
        self.set_parameters({
            "RC9_OPTION": 31,
        })
        self.set_rc(9, 2000)
        self.reboot_sitl()
        self.assert_prearm_failure(
            "Motors Emergency Stopped",
            other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()

    def assert_mode(self, mode):
        if not self.mode_is(mode):
            raise NotAchievedException("Mode is not %s" % str(mode))

    def ChangeModeByNumber(self):
        '''ensure we can set a mode by number, handy when we don't have a
        pymavlink number for it yet'''
        for (x, want) in (0, 'MANUAL'), (1, 'ACRO'), (3, 3):
            self.change_mode(x)
            self.assert_mode(want)

    def StickMixingAuto(self):
        '''Ensure Stick Mixing works in auto'''
        items = []
        self.set_parameter('STICK_MIXING', 1)
        # home
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0),)
        # 1 waypoint a long way away
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 0),)
        self.upload_simple_relhome_mission(items)
        if self.mavproxy is not None:
            # handy for getting pretty pictures
            self.mavproxy.send("wp list\n")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(1, 1150)
        self.wait_heading(45)
        self.wait_heading(90)
        self.disarm_vehicle()

    def AutoDock(self):
        '''Test automatic docking of rover for multiple FOVs of simulated beacon'''
        self.set_parameters({
            "PLND_ENABLED": 1,
            "PLND_TYPE": 4,
            "PLND_ORIENT": 0,
        })

        start = self.mav.location()
        target = self.offset_location_ne(start, 50, 0)
        self.progress("Setting target to %f %f" % (start.lat, start.lng))
        stopping_dist = 0.5

        self.set_parameters({
            "SIM_PLD_ENABLE": 1,
            "SIM_PLD_LAT": target.lat,
            "SIM_PLD_LON": target.lng,
            "SIM_PLD_HEIGHT": 0,
            "SIM_PLD_ALT_LMT": 30,
            "SIM_PLD_DIST_LMT": 30,
            "SIM_PLD_ORIENT": 4,    # emit beams towards south, vehicle's heading must be north to see it
            "SIM_PLD_OPTIONS": 1,
            "DOCK_SPEED": 2,
            "DOCK_STOP_DIST": stopping_dist,
        })

        for type in range(0, 3):  # CYLINDRICAL FOV, CONICAL FOV, SPHERICAL FOV
            self.set_parameter("SIM_PLD_TYPE", type)
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            initial_position = self.offset_location_ne(target, -20, -2)
            self.drive_to_location(initial_position)
            self.change_mode(8) # DOCK mode
            max_delta = 1
            self.wait_distance_to_location(target, 0, max_delta, timeout=180)
            self.disarm_vehicle()
            self.assert_receive_message('GLOBAL_POSITION_INT')
            new_pos = self.mav.location()
            delta = abs(self.get_distance(target, new_pos) - stopping_dist)
            self.progress("Docked %f metres from stopping point" % delta)
            if delta > max_delta:
                raise NotAchievedException("Did not dock close enough to stopping point (%fm > %fm" % (delta, max_delta))

            if not self.current_onboard_log_contains_message("PL"):
                raise NotAchievedException("Did not see expected PL message")

        self.progress("All done")

    def PrivateChannel(self):
        '''test the serial option bit specifying a mavlink channel as private'''
        global mav2
        port = self.adjust_ardupilot_port(5763)
        mav2 = mavutil.mavlink_connection("tcp:localhost:%u" % port,
                                          robust_parsing=True,
                                          source_system=7,
                                          source_component=7)
        # send a heartbeat or two to make sure ArduPilot's aware:

        def heartbeat_on_mav2(mav, m):
            '''send a heartbeat on mav2 whenever we get one on mav'''
            global mav2
            if mav == mav2:
                return
            if m.get_type() == 'HEARTBEAT':
                mav2.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0)
                return

        self.assert_receive_message("HEARTBEAT", mav=mav2)

        # ensure a targetted message is received:
        self.install_message_hook_context(heartbeat_on_mav2)

        self.progress("Ensuring we can get a message normally")
        self.poll_message("AUTOPILOT_VERSION", mav=mav2)

        self.progress("Polling AUTOPILOT_VERSION from random sysid")
        self.send_poll_message("AUTOPILOT_VERSION", mav=mav2, target_sysid=134)
        self.assert_not_receive_message("AUTOPILOT_VERSION", mav=mav2, timeout=10)

        # make sure we get heartbeats on the main channel from the non-private mav2:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                raise NotAchievedException("Did not get expected heartbeat from %u" % 7)
            m = self.assert_receive_message("HEARTBEAT")
            if m.get_srcSystem() == 7:
                self.progress("Got heartbeat from (%u) on non-private channel" % 7)
                break

        # make sure we receive heartbeats from the autotest suite into
        # the component:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                raise NotAchievedException("Did not get expected heartbeat from %u" % self.mav.source_system)
            m = self.assert_receive_message("HEARTBEAT", mav=mav2)
            if m.get_srcSystem() == self.mav.source_system:
                self.progress("Got heartbeat from (%u) on non-private channel" % self.mav.source_system)
                break

        def printmessage(mav, m):
            global mav2
            if mav == mav2:
                return

            print("Got (%u/%u) (%s) " % (m.get_srcSystem(), m.get_srcComponent(), str(m)))

#        self.install_message_hook_context(printmessage)

        # ensure setting the private channel mask doesn't cause us to
        # execute these commands:
        self.set_parameter("SERIAL2_OPTIONS", 1024)
        self.reboot_sitl()  # mavlink-private is reboot-required
        mav2 = mavutil.mavlink_connection("tcp:localhost:5763",
                                          robust_parsing=True,
                                          source_system=7,
                                          source_component=7)
#        self.send_debug_trap()
        self.send_poll_message("AUTOPILOT_VERSION", mav=mav2, target_sysid=134)
        self.assert_not_receive_message("AUTOPILOT_VERSION", mav=mav2, timeout=10)

        # make sure messages from a private channel don't make it to
        # the main channel:
        self.drain_mav(self.mav)
        self.drain_mav(mav2)

        # make sure we do NOT get heartbeats on the main channel from
        # the private mav2:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                break
            m = self.assert_receive_message("HEARTBEAT")
            if m.get_srcSystem() == 7:
                raise NotAchievedException("Got heartbeat from private channel")

        self.progress("ensure no outside heartbeats reach private channels")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                break
            m = self.assert_receive_message("HEARTBEAT")
            if m.get_srcSystem() == 1 and m.get_srcComponent() == 1:
                continue
            # note the above test which shows we get heartbeats from
            # both the vehicle and this tests's special heartbeat
            raise NotAchievedException("Got heartbeat on private channel from non-vehicle")

    def MAV_CMD_DO_SET_REVERSE(self):
        '''test MAV_CMD_DO_SET_REVERSE command'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        here = self.mav.location()
        target_loc = self.offset_location_ne(here, 2000, 0)
        self.send_guided_mission_item(target_loc)

        self.wait_groundspeed(3, 100, minimum_duration=5)

        for method in self.run_cmd, self.run_cmd_int:
            self.progress("Forwards!")
            method(mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, p1=0)
            self.wait_heading(0)

            self.progress("Backwards!")
            method(mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, p1=1)
            self.wait_heading(180)

            self.progress("Forwards!")
            method(mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, p1=0)
            self.wait_heading(0)

        self.disarm_vehicle()

    def MAV_CMD_NAV_RETURN_TO_LAUNCH(self):
        '''test MAV_CMD_NAV_RETURN_TO_LAUNCH mavlink command'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        here = self.mav.location()
        target_loc = self.offset_location_ne(here, 2000, 0)
        self.send_guided_mission_item(target_loc)
        self.wait_distance_to_home(20, 100)

        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self.wait_mode('RTL')

        self.change_mode('GUIDED')

        self.run_cmd_int(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self.wait_mode('RTL')

        self.wait_distance_to_home(0, 5, timeout=30)
        self.disarm_vehicle()

    def MAV_CMD_DO_CHANGE_SPEED(self):
        '''test MAV_CMD_NAV_RETURN_TO_LAUNCH mavlink command'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        original_loc = self.mav.location()
        here = original_loc
        target_loc = self.offset_location_ne(here, 2000, 0)
        self.send_guided_mission_item(target_loc)
        self.wait_distance_to_home(20, 100)

        speeds = 3, 7, 12, 4

        for speed in speeds:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, p2=speed)
            self.wait_groundspeed(speed-0.5, speed+0.5, minimum_duration=5)

        self.send_guided_mission_item(original_loc)

        for speed in speeds:
            self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, p2=speed)
            self.wait_groundspeed(speed-0.5, speed+0.5, minimum_duration=5)

        self.change_mode('RTL')

        self.wait_distance_to_home(0, 5, timeout=30)
        self.disarm_vehicle()

    def MAV_CMD_MISSION_START(self):
        '''simple test for starting missing using this command'''
        # home and 1 waypoint a long way away:
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        for method in self.run_cmd, self.run_cmd_int:
            self.change_mode('MANUAL')
            self.wait_groundspeed(0, 1)
            method(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_mode('AUTO')
            self.wait_groundspeed(3, 100)
        self.disarm_vehicle()

    def MAV_CMD_NAV_SET_YAW_SPEED(self):
        '''tests for MAV_CMD_NAV_SET_YAW_SPEED guided-mode command'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        for method in self.run_cmd, self.run_cmd_int:
            self.change_mode('MANUAL')
            self.wait_groundspeed(0, 1)
            self.change_mode('GUIDED')
            self.start_subtest("Absolute angles")
            for (heading, speed) in (10, 5), (190, 10), (0, 2), (135, 6):
                def cf(*args, **kwargs):
                    method(
                        mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
                        p1=heading,
                        p2=speed,
                        p3=0,  # zero is absolute-angles
                    )
                self.wait_groundspeed(speed-0.5, speed+0.5, called_function=cf, minimum_duration=2)
                self.wait_heading(heading-0.5, heading+0.5, called_function=cf, minimum_duration=2)

            self.start_subtest("relative angles")
            original_angle = 90
            method(
                mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
                p1=original_angle,
                p2=5,
                p3=0,  # zero is absolute-angles
            )
            self.wait_groundspeed(4, 6)
            self.wait_heading(original_angle-0.5, original_angle+0.5)

            expected_angle = original_angle
            for (angle_delta, speed) in (5, 6), (-30, 2), (180, 7):
                method(
                    mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
                    p1=angle_delta,
                    p2=speed,
                    p3=1,  # one is relative-angles
                )

                def cf(*args, **kwargs):
                    method(
                        mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
                        p1=0,
                        p2=speed,
                        p3=1,  # one is absolute-angles
                    )
                expected_angle += angle_delta
                if expected_angle < 0:
                    expected_angle += 360
                if expected_angle > 360:
                    expected_angle -= 360
                self.wait_groundspeed(speed-0.5, speed+0.5, called_function=cf, minimum_duration=2)
                self.wait_heading(expected_angle, called_function=cf, minimum_duration=2)
        self.do_RTL()
        self.disarm_vehicle()

    def _MAV_CMD_GET_HOME_POSITION(self, run_cmd):
        '''test handling of mavlink command MAV_CMD_GET_HOME_POSITION'''
        self.context_collect('HOME_POSITION')
        run_cmd(mavutil.mavlink.MAV_CMD_GET_HOME_POSITION)
        self.assert_receive_message('HOME_POSITION', check_context=True)

    def MAV_CMD_GET_HOME_POSITION(self):
        '''test handling of mavlink command MAV_CMD_GET_HOME_POSITION'''
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self._MAV_CMD_GET_HOME_POSITION(self.run_cmd)
        self._MAV_CMD_GET_HOME_POSITION(self.run_cmd_int)

    def MAV_CMD_DO_FENCE_ENABLE(self):
        '''ensure MAV_CMD_DO_FENCE_ENABLE mavlink command works'''
        here = self.mav.location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])

        # enable:
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, p1=1)
        self.assert_fence_enabled()

        # disable
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, p1=0)
        self.assert_fence_disabled()

    def MAV_CMD_BATTERY_RESET(self):
        '''manipulate battery levels with MAV_CMD_BATTERY_RESET'''
        for (run_cmd, value) in (self.run_cmd, 56), (self.run_cmd_int, 97):
            run_cmd(
                mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                p1=65535,  # battery mask
                p2=value,
            )
            self.assert_received_message_field_values('BATTERY_STATUS', {
                "battery_remaining": value,
            }, {
                "poll": True,
            })

    def TestWebServer(self, url):
        '''test active web server'''
        self.progress("Accessing webserver main page")
        import urllib.request

        main_page = urllib.request.urlopen(url).read().decode('utf-8')
        if main_page.find('ArduPilot Web Server') == -1:
            raise NotAchievedException("Expected banner on main page")

        board_status = urllib.request.urlopen(url + '/@DYNAMIC/board_status.shtml').read().decode('utf-8')
        if board_status.find('0 hours') == -1:
            raise NotAchievedException("Expected uptime in board status")
        if board_status.find('40.713') == -1:
            raise NotAchievedException("Expected lattitude in board status")

        self.progress("WebServer tests OK")

    def NetworkingWebServer(self):
        '''web server'''
        applet_script = "net_webserver.lua"

        self.context_push()
        self.install_applet_script_context(applet_script)

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "SIM_SPEEDUP": 20,
            "NET_ENABLE": 1,
        })

        self.reboot_sitl()

        self.context_push()
        self.context_collect('STATUSTEXT')

        self.set_parameters({
            "WEB_BIND_PORT": 8081,
        })

        self.scripting_restart()
        self.wait_text("WebServer: starting on port 8081", check_context=True)

        self.wait_ready_to_arm()

        self.TestWebServer("http://127.0.0.1:8081")

        self.context_pop()
        self.context_pop()
        self.reboot_sitl()

    def NetworkingWebServerPPP(self):
        '''web server over PPP'''
        applet_script = "net_webserver.lua"

        self.context_push()
        self.install_applet_script_context(applet_script)

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "SIM_SPEEDUP": 20,
            "NET_ENABLE": 1,
            "SERIAL5_PROTOCOL": 48,
        })

        self.progress('rebuilding rover with ppp enabled')
        import shutil
        shutil.copy('build/sitl/bin/ardurover', 'build/sitl/bin/ardurover.noppp')
        util.build_SITL('bin/ardurover', clean=False, configure=True, extra_configure_args=['--enable-ppp', '--debug'])

        self.reboot_sitl()

        self.progress("Starting PPP daemon")
        pppd = util.start_PPP_daemon("192.168.14.15:192.168.14.13", '127.0.0.1:5765')

        self.context_push()
        self.context_collect('STATUSTEXT')

        pppd.expect("remote IP address 192.168.14.13")

        self.progress("PPP daemon started")

        self.set_parameters({
            "WEB_BIND_PORT": 8081,
        })

        self.scripting_restart()
        self.wait_text("WebServer: starting on port 8081", check_context=True)

        self.wait_ready_to_arm()

        self.TestWebServer("http://192.168.14.13:8081")

        self.context_pop()
        self.context_pop()

        # restore rover without ppp enabled for next test
        os.unlink('build/sitl/bin/ardurover')
        shutil.copy('build/sitl/bin/ardurover.noppp', 'build/sitl/bin/ardurover')
        self.reboot_sitl()

    def FenceFullAndPartialTransfer(self, target_system=1, target_component=1):
        '''ensure starting a fence transfer then a partial transfer behaves
        appropriately'''
        # start uploading a 10 item list:
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, 0)
        # change our mind and try a partial mission upload:
        self.mav.mav.mission_write_partial_list_send(
            target_system,
            target_component,
            3,
            3,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        # should get denied for that one:
        self.assert_receive_mission_ack(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            want_type=mavutil.mavlink.MAV_MISSION_DENIED,
        )
        # now wait for the original upload to be "cancelled"
        self.assert_receive_mission_ack(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            want_type=mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED,
        )

    def MissionRetransfer(self, target_system=1, target_component=1):
        '''torture-test with MISSION_COUNT'''
#        self.send_debug_trap()
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, 0)
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10000,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.wait_statustext('Only [0-9]+ items are supported', regex=True, check_context=True)
        self.context_pop()
        self.assert_not_receive_message('MISSION_REQUEST')
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, 0)
        self.assert_receive_mission_ack(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            want_type=mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED,
        )

    def MissionPolyEnabledPreArm(self):
        '''check Polygon porearm checks'''
        self.set_parameters({
            'FENCE_ENABLE': 1,
        })
        self.progress("Ensure that we can arm if polyfence is enabled but we have no polyfence")
        self.assert_parameter_value('FENCE_TYPE', 6)
        self.wait_ready_to_arm()
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.progress("Ensure we can arm when we have an inclusion fence we are inside of")
        here = self.mav.location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.delay_sim_time(5)
        self.wait_ready_to_arm()

        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.progress("Ensure we can't arm when we are in breacnh of a polyfence")
        self.clear_fence()

        self.progress("Now create a fence we are in breach of")
        here = self.mav.location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, 20, 20), # bl
                self.offset_location_ne(here, 20, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, 20), # tl,
            ]),
        ])

        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False)
        self.reboot_sitl()

        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)

        self.progress("Ensure we can arm when a polyfence fence is cleared when we've previously been in breach")
        self.clear_fence()
        self.wait_ready_to_arm()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, 20, 20), # bl
                self.offset_location_ne(here, 20, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, 20), # tl,
            ]),
        ])
        self.reboot_sitl()
        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)
        self.clear_fence()
        self.wait_ready_to_arm()

        self.progress("Ensure we can arm after clearing polygon fence type enabled")
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, 20, 20), # bl
                self.offset_location_ne(here, 20, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, 20), # tl,
            ]),
        ])
        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)
        self.set_parameter('FENCE_TYPE', 2)
        self.wait_ready_to_arm()
        self.set_parameter('FENCE_TYPE', 6)
        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)

    def OpticalFlow(self):
        '''lightly test OpticalFlow'''
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, False, False, False, verbose=True)

        self.context_push()
        self.set_parameter("SIM_FLOW_ENABLE", 1)
        self.set_parameter("FLOW_TYPE", 10)

        self.reboot_sitl()
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, True, True, True, verbose=True)

        self.context_pop()
        self.reboot_sitl()

    def RCDuplicateOptionsExist(self):
        '''ensure duplicate RC option detection works'''
        self.wait_ready_to_arm()
        self.set_parameters({
            "RC6_OPTION": 118,
            "RC7_OPTION": 118,
        })
        self.assert_arm_failure("Duplicate Aux Switch Options")

    def JammingSimulation(self):
        '''Test jamming simulation works'''
        self.wait_ready_to_arm()
        start_loc = self.assert_receive_message('GPS_RAW_INT')
        self.set_parameter("SIM_GPS_JAM", 1)

        class Requirement():
            def __init__(self, field, min_value):
                self.field = field
                self.min_value = min_value

            def met(self, m):
                return getattr(m, self.field) > self.min_value

        requirements = set([
            Requirement('v_acc', 50000),
            Requirement('h_acc', 50000),
            Requirement('vel_acc', 1000),
            Requirement('vel', 10000),
        ])
        low_sats_seen = False
        seen_bad_loc = False
        tstart = self.get_sim_time()

        while True:
            if self.get_sim_time() - tstart > 120:
                raise NotAchievedException("Did not see all jamming")
            m = self.assert_receive_message('GPS_RAW_INT')
            new_requirements = copy.copy(requirements)
            for requirement in requirements:
                if requirement.met(m):
                    new_requirements.remove(requirement)
            requirements = new_requirements
            if m.satellites_visible < 6:
                low_sats_seen = True
            here = self.assert_receive_message('GPS_RAW_INT')
            if self.get_distance_int(start_loc, here) > 100:
                seen_bad_loc = True

            if len(requirements) == 0 and low_sats_seen and seen_bad_loc:
                break

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestRover, self).tests()

        ret.extend([
            self.MAVProxy_SetModeUsingSwitch,
            self.HIGH_LATENCY2,
            self.MAVProxy_SetModeUsingMode,
            self.ModeSwitch,
            self.AuxModeSwitch,
            self.DriveRTL,
            self.SmartRTL,
            self.DriveSquare,
            self.DriveMission,
            # self.DriveBrake,  # disabled due to frequent failures
            self.MAV_CMD_DO_SEND_BANNER,
            self.DO_SET_MODE,
            self.MAVProxy_DO_SET_MODE,
            self.ServoRelayEvents,
            self.RCOverrides,
            self.RCOverridesCancel,
            self.MANUAL_CONTROL,
            self.Sprayer,
            self.AC_Avoidance,
            self.CameraMission,
            self.Gripper,
            self.GripperMission,
            self.SET_MESSAGE_INTERVAL,
            self.MESSAGE_INTERVAL_COMMAND_INT,
            self.REQUEST_MESSAGE,
            self.SYSID_ENFORCE,
            self.SET_ATTITUDE_TARGET,
            self.SET_ATTITUDE_TARGET_heading,
            self.SET_POSITION_TARGET_LOCAL_NED,
            self.MAV_CMD_DO_SET_MISSION_CURRENT,
            self.MAV_CMD_DO_CHANGE_SPEED,
            self.MAV_CMD_MISSION_START,
            self.MAV_CMD_NAV_SET_YAW_SPEED,
            self.Button,
            self.Rally,
            self.Offboard,
            self.MAVProxyParam,
            self.GCSFence,
            self.GCSMission,
            self.GCSRally,
            self.MotorTest,
            self.WheelEncoders,
            self.DataFlashOverMAVLink,
            self.DataFlash,
            self.SkidSteer,
            self.PolyFence,
            self.PolyFenceAvoidance,
            self.PolyFenceObjectAvoidanceAuto,
            self.PolyFenceObjectAvoidanceGuided,
            self.PolyFenceObjectAvoidanceBendyRuler,
            self.SendToComponents,
            self.PolyFenceObjectAvoidanceBendyRulerEasierGuided,
            self.PolyFenceObjectAvoidanceBendyRulerEasierAuto,
            self.SlewRate,
            self.Scripting,
            self.ScriptingSteeringAndThrottle,
            self.MissionFrames,
            self.SetpointGlobalPos,
            self.SetpointGlobalVel,
            self.AccelCal,
            self.RangeFinder,
            self.AP_Proximity_MAV,
            self.EndMissionBehavior,
            self.FlashStorage,
            self.FRAMStorage,
            self.DepthFinder,
            self.ChangeModeByNumber,
            self.EStopAtBoot,
            self.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            self.StickMixingAuto,
            self.AutoDock,
            self.PrivateChannel,
            self.GCSFailsafe,
            self.RoverInitialMode,
            self.DriveMaxRCIN,
            self.NoArmWithoutMissionItems,
            self.CompassPrearms,
            self.MAV_CMD_DO_SET_REVERSE,
            self.MAV_CMD_GET_HOME_POSITION,
            self.MAV_CMD_DO_FENCE_ENABLE,
            self.MAV_CMD_BATTERY_RESET,
            self.NetworkingWebServer,
            self.NetworkingWebServerPPP,
            self.RTL_SPEED,
            self.MissionRetransfer,
            self.FenceFullAndPartialTransfer,
            self.MissionPolyEnabledPreArm,
            self.OpticalFlow,
            self.RCDuplicateOptionsExist,
            self.ClearMission,
            self.JammingSimulation,
        ])
        return ret

    def disabled_tests(self):
        return {
            "SlewRate": "got timing report failure on CI",
            "MAV_CMD_NAV_SET_YAW_SPEED": "compiled out of code by default",
            "PolyFenceObjectAvoidanceBendyRuler": "unreliable",
        }

    def rc_defaults(self):
        ret = super(AutoTestRover, self).rc_defaults()
        ret[3] = 1500
        ret[8] = 1800
        return ret

    def initial_mode_switch_mode(self):
        return "MANUAL"

    def default_mode(self):
        return 'MANUAL'
