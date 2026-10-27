'''
Test AntennaTracker vehicle in SITL

AP_FLAKE8_CLEAN

'''

import math
import operator
import os

from pymavlink import mavextra
from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = Location(-27.274439, 151.290064, 343, AltFrame.ABSOLUTE)
SITL_START_HEADING = 8.7


class AutoTestTracker(vehicle_test_suite.TestSuite):

    def log_name(self):
        return "AntennaTracker"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    def default_mode(self):
        return "AUTO"

    def is_tracker(self):
        return True

    def default_frame(self):
        return "tracker"

    def set_current_test_name(self, name):
        self.current_test_name_directory = "AntennaTracker_Tests/" + name + "/"

    def apply_defaultfile_parameters(self):
        # tracker doesn't have a default parameters file
        pass

    def sysid_thismav(self):
        return 2

    def achieve_attitude(self, desyaw, despitch, tolerance=1, target_system=2, target_component=1):
        '''use set_attitude_target to achieve desyaw / despitch'''
        tstart = self.get_sim_time()
        last_attitude_target_sent = 0
        last_debug = 0
        self.progress("Using set_attitude_target to achieve attitude")
        while True:
            now = self.get_sim_time()
            if now - tstart > 60:
                raise NotAchievedException("Did not achieve attitude")
            if now - last_attitude_target_sent > 0.5:
                last_attitude_target_sent = now
                type_mask = (
                    1 << 0 | # ignore roll rate
                    1 << 6 # ignore throttle
                )
                self.mav.mav.set_attitude_target_send(
                    0, # time_boot_ms
                    target_system, # target sysid
                    target_component, # target compid
                    type_mask, # bitmask of things to ignore
                    mavextra.euler_to_quat([0,
                                            math.radians(despitch),
                                            math.radians(desyaw)]), # att
                    0, # yaw rate (rad/s)
                    0, # pitch rate
                    0, # yaw rate
                    0) # thrust, 0 to 1, translated to a climb/descent rate
            m = self.assert_receive_message('ATTITUDE', timeout=2)
            if now - last_debug > 1:
                last_debug = now
                self.progress("yaw=%f desyaw=%f pitch=%f despitch=%f" %
                              (math.degrees(m.yaw), desyaw,
                               math.degrees(m.pitch), despitch))
            yaw_ok = abs(math.degrees(m.yaw) - desyaw) < tolerance
            pitch_ok = abs(math.degrees(m.pitch) - despitch) < tolerance
            if yaw_ok and pitch_ok:
                self.progress("Achieved attitude")
                break

    def reboot_sitl(self, *args, **kwargs):
        self.disarm_vehicle()
        super(AutoTestTracker, self).reboot_sitl(*args, **kwargs)

    def GUIDED(self):
        '''Test GUIDED mode'''
        self.reboot_sitl() # temporary hack around control issues
        self.change_mode(4) # "GUIDED"
        self.achieve_attitude(desyaw=10, despitch=30)
        self.achieve_attitude(desyaw=0, despitch=0)
        self.achieve_attitude(desyaw=45, despitch=10)

    def GUIDEDHoldsAttitudeOnEntry(self):
        '''ensure entering GUIDED holds the current attitude until a target arrives'''
        self.reboot_sitl()
        self.set_parameters({
            # keep pitch away from its limits while scanning
            "SCAN_SPEED_PIT": 0,
            # build up integrator for GUIDED to discard
            "YAW2SRV_I": 0.02,
            "PITCH2SRV_I": 0.02,
        })

        def enter_guided_from_auto_and_check_hold(stale_target_yaw):
            # scan away from the stale GUIDED target
            self.change_mode(2)  # "SCAN"
            tstart = self.get_sim_time()
            while True:
                # SCAN may have to go the long way round
                if self.get_sim_time_cached() - tstart > 240:
                    raise NotAchievedException("Did not scan away from yaw=%f" % stale_target_yaw)
                m = self.assert_receive_message('ATTITUDE')
                if abs(mavextra.angle_diff(math.degrees(m.yaw), stale_target_yaw)) > 60:
                    break

            # with no vehicle position AUTO leaves the servos where they are
            self.change_mode(10)  # "AUTO"
            self.wait_message_field_values('ATTITUDE', {
                'yawspeed': 0,
                'pitchspeed': 0,
            }, epsilon=math.radians(0.5), minimum_duration=2, timeout=60)

            self.change_mode(4)  # "GUIDED"
            m = self.assert_receive_message('ATTITUDE')
            entry_yaw = math.degrees(m.yaw)
            entry_pitch = math.degrees(m.pitch)
            self.progress("Entered GUIDED at yaw=%f pitch=%f" % (entry_yaw, entry_pitch))
            tstart = self.get_sim_time()
            while self.get_sim_time_cached() - tstart < 10:
                m = self.assert_receive_message('ATTITUDE')
                yaw_error = abs(mavextra.angle_diff(math.degrees(m.yaw), entry_yaw))
                pitch_error = abs(math.degrees(m.pitch) - entry_pitch)
                if yaw_error > 2 or pitch_error > 2:
                    raise NotAchievedException(
                        "Moved after entering GUIDED: yaw=%f pitch=%f" %
                        (math.degrees(m.yaw), math.degrees(m.pitch)))

        # first entry after boot must not slew to the default (north, level) target
        enter_guided_from_auto_and_check_hold(0)

        # re-entry must not slew to the previous GUIDED target; the I
        # gains can make the tracker oscillate around this target
        self.set_parameters({"YAW2SRV_I": 0, "PITCH2SRV_I": 0})
        self.achieve_attitude(desyaw=-90, despitch=10)
        self.set_parameters({"YAW2SRV_I": 0.02, "PITCH2SRV_I": 0.02})
        enter_guided_from_auto_and_check_hold(-90)

    def MANUAL(self):
        '''Test MANUAL mode'''
        self.change_mode(0) # "MANUAL"
        for chan in 1, 2:
            for pwm in 1200, 1600, 1367:
                self.set_rc(chan, pwm)
                self.wait_servo_channel_value(chan, pwm)

    def MAV_CMD_DO_SET_SERVO(self):
        '''Test SERVOTEST mode'''
        self.change_mode(0) # "MANUAL"
        # magically changes to SERVOTEST (3)
        for method in self.run_cmd, self.run_cmd_int:
            for value in 1900, 1200:
                channel = 1
                method(
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    p1=channel,
                    p2=value,
                    timeout=1,
                )
                self.wait_servo_channel_value(channel, value)
            for value in 1300, 1670:
                channel = 2
                method(
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    p1=channel,
                    p2=value,
                    timeout=1,
                )
                self.wait_servo_channel_value(channel, value)

    def sim_relative_yaw_pitch(self):
        '''return the simulated tracker's yaw and pitch in degrees,
        relative to the 270 degree yaw and 10 degree pitch at the centre
        of its range'''
        m = self.assert_receive_message('SIMSTATE')
        return (mavextra.wrap_180(math.degrees(m.yaw) - 270),
                math.degrees(m.pitch) - 10)

    def wait_sim_relative_yaw_pitch(self, yaw, pitch, tolerance=1, timeout=120):
        '''wait for the simulated tracker to reach a relative yaw and pitch.
        Pitch moves at only 1 degree/second, so this can take a while'''
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException(f"Did not reach yaw={yaw} pitch={pitch}")
            (current_yaw, current_pitch) = self.sim_relative_yaw_pitch()
            if (abs(mavextra.wrap_180(current_yaw - yaw)) < tolerance and
                    abs(current_pitch - pitch) < tolerance):
                return

    def UndrivenServosHoldPosition(self):
        '''check the simulated tracker does not move while its servos are not driven'''
        self.change_mode(0) # "MANUAL"
        # centre the tracker so that it has room to move in either direction
        self.set_rc(1, 1500)
        self.set_rc(2, 1500)
        self.wait_sim_relative_yaw_pitch(0, 0)
        self.disarm_vehicle()
        for channel in 1, 2:
            self.wait_servo_channel_value(channel, 0)
        (start_yaw, start_pitch) = self.sim_relative_yaw_pitch()
        self.delay_sim_time(12, reason="tracker to hold position")
        (end_yaw, end_pitch) = self.sim_relative_yaw_pitch()
        self.progress(f"yaw {start_yaw:.1f} -> {end_yaw:.1f}, pitch {start_pitch:.1f} -> {end_pitch:.1f}")
        if abs(mavextra.wrap_180(end_yaw - start_yaw)) > 1:
            raise NotAchievedException("Tracker yawed while undriven")
        if abs(end_pitch - start_pitch) > 1:
            raise NotAchievedException("Tracker pitched while undriven")
        self.arm_vehicle()

    def ServoRangeLimits(self):
        '''check the simulated tracker stops at the limits of its range'''
        self.set_parameters({
            "SERVO1_MAX": 2200,
            "SERVO2_MAX": 2200,
        })
        self.change_mode(0) # "MANUAL"
        # 2200 asks for 1.4 times the model's +/-170 degree yaw range
        # and +/-45 degree pitch range
        for channel in 1, 2:
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                p1=channel,
                p2=2200,
            )
            self.wait_servo_channel_value(channel, 2200)

        self.wait_sim_relative_yaw_pitch(170, 45)

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            (yaw, pitch) = self.sim_relative_yaw_pitch()
            if not 169 < yaw < 171:
                raise NotAchievedException(f"Tracker yaw {yaw:.1f} not held at limit")
            if not 44 < pitch < 46:
                raise NotAchievedException(f"Tracker pitch {pitch:.1f} not held at limit")

    def Tracking(self):
        '''check the tracker points at a vehicle sending its position'''
        self.change_mode("AUTO")
        self.wait_ready_to_arm()

        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        tracker_loc = Location(m.lat * 1.0e-7, m.lon * 1.0e-7, m.alt * 1.0e-3, AltFrame.ABSOLUTE)
        # the vehicle reports the tracker's own pressure, so the
        # tracker sees no altitude difference and should pitch to zero
        m = self.assert_receive_message('SCALED_PRESSURE')
        vehicle_press_abs = m.press_abs
        vehicle_temperature = m.temperature

        vehicle_loc = None
        last_sent = 0

        def send_vehicle_position(mav, m):
            nonlocal last_sent
            now = self.get_sim_time_cached()
            if vehicle_loc is None or now - last_sent < 0.1:
                return
            last_sent = now
            self.mav.mav.global_position_int_send(
                int(now * 1000),  # time_boot_ms
                int(vehicle_loc.lat * 1e7),
                int(vehicle_loc.lng * 1e7),
                int(vehicle_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0,  # relative_alt
                0,  # vx
                0,  # vy
                0,  # vz
                65535  # hdg
            )
            self.mav.mav.scaled_pressure_send(
                int(now * 1000),  # time_boot_ms
                vehicle_press_abs,
                0,  # press_diff
                vehicle_temperature
            )

        self.context_push()
        self.install_message_hook_context(send_vehicle_position)

        # the simulated tracker's yaw range is centred on west and
        # spans 170 degrees either way, so keep the vehicle to the west
        for (north, east) in (0, -20), (20, -20), (-20, -20), (30, -40), (-30, -40):
            desyaw = math.degrees(math.atan2(east, north))
            self.start_subtest(f"Vehicle {north}m north {east}m east (bearing {desyaw:.0f})")
            vehicle_loc = self.offset_location_ne(tracker_loc, north, east)
            self.wait_attitude(despitch=0, desyaw=desyaw, tolerance=2, timeout=120, minimum_duration=5)

        self.context_pop()

    def MAV_CMD_MISSION_START(self):
        '''test MAV_CMD_MISSION_START mavlink command'''
        for method in self.run_cmd, self.run_cmd_int:
            self.change_mode(0)  # "MANUAL"
            method(mavutil.mavlink.MAV_CMD_MISSION_START)
            self.wait_mode("AUTO")

    def SCAN(self):
        '''Test SCAN mode'''
        self.change_mode(2) # "SCAN"
        self.set_parameter("SCAN_SPEED_YAW", 20)
        for channel in 1, 2:
            self.wait_servo_channel_value(channel,
                                          1900,
                                          timeout=90,
                                          comparator=operator.ge)
        for channel in 1, 2:
            self.wait_servo_channel_value(channel,
                                          1200,
                                          timeout=90,
                                          comparator=operator.le)

    def BaseMessageSet(self):
        '''ensure we're getting messages we expect'''
        self.set_parameter('BATT_MONITOR', 4)
        self.reboot_sitl()
        for msg in 'BATTERY_STATUS', :
            self.assert_receive_message(msg)

    def disabled_tests(self):
        return {
            "ArmFeatures": "See https://github.com/ArduPilot/ardupilot/issues/10652",
            "CPUFailsafe": " tracker doesn't have a CPU failsafe",
        }

    def GPSForYaw(self):
        '''Moving baseline GPS yaw'''
        self.load_default_params_file("tracker-gps-for-yaw.parm")
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                break
            m_gps_raw = self.assert_receive_message("GPS2_RAW", verbose=True)
            m_sim = self.assert_receive_message("SIMSTATE", verbose=True)
            gps_raw_hdg = m_gps_raw.yaw * 0.01
            sim_hdg = mavextra.wrap_360(math.degrees(m_sim.yaw))
            if abs(gps_raw_hdg - sim_hdg) > 5:
                raise NotAchievedException("GPS_RAW not tracking simstate yaw")
            self.progress(f"yaw match ({gps_raw_hdg} vs {sim_hdg}")

    def StationaryGlobalPositionIntAlt(self):
        '''Test GLOBAL_POSITION_INT.alt is correct in stationary mode'''
        # Disable GPS so the tracker stays in stationary mode (stationary=true).
        # With GPS enabled the tracker gets a fix, sets stationary=false, and
        # delegates to the base-class send_global_position_int() which is correct.
        self.set_parameter("GPS1_TYPE", 0)
        self.reboot_sitl()

        # Set a known home position; Tracker::set_home() stores this in
        # current_loc, which the stationary path uses directly.
        home_loc = self.sitl_start_location()
        self.set_home(home_loc)

        self.assert_received_message_field_values("GLOBAL_POSITION_INT", {
            "alt": int(home_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000),
        })

    def LoggerMsgChunks(self):
        '''create MSG dataflash entries for very long messages'''
        self.assert_parameter_value('LOG_DISARMED', 1)
        short_message_text = "This is a short message"
        long_message_text = "This is undubitably a ridiculously verbose and needlessly wordy missive, unavoidably designed to exceed disappointingly minuscule log buffers"  # noqa:E501
        self.send_statustext(short_message_text)
        self.send_statustext(long_message_text)

        self.delay_sim_time(10, reason="statustext to be logged")
        dfreader = self.dfreader_for_current_onboard_log()
        self.reboot_sitl()

        phase = "short"
        seq = 0
        received_long_message_text = ""
        while True:
            m = dfreader.recv_match(type='MSG')
            if m is None:
                break
            self.progress(f"{m.Message=}")
            msg = m.Message.removeprefix("SRC=250/250:")
            if phase == "short":
                if msg != short_message_text:
                    continue
                self.progress("Received short message")
                phase = "long"
            elif phase == "long":
                if m.Seq != seq:
                    received_long_message_text = ""
                    seq = 0
                    if m.Seq != 0:
                        raise NotAchievedException("Weird")
                else:
                    seq += 1
                received_long_message_text += msg
                self.progress(f"Text: {received_long_message_text}")
                if received_long_message_text == long_message_text:
                    phase = "done"
                    break

        if phase != "done":
            raise NotAchievedException(f"Did not get message text; {phase=}")  # noqa:E501

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestTracker, self).tests()
        ret.extend([
            self.GUIDED,
            self.GUIDEDHoldsAttitudeOnEntry,
            self.MANUAL,
            self.MAV_CMD_DO_SET_SERVO,
            self.MAV_CMD_MISSION_START,
            self.NMEAOutput,
            self.SCAN,
            self.BaseMessageSet,
            self.GPSForYaw,
            self.LoggerMsgChunks,
            self.StationaryGlobalPositionIntAlt,
            self.UndrivenServosHoldPosition,
            self.ServoRangeLimits,
            self.Tracking,
        ])
        return ret
