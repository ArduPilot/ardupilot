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
from vehicle_test_suite import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-27.274439, 151.290064, 343, 8.7)


class AutoTestTracker(vehicle_test_suite.TestSuite):

    def log_name(self):
        return "AntennaTracker"

    def default_speedup(self):
        '''Tracker seems to be race-free'''
        return 100

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

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

    def LoggerMsgChunks(self):
        '''create MSG dataflash entries for very long messages'''
        self.assert_parameter_value('LOG_DISARMED', 1)
        short_message_text = "This is a short message"
        long_message_text = "This is undubitably a ridiculously verbose and needlessly wordy missive, unavoidably designed to exceed disappointingly minuscule log buffers"  # noqa:E501
        self.send_statustext(short_message_text)
        self.send_statustext(long_message_text)

        self.delay_sim_time(10)
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
            msg = m.Message.lstrip("SRC=250/250:")
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
            self.MANUAL,
            self.MAV_CMD_DO_SET_SERVO,
            self.MAV_CMD_MISSION_START,
            self.NMEAOutput,
            self.SCAN,
            self.BaseMessageSet,
            self.GPSForYaw,
            self.LoggerMsgChunks,
        ])
        return ret
