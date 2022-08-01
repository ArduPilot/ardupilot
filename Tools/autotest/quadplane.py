'''
Fly ArduPlane QuadPlane in SITL

AP_FLAKE8_CLEAN

'''

from __future__ import print_function
import os
import numpy
import math

from pymavlink import mavutil

from common import AutoTest
from common import AutoTestTimeoutException, NotAchievedException, PreconditionFailedException

import operator


# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
WIND = "0,180,0.2"  # speed,direction,variance
SITL_START_LOCATION = mavutil.location(-27.274439, 151.290064, 343, 8.7)


class AutoTestQuadPlane(AutoTest):

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

    def vehicleinfo_key(self):
        return 'ArduPlane'

    def default_frame(self):
        return "quadplane"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_speedup(self):
        '''QuadPlane seems to be race-free'''
        return 100

    def log_name(self):
        return "QuadPlane"

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduPlane_Tests/" + name + "/"

    def apply_defaultfile_parameters(self):
        # plane passes in a defaults_filepath in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        return self.model_defaults_filepath(self.frame)

    def is_plane(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("LAND_DISARMDELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("LAND_DISARMDELAY", delay)

    def test_airmode(self):
        """Check that plane.air_mode turns on and off as required"""
        self.progress("########## Testing AirMode operation")
        self.set_parameter("AHRS_EKF_TYPE", 10)
        self.change_mode('QSTABILIZE')
        self.wait_ready_to_arm()

        """
        SPIN_ARM and SPIN_MIN default to 0.10 and 0.15
        when armed with zero throttle in AirMode, motor PWM should be at SPIN_MIN
        If AirMode is off, motor PWM will drop to SPIN_ARM
        """

        self.progress("Verify that SERVO5 is Motor1 (default)")
        motor1_servo_function_lp = 33
        if (self.get_parameter('SERVO5_FUNCTION') != motor1_servo_function_lp):
            raise PreconditionFailedException("SERVO5_FUNCTION not %d" % motor1_servo_function_lp)

        self.progress("Verify that flightmode channel is 5 (default)")
        default_fltmode_ch = 5
        if (self.get_parameter("FLTMODE_CH") != default_fltmode_ch):
            raise PreconditionFailedException("FLTMODE_CH not %d" % default_fltmode_ch)

        """When disarmed, motor PWM will drop to min_pwm"""
        min_pwm = self.get_parameter("Q_M_PWM_MIN")

        self.progress("Verify Motor1 is at min_pwm when disarmed")
        self.wait_servo_channel_value(5, min_pwm, comparator=operator.eq)

        armdisarm_option = 154
        arm_ch = 8
        self.set_parameter("RC%d_OPTION" % arm_ch, armdisarm_option)
        self.progress("Configured RC%d as ARMDISARM switch" % arm_ch)

        """arm with GCS, record Motor1 SPIN_ARM PWM output and disarm"""
        spool_delay = self.get_parameter("Q_M_SPOOL_TIME") + 0.25
        self.zero_throttle()
        self.arm_vehicle()
        self.progress("Waiting for Motor1 to spool up to SPIN_ARM")
        self.delay_sim_time(spool_delay)
        spin_arm_pwm = self.wait_servo_channel_value(5, min_pwm, comparator=operator.gt)
        self.progress("spin_arm_pwm: %d" % spin_arm_pwm)
        self.disarm_vehicle()

        """arm with switch, record Motor1 SPIN_MIN PWM output and disarm"""
        self.set_rc(8, 2000)
        self.delay_sim_time(spool_delay)
        self.progress("Waiting for Motor1 to spool up to SPIN_MIN")
        spin_min_pwm = self.wait_servo_channel_value(5, spin_arm_pwm, comparator=operator.gt)
        self.progress("spin_min_pwm: %d" % spin_min_pwm)
        self.set_rc(8, 1000)

        if (spin_arm_pwm >= spin_min_pwm):
            raise PreconditionFailedException("SPIN_MIN pwm not greater than SPIN_ARM pwm")

        self.start_subtest("Test auxswitch arming with AirMode Switch")
        for mode in ('QSTABILIZE', 'QACRO'):
            """verify that arming with switch results in higher PWM output"""
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with switch at zero throttle")
            self.arm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to speed up")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)

            self.progress("Verify that rudder disarm is disabled")
            try:
                self.disarm_motors_with_rc_input()
            except NotAchievedException:
                pass
            if not self.armed():
                raise NotAchievedException("Rudder disarm not disabled")

            self.progress("Disarming with switch")
            self.disarm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to stop")
            self.wait_servo_channel_value(5, min_pwm, comparator=operator.le)
            self.wait_ready_to_arm()

        self.start_subtest("Verify that arming with switch does not spin motors in other modes")
        # introduce a large attitude error to verify that stabilization is not active
        ahrs_trim_x = self.get_parameter("AHRS_TRIM_X")
        self.set_parameter("AHRS_TRIM_X", math.radians(-60))
        self.wait_roll(60, 1)
        # test all modes except QSTABILIZE, QACRO, AUTO and QAUTOTUNE and QLAND and QRTL
        # QRTL and QLAND aren't tested because we can't arm in that mode
        for mode in (
                'ACRO',
                'AUTOTUNE',
                'AVOID_ADSB',
                'CIRCLE',
                'CRUISE',
                'FBWA',
                'FBWB',
                'GUIDED',
                'LOITER',
                'QHOVER',
                'QLOITER',
                'STABILIZE',
                'TRAINING',
        ):
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with switch at zero throttle")
            self.arm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to (not) speed up")
            self.delay_sim_time(spool_delay)
            self.wait_servo_channel_value(5, spin_arm_pwm, comparator=operator.le)
            self.wait_servo_channel_value(6, spin_arm_pwm, comparator=operator.le)
            self.wait_servo_channel_value(7, spin_arm_pwm, comparator=operator.le)
            self.wait_servo_channel_value(8, spin_arm_pwm, comparator=operator.le)

            self.progress("Disarming with switch")
            self.disarm_motors_with_switch(arm_ch)
            self.progress("Waiting for Motor1 to stop")
            self.wait_servo_channel_value(5, min_pwm, comparator=operator.le)
            self.wait_ready_to_arm()
        # remove attitude error
        self.set_parameter("AHRS_TRIM_X", ahrs_trim_x)

        self.start_subtest("verify that AIRMODE auxswitch turns airmode on/off while armed")
        """set  RC7_OPTION to AIRMODE"""
        option_airmode = 84
        self.set_parameter("RC7_OPTION", option_airmode)

        for mode in ('QSTABILIZE', 'QACRO'):
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with GCS at zero throttle")
            self.arm_vehicle()

            self.progress("Turn airmode on with auxswitch")
            self.set_rc(7, 2000)
            self.progress("Waiting for Motor1 to speed up")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)

            self.progress("Turn airmode off with auxswitch")
            self.set_rc(7, 1000)
            self.progress("Waiting for Motor1 to slow down")
            self.wait_servo_channel_value(5, spin_arm_pwm, comparator=operator.le)
            self.disarm_vehicle()
            self.wait_ready_to_arm()

        self.start_subtest("Test GCS arming")
        for mode in ('QSTABILIZE', 'QACRO'):
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.zero_throttle()
            self.progress("Arming with GCS at zero throttle")
            self.arm_vehicle()

            self.progress("Turn airmode on with auxswitch")
            self.set_rc(7, 2000)
            self.progress("Waiting for Motor1 to speed up")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)

            self.disarm_vehicle_expect_fail()
            self.arm_vehicle()

            self.progress("Verify that airmode is still on")
            self.wait_servo_channel_value(5, spin_min_pwm, comparator=operator.ge)
            self.disarm_vehicle(force=True)
            self.wait_ready_to_arm()

    def test_motor_mask(self):
        """Check operation of output_motor_mask"""
        """copter tailsitters will add condition: or (int(self.get_parameter('Q_TAILSIT_MOTMX')) & 1)"""
        if not(int(self.get_parameter('Q_TILT_MASK')) & 1):
            self.progress("output_motor_mask not in use")
            return
        self.progress("Testing output_motor_mask")
        self.wait_ready_to_arm()

        """Default channel for Motor1 is 5"""
        self.progress('Assert that SERVO5 is Motor1')
        assert(33 == self.get_parameter('SERVO5_FUNCTION'))

        modes = ('MANUAL', 'FBWA', 'QHOVER')
        for mode in modes:
            self.progress("Testing %s mode" % mode)
            self.change_mode(mode)
            self.arm_vehicle()
            self.progress("Raising throttle")
            self.set_rc(3, 1800)
            self.progress("Waiting for Motor1 to start")
            self.wait_servo_channel_value(5, 1100, comparator=operator.gt)

            self.set_rc(3, 1000)
            self.disarm_vehicle()
            self.wait_ready_to_arm()

    def fly_mission(self, filename, fence=None, height_accuracy=-1, include_terrain_timeout=False):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        self.load_mission(filename)
        if fence is not None:
            self.load_fence(fence)
        if self.mavproxy is not None:
            self.mavproxy.send('wp list\n')
        self.install_terrain_handlers_context()
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, 19, max_dist=60, timeout=1200)

        self.wait_disarmed(timeout=120) # give quadplane a long time to land
        # wait for blood sample here
        self.set_current_waypoint(20)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(20, 34, max_dist=60, timeout=1200)

        self.wait_disarmed(timeout=120) # give quadplane a long time to land
        self.progress("Mission OK")

    def enum_state_name(self, enum_name, state, pretrim=None):
        e = mavutil.mavlink.enums[enum_name]
        e_value = e[state]
        name = e_value.name
        if pretrim is not None:
            if not pretrim.startswith(pretrim):
                raise NotAchievedException("Expected %s to pretrim" % (pretrim))
            name = name.replace(pretrim, "")
        return name

    def vtol_state_name(self, state):
        return self.enum_state_name("MAV_VTOL_STATE", state, pretrim="MAV_VTOL_STATE_")

    def landed_state_name(self, state):
        return self.enum_state_name("MAV_LANDED_STATE", state, pretrim="MAV_LANDED_STATE_")

    def assert_extended_sys_state(self, vtol_state, landed_state):
        m = self.assert_receive_message('EXTENDED_SYS_STATE', timeout=1)
        if m.vtol_state != vtol_state:
            raise ValueError("Bad MAV_VTOL_STATE.  Want=%s got=%s" %
                             (self.vtol_state_name(vtol_state),
                              self.vtol_state_name(m.vtol_state)))
        if m.landed_state != landed_state:
            raise ValueError("Bad MAV_LANDED_STATE.  Want=%s got=%s" %
                             (self.landed_state_name(landed_state),
                              self.landed_state_name(m.landed_state)))

    def wait_extended_sys_state(self, vtol_state, landed_state):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 10:
                raise NotAchievedException("Did not achieve vol/landed states")
            self.progress("Waiting for MAV_VTOL_STATE=%s MAV_LANDED_STATE=%s" %
                          (self.vtol_state_name(vtol_state),
                           self.landed_state_name(landed_state)))
            m = self.assert_receive_message('EXTENDED_SYS_STATE', verbose=True)
            if m.landed_state != landed_state:
                self.progress("Wrong MAV_LANDED_STATE (want=%s got=%s)" %
                              (self.landed_state_name(landed_state),
                               self.landed_state_name(m.landed_state)))
                continue
            if m.vtol_state != vtol_state:
                self.progress("Wrong MAV_VTOL_STATE (want=%s got=%s)" %
                              (self.vtol_state_name(vtol_state),
                               self.vtol_state_name(m.vtol_state)))
                continue

            self.progress("vtol and landed states match")
            return

    def EXTENDED_SYS_STATE_SLT(self):
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)
        self.change_mode("QHOVER")
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("FBWA")
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_FW,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("QHOVER")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # should not change just because we arm:
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("MANUAL")
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_FW,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("QHOVER")

        self.progress("Taking off")
        self.set_rc(3, 1750)
        self.wait_altitude(1, 5, relative=True)
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                       mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)
        self.wait_altitude(10, 15, relative=True)

        self.progress("Transitioning to fixed wing")
        self.change_mode("FBWA")
        self.set_rc(3, 1900) # apply spurs
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW,
                                     mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_FW,
                                     mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)

        self.progress("Transitioning to multicopter")
        self.set_rc(3, 1500) # apply reins
        self.change_mode("QHOVER")
        # for a standard quadplane there is no transition-to-mc stage.
        # tailsitters do have such a state.
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                     mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)
        self.change_mode("QLAND")
        self.wait_altitude(0, 2, relative=True, timeout=60)
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                     mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.mav.motors_disarmed_wait()

    def EXTENDED_SYS_STATE(self):
        self.EXTENDED_SYS_STATE_SLT()

    def fly_qautotune(self):
        self.change_mode("QHOVER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1800)
        self.wait_altitude(30,
                           40,
                           relative=True,
                           timeout=30)
        self.set_rc(3, 1500)
        self.change_mode("QAUTOTUNE")
        tstart = self.get_sim_time()
        sim_time_expected = 5000
        deadline = tstart + sim_time_expected
        while self.get_sim_time_cached() < deadline:
            now = self.get_sim_time_cached()
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("STATUSTEXT (%u<%u): %s" % (now, deadline, m.text))
            if "AutoTune: Success" in m.text:
                break
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.set_rc(3, 1200)
        self.wait_altitude(-5, 1, relative=True, timeout=30)
        while self.get_sim_time_cached() < deadline:
            self.mavproxy.send('disarm\n')
            try:
                self.wait_text("AutoTune: Saved gains for Roll Pitch Yaw", timeout=0.5)
            except AutoTestTimeoutException:
                continue
            break
        self.wait_disarmed()

    def takeoff(self, height, mode):
        """climb to specified height and set throttle to 1500"""
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode(mode)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1800)
        self.wait_altitude(height,
                           height+5,
                           relative=True,
                           timeout=30)
        self.set_rc(3, 1500)

    def do_RTL(self):
        self.change_mode("QRTL")
        self.wait_altitude(-5, 1, relative=True, timeout=60)
        self.wait_disarmed()
        self.zero_throttle()

    def fly_home_land_and_disarm(self, timeout=30):
        self.set_parameter("LAND_TYPE", 0)
        filename = "flaps.txt"
        self.progress("Using %s to fly home" % filename)
        self.load_mission(filename)
        self.change_mode("AUTO")
        self.set_current_waypoint(7)
        self.wait_disarmed(timeout=timeout)
        self.set_current_waypoint(0, check_afterwards=False)

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

    def fly_left_circuit(self):
        """Fly a left circuit, 200m on a side."""
        self.mavproxy.send('switch 4\n')
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
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
        self.change_mode('QHOVER')
        self.set_rc(3, 1100)
        self.wait_altitude(10, 15,
                           relative=True,
                           timeout=60)
        self.set_rc(3, 1500)

    def hover_and_check_matched_frequency(self, dblevel=-15, minhz=200, maxhz=300, fftLength=32, peakhz=None):

        # find a motor peak
        self.takeoff(10, mode="QHOVER")

        hover_time = 15
        tstart = self.get_sim_time()
        self.progress("Hovering for %u seconds" % hover_time)
        while self.get_sim_time_cached() < tstart + hover_time:
            self.mav.recv_match(type='ATTITUDE', blocking=True)
        vfr_hud = self.mav.recv_match(type='VFR_HUD', blocking=True)
        tend = self.get_sim_time()

        self.do_RTL()
        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(minhz * scale)
        smaxhz = int(maxhz * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])
        if peakdb < dblevel or (peakhz is not None and abs(freq - peakhz) / peakhz > 0.05):
            raise NotAchievedException("No motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            self.progress("motor peak %fHz, thr %f%%, %fdB" % (freq, vfr_hud.throttle, peakdb))

        # we have a peak make sure that the FFT detected something close
        # logging is at 10Hz
        mlog = self.dfreader_for_current_onboard_log()
        # accuracy is determined by sample rate and fft length, given our use of quinn we could probably use half of this
        freqDelta = 1000. / fftLength
        pkAvg = freq
        freqs = []

        while True:
            m = mlog.recv_match(
                type='FTN1',
                blocking=True,
                condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6))
            if m is None:
                break
            freqs.append(m.PkAvg)

        # peak within resolution of FFT length
        pkAvg = numpy.median(numpy.asarray(freqs))
        if abs(pkAvg - freq) > freqDelta:
            raise NotAchievedException("FFT did not detect a motor peak at %f, found %f, wanted %f" % (dblevel, pkAvg, freq))

        return freq

    def fly_gyro_fft(self):
        """Use dynamic harmonic notch to control motor noise."""
        # basic gyro sample rate test
        self.progress("Flying with gyro FFT - Gyro sample rate")
        self.context_push()
        ex = None
        try:
            self.set_rc_default()

            # magic tridge EKF type that dramatically speeds up the test
            self.set_parameters({
                "AHRS_EKF_TYPE": 10,

                "INS_LOG_BAT_MASK": 3,
                "INS_LOG_BAT_OPT": 0,
                "INS_GYRO_FILTER": 100,
                "LOG_BITMASK": 45054,
                "LOG_DISARMED": 0,
                "SIM_DRIFT_SPEED": 0,
                "SIM_DRIFT_TIME": 0,
                # enable a noisy motor peak
                "SIM_GYR1_RND": 20,
                # enabling FFT will also enable the arming check: self-testing the functionality
                "FFT_ENABLE": 1,
                "FFT_MINHZ": 80,
                "FFT_MAXHZ": 350,
                "FFT_SNR_REF": 10,
                "FFT_WINDOW_SIZE": 128,
                "FFT_WINDOW_OLAP": 0.75,
            })
            # Step 1: inject a very precise noise peak at 250hz and make sure the in-flight fft
            # can detect it really accurately. For a 128 FFT the frequency resolution is 8Hz so
            # a 250Hz peak should be detectable within 5%
            self.set_parameters({
                "SIM_VIB_FREQ_X": 250,
                "SIM_VIB_FREQ_Y": 250,
                "SIM_VIB_FREQ_Z": 250,
            })
            self.reboot_sitl()

            # find a motor peak
            self.hover_and_check_matched_frequency(-15, 100, 350, 128, 250)

            # Step 2: inject actual motor noise and use the standard length FFT to track it
            self.set_parameters({
                "SIM_VIB_MOT_MAX": 350,
                "FFT_WINDOW_SIZE": 32,
                "FFT_WINDOW_OLAP": 0.5,
            })
            self.reboot_sitl()
            # find a motor peak
            freq = self.hover_and_check_matched_frequency(-15, 200, 300, 32)

            # Step 3: add a FFT dynamic notch and check that the peak is squashed
            self.set_parameters({
                "INS_LOG_BAT_OPT": 2,
                "INS_HNTCH_ENABLE": 1,
                "INS_HNTCH_FREQ": freq,
                "INS_HNTCH_REF": 1.0,
                "INS_HNTCH_ATT": 50,
                "INS_HNTCH_BW": freq/2,
                "INS_HNTCH_MODE": 4,
            })
            self.reboot_sitl()

            self.takeoff(10, mode="QHOVER")
            hover_time = 15
            ignore_bins = 20

            self.progress("Hovering for %u seconds" % hover_time)
            tstart = self.get_sim_time()
            while self.get_sim_time_cached() < tstart + hover_time:
                self.mav.recv_match(type='ATTITUDE', blocking=True)
            tend = self.get_sim_time()

            self.do_RTL()
            psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)
            freq = psd["F"][numpy.argmax(psd["X"][ignore_bins:]) + ignore_bins]
            peakdB = numpy.amax(psd["X"][ignore_bins:])
            if peakdB < -10:
                self.progress("No motor peak, %f at %f dB" % (freq, peakdB))
            else:
                raise NotAchievedException("Detected peak at %f Hz of %.2f dB" % (freq, peakdB))

            # Step 4: take off as a copter land as a plane, make sure we track
            self.progress("Flying with gyro FFT - vtol to plane")
            self.load_mission("quadplane-gyro-mission.txt")
            if self.mavproxy is not None:
                self.mavproxy.send('wp list\n')
            self.change_mode('AUTO')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.wait_waypoint(1, 7, max_dist=60, timeout=1200)
            self.wait_disarmed(timeout=120) # give quadplane a long time to land

            # prevent update parameters from messing with the settings when we pop the context
            self.set_parameter("FFT_ENABLE", 0)
            self.reboot_sitl()

        except Exception as e:
            self.progress("Exception caught: %s" % (
                self.get_exception_stacktrace(e)))
            ex = e

        self.context_pop()

        self.reboot_sitl()

        if ex is not None:
            raise ex

    def test_pid_tuning(self):
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestQuadPlane, self).test_pid_tuning()

    def test_parameter_checks(self):
        self.test_parameter_checks_poscontrol("Q_P")

    def rc_defaults(self):
        ret = super(AutoTestQuadPlane, self).rc_defaults()
        ret[3] = 1000
        return ret

    def default_mode(self):
        return "MANUAL"

    def disabled_tests(self):
        return {
            "FRSkyPassThrough": "Currently failing",
            "CPUFailsafe": "servo channel values not scaled like ArduPlane",
            "GyroFFT": "flapping test",
            "ConfigErrorLoop": "failing because RC values not settable",
        }

    def test_pilot_yaw(self):
        self.takeoff(10, mode="QLOITER")
        self.set_parameter("STICK_MIXING", 0)
        self.set_rc(4, 1700)
        for mode in "QLOITER", "QHOVER":
            self.wait_heading(45)
            self.wait_heading(90)
            self.wait_heading(180)
            self.wait_heading(275)
        self.set_rc(4, 1500)
        self.do_RTL()

    def weathervane_test(self):
        # We test nose into wind code paths and yaw direction in copter autotest,
        # so we shall test the side into wind yaw direction and plane code paths here.
        self.set_parameters({"SIM_WIND_SPD": 10,
                             "SIM_WIND_DIR": 240,
                             "Q_WVANE_ENABLE": 3, # WVANE_ENABLE = 3 gives direction of side into wind
                             "Q_WVANE_GAIN": 3,
                             "STICK_MIXING": 0})

        self.takeoff(10, mode="QLOITER")

        # Turn aircraft to heading 90 deg
        self.set_rc(4, 1700)
        self.wait_heading(90)
        self.set_rc(4, 1500)

        # Now wait for weathervaning to activate and turn side-on to wind at 240 deg therefore heading 150 deg
        self.wait_heading(150, accuracy=5, timeout=180)

        self.do_RTL()

    def CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        self.plane_CPUFailsafe()

    def test_qassist(self):
        # find a motor peak
        self.takeoff(10, mode="QHOVER")
        self.set_rc(3, 1800)
        self.change_mode("FBWA")

        # disable stall prevention so roll angle is not limited
        self.set_parameter("STALL_PREVENTION", 0)

        thr_min_pwm = self.get_parameter("Q_M_PWM_MIN")
        lim_roll_deg = self.get_parameter("LIM_ROLL_CD") * 0.01
        self.progress("Waiting for motors to stop (transition completion)")
        self.wait_servo_channel_value(5,
                                      thr_min_pwm,
                                      timeout=30,
                                      comparator=operator.eq)
        self.delay_sim_time(5)
        self.wait_servo_channel_value(5,
                                      thr_min_pwm,
                                      timeout=30,
                                      comparator=operator.eq)
        self.progress("Stopping forward motor to kill airspeed below limit")
        self.set_rc(3, 1000)
        self.progress("Waiting for qassist to kick in")
        self.wait_servo_channel_value(5, 1400, timeout=30, comparator=operator.gt)
        self.progress("Move forward again, check qassist stops")
        self.set_rc(3, 1800)
        self.progress("Checking qassist stops")
        self.wait_servo_channel_value(5,
                                      thr_min_pwm,
                                      timeout=30,
                                      comparator=operator.eq)
        self.set_rc(3, 1300)

        self.context_push()
        self.progress("Rolling over to %.0f degrees" % -lim_roll_deg)
        self.set_rc(1, 1000)
        self.wait_roll(-lim_roll_deg, 5)
        self.progress("Killing servo outputs to force qassist to help")
        self.set_parameter("SERVO1_MIN", 1480)
        self.set_parameter("SERVO1_MAX", 1480)
        self.set_parameter("SERVO1_TRIM", 1480)
        self.progress("Trying to roll over hard the other way")
        self.set_rc(1, 2000)
        self.progress("Waiting for qassist (angle) to kick in")
        self.wait_servo_channel_value(5, 1100, timeout=30, comparator=operator.gt)
        self.wait_roll(lim_roll_deg, 5)
        self.context_pop()
        self.set_rc(1, 1500)
        self.set_parameter("Q_RTL_MODE", 1)
        self.change_mode("RTL")
        self.wait_disarmed(timeout=300)

    def tailsitter(self):
        '''tailsitter test'''
        self.set_parameter('Q_FRAME_CLASS', 10)
        self.set_parameter('Q_ENABLE', 1)
        self.set_parameter('Q_TAILSIT_ENABLE', 1)

        self.reboot_sitl()
        self.wait_ready_to_arm()
        value_before = self.get_servo_channel_value(3)
        self.progress("Before: %u" % value_before)
        self.change_mode('QHOVER')
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                break
            value_after = self.get_servo_channel_value(3)
            self.progress("After: t=%f output=%u" % ((now - tstart), value_after))
            if value_before != value_after:
                raise NotAchievedException("Changed throttle output on mode change to QHOVER")
        self.disarm_vehicle()

    def ICEngine(self):
        rc_engine_start_chan = 11
        self.set_parameters({
            'SERVO13_FUNCTION': 67,  # ignition
            'SERVO14_FUNCTION': 69,  # starter
            'ICE_ENABLE': 1,
            'ICE_START_CHAN': rc_engine_start_chan,
            'ICE_RPM_CHAN': 1,
            'RPM1_TYPE': 10,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.wait_rpm(1, 0, 0, minimum_duration=1)
        self.arm_vehicle()
        self.wait_rpm(1, 0, 0, minimum_duration=1)
        self.context_collect("STATUSTEXT")
        self.progress("Setting engine-start RC switch to HIGH")
        self.set_rc(rc_engine_start_chan, 2000)
        self.wait_statustext("Starting engine", check_context=True)
        self.wait_rpm(1, 300, 400, minimum_duration=1)
        self.progress("Setting engine-start RC switch to MID")
        self.set_rc(rc_engine_start_chan, 1500)
        self.progress("Setting full throttle")
        self.set_rc(3, 2000)
        self.wait_rpm(1, 6500, 7500, minimum_duration=30, timeout=40)
        self.progress("Setting min-throttle")
        self.set_rc(3, 1000)
        self.wait_rpm(1, 300, 400, minimum_duration=1)
        self.progress("Setting engine-start RC switch to LOW")
        self.set_rc(rc_engine_start_chan, 1000)
        self.wait_rpm(1, 0, 0, minimum_duration=1)
        self.disarm_vehicle()

    def ICEngineMission(self):
        rc_engine_start_chan = 11
        self.set_parameters({
            'SERVO13_FUNCTION': 67,  # ignition
            'SERVO14_FUNCTION': 69,  # starter
            'ICE_ENABLE': 1,
            'ICE_START_CHAN': rc_engine_start_chan,
            'ICE_RPM_CHAN': 1,
            'RPM1_TYPE': 10,
        })
        self.load_mission("mission.txt")
        self.wait_ready_to_arm()
        self.set_rc(rc_engine_start_chan, 2000)
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_disarmed(timeout=300)

    def tests(self):
        '''return list of all tests'''

        ret = super(AutoTestQuadPlane, self).tests()
        ret.extend([
            ("TestAirMode", "Test airmode", self.test_airmode),

            ("TestMotorMask", "Test output_motor_mask", self.test_motor_mask),

            ("PilotYaw",
             "Test pilot yaw in various modes",
             self.test_pilot_yaw),

            ("ParameterChecks",
             "Test Arming Parameter Checks",
             self.test_parameter_checks),

            ("TestLogDownload",
             "Test Onboard Log Download",
             self.test_log_download),

            ("EXTENDED_SYS_STATE",
             "Check extended sys state works",
             self.EXTENDED_SYS_STATE),

            ("Mission", "Dalby Mission",
             lambda: self.fly_mission(
                 "Dalby-OBC2016.txt",
                 "Dalby-OBC2016-fence.txt",
                 include_terrain_timeout=True
             )
             ),

            ("Weathervane",
             "Test Weathervane Functionality",
             self.weathervane_test),

            ("QAssist",
             "QuadPlane Assist tests",
             self.test_qassist),

            ("GyroFFT", "Fly Gyro FFT",
             self.fly_gyro_fft),

            ("Tailsitter",
             "Test tailsitter support",
             self.tailsitter),

            ("ICEngine",
             "Test ICE Engine support",
             self.ICEngine),

            ("ICEngineMission",
             "Test ICE Engine Mission support",
             self.ICEngineMission),

            ("LogUpload",
             "Log upload",
             self.log_upload),
        ])
        return ret
