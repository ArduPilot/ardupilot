'''
Fly ArduPlane QuadPlane in SITL

AP_FLAKE8_CLEAN

'''

from __future__ import print_function
import os
import numpy
import math

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

import vehicle_test_suite
from vehicle_test_suite import Test
from vehicle_test_suite import AutoTestTimeoutException, NotAchievedException, PreconditionFailedException

import operator


# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
WIND = "0,180,0.2"  # speed,direction,variance
SITL_START_LOCATION = mavutil.location(-27.274439, 151.290064, 343, 8.7)


class AutoTestQuadPlane(vehicle_test_suite.TestSuite):

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

    def AirMode(self):
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
        # disable compass magnetic field arming check that is triggered by the simulated lean of vehicle
        # this is required because adjusting the AHRS_TRIM values only affects the IMU and not external compasses
        arming_magthresh = self.get_parameter("ARMING_MAGTHRESH")
        self.set_parameter("ARMING_MAGTHRESH", 0)
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
        # remove attitude error and reinstance compass arming check
        self.set_parameter("AHRS_TRIM_X", ahrs_trim_x)
        self.set_parameter("ARMING_MAGTHRESH", arming_magthresh)

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

    def TestMotorMask(self):
        """Check operation of output_motor_mask"""
        """copter tailsitters will add condition: or (int(self.get_parameter('Q_TAILSIT_MOTMX')) & 1)"""
        if not (int(self.get_parameter('Q_TILT_MASK')) & 1):
            self.progress("output_motor_mask not in use")
            return
        self.progress("Testing output_motor_mask")
        self.wait_ready_to_arm()

        """Default channel for Motor1 is 5"""
        self.progress('Assert that SERVO5 is Motor1')
        assert 33 == self.get_parameter('SERVO5_FUNCTION')

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

    def fly_mission(self, filename, fence=None, height_accuracy=-1):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        num_wp = self.load_mission(filename)
        if self.mavproxy is not None:
            self.mavproxy.send('wp list\n')
        if fence is not None:
            self.load_fence(fence)
            if self.mavproxy is not None:
                self.mavproxy.send('fence list\n')
        # self.install_terrain_handlers_context()
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(1, num_wp-1)
        self.wait_disarmed(timeout=120) # give quadplane a long time to land

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
                                     mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                     timeout=30)
        self.mav.motors_disarmed_wait()

    def EXTENDED_SYS_STATE(self):
        '''Check extended sys state works'''
        self.EXTENDED_SYS_STATE_SLT()

    def QAUTOTUNE(self):
        '''test Plane QAutoTune mode'''

        # this is a list of all parameters modified by QAUTOTUNE.  Set
        # them so that when the context is popped we get the original
        # values back:
        parameter_values = self.get_parameters([
            "Q_A_RAT_RLL_P",
            "Q_A_RAT_RLL_I",
            "Q_A_RAT_RLL_D",
            "Q_A_ANG_RLL_P",
            "Q_A_ACCEL_R_MAX",
            "Q_A_RAT_PIT_P",
            "Q_A_RAT_PIT_I",
            "Q_A_RAT_PIT_D",
            "Q_A_ANG_PIT_P",
            "Q_A_ACCEL_P_MAX",
            "Q_A_RAT_YAW_P",
            "Q_A_RAT_YAW_I",
            "Q_A_RAT_YAW_FLTE",
            "Q_A_ANG_YAW_P",
            "Q_A_ACCEL_Y_MAX",
        ])
        self.set_parameters(parameter_values)

        self.takeoff(15, mode='GUIDED')
        self.set_rc(3, 1500)
        self.change_mode("QLOITER")
        self.change_mode("QAUTOTUNE")
        tstart = self.get_sim_time()
        self.context_collect('STATUSTEXT')
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 5000:
                raise NotAchievedException("Did not get success message")
            try:
                self.wait_text("AutoTune: Success", timeout=1, check_context=True)
            except AutoTestTimeoutException:
                continue
            # got success message
            break
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.context_clear_collection('STATUSTEXT')

        self.progress("Landing to save gains")
        self.set_rc(3, 1200)
        self.wait_speed_vector(
            Vector3(float('nan'), float('nan'), 1.4),
            timeout=5,
        )
        self.wait_speed_vector(
            Vector3(0.0, 0.0, 0.0),
            timeout=20,
        )
        distance = self.distance_to_home()
        if distance > 20:
            raise NotAchievedException("wandered from home (distance=%f)" %
                                       (distance,))
        self.set_rc(3, 1000)
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 500:
                raise NotAchievedException("Did not get success message")
            self.send_mavlink_disarm_command()
            try:
                self.wait_text(
                    "AutoTune: Saved gains for Roll Pitch Yaw.*",
                    timeout=0.5,
                    check_context=True,
                    regex=True,
                    )
            except AutoTestTimeoutException:
                continue
            break

        self.wait_disarmed()
        self.reboot_sitl()  # far from home

    def takeoff(self, height, mode, timeout=30):
        """climb to specified height and set throttle to 1500"""
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode(mode)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        if mode == 'GUIDED':
            self.user_takeoff(alt_min=height, timeout=timeout)
            return
        self.set_rc(3, 1800)
        self.wait_altitude(height,
                           height+5,
                           relative=True,
                           timeout=timeout)
        self.set_rc(3, 1500)

    def do_RTL(self):
        self.change_mode("QRTL")
        self.wait_altitude(-5, 1, relative=True, timeout=60)
        self.wait_disarmed()
        self.zero_throttle()

    def fly_home_land_and_disarm(self, timeout=30):
        self.context_push()
        self.change_mode('LOITER')
        self.set_parameter('RTL_AUTOLAND', 2)
        filename = "QuadPlaneDalbyRTL.txt"
        self.progress("Using %s to fly home" % filename)
        self.load_generic_mission(filename)
        self.send_cmd_do_set_mode("RTL")
        self.wait_mode('AUTO')
        self.wait_current_waypoint(4)
        self.wait_statustext('Land descend started')
        self.wait_statustext('Land final started', timeout=60)
        self.wait_disarmed(timeout=timeout)
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        # the following command is accepted, but doesn't actually
        # work!  Should be able to remove check_afterwards!
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode('MANUAL')
        self.context_pop()

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

    def GyroFFT(self):
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

    def PIDTuning(self):
        '''Test PID Tuning'''
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestQuadPlane, self).PIDTuning()

    def ParameterChecks(self):
        '''basic parameter checks'''
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

    def BootInAUTO(self):
        '''Test behaviour when booting in auto'''
        self.load_mission("mission.txt")
        self.set_parameters({
        })
        self.set_rc(5, 1000)
        self.wait_mode('AUTO')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.delay_sim_time(20)
        self.assert_current_waypoint(1)
        self.arm_vehicle()
        self.wait_altitude(9, 11, relative=True)  # value from mission file is 10
        distance = self.distance_to_home()
        # this distance check is very, very loose.  At time of writing
        # the vehicle actually pitches ~6 degrees on trakeoff,
        # wandering over 1m.
        if distance > 2:
            raise NotAchievedException("wandered from home (distance=%f)" %
                                       (distance,))
        self.change_mode('QLAND')
        self.wait_disarmed(timeout=60)

    def PilotYaw(self):
        '''Test pilot yaw in various modes'''
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

    def FwdThrInVTOL(self):
        '''test use of fwd motor throttle into wind'''
        self.set_parameters({"SIM_WIND_SPD": 25, # need very strong wind for this test
                             "SIM_WIND_DIR": 360,
                             "Q_WVANE_ENABLE": 1,
                             "Q_WVANE_GAIN": 1,
                             "STICK_MIXING": 0,
                             "Q_FWD_THR_USE": 2,
                             "SIM_ENGINE_FAIL": 2}) # we want to fail the forward thrust motor only

        self.takeoff(10, mode="QLOITER")
        self.set_rc(2, 1000)
        self.delay_sim_time(10)
        # Check that it is using some forward throttle
        fwd_thr_pwm = self.get_servo_channel_value(3)
        if fwd_thr_pwm < 1150 :
            raise NotAchievedException("fwd motor pwm command low, want >= 1150 got %f" % (fwd_thr_pwm))
        # check that pitch is on limit
        m = self.mav.recv_match(type='ATTITUDE', blocking=True)
        pitch = math.degrees(m.pitch)
        if abs(pitch + 3.0) > 0.5 :
            raise NotAchievedException("pitch should be -3.0 +- 0.5 deg, got %f" % (pitch))
        self.set_rc(2, 1500)
        self.delay_sim_time(5)
        loc1 = self.mav.location()
        self.set_parameter("SIM_ENGINE_MUL", 0) # simulate a complete loss of forward motor thrust
        self.delay_sim_time(20)
        self.change_mode('QLAND')
        self.wait_disarmed(timeout=60)
        loc2 = self.mav.location()
        position_drift = self.get_distance(loc1, loc2)
        if position_drift > 5.0 :
            raise NotAchievedException("position drift high, want < 5.0 m got %f m" % (position_drift))

    def Weathervane(self):
        '''test nose-into-wind functionality'''
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

    def QAssist(self):
        '''QuadPlane Assist tests'''
        self.takeoff(10, mode="QHOVER")
        self.set_rc(3, 1800)
        self.change_mode("FBWA")

        # disable stall prevention so roll angle is not limited
        self.set_parameter("STALL_PREVENTION", 0)

        thr_min_pwm = self.get_parameter("Q_M_PWM_MIN")
        lim_roll_deg = self.get_parameter("ROLL_LIMIT_DEG")
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

        # Test angle assist
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

        # Test alt assist, climb to 60m and set assist alt to 50m
        self.context_push()
        guided_loc = self.home_relative_loc_ne(0, 0)
        guided_loc.alt = 60
        self.change_mode("GUIDED")
        self.send_do_reposition(guided_loc)
        self.wait_altitude(58, 62, relative=True)
        self.set_parameter("Q_ASSIST_ALT", 50)

        # Try and descent to 40m
        guided_loc.alt = 40
        self.send_do_reposition(guided_loc)

        # Expect alt assist to kick in, eg "Alt assist 48.9m"
        self.wait_statustext(r"Alt assist \d*.\d*m", regex=True, timeout=100)

        # Test transition timeout, should switch to QRTL
        self.set_parameter("Q_TRANS_FAIL_ACT", 1)
        self.set_parameter("Q_TRANS_FAIL", 10)
        self.wait_mode("QRTL")

        self.context_pop()

        self.wait_disarmed(timeout=200)

    def LoiterAltQLand(self):
        '''test loitering and qland with terrain involved'''
        self.LoiterAltQLand_Terrain(
            home="LakeGeorgeLookout",
            ofs_n=0,
            ofs_e=300,
        )
#        self.LoiterAltQLand_Terrain(
#            home="KalaupapaCliffs",
#            ofs_n=500,
#            ofs_e=500,
#        )
        self.LoiterAltQLand_Relative()

    def LoiterAltQLand_Relative(self):
        '''test failsafe where vehicle loiters in fixed-wing mode to a
        specific altitude then changes mode to QLAND'''
        self.set_parameters({
            'BATT_MONITOR': 4,  # LoiterAltQLand
            'BATT_FS_LOW_ACT': 6,  # LoiterAltQLand
        })
        self.reboot_sitl()
        takeoff_alt = 5
        self.takeoff(takeoff_alt, mode='QLOITER')
        loc = self.mav.location()
        self.location_offset_ne(loc, 500, 500)
        new_alt = 100
        initial_altitude = self.get_altitude(relative=False, timeout=2)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            new_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_altitude(
            new_alt-1,
            new_alt+1,
            timeout=60,
            relative=True,
            minimum_duration=10)
        self.wait_location(loc, timeout=120, accuracy=100)
        self.progress("Triggering failsafe")
        self.set_parameter('BATT_LOW_VOLT', 50)
        self.wait_mode(25)  # LoiterAltQLand
        self.drain_mav()
        m = self.assert_receive_message('POSITION_TARGET_GLOBAL_INT', very_verbose=True)
        q_rtl_alt = self.get_parameter('Q_RTL_ALT')
        expected_alt = initial_altitude - takeoff_alt + q_rtl_alt

        if abs(m.alt - expected_alt) > 20:
            raise NotAchievedException("Unexpected altitude; expected=%f got=%f" %
                                       (expected_alt, m.alt))
        self.assert_mode('LOITERALTQLAND')
        self.wait_mode('QLAND')
        alt = self.get_altitude(relative=True)
        if abs(alt - q_rtl_alt) > 2:
            raise NotAchievedException("qland too late; want=%f got=%f" %
                                       (alt, q_rtl_alt))

        self.wait_disarmed(timeout=300)

    def LoiterAltQLand_Terrain(self,
                               home=None,
                               ofs_n=None,
                               ofs_e=None,
                               reposition_alt=100):
        '''test failsafe where vehicle loiters in fixed-wing mode to a
        specific altitude then changes mode to QLAND'''
        self.context_push()
        self.install_terrain_handlers_context()
        self.set_parameters({
            'BATT_MONITOR': 4,  # LoiterAltQLand
            'BATT_FS_LOW_ACT': 6,  # LoiterAltQLand
            'TERRAIN_FOLLOW': 1,  # enabled in all modes
        })
        self.customise_SITL_commandline(
            ["--home", home]
        )
        takeoff_alt = 5
        self.takeoff(takeoff_alt, mode='QLOITER')
        loc = self.mav.location()
        self.location_offset_ne(loc, ofs_n, ofs_e)
        initial_altitude = self.get_altitude(relative=False, timeout=2)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            reposition_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_altitude(
            reposition_alt-1,
            reposition_alt+1,
            timeout=60,
            relative=True,
            minimum_duration=10)

        self.wait_location(loc, timeout=500, accuracy=100)

        self.progress("Triggering failsafe")
        self.set_parameter('BATT_LOW_VOLT', 50)
        self.wait_mode(25)  # LoiterAltQLand
        terrain_alt = self.get_terrain_height(verbose=True)
        self.drain_mav()
        m = self.assert_receive_message('POSITION_TARGET_GLOBAL_INT', very_verbose=True)
        q_rtl_alt = self.get_parameter('Q_RTL_ALT')
        expected_alt = terrain_alt + q_rtl_alt

        if abs(m.alt - expected_alt) > 20:
            raise NotAchievedException("Unexpected altitude; expected=%f got=%f" %
                                       (expected_alt, m.alt))
        self.assert_mode('LOITERALTQLAND')
        self.wait_mode('QLAND')
        alt = initial_altitude + self.get_altitude(relative=True)
        if abs(alt - expected_alt) > 10:
            raise NotAchievedException("qland too late; want=%f got=%f" %
                                       (expected_alt, alt))

        self.wait_disarmed(timeout=300)
        self.zero_throttle()
        self.reset_SITL_commandline()
        self.context_pop()

    def GUIDEDToAUTO(self):
        '''Test using GUIDED mode for takeoff before shifting to auto'''
        self.load_mission("mission.txt")
        self.takeoff(30, mode='GUIDED')

        # extra checks would go here
        self.assert_not_receiving_message('CAMERA_FEEDBACK')

        self.change_mode('AUTO')
        self.wait_current_waypoint(3)
        self.change_mode('QRTL')
        self.wait_disarmed(timeout=240)

    def Tailsitter(self):
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

    def setup_ICEngine_vehicle(self):
        '''restarts SITL with an IC Engine setup'''
        model = "quadplane-ice"
        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath=self.model_defaults_filepath(model),
            wipe=False,
        )

    def ICEngine(self):
        '''Test ICE Engine support'''
        rc_engine_start_chan = 11
        self.setup_ICEngine_vehicle()

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
        self.wait_rpm(1, 65, 75, minimum_duration=1)
        self.progress("Setting engine-start RC switch to LOW")
        self.set_rc(rc_engine_start_chan, 1000)
        self.wait_rpm(1, 0, 0, minimum_duration=1)
        # ICE provides forward thrust, which can make us think we're flying:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.start_subtest("Testing throttle out in manual mode")
        self.change_mode('MANUAL')
        self.set_rc(3, 1700)
        self.wait_servo_channel_value(3, 2000)
        self.set_parameter("ICE_OPTIONS", 4)
        # remember that throttle is reversed!
        self.wait_servo_channel_value(3, 1300)
        self.change_mode('FBWA')
        self.wait_servo_channel_value(3, 2000)

    def ICEngineMission(self):
        '''Test ICE Engine Mission support'''
        rc_engine_start_chan = 11
        self.setup_ICEngine_vehicle()

        self.load_mission("mission.txt")
        self.wait_ready_to_arm()
        self.set_rc(rc_engine_start_chan, 2000)
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_disarmed(timeout=300)

    def MAV_CMD_DO_ENGINE_CONTROL(self):
        '''test MAV_CMD_DO_ENGINE_CONTROL mavlink command'''

        expected_idle_rpm_min = 65
        expected_idle_rpm_max = 75
        expected_starter_rpm_min = 345
        expected_starter_rpm_max = 355

        rc_engine_start_chan = 11
        self.setup_ICEngine_vehicle()

        self.wait_ready_to_arm()

        for method in self.run_cmd, self.run_cmd_int:
            self.change_mode('MANUAL')
            self.set_rc(rc_engine_start_chan, 1500)  # allow motor to run
            self.wait_rpm(1, 0, 0, minimum_duration=1)
            self.arm_vehicle()
            self.wait_rpm(1, 0, 0, minimum_duration=1)
            self.start_subtest("Start motor")
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1)
            self.wait_rpm(1, expected_starter_rpm_min, expected_starter_rpm_max)
            self.wait_rpm(1, expected_idle_rpm_min, expected_idle_rpm_max, minimum_duration=10)

            # starting the motor while it is running is failure
            # (probably wrong, but that's how this works):
            self.start_subtest("try start motor again")
            self.context_collect('STATUSTEXT')
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
            self.wait_statustext("already running", check_context=True)
            self.context_stop_collecting('STATUSTEXT')
            # shouldn't affect run state:
            self.wait_rpm(1, expected_idle_rpm_min, expected_idle_rpm_max, minimum_duration=1)

            self.start_subtest("Stop motor")
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.start_subtest("Stop motor (again)")
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.start_subtest("Check start chan control disable")
            old_start_channel_value = self.get_rc_channel_value(rc_engine_start_chan)
            self.set_rc(rc_engine_start_chan, 1000)
            self.context_collect('STATUSTEXT')
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
            self.wait_statustext("start control disabled", check_context=True)
            self.context_stop_collecting('STATUSTEXT')
            self.set_rc(rc_engine_start_chan, old_start_channel_value)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.start_subtest("test start-at-height")
            self.wait_rpm(1, 0, 0, minimum_duration=1)
            self.context_collect('STATUSTEXT')
            method(
                mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL,
                p1=1,  # start
                p3=15.5, # ... at 15.5 metres
            )
            self.wait_statustext("height set to 15.5m", check_context=True)
            self.wait_rpm(1, 0, 0, minimum_duration=2)

            self.takeoff(20, mode='GUIDED')
            self.wait_rpm(1, expected_starter_rpm_min, expected_starter_rpm_max, minimum_duration=1)
            self.wait_statustext("Engine running", check_context=True)
            self.context_stop_collecting('STATUSTEXT')

            # stop the motor again:
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.change_mode('QLAND')
            self.wait_disarmed()

    def Ship(self):
        '''Ensure we can take off from simulated ship'''
        self.context_push()
        self.set_parameters({
            'SIM_SHIP_ENABLE': 1,
            'SIM_SHIP_SPEED': 1,  # the default of 3 will break this test
        })
        self.change_mode('QLOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1700)
        # self.delay_sim_time(1)
        # self.send_debug_trap()
        # output here is a bit weird as we also receive altitude from
        # the simulated ship....
        self.wait_altitude(20, 30, relative=True)
        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()

    def MidAirDisarmDisallowed(self):
        '''Check disarm behaviour in Q-mode'''
        self.start_subtest("Basic arm in qloiter")
        self.set_parameter("FLIGHT_OPTIONS", 0)
        self.change_mode('QLOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        self.context_push()
        self.start_subtest("Ensure disarming in q-modes on ground works")
        self.set_parameter("FLIGHT_OPTIONS", 1 << 11)
        self.arm_vehicle()
        self.disarm_vehicle()  # should be OK as we're not flying yet
        self.context_pop()

        self.start_subtest("Ensure no disarming mid-air")
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(5, 50, relative=True)
        self.set_rc(3, 1000)
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

        self.change_mode('QLAND')
        self.wait_disarmed()

        self.start_subtest("Check we can disarm after a short period on the ground")
        self.takeoff(5, 'QHOVER')
        self.change_mode('QLAND')
        try:
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)
            self.wait_extended_sys_state(
                landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                timeout=60
            )
        except Exception:
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 0)
            raise

        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, -1)
        self.disarm_vehicle()

    def MAV_CMD_NAV_LOITER_TO_ALT(self, target_system=1, target_component=1):
        '''ensure consecutive loiter to alts work'''
        self.load_mission('mission.txt')
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(4, timeout=240)
        self.assert_altitude(120, accuracy=5, relative=True)
        self.delay_sim_time(30)
        self.assert_altitude(120, accuracy=5, relative=True)
        self.set_current_waypoint(5)
        self.wait_altitude(altitude_min=65, altitude_max=75, relative=True)
        if self.current_waypoint() != 5:
            raise NotAchievedException("Should pass 90m before passing waypoint 5")
        self.wait_disarmed(timeout=300)

    def Mission(self):
        '''fly the OBC 2016 mission in Dalby'''
        self.load_mission("Dalby-OBC2016.txt")
        self.load_fence("Dalby-OBC2016-fence.txt")
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

    def VTOLLandSpiral(self):
        '''check spiral-to-alt option for landing'''
        self.fly_mission('mission.txt')
        self.set_parameter('WP_LOITER_RAD', -self.get_parameter('WP_LOITER_RAD'))
        self.set_current_waypoint(0, check_afterwards=False)
        self.fly_mission('mission.txt')

    def VTOLQuicktune(self):
        '''VTOL Quicktune'''
        self.install_applet_script_context("VTOL-quicktune.lua")

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_SPEEDUP": 20, # need to give some cycles to lua
            "RC7_OPTION": 300,
        })

        self.reboot_sitl()

        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "QUIK_ENABLE" : 1,
            "QUIK_DOUBLE_TIME" : 5, # run faster for autotest
            })

        self.scripting_restart()
        self.wait_text("Quicktune for quadplane loaded", check_context=True)

        self.wait_ready_to_arm()
        self.change_mode("QLOITER")
        self.arm_vehicle()
        self.takeoff(20, 'QLOITER')

        # use rc switch to start tune
        self.set_rc(7, 1500)

        self.wait_text("Tuning: starting tune", check_context=True)
        for axis in ['RLL', 'PIT', 'YAW']:
            self.wait_text("Starting %s tune" % axis, check_context=True)
            self.wait_text("Tuning: %s_D done" % axis, check_context=True, timeout=120)
            self.wait_text("Tuning: %s_P done" % axis, check_context=True, timeout=120)
            self.wait_text("Tuning: %s done" % axis, check_context=True, timeout=120)
        self.wait_text("Tuning: YAW done", check_context=True, timeout=120)

        # to test aux function method, use aux fn for save
        self.run_auxfunc(300, 2)
        self.wait_text("Tuning: saved", check_context=True)
        self.change_mode("QLAND")

        self.wait_disarmed(timeout=120)

    def PrecisionLanding(self):
        '''VTOL precision landing'''

        self.install_applet_script_context("plane_precland.lua")

        here = self.mav.location()
        target = self.offset_location_ne(here, 20, 0)

        self.set_parameters({
            "SCR_ENABLE": 1,
            "PLND_ENABLED": 1,
            "PLND_TYPE": 4,
            "SIM_PLD_ENABLE":   1,
            "SIM_PLD_LAT" : target.lat,
            "SIM_PLD_LON" : target.lng,
            "SIM_PLD_HEIGHT" : 0,
            "SIM_PLD_ALT_LMT" : 50,
            "SIM_PLD_DIST_LMT" : 30,
            "RNGFND1_TYPE": 100,
            "RNGFND1_PIN" : 0,
            "RNGFND1_SCALING" : 12.2,
            "RNGFND1_MAX_CM" : 5000,
            "RNGFND_LANDING" : 1,
        })

        self.reboot_sitl()

        self.set_parameters({
            "PLND_ALT_CUTOFF" : 5,
            "SIM_SPEEDUP" : 10,
            })

        self.context_collect('STATUSTEXT')

        self.scripting_restart()
        self.wait_text("PLND: Loaded", check_context=True)

        self.wait_ready_to_arm()
        self.change_mode("GUIDED")
        self.arm_vehicle()
        self.takeoff(60, 'GUIDED')
        self.wait_altitude(58, 62, relative=True)
        self.drain_mav()
        self.change_mode("QRTL")

        self.wait_text("PLND: Target Acquired", check_context=True, timeout=60)

        self.wait_disarmed(timeout=180)
        loc2 = self.mav.location()
        error = self.get_distance(target, loc2)
        self.progress("Target error %.1fm" % error)
        if error > 2:
            raise NotAchievedException("too far from target %.1fm" % error)

    def ShipLanding(self):
        '''ship landing test'''
        self.install_applet_script_context("plane_ship_landing.lua")

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_SHIP_ENABLE": 1,
            "SIM_SHIP_SPEED": 5,
            "SIM_SHIP_DSIZE": 10,
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": 17,
            "FOLL_OFS_TYPE": 1,
            "SIM_TERRAIN" : 0,
            "TERRAIN_ENABLE" : 0,
        })

        self.load_mission("takeoff100.txt")

        self.reboot_sitl(check_position=False)

        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SHIP_ENABLE" : 1,
            "SIM_SPEEDUP" : 10,
            })

        self.scripting_restart()
        self.wait_text("ShipLanding: loaded", check_context=True)

        self.wait_ready_to_arm()
        self.change_mode("AUTO")
        self.arm_vehicle()
        self.wait_altitude(95, 105, relative=True, timeout=90)
        self.drain_mav()

        self.wait_text("Mission complete, changing mode to RTL", check_context=True, timeout=60)
        self.wait_text("Descending for approach", check_context=True, timeout=60)
        self.wait_text("Reached target altitude", check_context=True, timeout=120)
        self.wait_text("Starting approach", check_context=True, timeout=120)
        self.wait_text("Land complete", check_context=True, timeout=120)

        self.wait_disarmed(timeout=180)

        # we confirm successful landing on the ship from our ground speed. The
        # deck is just 10m in size, so we must be within 10m if we are moving
        # with the deck
        self.wait_groundspeed(4.8, 5.2)

    def RCDisableAirspeedUse(self):
        '''check disabling airspeed using RC switch'''
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

        self.progress("Disabling airspeed sensor")
        self.context_push()
        self.set_rc(9, 2000)
        self.set_parameters({
            "COMPASS_ENABLE": 0,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "COMPASS_USE": 0,
            "COMPASS_USE2": 0,
            "COMPASS_USE3": 0,
            "ARMING_CHECK": 589818,  # from a logfile, disables compass
        })

        self.reboot_sitl()

        self.context_collect('STATUSTEXT')
        self.wait_prearm_sys_status_healthy(timeout=120)
        self.change_mode('QLOITER')
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(10, 30, relative=True)
        self.change_mode('FBWA')
        self.wait_statustext('Transition done')
        # the vehicle stays in DCM until there's velocity - make sure
        # we did go to EK3 evenutally, 'though:
        self.wait_statustext('EKF3 active', check_context=True)

        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()

    def mission_MAV_CMD_DO_VTOL_TRANSITION(self):
        '''mission item forces transition'''
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, 30),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                p1=mavutil.mavlink.MAV_VTOL_STATE_MC
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 300, 200, 30),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                p1=mavutil.mavlink.MAV_VTOL_STATE_FW
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 200, 30),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.wait_current_waypoint(4)
        self.wait_servo_channel_value(5, 1200, comparator=operator.gt)
        self.wait_current_waypoint(6)
        self.wait_servo_channel_value(5, 1000, comparator=operator.eq, timeout=90)

        self.fly_home_land_and_disarm()

    def mavlink_MAV_CMD_DO_VTOL_TRANSITION(self):
        '''mavlink command forces transition during mission'''
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.wait_current_waypoint(2)
        self.wait_servo_channel_value(5, 1000, comparator=operator.eq, timeout=90)

        for command in self.run_cmd, self.run_cmd_int:
            command(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, p1=mavutil.mavlink.MAV_VTOL_STATE_MC)
            self.wait_servo_channel_value(5, 1200, comparator=operator.gt, timeout=300)
            command(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, p1=mavutil.mavlink.MAV_VTOL_STATE_FW)
            self.wait_servo_channel_value(5, 1000, comparator=operator.eq, timeout=90)

        self.fly_home_land_and_disarm()

    def TransitionMinThrottle(self):
        '''Ensure that TKOFF_THR_MIN is applied during the forward transition'''
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)
        self.set_parameter('TKOFF_THR_MIN', 80)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.wait_current_waypoint(2)
        # Wait for 5 seconds into the transition.
        self.delay_sim_time(5)
        # Ensure TKOFF_THR_MIN is still respected.
        thr_min = self.get_parameter('TKOFF_THR_MIN')
        self.wait_servo_channel_value(3, 1000+thr_min*10, comparator=operator.eq)

        self.fly_home_land_and_disarm()

    def MAV_CMD_NAV_TAKEOFF(self):
        '''test issuing takeoff command via mavlink'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=5)
        self.wait_altitude(4.5, 5.5, minimum_duration=5, relative=True)
        self.change_mode('QLAND')
        self.wait_disarmed()

        self.start_subtest("Check NAV_TAKEOFF is above current location, not home location")
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        # reset home 20 metres above current location
        current_alt_abs = self.get_altitude(relative=False)

        loc = self.mav.location()

        home_z_ofs = 20
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            p5=loc.lat,
            p6=loc.lng,
            p7=current_alt_abs + home_z_ofs,
        )

        self.arm_vehicle()
        takeoff_alt = 5
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=takeoff_alt)
        self.wait_altitude(
            current_alt_abs + takeoff_alt - 0.5,
            current_alt_abs + takeoff_alt + 0.5,
            minimum_duration=5,
            relative=False,
        )
        self.change_mode('QLAND')
        self.wait_disarmed()

        self.reboot_sitl()  # unlock home position

    def Q_GUIDED_MODE(self):
        '''test moving in VTOL mode with SET_POSITION_TARGET_GLOBAL_INT'''
        self.set_parameter('Q_GUIDED_MODE', 1)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=15)
        self.wait_altitude(14, 16, relative=True)

        loc = self.mav.location()
        self.location_offset_ne(loc, 50, 50)

        # set position target
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            30,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_location(loc, timeout=120)

        self.fly_home_land_and_disarm()

    def DCMClimbRate(self):
        '''Test the climb rate measurement in DCM with and without GPS'''
        self.wait_ready_to_arm()

        self.change_mode('QHOVER')
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(30, 50, relative=True)

        # Start Descending
        self.set_rc(3, 1000)
        self.wait_climbrate(-5, -0.5, timeout=10)

        # Switch to DCM
        self.set_parameter('AHRS_EKF_TYPE', 0)
        self.delay_sim_time(5)

        # Start Climbing
        self.set_rc(3, 2000)
        self.wait_climbrate(0.5, 5, timeout=10)

        # Kill any GPSs
        self.set_parameters({
            'SIM_GPS_DISABLE': 1,
            'SIM_GPS2_DISABLE': 1,
        })
        self.delay_sim_time(5)

        # Start Descending
        self.set_rc(3, 1000)
        self.wait_climbrate(-5, -0.5, timeout=10)

        # Force disarm
        self.disarm_vehicle(force=True)

    def RTL_AUTOLAND_1(self):
        '''test behaviour when RTL_AUTOLAND==1'''

        self.set_parameters({
            "RTL_AUTOLAND": 1,
        })

        # when RTL is entered and RTL_AUTOLAND is 1 we should fly home
        # then to the landing sequence.  This mission puts the landing
        # sequence well to the West of home so if we go directly there
        # we won't come within 200m of home
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            # fly North
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 30),
            # add a waypoint 1km North (which we will look for and trigger RTL
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 30),

            # *exciting* landing sequence is ~1km West and points away from Home.
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1000, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1300, 15),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1600, 5),
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, 0, -1750, 0),
        ])
        self.check_mission_upload_download(wps)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.wait_current_waypoint(3)  # will be 2km North here
        self.change_mode('RTL')

        self.wait_distance_to_home(100, 200, timeout=120)
        self.wait_current_waypoint(7)

        self.fly_home_land_and_disarm()

    def send_reposition_to_loc(self, loc):
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            20,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )

    def reposition_to_loc(self, loc, accuracy=100):
        self.send_reposition_to_loc(loc)
        self.wait_location(
            loc,
            accuracy=accuracy,
            minimum_duration=20,
            timeout=120,
        )

    def RTL_AUTOLAND_1_FROM_GUIDED(self):
        '''test behaviour when RTL_AUTOLAND==1 and entering from guided'''

        self.set_parameters({
            "RTL_AUTOLAND": 1,
        })

        # when RTL is entered and RTL_AUTOLAND is 1 we should fly home
        # then to the landing sequence.  This mission puts the landing
        # sequence well to the West of home so if we go directly there
        # we won't come within 200m of home
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            # fly North
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 30),
            # add a waypoint 1km North (which we will look for and trigger RTL
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 30),

            # *exciting* landing sequence is ~1km West and points away from Home.
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1000, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1300, 15),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1600, 5),
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, 0, -1750, 0),
        ])
        self.check_mission_upload_download(wps)
        self.set_current_waypoint(0, check_afterwards=False)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        here = self.mav.location()
        guided_loc = self.offset_location_ne(here, 500, -500)

        self.arm_vehicle()
        self.wait_current_waypoint(3)  # will be 2km North here
        self.reposition_to_loc(guided_loc)
        self.send_cmd_do_set_mode('RTL')

        self.wait_distance_to_home(100, 200, timeout=120)
        self.wait_current_waypoint(7)

        self.fly_home_land_and_disarm()

    def tests(self):
        '''return list of all tests'''

        ret = super(AutoTestQuadPlane, self).tests()
        ret.extend([
            self.FwdThrInVTOL,
            self.AirMode,
            self.TestMotorMask,
            self.PilotYaw,
            self.ParameterChecks,
            self.QAUTOTUNE,
            self.TestLogDownload,
            self.TestLogDownloadWrap,
            self.EXTENDED_SYS_STATE,
            self.Mission,
            self.Weathervane,
            self.QAssist,
            self.GyroFFT,
            self.Tailsitter,
            self.ICEngine,
            self.ICEngineMission,
            self.MAV_CMD_DO_ENGINE_CONTROL,
            self.MidAirDisarmDisallowed,
            self.GUIDEDToAUTO,
            self.BootInAUTO,
            self.Ship,
            self.MAV_CMD_NAV_LOITER_TO_ALT,
            self.LoiterAltQLand,
            self.VTOLLandSpiral,
            self.VTOLQuicktune,
            self.PrecisionLanding,
            self.ShipLanding,
            Test(self.MotorTest, kwargs={  # tests motors 4 and 2
                "mot1_servo_chan": 8,  # quad-x second motor cw from f-r
                "mot4_servo_chan": 6,  # quad-x third motor cw from f-r
                "wait_finish_text": False,
                "quadplane": True,
            }),
            self.RCDisableAirspeedUse,
            self.mission_MAV_CMD_DO_VTOL_TRANSITION,
            self.mavlink_MAV_CMD_DO_VTOL_TRANSITION,
            self.TransitionMinThrottle,
            self.MAV_CMD_NAV_TAKEOFF,
            self.Q_GUIDED_MODE,
            self.DCMClimbRate,
            self.RTL_AUTOLAND_1,  # as in fly-home then go to landing sequence
            self.RTL_AUTOLAND_1_FROM_GUIDED,  # as in fly-home then go to landing sequence
        ])
        return ret
