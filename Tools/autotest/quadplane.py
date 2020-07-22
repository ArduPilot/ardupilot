#!/usr/bin/env python

# Fly ArduPlane QuadPlane in SITL
from __future__ import print_function
import os
import numpy
import math

from pymavlink import mavutil

from common import AutoTest
from common import AutoTestTimeoutException, NotAchievedException

from pysim import vehicleinfo
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

    def default_frame(self):
        return "quadplane"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def log_name(self):
        return "QuadPlane"

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduPlane_Tests/" + name + "/"

    def apply_defaultfile_parameters(self):
        # plane passes in a defaults_filepath in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        return self.model_defaults_filepath("ArduPlane",self.frame)

    def is_plane(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("LAND_DISARMDELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("LAND_DISARMDELAY", delay)

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

    def fly_mission(self, filename, fence=None, height_accuracy=-1):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        self.load_mission(filename)
        if fence is not None:
            self.load_fence(fence)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mavproxy.send('mode AUTO\n')
        self.wait_mode('AUTO')
        self.wait_waypoint(1, 19, max_dist=60, timeout=1200)

        self.wait_disarmed(timeout=120) # give quadplane a long time to land
        # wait for blood sample here
        self.mavproxy.send('wp set 20\n')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(20, 34, max_dist=60, timeout=1200)

        self.wait_disarmed(timeout=120) # give quadplane a long time to land
        self.progress("Mission OK")

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
            except AutoTestTimeoutException as e:
                continue
            break
        self.wait_disarmed()

    def takeoff(self, height, mode):
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

    def fly_home_land_and_disarm(self, timeout=30):
        self.set_parameter("LAND_TYPE", 0)
        filename = "flaps.txt"
        self.progress("Using %s to fly home" % filename)
        self.load_mission(filename)
        self.change_mode("AUTO")
        self.mavproxy.send('wp set 7\n')
        self.wait_disarmed(timeout=timeout)

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
            raise NotAchievedException("Did not detect a motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            self.progress("Detected motor peak at %fHz, throttle %f%%, %fdB" % (freq, vfr_hud.throttle, peakdb))

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
            self.set_parameter("AHRS_EKF_TYPE", 10)

            self.set_parameter("INS_LOG_BAT_MASK", 3)
            self.set_parameter("INS_LOG_BAT_OPT", 0)
            self.set_parameter("INS_GYRO_FILTER", 100)
            self.set_parameter("LOG_BITMASK", 45054)
            self.set_parameter("LOG_DISARMED", 0)
            self.set_parameter("SIM_DRIFT_SPEED", 0)
            self.set_parameter("SIM_DRIFT_TIME", 0)
            # enable a noisy motor peak
            self.set_parameter("SIM_GYR_RND", 20)
            # enabling FFT will also enable the arming check, self-testing the functionality
            self.set_parameter("FFT_ENABLE", 1)
            self.set_parameter("FFT_MINHZ", 80)
            self.set_parameter("FFT_MAXHZ", 350)
            self.set_parameter("FFT_SNR_REF", 10)
            self.set_parameter("FFT_WINDOW_SIZE", 128)
            self.set_parameter("FFT_WINDOW_OLAP", 0.75)

            # Step 1: inject a very precise noise peak at 250hz and make sure the in-flight fft
            # can detect it really accurately. For a 128 FFT the frequency resolution is 8Hz so
            # a 250Hz peak should be detectable within 5%
            self.set_parameter("SIM_VIB_FREQ_X", 250)
            self.set_parameter("SIM_VIB_FREQ_Y", 250)
            self.set_parameter("SIM_VIB_FREQ_Z", 250)

            self.reboot_sitl()

            # find a motor peak
            self.hover_and_check_matched_frequency(-15, 100, 350, 128, 250)

            # Step 2: inject actual motor noise and use the standard length FFT to track it
            self.set_parameter("SIM_VIB_MOT_MAX", 350)
            self.set_parameter("FFT_WINDOW_SIZE", 32)
            self.set_parameter("FFT_WINDOW_OLAP", 0.5)

            self.reboot_sitl()
            # find a motor peak
            freq = self.hover_and_check_matched_frequency(-15, 200, 300, 32)

            # Step 3: add a FFT dynamic notch and check that the peak is squashed
            self.set_parameter("INS_LOG_BAT_OPT", 2)
            self.set_parameter("INS_HNTCH_ENABLE", 1)
            self.set_parameter("INS_HNTCH_FREQ", freq)
            self.set_parameter("INS_HNTCH_REF", 1.0)
            self.set_parameter("INS_HNTCH_ATT", 50)
            self.set_parameter("INS_HNTCH_BW", freq/2)
            self.set_parameter("INS_HNTCH_MODE", 4)
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
            if numpy.amax(psd["X"][ignore_bins:]) < -10:
                self.progress("Did not detect a motor peak, found %f at %f dB" % (freq, numpy.amax(psd["X"][ignore_bins:])))
            else:
                raise NotAchievedException("Detected motor peak at %f Hz" % (freq))

            # Step 4: take off as a copter land as a plane, make sure we track
            self.progress("Flying with gyro FFT - vtol to plane")
            self.load_mission("quadplane-gyro-mission.txt")
            self.mavproxy.send('wp list\n')
            self.mavproxy.expect('Requesting [0-9]+ waypoints')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.mavproxy.send('mode AUTO\n')
            self.wait_mode('AUTO')
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
            "QAutoTune": "See https://github.com/ArduPilot/ardupilot/issues/10411",
            "FRSkyPassThrough": "Currently failing",
            "CPUFailsafe": "servo channel values not scaled like ArduPlane",
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

    def CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        self.plane_CPUFailsafe()

    def test_qassist(self):
        # find a motor peak
        self.takeoff(10, mode="QHOVER")
        self.set_rc(3, 1800)
        self.change_mode("FBWA")
        thr_min_pwm = self.get_parameter("Q_THR_MIN_PWM")
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
        self.set_rc(3, 1500)

        self.context_push()
        self.progress("Rolling over hard")
        self.set_rc(1, 1000)
        self.wait_roll(-65, 5)
        self.progress("Killing servo outputs to force qassist to help")
        self.set_parameter("SERVO1_MIN", 1480)
        self.set_parameter("SERVO1_MAX", 1480)
        self.set_parameter("SERVO1_TRIM", 1480)
        self.progress("Trying to roll over hard the other way")
        self.set_rc(1, 2000)
        self.progress("Waiting for qassist (angle) to kick in")
        self.wait_servo_channel_value(5, 1100, timeout=30, comparator=operator.gt)
        self.wait_roll(85, 5)
        self.context_pop()

        self.change_mode("RTL")
        self.delay_sim_time(20)
        self.change_mode("QRTL")
        self.wait_disarmed(timeout=300)

    def tests(self):
        '''return list of all tests'''

        ret = super(AutoTestQuadPlane, self).tests()
        ret.extend([
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

            ("Mission", "Dalby Mission",
             lambda: self.fly_mission("Dalby-OBC2016.txt", "Dalby-OBC2016-fence.txt")),

            ("QAssist",
             "QuadPlane Assist tests",
             self.test_qassist),

            ("GyroFFT", "Fly Gyro FFT",
             self.fly_gyro_fft),

            ("LogUpload",
             "Log upload",
             self.log_upload),
        ])
        return ret
