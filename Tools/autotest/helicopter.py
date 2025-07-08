'''
Fly Helicopter in SITL

AP_FLAKE8_CLEAN
'''

from arducopter import AutoTestCopter

import vehicle_test_suite
from vehicle_test_suite import NotAchievedException, AutoTestTimeoutException

from pymavlink import mavutil
from pysim import vehicleinfo

import copy
import operator


class AutoTestHelicopter(AutoTestCopter):

    sitl_start_loc = mavutil.location(40.072842, -105.230575, 1586, 0)     # Sparkfun AVC Location

    def vehicleinfo_key(self):
        return 'Helicopter'

    def log_name(self):
        return "HeliCopter"

    def default_frame(self):
        return "heli"

    def sitl_start_location(self):
        return self.sitl_start_loc

    def default_speedup(self):
        '''Heli seems to be race-free'''
        return 100

    def is_heli(self):
        return True

    def rc_defaults(self):
        ret = super(AutoTestHelicopter, self).rc_defaults()
        ret[8] = 1000
        ret[3] = 1000 # collective
        return ret

    @staticmethod
    def get_position_armable_modes_list():
        '''filter THROW mode out of armable modes list; Heli is special-cased'''
        ret = AutoTestCopter.get_position_armable_modes_list()
        ret = filter(lambda x : x != "THROW", ret)
        return ret

    def loiter_requires_position(self):
        self.progress("Skipping loiter-requires-position for heli; rotor runup issues")

    def get_collective_out(self):
        servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
        chan_pwm = (servo.servo1_raw + servo.servo2_raw + servo.servo3_raw)/3.0
        return chan_pwm

    def RotorRunup(self):
        '''Test rotor runip'''
        # Takeoff and landing in Loiter
        TARGET_RUNUP_TIME = 10
        self.zero_throttle()
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
        coll = servo.servo1_raw
        coll = coll + 50
        self.set_parameter("H_RSC_RUNUP_TIME", TARGET_RUNUP_TIME)
        self.progress("Initiate Runup by putting some throttle")
        self.set_rc(8, 2000)
        self.set_rc(3, 1700)
        self.progress("Collective threshold PWM %u" % coll)
        tstart = self.get_sim_time()
        self.progress("Wait that collective PWM pass threshold value")
        servo = self.assert_receive_message(
            "SERVO_OUTPUT_RAW",
            condition=f'SERVO_OUTPUT_RAW.servo1_raw>{coll}'
        )
        runup_time = self.get_sim_time() - tstart
        self.progress("Collective is now at PWM %u" % servo.servo1_raw)
        self.mav.wait_heartbeat()
        if runup_time < TARGET_RUNUP_TIME:
            self.zero_throttle()
            self.set_rc(8, 1000)
            self.disarm_vehicle()
            self.mav.wait_heartbeat()
            raise NotAchievedException("Takeoff initiated before runup time complete %u" % runup_time)
        self.progress("Runup time %u" % runup_time)
        self.zero_throttle()
        self.land_and_disarm()
        self.mav.wait_heartbeat()

    # fly_avc_test - fly AVC mission
    def AVCMission(self):
        '''fly AVC mission'''
        self.change_mode('STABILIZE')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)

        # upload mission from file
        self.progress("# Load copter_AVC2013_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_AVC2013_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_AVC2013_mission failed")

        self.progress("Fly AVC mission from 1 to %u" % num_wp)
        self.set_current_waypoint(1)

        # wait for motor runup
        self.delay_sim_time(20)

        # switch into AUTO mode and raise throttle
        self.change_mode('AUTO')
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # set throttle to minimum
        self.zero_throttle()

        # wait for disarm
        self.wait_disarmed()
        self.progress("MOTORS DISARMED OK")

        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

        self.progress("AVC mission completed: passed!")

    def takeoff(self,
                alt_min=30,
                takeoff_throttle=1700,
                require_absolute=True,
                mode="STABILIZE",
                timeout=120):
        """Takeoff get to 30m altitude."""
        self.progress("TAKEOFF")
        self.change_mode(mode)
        if not self.armed():
            self.wait_ready_to_arm(require_absolute=require_absolute, timeout=timeout)
            self.zero_throttle()
            self.arm_vehicle()

        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        if self.get_parameter("H_RSC_MODE") == 4:
            self.context_collect('STATUSTEXT')
            self.wait_statustext("Governor Engaged", check_context=True)
        elif self.get_parameter("H_RSC_MODE") == 3:
            self.wait_rpm(1, 1300, 1400)
        else:
            self.wait_servo_channel_value(8, 1659, timeout=10)

        # wait for motor runup
        self.delay_sim_time(20)

        if mode == 'GUIDED':
            self.user_takeoff(alt_min=alt_min)
        else:
            self.set_rc(3, takeoff_throttle)
        self.wait_altitude(alt_min-1, alt_min+5, relative=True, timeout=timeout)
        self.hover()
        self.progress("TAKEOFF COMPLETE")

    def FlyEachFrame(self):
        '''Fly each supported internal frame'''
        vinfo = vehicleinfo.VehicleInfo()
        vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
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
            if not isinstance(defaults, list):
                defaults = [defaults]
            self.customise_SITL_commandline(
                [],
                defaults_filepath=defaults,
                model=model,
                wipe=True,
            )
            self.takeoff(10)
            self.do_RTL()

    def governortest(self):
        '''Test Heli Internal Throttle Curve and Governor'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli-gas'),
            model="heli-gas",
            wipe=True,
        )
        self.set_parameter("H_RSC_MODE", 4)
        self.takeoff(10)
        self.do_RTL()

    def hover(self):
        self.progress("Setting hover collective")
        self.set_rc(3, 1500)

    def PosHoldTakeOff(self):
        """ensure vehicle stays put until it is ready to fly"""
        self.set_parameter("PILOT_TKOFF_ALT", 700)
        self.change_mode('POSHOLD')
        self.zero_throttle()
        self.set_rc(8, 1000)
        self.wait_ready_to_arm()
        # Arm
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        self.wait_servo_channel_value(8, 1659, timeout=10)
        self.delay_sim_time(20)
        # check we are still on the ground...
        max_relalt = 1  # metres
        relative_alt = self.get_altitude(relative=True)
        if abs(relative_alt) > max_relalt:
            raise NotAchievedException("Took off prematurely (abs(%f)>%f)" %
                                       (relative_alt, max_relalt))

        self.progress("Pushing collective past half-way")
        self.set_rc(3, 1600)
        self.delay_sim_time(0.5)
        self.hover()

        # make sure we haven't already reached alt:
        relative_alt = self.get_altitude(relative=True)
        max_initial_alt = 1.5  # metres
        if abs(relative_alt) > max_initial_alt:
            raise NotAchievedException("Took off too fast (%f > %f" %
                                       (abs(relative_alt), max_initial_alt))

        self.progress("Monitoring takeoff-to-alt")
        self.wait_altitude(6, 8, relative=True, minimum_duration=5)
        self.progress("takeoff OK")

        self.land_and_disarm()

    def StabilizeTakeOff(self):
        """Fly stabilize takeoff"""
        self.change_mode('STABILIZE')
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        self.wait_servo_channel_value(8, 1659, timeout=10)
        self.delay_sim_time(20)
        # check we are still on the ground...
        relative_alt = self.get_altitude(relative=True)
        if abs(relative_alt) > 0.1:
            raise NotAchievedException("Took off prematurely")
        self.progress("Pushing throttle past half-way")
        self.set_rc(3, 1650)

        self.progress("Monitoring takeoff")
        self.wait_altitude(6.9, 8, relative=True)

        self.progress("takeoff OK")

        self.land_and_disarm()

    def SplineWaypoint(self, timeout=600):
        """ensure basic spline functionality works"""
        self.load_mission("copter_spline_mission.txt", strict=False)
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.delay_sim_time(20)
        self.change_mode("AUTO")
        self.set_rc(3, 1500)
        self.wait_disarmed(timeout=600)
        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

    def Autorotation(self, timeout=600):
        """Check engine-out behaviour"""
        self.context_push()
        start_alt = 100 # metres
        self.set_parameters({
            "AROT_ENABLE": 1,
            "H_RSC_AROT_ENBL": 1,
            "H_COL_LAND_MIN" : -2.0
        })
        bail_out_time = self.get_parameter('H_RSC_AROT_RUNUP')
        self.change_mode('POSHOLD')
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        self.wait_servo_channel_value(8, 1659, timeout=10)
        self.delay_sim_time(20)
        self.set_rc(3, 2000)
        self.wait_altitude(start_alt - 1,
                           (start_alt + 5),
                           relative=True,
                           timeout=timeout)
        self.context_collect('STATUSTEXT')

        # Reset collective to enter hover
        self.set_rc(3, 1500)

        # Change to the autorotation flight mode
        self.progress("Triggering autorotate mode")
        self.change_mode('AUTOROTATE')

        # Disengage the interlock to remove power
        self.set_rc(8, 1000)

        # Ensure we have progressed through the mode's state machine
        self.wait_statustext("Glide Phase", check_context=True)

        self.progress("Testing bailout from autorotation")
        self.set_rc(8, 2000)
        # See if the output ramps to a value close to expected with the prescribed time
        self.wait_servo_channel_value(8, 1659, timeout=bail_out_time+1, comparator=operator.ge)

        # Successfully bailed out, disengage the interlock and allow autorotation to progress
        self.set_rc(8, 1000)
        self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                             check_context=True,
                             regex=True)
        speed = float(self.re_match.group(1))
        if speed > 30:
            raise NotAchievedException("Hit too hard")

        # Set throttle low to trip auto disarm
        self.set_rc(3, 1000)

        self.wait_disarmed()
        self.context_pop()

    def AutorotationPreArm(self):
        """Check autorotation pre-arms are working"""
        self.context_push()
        self.start_subtest("Check pass when autorotation mode not enabled")
        self.set_parameters({
            "AROT_ENABLE": 0,
            "RPM1_TYPE": 0
        })
        self.reboot_sitl()
        try:
            self.wait_statustext("PreArm: AROT: RPM1 not enabled", timeout=50)
            raise NotAchievedException("Received AROT prearm when not AROT not enabled")
        except AutoTestTimeoutException:
            # We want to hit the timeout on wait_statustext()
            pass

        self.start_subtest("Check pre-arm fails when autorotation mode enabled")
        self.set_parameter("AROT_ENABLE", 1)
        self.wait_statustext("PreArm: AROT: RPM1 not enabled", timeout=50)
        self.set_parameter("RPM1_TYPE", 10) # reboot required to take effect
        self.reboot_sitl()

        self.start_subtest("Check pre-arm fails with bad HS_Sensor config")
        self.context_push()
        self.set_parameter("AROT_HS_SENSOR", -1)
        self.wait_statustext("PreArm: AROT: RPM instance <0", timeout=50)
        self.context_pop()

        self.start_subtest("Check pre-arm fails with bad RSC config")
        self.wait_statustext("PreArm: AROT: H_RSC_AROT_* not configured", timeout=50)

        self.start_subtest("Check pre-arms clear with all issues corrected")
        self.set_parameter("H_RSC_AROT_ENBL", 1)
        self.wait_ready_to_arm()

        self.context_pop()

    def ManAutorotation(self, timeout=600):
        """Check autorotation power recovery behaviour"""
        RSC_CHAN = 8

        def check_rsc_output(self, throttle, timeout):
            # Check we get a sensible throttle output
            expected_pwm = int(throttle * 0.01 * 1000 + 1000)

            # Help out the detection by accepting some margin
            margin = 2

            # See if the output ramps to a value close to expected with the prescribed time
            self.wait_servo_channel_in_range(RSC_CHAN, expected_pwm-margin, expected_pwm+margin, timeout=timeout)

        def TestAutorotationConfig(self, rsc_idle, arot_ramp_time, arot_idle, cool_down):
            RAMP_TIME = 10
            RUNUP_TIME = 15
            AROT_RUNUP_TIME = arot_ramp_time + 4
            RSC_SETPOINT = 66
            self.set_parameters({
                "H_RSC_AROT_ENBL": 1,
                "H_RSC_AROT_RAMP": arot_ramp_time,
                "H_RSC_AROT_RUNUP": AROT_RUNUP_TIME,
                "H_RSC_AROT_IDLE": arot_idle,
                "H_RSC_RAMP_TIME": RAMP_TIME,
                "H_RSC_RUNUP_TIME": RUNUP_TIME,
                "H_RSC_IDLE": rsc_idle,
                "H_RSC_SETPOINT": RSC_SETPOINT,
                "H_RSC_CLDWN_TIME": cool_down
            })

            # Check the RSC config so we know what to expect on the throttle output
            if self.get_parameter("H_RSC_MODE") != 2:
                self.set_parameter("H_RSC_MODE", 2)
                self.reboot_sitl()

            self.change_mode('POSHOLD')
            self.set_rc(3, 1000)
            self.set_rc(8, 1000)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_rc(8, 2000)
            self.progress("wait for rotor runup to complete")
            check_rsc_output(self, RSC_SETPOINT, RUNUP_TIME+1)

            self.delay_sim_time(20)
            self.set_rc(3, 2000)
            self.wait_altitude(100,
                               105,
                               relative=True,
                               timeout=timeout)
            self.context_collect('STATUSTEXT')
            self.change_mode('STABILIZE')

            self.progress("Triggering manual autorotation by disabling interlock")
            self.set_rc(3, 1000)
            self.set_rc(8, 1000)

            self.wait_statustext(r"RSC: In Autorotation", check_context=True)

            # Check we are using the correct throttle output. This should happen instantly on ramp down.
            idle_thr = rsc_idle
            if (arot_idle > 0):
                idle_thr = arot_idle

            check_rsc_output(self, idle_thr, 1)

            self.progress("RSC is outputting correct idle throttle")

            # Wait to establish autorotation.
            self.delay_sim_time(2)

            # Re-engage interlock to start bailout sequence
            self.set_rc(8, 2000)

            # Ensure we see the bailout state
            self.wait_statustext("RSC: Bailing Out", check_context=True)

            # Check we are back up to flight throttle. Autorotation ramp up time should be used
            check_rsc_output(self, RSC_SETPOINT, arot_ramp_time+1)

            # Give time for engine to power up
            self.set_rc(3, 1400)
            self.delay_sim_time(2)

            self.progress("in-flight power recovery")
            self.set_rc(3, 1500)
            self.delay_sim_time(5)

            # Initiate autorotation again
            self.set_rc(3, 1000)
            self.set_rc(8, 1000)

            self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                                 check_context=True,
                                 regex=True)
            speed = float(self.re_match.group(1))
            if speed > 30:
                raise NotAchievedException("Hit too hard")

            # Check that cool down is still used correctly if set
            # First wait until we are out of the autorotation state
            self.wait_statustext("RSC: Autorotation Stopped")
            if (cool_down > 0):
                check_rsc_output(self, rsc_idle*1.5, cool_down)

            # Verify RSC output resets to RSC_IDLE after land complete
            check_rsc_output(self, rsc_idle, 20)
            self.wait_disarmed()

        # We test the bailout behavior of two different configs
        # First we test config with a regular throttle curve
        self.start_subtest("testing autorotation with throttle curve config")
        self.context_push()
        TestAutorotationConfig(self, rsc_idle=5.0, arot_ramp_time=2.0, arot_idle=0, cool_down=0)

        # Now we test a config that would be used with an ESC with internal governor and an autorotation window
        self.start_subtest("testing autorotation with ESC autorotation window config")
        TestAutorotationConfig(self, rsc_idle=0.0, arot_ramp_time=0.0, arot_idle=20.0, cool_down=0)

        # Check rsc output behavior when using the cool down feature
        self.start_subtest("testing autorotation with cool down enabled and zero autorotation idle")
        TestAutorotationConfig(self, rsc_idle=5.0, arot_ramp_time=2.0, arot_idle=0, cool_down=5.0)

        self.start_subtest("testing that H_RSC_AROT_IDLE is used over RSC_IDLE when cool down is enabled")
        TestAutorotationConfig(self, rsc_idle=5.0, arot_ramp_time=2.0, arot_idle=10, cool_down=5.0)

        self.context_pop()

    def mission_item_home(self, target_system, target_component):
        '''returns a mission_item_int which can be used as home in a mission'''
        return self.mav.mav.mission_item_int_encode(
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
            int(2.0000 * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_item_takeoff(self, target_system, target_component):
        '''returns a mission_item_int which can be used as takeoff in a mission'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            1, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            int(1.0000 * 1e7), # latitude
            int(1.0000 * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_item_rtl(self, target_system, target_component):
        '''returns a mission_item_int which can be used as takeoff in a mission'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            1, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def scurve_nasty_mission(self, target_system=1, target_component=1):
        '''returns a mission which attempts to give the SCurve library
        indigestion.  The same destination is given several times.'''

        wp2_loc = self.mav.location()
        wp2_offset_n = 20
        wp2_offset_e = 30
        self.location_offset_ne(wp2_loc, wp2_offset_n, wp2_offset_e)

        wp2_by_three = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            2, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp2_loc.lat * 1e7), # latitude
            int(wp2_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        wp5_loc = self.mav.location()
        wp5_offset_n = -20
        wp5_offset_e = 30
        self.location_offset_ne(wp5_loc, wp5_offset_n, wp5_offset_e)

        wp5_by_three = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            5, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp5_loc.lat * 1e7), # latitude
            int(wp5_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        ret = copy.copy([
            # slot 0 is home
            self.mission_item_home(target_system=target_system, target_component=target_component),
            # slot 1 is takeoff
            self.mission_item_takeoff(target_system=target_system, target_component=target_component),
            # now three spline waypoints right on top of one another:
            copy.copy(wp2_by_three),
            copy.copy(wp2_by_three),
            copy.copy(wp2_by_three),
            # now three MORE spline waypoints right on top of one another somewhere else:
            copy.copy(wp5_by_three),
            copy.copy(wp5_by_three),
            copy.copy(wp5_by_three),
            self.mission_item_rtl(target_system=target_system, target_component=target_component),
        ])
        self.correct_wp_seq_numbers(ret)
        return ret

    def scurve_nasty_up_mission(self, target_system=1, target_component=1):
        '''returns a mission which attempts to give the SCurve library
        indigestion.  The same destination is given several times but with differing altitudes.'''

        wp2_loc = self.mav.location()
        wp2_offset_n = 20
        wp2_offset_e = 30
        self.location_offset_ne(wp2_loc, wp2_offset_n, wp2_offset_e)

        wp2 = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            2, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp2_loc.lat * 1e7), # latitude
            int(wp2_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        wp3 = copy.copy(wp2)
        wp3.alt = 40
        wp4 = copy.copy(wp2)
        wp4.alt = 31

        wp5_loc = self.mav.location()
        wp5_offset_n = -20
        wp5_offset_e = 30
        self.location_offset_ne(wp5_loc, wp5_offset_n, wp5_offset_e)

        wp5 = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            5, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp5_loc.lat * 1e7), # latitude
            int(wp5_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        wp6 = copy.copy(wp5)
        wp6.alt = 41
        wp7 = copy.copy(wp5)
        wp7.alt = 51

        ret = copy.copy([
            # slot 0 is home
            self.mission_item_home(target_system=target_system, target_component=target_component),
            # slot 1 is takeoff
            self.mission_item_takeoff(target_system=target_system, target_component=target_component),
            wp2,
            wp3,
            wp4,
            # now three MORE spline waypoints right on top of one another somewhere else:
            wp5,
            wp6,
            wp7,
            self.mission_item_rtl(target_system=target_system, target_component=target_component),
        ])
        self.correct_wp_seq_numbers(ret)
        return ret

    def fly_mission_points(self, points):
        '''takes a list of waypoints and flies them, expecting a disarm at end'''
        self.check_mission_upload_download(points)
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.set_rc(8, 1000)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.wait_waypoint(0, len(points)-1)
        self.wait_disarmed()
        self.set_rc(8, 1000)

    def NastyMission(self):
        '''constructs and runs missions designed to test scurves'''
        self.fly_mission_points(self.scurve_nasty_mission())
        # hopefully we don't need this step forever:
        self.progress("Restting mission state machine by changing into LOITER")
        self.change_mode('LOITER')
        self.fly_mission_points(self.scurve_nasty_up_mission())

    def MountFailsafeAction(self):
        """Fly Mount Failsafe action"""
        self.context_push()

        self.progress("Setting up servo mount")
        roll_servo = 12
        pitch_servo = 11
        yaw_servo = 10
        open_servo = 9
        roll_limit = 50
        self.set_parameters({
            "MNT1_TYPE": 1,
            "SERVO%u_MIN" % roll_servo: 1000,
            "SERVO%u_MAX" % roll_servo: 2000,
            "SERVO%u_FUNCTION" % yaw_servo: 6,  # yaw
            "SERVO%u_FUNCTION" % pitch_servo: 7,  # roll
            "SERVO%u_FUNCTION" % roll_servo: 8,  # pitch
            "SERVO%u_FUNCTION" % open_servo: 9,  # mount open
            "MNT1_OPTIONS": 2,  # retract
            "MNT1_DEFLT_MODE": 3,  # RC targeting
            "MNT1_ROLL_MIN": -roll_limit,
            "MNT1_ROLL_MAX": roll_limit,
        })

        self.reboot_sitl()

        retract_roll = 25.0
        self.set_parameter("MNT1_NEUTRAL_X", retract_roll)
        self.progress("Killing RC")
        self.set_parameter("SIM_RC_FAIL", 2)
        self.delay_sim_time(10)
        want_servo_channel_value = int(1500 + 500*retract_roll/roll_limit)
        self.wait_servo_channel_value(roll_servo, want_servo_channel_value, epsilon=1)

        self.progress("Resurrecting RC")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.wait_servo_channel_value(roll_servo, 1500)

        self.context_pop()

        self.reboot_sitl()

    def set_rc_default(self):
        super(AutoTestHelicopter, self).set_rc_default()
        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

    def fly_mission(self, filename, strict=True):
        num_wp = self.load_mission(filename, strict=strict)
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)    # Raise rotor speed
        self.delay_sim_time(20)
        self.change_mode("AUTO")
        self.set_rc(3, 1500)

        self.wait_waypoint(1, num_wp-1)
        self.wait_disarmed()
        self.set_rc(8, 1000)    # Lower rotor speed

    # FIXME move this & plane's version to common
    def AirspeedDrivers(self, timeout=600):
        '''Test AirSpeed drivers'''

        # Copter's airspeed sensors are off by default
        self.set_parameters({
            "ARSPD_ENABLE": 1,
            "ARSPD_TYPE": 2,     # Analog airspeed driver
            "ARSPD_PIN": 1,      # Analog airspeed driver pin for SITL
        })
        # set the start location to CMAC to use same test script as other vehicles

        self.sitl_start_loc = mavutil.location(-35.362881, 149.165222, 582.000000, 90.0)   # CMAC
        self.customise_SITL_commandline(["--home", "%s,%s,%s,%s"
                                         % (-35.362881, 149.165222, 582.000000, 90.0)])

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
            if delta > 3:
                raise NotAchievedException("Airspeed mismatch (as1=%f as2=%f)" % (airspeed[0], airspeed[1]))

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

            self.install_message_hook_context(check_airspeeds)
            self.fly_mission("ap1.txt", strict=False)

            if airspeed[0] is None:
                raise NotAchievedException("Never saw an airspeed1")
            if airspeed[1] is None:
                raise NotAchievedException("Never saw an airspeed2")
            if not self.current_onboard_log_contains_message("ARSP"):
                raise NotAchievedException("Expected ARSP log message")
            self.disarm_vehicle()
            self.context_pop()

    def TurbineCoolDown(self, timeout=200):
        """Check Turbine Cool Down Feature"""
        self.context_push()
        # set option for Turbine
        RAMP_TIME = 4
        SETPOINT = 66
        IDLE = 15
        COOLDOWN_TIME = 5
        self.set_parameters({"RC6_OPTION": 161,
                             "H_RSC_RAMP_TIME": RAMP_TIME,
                             "H_RSC_SETPOINT": SETPOINT,
                             "H_RSC_IDLE": IDLE,
                             "H_RSC_CLDWN_TIME": COOLDOWN_TIME})
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)

        self.progress("Starting turbine")
        self.wait_ready_to_arm()
        self.context_collect("STATUSTEXT")
        self.arm_vehicle()

        self.set_rc(6, 2000)
        self.wait_statustext('Turbine startup', check_context=True)

        # Engage interlock to run up to head speed
        self.set_rc(8, 2000)

        # Check throttle gets to setpoint
        expected_thr = SETPOINT * 0.01 * 1000 + 1000 - 1 # servo end points are 1000 to 2000
        self.wait_servo_channel_value(8, expected_thr, timeout=RAMP_TIME+1, comparator=operator.ge)

        self.progress("Checking cool down behaviour, idle x 1.5")
        self.set_rc(8, 1000)
        tstart = self.get_sim_time()
        expected_thr = IDLE * 1.5 * 0.01 * 1000 + 1000 + 1
        self.wait_servo_channel_value(8, expected_thr, timeout=2, comparator=operator.le)

        # Check that the throttle drops to idle after cool down time
        expected_thr = IDLE * 0.01 * 1000 + 1000 + 1
        self.wait_servo_channel_value(8, expected_thr, timeout=COOLDOWN_TIME+1, comparator=operator.le)

        measured_time = self.get_sim_time() - tstart
        if (abs(measured_time - COOLDOWN_TIME) > 1.0):
            raise NotAchievedException('Throttle did not reduce to idle within H_RSC_CLDWN_TIME')

        self.set_rc(6, 1000)
        self.wait_disarmed(timeout=20)
        self.context_pop()

    def TurbineStart(self, timeout=200):
        """Check Turbine Start Feature"""
        RAMP_TIME = 4
        # set option for Turbine Start
        self.set_parameter("RC6_OPTION", 161)
        self.set_parameter("H_RSC_RAMP_TIME", RAMP_TIME)
        self.set_parameter("H_RSC_SETPOINT", 66)
        self.set_parameter("DISARM_DELAY", 0)
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)

        # check that turbine start doesn't activate while disarmed
        self.progress("Checking Turbine Start doesn't activate while disarmed")
        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 2:
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1050:
                raise NotAchievedException("Turbine Start activated while disarmed")
        self.set_rc(6, 1000)

        # check that turbine start doesn't activate armed with interlock enabled
        self.progress("Checking Turbine Start doesn't activate while armed with interlock enabled")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)
        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 5:
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1660:
                raise NotAchievedException("Turbine Start activated with interlock enabled")

        self.set_rc(8, 1000)
        self.set_rc(6, 1000)
        self.disarm_vehicle()

        # check that turbine start activates as designed (armed with interlock disabled)
        self.progress("Checking Turbine Start activates as designed (armed with interlock disabled)")
        self.delay_sim_time(2)
        self.arm_vehicle()

        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise AutoTestTimeoutException("Turbine Start did not activate")
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1800:
                break

        self.wait_servo_channel_value(8, 1000, timeout=3)
        self.set_rc(6, 1000)

        # check that turbine start will not reactivate after interlock enabled
        self.progress("Checking Turbine Start doesn't activate once interlock is enabled after start)")
        self.set_rc(8, 2000)
        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 5:
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1660:
                raise NotAchievedException("Turbine Start activated with interlock enabled")
        self.set_rc(6, 1000)
        self.set_rc(8, 1000)
        self.disarm_vehicle()

    def PIDNotches(self):
        """Use dynamic harmonic notch to control motor noise."""
        self.progress("Flying with PID notches")
        self.set_parameters({
            "FILT1_TYPE": 1,
            "FILT2_TYPE": 1,
            "AHRS_EKF_TYPE": 10,
            "INS_LOG_BAT_MASK": 3,
            "INS_LOG_BAT_OPT": 0,
            "INS_GYRO_FILTER": 100, # set the gyro filter high so we can observe behaviour
            "LOG_BITMASK": 65535,
            "LOG_DISARMED": 0,
            "SIM_VIB_FREQ_X": 120,  # roll
            "SIM_VIB_FREQ_Y": 120,  # pitch
            "SIM_VIB_FREQ_Z": 180,  # yaw
            "FILT1_NOTCH_FREQ": 120,
            "FILT2_NOTCH_FREQ": 180,
            "ATC_RAT_RLL_NEF": 1,
            "ATC_RAT_PIT_NEF": 1,
            "ATC_RAT_YAW_NEF": 2,
            "SIM_GYR1_RND": 5,
        })
        self.reboot_sitl()

        self.hover_and_check_matched_frequency_with_fft(5, 20, 350, reverse=True, takeoff=True)

    def AutoTune(self):
        """Test autotune mode"""
        # test roll and pitch FF tuning
        self.set_parameters({
            "ATC_ANG_RLL_P": 4.5,
            "ATC_RAT_RLL_P": 0,
            "ATC_RAT_RLL_I": 0.1,
            "ATC_RAT_RLL_D": 0,
            "ATC_RAT_RLL_FF": 0.15,
            "ATC_ANG_PIT_P": 4.5,
            "ATC_RAT_PIT_P": 0,
            "ATC_RAT_PIT_I": 0.1,
            "ATC_RAT_PIT_D": 0,
            "ATC_RAT_PIT_FF": 0.15,
            "ATC_ANG_YAW_P": 4.5,
            "ATC_RAT_YAW_P": 0.18,
            "ATC_RAT_YAW_I": 0.024,
            "ATC_RAT_YAW_D": 0.003,
            "ATC_RAT_YAW_FF": 0.024,
            "AUTOTUNE_AXES": 3,
            "AUTOTUNE_SEQ": 1,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test pitch rate P and Rate D tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 2,
            "AUTOTUNE_SEQ": 2,
            "AUTOTUNE_GN_MAX": 1.8,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test Roll rate P and Rate D tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 1,
            "AUTOTUNE_SEQ": 2,
            "AUTOTUNE_GN_MAX": 1.6,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test Roll and pitch angle P tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 3,
            "AUTOTUNE_SEQ": 4,
            "AUTOTUNE_FRQ_MIN": 5,
            "AUTOTUNE_FRQ_MAX": 50,
            "AUTOTUNE_GN_MAX": 1.6,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test yaw FF and rate P and Rate D
        self.set_parameters({
            "AUTOTUNE_AXES": 4,
            "AUTOTUNE_SEQ": 3,
            "AUTOTUNE_FRQ_MIN": 10,
            "AUTOTUNE_FRQ_MAX": 70,
            "AUTOTUNE_GN_MAX": 1.4,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test yaw angle P tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 4,
            "AUTOTUNE_SEQ": 4,
            "AUTOTUNE_FRQ_MIN": 5,
            "AUTOTUNE_FRQ_MAX": 50,
            "AUTOTUNE_GN_MAX": 1.5,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # tune check
        self.set_parameters({
            "AUTOTUNE_AXES": 7,
            "AUTOTUNE_SEQ": 16,
            "AUTOTUNE_FRQ_MIN": 10,
            "AUTOTUNE_FRQ_MAX": 80,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.land_and_disarm()

    def autotune_land_and_save_gains(self):
        self.set_rc(3, 1000)
        self.context_collect('STATUSTEXT')
        self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                             check_context=True,
                             regex=True)
        self.set_rc(8, 1000)
        self.wait_disarmed()

    def land_and_disarm(self, **kwargs):
        super(AutoTestHelicopter, self).land_and_disarm(**kwargs)
        self.progress("Killing rotor speed")
        self.set_rc(8, 1000)

    def do_RTL(self, **kwargs):
        super(AutoTestHelicopter, self).do_RTL(**kwargs)
        self.progress("Killing rotor speed")
        self.set_rc(8, 1000)

    def tests(self):
        '''return list of all tests'''
        ret = vehicle_test_suite.TestSuite.tests(self)
        ret.extend([
            self.AVCMission,
            self.RotorRunup,
            self.PosHoldTakeOff,
            self.StabilizeTakeOff,
            self.SplineWaypoint,
            self.AutorotationPreArm,
            self.Autorotation,
            self.ManAutorotation,
            self.governortest,
            self.FlyEachFrame,
            self.AirspeedDrivers,
            self.TurbineStart,
            self.TurbineCoolDown,
            self.NastyMission,
            self.PIDNotches,
            self.AutoTune,
            self.MountFailsafeAction,
        ])
        return ret

    def disabled_tests(self):
        return {
        }
