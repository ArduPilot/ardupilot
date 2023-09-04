'''
Fly Helicopter in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
from arducopter import AutoTestCopter
from common import AutoTest
from common import NotAchievedException, AutoTestTimeoutException

from pymavlink import mavutil
from pysim import vehicleinfo

import copy


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
        servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
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
        servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        coll = servo.servo1_raw
        coll = coll + 50
        self.set_parameter("H_RSC_RUNUP_TIME", TARGET_RUNUP_TIME)
        self.progress("Initiate Runup by putting some throttle")
        self.set_rc(8, 2000)
        self.set_rc(3, 1700)
        self.progress("Collective threshold PWM %u" % coll)
        tstart = self.get_sim_time()
        self.progress("Wait that collective PWM pass threshold value")
        servo = self.mav.recv_match(condition='SERVO_OUTPUT_RAW.servo1_raw>%u' % coll, blocking=True)
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
        self.set_rc(8, 1000)
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
                ["--defaults", ','.join(defaults), ],
                model=model,
                wipe=True,
            )
            self.takeoff(10)
            self.do_RTL()
            self.set_rc(8, 1000)

    def governortest(self):
        '''Test Heli Internal Throttle Curve and Governor'''
        self.customise_SITL_commandline(
            ["--defaults", ','.join(self.model_defaults_filepath('heli-gas')), ],
            model="heli-gas",
            wipe=True,
        )
        self.set_parameter("H_RSC_MODE", 4)
        self.takeoff(10)
        self.do_RTL()
        self.set_rc(8, 1000)

    def hover(self):
        self.progress("Setting hover collective")
        self.set_rc(3, 1500)

    def PosHoldTakeOff(self):
        """ensure vehicle stays put until it is ready to fly"""
        self.context_push()

        ex = None
        try:
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
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            max_relalt_mm = 1000
            if abs(m.relative_alt) > max_relalt_mm:
                raise NotAchievedException("Took off prematurely (abs(%f)>%f)" %
                                           (m.relative_alt, max_relalt_mm))

            self.progress("Pushing collective past half-way")
            self.set_rc(3, 1600)
            self.delay_sim_time(0.5)
            self.hover()

            # make sure we haven't already reached alt:
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            max_initial_alt = 1500
            if abs(m.relative_alt) > max_initial_alt:
                raise NotAchievedException("Took off too fast (%f > %f" %
                                           (abs(m.relative_alt), max_initial_alt))

            self.progress("Monitoring takeoff-to-alt")
            self.wait_altitude(6.9, 8, relative=True)

            self.progress("Making sure we stop at our takeoff altitude")
            tstart = self.get_sim_time()
            while self.get_sim_time() - tstart < 5:
                m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                delta = abs(7000 - m.relative_alt)
                self.progress("alt=%f delta=%f" % (m.relative_alt/1000,
                                                   delta/1000))
                if delta > 1000:
                    raise NotAchievedException("Failed to maintain takeoff alt")
            self.progress("takeoff OK")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.land_and_disarm()
        self.set_rc(8, 1000)

        self.context_pop()

        if ex is not None:
            raise ex

    def StabilizeTakeOff(self):
        """Fly stabilize takeoff"""
        self.context_push()

        ex = None
        try:
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
            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if abs(m.relative_alt) > 100:
                raise NotAchievedException("Took off prematurely")
            self.progress("Pushing throttle past half-way")
            self.set_rc(3, 1650)

            self.progress("Monitoring takeoff")
            self.wait_altitude(6.9, 8, relative=True)

            self.progress("takeoff OK")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.land_and_disarm()
        self.set_rc(8, 1000)

        self.context_pop()

        if ex is not None:
            raise ex

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
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Vehicle did not disarm after mission")
            if not self.armed():
                break
            self.delay_sim_time(1)
        self.progress("Lowering rotor speed")
        self.set_rc(8, 1000)

    def AutoRotation(self, timeout=600):
        """Check engine-out behaviour"""
        self.set_parameter("AROT_ENABLE", 1)
        start_alt = 100 # metres
        self.set_parameter("PILOT_TKOFF_ALT", start_alt * 100)
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
        self.progress("Triggering autorotate by raising interlock")
        self.set_rc(8, 1000)
        self.wait_statustext("SS Glide Phase", check_context=True)
        self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                             check_context=True,
                             regex=True)
        speed = float(self.re_match.group(1))
        if speed > 30:
            raise NotAchievedException("Hit too hard")
        self.wait_disarmed()

    def ManAutoRotation(self, timeout=600):
        """Check autorotation power recovery behaviour"""
        RAMP_TIME = 4
        AROT_RAMP_TIME = 2
        self.set_parameter("H_RSC_AROT_MN_EN", 1)
        self.set_parameter("H_RSC_AROT_ENG_T", AROT_RAMP_TIME)
        self.set_parameter("H_RSC_AROT_IDLE", 20)
        self.set_parameter("H_RSC_RAMP_TIME", RAMP_TIME)
        self.set_parameter("H_RSC_IDLE", 0)
        start_alt = 100 # metres
        self.set_parameter("PILOT_TKOFF_ALT", start_alt * 100)
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
        self.change_mode('STABILIZE')
        self.progress("Triggering manual autorotation by disabling interlock")
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)
        self.wait_servo_channel_value(8, 1199, timeout=3)
        self.progress("channel 8 set to autorotation window")

        # wait to establish autorotation
        self.delay_sim_time(2)

        self.set_rc(8, 2000)
        self.wait_servo_channel_value(8, 1659, timeout=AROT_RAMP_TIME * 1.1)

        # give time for engine to power up
        self.set_rc(3, 1400)
        self.delay_sim_time(2)

        self.progress("in-flight power recovery")
        self.set_rc(3, 1500)
        self.delay_sim_time(5)

        # initiate autorotation again
        self.set_rc(3, 1000)
        self.set_rc(8, 1000)

        self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                             check_context=True,
                             regex=True)
        speed = float(self.re_match.group(1))
        if speed > 30:
            raise NotAchievedException("Hit too hard")

        self.set_rc(3, 1000)
        # verify servo 8 resets to RSC_IDLE after land complete
        self.wait_servo_channel_value(8, 1000, timeout=3)
        self.wait_disarmed()

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
        # renumber the items:
        count = 0
        for item in ret:
            item.seq = count
            count += 1
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
        # renumber the items:
        count = 0
        for item in ret:
            item.seq = count
            count += 1
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

        # Copter's airspeed sensors are off by default
        self.set_parameter("ARSPD_ENABLE", 1)
        self.set_parameter("ARSPD_TYPE", 2)     # Analog airspeed driver
        self.set_parameter("ARSPD_PIN", 1)      # Analog airspeed driver pin for SITL
        self.reboot_sitl()

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
            self.context_pop()
            if not self.current_onboard_log_contains_message("ARSP"):
                raise NotAchievedException("Expected ARSP log message")

        self.reboot_sitl()

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
            servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
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
            servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
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
            servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
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
            servo = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
            if servo.servo8_raw > 1660:
                raise NotAchievedException("Turbine Start activated with interlock enabled")
        self.set_rc(6, 1000)
        self.set_rc(8, 1000)
        self.disarm_vehicle()

    def tests(self):
        '''return list of all tests'''
        ret = AutoTest.tests(self)
        ret.extend([
            self.AVCMission,
            self.RotorRunup,
            self.PosHoldTakeOff,
            self.StabilizeTakeOff,
            self.SplineWaypoint,
            self.AutoRotation,
            self.ManAutoRotation,
            self.governortest,
            self.FlyEachFrame,
            self.AirspeedDrivers,
            self.TurbineStart,
            self.NastyMission,
        ])
        return ret

    def disabled_tests(self):
        return {
        }
