#!/usr/bin/env python

# Drive APMrover2 in SITL
from __future__ import print_function
import os
import shutil
import pexpect

from common import *
from pysim import util
from pysim import vehicleinfo
from pymavlink import mavutil, mavwp

'''
Class implementing methods to test ArduRover
'''
class drive_apmrover2(vehicle_tester):
    def __init__(self,
                 binary,
                 viewerip=None,
                 use_map=False,
                 valgrind=False,
                 gdb=False,
                 frame='rover',
                 params=None,
                 gdbserver=False,
                 speedup=10):

        self.binary = binary
        self.viewerip = viewerip
        self.use_map = use_map
        self.valgrind = valgrind
        self.gdb = gdb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver
        self.speedup = speedup

        if self.frame is None:
            # FIXME: should not be needed
            self.frame = 'rover'

        # get location of scripts
        self.testdir = os.path.dirname(os.path.realpath(__file__))

        self.home = mavutil.location(40.071374969556928,
                                     -105.22978898137808,
                                     1583.702759,
                                     246)
        self.homeloc = None
        self.num_wp = 0
        self.speedup_default = 10

    ##########################################################
    #   TESTS DRIVE
    ##########################################################
    def drive_left_circuit(self):
        """Drive a left circuit, 50m on a side."""
        self.mavproxy.send('switch 6\n')
        self.wait_mode('MANUAL')
        self.set_rc(3, 2000)

        self.progress("Driving left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(1, 1000)
            if not self.wait_heading(270 - (90*i), accuracy=10):
                return False
            self.set_rc(1, 1500)
            self.progress("Starting leg %u" % i)
            if not self.wait_distance(50, accuracy=7):
                return False
        self.set_rc(3, 1500)
        self.progress("Circuit complete")
        return True


    def drive_RTL(self):
        """Drive to home."""
        self.progress("Driving home in RTL")
        self.mavproxy.send('switch 3\n')
        if not self.wait_location(self.homeloc, accuracy=22, timeout=90):
            return False
        self.progress("RTL Complete")
        return True


    #################################################
    # AUTOTEST ALL
    #################################################
    def drive_mission(self, filename):
        """Drive a mission from a file."""
        self.progress("Driving mission %s" % filename)
        self.mavproxy.send('wp load %s\n' % filename)
        self.mavproxy.expect('Flight plan received')
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('switch 4\n')  # auto mode
        self.set_rc(3, 1500)
        self.wait_mode('AUTO')
        if not self.wait_waypoint(1, 4, max_dist=5):
            return False
        self.wait_mode('HOLD')
        self.progress("Mission OK")
        return True


    def do_get_banner(self):
        self.mavproxy.send("long DO_SEND_BANNER 1\n")
        start = time.time()
        while True:
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None and "APM:Rover" in m.text:
                self.progress("banner received: %s" % (m.text))
                return True
            if time.time() - start > 10:
                break

        self.progress("banner not received")

        return False


    def drive_brake_get_stopping_distance(self, speed):
        # measure our stopping distance:
        old_cruise_speed = self.get_parameter('CRUISE_SPEED')
        old_accel_max = self.get_parameter('ATC_ACCEL_MAX')

        # controller tends not to meet cruise speed (max of ~14 when 15
        # set), thus *1.2
        self.set_parameter('CRUISE_SPEED', speed*1.2)
        # at time of writing, the vehicle is only capable of 10m/s/s accel
        self.set_parameter('ATC_ACCEL_MAX', 15)
        self.mavproxy.send("mode STEERING\n")
        self.wait_mode('STEERING')
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

        self.set_parameter('CRUISE_SPEED', old_cruise_speed)
        self.set_parameter('ATC_ACCEL_MAX', old_accel_max)

        return delta


    def drive_brake(self):
        old_using_brake = self.get_parameter('ATC_BRAKE')
        old_cruise_speed = self.get_parameter('CRUISE_SPEED')

        self.set_parameter('CRUISE_SPEED', 15)
        self.set_parameter('ATC_BRAKE', 0)

        distance_without_brakes = self.drive_brake_get_stopping_distance(15)

        # brakes on:
        self.set_parameter('ATC_BRAKE', 1)
        distance_with_brakes = self.drive_brake_get_stopping_distance(15)
        # revert state:
        self.set_parameter('ATC_BRAKE', old_using_brake)
        self.set_parameter('CRUISE_SPEED', old_cruise_speed)

        delta = distance_without_brakes - distance_with_brakes
        if delta < distance_without_brakes * 0.05:  # 5% isn't asking for much
            self.progress("Brakes have negligible effect (with=%0.2fm without=%0.2fm delta=%0.2fm)" % (distance_with_brakes, distance_without_brakes, delta))
            return False
        else:
            self.progress("Brakes work (with=%0.2fm without=%0.2fm delta=%0.2fm)" % (distance_with_brakes, distance_without_brakes, delta))

        return True


    def run(self):
        """Drive APMrover2 in SITL.

        you can pass viewerip as an IP address to optionally send fg and
        mavproxy packets too for local viewing of the mission in real time
        """

        options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
        if self.viewerip:
            options += " --out=%s:14550" % self.viewerip
        if self.use_map:
            options += ' --map'

        home = "%f,%f,%u,%u" % (self.home.lat,
                                self.home.lng,
                                self.home.alt,
                                self.home.heading)
        sitl = util.start_SITL(self.binary,
                               wipe=True,
                               model=self.frame,
                               home=home,
                               speedup=self.speedup)
        self.mavproxy = util.start_MAVProxy_SITL('APMrover2')

        self.progress("WAITING FOR PARAMETERS")
        self.mavproxy.expect('Received [0-9]+ parameters')

        # setup test parameters
        if self.params is None:
            vinfo = vehicleinfo.VehicleInfo()
            self.params = vinfo.options["APMrover2"]["frames"][self.frame]["default_params_filename"]
        if not isinstance(self.params, list):
            self.params = [self.params]
        for x in self.params:
            self.mavproxy.send("param load %s\n" % os.path.join(self.testdir, x))
            self.mavproxy.expect('Loaded [0-9]+ parameters')
        self.set_parameter('LOG_REPLAY', 1)
        self.set_parameter('LOG_DISARMED', 1)

        # restart with new parms
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(sitl)

        sitl = util.start_SITL(self.binary,
                               model='rover',
                               home=home,
                               speedup=self.speedup,
                               valgrind=self.valgrind,
                               gdb=self.gdb,
                               gdbserver=self.gdbserver)
        self.mavproxy = util.start_MAVProxy_SITL('APMrover2', options=options)
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % logfile)

        buildlog = util.reltopdir("../buildlogs/APMrover2-test.tlog")
        self.progress("buildlog=%s" % buildlog)
        if os.path.exists(buildlog):
            os.unlink(buildlog)
        try:
            os.link(logfile, buildlog)
        except Exception:
            pass

        self.mavproxy.expect('Received [0-9]+ parameters')

        util.expect_setup_callback(self.mavproxy, expect_callback)

        expect_list_clear()
        expect_list_extend([sitl, self.mavproxy])

        self.progress("Started simulator")

        # get a mavlink connection going
        try:
            self.mav = mavutil.mavlink_connection('127.0.0.1:19550',
                                                  robust_parsing=True)
        except Exception as msg:
            self.progress("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
            raise
        self.mav.message_hooks.append(message_hook)
        self.mav.idle_hooks.append(idle_hook)

        failed = False
        e = 'None'
        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s" %
                          self.mav.WIRE_PROTOCOL_VERSION)
            self.mav.wait_heartbeat()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.set_rc(8, 1800)
            self.progress("Waiting for GPS fix")
            self.mav.wait_gps_fix()
            self.homeloc = self.mav.location()
            self.progress("Home location: %s" % self.homeloc)
            self.mavproxy.send('switch 6\n')  # Manual mode
            self.wait_mode('MANUAL')
            self.progress("Waiting reading for arm")
            self.wait_ready_to_arm()
            if not self.arm_vehicle():
                self.progress("Failed to ARM")
                failed = True
            self.progress("#")
            self.progress("########## Drive a square and save WPs with CH7 switch  ##########")
            self.progress("#")
            # Drive a square in learning mode
            if not self.drive_mission(os.path.join(self.testdir, "rover1.txt")):
                self.progress("Failed mission")
                failed = True

            if not self.drive_brake():
                self.progress("Failed brake")
                failed = True

            if not self.disarm_vehicle():
                self.progress("Failed to DISARM")
                failed = True

            # do not move this to be the first test.  MAVProxy's dedupe
            # function may bite you.
            self.progress("Getting banner")
            if not self.do_get_banner():
                self.progress("FAILED: get banner")
                failed = True

            self.progress("Getting autopilot capabilities")
            if not self.do_get_autopilot_capabilities():
                self.progress("FAILED: get capabilities")
                failed = True

            self.progress("Setting mode via MAV_COMMAND_DO_SET_MODE")
            if not self.do_set_mode_via_command_long():
                failed = True

            if not self.log_download(util.reltopdir("../buildlogs/APMrover2-log.bin")):
                self.progress("Failed log download")
                failed = True
    #        if not drive_left_circuit(mavproxy, mav):
    #            self.progress("Failed left circuit")
    #            failed = True
    #        if not drive_RTL(mavproxy, mav):
    #            self.progress("Failed RTL")
    #            failed = True

        except pexpect.TIMEOUT as e:
            self.progress("Failed with timeout")
            failed = True

        self.mav.close()
        util.pexpect_close(self.mavproxy)
        util.pexpect_close(sitl)

        valgrind_log = util.valgrind_log_filepath(binary=self.binary,
                                                  model='rover')
        if os.path.exists(valgrind_log):
            os.chmod(valgrind_log, 0o644)
            shutil.copy(valgrind_log,
                        util.reltopdir("../buildlogs/APMrover2-valgrind.log"))

        if failed:
            self.progress("FAILED: %s" % e)
            return False
        return True
