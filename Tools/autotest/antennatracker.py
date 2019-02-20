#!/usr/bin/env python

from __future__ import print_function
import os
import pexpect
from pymavlink import mavutil

from common import AutoTest
from pysim import util
from pysim import vehicleinfo
import operator

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
HOME = mavutil.location(-27.274439, 151.290064, 343, 8.7)

class AutoTestTracker(AutoTest):
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
        super(AutoTestTracker, self).__init__(**kwargs)
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

        self.log_name = "AntennaTracker"
        self.logfile = None

        self.sitl = None

    def start_stream_systemtime(self):
        '''AntennaTracker doesn't stream this by default but we need it for get_sim_time'''
        try:
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 10)
        except Exception:
            pass
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 10)

    def set_rc_default(self):
        '''tracker does not send RC_CHANNELS, so can't set_rc_default'''
        '''... however, dodgily hook in here to get system time:'''
        self.start_stream_systemtime()

    def initialise_after_reboot_sitl(self):
        self.start_stream_systemtime()

    def default_mode(self):
        return "AUTO"

    def is_tracker(self):
        return True

    def init(self):
        if self.frame is None:
            self.frame = "tracker"

        self.mavproxy_logfile = self.open_mavproxy_logfile()

        self.sitl = util.start_SITL(self.binary,
                                    wipe=True,
                                    model=self.frame,
                                    home=self.home,
                                    speedup=self.speedup,
                                    valgrind=self.valgrind,
                                    gdb=self.gdb,
                                    gdbserver=self.gdbserver,
                                    breakpoints=self.breakpoints,
                                    )
        self.mavproxy = util.start_MAVProxy_SITL(
            'AntennaTracker', options=self.mavproxy_options())
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

        self.progress("Ready to start testing!")

    def sysid_thismav(self):
        return 2

    def reboot_sitl(self):
        """Reboot SITL instance and wait it to reconnect."""
        self.mavproxy.send("reboot\n")
        self.mavproxy.expect("Initialising APM")
        # empty mav to avoid getting old timestamps:
        while self.mav.recv_match(blocking=False):
            pass
        self.initialise_after_reboot_sitl()

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestTracker, self).tests()
        ret.extend([
        ])
        return ret
