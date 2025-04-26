#!/usr/bin/env python

"""
Script designed to provide a simple console-based program to
command the Loweheiser generator via its mavlink interface

AP_FLAKE8_CLEAN

"""

from __future__ import print_function

import sys
import time
import optparse
import os
import select
import re

os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil  # NOQA

mavutil.set_dialect("all")

# Detect python version
if sys.version_info[0] < 3:
    runningPython3 = False
else:
    runningPython3 = True


class TestLoweheiser(object):
    def __init__(self,
                 master,
                 source_system=3,
                 source_component=4,
                 command_interval=0.1,  # in seconds
                 heartbeat_interval=0.2,  # in seconds
                 log_filepath=None,
                 log_raw_filepath=None):
        self.master = master
        self.conns = None
        self.source_system = source_system
        self.source_component = source_component
        self.target_system = None
        self.target_component = None
        self.command_interval = command_interval
        self.heartbeat_interval = heartbeat_interval
        self.log_raw_filepath = log_raw_filepath
        self.log_filepath = log_filepath

        self.last_command_sent = 0
        self.last_heartbeat_sent = 0

        self.ack_received_count = 0
        self.bad_ack_received_count = 0
        self.commands_sent_count = 0
        self.last_ack_received = 0

        self.desired_state = "off"
        self.desired_governor_state = "off"
        self.desired_startup_state = "off"
        self.desired_manual_throttle_level = 0

        self.bad_byte_count = 0

        self.last_heartbeat_time = 0

        self.last_status_out = 0

        self.LOWEHEISER_GOV_EFI = None

    def progress(self, text):
        '''emit text with possible timestamps etc'''
        print("%u: %s" % (time.time(), text))

    def dump_message_verbose(self, m):
        '''wrapper around verbose-message-dumper'''
        mavutil.dump_message_verbose(sys.stdout, m)

    def dump_messages(self):
        '''dump messages on connection to stdout'''
        m = self.conn.recv_match(blocking=True, timeout=0.1)
        if m is None:
            # self.progress(".")
            return

        # self.dump_message_verbose(m)
        # print("%s" % m.get_type())

        t = m.get_type()
        if t == 'HEARTBEAT':
            #            print("Received heartbeat from %u/%u" %
            #                  (m.get_srcSystem(), m.get_srcComponent()))
            self.last_heartbeat_time = time.time()
            return

        if t == 'LOWEHEISER_GOV_EFI':
            self.target_system = m.get_srcSystem()
            self.target_component = m.get_srcComponent()

            # self.dump_message_verbose(m)
            # self.progress("LOWEHEISER_GOV_EFI: rpm=%f" % m.efi_rpm)
            self.LOWEHEISER_GOV_EFI = m
            return

        if t == "COMMAND_ACK":
            # print("m: %s" % str(m))
            if (m.get_srcSystem() == self.target_system and
                    m.get_srcComponent() == self.target_component and
                    m.target_system == self.source_system and
                    m.target_component == self.source_component and
                    m.command == mavutil.mavlink.MAV_CMD_LOWEHEISER_SET_STATE):
                self.last_ack_received = time.time()
                self.ack_received_count += 1
            else:
                print("Bad command ack: %s" % str(m))

                self.bad_ack_received_count += 1

        if t == 'BAD_DATA':
            print("bad data: %s" % str(m))
            self.bad_byte_count += len(m.data)

    def die(self, message):
        self.progress(message)
        sys.exit(1)

    def send_commands(self):
        '''send any pending commands to the generator'''

        now = time.time()
        if now - self.last_command_sent < self.command_interval:
            return
        self.last_command_sent = now

        if self.target_system is None:
            self.progress("Have not seen LOWEHEISER_GOV_EFI yet")  # assumption
            return

        if self.desired_state == "off":
            commanded_engine_state = 0
        elif self.desired_state == "run":
            commanded_engine_state = 1
        else:
            self.die("Bad desired engine state (%s)" % self.desired_state)

        if self.desired_governor_state == "off":
            commanded_governor_state = 0
        elif self.desired_governor_state == "on":
            commanded_governor_state = 1
        else:
            self.die("Bad desired governor state")

        if self.desired_startup_state == "off":
            commanded_startup_state = 0
        elif self.desired_startup_state == "on":
            commanded_startup_state = 1
        else:
            self.die("Bad desired startup state %s" % self.desired_startup_state)

        commanded_manual_throttle_level = self.desired_manual_throttle_level

        # self.progress("Sending command")
        self.conn.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOWEHEISER_SET_STATE,
            1,  # confirmation
            1,  # param1  # EFI index
            commanded_engine_state,  # param2
            commanded_governor_state,  # param3
            commanded_manual_throttle_level,  # param4
            commanded_startup_state,  # param5
            0,  # param6
            0   # param7
        )
        self.commands_sent_count += 1

        if now - self.last_ack_received > 10:
            self.progress("We're missing acks.... commands=%u acks=%u bad_acks=%u"  %
                          (self.commands_sent_count,
                           self.ack_received_count,
                           self.bad_ack_received_count))

    def handle_stdin(self):
        '''read from stdin, handle commands'''
        # self.progress("Checking stdin")
        (rout, wout, eout) = select.select([sys.stdin], [], [], 0.0001)
        if not rout:
            # nothing on stdin
            return
        line = sys.stdin.readline()
        line = line.rstrip()

        if line == "off" or line == "run":
            self.desired_state = line
            return

        if line == "governor_on":
            self.desired_governor_state = "on"
            return
        if line == "governor_off":
            self.desired_governor_state = "off"
            return

        if line == "startup_on":
            self.desired_startup_state = "on"
            return
        if line == "startup_off":
            self.desired_startup_state = "off"
            return

        m = re.match(r"throttle (\d+)", line)
        if m is not None:
            self.desired_manual_throttle_level = float(m.group(1))
            return
        self.progress("Unrecognised command (%s)" % line)

        self.progress("Checked stdin")

    def send_heartbeats(self):
        '''send heartbeat to mavlink connection if appropriate'''
        now = time.time()
        if now - self.last_heartbeat_sent < self.heartbeat_interval:
            return
        self.last_heartbeat_sent = now

        # self.progress("Sending heatbeat")
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0,
            0,
            0
        )

    def emit_status(self):
        now = time.time()
        if now - self.last_status_out < 1:
            return
        self.last_status_out = now

        rpm = None
        if self.LOWEHEISER_GOV_EFI is not None:
            rpm = self.LOWEHEISER_GOV_EFI.efi_rpm
        heartbeat_age = time.time() - self.last_heartbeat_time
        self.progress("dg=%s dstartup=%s dt=%.02f ds=%s  m.rpm=%s hba=%f bb=%u" % (
            self.desired_governor_state,
            self.desired_startup_state,
            self.desired_manual_throttle_level,
            self.desired_state,
            rpm,
            heartbeat_age,
            self.bad_byte_count
        ))

    def run(self):

        self.progress("Creating connection")
        self.conn = mavutil.mavlink_connection(
            self.master,
            source_system=self.source_system,
            source_component=self.source_component,
        )

        if self.log_filepath is not None:
            print("here (%s)" % self.log_filepath)
            self.conn.setup_logfile(self.log_filepath)

        if self.log_raw_filepath is not None:
            self.conn.setup_logfile_raw(self.log_raw_filepath)

        self.progress("Entering read loop")
        while True:
            # read text commands from stdin
            self.handle_stdin()

            # consider heartbeating:
            self.send_heartbeats()

            # dump messages from connection:
            self.dump_messages()  # may set self.target_system

            # consider sending a command:
            self.send_commands()

            # print status out:
            self.emit_status()


if __name__ == '__main__':
    parser = optparse.OptionParser("test-loweheiser.py [options]")

    parser.add_option(
        "--source-system",
        default=3,
        help="script system ID"
    )
    parser.add_option(
        "--source-component",
        default=4,
        type='int',
        help="script component ID"
    )
    parser.add_option(
        "--target-compid",
        default=None,
        type='int',
        help="generator component ID"
    )
    parser.add_option(
        "--log-raw-filepath",
        default=None,
        help="Path to emit raw tlog to"
    )
    parser.add_option(
        "--log-filepath",
        default=None,
        help="Path to emit tlog to"
    )

    (opts, args) = parser.parse_args()

    def usage():
        parser.print_help()
        sys.exit(1)

    if len(args) < 1:
        usage()

    master = args[0]
    print("master=%s" % master)

    tester = TestLoweheiser(
        master,
        log_raw_filepath=opts.log_raw_filepath,
        log_filepath=opts.log_filepath,
        source_system=int(opts.source_system),
        source_component=int(opts.source_component),
    )
    tester.run()
