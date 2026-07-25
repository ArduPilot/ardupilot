#!/usr/bin/env python3

'''
Copyright Peter Barker 2021
'''

import optparse
import os
import pymavlink
import re
import select
import subprocess
import time
import fcntl

from pymavlink import mavutil

class SerialControlToShell(object):
    '''reads SERIAL_CONTROL packets and passes them to a shell, returning textual  results'''

    def __init__(self, connection_string, system_id=1, component_id=10):
        self.connection_string = connection_string

        self.serial_control_dev = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL

        self.mav = mavutil.mavlink_connection(
            self.connection_string,
            source_system=system_id,
            source_component=component_id,
        )

        self.mixed_output_from_shell = ""

        self.last_heartbeat_sent = 0

    def send_heartbeats(self):
        now = time.time()
        if now - self.last_heartbeat_sent > 0.5:
            self.last_heartbeat_sent = now
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                        0,
                                        0,
                                        0)

    def debug(self, msg):
        print("DEBUG: %s" % msg)

    def open_shell(self):
        self.shell = subprocess.Popen(["/bin/bash"],
                                      stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE,
                                      cwd="/tmp")
        fcntl.fcntl(self.shell.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        fcntl.fcntl(self.shell.stderr.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

    def run(self):
        self.open_shell()
        os.environ["PYTHONUNBUFFERED"]="1"
        while True:
            m = self.mav.recv_match(type="SERIAL_CONTROL", timeout=1000)
            self.send_heartbeats()
            shell_failure = self.shell.poll()
            if shell_failure is not None:
                self.debug("Shell is dead, restarting (%s)" % shell_failure)
                self.open_shell()
            if select.select([self.shell.stderr, self.shell.stdout],[],[],0)[0] != []:
                try:
                    self.mixed_output_from_shell += self.shell.stderr.read()
                except IOError as e:
                    if e.errno != 11:
                        raise
                try:
                    self.mixed_output_from_shell += self.shell.stdout.read()
                except IOError as e:
                    if e.errno != 11:
                        raise

            while len(self.mixed_output_from_shell):
                data = self.mixed_output_from_shell[:70]
                self.mixed_output_from_shell = self.mixed_output_from_shell[70:]
                data_len = len(data)
                data = [ord(x) for x in list(data)]
                data = data + ([0] * (70-len(data)))

                self.mav.mav.serial_control_send(
                    self.serial_control_dev,
                    mavutil.mavlink.SERIAL_CONTROL_FLAG_REPLY,
                    0,  # timeout
                    0,  # baud
                    data_len,
                    data
                )

            if m is None:
                time.sleep(0.1)
                continue
            if m.device != self.serial_control_dev:
                continue
            if m.count == 0:
                continue

            b = m.data[:m.count]
            text = "".join([chr(a) for a in b])
            text = re.sub("\r\n", "\n", text)  # not quite right, doesn't take into account \r at end of data
            self.shell.stdin.write(text)

if __name__ == '__main__':

    parser = optparse.OptionParser("bisect.py ")

    parser.add_option("", "--system-id",
                      type=int,
                      help="This script's system ID",
                      default=1,
    )

    parser.add_option("", "--component-id",
                      type=int,
                      help="This script's component ID",
                      default=10,
    )

    (opts, args) = parser.parse_args()

    s = SerialControlToShell(
        args[0],
        system_id=opts.system_id,
        component_id=opts.component_id,
    )
    s.run()
