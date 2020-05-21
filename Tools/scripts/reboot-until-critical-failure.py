#!/usr/bin/env python

import optparse
import serial
import time
import sys

from pymavlink import mavutil


class RebootUntilCriticalFailure:
    '''script to keep rebooting until the device comes up with a critical failure'''

    def __init__(self,
                 device,
                 source_system=37,
                 source_component=37,
                 target_system=1,
                 target_component=1):
        self.source_system = source_system
        self.source_component = source_component
        self.target_component = target_component
        self.target_system = target_system
        self.device = device

        self.reboot_message = mavutil.mavlink.MAVLink_command_long_message(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            1, # confirmation
            0,
            0,
            0,
            0,
            0,
            0,
            0)

    def open_serial_connection(self):
        self.mav = mavutil.mavlink_connection(
            self.device,
            robust_parsing=True,
            source_component=self.source_component)

    def run(self):
        self.open_serial_connection()
        attempts = 0
        while True:
            attempts += 1
            print("Attempt %u" % attempts)
            print("Sleeping 10")
            time.sleep(10) # to reliably detect reboot
            while True:
                if self.mav.recv_match(timeout=0.01) is None:
                    break
            m = self.mav.recv_match(type='ATTITUDE',
                                    blocking=True,
                                    timeout=10)
            if m is None:
                raise Exception("Did not get attitude")
            old_timestamp = m.time_boot_ms
            print("Got old timestamp (%u)" % old_timestamp)
            self.mav.mav.command_long_send(self.target_system,
                                           self.target_component,
                                           mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                           1,  # confirmation
                                           1, # reboot autopilot
                                           0,
                                           0,
                                           0,
                                           0,
                                           0,
                                           0)
            while True:
                try:
                    m = self.mav.recv_match(type='ATTITUDE',
                                            blocking=True,
                                            timeout=60)
                except serial.serialutil.SerialException as x:
                    print("Serial exception caught")
                    try:
                        self.open_serial_connection()
                    except Exception:
                        time.sleep(1)
                    continue
                if m is None:
                    raise Exception("timeout waiting for ATTITUDE")
                new_timestamp = m.time_boot_ms
                print("Got new timestamp (%u)" % new_timestamp)
                if new_timestamp < old_timestamp:
                    print("reboot detected")
                    break
                if new_timestamp - old_timestamp > 60000:
                    raise Exception("Did not detect reboot within 60s")

            tstart = time.time()
            timeout = 10
            while time.time() - tstart < timeout:
                m = self.mav.recv_match(type='SYS_STATUS',
                                        blocking=True,
                                        timeout=timeout)
                print("Received SYS_STATUS")
                if m is None:
                    raise Exception("timeout waiting for SYS_STATUS")
                if m.errors_count4:
                    print("Failure detected after %u attempts" % attempts)
                    sys.exit(0)
                if m.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_LOGGING:
                    if (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_LOGGING) == 0:
                        print("Logging Failure detected after %u attempts" % attempts)
                        sys.exit(0)
            print("No failure detected after %u seconds; rebooting again (this was attempt #%u)" % (timeout, attempts))


if __name__ == '__main__':
    parser = optparse.OptionParser("reboot-until-critical-failure.py")
    parser.add_option("-d", "--device", type='string')

    cmd_opts, cmd_args = parser.parse_args()

    if cmd_opts.device is None:
        raise Exception("Need a device")

    rebooter = RebootUntilCriticalFailure(device=cmd_opts.device)
    rebooter.run()
