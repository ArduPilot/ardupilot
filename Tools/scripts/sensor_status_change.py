#!/usr/bin/env python3

"""
Parses a log file and shows how the SENSOR_STATUS flags changed over time.
Wrapper around bitmask_change.py.

AP_FLAKE8_CLEAN

"""

import sys

from bitmask_change import BitmaskChange

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: sensor_status_change.py <logfile>", file=sys.stderr)
        sys.exit(1)
    fields = [
        ("SYS_STATUS", "onboard_control_sensors_present"),
        ("SYS_STATUS", "onboard_control_sensors_enabled"),
        ("SYS_STATUS", "onboard_control_sensors_health"),
    ]
    BitmaskChange(sys.argv[1], fields).run()
