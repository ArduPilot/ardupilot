#!/usr/bin/env python

"""
Extract version information for the various vehicle types, print it
"""

import os
import re
import sys

from optparse import OptionParser

parser = OptionParser("print_version.py [options] ArduCopter|ArduPlane|Rover|AntennaTracker")

(opts, args) = parser.parse_args()

includefiles = {
    "ArduCopter": "version.h",
    "ArduPlane": "version.h",
    "Rover": "version.h",
    "AntennaTracker": "version.h",
    "ArduSub": "version.h",
}

if len(args) > 0:
    vehicle = args[0]
    if vehicle not in includefiles:
        print("Unknown vehicle (%s) (be in a vehicle directory or supply a vehicle type as an argument)" % (vehicle,))
        sys.exit(1)
    includefilepath = "%s/%s" % (vehicle, includefiles[vehicle])
else:
    # assume we are in e.g. APM/Rover/
    vehicle = os.path.basename(os.getcwd())
    if vehicle not in includefiles:
        print("Unknown vehicle (%s) (be in a vehicle directory or supply a vehicle type as an argument)" % (vehicle,))
        sys.exit(1)
    includefilepath = includefiles[vehicle]


file = open(includefilepath)

firmware_version_regex = re.compile(".*define +FIRMWARE_VERSION.*")
firmware_version_extract_regex = re.compile(".*define +FIRMWARE_VERSION[	 ]+(?P<major>\d+)[ ]*,[ 	]*(?P<minor>\d+)[ ]*,[	 ]*(?P<point>\d+)[ ]*,[	 ]*(?P<type>[A-Z_]+)[	 ]*")

for line in file:
    if not firmware_version_regex.match(line):
        continue
    match = firmware_version_extract_regex.match(line)
    if not match:
        print("Failed to match FIRMWARE_VERSION line (%s)" % (line,))
        sys.exit(1)
    print("%d.%d.%d-%s" % (int(match.group("major")),
                           int(match.group("minor")),
                           int(match.group("point")),
                           match.group("type")))
