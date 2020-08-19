#!/usr/bin/env python

from __future__ import print_function

'''
Build ArduPilot with various build-time options enabled or disabled

Usage is straight forward; invoke this script from the root directory
of an ArduPilot checkout:

pbarker@bluebottle:~/rc/ardupilot(build-with-disabled-features)$ ./Tools/autotest/build-with-disabled-features.py

BWFD: Building
Running: ("/home/pbarker/rc/ardupilot/Tools/autotest/autotest.py" "build.ArduCopter") in (.)
lckfile='/home/pbarker/rc/buildlogs/autotest.lck'
.
.
.
>>>> PASSED STEP: build.ArduCopter at Thu Feb 22 09:46:43 2018
check step:  build.ArduCopter
BWFD: ADVANCED_FAILSAFE OK
BWFD: Successes: ['MOUNT', 'AUTOTUNE_ENABLED', 'AC_FENCE', 'CAMERA', 'RANGEFINDER_ENABLED', 'PROXIMITY_ENABLED', 'AC_RALLY', 'AC_AVOID_ENABLED', 'AC_TERRAIN', 'PARACHUTE', 'NAV_GUIDED', 'OPTFLOW', 'VISUAL_ODOMETRY_ENABLED', 'FRSKY_TELEM_ENABLED', 'ADSB_ENABLED', 'PRECISION_LANDING', 'SPRAYER', 'WINCH_ENABLED', 'ADVANCED_FAILSAFE']
BWFD: Failures: ['LOGGING_ENABLED']
pbarker@bluebottle:~/rc/ardupilot(build-with-disabled-features)$ q

''' # noqa

import re
import shutil
import subprocess
import sys

from pysim import util


class Builder():

    def __init__(self, spec, autotest=False, board=None):
        self.config = spec["config"]
        self.autotest_build = spec["autotest_target"]
        self.target_binary = spec["target_binary"]
        if "blacklist_options" in spec:
            self.blacklist_options = spec["blacklist_options"]
        else:
            self.blacklist_options = []

        # list other features that have to be disabled when a feature
        # is disabled (recursion not done; be exhaustive):
        self.reverse_deps = spec["reverse-deps"]
        self.autotest = autotest
        self.board = board

    def description(self):
        if self.autotest:
            return self.autotest_build
        if self.target_binary:
            return "%s:%s" % (self.board, self.target_binary)
        print("Bad config")
        sys.exit(1)

    def reverse_deps_for_var(self, var):
        return self.reverse_deps.get(var, [])

    def progress(self, string):
        print("BWFD: %s" % string)

    def get_config_variables(self):
        ret = []
        r = (' *# *define +([A-Z_]+)\s+'
             '(ENABLED|DISABLED|!HAL_MINIMIZE_FEATURES)')
        with open(util.reltopdir(self.config)) as fd:
            for line in fd:
                match = re.match(r, line)
                if match is None:
                    continue
                if match.group(1) in ("ENABLE", "DISABLE",
                                      "!HAL_MINIMIZE_FEATURES"):
                    continue
                if match.group(1) in self.blacklist_options:
                    print("Skipping (%s)" % match.group(1))
                    continue
                ret.append((match.group(1), match.group(2)))
        return set(ret)

    def disable_option_in_config(self, var):
        tmpfile = util.reltopdir(self.config) + ".tmp"
        shutil.move(self.config, tmpfile)
        with open(self.config, 'w+') as out_fd:
            with open(util.reltopdir(tmpfile)) as fd:
                did_enable = False
                for line in fd:
                    regex = ' *# *define +%s\s+(ENABLED|DISABLED|!HAL_MINIMIZE_FEATURES)' % (var[0],)
                    match = re.match(regex, line)
                    if match is not None:
                        if (match.group(1) in ["ENABLED",
                                               "!HAL_MINIMIZE_FEATURES"]):
                            fnoo = "DISABLED"
                        else:
                            fnoo = "ENABLED"
                            did_enable = True

                        line = "#define %s %s\n" % (var[0], fnoo)
                    out_fd.write(line)
            # turn dependencies on or off:
        tmpfile = util.reltopdir(self.config) + ".tmp-deps"
        shutil.move(self.config, tmpfile)
        with open(self.config, 'w+') as out_fd:
            with open(util.reltopdir(tmpfile)) as fd:
                for line in fd:
                    things_to_toggle = self.reverse_deps_for_var(var[0])
                    for thing in things_to_toggle:
                        regex = ' *# *define +%s\s+(ENABLED|DISABLED|!HAL_MINIMIZE_FEATURES)' % thing
                        match = re.match(regex, line)
                        if match is not None:
                            if did_enable:
                                fnoo = "ENABLED"
                            else:
                                fnoo = "DISABLED"

                            line = "#define %s %s\n" % (thing, fnoo)
                    out_fd.write(line)

    def backup_config_filepath(self):
        return util.reltopdir(self.config) + ".backup"

    def backup_config(self):
        shutil.copy(self.config, self.backup_config_filepath())

    def restore_config(self):
        shutil.copy(self.backup_config_filepath(), self.config)

    def build_works(self):
        self.progress("Building")

        if self.autotest:
            return self.build_works_autotest()

        try:
            ret = util.run_cmd(["./waf", "configure", "--board", self.board])
        except subprocess.CalledProcessError:
            return False
        if ret != 0:
            return False
        try:
            ret = util.run_cmd(["./waf", "build", "--target", self.target_binary])
        except subprocess.CalledProcessError:
            return False
        if ret != 0:
            return False

        return True

    def build_works_autotest(self):
        autotest = util.reltopdir("Tools/autotest/autotest.py")
        try:
            ret = util.run_cmd([autotest, self.autotest_build])
        except subprocess.CalledProcessError:
            return False
        return ret == 0

    def run(self):
        self.progress("Doing: %s" % (self.autotest_build,))
        self.backup_config()
        successes = []
        failures = []
        for var in self.get_config_variables():
            print("var: %s" % str(var))
            self.disable_option_in_config(var)
            if self.build_works():
                self.progress("%s OK" % var[0])
                successes.append(var[0])
            else:
                self.progress("%s BAD" % var[0])
                failures.append(var[0])
            self.restore_config()

        self.successes = successes
        self.failures = failures

        self.progress("Successes: %s" % str(successes))
        self.progress("Failures: %s" % str(failures))


class BuilderCopter(Builder):
    def get_config_variables(self):
        ret = []
        r = '//#define ([A-Z_]+)\s+(ENABLED|DISABLED!HAL_MINIMIZE_FEATURES)'
        with open(util.reltopdir(self.config)) as fd:
            for line in fd:
                print("line: %s" % line)
                match = re.match(r, line)
                if match is not None:
                    ret.append(match.group(1))
        return ret


# read reverse dep "MODE_AUTO_ENABLED": ["AC_TERRAIN", "MODE_GUIDED"] thusly:
# "if mode-auto is disabled then you must also disable terrain and guided mode"

specs = [
    {
        "config": 'ArduCopter/config.h',
        "autotest_target": "build.Copter",
        "target_binary": "bin/arducopter",
        "reverse-deps": {
            "AC_FENCE": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "PROXIMITY_ENABLED": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "AC_RALLY": ["AC_TERRAIN"],
            "MODE_AUTO_ENABLED": ["AC_TERRAIN", "MODE_GUIDED"],
            "MODE_RTL_ENABLED": ["MODE_AUTO_ENABLED", "AC_TERRAIN", "MODE_SMARTRTL_ENABLED"],
            "BEACON_ENABLED": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "MODE_CIRCLE_ENABLED": ["MODE_AUTO_ENABLED", "AC_TERRAIN"],
            "MODE_GUIDED_ENABLED": ["MODE_AUTO_ENABLED",
                                    "AC_TERRAIN",
                                    "ADSB_ENABLED",
                                    "MODE_FOLLOW_ENABLED",
                                    "MODE_GUIDED_NOGPS_ENABLED"],
            "AC_AVOID_ENABLED": ["MODE_FOLLOW_ENABLED"],
        },
    },
    {
        "config": 'ArduCopter/config.h',
        "autotest_target": "build.Helicopter",
        "target_binary": "bin/arducopter-heli",
        "blacklist_options": ["TOY_MODE_ENABLED",
                              "MODE_ACRO_ENABLED",
                              "AUTOTUNE_ENABLED"],
        "reverse-deps": {
            "AC_FENCE": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "PROXIMITY_ENABLED": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "AC_RALLY": ["AC_TERRAIN"],
            "MODE_AUTO_ENABLED": ["AC_TERRAIN", "MODE_GUIDED"],
            "MODE_RTL_ENABLED": ["MODE_AUTO_ENABLED", "AC_TERRAIN", "MODE_SMARTRTL_ENABLED"],
            "BEACON_ENABLED": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "MODE_CIRCLE_ENABLED": ["MODE_AUTO_ENABLED", "AC_TERRAIN"],
            "MODE_GUIDED_ENABLED": ["MODE_AUTO_ENABLED",
                                    "AC_TERRAIN",
                                    "ADSB_ENABLED",
                                    "MODE_FOLLOW_ENABLED",
                                    "MODE_GUIDED_NOGPS_ENABLED"],
            "AC_AVOID_ENABLED": ["MODE_FOLLOW_ENABLED"],
        },
    },
    {
        "config": 'ArduPlane/config.h',
        "autotest_target": "build.Plane",
        "target_binary": "bin/arduplane",
        "reverse-deps": {
        },
    }, {
        "config": 'Rover/config.h',
        "autotest_target": "build.Rover",
        "target_binary": "bin/ardurover",
        "reverse-deps": {
        },
    }, {
        "config": 'ArduSub/config.h',
        "autotest_target": "build.Sub",
        "target_binary": "bin/ardusub",
        "reverse-deps": {
            "AC_FENCE": ["AVOIDANCE_ENABLED"],
            "PROXIMITY_ENABLED": ["AVOIDANCE_ENABLED"],
            "AC_RALLY": ["AC_TERRAIN"],
        },
    }, {
        "config": 'AntennaTracker/config.h',
        "autotest_target": "build.Tracker",
        "target_binary": "bin/antennatracker",
        "reverse-deps": {
        },
    },
]


builders = []

# append autotest builders:
for spec in specs:
    builder = Builder(spec, autotest=True)
    builder.run()
    builders.append(builder)

# append directly-build-by-waf targets
for spec in specs:
    for board in ["CubeOrange"]:
        builder = Builder(spec, board=board)
        builder.run()
        builders.append(builder)


print("")
for builder in builders:
    print("Builder: %s" % builder.description())
#    print("  Successes: %s" % builder.successes)
    print("   Failures: %s" % builder.failures)
