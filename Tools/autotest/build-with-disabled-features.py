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

'''

import re
import shutil
import subprocess

from pysim import util


class Builder():

    def __init__(self, spec):
        self.config = spec["config"]
        self.autotest_build = spec["builddir"]

        # list other features that have to be disabled when a feature
        # is disabled (recursion not done; be exhaustive):
        self.reverse_deps = spec["reverse-deps"]

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
                ret.append( (match.group(1), match.group(2) ))
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
        "builddir": "build.ArduCopter",
        "reverse-deps": {
            "AC_FENCE": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "PROXIMITY_ENABLED": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED" ],
            "AC_RALLY": ["AC_TERRAIN"],
            "MODE_AUTO_ENABLED": ["AC_TERRAIN", "MODE_GUIDED"],
            "MODE_RTL_ENABLED": ["MODE_AUTO_ENABLED", "AC_TERRAIN"],
            "BEACON_ENABLED": ["AC_AVOID_ENABLED", "MODE_FOLLOW_ENABLED"],
            "MODE_CIRCLE_ENABLED": ["MODE_AUTO_ENABLED", "AC_TERRAIN"],
            "MODE_GUIDED_ENABLED": ["MODE_AUTO_ENABLED", "AC_TERRAIN"],
            "AC_AVOID_ENABLED": ["MODE_FOLLOW_ENABLED"],
        },
    }, {
        "config": 'ArduPlane/config.h',
        "builddir": "build.ArduPlane",
        "reverse-deps": {
        },
    }, {
        "config": 'APMrover2/config.h',
        "builddir": "build.APMrover2",
        "reverse-deps": {
        },
    }, {
        "config": 'ArduSub/config.h',
        "builddir": "build.ArduSub",
        "reverse-deps": {
            "AC_FENCE": ["AVOIDANCE_ENABLED"],
            "PROXIMITY_ENABLED": ["AVOIDANCE_ENABLED"],
            "AC_RALLY": ["AC_TERRAIN"],
        },
    }, {
        "config": 'AntennaTracker/config.h',
        "builddir": "build.AntennaTracker",
        "reverse-deps": {
        },
    },
]

builders = []
for spec in specs:
    builder = Builder(spec)
    builder.run()
    builders.append(builder)

print("")
for builder in builders:
    print("Builder: %s" % builder.autotest_build)
    print("  Successes: %s" % builder.successes)
    print("   Failures: %s" % builder.failures)
