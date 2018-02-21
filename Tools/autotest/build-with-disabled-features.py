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

    def __init__(self):
        self.config = 'ArduCopter/APM_Config.h'
        self.autotest_build = "build.ArduCopter"

        # list other features that have to be disabled when a feature
        # is disabled (recursion not done; be exhaustive):
        self.reverse_deps = {
            "AC_FENCE": ["AC_AVOID_ENABLED"],
            "PROXIMITY_ENABLED": ["AC_AVOID_ENABLED"],
            "AC_RALLY": ["AC_TERRAIN"],
        }

    def reverse_deps_for_var(self, var):
        return self.reverse_deps.get(var, [])

    def progress(self, string):
        print("BWFD: %s" % string)

    def get_config_variables(self):
        ret = []
        with open(util.reltopdir(self.config)) as fd:
            for line in fd:
                match = re.match('//#define ([A-Z_]+)\s+(ENABLED|DISABLED)',
                                 line)
                if match is not None:
                    ret.append(match.group(1))
        return ret

    def disable_option_in_config(self, var):
        tmpfile = util.reltopdir(self.config) + ".tmp"
        shutil.move(self.config, tmpfile)
        out_fd = open(self.config, 'w+')
        with open(util.reltopdir(tmpfile)) as fd:
            for line in fd:
                things_to_toggle = self.reverse_deps_for_var(var)
                things_to_toggle.append(var)
                for thing in things_to_toggle:
                    line = re.sub(
                        '//(#define\s+%s\s+(ENABLED|DISABLED))' % thing,
                        "\\1",
                        line)
                out_fd.write(line)
        out_fd.close()

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
        self.backup_config()
        successes = []
        failures = []
        for var in self.get_config_variables():
            self.disable_option_in_config(var)
            if self.build_works():
                self.progress("%s OK" % var)
                successes.append(var)
            else:
                self.progress("%s BAD" % var)
                failures.append(var)
            self.restore_config()

        self.progress("Successes: %s" % str(successes))
        self.progress("Failures: %s" % str(failures))


builder = Builder()
builder.run()
