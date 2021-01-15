#!/usr/bin/env python
"""
 APM automatic test suite
 Andrew Tridgell, October 2011
"""
from __future__ import print_function
import atexit
import fnmatch
import glob
import optparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback
import threading
from distutils.dir_util import copy_tree

import rover
import arducopter
import arduplane
import ardusub
import antennatracker
import quadplane
import balancebot

import examples
from pysim import util
from pymavlink import mavutil
from pymavlink.generator import mavtemplate

tester = None

def buildlogs_dirpath():
    return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))


def buildlogs_path(path):
    """return a string representing path in the buildlogs directory"""
    bits = [buildlogs_dirpath()]
    if isinstance(path, list):
        bits.extend(path)
    else:
        bits.append(path)
    return os.path.join(*bits)


def get_default_params(atype, binary):
    """Get default parameters."""

    # use rover simulator so SITL is not starved of input
    HOME = mavutil.location(40.071374969556928,
                            -105.22978898137808,
                            1583.702759,
                            246)
    if "plane" in binary or "rover" in binary:
        frame = "rover"
    else:
        frame = "+"

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sitl = util.start_SITL(binary,
                           wipe=True,
                           model=frame,
                           home=home,
                           speedup=10,
                           unhide_parameters=True)
    mavproxy = util.start_MAVProxy_SITL(atype)
    print("Dumping defaults")
    idx = mavproxy.expect(['Saved [0-9]+ parameters to (\S+)'])
    if idx == 0:
        # we need to restart it after eeprom erase
        util.pexpect_close(mavproxy)
        util.pexpect_close(sitl)
        sitl = util.start_SITL(binary, model=frame, home=home, speedup=10)
        mavproxy = util.start_MAVProxy_SITL(atype)
        mavproxy.expect('Saved [0-9]+ parameters to (\S+)')
    parmfile = mavproxy.match.group(1)
    dest = buildlogs_path('%s-defaults.parm' % atype)
    shutil.copy(parmfile, dest)
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)
    print("Saved defaults for %s to %s" % (atype, dest))
    return True


def build_all_filepath():
    return util.reltopdir('Tools/scripts/build_all.sh')


def build_all():
    """Run the build_all.sh script."""
    print("Running build_all.sh")
    if util.run_cmd(build_all_filepath(), directory=util.reltopdir('.')) != 0:
        print("Failed build_all.sh")
        return False
    return True


def build_binaries():
    """Run the build_binaries.py script."""
    print("Running build_binaries.py")

    # copy the script (and various libraries used by the script) as it
    # changes git branch, which can change the script while running
    for thing in "build_binaries.py", "generate_manifest.py", "gen_stable.py", "build_binaries_history.py":
        orig = util.reltopdir('Tools/scripts/%s' % thing)
        copy = util.reltopdir('./%s' % thing)
        shutil.copy2(orig, copy)
    
    if util.run_cmd("./build_binaries.py", directory=util.reltopdir('.')) != 0:
        print("Failed build_binaries.py")
        return False
    return True


def build_examples():
    """Build examples."""
    for target in 'fmuv2', 'Pixhawk1', 'navio', 'linux':
        print("Running build.examples for %s" % target)
        try:
            util.build_examples(target)
        except Exception as e:
            print("Failed build_examples on board=%s" % target)
            print(str(e))
            return False

    return True

def build_unit_tests():
    """Build tests."""
    for target in ['linux']:
        print("Running build.unit_tests for %s" % target)
        try:
            util.build_tests(target)
        except Exception as e:
            print("Failed build.unit_tests on board=%s" % target)
            print(str(e))
            return False

    return True

def run_unit_test(test):
    print("Running (%s)" % test)
    subprocess.check_call([test])


def run_unit_tests():
    binary_dir = util.reltopdir(os.path.join('build',
                                             'linux',
                                             'tests',
                                             ))
    tests = glob.glob("%s/*" % binary_dir)
    success = True
    for test in tests:
        try:
            run_unit_test(test)
        except Exception as e:
            print("Exception running (%s): %s" % (test, e.message))
            success = False
    return success

def run_clang_scan_build():
    if util.run_cmd("scan-build python waf configure",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-configure")
        return False

    if util.run_cmd("scan-build python waf clean",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-clean")
        return False

    if util.run_cmd("scan-build python waf build",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-build")
        return False

    return True

def param_parse_filepath():
    return util.reltopdir('Tools/autotest/param_metadata/param_parse.py')


def all_vehicles():
    return ('ArduPlane',
            'ArduCopter',
            'Rover',
            'AntennaTracker',
            'ArduSub')


def build_parameters():
    """Run the param_parse.py script."""
    print("Running param_parse.py")
    for vehicle in all_vehicles():
        if util.run_cmd([param_parse_filepath(), '--vehicle', vehicle],
                        directory=util.reltopdir('.')) != 0:
            print("Failed param_parse.py (%s)" % vehicle)
            return False
    return True


def mavtogpx_filepath():
    return util.reltopdir("modules/mavlink/pymavlink/tools/mavtogpx.py")


def convert_gpx():
    """Convert any tlog files to GPX and KML."""
    mavlog = glob.glob(buildlogs_path("*.tlog"))
    passed = True
    for m in mavlog:
        util.run_cmd(mavtogpx_filepath() + " --nofixcheck " + m)
        gpx = m + '.gpx'
        kml = m + '.kml'
        try:
            util.run_cmd('gpsbabel -i gpx -f %s '
                         '-o kml,units=m,floating=1,extrude=1 -F %s' %
                         (gpx, kml))
        except subprocess.CalledProcessError as e:
            passed = False
        try:
            util.run_cmd('zip %s.kmz %s.kml' % (m, m))
        except subprocess.CalledProcessError as e:
            passed = False
        util.run_cmd("mavflightview.py --imagefile=%s.png %s" % (m, m))
    return passed


def test_prerequisites():
    """Check we have the right directories and tools to run tests."""
    print("Testing prerequisites")
    util.mkdir_p(buildlogs_dirpath())
    return True


def alarm_handler(signum, frame):
    """Handle test timeout."""
    global results, opts, tester
    try:
        print("Alarm handler called")
        if tester is not None:
            if tester.rc_thread is not None:
                tester.rc_thread_should_quit = True
                tester.rc_thread.join()
                tester.rc_thread = None
        results.add('TIMEOUT',
                    '<span class="failed-text">FAILED</span>',
                    opts.timeout)
        util.pexpect_close_all()
        convert_gpx()
        write_fullresults()
        os.killpg(0, signal.SIGKILL)
    except Exception:
        pass
    sys.exit(1)


def should_run_step(step):
    """See if a step should be skipped."""
    for skip in skipsteps:
        if fnmatch.fnmatch(step.lower(), skip.lower()):
            return False
    return True


__bin_names = {
    "Copter": "arducopter",
    "CopterTests1": "arducopter",
    "CopterTests1a": "arducopter",
    "CopterTests1b": "arducopter",
    "CopterTests1c": "arducopter",
    "CopterTests1d": "arducopter",
    "CopterTests1e": "arducopter",

    "CopterTests2": "arducopter",
    "CopterTests2a": "arducopter",
    "CopterTests2b": "arducopter",

    "Plane": "arduplane",
    "Rover": "ardurover",
    "Tracker": "antennatracker",
    "Helicopter": "arducopter-heli",
    "QuadPlane": "arduplane",
    "Sub": "ardusub",
    "BalanceBot": "ardurover",
    "SITLPeriphGPS" : "sitl_periph_gp.AP_Periph",
    "CAN" : "arducopter",
}


def binary_path(step, debug=False):
    try:
        vehicle = step.split(".")[1]
    except Exception:
        return None

    if vehicle in __bin_names:
        if len(__bin_names[vehicle].split(".")) == 2:
            config_name = __bin_names[vehicle].split(".")[0]
            binary_name = __bin_names[vehicle].split(".")[1]
        else:
            config_name = 'sitl'
            binary_name = __bin_names[vehicle]
    else:
        # cope with builds that don't have a specific binary
        return None

    binary = util.reltopdir(os.path.join('build',
                                         config_name,
                                         'bin',
                                         binary_name))
    if not os.path.exists(binary):
        if os.path.exists(binary + ".exe"):
            binary += ".exe"
        else:
            raise ValueError("Binary (%s) does not exist" % (binary,))

    return binary

def split_specific_test_step(step):
    print('step=%s' % str(step))
    m = re.match("((fly|drive|dive|test)[.][^.]+)[.](.*)", step)
    if m is None:
        return None
    return ( (m.group(1), m.group(3)) )

def find_specific_test_to_run(step):
    t = split_specific_test_step(step)
    if t is None:
        return None
    (testname, test) = t
    return "%s.%s" % (testname, test)

tester_class_map = {
    "test.Copter": arducopter.AutoTestCopter,
    "test.CopterTests1": arducopter.AutoTestCopterTests1,               #travis-ci
    "test.CopterTests1a": arducopter.AutoTestCopterTests1a, # 8m43s
    "test.CopterTests1b": arducopter.AutoTestCopterTests1b, # 8m5s 
    "test.CopterTests1c": arducopter.AutoTestCopterTests1c, # 5m17s 
    "test.CopterTests1d": arducopter.AutoTestCopterTests1d, # 8m20s
    "test.CopterTests1e": arducopter.AutoTestCopterTests1e, # 8m32s
    "test.CopterTests2": arducopter.AutoTestCopterTests2,               #travis-ci
    "test.CopterTests2a": arducopter.AutoTestCopterTests2a, # 8m23s
    "test.CopterTests2b": arducopter.AutoTestCopterTests2b, # 8m18s
    "test.Plane": arduplane.AutoTestPlane,
    "test.QuadPlane": quadplane.AutoTestQuadPlane,
    "test.Rover": rover.AutoTestRover,
    "test.BalanceBot": balancebot.AutoTestBalanceBot,
    "test.Helicopter": arducopter.AutoTestHeli,
    "test.Sub": ardusub.AutoTestSub,
    "test.Tracker": antennatracker.AutoTestTracker,
    "test.CAN": arducopter.AutoTestCAN,
}

suplementary_test_binary_map = {
    "test.CAN": "sitl_periph_gps.AP_Periph",
}

def run_specific_test(step, *args, **kwargs):
    t = split_specific_test_step(step)
    if t is None:
        return []
    (testname, test) = t

    tester_class = tester_class_map[testname]
    global tester
    tester = tester_class(*args, **kwargs)

    print("Got %s" % str(tester))
    for a in tester.tests():
        print("Got %s" % (a[0]))
        if a[0] == test:
            return tester.run_tests([a])
    print("Failed to find test %s on %s" % (test, testname))
    sys.exit(1)

def run_step(step):
    """Run one step."""

    # remove old logs
    util.run_cmd('/bin/rm -f logs/*.BIN logs/LASTLOG.TXT')

    if step == "prerequisites":
        return test_prerequisites()

    build_opts = {
        "j": opts.j,
        "debug": opts.debug,
        "clean": not opts.no_clean,
        "configure": not opts.no_configure,
        "math_check_indexes": opts.math_check_indexes,
        "extra_configure_args": opts.waf_configure_args,
    }

    if opts.Werror:
        build_opts['extra_configure_args'].append("--Werror")

    vehicle_binary = None
    if step == 'build.Plane':
        vehicle_binary = 'bin/arduplane'

    if step == 'build.Rover':
        vehicle_binary = 'bin/ardurover'

    if step == 'build.Copter':
        vehicle_binary = 'bin/arducopter'

    if step == 'build.Tracker':
        vehicle_binary = 'bin/antennatracker'

    if step == 'build.Helicopter':
        vehicle_binary = 'bin/arducopter-heli'

    if step == 'build.Sub':
        vehicle_binary = 'bin/ardusub'
    
    if step == 'build.SITLPeriphGPS':
        vehicle_binary = 'sitl_periph_gps.bin/AP_Periph'

    if step == 'build.Replay':
        return util.build_SITL('tools/Replay', clean=False, configure=False)
        
    if vehicle_binary is not None:
        if len(vehicle_binary.split(".")) == 1:
            return util.build_SITL(vehicle_binary, **build_opts)
        else:
            return util.build_SITL(vehicle_binary.split(".")[1], board = vehicle_binary.split(".")[0], **build_opts)

    binary = binary_path(step, debug=opts.debug)

    if step.startswith("defaults"):
        vehicle = step[9:]
        return get_default_params(vehicle, binary)

    if step in suplementary_test_binary_map:
        config_name = suplementary_test_binary_map[step].split('.')[0]
        binary_name = suplementary_test_binary_map[step].split('.')[1]
        supplementary_binary =  util.reltopdir(os.path.join('build',
                                                            config_name,
                                                            'bin',
                                                            binary_name))
        # we are running in conjunction with a supplementary app
        # can't have speedup
        opts.speedup = 1.0
    else:
        supplementary_binary = None
    fly_opts = {
        "viewerip": opts.viewerip,
        "use_map": opts.map,
        "valgrind": opts.valgrind,
        "gdb": opts.gdb,
        "lldb": opts.lldb,
        "gdbserver": opts.gdbserver,
        "breakpoints": opts.breakpoint,
        "disable_breakpoints": opts.disable_breakpoints,
        "frame": opts.frame,
        "_show_test_timings": opts.show_test_timings,
        "force_ahrs_type": opts.force_ahrs_type,
        "logs_dir": buildlogs_dirpath(),
        "sup_binary": supplementary_binary,
    }
    if opts.speedup is not None:
        fly_opts["speedup"] = opts.speedup

    # handle "test.Copter" etc:
    if step in tester_class_map:
        # create an instance of the tester class:
        global tester
        tester = tester_class_map[step](binary, **fly_opts)
        # run the test and return its result and the tester itself
        return (tester.autotest(), tester)

    # handle "test.Copter.CPUFailsafe" etc:
    specific_test_to_run = find_specific_test_to_run(step)
    if specific_test_to_run is not None:
        return run_specific_test(specific_test_to_run, binary, **fly_opts)

    if step == 'build.All':
        return build_all()

    if step == 'build.Binaries':
        return build_binaries()

    if step == 'build.examples':
        return build_examples()

    if step == 'run.examples':
        return examples.run_examples(debug=opts.debug, valgrind=False, gdb=False)

    if step == 'build.Parameters':
        return build_parameters()

    if step == 'convertgpx':
        return convert_gpx()

    if step == 'build.unit_tests':
        return build_unit_tests()

    if step == 'run.unit_tests':
        return run_unit_tests()

    if step == 'clang-scan-build':
        return run_clang_scan_build()

    raise RuntimeError("Unknown step %s" % step)


class TestResult(object):
    """Test result class."""
    def __init__(self, name, result, elapsed):
        self.name = name
        self.result = result
        self.elapsed = "%.1f" % elapsed


class TestFile(object):
    """Test result file."""
    def __init__(self, name, fname):
        self.name = name
        self.fname = fname


class TestResults(object):
    """Test results class."""
    def __init__(self):
        self.date = time.asctime()
        self.githash = util.run_cmd('git rev-parse HEAD',
                                    output=True,
                                    directory=util.reltopdir('.')).strip()
        if sys.version_info.major >= 3:
            self.githash = self.githash.decode('utf-8')
        self.tests = []
        self.files = []
        self.images = []

    def add(self, name, result, elapsed):
        """Add a result."""
        self.tests.append(TestResult(name, result, elapsed))

    def addfile(self, name, fname):
        """Add a result file."""
        self.files.append(TestFile(name, fname))

    def addimage(self, name, fname):
        """Add a result image."""
        self.images.append(TestFile(name, fname))

    def addglob(self, name, pattern):
        """Add a set of files."""
        for f in glob.glob(buildlogs_path(pattern)):
            self.addfile(name, os.path.basename(f))

    def addglobimage(self, name, pattern):
        """Add a set of images."""
        for f in glob.glob(buildlogs_path(pattern)):
            self.addimage(name, os.path.basename(f))

    def generate_badge(self):
        """
        Gets the badge template, populates and saves the result to buildlogs
        path.
        """
        passed_tests = len([t for t in self.tests if "PASSED" in t.result])
        total_tests = len(self.tests)
        badge_color = "#4c1" if passed_tests == total_tests else "#e05d44"

        badge_text = "{0}/{1}".format(passed_tests, total_tests)
        # Text length so it is not stretched by svg
        text_length = len(badge_text) * 70

        # Load template file
        template_path = 'Tools/autotest/web/autotest-badge-template.svg'
        with open(util.reltopdir(template_path), "r") as f:
            template = f.read()

        # Add our results to the template
        badge = template.format(color=badge_color,
                                text=badge_text,
                                text_length=text_length)
        with open(buildlogs_path("autotest-badge.svg"), "w") as f:
            f.write(badge)


def write_webresults(results_to_write):
    """Write webpage results."""
    t = mavtemplate.MAVTemplate()
    for h in glob.glob(util.reltopdir('Tools/autotest/web/*.html')):
        html = util.loadfile(h)
        f = open(buildlogs_path(os.path.basename(h)), mode='w')
        t.write(f, html, results_to_write)
        f.close()
    for f in glob.glob(util.reltopdir('Tools/autotest/web/*.png')):
        shutil.copy(f, buildlogs_path(os.path.basename(f)))
    copy_tree(util.reltopdir("Tools/autotest/web/css"), buildlogs_path("css"))
    results_to_write.generate_badge()


def write_fullresults():
    """Write out full results set."""
    global results
    results.addglob("Google Earth track", '*.kmz')
    results.addfile('Full Logs', 'autotest-output.txt')
    results.addglob('DataFlash Log', '*-log.bin')
    results.addglob("MAVLink log", '*.tlog')
    results.addglob("GPX track", '*.gpx')

    # results common to all vehicles:
    vehicle_files = [('{vehicle} defaults', '{vehicle}-defaults.parm'),
                     ('{vehicle} core', '{vehicle}.core'),
                     ('{vehicle} ELF', '{vehicle}.elf'), ]
    vehicle_globs = [('{vehicle} log', '{vehicle}-*.BIN'), ]
    for vehicle in all_vehicles():
        subs = {'vehicle': vehicle}
        for vehicle_file in vehicle_files:
            description = vehicle_file[0].format(**subs)
            filename = vehicle_file[1].format(**subs)
            results.addfile(description, filename)
        for vehicle_glob in vehicle_globs:
            description = vehicle_glob[0].format(**subs)
            glob = vehicle_glob[1].format(**subs)
            results.addglob(description, glob)

    results.addglob("CopterAVC log", 'CopterAVC-*.BIN')
    results.addfile("CopterAVC core", 'CopterAVC.core')

    results.addglob('APM:Libraries documentation', 'docs/libraries/index.html')
    results.addglob('APM:Plane documentation', 'docs/ArduPlane/index.html')
    results.addglob('APM:Copter documentation', 'docs/ArduCopter/index.html')
    results.addglob('APM:Rover documentation', 'docs/Rover/index.html')
    results.addglob('APM:Sub documentation', 'docs/ArduSub/index.html')
    results.addglobimage("Flight Track", '*.png')

    write_webresults(results)


def run_tests(steps):
    """Run a list of steps."""
    global results

    passed = True
    failed = []
    failed_testinstances = dict()
    for step in steps:
        util.pexpect_close_all()

        t1 = time.time()
        print(">>>> RUNNING STEP: %s at %s" % (step, time.asctime()))
        try:
            success = run_step(step)
            testinstance = None
            if type(success) == tuple:
                (success, testinstance) = success
            if success:
                results.add(step, '<span class="passed-text">PASSED</span>',
                            time.time() - t1)
                print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
            else:
                print(">>>> FAILED STEP: %s at %s" % (step, time.asctime()))
                passed = False
                failed.append(step)
                if testinstance is not None:
                    if failed_testinstances.get(step) is None:
                        failed_testinstances[step] = []
                    failed_testinstances[step].append(testinstance)
                results.add(step, '<span class="failed-text">FAILED</span>',
                            time.time() - t1)
        except Exception as msg:
            passed = False
            failed.append(step)
            print(">>>> FAILED STEP: %s at %s (%s)" %
                  (step, time.asctime(), msg))
            traceback.print_exc(file=sys.stdout)
            results.add(step,
                        '<span class="failed-text">FAILED</span>',
                        time.time() - t1)
    if not passed:
        keys = failed_testinstances.keys()
        if len(keys):
            print("Failure Summary:")
        for key in keys:
            print("  %s:" % key)
            for testinstance in failed_testinstances[key]:
                for failure in testinstance.fail_list:
                    (desc, exception, debug_filename) = failure
                    print("    %s (%s) (see %s)" % (desc, exception, debug_filename))

        print("FAILED %u tests: %s" % (len(failed), failed))

    global tester
    if tester is not None and tester.rc_thread is not None:
        if passed:
            print("BAD: RC Thread still alive after tests passed")
        tester.rc_thread_should_quit = True
        tester.rc_thread.join()
        tester.rc_thread = None

    util.pexpect_close_all()

    write_fullresults()

    return passed


vehicle_list = ['Sub', 'Copter', 'Plane', 'Tracker', 'Rover', 'QuadPlane', 'BalanceBot', 'Helicopter']


def list_subtests():
    """Print the list of tests and tests description for each vehicle."""
    for vehicle in sorted(vehicle_list):
        tester_class = tester_class_map["test.%s" % vehicle]
        tester = tester_class("/bin/true", None)
        subtests = tester.tests()
        print("%s:" % vehicle)
        for subtest in sorted(subtests, key=lambda x: x[0]):
            (name, description, function) = subtest
            print("    %s: %s" % (name, description))
        print("")


def list_subtests_for_vehicle(vehicle_type):
    """Print the list of tests for a vehicle."""
    # Check that we aren't in a sub test
    if "Test" in vehicle_type:
        vehicle_type = re.findall('[A-Z][a-z0-9]*', vehicle_type)[0]
    if vehicle_type in vehicle_list:
        tester_class = tester_class_map["test.%s" % vehicle_type]
        tester = tester_class("/bin/true", None)
        subtests = tester.tests()
        for subtest in sorted(subtests, key=lambda x: x[0]):
            (name, _, _) = subtest
            print("%s " % name, end='')
        print("")  # needed to clear the trailing %

if __name__ == "__main__":
    ''' main program '''
    os.environ['PYTHONUNBUFFERED'] = '1'
    
    if sys.platform != "darwin":
        os.putenv('TMPDIR', util.reltopdir('tmp'))

    class MyOptionParser(optparse.OptionParser):
        def format_epilog(self, formatter):
            return self.epilog

    parser = MyOptionParser(
        "autotest", epilog=""
        "e.g. autotest.py build.Rover test.Rover # test Rover\n"
        "e.g. autotest.py build.Rover test.Rover build.Plane test.Plane # test Rover and Plane\n"
        "e.g. autotest.py --debug --valgrind build.Rover test.Rover # test Rover under Valgrind\n"
        "e.g. autotest.py --debug --gdb build.Tracker test.Tracker # run Tracker under gdb\n"
        "e.g. autotest.py --debug --gdb build.Sub test.Sub.DiveManual # do specific Sub test\n"
    )
    parser.add_option("--skip",
                      type='string',
                      default='',
                      help='list of steps to skip (comma separated)')
    parser.add_option("--list",
                      action='store_true',
                      default=False,
                      help='list the available steps')
    parser.add_option("--list-subtests",
                      action='store_true',
                      default=False,
                      help='list available subtests e.g. test.Copter')
    parser.add_option("--viewerip",
                      default=None,
                      help='IP address to send MAVLink and fg packets to')
    parser.add_option("--map",
                      action='store_true',
                      default=False,
                      help='show map')
    parser.add_option("--experimental",
                      default=False,
                      action='store_true',
                      help='enable experimental tests')
    parser.add_option("--timeout",
                      default=5400,
                      type='int',
                      help='maximum runtime in seconds')
    parser.add_option("--frame",
                      type='string',
                      default=None,
                      help='specify frame type')
    parser.add_option("--show-test-timings",
                      action="store_true",
                      default=False,
                      help="show how long each test took to run")
    parser.add_option("--validate-parameters",
                      action="store_true",
                      default=False,
                      help="validate vehicle parameter files")
    parser.add_option("--Werror",
                      action='store_true',
                      default=False,
                      help='configure with --Werror')

    group_build = optparse.OptionGroup(parser, "Build options")
    group_build.add_option("--no-configure",
                           default=False,
                           action='store_true',
                           help='do not configure before building',
                           dest="no_configure")
    group_build.add_option("", "--waf-configure-args",
                           action="append",
                           dest="waf_configure_args",
                           type="string",
                           default=[],
                           help="extra arguments passed to waf in configure")
    group_build.add_option("-j", default=None, type='int', help='build CPUs')
    group_build.add_option("--no-clean",
                           default=False,
                           action='store_true',
                           help='do not clean before building',
                           dest="no_clean")
    group_build.add_option("--debug",
                           default=False,
                           action='store_true',
                           help='make built binaries debug binaries')
    group_build.add_option("--enable-math-check-indexes",
                           default=False,
                           action="store_true",
                           dest="math_check_indexes",
                           help="enable checking of math indexes")
    parser.add_option_group(group_build)

    group_sim = optparse.OptionGroup(parser, "Simulation options")
    group_sim.add_option("--speedup",
                         default=None,
                         type='int',
                         help='speedup to run the simulations at')
    group_sim.add_option("--valgrind",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under valgrind')
    group_sim.add_option("--gdb",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under gdb')
    group_sim.add_option("--gdbserver",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under gdbserver')
    group_sim.add_option("--lldb",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under lldb')
    group_sim.add_option("-B", "--breakpoint",
                         type='string',
                         action="append",
                         default=[],
                         help="add a breakpoint at given location in debugger")
    group_sim.add_option("--disable-breakpoints",
                         default=False,
                         action='store_true',
                         help="disable all breakpoints before starting")
    group_sim.add_option("", "--force-ahrs-type",
                         dest="force_ahrs_type",
                         default=None,
                         help="force a specific AHRS type (e.g. 10 for SITL-ekf")
    parser.add_option_group(group_sim)

    group_completion = optparse.OptionGroup(parser, "Completion helpers")
    group_completion.add_option("--list-vehicles",
                                action='store_true',
                                default=False,
                                help='list available vehicles')
    group_completion.add_option("--list-vehicles-test",
                                action='store_true',
                                default=False,
                                help='list available vehicle tester')
    group_completion.add_option("--list-subtests-for-vehicle",
                                type='string',
                                default="",
                                 help='list available subtests for a vehicle e.g Copter')
    parser.add_option_group(group_completion)

    opts, args = parser.parse_args()

    steps = [
        'prerequisites',
        'build.Binaries',
        'build.All',
        'build.Parameters',

        'build.Replay',

        'build.unit_tests',
        'run.unit_tests',
        'build.examples',
        'run.examples',

        'build.Plane',
        'defaults.Plane',
        'test.Plane',
        'test.QuadPlane',

        'build.Rover',
        'defaults.Rover',
        'test.Rover',
        'test.BalanceBot',

        'build.Copter',
        'defaults.Copter',
        'test.Copter',

        'build.Helicopter',
        'test.Helicopter',

        'build.Tracker',
        'defaults.Tracker',
        'test.Tracker',

        'build.Sub',
        'defaults.Sub',
        'test.Sub',

        'build.SITLPeriphGPS',
        'test.CAN',

        'convertgpx',
    ]

    moresteps = [
        'test.CopterTests1',
        'test.CopterTests1a',
        'test.CopterTests1b',
        'test.CopterTests1c',
        'test.CopterTests1d',
        'test.CopterTests1e',

        'test.CopterTests2',
        'test.CopterTests2a',
        'test.CopterTests2b',

        'clang-scan-build',
    ]

    # canonicalise the step names.  This allows
    # backwards-compatability from the hodge-podge
    # fly.ArduCopter/drive.APMrover2 to the more common test.Copter
    # test.Rover
    step_mapping = {
        "build.ArduPlane": "build.Plane",
        "build.ArduCopter": "build.Copter",
        "build.APMrover2": "build.Rover",
        "build.ArduSub": "build.Sub",
        "build.AntennaTracker": "build.Tracker",
        "fly.ArduCopter": "test.Copter",
        "fly.ArduPlane": "test.Plane",
        "fly.QuadPlane": "test.QuadPlane",
        "dive.ArduSub": "test.Sub",
        "drive.APMrover2": "test.Rover",
        "drive.BalanceBot": "test.BalanceBot",
        "drive.balancebot": "test.BalanceBot",
        "fly.CopterAVC": "test.Helicopter",
        "test.AntennaTracker": "test.Tracker",
        "defaults.ArduCopter": "defaults.Copter",
        "defaults.ArduPlane": "defaults.Plane",
        "defaults.ArduSub": "defaults.Sub",
        "defaults.APMrover2": "defaults.Rover",
        "defaults.AntennaTracker": "defaults.Tracker",
        "fly.ArduCopterTests1": "test.CopterTests1",
        "fly.ArduCopterTests1a": "test.CopterTests1a",
        "fly.ArduCopterTests1b": "test.CopterTests1b",
        "fly.ArduCopterTests1c": "test.CopterTests1c",
        "fly.ArduCopterTests1d": "test.CopterTests1d",
        "fly.ArduCopterTests1e": "test.CopterTests1e",

        "fly.ArduCopterTests2": "test.CopterTests2",
        "fly.ArduCopterTests2a": "test.CopterTests2a",
        "fly.ArduCopterTests2b": "test.CopterTests2b",

    }

    # form up a list of bits NOT to run, mapping from old step names
    # to new step names as appropriate.
    skipsteps = opts.skip.split(',')
    new_skipsteps = []
    for skipstep in skipsteps:
        if skipstep in step_mapping:
            new_skipsteps.append(step_mapping[skipstep])
        else:
            new_skipsteps.append(skipstep)
    skipsteps = new_skipsteps

    # ensure we catch timeouts
    signal.signal(signal.SIGALRM, alarm_handler)
    signal.alarm(opts.timeout)

    if opts.list:
        for step in steps:
            print(step)
        sys.exit(0)

    if opts.list_subtests:
        list_subtests()
        sys.exit(0)

    if opts.list_subtests_for_vehicle:
        list_subtests_for_vehicle(opts.list_subtests_for_vehicle)
        sys.exit(0)

    if opts.list_vehicles_test:
        print(' '.join(__bin_names.keys()))
        sys.exit(0)

    if opts.list_vehicles:
        print(' '.join(vehicle_list))
        sys.exit(0)

    util.mkdir_p(buildlogs_dirpath())

    lckfile = buildlogs_path('autotest.lck')
    print("lckfile=%s" % repr(lckfile))
    lck = util.lock_file(lckfile)

    if lck is None:
        print("autotest is locked - exiting.  lckfile=(%s)" % (lckfile,))
        sys.exit(0)

    atexit.register(util.pexpect_close_all)

    # provide backwards-compatability from (e.g.) drive.APMrover2 -> test.Rover
    newargs = []
    for arg in args:
        for _from, to in step_mapping.items():
            arg = re.sub("^%s" % _from, to, arg)
        newargs.append(arg)
    args = newargs

    if len(args) > 0:
        # allow a wildcard list of steps
        matched = []
        for a in args:
            matches = [step for step in steps
                       if fnmatch.fnmatch(step.lower(), a.lower())]
            x = find_specific_test_to_run(a)
            if x is not None:
                matches.append(x)

            if a in moresteps:
                matches.append(a)

            if not len(matches):
                print("No steps matched {}".format(a))
                sys.exit(1)
            matched.extend(matches)
        steps = matched

    # skip steps according to --skip option:
    steps_to_run = [s for s in steps if should_run_step(s)]

    results = TestResults()

    try:
        if not run_tests(steps_to_run):
            sys.exit(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught; closing pexpect connections")
        util.pexpect_close_all()
        raise
    except Exception:
        # make sure we kill off any children
        util.pexpect_close_all()
        raise
