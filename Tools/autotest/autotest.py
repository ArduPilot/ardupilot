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

import apmrover2
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
        idx = mavproxy.expect('Saved [0-9]+ parameters to (\S+)')
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
    # copy the script as it changes git branch, which can change the
    # script while running
    orig = util.reltopdir('Tools/scripts/build_binaries.py')
    copy = util.reltopdir('./build_binaries.py')
    shutil.copy2(orig, copy)

    # also copy generate_manifest library:
    orig_gm = util.reltopdir('Tools/scripts/generate_manifest.py')
    copy_gm = util.reltopdir('./generate_manifest.py')
    shutil.copy2(orig_gm, copy_gm)

    if util.run_cmd(copy, directory=util.reltopdir('.')) != 0:
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

def param_parse_filepath():
    return util.reltopdir('Tools/autotest/param_metadata/param_parse.py')


def all_vehicles():
    return ('ArduPlane',
            'ArduCopter',
            'APMrover2',
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
    global results, opts
    try:
        print("Alarm handler called")
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
    "ArduCopter": "arducopter",
    "ArduPlane": "arduplane",
    "APMrover2": "ardurover",
    "AntennaTracker": "antennatracker",
    "CopterAVC": "arducopter-heli",
    "QuadPlane": "arduplane",
    "ArduSub": "ardusub",
    "balancebot": "ardurover",
    "BalanceBot": "ardurover",
}


def binary_path(step, debug=False):
    try:
        vehicle = step.split(".")[1]
    except Exception:
        return None

    if vehicle in __bin_names:
        binary_name = __bin_names[vehicle]
    else:
        # cope with builds that don't have a specific binary
        return None

    binary = util.reltopdir(os.path.join('build',
                                         'sitl',
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
    "fly.ArduCopter": arducopter.AutoTestCopter,
    "fly.ArduPlane": arduplane.AutoTestPlane,
    "fly.QuadPlane": quadplane.AutoTestQuadPlane,
    "drive.APMrover2": apmrover2.AutoTestRover,
    "drive.balancebot": balancebot.AutoTestBalanceBot,
    "fly.CopterAVC": arducopter.AutoTestHeli,
    "dive.ArduSub": ardusub.AutoTestSub,
    "test.AntennaTracker": antennatracker.AutoTestTracker,
}

def run_specific_test(step, *args, **kwargs):
    t = split_specific_test_step(step)
    if t is None:
        return []
    (testname, test) = t

    tester_class = tester_class_map[testname]
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
        "extra_configure_args": opts.waf_configure_args,
    }

    vehicle_binary = None
    if step == 'build.ArduPlane':
        vehicle_binary = 'bin/arduplane'

    if step == 'build.APMrover2':
        vehicle_binary = 'bin/ardurover'

    if step == 'build.ArduCopter':
        vehicle_binary = 'bin/arducopter'

    if step == 'build.AntennaTracker':
        vehicle_binary = 'bin/antennatracker'

    if step == 'build.Helicopter':
        vehicle_binary = 'bin/arducopter-heli'

    if step == 'build.ArduSub':
        vehicle_binary = 'bin/ardusub'

    if vehicle_binary is not None:
        return util.build_SITL(vehicle_binary, **build_opts)

    binary = binary_path(step, debug=opts.debug)

    if step.startswith("defaults"):
        vehicle = step[9:]
        return get_default_params(vehicle, binary)

    fly_opts = {
        "viewerip": opts.viewerip,
        "use_map": opts.map,
        "valgrind": opts.valgrind,
        "gdb": opts.gdb,
        "gdbserver": opts.gdbserver,
        "breakpoints": opts.breakpoint,
        "frame": opts.frame,
        "_show_test_timings": opts.show_test_timings,
    }
    if opts.speedup is not None:
        fly_opts["speedup"] = opts.speedup

    # handle "fly.ArduCopter" etc:
    if step in tester_class_map:
        return tester_class_map[step](binary, **fly_opts).autotest()

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
    vehicle_files = [('{vehicle} build log', '{vehicle}.txt'),
                     ('{vehicle} code size', '{vehicle}.sizes.txt'),
                     ('{vehicle} stack sizes', '{vehicle}.framesizes.txt'),
                     ('{vehicle} defaults', '{vehicle}-defaults.parm'),
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
    results.addglob('APM:Rover documentation', 'docs/APMrover2/index.html')
    results.addglob('APM:Sub documentation', 'docs/ArduSub/index.html')
    results.addglobimage("Flight Track", '*.png')

    write_webresults(results)


def check_logs(step):
    """Check for log files from a step."""
    print("check step: ", step)
    if step.startswith('fly.'):
        vehicle = step[4:]
    elif step.startswith('drive.'):
        vehicle = step[6:]
    elif step.startswith('dive.'):
        vehicle = step[5:]
    else:
        return
    logs = glob.glob("logs/*.BIN")
    for log in logs:
        bname = os.path.basename(log)
        newname = buildlogs_path("%s-%s" % (vehicle, bname))
        print("Renaming %s to %s" % (log, newname))
        shutil.move(log, newname)

    corefile = "core"
    if os.path.exists(corefile):
        newname = buildlogs_path("%s.core" % vehicle)
        print("Renaming %s to %s" % (corefile, newname))
        shutil.move(corefile, newname)
        try:
            util.run_cmd('/bin/cp build/sitl/bin/* %s' % buildlogs_dirpath(),
                         directory=util.reltopdir('.'))
        except Exception:
            print("Unable to save binary")


def run_tests(steps):
    """Run a list of steps."""
    global results

    passed = True
    failed = []
    for step in steps:
        util.pexpect_close_all()

        t1 = time.time()
        print(">>>> RUNNING STEP: %s at %s" % (step, time.asctime()))
        try:
            if run_step(step):
                results.add(step, '<span class="passed-text">PASSED</span>',
                            time.time() - t1)
                print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
                check_logs(step)
            else:
                print(">>>> FAILED STEP: %s at %s" % (step, time.asctime()))
                passed = False
                failed.append(step)
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
            check_logs(step)
    if not passed:
        print("FAILED %u tests: %s" % (len(failed), failed))

    util.pexpect_close_all()

    write_fullresults()

    return passed


if __name__ == "__main__":
    ''' main program '''
    os.environ['PYTHONUNBUFFERED'] = '1'

    os.putenv('TMPDIR', util.reltopdir('tmp'))

    parser = optparse.OptionParser("autotest")
    parser.add_option("--skip",
                      type='string',
                      default='',
                      help='list of steps to skip (comma separated)')
    parser.add_option("--list",
                      action='store_true',
                      default=False,
                      help='list the available steps')
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
                      default=3600,
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
    group_sim.add_option("-B", "--breakpoint",
                         type='string',
                         action="append",
                         default=[],
                         help="add a breakpoint at given location in debugger")
    parser.add_option_group(group_sim)

    opts, args = parser.parse_args()

    steps = [
        'prerequisites',
        'build.All',
        'build.Binaries',
        'build.Parameters',

        'build.unit_tests',
        'run.unit_tests',
        'build.examples',
        'run.examples',

        'build.ArduPlane',
        'defaults.ArduPlane',
        'fly.ArduPlane',
        'fly.QuadPlane',

        'build.APMrover2',
        'defaults.APMrover2',
        'drive.APMrover2',
        'drive.balancebot',

        'build.ArduCopter',
        'defaults.ArduCopter',
        'fly.ArduCopter',

        'build.Helicopter',
        'fly.CopterAVC',

        'build.AntennaTracker',
        'defaults.AntennaTracker',
        'test.AntennaTracker',

        'build.ArduSub',
        'defaults.ArduSub',
        'dive.ArduSub',

        'convertgpx',
    ]

    skipsteps = opts.skip.split(',')

    # ensure we catch timeouts
    signal.signal(signal.SIGALRM, alarm_handler)
    signal.alarm(opts.timeout)

    if opts.list:
        for step in steps:
            print(step)
        sys.exit(0)

    util.mkdir_p(buildlogs_dirpath())

    lckfile = buildlogs_path('autotest.lck')
    print("lckfile=%s" % repr(lckfile))
    lck = util.lock_file(lckfile)

    if lck is None:
        print("autotest is locked - exiting.  lckfile=(%s)" % (lckfile,))
        sys.exit(0)

    atexit.register(util.pexpect_close_all)

    if len(args) > 0:
        # allow a wildcard list of steps
        matched = []
        for a in args:
            matches = [step for step in steps
                       if fnmatch.fnmatch(step.lower(), a.lower())]
            x = find_specific_test_to_run(a)
            if x is not None:
                matches.append(x)

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
        util.pexpect_close_all()
        sys.exit(1)
    except Exception:
        # make sure we kill off any children
        util.pexpect_close_all()
        raise
