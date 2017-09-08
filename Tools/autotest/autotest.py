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
import shutil
import signal
import sys
import time
import traceback

import apmrover2
import arducopter
import arduplane
import quadplane
import ardusub
from pysim import util
from pymavlink import mavutil
from pymavlink.generator import mavtemplate

def buildlogs_dirpath():
    return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))

def buildlogs_path(path):
    '''return a string representing path in the buildlogs directory'''
    bits = [buildlogs_dirpath()]
    if isinstance(path, list):
        bits.extend(path)
    else:
        bits.append(path)
    return os.path.join(*bits)

def get_default_params(atype, binary):
    """Get default parameters."""

    # use rover simulator so SITL is not starved of input
    HOME = mavutil.location(40.071374969556928, -105.22978898137808, 1583.702759, 246)
    if "plane" in binary or "rover" in binary:
        frame = "rover"
    else:
        frame = "+"

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sitl = util.start_SITL(binary, wipe=True, model=frame, home=home, speedup=10, unhide_parameters=True)
    mavproxy = util.start_MAVProxy_SITL(atype)
    print("Dumping defaults")
    idx = mavproxy.expect(['Please Run Setup', 'Saved [0-9]+ parameters to (\S+)'])
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


def build_all():
    """Run the build_all.sh script."""
    print("Running build_all.sh")
    if util.run_cmd(util.reltopdir('Tools/scripts/build_all.sh'), directory=util.reltopdir('.')) != 0:
        print("Failed build_all.sh")
        return False
    return True


def build_binaries():
    """Run the build_binaries.sh script."""
    print("Running build_binaries.sh")
    # copy the script as it changes git branch, which can change the script while running
    orig = util.reltopdir('Tools/scripts/build_binaries.sh')
    copy = util.reltopdir('./build_binaries.sh')
    shutil.copy2(orig, copy)

    if util.run_cmd(copy, directory=util.reltopdir('.')) != 0:
        print("Failed build_binaries.sh")
        return False
    return True


def build_devrelease():
    """Run the build_devrelease.sh script."""
    print("Running build_devrelease.sh")
    # copy the script as it changes git branch, which can change the script while running
    orig = util.reltopdir('Tools/scripts/build_devrelease.sh')
    copy = util.reltopdir('./build_devrelease.sh')
    shutil.copy2(orig, copy)

    if util.run_cmd(copy, directory=util.reltopdir('.')) != 0:
        print("Failed build_devrelease.sh")
        return False
    return True


def build_examples():
    """Build examples."""
    for target in 'px4-v2', 'navio':
        print("Running build.examples for %s" % target)
        try:
            util.build_examples(target)
        except Exception as e:
            print("Failed build_examples on board=%s" % target)
            print(str(e))
            return False

    return True


def build_parameters():
    """Run the param_parse.py script."""
    print("Running param_parse.py")
    if util.run_cmd(util.reltopdir('Tools/autotest/param_metadata/param_parse.py'), directory=util.reltopdir('.')) != 0:
        print("Failed param_parse.py")
        return False
    return True


def convert_gpx():
    """Convert any tlog files to GPX and KML."""
    mavlog = glob.glob(buildlogs_path("*.tlog"))
    for m in mavlog:
        util.run_cmd(util.reltopdir("modules/mavlink/pymavlink/tools/mavtogpx.py") + " --nofixcheck " + m)
        gpx = m + '.gpx'
        kml = m + '.kml'
        util.run_cmd('gpsbabel -i gpx -f %s -o kml,units=m,floating=1,extrude=1 -F %s' % (gpx, kml), checkfail=False)
        util.run_cmd('zip %s.kmz %s.kml' % (m, m), checkfail=False)
        util.run_cmd("mavflightview.py --imagefile=%s.png %s" % (m, m))
    return True


def test_prerequisites():
    """Check we have the right directories and tools to run tests."""
    print("Testing prerequisites")
    util.mkdir_p(buildlogs_dirpath())
    return True


def alarm_handler(signum, frame):
    """Handle test timeout."""
    global results, opts
    try:
        results.add('TIMEOUT', '<span class="failed-text">FAILED</span>', opts.timeout)
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
    "ArduCopter" : "arducopter",
    "ArduPlane" : "arduplane",
    "APMrover2" : "ardurover",
    "AntennaTracker" : "antennatracker",
    "CopterAVC" : "arducopter-heli",
    "QuadPlane" : "arduplane",
    "ArduSub" : "ardusub"
}

def binary_path(step, debug=False):
    vehicle = step.split(".")[1]

    if vehicle in __bin_names:
        binary_name = __bin_names[vehicle]
    else:
        # cope with builds that don't have a specific binary
        return None

    if debug:
        binary_basedir = "sitl-debug"
    else:
        binary_basedir = "sitl"

    binary = util.reltopdir(os.path.join('build', binary_basedir, 'bin', binary_name))
    if not os.path.exists(binary):
        if os.path.exists(binary + ".exe"):
            binary += ".exe"
        else:
            raise ValueError("Binary (%s) does not exist" % (binary,))

    return binary


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
    }
    if step == 'build.ArduPlane':
        return util.build_SITL('bin/arduplane', **build_opts)

    if step == 'build.APMrover2':
        return util.build_SITL('bin/ardurover', **build_opts)

    if step == 'build.ArduCopter':
        return util.build_SITL('bin/arducopter', **build_opts)

    if step == 'build.AntennaTracker':
        return util.build_SITL('bin/antennatracker', **build_opts)

    if step == 'build.Helicopter':
        return util.build_SITL('bin/arducopter-heli', **build_opts)
    
    if step == 'build.ArduSub':
        return util.build_SITL('bin/ardusub', **build_opts)

    binary = binary_path(step, debug=opts.debug)

    if step.startswith("default"):
        vehicle = step[8:]
        return get_default_params(vehicle, binary)

    fly_opts = {
        "viewerip": opts.viewerip,
        "use_map": opts.map,
        "valgrind": opts.valgrind,
        "gdb": opts.gdb,
        "gdbserver": opts.gdbserver,
    }
    if opts.speedup is not None:
        fly_opts.speedup = opts.speedup

    if step == 'fly.ArduCopter':
        return arducopter.fly_ArduCopter(binary, frame=opts.frame, **fly_opts)

    if step == 'fly.CopterAVC':
        return arducopter.fly_CopterAVC(binary, **fly_opts)

    if step == 'fly.ArduPlane':
        return arduplane.fly_ArduPlane(binary, **fly_opts)

    if step == 'fly.QuadPlane':
        return quadplane.fly_QuadPlane(binary, **fly_opts)

    if step == 'drive.APMrover2':
        return apmrover2.drive_APMrover2(binary, frame=opts.frame, **fly_opts)

    if step == 'dive.ArduSub':
        return ardusub.dive_ArduSub(binary, **fly_opts)

    if step == 'build.All':
        return build_all()

    if step == 'build.Binaries':
        return build_binaries()

    if step == 'build.DevRelease':
        return build_devrelease()

    if step == 'build.Examples':
        return build_examples()

    if step == 'build.Parameters':
        return build_parameters()

    if step == 'convertgpx':
        return convert_gpx()

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
        self.githash = util.run_cmd('git rev-parse HEAD', output=True, directory=util.reltopdir('.')).strip()
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


def write_fullresults():
    """Write out full results set."""
    global results
    results.addglob("Google Earth track", '*.kmz')
    results.addfile('Full Logs', 'autotest-output.txt')
    results.addglob('DataFlash Log', '*-log.bin')
    results.addglob("MAVLink log", '*.tlog')
    results.addglob("GPX track", '*.gpx')

    # results common to all vehicles:
    vehicle_files = [ ('{vehicle} build log', '{vehicle}.txt'),
                      ('{vehicle} code size', '{vehicle}.sizes.txt'),
                      ('{vehicle} stack sizes', '{vehicle}.framesizes.txt'),
                      ('{vehicle} defaults', 'default_params/{vehicle}-defaults.parm'),
                      ('{vehicle} core', '{vehicle}.core'),
                      ('{vehicle} ELF', '{vehicle}.elf'),
    ]
    vehicle_globs = [('{vehicle} log', '{vehicle}-*.BIN'),
    ]
    for vehicle in 'ArduPlane','ArduCopter','APMrover2','AntennaTracker', 'ArduSub':
        subs = { 'vehicle': vehicle }
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
        util.run_cmd('/bin/cp A*/A*.elf %s' % buildlogs_dirpath(),
                     directory=util.reltopdir('.'))

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
                results.add(step, '<span class="passed-text">PASSED</span>', time.time() - t1)
                print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
                check_logs(step)
            else:
                print(">>>> FAILED STEP: %s at %s" % (step, time.asctime()))
                passed = False
                failed.append(step)
                results.add(step, '<span class="failed-text">FAILED</span>', time.time() - t1)
        except Exception as msg:
            passed = False
            failed.append(step)
            print(">>>> FAILED STEP: %s at %s (%s)" % (step, time.asctime(), msg))
            traceback.print_exc(file=sys.stdout)
            results.add(step, '<span class="failed-text">FAILED</span>', time.time() - t1)
            check_logs(step)
    if not passed:
        print("FAILED %u tests: %s" % (len(failed), failed))

    util.pexpect_close_all()

    write_fullresults()

    return passed

if __name__ == "__main__":
############## main program #############
    os.environ['PYTHONUNBUFFERED'] = '1'

    os.putenv('TMPDIR', util.reltopdir('tmp'))

    parser = optparse.OptionParser("autotest")
    parser.add_option("--skip", type='string', default='', help='list of steps to skip (comma separated)')
    parser.add_option("--list", action='store_true', default=False, help='list the available steps')
    parser.add_option("--viewerip", default=None, help='IP address to send MAVLink and fg packets to')
    parser.add_option("--map", action='store_true', default=False, help='show map')
    parser.add_option("--experimental", default=False, action='store_true', help='enable experimental tests')
    parser.add_option("--timeout", default=3000, type='int', help='maximum runtime in seconds')
    parser.add_option("--speedup", default=None, type='int', help='speedup to run the simulations at')
    parser.add_option("--valgrind", default=False, action='store_true', help='run ArduPilot binaries under valgrind')
    parser.add_option("--gdb", default=False, action='store_true', help='run ArduPilot binaries under gdb')
    parser.add_option("--debug", default=False, action='store_true', help='make built binaries debug binaries')
    parser.add_option("-j", default=None, type='int', help='build CPUs')
    parser.add_option("--frame", type='string', default=None, help='specify frame type')
    parser.add_option("--gdbserver", default=False, action='store_true', help='run ArduPilot binaries under gdbserver')
    parser.add_option("--no-clean", default=False, action='store_true', help='do not clean before building', dest="no_clean")
    parser.add_option("--no-configure", default=False, action='store_true', help='do not configure before building', dest="no_configure")

    opts, args = parser.parse_args()


    steps = [
    'prerequisites',
    'build.All',
    'build.Binaries',
    # 'build.DevRelease',
    'build.Examples',
    'build.Parameters',

    'build.ArduPlane',
    'defaults.ArduPlane',
    'fly.ArduPlane',
    'fly.QuadPlane',

    'build.APMrover2',
    'defaults.APMrover2',
    'drive.APMrover2',

    'build.ArduCopter',
    'defaults.ArduCopter',
    'fly.ArduCopter',

    'build.Helicopter',
    'fly.CopterAVC',

    'build.AntennaTracker',

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
            matches = [step for step in steps if fnmatch.fnmatch(step.lower(), a.lower())]
            if not len(matches):
                print("No steps matched {}".format(a))
                sys.exit(1)
            matched.extend(matches)
        steps = matched

    # skip steps according to --skip option:
    steps_to_run = [ s for s in steps if should_run_step(s) ]

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
