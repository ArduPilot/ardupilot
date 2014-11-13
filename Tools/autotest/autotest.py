#!/usr/bin/env python
# APM automatic test suite
# Andrew Tridgell, October 2011

import pexpect, os, sys, shutil, atexit
import optparse, fnmatch, time, glob, traceback, signal

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pysim'))

import util

os.environ['PYTHONUNBUFFERED'] = '1'

os.putenv('TMPDIR', util.reltopdir('tmp'))

def get_default_params(atype):
    '''get default parameters'''
    sil = util.start_SIL(atype, wipe=True)
    mavproxy = util.start_MAVProxy_SIL(atype)
    print("Dumping defaults")
    idx = mavproxy.expect(['Please Run Setup', 'Saved [0-9]+ parameters to (\S+)'])
    if idx == 0:
        # we need to restart it after eeprom erase
        util.pexpect_close(mavproxy)
        util.pexpect_close(sil)
        sil = util.start_SIL(atype)
        mavproxy = util.start_MAVProxy_SIL(atype)
        idx = mavproxy.expect('Saved [0-9]+ parameters to (\S+)')
    parmfile = mavproxy.match.group(1)
    dest = 'buildlogs/%s-defaults.parm' % atype
    shutil.copy(parmfile, dest)
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    print("Saved defaults for %s to %s" % (atype, dest))
    return True

def build_all():
    '''run the build_all.sh script'''
    print("Running build_all.sh")
    if util.run_cmd(util.reltopdir('Tools/scripts/build_all.sh'), dir=util.reltopdir('.')) != 0:
        print("Failed build_all.sh")
        return False
    return True

def build_binaries():
    '''run the build_binaries.sh script'''
    print("Running build_binaries.sh")
    import shutil
    # copy the script as it changes git branch, which can change the script while running
    orig=util.reltopdir('Tools/scripts/build_binaries.sh')
    copy=util.reltopdir('./build_binaries.sh')
    shutil.copyfile(orig, copy)
    shutil.copymode(orig, copy)
    if util.run_cmd(copy, dir=util.reltopdir('.')) != 0:
        print("Failed build_binaries.sh")
        return False
    return True

def build_devrelease():
    '''run the build_devrelease.sh script'''
    print("Running build_devrelease.sh")
    import shutil
    # copy the script as it changes git branch, which can change the script while running
    orig=util.reltopdir('Tools/scripts/build_devrelease.sh')
    copy=util.reltopdir('./build_devrelease.sh')
    shutil.copyfile(orig, copy)
    shutil.copymode(orig, copy)
    if util.run_cmd(copy, dir=util.reltopdir('.')) != 0:
        print("Failed build_devrelease.sh")
        return False
    return True

def build_examples():
    '''run the build_examples.sh script'''
    print("Running build_examples.sh")
    if util.run_cmd(util.reltopdir('Tools/scripts/build_examples.sh'), dir=util.reltopdir('.')) != 0:
        print("Failed build_examples.sh")
        return False
    return True

def build_parameters():
    '''run the param_parse.py script'''
    print("Running param_parse.py")
    if util.run_cmd(util.reltopdir('Tools/autotest/param_metadata/param_parse.py'), dir=util.reltopdir('.')) != 0:
        print("Failed param_parse.py")
        return False
    return True


def convert_gpx():
    '''convert any tlog files to GPX and KML'''
    import glob
    mavlog = glob.glob("buildlogs/*.tlog")
    for m in mavlog:
        util.run_cmd(util.reltopdir("../mavlink/pymavlink/tools/mavtogpx.py") + " --nofixcheck " + m)
        gpx = m + '.gpx'
        kml = m + '.kml'
        util.run_cmd('gpsbabel -i gpx -f %s -o kml,units=m,floating=1,extrude=1 -F %s' % (gpx, kml), checkfail=False)
        util.run_cmd('zip %s.kmz %s.kml' % (m, m), checkfail=False)
        util.run_cmd("mavflightview.py --imagefile=%s.png %s" % (m,m))
    return True


def test_prerequisites():
    '''check we have the right directories and tools to run tests'''
    print("Testing prerequisites")
    util.mkdir_p('buildlogs')
    return True

def alarm_handler(signum, frame):
    '''handle test timeout'''
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

############## main program #############
parser = optparse.OptionParser("autotest")
parser.add_option("--skip", type='string', default='', help='list of steps to skip (comma separated)')
parser.add_option("--list", action='store_true', default=False, help='list the available steps')
parser.add_option("--viewerip", default=None, help='IP address to send MAVLink and fg packets to')
parser.add_option("--map", action='store_true', default=False, help='show map')
parser.add_option("--experimental", default=False, action='store_true', help='enable experimental tests')
parser.add_option("--timeout", default=3000, type='int', help='maximum runtime in seconds')

opts, args = parser.parse_args()

import  arducopter, arduplane, apmrover2

steps = [
    'prerequisites',
    'build.All',
    'build.Binaries',
    'build.DevRelease',
    'build.Examples',
    'build.Parameters',

    'build2560.ArduPlane',
    'build.ArduPlane',
    'defaults.ArduPlane',
    'fly.ArduPlane',

    'build2560.APMrover2',
    'build.APMrover2',
    'defaults.APMrover2',
    'drive.APMrover2',

    'build2560.ArduCopter',
    'build.ArduCopter',
    'defaults.ArduCopter',
    'fly.ArduCopter',
    'fly.CopterAVC',

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

def skip_step(step):
    '''see if a step should be skipped'''
    for skip in skipsteps:
        if fnmatch.fnmatch(step.lower(), skip.lower()):
            return True
    return False

def run_step(step):
    '''run one step'''

    # remove old logs
    util.run_cmd('/bin/rm -f logs/*.BIN logs/LASTLOG.TXT')

    if step == "prerequisites":
        return test_prerequisites()

    if step == 'build.ArduPlane':
        return util.build_SIL('ArduPlane')

    if step == 'build.APMrover2':
        return util.build_SIL('APMrover2')

    if step == 'build.ArduCopter':
        return util.build_SIL('ArduCopter')

    if step == 'build2560.ArduCopter':
        return util.build_AVR('ArduCopter', board='mega2560')

    if step == 'build2560.ArduPlane':
        return util.build_AVR('ArduPlane', board='mega2560')

    if step == 'build2560.APMrover2':
        return util.build_AVR('APMrover2', board='mega2560')

    if step == 'defaults.ArduPlane':
        return get_default_params('ArduPlane')

    if step == 'defaults.ArduCopter':
        return get_default_params('ArduCopter')

    if step == 'defaults.APMrover2':
        return get_default_params('APMrover2')

    if step == 'fly.ArduCopter':
        return arducopter.fly_ArduCopter(viewerip=opts.viewerip, map=opts.map)

    if step == 'fly.CopterAVC':
        return arducopter.fly_CopterAVC(viewerip=opts.viewerip, map=opts.map)

    if step == 'fly.ArduPlane':
        return arduplane.fly_ArduPlane(viewerip=opts.viewerip, map=opts.map)

    if step == 'drive.APMrover2':
        return apmrover2.drive_APMrover2(viewerip=opts.viewerip, map=opts.map)

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
    '''test result class'''
    def __init__(self, name, result, elapsed):
        self.name = name
        self.result = result
        self.elapsed = "%.1f" % elapsed

class TestFile(object):
    '''test result file'''
    def __init__(self, name, fname):
        self.name = name
        self.fname = fname

class TestResults(object):
    '''test results class'''
    def __init__(self):
        self.date = time.asctime()
        self.githash = util.run_cmd('git rev-parse HEAD', output=True, dir=util.reltopdir('.')).strip()
        self.tests = []
        self.files = []
        self.images = []

    def add(self, name, result, elapsed):
        '''add a result'''
        self.tests.append(TestResult(name, result, elapsed))

    def addfile(self, name, fname):
        '''add a result file'''
        self.files.append(TestFile(name, fname))

    def addimage(self, name, fname):
        '''add a result image'''
        self.images.append(TestFile(name, fname))

    def addglob(self, name, pattern):
        '''add a set of files'''
        import glob
        for f in glob.glob('buildlogs/%s' % pattern):
            self.addfile(name, os.path.basename(f))

    def addglobimage(self, name, pattern):
        '''add a set of images'''
        import glob
        for f in glob.glob('buildlogs/%s' % pattern):
            self.addimage(name, os.path.basename(f))



def write_webresults(results):
    '''write webpage results'''
    from pymavlink.generator import mavtemplate
    t = mavtemplate.MAVTemplate()
    for h in glob.glob(util.reltopdir('Tools/autotest/web/*.html')):
        html = util.loadfile(h)
        f = open("buildlogs/%s" % os.path.basename(h), mode='w')
        t.write(f, html, results)
        f.close()
    for f in glob.glob(util.reltopdir('Tools/autotest/web/*.png')):
        shutil.copy(f, 'buildlogs/%s' % os.path.basename(f))

def write_fullresults():
    '''write out full results set'''
    global results
    results.addglob("Google Earth track", '*.kmz')
    results.addfile('Full Logs', 'autotest-output.txt')
    results.addglob('DataFlash Log', '*-log.bin')
    results.addglob("MAVLink log", '*.tlog')
    results.addglob("GPX track", '*.gpx')
    results.addfile('ArduPlane build log', 'ArduPlane.txt')
    results.addfile('ArduPlane code size', 'ArduPlane.sizes.txt')
    results.addfile('ArduPlane stack sizes', 'ArduPlane.framesizes.txt')
    results.addfile('ArduPlane defaults', 'ArduPlane-defaults.parm')
    results.addglob("ArduPlane log", 'ArduPlane-*.BIN')
    results.addglob("ArduPlane core", 'ArduPlane.core')
    results.addglob("ArduPlane ELF", 'ArduPlane.elf')
    results.addfile('ArduCopter build log', 'ArduCopter.txt')
    results.addfile('ArduCopter code size', 'ArduCopter.sizes.txt')
    results.addfile('ArduCopter stack sizes', 'ArduCopter.framesizes.txt')
    results.addfile('ArduCopter defaults', 'ArduCopter-defaults.parm')
    results.addglob("ArduCopter log", 'ArduCopter-*.BIN')
    results.addglob("ArduCopter core", 'ArduCopter.core')
    results.addglob("ArduCopter elf", 'ArduCopter.elf')
    results.addglob("CopterAVC log", 'CopterAVC-*.BIN')
    results.addglob("CopterAVC core", 'CopterAVC.core')
    results.addfile('APMrover2 build log', 'APMrover2.txt')
    results.addfile('APMrover2 code size', 'APMrover2.sizes.txt')
    results.addfile('APMrover2 stack sizes', 'APMrover2.framesizes.txt')
    results.addfile('APMrover2 defaults', 'APMrover2-defaults.parm')
    results.addglob("APMrover2 log", 'APMrover2-*.BIN')
    results.addglob("APMrover2 core", 'APMrover2.core')
    results.addglob("APMrover2 ELF", 'APMrover2.elf')
    results.addglob('APM:Libraries documentation', 'docs/libraries/index.html')
    results.addglob('APM:Plane documentation', 'docs/ArduPlane/index.html')
    results.addglob('APM:Copter documentation', 'docs/ArduCopter/index.html')
    results.addglob('APM:Rover documentation', 'docs/APMrover2/index.html')
    results.addglobimage("Flight Track", '*.png')

    write_webresults(results)


results = TestResults()

def check_logs(step):
    '''check for log files from a step'''
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
        newname = "buildlogs/%s-%s" % (vehicle, bname)
        print("Renaming %s to %s" % (log, newname))
        os.rename(log, newname)

    corefile = "core"
    if os.path.exists(corefile):
        newname = "buildlogs/%s.core" % vehicle
        print("Renaming %s to %s" % (corefile, newname))
        os.rename(corefile, newname)
        util.run_cmd('/bin/cp A*/A*.elf ../buildlogs', dir=util.reltopdir('.'))
        
def run_tests(steps):
    '''run a list of steps'''
    global results

    passed = True
    failed = []
    for step in steps:
        util.pexpect_close_all()
        if skip_step(step):
            continue

        t1 = time.time()
        print(">>>> RUNNING STEP: %s at %s" % (step, time.asctime()))
        try:
            if not run_step(step):
                print(">>>> FAILED STEP: %s at %s" % (step, time.asctime()))
                passed = False
                failed.append(step)
                results.add(step, '<span class="failed-text">FAILED</span>', time.time() - t1)
                continue
        except Exception, msg:
            passed = False
            failed.append(step)
            print(">>>> FAILED STEP: %s at %s (%s)" % (step, time.asctime(), msg))
            traceback.print_exc(file=sys.stdout)
            results.add(step, '<span class="failed-text">FAILED</span>', time.time() - t1)
            check_logs(step)
            continue
        results.add(step, '<span class="passed-text">PASSED</span>', time.time() - t1)
        print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
        check_logs(step)
    if not passed:
        print("FAILED %u tests: %s" % (len(failed), failed))

    util.pexpect_close_all()

    write_fullresults()

    return passed


util.mkdir_p('buildlogs')

lck = util.lock_file('buildlogs/autotest.lck')
if lck is None:
    print("autotest is locked - exiting")
    sys.exit(0)

atexit.register(util.pexpect_close_all)

if len(args) > 0:
    # allow a wildcard list of steps
    matched = []
    for a in args:
        for s in steps:
            if fnmatch.fnmatch(s.lower(), a.lower()):
                matched.append(s)
    steps = matched

try:
    if not run_tests(steps):
        sys.exit(1)
except KeyboardInterrupt:
    util.pexpect_close_all()
    sys.exit(1)
except Exception:
    # make sure we kill off any children
    util.pexpect_close_all()
    raise
