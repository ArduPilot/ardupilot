#!/usr/bin/env python
# APM automatic test suite
# Andrew Tridgell, October 2011

import pexpect, os, sys, shutil, atexit
import optparse, fnmatch, time, glob, traceback

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pysim'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pymavlink'))
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
    dest = util.reltopdir('../buildlogs/%s.defaults.txt' % atype)
    shutil.copy(parmfile, dest)
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    print("Saved defaults for %s to %s" % (atype, dest))
    return True

def dump_logs(atype):
    '''dump DataFlash logs'''
    print("Dumping logs for %s" % atype)
    sil = util.start_SIL(atype, CLI=True)
    logfile = util.reltopdir('../buildlogs/%s.flashlog' % atype)
    log = open(logfile, mode='w')
    mavproxy = util.start_MAVProxy_SIL(atype, setup=True, logfile=log)
    mavproxy.send('\n\n\n')
    print("navigating menus")
    mavproxy.expect(']')
    mavproxy.send("logs\n")
    mavproxy.expect("logs enabled:")
    lognums = []
    i = mavproxy.expect(["No logs", "(\d+) logs"])
    if i == 0:
        numlogs = 0
    else:
        numlogs = int(mavproxy.match.group(1))
    for i in range(numlogs):
        mavproxy.expect("Log (\d+),")
        lognums.append(int(mavproxy.match.group(1)))
    mavproxy.expect("Log]")
    for i in range(numlogs):
        print("Dumping log %u (i=%u)" % (lognums[i], i))
        mavproxy.send("dump %u\n" % lognums[i])
        mavproxy.expect("logs enabled:", timeout=120)
        mavproxy.expect("Log]")
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    log.close()
    print("Saved log for %s to %s" % (atype, logfile))
    return True


def build_all():
    '''run the build_all.sh script'''
    print("Running build_all.sh")
    if util.run_cmd(util.reltopdir('Tools/scripts/build_all.sh'), dir=util.reltopdir('.')) != 0:
        print("Failed build_all.sh")
        return False
    return True


def convert_gpx():
    '''convert any mavlog files to GPX and KML'''
    import glob
    mavlog = glob.glob(util.reltopdir("../buildlogs/*.mavlog"))
    for m in mavlog:
        util.run_cmd(util.reltopdir("../pymavlink/examples/mavtogpx.py") + " --nofixcheck " + m)
        gpx = m + '.gpx'
        kml = m + '.kml'
        util.run_cmd('gpsbabel -i gpx -f %s -o kml,units=m,floating=1,extrude=1 -F %s' % (gpx, kml), checkfail=False)
        util.run_cmd('zip %s.kmz %s.kml' % (m, m), checkfail=False)
    return True


def test_prerequesites():
    '''check we have the right directories and tools to run tests'''
    print("Testing prerequesites")
    util.mkdir_p(util.reltopdir('../buildlogs'))
    return True


############## main program #############
parser = optparse.OptionParser("autotest")
parser.add_option("--skip", type='string', default='', help='list of steps to skip (comma separated)')
parser.add_option("--list", action='store_true', default=False, help='list the available steps')
parser.add_option("--viewerip", default=None, help='IP address to send MAVLink and fg packets to')
parser.add_option("--experimental", default=False, action='store_true', help='enable experimental tests')
parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")

opts, args = parser.parse_args()

if opts.mav10 or os.getenv('MAVLINK10'):
    os.environ['MAVLINK10'] = '1'
    import mavlinkv10 as mavlink
else:
    import mavlink as mavlink

import  arducopter, arduplane

steps = [
    'prerequesites',
    'build1280.ArduPlane',
    'build2560.ArduPlane',
    'build.All',
    'build.ArduPlane',
    'defaults.ArduPlane',
    'fly.ArduPlane',
    'logs.ArduPlane',
    'build1280.ArduCopter',
    'build2560.ArduCopter',
    'build.ArduCopter',
    'defaults.ArduCopter',
    'fly.ArduCopter',
    'logs.ArduCopter',
    'convertgpx',
    ]

skipsteps = opts.skip.split(',')

def skip_step(step):
    '''see if a step should be skipped'''
    for skip in skipsteps:
        if fnmatch.fnmatch(step, skip):
            return True
    return False

def run_step(step):
    '''run one step'''
    if step == "prerequesites":
        return test_prerequesites()

    if step == 'build.ArduPlane':
        return util.build_SIL('ArduPlane')

    if step == 'build.ArduCopter':
        return util.build_SIL('ArduCopter')

    if step == 'build1280.ArduCopter':
        return util.build_AVR('ArduCopter', board='mega')

    if step == 'build2560.ArduCopter':
        return util.build_AVR('ArduCopter', board='mega2560')

    if step == 'build1280.ArduPlane':
        return util.build_AVR('ArduPlane', board='mega')

    if step == 'build2560.ArduPlane':
        return util.build_AVR('ArduPlane', board='mega2560')

    if step == 'defaults.ArduPlane':
        return get_default_params('ArduPlane')

    if step == 'defaults.ArduCopter':
        return get_default_params('ArduCopter')

    if step == 'logs.ArduPlane':
        return dump_logs('ArduPlane')

    if step == 'logs.ArduCopter':
        return dump_logs('ArduCopter')

    if step == 'fly.ArduCopter':
        return arducopter.fly_ArduCopter(viewerip=opts.viewerip)

    if step == 'fly.ArduPlane':
        return arduplane.fly_ArduPlane(viewerip=opts.viewerip)

    if step == 'build.All':
        return build_all()

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

    def add(self, name, result, elapsed):
        '''add a result'''
        self.tests.append(TestResult(name, result, elapsed))

    def addfile(self, name, fname):
        '''add a result file'''
        self.files.append(TestFile(name, fname))

    def addglob(self, name, pattern):
        '''add a set of files'''
        import glob
        for f in glob.glob(util.reltopdir('../buildlogs/%s' % pattern)):
            self.addfile(name, os.path.basename(f))



def write_webresults(results):
    '''write webpage results'''
    sys.path.insert(0, os.path.join(util.reltopdir("../pymavlink/generator")))
    import mavtemplate
    t = mavtemplate.MAVTemplate()
    for h in glob.glob(util.reltopdir('Tools/autotest/web/*.html')):
        html = util.loadfile(h)
        f = open(util.reltopdir("../buildlogs/%s" % os.path.basename(h)), mode='w')
        t.write(f, html, results)
        f.close()
    for f in glob.glob(util.reltopdir('Tools/autotest/web/*.png')):
        shutil.copy(f, util.reltopdir('../buildlogs/%s' % os.path.basename(f)))



def run_tests(steps):
    '''run a list of steps'''

    results = TestResults()

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
            continue
        results.add(step, '<span class="passed-text">PASSED</span>', time.time() - t1)
        print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
    if not passed:
        print("FAILED %u tests: %s" % (len(failed), failed))

    util.pexpect_close_all()

    results.addglob("Google Earth track", '*.kmz')
    results.addfile('Full Logs', 'autotest-output.txt')
    results.addglob('DataFlash Log', '*.flashlog')
    results.addglob("MAVLink log", '*.mavlog')
    results.addglob("GPX track", '*.gpx')
    results.addfile('ArduPlane build log', 'ArduPlane.txt')
    results.addfile('ArduPlane code size', 'ArduPlane.sizes.txt')
    results.addfile('ArduPlane stack sizes', 'ArduPlane.framesizes.txt')
    results.addfile('ArduPlane defaults', 'ArduPlane.defaults.txt')
    results.addfile('ArduCopter build log', 'ArduCopter.txt')
    results.addfile('ArduCopter code size', 'ArduCopter.sizes.txt')
    results.addfile('ArduCopter stack sizes', 'ArduCopter.framesizes.txt')
    results.addfile('ArduCopter defaults', 'ArduCopter.defaults.txt')

    write_webresults(results)

    return passed


lck = util.lock_file(util.reltopdir('../buildlogs/autotest.lck'))
if lck is None:
    print("autotest is locked - exiting")
    sys.exit(0)

atexit.register(util.pexpect_close_all)

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
