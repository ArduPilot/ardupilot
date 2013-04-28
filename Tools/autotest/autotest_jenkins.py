#!/usr/bin/env python
# APM automatic test suite
# Andrew Tridgell, October 2011

import pexpect, os, sys, shutil, atexit


sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pysim'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'mavlink', 'pymavlink'))

import optparse, fnmatch, time, glob, traceback, signal, util, time, math, common
from common import *


import mavutil, mavwp, random
import arduplane, arducopter

# Defaults
HOME=mavutil.location(-35.362938,149.165085,584,270)

# ArduCopter defaults
FRAME='+'
homeloc = None
num_wp = 0

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

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
    sil = util.start_SIL(atype)
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

def convert_gpx():
    '''convert any tlog files to GPX and KML'''
    import glob
    mavlog = glob.glob(util.reltopdir("../buildlogs/*.tlog"))
    for m in mavlog:
        util.run_cmd(util.reltopdir("../mavlink/pymavlink/examples/mavtogpx.py") + " --nofixcheck " + m)
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


def alarm_handler(signum, frame):
    '''handle test timeout'''
    global results, opts
    try:
        results.add('TIMEOUT', '<span class="failed-text">FAILED</span>', opts.timeout)
        util.pexpect_close_all()
        results.addfile('Full Logs', 'autotest-output.txt')
        results.addglob('DataFlash Log', '*.flashlog')
        results.addglob("MAVLink log", '*.tlog')
        results.addfile('ArduPlane build log', 'ArduPlane.txt')
        results.addfile('ArduPlane defaults', 'ArduPlane.defaults.txt')
        results.addfile('ArduCopter build log', 'ArduCopter.txt')
        results.addfile('ArduCopter defaults', 'ArduCopter.defaults.txt')
        write_webresults(results)
        os.killpg(0, signal.SIGKILL)
    except Exception:
        pass
    sys.exit(1)

import  arducopter, arduplane

class Test(object):
    ''' unit test class '''
    def __init__(self, name, path, test):
        self.name = name
        self.path = path
        self.test = test
        self.result = False
        self.log = ''


def fly_ArduCopter_scripted(testname):
    '''fly ArduCopter in SIL

    '''
    global homeloc

    sim_cmd = util.reltopdir('Tools/autotest/pysim/sim_multicopter.py') + ' --frame=%s --rate=400 --home=%f,%f,%u,%u' % (
        FRAME, HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sim_cmd += ' --wind=6,45,.3'

    sil = util.start_SIL('ArduCopter', wipe=True)
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send('param set SYSID_THISMAV %u\n' % random.randint(100, 200))
    mavproxy.send("param load %s/ArduCopter.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    sil = util.start_SIL('ArduCopter', height=HOME.alt)
    sim = pexpect.spawn(sim_cmd, logfile=sys.stdout, timeout=10)
    sim.delaybeforesend = 0
    util.pexpect_autoclose(sim)
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter --streamrate=5'

    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduCopter-test.tlog")
    print("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    os.link(logfile, buildlog)

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, arducopter.expect_callback)

    expect_list_clear()
    expect_list_extend([sim, sil, mavproxy])

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception, msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)

    failed = False
    e = 'None'
    try:
        mav.wait_heartbeat()
        arducopter.setup_rc(mavproxy)
        homeloc = mav.location()

        print("# Executing test - " + testname)
        if not test_suite[testname].test(mavproxy, mav):
            print("Test script failed")
            failed = True

    except pexpect.TIMEOUT, e:
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    util.pexpect_close(sim)

    if os.path.exists('ArduCopter-valgrind.log'):
        os.chmod('ArduCopter-valgrind.log', 0644)
        shutil.copy("ArduCopter-valgrind.log", util.reltopdir("../buildlogs/ArduCopter-valgrind.log"))

    if failed:
        print("FAILED: %s" % e)
        return False
    return True


class TestResult(object):
    '''test result class'''
    def __init__(self, name, result="", elapsed=0):
        self.name = name
        self.success = False
        self.result = result
        self.elapsed = "%1.f" % elapsed

class TestFile(object):
    '''test result file'''
    def __init__(self, name, fname):
        self.name = name
        self.fname = fname

class TestResults(object):
    '''test results class'''
    def __init__(self):
        self.date = time.asctime()
        self.isotime = time.strftime("%Y-%m-%dT%H:%M:%S")
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

def write_XMLresults(atype, results):
    '''write XML JUnit results'''
    sys.path.insert(0, os.path.join(util.reltopdir("../mavlink/pymavlink/generator")))
    import mavtemplate
    t = mavtemplate.MAVTemplate()
    for x in glob.glob(util.reltopdir('Tools/autotest/junit.xml')):
        junit_xml = util.loadfile(x)
        f = open(util.reltopdir("../buildlogs/%s-%s" % (atype, os.path.basename(x))), mode='w')
        t.write(f, junit_xml, results)
        f.close()

def write_webresults(results):
    '''write webpage results'''
    sys.path.insert(0, os.path.join(util.reltopdir("../mavlink/pymavlink/generator")))
    import mavtemplate
    t = mavtemplate.MAVTemplate()
    for h in glob.glob(util.reltopdir('Tools/autotest/web/*.html')):
        html = util.loadfile(h)
        f = open(util.reltopdir("../buildlogs/%s" % os.path.basename(h)), mode='w')
        t.write(f, html, results)
        f.close()
    for f in glob.glob(util.reltopdir('Tools/autotest/web/*.png')):
        shutil.copy(f, util.reltopdir('../buildlogs/%s' % os.path.basename(f)))

results = TestResults()
unit_test_results = TestResults()

def run_tests(tests):
    '''run a list of tests'''
    global results, unit_test_results

    passed = True
    failed = []
    for testname in tests:
        util.pexpect_close_all()
        testcase = TestResult(testname)
        testcase.success = False
 
        t1 = time.time()
        print(">>>> RUNNING test: %s at %s" % (testname, time.asctime()))
        try:
            if not fly_ArduCopter_scripted(testname):
                print(">>>> FAILED test: %s at %s" % (testname, time.asctime()))
                passed = False
                testcase.elapsed = "%.1f" % (time.time() - t1)   
                testcase.result = '''
                    <testcase classname="ardupilot-mega-autotest" 
                        name=\"%s\" time=\"%s\">
                        <failure type=\"test failure\">
                            %s
                        </failure>
                    </testcase>
                    ''' %  (testcase.name, testcase.elapsed, "failed")
                unit_test_results.tests.append(testcase)
                print testcase.result
                failed.append(testname)
                continue
        except Exception, msg:
            testcase.elapsed = "%.1f" % (time.time() - t1)   
            passed = False
            print(">>>> FAILED test: %s at %s (%s)" % (testname, time.asctime(), msg))
            traceback.print_exc(file=sys.stdout)
            testcase.result = '''
                    <testcase classname="ardupilot-mega-autotest" 
                        name=\"%s\" time=\"%s\">
                        <error type=\"Test error\">
                            %s
                        </error>
                    </testcase>
                    ''' % (testcase.name, testcase.elapsed, traceback.format_exc() )
            print testcase.result
            unit_test_results.tests.append(testcase)
            failed.append(testcase.name)
            continue
        
        #success
        testcase.elapsed = "%.1f" % (time.time() - t1)
        testcase.success =  True
        testcase.result = '''
                    <testcase classname="ardupilot-mega-autotest" 
                        name=\"%s\" time=\"%s\">
                    </testcase>
                    ''' % (testcase.name, testcase.elapsed )
        unit_test_results.tests.append(testcase)
        print(">>>> PASSED test: %s at %s" % (testname, time.asctime()))
        print testcase.result
    
    dump_logs('ArduCopter')
    convert_gpx()
    
    if not passed:
        print("FAILED %u tests: %s" % (len(failed), failed))

    util.pexpect_close_all()

    results.addglob("Google Earth track", '*.kmz')
    results.addfile('Full Logs', 'autotest-output.txt')
    results.addglob('DataFlash Log', '*.flashlog')
    results.addglob("MAVLink log", '*.tlog')
    results.addglob("GPX track", '*.gpx')
    
    unit_test_results.num_tests = len(results.tests)
    unit_test_results.num_failures = len(failed)

    write_XMLresults("ArduCopter", unit_test_results)
    write_webresults(results)

    # individual test results in XML. Always True 
    return True

############## main program #############
parser = optparse.OptionParser(sys.argv[0])

parser.add_option("--list", action='store_true', default=False, help='list the available tests (comma separated, can be used as TESTLIST')
parser.add_option("--run", type='string', default='', metavar="TESTLIST", help='list of tests to run (comma separated)' )
parser.add_option("--rungroup", type='string', default='', metavar="TESTLIST", help='list of test groups to run (comma separated)' )
parser.add_option("--skip", type='string', default='', metavar="TESTLIST", help='list of tests to skip (comma separated)')
parser.add_option("--skipgroup", type='string', default='', metavar="TESTLIST", help='list of test groups to skip (comma separated)')


opts, args = parser.parse_args()

#### Load test modules  ####
print '>>>>>>    Loading ArduCopter TEST MODULES'

import glob

test_suite = {}
test_groups = {}

test_base_path = 'APM/Tools/autotest/apm_unit_tests/'
print test_base_path
print os.path.dirname(os.path.realpath(__file__))

test_group_names = glob.glob(test_base_path+'*')

for gname in test_group_names:
    sys.path.insert(0, gname)
    gname = os.path.basename(gname)
    gtests = glob.glob(test_base_path + gname + '/*.py')
    if gtests:
        test_groups[gname] = []
        for tmodule in gtests:
            tname = os.path.basename(tmodule)
            if (tname.endswith('.py')):
                tname = (tname[:-3]).strip() # chop off the .py
                test_groups[gname].append(tname)
                test_suite[tname.strip()] = tmodule
      
alltests = test_suite.keys()
allgroups = test_groups.keys()
  
if (len(alltests) == 0):
    print "Error: no test modules selected"
    exit(1)

                                         
if opts.list:
    print "\n\nFound %s modules in %s groups" % ((len(alltests), len(allgroups)))

    print "\n\nAvailable tests: "
    print ', '.join(alltests)
    print "\n\nAvailable groups: "
    for g in allgroups:
        print g + " --- " + ','.join(test_groups[g])
    
if opts.rungroup:
    for group in opts.rungroup.split(','):
        opts.run += ','.join(test_groups[group])
     
if opts.skipgroup:
    for group in opts.skipgroup.split(','):
        opts.skip += ','.join(test_groups[group])


runtests = []
print "Selected: " + opts.run
print "Skipping: " + opts.skip

if opts.run:
    for runtest in opts.run.split(','):
        runtests.append(runtest.strip())
        print "Adding test - %s" % runtest
else:
    runtests = alltests 
        
finaltests = []
if opts.skip:
    skiptests = opts.skip.split(',')
    print "Skipping tests: " + ','.join(skiptests)
    for runtest in runtests:
        if runtest not in skiptests:
            finaltests.append(runtest)
        else:
            print "Skipping test - %s" % runtest
else:
    finaltests = runtests

if (len(finaltests) > 0):
    print "\n\n***\nLoading selected tests: " + ','.join(finaltests)
else:
    print "Error: no test modules selected"
    exit(1)

for tname in finaltests:
    print "Loading ArduCopter test module: " + tname
    try:
        mpath = test_suite[tname]
        m = __import__ (tname)
        unit_test = getattr(m,'unit_test')
        
        test_module = Test(tname, mpath, unit_test)
        test_suite[tname] = test_module
        
    except Exception:
        raise

print '>>>>>>    Running TEST MODULES'

atexit.register(util.pexpect_close_all)

try:
    run_tests(finaltests)
    sys.exit(0)
except KeyboardInterrupt:
    util.pexpect_close_all()
    sys.exit(1)
except Exception:
    # make sure we kill off any children
    util.pexpect_close_all()
    raise
