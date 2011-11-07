#!/usr/bin/env python
# APM automatic test suite
# Andrew Tridgell, October 2011

import pexpect, os, util, sys, shutil, arducopter
import optparse, fnmatch

os.putenv('TMPDIR', util.reltopdir('tmp'))

def get_default_params(atype):
    '''get default parameters'''
    util.rmfile('eeprom.bin')
    sil = util.start_SIL(atype)
    mavproxy = util.start_MAVProxy_SIL(atype)
    idx = mavproxy.expect(['Please Run Setup', 'Saved [0-9]+ parameters to (\S+)'])
    if idx == 0:
        # we need to restart it after eeprom erase
        mavproxy.close()
        sil.close()
        sil = util.start_SIL(atype)
        mavproxy = util.start_MAVProxy_SIL(atype)
        idx = mavproxy.expect('Saved [0-9]+ parameters to (\S+)')
    parmfile = mavproxy.match.group(1)
    dest = util.reltopdir('../buildlogs/%s.defaults.txt' % atype)
    shutil.copy(parmfile, dest)
    mavproxy.close()
    sil.close()
    print("Saved defaults for %s to %s" % (atype, dest))

def test_prerequesites():
    '''check we have the right directories and tools to run tests'''
    print("Testing prerequesites")
    util.mkdir_p(util.reltopdir('../buildlogs'))
    if not os.path.exists(util.reltopdir('../HILTest/hil_quad.py')):
        print('''
You need to install HILTest in %s

You can get it from git://git.samba.org/tridge/UAV/HILTest.git

        ''' % util.reltopdir('../HILTest'))
    

############## main program #############
parser = optparse.OptionParser("autotest")
parser.add_option("--skip", type='string', default='', help='list of steps to skip (comma separated)')
parser.add_option("--list", action='store_true', default=False, help='list the available steps')

opts, args = parser.parse_args()

steps = [
    'build.ArduPlane',
    'build.ArduCopter',
    'defaults.ArduPlane',
    'defaults.ArduCopter',
    'fly.ArduCopter'
    ]

skipsteps = opts.skip.split(',')

def skip_step(step):
    '''see if a step should be skipped'''
    for skip in skipsteps:
        if fnmatch.fnmatch(step, skip):
            return True
    return False

# kill any previous instance
util.kill('ArduCopter.elf')
util.kill('ArduPilot.elf')

test_prerequesites()

for step in steps:
    if skip_step(step):
        continue
    if step == 'build.ArduPlane':
        util.build_SIL('ArduPlane')
    elif step == 'build.ArduCopter':
        util.build_SIL('ArduCopter')
    elif step == 'defaults.ArduPlane':
        get_default_params('ArduPlane')
    elif step == 'defaults.ArduCopter':
        get_default_params('ArduCopter')
    elif step == 'fly.ArduCopter':
        arducopter.fly_ArduCopter()
    else:
        raise RuntimeError("Unknown step %s" % step)

util.kill('ArduCopter.elf')
util.kill('ArduPilot.elf')
