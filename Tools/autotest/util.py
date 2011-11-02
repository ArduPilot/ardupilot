# utility code for autotest

import os, pexpect, sys, time
from subprocess import call, check_call,Popen, PIPE


def topdir():
    '''return top of git tree where autotest is running from'''
    d = os.path.dirname(os.path.realpath(__file__))
    assert(os.path.basename(d)=='autotest')
    d = os.path.dirname(d)
    assert(os.path.basename(d)=='Tools')
    d = os.path.dirname(d)
    return d

def reltopdir(path):
    '''return a path relative to topdir()'''
    return os.path.normpath(os.path.join(topdir(), path))


def run_cmd(cmd, dir=".", show=False, output=False, checkfail=True):
    '''run a shell command'''
    if show:
        print("Running: '%s' in '%s'" % (cmd, dir))
    if output:
        return Popen([cmd], shell=True, stdout=PIPE, cwd=dir).communicate()[0]
    elif checkfail:
        return check_call(cmd, shell=True, cwd=dir)
    else:
        return call(cmd, shell=True, cwd=dir)

def rmfile(path):
    '''remove a file if it exists'''
    try:
        os.unlink('eeprom.bin')
    except Exception:
        pass

def deltree(path):
    '''delete a tree of files'''
    run_cmd('rm -rf %s' % path)
    
    

def build_SIL(atype):
    '''build desktop SIL'''
    run_cmd("make -f ../libraries/Desktop/Makefile.desktop clean hil",
            dir=reltopdir(atype),
            checkfail=True)

def start_SIL(atype):
    '''launch a SIL instance'''
    ret = pexpect.spawn(reltopdir('tmp/%s.build/%s.elf' % (atype, atype)),
                        logfile=sys.stdout, timeout=5)
    ret.expect('Waiting for connection')
    return ret

def start_MAVProxy_SIL(atype, options=''):
    '''launch mavproxy connected to a SIL instance'''
    MAVPROXY = reltopdir('../MAVProxy/mavproxy.py')
    ret = pexpect.spawn('%s --master=tcp:127.0.0.1:5760 --aircraft=test.%s %s' % (
        MAVPROXY,atype,options),
                        logfile=sys.stdout, timeout=60)
    return ret


def kill(name):
    '''kill a process'''
    run_cmd('killall -9 -q %s' % name, checkfail=False)


def expect_setup_callback(e, callback):
    '''setup a callback that is called once a second while waiting for
       patterns'''
    def _expect_callback(pattern, timeout=e.timeout):
        tstart = time.time()
        while time.time() < tstart + timeout:
            try:
                ret = e.expect_saved(pattern, timeout=1)
                return ret
            except pexpect.TIMEOUT:
                e.expect_user_callback(e)
                pass
        raise pexpect.TIMEOUT

    e.expect_user_callback = callback
    e.expect_saved = e.expect
    e.expect = _expect_callback
