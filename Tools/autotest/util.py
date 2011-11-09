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
        os.unlink(path)
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

close_list = []

def pexpect_close(p):
    '''close a pexpect child'''
    global close_list
    p.close()
    close_list.remove(p)

def pexpect_close_all():
    '''close all pexpect children'''
    global close_list
    for p in close_list[:]:
        try:
            p.close()
        except Exception:
            pass

def start_SIL(atype, valgrind=True, wipe=False, CLI=False):
    '''launch a SIL instance'''
    global close_list
    cmd=""
    if valgrind and os.path.exists('/usr/bin/valgrind'):
        cmd += 'valgrind -q --log-file=%s-valgrind.log ' % atype
    cmd += reltopdir('tmp/%s.build/%s.elf' % (atype, atype))
    if wipe:
        cmd += ' -w'
    if CLI:
        cmd += ' -s'
    ret = pexpect.spawn(cmd, logfile=sys.stdout, timeout=5)
    close_list.append(ret)
    ret.expect('Waiting for connection')
    return ret

def start_MAVProxy_SIL(atype, aircraft=None, setup=False, master='tcp:127.0.0.1:5760',
                       options=None, logfile=sys.stdout):
    '''launch mavproxy connected to a SIL instance'''
    global close_list
    MAVPROXY = reltopdir('../MAVProxy/mavproxy.py')
    cmd = MAVPROXY + ' --master=%s' % master
    if setup:
        cmd += ' --setup'
    if aircraft is None:
        aircraft = 'test.%s' % atype
    cmd += ' --aircraft=%s' % aircraft
    if options is not None:
        cmd += ' ' + options
    ret = pexpect.spawn(cmd, logfile=logfile, timeout=60)
    close_list.append(ret)
    return ret


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

def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)
