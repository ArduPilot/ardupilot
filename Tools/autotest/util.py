# utility code for autotest

import os, pexpect, sys
from subprocess import call, check_call,Popen, PIPE


def relhome(path):
    '''return a path relative to $HOME'''
    return os.path.join(os.getenv('HOME'), path)

def relcwd(path):
    '''return a path relative to $CWD'''
    return os.path.join(os.getcwd(), path)


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
            dir=relcwd('APM/' + atype),
            checkfail=True)

def start_SIL(atype):
    '''launch a SIL instance'''
    ret = pexpect.spawn('tmp/%s.build/%s.elf' % (atype, atype),
                        logfile=sys.stdout, timeout=5)
    ret.expect('Waiting for connection')
    return ret

def start_MAVProxy_SIL(atype, options=''):
    '''launch mavproxy connected to a SIL instance'''
    MAVPROXY = relcwd('MAVProxy/mavproxy.py')
    ret = pexpect.spawn('%s --master=tcp:127.0.0.1:5760 --aircraft=test.%s %s' % (
        MAVPROXY,atype,options),
                        logfile=sys.stdout, timeout=60)
    return ret


def kill(name):
    '''kill a process'''
    run_cmd('killall -9 -q %s' % name, checkfail=False)
