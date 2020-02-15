from __future__ import print_function

import os
import signal
import subprocess
import time

from pysim import util

def run_example(filepath, valgrind=False, gdb=False):
    cmd = []
    if valgrind:
        cmd.append("valgrind")
    if gdb:
        cmd.append("gdb")
    cmd.append(filepath)
    print("Running: (%s)" % str(cmd))
    bob = subprocess.Popen(cmd, stdin=None, close_fds=True)
    retcode = bob.poll()
    time.sleep(10)
    print("pre-kill retcode: %s" % str(retcode))
    if retcode is not None:
        raise ValueError("Process exited before I could kill it (%s)" % str(retcode))
    bob.send_signal(signal.SIGTERM)
    time.sleep(1)
    retcode = bob.poll()
    print("retcode: %s" % str(retcode))
    if retcode is None:
        # if we get this far then we're not going to get a gcda file
        # out of this process for coverage analysis; it has to exit
        # normally, and it hasn't responded to a TERM.
        bob.kill()
        retcode2 = bob.wait()
        print("retcode2: %s" % str(retcode2))
    elif retcode == -15:
        print("process exited with -15, indicating it didn't catch the TERM signal and exit properly")
    elif retcode != 0:
        # note that process could exit with code 0 and we couldn't tell...
        raise ValueError("Process exitted with non-zero exitcode %s" % str(retcode))

    print("Ran: (%s)" % str(cmd))

def run_examples(debug=False, valgrind=False, gdb=False):
    dirpath = util.reltopdir(os.path.join('build', 'linux', 'examples'))

    skip = {
        "BARO_generic": "Most linux computers don't have baros...",
        "RCProtocolDecoder": "This assumes specific hardware is connected",
    }
    for afile in os.listdir(dirpath):
        if afile in skip:
            print("Skipping %s: %s" % (afile, skip[afile]))
            continue
        filepath = os.path.join(dirpath, afile)
        if not os.path.isfile(filepath):
            continue
        run_example(filepath, valgrind=valgrind, gdb=gdb)

    return True
