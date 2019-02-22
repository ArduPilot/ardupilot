from __future__ import print_function

import os
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
    time.sleep(10)
    bob.kill()
    bob.wait()
    if bob.returncode is None:
        raise ValueError("Unable to kill subprocess")
    print("returncode: %u" % (bob.returncode))
    if bob.returncode != -9:
        raise ValueError("Process exitted before I got to kill it (exit code=%u)" % bob.returncode)
    print("returncode2: %u" % (bob.returncode))

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
