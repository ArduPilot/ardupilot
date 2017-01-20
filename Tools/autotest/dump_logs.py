#!/usr/bin/env python
"""
 dump flash logs from SITL
 Andrew Tridgell, April 2013
"""
from __future__ import print_function
import optparse
import os
import sys

from pysim import util


############## main program #############
parser = optparse.OptionParser(sys.argv[0])

parser.add_option("--cli", action='store_true', default=False, help='put us in the CLI menu in logs')

opts, args = parser.parse_args()

os.environ['PYTHONUNBUFFERED'] = '1'


def dump_logs(atype):
    """Dump DataFlash logs."""
    logfile = '%s.log' % atype
    print("Dumping logs for %s to %s" % (atype, logfile))
    sitl = util.start_SITL(atype)
    log = open(logfile, mode='w')
    mavproxy = util.start_MAVProxy_SITL(atype, setup=True, logfile=log)
    mavproxy.send('\n\n\n')
    print("navigating menus")
    mavproxy.expect(']')
    mavproxy.send("logs\n")
    if opts.cli:
        mavproxy.interact()
        return
    mavproxy.expect("logs enabled:")
    lognums = []
    i = mavproxy.expect(["No logs", "(\d+) logs"])
    if i == 0:
        numlogs = 0
    else:
        numlogs = int(mavproxy.match.group(1))
    for i in range(numlogs):
        mavproxy.expect("Log (\d+)")
        lognums.append(int(mavproxy.match.group(1)))
    mavproxy.expect("Log]")
    for i in range(numlogs):
        print("Dumping log %u (i=%u)" % (lognums[i], i))
        mavproxy.send("dump %u\n" % lognums[i])
        mavproxy.expect("logs enabled:", timeout=120)
        mavproxy.expect("Log]")
    util.pexpect_close(mavproxy)
    util.pexpect_close(sitl)
    log.close()
    print("Saved log for %s to %s" % (atype, logfile))
    return True

vehicle = os.path.basename(os.getcwd())
dump_logs(vehicle)
