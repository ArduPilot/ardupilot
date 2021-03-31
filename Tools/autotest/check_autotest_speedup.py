#!/usr/bin/env python

'''
Run autotest repeatedly at different speedups to help find best default speedup

AP_FLAKE8_CLEAN
'''

import optparse
import os
import re
import subprocess
import time


class CheckAutoTestSpeedup(object):
    def __init__(
            self,
            build_target="build.Plane",
            test_target="test.QuadPlane",
            min_speedup=1,
            max_speedup=50,
            gdb=False,
            debug=False
    ):
        self.build_target = build_target
        self.test_target = test_target
        self.min_speedup = min_speedup
        self.max_speedup = max_speedup
        self.gdb = gdb
        self.debug = debug

    def progress(self, message):
        print("PROGRESS: %s" % (message,))

    def run_program(self, prefix, cmd_list):
        '''copied in from build_binaries.py'''
        '''run cmd_list, spewing and setting output in self'''
        self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(cmd_list,
                             bufsize=1,
                             stdin=None,
                             close_fds=True,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
        self.program_output = ""
        while True:
            x = p.stdout.readline()
            if len(x) == 0:
                returncode = os.waitpid(p.pid, 0)
                if returncode:
                    break
                    # select not available on Windows... probably...
                time.sleep(0.1)
                continue
            if type(x) == bytes:
                x = x.decode('utf-8')
            self.program_output += x
            x = x.rstrip()
            print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0:
            self.progress("Process failed (%s)" %
                          str(returncode))
            raise subprocess.CalledProcessError(
                returncode, cmd_list)

    def run(self):
        build_args = [
            "./Tools/autotest/autotest.py",
            self.build_target
        ]
        if opts.debug:
            build_args.append("--debug")
        self.run_program("BUILD", build_args)
        results = {
        }
        f = open(os.path.join("/tmp/speedup.txt"), "w")
        for i in range(self.max_speedup, self.min_speedup-1, -1):
            self.progress("Checking speedup %u" % i)
            run_args = [
                "./Tools/autotest/autotest.py",
                "--speedup", str(i),
                "--show-test-timings",
                self.test_target,
            ]
            if opts.gdb:
                run_args.append("--gdb")
            self.run_program("SPEEDUP-%03u" % i, run_args)
            for line in self.program_output.split("\n"):
                match = re.match(".*tests_total_time.*?([0-9.]+)s.*", line)
                if match is not None:
                    break
            results[i] = float(match.group(1))
            prog = "%u %f" % (i, results[i])
            self.progress(prog)
            print(prog, file=f)
            f.flush()

        for (speedup, t) in results.items():
            print("%u %f" % (speedup, t))


if __name__ == '__main__':
    parser = optparse.OptionParser(
        "check_autotest_speedup.py",
        epilog=""
        "e.g. ./Tools/autotest/check_autotest_speedup.py --max-speedup=40 --build-target=build.Sub --test-target=test.Sub"
    )
    parser.add_option("--debug",
                      default=False,
                      help='compile with debugging')
    parser.add_option("--gdb",
                      default=False,
                      help='run under gdb')
    parser.add_option("--max-speedup",
                      type=int,
                      default=50,
                      help='max speedup to test')
    parser.add_option("--min-speedup",
                      type=int,
                      default=1,
                      help='min speedup to test')
    parser.add_option("--build-target",
                      type='string',
                      default='build.Plane',
                      help='build target (e.g. build.Plane)')
    parser.add_option("--test-target",
                      type='string',
                      default='test.QuadPlane',
                      help='test target (e.g. test.QuadPlane)')

    opts, args = parser.parse_args()

    checker = CheckAutoTestSpeedup(
        gdb=opts.gdb,
        debug=opts.debug,
        max_speedup=opts.max_speedup,
        min_speedup=opts.min_speedup,
        build_target=opts.build_target,
        test_target=opts.test_target
    )

    checker.run()
