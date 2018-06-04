#!/usr/bin/env python

'''
A helper script for bisecting common problems when working with ArduPilot

Bisect between a commit which builds and one which doesn't,
finding the first commit which broke the build with a
 specific failure:

git bisect good a7647e77d9
git bisect bad 153ad9539866f8d93a99e9998118bb090d2f747f
cp -a Tools/autotest/bisect-helper.py /tmp
git bisect run /tmp/bisect-helper.py --build \
     --build-failure-string= \
     "reference to 'OpticalFlow' is ambiguous"

Work out who killed bebop:
cp -a Tools/autotest/bisect-helper.py /tmp
git bisect good a7647e77d9 &&
  git bisect bad 153ad9539866f8d93a99e9998118bb090d2f747f &&
  git bisect run /tmp/bisect-helper.py --build \
    --waf-configure-arg="--board bebop"

'''

import optparse
import os
import subprocess
import shlex
import sys
import time


class Bisect(object):
    def __init__(self, opts):
        self.opts = opts

    def exit_skip(self):
        self.progress("SKIP")
        sys.exit(125)

    def exit_pass(self):
        self.progress("PASS")
        sys.exit(0)

    def exit_fail(self):
        self.progress("FAIL")
        sys.exit(1)

    def progress(self, string):
        '''pretty-print progress'''
        print("BH: %s" % string)

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
            self.program_output += x
            x = x.rstrip()
            print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0:
            self.progress("Process failed (%s)" %
                          str(returncode))
            raise subprocess.CalledProcessError(
                returncode, cmd_list)

    def build(self):
        '''run ArduCopter build.  May exit with skip or fail'''
        self.run_program("WAF-clean", ["./waf", "clean"])
        cmd_configure = ["./waf", "configure"]
        pieces = [shlex.split(x)
                  for x in opts.waf_configure_args]
        for piece in pieces:
            cmd_configure.extend(piece)
        self.run_program("WAF-configure", cmd_configure)
        cmd_build = ["./waf", "build"]
        pieces = [shlex.split(x)
                  for x in opts.waf_build_args]
        for piece in pieces:
            cmd_build.extend(piece)
        try:
            self.run_program("WAF-build", cmd_build)
        except subprocess.CalledProcessError as e:
            # well, it definitely failed....
            if opts.build_failure_string is not None:
                if opts.build_failure_string in self.program_output:
                    self.progress("Found relevant build failure")
                    self.exit_fail()
                # it failed, but not for the reason we're looking
                # for...
                self.exit_skip()
            else:
                self.exit_fail()


class BisectBuild(Bisect):

    def __init__(self, opts):
        super(BisectBuild, self).__init__(opts)

    def run(self):
        self.build()  # may exit with skip or fail
        self.exit_pass()


class BisectCITest(Bisect):

    def __init__(self, opts):
        super(BisectCITest, self).__init__(opts)


if __name__ == '__main__':

    parser = optparse.OptionParser("bisect.py ")
    parser.add_option("--build",
                      action='store_true',
                      default=False,
                      help="Help bisect a build failure")
    parser.add_option("--build-failure-string",
                      type='string',
                      default=None,
                      help="If supplied, must be present in"
                      "build output to count as a failure")

    group_build = optparse.OptionGroup(parser, "Build options")
    group_build.add_option("", "--waf-configure-arg",
                           action="append",
                           dest="waf_configure_args",
                           type="string",
                           default=["--board skyviper-v2450"],
                           help="extra arguments to pass to"
                           "waf in configure step")
    group_build.add_option("", "--waf-build-arg",
                           action="append",
                           dest="waf_build_args",
                           type="string",
                           default=["--target bin/arducopter"],
                           help="extra arguments to pass"
                           "to waf in its build step")

    parser.add_option_group(group_build)

    (opts, args) = parser.parse_args()

    if opts.build:
        bisecter = BisectBuild(opts)
    else:
        bisecter = BisectCITest(opts)

try:
    bisecter.run()
except Exception as e:
    print("Caught exception in bisect-helper: %s" % str(e))
    sys.exit(129)  # should abort the bisect process
