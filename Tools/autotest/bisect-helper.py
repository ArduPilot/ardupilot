#!/usr/bin/env python

'''A helper script for bisecting common problems when working with ArduPilot

Bisect between a commit which builds and one which doesn't,
finding the first commit which broke the build with a
 specific failure:

git bisect reset
git bisect good a7647e77d9
git bisect bad 153ad9539866f8d93a99e9998118bb090d2f747f
cp -a Tools/autotest/bisect-helper.py /tmp
git bisect run /tmp/bisect-helper.py --build \
     --build-failure-string= \
     "reference to 'OpticalFlow' is ambiguous"

Work out who killed bebop:
cp -a Tools/autotest/bisect-helper.py /tmp
git bisect reset
git bisect good a7647e77d9 &&
  git bisect bad 153ad9539866f8d93a99e9998118bb090d2f747f &&
  git bisect run /tmp/bisect-helper.py --build \
    --waf-configure-arg="--board bebop"

# Use a failing test to work out which commit broke things:
cp Tools/autotest/bisect-helper.py /tmp
git bisect reset
git bisect start
git bisect bad
git bisect good HEAD~1024
time git bisect run /tmp/bisect-helper.py --autotest --autotest-vehicle=Plane --autotest-test=NeedEKFToArm --autotest-branch=wip/bisection-using-named-test  # noqa

Work out who overflowed Omnbusf4pro:
cp -a Tools Tools2
GOOD=c4ce6fa3851f93df34393c376fee5b37e0a270d2
BAD=f00bf77af75f828334f735580d6b19698b639a74
BFS="overflowed by"
git bisect reset
git bisect start
git bisect good $GOOD &&
  git bisect bad $BAD &&
  git bisect run Tools2/autotest/bisect-helper.py --build \
    --waf-configure-arg="--board OmniBusF4Pro" \
     --build-failure-string="$BFS"

# Use a flapping test to work out which commit broke things.  The
# "autotest-branch" is the branch containing the flapping test (which
# may be master)
rm /tmp/bisect-debug/*; git commit -m "stuff" -a ; cp Tools/autotest/bisect-helper.py /tmp; git bisect reset; git bisect start; git bisect bad d24e569b20; git bisect good 3f6fd49507f286ad8f6ccc9e29b110d5e9fc9207^
time git bisect run /tmp/bisect-helper.py --autotest --autotest-vehicle=Copter --autotest-test=Replay --autotest-branch=wip/bisection-using-flapping-test --autotest-test-passes=40 --autotest-failure-require-string="Mismatch in field XKF1.Pitch" --autotest-failure-ignore-string="HALSITL::SITL_State::_check_rc_input"

AP_FLAKE8_CLEAN

'''

import optparse
import os
import subprocess
import shlex
import sys
import time
import traceback


def get_exception_stacktrace(e):
    if sys.version_info[0] >= 3:
        ret = "%s\n" % e
        ret += ''.join(traceback.format_exception(type(e),
                                                  value=e,
                                                  tb=e.__traceback__))
        return ret
    return traceback.format_exc(e)


class Bisect(object):
    def __init__(self, opts):
        self.opts = opts

    def exit_skip_code(self):
        return 125

    def exit_pass_code(self):
        return 0

    def exit_fail_code(self):
        return 1

    def exit_abort_code(self):
        return 129

    def exit_skip(self):
        self.progress("SKIP")
        sys.exit(self.exit_skip_code())

    def exit_pass(self):
        self.progress("PASS")
        sys.exit(self.exit_pass_code())

    def exit_fail(self):
        self.progress("FAIL")
        sys.exit(self.exit_fail_code())

    def exit_abort(self):
        '''call when this harness has failed (e.g. to reset to required
        state)'''
        self.progress("ABORT")
        sys.exit(self.exit_abort_code())

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

    def build(self):
        '''run ArduCopter build.  May exit with skip or fail'''
        self.run_program("WAF-clean", ["./waf", "clean"])
        cmd_configure = ["./waf", "configure"]
        pieces = [shlex.split(x)
                  for x in self.opts.waf_configure_args]
        for piece in pieces:
            cmd_configure.extend(piece)
        self.run_program("WAF-configure", cmd_configure)
        cmd_build = ["./waf", "build"]
        pieces = [shlex.split(x)
                  for x in self.opts.waf_build_args]
        for piece in pieces:
            cmd_build.extend(piece)
        try:
            self.run_program("WAF-build", cmd_build)
        except subprocess.CalledProcessError:
            # well, it definitely failed....
            if self.opts.build_failure_string is not None:
                if self.opts.build_failure_string in self.program_output:
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

    def autotest_script(self):
        return os.path.join("Tools", "autotest", "autotest.py")

    def git_reset(self):
        try:
            self.run_program("Reset autotest directory", ["git", "reset", "--hard"])
        except subprocess.CalledProcessError:
            self.exit_abort()

    def get_current_hash(self):
        self.run_program("Get current hash", ["git", "rev-parse", "HEAD"])
        x = self.program_output
        return x.strip()

    def run(self):

        current_hash = self.get_current_hash()

        self.debug_dir = os.path.join("/tmp", "bisect-debug")
        if not os.path.exists(self.debug_dir):
            os.mkdir(self.debug_dir)

        if self.opts.autotest_branch is None:
            raise ValueError("expected autotest branch")

        try:
            self.run_program("Update submodules",
                             ["git", "submodule", "update", "--init", "--recursive"])
        except subprocess.CalledProcessError:
            self.exit_abort()

        try:
            self.run_program("Check autotest directory out from master",
                             ["git", "checkout", self.opts.autotest_branch, "Tools/autotest"])
        except subprocess.CalledProcessError:
            self.exit_abort()

        self.progress("Build")
        cmd = [self.autotest_script()]
        if self.opts.autotest_valgrind:
            cmd.append("--debug")
        cmd.append("build.%s" % self.opts.autotest_vehicle)
        print("build cmd: %s" % str(cmd))

        try:
            self.run_program("Run autotest (build)", cmd)
        except subprocess.CalledProcessError:
            self.git_reset()
            self.exit_skip()

        cmd = [self.autotest_script()]
        if self.opts.autotest_valgrind:
            cmd.append("--valgrind")
        cmd.append("test.%s.%s" % (self.opts.autotest_vehicle, self.opts.autotest_test))

        code = self.exit_pass_code()
        for i in range(0, self.opts.autotest_test_passes):
            ignore = False
            try:
                self.run_program(
                    "Run autotest (%u/%u)" % (i+1, self.opts.autotest_test_passes),
                    cmd)
            except subprocess.CalledProcessError:
                for ignore_string in self.opts.autotest_failure_ignore_string:
                    if ignore_string in self.program_output:
                        self.progress("Found ignore string (%s) in program output" % ignore_string)
                        ignore = True
                if not ignore and self.opts.autotest_failure_require_string is not None:
                    if self.opts.autotest_failure_require_string not in self.program_output:
                        # it failed, but not for the reason we're looking
                        # for...
                        self.progress("Did not find test failure string (%s); skipping" %
                                      self.opts.autotest_failure_require_string)
                        code = self.exit_skip_code()
                        break
                if not ignore:
                    code = self.exit_fail_code()

            with open(os.path.join(self.debug_dir, "run-%s-%u.txt" % (current_hash, i+1)), "w") as f:
                f.write(self.program_output)

            if code == self.exit_fail_code():
                with open("/tmp/fail-counts", "a") as f:
                    print("Failed on run %u" % (i+1,), file=f)
            if ignore:
                self.progress("Ignoring this run")
                continue
            if code != self.exit_pass_code():
                break

        self.git_reset()

        sys.exit(code)


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

    group_autotest = optparse.OptionGroup(parser, "Run-AutoTest Options")
    group_autotest.add_option("--autotest",
                              action='store_true',
                              default=False,
                              help="Bisect a failure with an autotest test")
    group_autotest.add_option("", "--autotest-vehicle",
                              dest="autotest_vehicle",
                              type="string",
                              default="ArduCopter",
                              help="Which vehicle to run tests for")
    group_autotest.add_option("", "--autotest-test",
                              dest="autotest_test",
                              type="string",
                              default="NavDelayAbsTime",
                              help="Test to run to find failure")
    group_autotest.add_option("", "--autotest-valgrind",
                              dest="autotest_valgrind",
                              action='store_true',
                              default=False,
                              help="Run autotest under valgrind")
    group_autotest.add_option("", "--autotest-test-passes",
                              dest="autotest_test_passes",
                              type=int,
                              default=1,
                              help="Number of times to run test before declaring it is good")
    group_autotest.add_option("", "--autotest-branch",
                              dest="autotest_branch",
                              type="string",
                              help="Branch on which the test exists.  The autotest directory will be reset to this branch")
    group_autotest.add_option("--autotest-failure-require-string",
                              type='string',
                              default=None,
                              help="If supplied, must be present in"
                              "test output to count as a failure")
    group_autotest.add_option("--autotest-failure-ignore-string",
                              type='string',
                              default=[],
                              action="append",
                              help="If supplied and present in"
                              "test output run will be ignored")

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
    elif opts.autotest:
        bisecter = BisectCITest(opts)
    else:
        raise ValueError("Not told how to bisect")

try:
    bisecter.run()
except Exception as e:
    print("Caught exception in bisect-helper: %s" % str(e))
    print(get_exception_stacktrace(e))
    sys.exit(129)  # should abort the bisect process
