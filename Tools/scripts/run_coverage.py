#!/usr/bin/env python3

"""
Runs tests with gcov coverage support.

 AP_FLAKE8_CLEAN
"""
import argparse
import os
import tempfile
import time
import shutil
import subprocess
import sys

os.environ['PYTHONUNBUFFERED'] = '1'
os.set_blocking(sys.stdout.fileno(), True)
os.set_blocking(sys.stderr.fileno(), True)

tools_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.realpath(os.path.join(tools_dir, '../..'))


class CoverageRunner(object):
    """Coverage Runner Class."""

    def __init__(self, verbose=False, check_tests=True) -> None:
        """Set the files Path."""
        self.REPORT_DIR = os.path.join(root_dir, "reports/lcov-report")
        self.INFO_FILE = os.path.join(root_dir, self.REPORT_DIR, "lcov.info")
        self.INFO_FILE_BASE = os.path.join(root_dir, self.REPORT_DIR, "lcov_base.info")
        self.LCOV_LOG = os.path.join(root_dir, "GCOV_lcov.log")
        self.GENHTML_LOG = os.path.join(root_dir, "GCOV_genhtml.log")

        self.autotest = os.path.join(root_dir, "Tools/autotest/autotest.py")
        self.verbose = verbose
        self.check_tests = check_tests
        self.start_time = time.time()

    def progress(self, text) -> None:
        """Pretty printer."""
        delta_time = time.time() - self.start_time
        formatted_text = "****** AT-%06.1f: %s" % (delta_time, text)
        print(formatted_text)

    def init_coverage(self, use_example=False) -> None:
        """Initialize ArduPilot for coverage.

        This needs to be run with the binaries built.
        """
        self.progress("Initializing Coverage...")
        self.progress("Removing previous reports")
        try:
            shutil.rmtree(self.REPORT_DIR)
        except FileNotFoundError:
            pass

        try:
            os.makedirs(self.REPORT_DIR)
        except FileExistsError:
            pass

        self.progress("Checking that vehicles binaries are set up and built")
        binaries_dir = os.path.join(root_dir, 'build/sitl/bin')
        dirs_to_check = [["binaries", binaries_dir], ["tests", os.path.join(root_dir, 'build/linux/tests')]]
        if use_example:
            self.progress("Adding examples")
            dirs_to_check.append(["examples", os.path.join(root_dir, 'build/linux/examples')])
        for dirc in dirs_to_check:
            if not (self.check_build(dirc[0], dirc[1])):
                self.run_build()
                break

        self.progress("Zeroing previous build")
        retcode = subprocess.call(["lcov", "--zerocounters", "--directory", root_dir])
        if retcode != 0:
            self.progress("Failed with retcode (%s)" % retcode)
            exit(1)

        self.progress("Initializing Coverage with current build")
        try:
            result = subprocess.run(["lcov",
                                     "--no-external",
                                     "--initial",
                                     "--capture",
                                     "--exclude", root_dir + "/build/sitl/modules/*",
                                     "--directory", root_dir,
                                     "-o", self.INFO_FILE_BASE,
                                     ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=True)
            if self.verbose:
                print(*result.args)
                print(result.stdout)
            with open(self.LCOV_LOG, 'w') as log_file:
                log_file.write(result.stdout)
        except subprocess.CalledProcessError as err:
            print("ERROR :")
            print(err.cmd)
            print(err.output)
            exit(1)
        self.progress("Initialization done")

    def check_build(self, name, path) -> bool:
        """Check that build directory is not empty and that binaries are built with the coverage flags."""
        self.progress("Checking that %s are set up and built" % name)
        if os.path.exists(path):
            if not os.listdir(path):
                return False
        else:
            return False
        self.progress("Checking %s build configuration for coverage" % name)
        with open(os.path.join(path, "../compile_commands.json"), "r") as searchfile:
            for line in searchfile:
                if "-ftest-coverage" in line:
                    return True
            self.progress("%s was't built with coverage support" % name)
            return False

    def run_build(self, use_example=False) -> None:
        """Clean the build directory and build binaries for coverage."""

        os.chdir(root_dir)
        waf_light = os.path.join(root_dir, "modules/waf/waf-light")
        self.progress("Removing previous build binaries")
        subprocess.run([waf_light, "configure", "--debug"], check=True)
        subprocess.run([waf_light, "clean"], check=True)

        self.progress("Building examples and SITL binaries")

        try:
            if use_example:
                self.progress("Building examples")
                subprocess.run([waf_light, "configure", "--board=linux", "--debug", "--coverage"], check=True)
                subprocess.run([waf_light, "examples"], check=True)
            subprocess.run(
                [self.autotest,
                 "--debug",
                 "--coverage",
                 "build.unit_tests"],
                check=True)
            subprocess.run([waf_light, "configure", "--debug", "--coverage"], check=True)
            subprocess.run([waf_light], check=True)
        except subprocess.CalledProcessError as err:
            print("ERROR :")
            print(err.cmd)
            print(err.output)
            exit(1)
        self.progress("Build examples and vehicle binaries done !")

    def run_full(self, use_example=False) -> None:
        """Run full coverage on maximum of ArduPilot binaries and test functions."""
        self.progress("Running full test suite...")
        self.run_build()
        self.init_coverage()
        self.progress("Running tests")
        SPEEDUP = 5
        TIMEOUT = 14400

        if use_example:
            self.progress("Running run.examples")
            subprocess.run([self.autotest,
                            "--timeout=" + str(TIMEOUT),
                            "--debug",
                            "--coverage",
                            "--no-clean",
                            "--speedup=" + str(SPEEDUP),
                            "run.examples"], check=self.check_tests)
        self.progress("Running run.unit_tests")
        subprocess.run(
            [self.autotest,
             "--timeout=" + str(TIMEOUT),
             "--debug",
             "--no-clean",
             "run.unit_tests"], check=self.check_tests)
        subprocess.run(["reset"], check=True)
        os.set_blocking(sys.stdout.fileno(), True)
        os.set_blocking(sys.stderr.fileno(), True)
        test_list = ["Plane", "QuadPlane", "Sub", "Copter", "Helicopter", "Rover", "Tracker", "BalanceBot", "Sailboat"]
        for test in test_list:
            self.progress("Running test.%s" % test)
            try:
                subprocess.run([self.autotest,
                                "--timeout=" + str(TIMEOUT),
                                "--debug",
                                "--no-clean",
                                "test.%s" % test], check=self.check_tests)
            except subprocess.CalledProcessError:
                # pass in case of failing tests
                pass
        # TODO add any other execution path/s we can to maximise the actually
        # used code, can we run other tests or things?  Replay, perhaps?
        self.update_stats()

    def update_stats(self) -> None:
        """Update Coverage statistics only.

        Assumes that coverage tests have been run.
        """
        self.progress("Generating Coverage statistics")
        with open(self.LCOV_LOG, 'a') as log_file:
            # we cannot use subprocess.PIPE and result.stdout to get the output as it will be too long and trigger
            # BlockingIOError: [Errno 11] write could not complete without blocking
            # thus we ouput to temp file, and print the file line by line...
            with tempfile.NamedTemporaryFile(mode="w+") as tmp_file:
                try:
                    self.progress("Capturing Coverage statistics")
                    subprocess.run(["lcov",
                                    "--no-external",
                                    "--capture",
                                    "--directory", root_dir,
                                    "-o", self.INFO_FILE,
                                    ], stdout=tmp_file, stderr=subprocess.STDOUT, text=True, check=True)
                    if self.verbose:
                        tmp_file.seek(0)
                        content = tmp_file.read().splitlines()
                        for line in content:
                            print(line, flush=True)
                            log_file.write(line)

                    self.progress("Matching Coverage with binaries")
                    subprocess.run(["lcov",
                                    "--add-tracefile", self.INFO_FILE_BASE,
                                    "--add-tracefile", self.INFO_FILE,
                                    ], stdout=tmp_file, stderr=subprocess.STDOUT, text=True, check=True)
                    if self.verbose:
                        tmp_file.seek(0)
                        content = tmp_file.read().splitlines()
                        for line in content:
                            # print(line, flush=True)  # not usefull to print
                            log_file.write(line)
                    # remove files we do not intentionally test:
                    self.progress("Removing unwanted coverage statistics")
                    subprocess.run(["lcov",
                                    "--remove", self.INFO_FILE,
                                    ".waf*",
                                    root_dir + "/modules/gtest/*",
                                    root_dir + "/modules/DroneCAN/libcanard/*",
                                    root_dir + "/build/linux/libraries/*",
                                    root_dir + "/build/linux/modules/*",
                                    root_dir + "/build/sitl/libraries/*",
                                    root_dir + "/build/sitl/modules/*",
                                    root_dir + "/build/sitl_periph_universal/libraries/*",
                                    root_dir + "/build/sitl_periph_universal/modules/*",
                                    root_dir + "/libraries/*/examples/*",
                                    root_dir + "/libraries/*/tests/*",
                                    "-o", self.INFO_FILE
                                    ], stdout=tmp_file, stderr=subprocess.STDOUT, text=True, check=True)
                    if self.verbose:
                        tmp_file.seek(0)
                        content = tmp_file.read().splitlines()
                        for line in content:
                            # print(line, flush=True)  # not usefull to print
                            log_file.write(line)

                except subprocess.CalledProcessError as err:
                    print("ERROR :")
                    print(err.cmd)
                    print(err.output)
                    sys.exit(1)

        with open(self.GENHTML_LOG, 'w+') as log_file:
            try:
                self.progress("Generating HTML files")
                subprocess.run(["genhtml", self.INFO_FILE,
                                "-o", self.REPORT_DIR,
                                "--demangle-cpp",
                                ], stdout=log_file, stderr=subprocess.STDOUT, text=True, check=True)

                log_file.seek(0)
                content = log_file.read().splitlines()
                if self.verbose:
                    for line in content[1: -3]:
                        print(line, flush=True)
                for line in content[-3:]:
                    print(line, flush=True)
            except subprocess.CalledProcessError as err:
                print("ERROR :")
                print(err.cmd)
                print(err.output)
                exit(1)
        self.progress("Coverage successful. Open " + self.REPORT_DIR + "/index.html")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs tests with gcov coverage support.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Output everything on terminal.')
    parser.add_argument('-c', '--no-check-tests', action='store_true',
                        help='Do not fail if tests do not run.')
    parser.add_argument('--add-examples', action='store_true',
                        help='Add examples to coverage.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-i', '--init', action='store_true',
                       help='Initialise ArduPilot for coverage. It should be run after building the binaries.')
    group.add_argument('-f', '--full', action='store_true',
                       help='Run ArduPilot full coverage. This will run all tests and example. It is really long.')
    group.add_argument('-b', '--build', action='store_true',
                       help='Clean the build directory and build binaries for coverage.')
    group.add_argument('-u', '--update', action='store_true',
                       help='Update coverage statistics. To be used after running some tests.')
    args = parser.parse_args()

    runner = CoverageRunner(verbose=args.verbose, check_tests=not args.no_check_tests)
    if args.init:
        runner.init_coverage(args.add_examples)
        sys.exit(0)
    if args.full:
        runner.run_full(args.add_examples)
        sys.exit(0)
    if args.build:
        runner.run_build(args.add_examples)
        sys.exit(0)
    if args.update:
        runner.update_stats()
        sys.exit(0)
    parser.print_help()
    sys.exit(0)
