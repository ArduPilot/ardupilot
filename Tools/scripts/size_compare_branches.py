#!/usr/bin/env python3

'''
Wrapper around elf_diff (https://github.com/noseglasses/elf_diff)
to create a html report comparing an ArduPilot build across two
branches

pip3 install --user elf_diff weasyprint

AP_FLAKE8_CLEAN

How to use?
Starting in the ardupilot directory.
~/ardupilot $ python Tools/scripts/size_compare_branches.py --branch=[PR_BRANCH_NAME] --vehicle=copter

Output is placed into ../ELF_DIFF_[VEHICLE_NAME]
'''

import optparse
import os
import shutil
import string
import subprocess
import sys
import time

if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


class SizeCompareBranches(object):
    '''script to build and compare branches using elf_diff'''

    def __init__(self,
                 branch=None,
                 master_branch="master",
                 board="MatekF405-Wing",
                 vehicle="plane",
                 bin_dir=None,
                 pdf_file=None,
                 extra_hwdef=None):
        if branch is None:
            raise Exception("branch required")  # FIXME: narrow exception
        self.master_branch = master_branch
        self.branch = branch
        self.board = board
        self.vehicle = vehicle
        self.bin_dir = bin_dir
        self.pdf_file = pdf_file
        self.extra_hwdef = extra_hwdef

        if self.bin_dir is None:
            self.bin_dir = self.find_bin_dir()

    def find_bin_dir(self):
        '''attempt to find where the arm-none-eabi tools are'''
        binary = shutil.which("arm-none-eabi-g++")
        if binary is None:
            raise Exception("No arm-none-eabi-g++?")
        return os.path.dirname(binary)

    # vast amounts of stuff copied into here from build_binaries.py

    def run_program(self, prefix, cmd_list, show_output=True, env=None):
        if show_output:
            self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(cmd_list, bufsize=1, stdin=None,
                             stdout=subprocess.PIPE, close_fds=True,
                             stderr=subprocess.STDOUT, env=env)
        output = ""
        while True:
            x = p.stdout.readline()
            if len(x) == 0:
                returncode = os.waitpid(p.pid, 0)
                if returncode:
                    break
                    # select not available on Windows... probably...
                time.sleep(0.1)
                continue
            if running_python3:
                x = bytearray(x)
                x = filter(lambda x : chr(x) in string.printable, x)
                x = "".join([chr(c) for c in x])
            output += x
            x = x.rstrip()
            if show_output:
                print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0 and show_output:
            self.progress("Process failed (%s)" %
                          str(returncode))
            raise subprocess.CalledProcessError(
                returncode, cmd_list)
        return output

    def run_git(self, args):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("SCB-GIT", cmd_list)

    def run_waf(self, args, compiler=None):
        if os.path.exists("waf"):
            waf = "./waf"
        else:
            waf = os.path.join(".", "modules", "waf", "waf-light")
        cmd_list = [waf]
        cmd_list.extend(args)
        env = None
        if compiler is not None:
            # default to $HOME/arm-gcc, but allow for any path with AP_GCC_HOME environment variable
            gcc_home = os.environ.get("AP_GCC_HOME", os.path.join(os.environ["HOME"], "arm-gcc"))
            gcc_path = os.path.join(gcc_home, compiler, "bin")
            if os.path.exists(gcc_path):
                # setup PATH to point at the right compiler, and setup to use ccache
                env = os.environ.copy()
                env["PATH"] = gcc_path + ":" + env["PATH"]
                env["CC"] = "ccache arm-none-eabi-gcc"
                env["CXX"] = "ccache arm-none-eabi-g++"
            else:
                raise Exception("BB-WAF: Missing compiler %s" % gcc_path)
        self.run_program("SCB-WAF", cmd_list, env=env)

    def progress(self, string):
        '''pretty-print progress'''
        print("SCB: %s" % string)

    def build_branch_into_dir(self, board, branch, vehicle, outdir):
        self.run_git(["checkout", branch])
        self.run_git(["submodule", "update", "--recursive"])
        shutil.rmtree("build", ignore_errors=True)
        waf_configure_args = ["configure", "--board", board]
        if self.extra_hwdef is not None:
            waf_configure_args.extend(["--extra-hwdef", self.extra_hwdef])
        self.run_waf(waf_configure_args)
        self.run_waf([vehicle])
        shutil.rmtree(outdir, ignore_errors=True)
        shutil.copytree("build", outdir)

    def run(self):
        outdir_1 = "/tmp/out-master"
        outdir_2 = "/tmp/out-branch"

        self.progress("Building branch 1")
        self.build_branch_into_dir(self.board, self.master_branch, self.vehicle, outdir_1)

        self.progress("Building branch 2")
        self.build_branch_into_dir(self.board, self.branch, self.vehicle, outdir_2)

        self.progress("Starting compare (~10 minutes!)")

        # map from vehicle names to binary names
        vehicle_map = {
            "rover"     : "ardurover",
            "copter"    : "arducopter",
            "plane"     : "arduplane",
            "sub"       : "ardusub",
            "heli"      : "arducopter-heli",
            "blimp"     : "blimp",
            "antennatracker" : "antennatracker",
            "AP_Periph" : "AP_Periph",
        }
        if self.vehicle in vehicle_map:
            binary_filename = vehicle_map[self.vehicle]
        else:
            raise Exception("Vehicle name (%s) incorrect" % (self.vehicle))

        elf_diff_commandline = [
            "time",
            "python3",
            "-m", "elf_diff",
            "--bin_dir", self.bin_dir,
            '--bin_prefix=arm-none-eabi-',
            "--old_alias", "%s %s" % (self.master_branch, binary_filename),
            "--new_alias", "%s %s" % (self.branch, binary_filename),
            "--html_dir", "../ELF_DIFF_%s" % (self.vehicle),
            os.path.join(outdir_1, self.board, "bin", binary_filename),
            os.path.join(outdir_2, self.board, "bin", binary_filename)
        ]

        #        if self.pdf_file is not None:
        #            elf_diff_commandline.extend(["--pdf_file", self.pdf_file])

        self.run_program("SCB", elf_diff_commandline)


if __name__ == '__main__':
    parser = optparse.OptionParser("size_compare_branches.py")
    parser.add_option("",
                      "--master-branch",
                      type="string",
                      default="master",
                      help="master branch to use")
    parser.add_option("",
                      "--branch",
                      type="string",
                      default=None,
                      help="branch to compare")
    parser.add_option("",
                      "--vehicle",
                      type="string",
                      default="plane",
                      help="vehicle to build for")
    parser.add_option("",
                      "--board",
                      type="string",
                      default="MatekF405-Wing",
                      help="board to build for")
    parser.add_option("",
                      "--extra-hwdef",
                      type="string",
                      default=None,
                      help="configure with this extra hwdef file")
#    parser.add_option("",
#                      "--pdf_file",
#                      type="string",
#                      default=None,
#                      help="output PDF to this file")
    cmd_opts, cmd_args = parser.parse_args()

    # we require --branch rather than taking a fixed-position argument
    # so that in the future we can assume the user wants to test the
    # currently checked out branch.  That requires a bit of work...
    if cmd_opts.branch is None:
        raise Exception("--branch must be supplied")  # FIXME: narrow exception

    x = SizeCompareBranches(
        branch=cmd_opts.branch,
        master_branch=cmd_opts.master_branch,
        board=cmd_opts.board,
        vehicle=cmd_opts.vehicle,
        extra_hwdef=cmd_opts.extra_hwdef,
        #        pdf_file=cmd_opts.pdf_file
    )
    x.run()
