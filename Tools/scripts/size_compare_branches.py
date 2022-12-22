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
import tempfile
import time
import board_list

if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


class SizeCompareBranchesResult(object):
    '''object to return results from a comparison'''

    def __init__(self, board, vehicle, bytes_delta):
        self.board = board
        self.vehicle = vehicle
        self.bytes_delta = bytes_delta


class SizeCompareBranches(object):
    '''script to build and compare branches using elf_diff'''

    def __init__(self,
                 branch=None,
                 master_branch="master",
                 board=["MatekF405-Wing"],
                 vehicle=["plane"],
                 bin_dir=None,
                 run_elf_diff=True,
                 all_vehicles=False,
                 all_boards=False,
                 use_merge_base=True,
                 extra_hwdef=None):
        if branch is None:
            branch = self.find_current_git_branch()

        self.master_branch = master_branch
        self.branch = branch
        self.board = board
        self.vehicle = vehicle
        self.bin_dir = bin_dir
        self.run_elf_diff = run_elf_diff
        self.extra_hwdef = extra_hwdef
        self.all_vehicles = all_vehicles
        self.all_boards = all_boards
        self.use_merge_base = use_merge_base

        if self.bin_dir is None:
            self.bin_dir = self.find_bin_dir()

        self.boards_by_name = {}
        for board in board_list.BoardList().boards:
            self.boards_by_name[board.name] = board

        # map from vehicle names to binary names
        self.vehicle_map = {
            "rover"     : "ardurover",
            "copter"    : "arducopter",
            "plane"     : "arduplane",
            "sub"       : "ardusub",
            "heli"      : "arducopter-heli",
            "blimp"     : "blimp",
            "antennatracker" : "antennatracker",
            "AP_Periph" : "AP_Periph",
            "iofirmware": "iofirmware_highpolh",  # FIXME: lowpolh?
        }

        if all_boards:
            self.board = sorted(list(self.boards_by_name.keys()), key=lambda x: x.lower())
        else:
            # validate boards
            all_boards = set(self.boards_by_name.keys())
            for b in self.board:
                if b not in all_boards:
                    raise ValueError("Bad board %s" % str(b))

        if all_vehicles:
            self.vehicle = sorted(list(self.vehicle_map.keys()), key=lambda x: x.lower())
        else:
            for v in self.vehicle:
                if v not in self.vehicle_map.keys():
                    raise ValueError("Bad vehicle (%s); choose from %s" % (v, ",".join(self.vehicle_map.keys())))

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
        p = subprocess.Popen(
            cmd_list,
            stdin=None,
            stdout=subprocess.PIPE,
            close_fds=True,
            stderr=subprocess.STDOUT,
            env=env)
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

    def find_current_git_branch(self):
        output = self.run_git(["symbolic-ref", "--short", "HEAD"])
        output = output.strip()
        return output

    def find_git_branch_merge_base(self, branch, master_branch):
        output = self.run_git(["merge-base", branch, master_branch])
        output = output.strip()
        return output

    def run_git(self, args):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("SCB-GIT", cmd_list)

    def run_waf(self, args, compiler=None):
        # try to modify the environment so we can consistent builds:
        consistent_build_envs = {
            "CHIBIOS_GIT_VERSION": "12345678",
            "GIT_VERSION": "abcdef",
            "GIT_VERSION_INT": "15",
        }
        for (n, v) in consistent_build_envs.items():
            os.environ[n] = v

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
        # we can't run `./waf copter blimp plane` without error, so do
        # them one-at-a-time:
        for v in vehicle:
            self.run_waf([v])
            self.run_program("rsync", ["rsync", "-aP", "build/", outdir])

    def run_all(self):
        '''run tests for boards and vehicles passed in constructor'''

        results = {}
        for board in self.board:
            vehicle_results = self.run_board(board)
            results[board] = vehicle_results
            with open("/tmp/some.csv", "w") as f:
                f.write(self.csv_for_results(results))

        return results

    def emit_csv_for_results(self, results):
        '''emit dictionary of dictionaries as a CSV'''
        print(self.csv_for_results(results))

    def csv_for_results(self, results):
        '''return a string with csv for results'''
        boards = sorted(results.keys())
        all_vehicles = set()
        for board in boards:
            all_vehicles.update(list(results[board].keys()))
        sorted_all_vehicles = sorted(list(all_vehicles))
        ret = ""
        ret += ",".join(["Board"] + sorted_all_vehicles) + "\n"
        for board in boards:
            line = [board]
            board_results = results[board]
            for vehicle in sorted_all_vehicles:
                bytes_delta = ""
                if vehicle in board_results:
                    result = board_results[vehicle]
                    bytes_delta = result.bytes_delta
                line.append(str(bytes_delta))
            ret += ",".join(line) + "\n"
        return ret

    def run(self):
        results = self.run_all()
        self.emit_csv_for_results(results)

    def run_board(self, board):
        ret = {}
        board_info = self.boards_by_name[board]

        vehicles_to_build = []
        for vehicle in self.vehicle:
            if vehicle == 'AP_Periph':
                if not board_info.is_ap_periph:
                    continue
            else:
                if board_info.is_ap_periph:
                    continue
                if vehicle.lower() not in [x.lower() for x in board_info.autobuild_targets]:
                    continue
            vehicles_to_build.append(vehicle)
        if len(vehicles_to_build) == 0:
            return ret

        tmpdir = tempfile.mkdtemp()
        outdir_1 = os.path.join(tmpdir, "out-master-%s" % (board,))
        outdir_2 = os.path.join(tmpdir, "out-branch-%s" % (board,))

        self.progress("Building branch 1 (%s)" % self.master_branch)
        master_commit = self.master_branch
        if self.use_merge_base:
            master_commit = self.find_git_branch_merge_base(self.branch, self.master_branch)
            self.progress("Using merge base (%s)" % master_commit)
        shutil.rmtree(outdir_1, ignore_errors=True)
        self.build_branch_into_dir(board, master_commit, vehicles_to_build, outdir_1)

        self.progress("Building branch 2 (%s)" % self.branch)
        shutil.rmtree(outdir_2, ignore_errors=True)
        self.build_branch_into_dir(board, self.branch, vehicles_to_build, outdir_2)

        for vehicle in vehicles_to_build:
            elf_filename = self.vehicle_map[vehicle]
            bin_filename = self.vehicle_map[vehicle] + '.bin'

            master_bin_dir = os.path.join(outdir_1, board, "bin")
            new_bin_dir = os.path.join(outdir_2, board, "bin")

            if self.run_elf_diff:
                self.progress("Starting compare (~10 minutes!)")
                elf_diff_commandline = [
                    "time",
                    "python3",
                    "-m", "elf_diff",
                    "--bin_dir", self.bin_dir,
                    '--bin_prefix=arm-none-eabi-',
                    "--old_alias", "%s %s" % (self.master_branch, elf_filename),
                    "--new_alias", "%s %s" % (self.branch, elf_filename),
                    "--html_dir", "../ELF_DIFF_%s_%s" % (board, vehicle),
                    os.path.join(master_bin_dir, elf_filename),
                    os.path.join(new_bin_dir, elf_filename)
                ]

                self.run_program("SCB", elf_diff_commandline)

            try:
                master_size = os.path.getsize(os.path.join(master_bin_dir, bin_filename))
                new_size = os.path.getsize(os.path.join(new_bin_dir, bin_filename))
            except FileNotFoundError:
                master_size = os.path.getsize(os.path.join(master_bin_dir, elf_filename))
                new_size = os.path.getsize(os.path.join(new_bin_dir, elf_filename))

            ret[vehicle] = SizeCompareBranchesResult(board, vehicle, new_size - master_size)

        return ret


if __name__ == '__main__':
    parser = optparse.OptionParser("size_compare_branches.py")
    parser.add_option("",
                      "--no-elf-diff",
                      action="store_true",
                      default=False,
                      help="do not run elf_diff on output files")
    parser.add_option("",
                      "--master-branch",
                      type="string",
                      default="master",
                      help="master branch to use")
    parser.add_option("",
                      "--no-merge-base",
                      action="store_true",
                      default=False,
                      help="do not use the merge-base for testing, do a direct comparison between branches")
    parser.add_option("",
                      "--branch",
                      type="string",
                      default=None,
                      help="branch to compare")
    parser.add_option("",
                      "--vehicle",
                      action='append',
                      default=[],
                      help="vehicle to build for")
    parser.add_option("",
                      "--board",
                      action='append',
                      default=[],
                      help="board to build for")
    parser.add_option("",
                      "--extra-hwdef",
                      type="string",
                      default=None,
                      help="configure with this extra hwdef file")
    parser.add_option("",
                      "--all-boards",
                      action='store_true',
                      default=False,
                      help="Build all boards")
    parser.add_option("",
                      "--all-vehicles",
                      action='store_true',
                      default=False,
                      help="Build all vehicles")
    cmd_opts, cmd_args = parser.parse_args()

    vehicle = []
    for v in cmd_opts.vehicle:
        vehicle.extend(v.split(','))
    if len(vehicle) == 0:
        vehicle.append("plane")

    board = []
    for b in cmd_opts.board:
        board.extend(b.split(','))
    if len(board) == 0:
        board.append("MatekF405-Wing")

    x = SizeCompareBranches(
        branch=cmd_opts.branch,
        master_branch=cmd_opts.master_branch,
        board=board,
        vehicle=vehicle,
        extra_hwdef=cmd_opts.extra_hwdef,
        run_elf_diff=(not cmd_opts.no_elf_diff),
        all_vehicles=cmd_opts.all_vehicles,
        all_boards=cmd_opts.all_boards,
        use_merge_base=not cmd_opts.no_merge_base,
    )
    x.run()
