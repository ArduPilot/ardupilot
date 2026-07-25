'''
Base class for ArduPilot build scripts providing common utilities

AP_FLAKE8_CLEAN
'''

from __future__ import annotations

import copy
import fnmatch
import os
import pathlib
import queue
import re
import string
import subprocess
import sys
import tempfile
import threading
import time

from abc import ABC
from abc import abstractmethod

import board_list

# map from vehicle names to binary names
VEHICLE_MAP = {
    "rover"          : "ardurover",
    "copter"         : "arducopter",
    "plane"          : "arduplane",
    "sub"            : "ardusub",
    "heli"           : "arducopter-heli",
    "blimp"          : "blimp",
    "antennatracker" : "antennatracker",
    "AP_Periph"      : "AP_Periph",
    "bootloader"     : "AP_Bootloader",
    "iofirmware"     : "iofirmware_highpolh",  # FIXME: lowpolh?
}


class BuildScriptBase(ABC):
    """Base class for build scripts with common utilities for running programs"""

    def __init__(self, progress_file=None):
        self.tmpdir = None  # Can be set by subclasses that need it
        self.progress_file = progress_file

    def write_progress_file(self, content: str):
        '''write content to the progress file, if one was specified'''
        if self.progress_file is None:
            return
        pathlib.Path(self.progress_file).write_text(content)

    def resolve_board_and_vehicle_lists(self, all_boards=False, all_vehicles=False, exclude_board_glob=None):
        '''populate self.boards_by_name and self.vehicle_map; expand or
        validate self.board and self.vehicle; apply exclude globs to
        self.board'''
        if exclude_board_glob is None:
            exclude_board_glob = []

        self.boards_by_name = {}
        for board in board_list.BoardList().boards:
            self.boards_by_name[board.name] = board

        self.vehicle_map = VEHICLE_MAP

        if all_boards:
            self.board = sorted(list(self.boards_by_name.keys()), key=lambda x: x.lower())
        else:
            # validate boards
            for b in self.board:
                if b not in self.boards_by_name:
                    raise ValueError("Bad board %s" % str(b))

        if all_vehicles:
            self.vehicle = sorted(list(self.vehicle_map.keys()), key=lambda x: x.lower())
        else:
            for v in self.vehicle:
                if v not in self.vehicle_map.keys():
                    raise ValueError("Bad vehicle (%s); choose from %s" % (v, ",".join(self.vehicle_map.keys())))

        # remove boards based on --exclude-board-glob
        new_self_board = []
        for board_name in self.board:
            exclude = False
            for exclude_glob in exclude_board_glob:
                if fnmatch.fnmatch(board_name, exclude_glob):
                    exclude = True
                    break
            if not exclude:
                new_self_board.append(board_name)
        self.board = new_self_board

    def make_bootloader_blacklist(self):
        '''return set of board names for which we do not build bootloaders;
        requires self.boards_by_name to have been populated'''
        # some boards we don't have a -bl.dat for, so skip them.
        # TODO: find a way to get this information from board_list:
        ret = set([
            'CubeOrange-SimOnHardWare',
            'CubeOrangePlus-SimOnHardWare',
            'CubeRedSecondary-IO',
            'fmuv2',
            'fmuv3-bdshot',
            'iomcu',
            'iomcu-dshot',
            'iomcu-f103',
            'iomcu-f103-dshot',
            'iomcu-f103-8MHz-dshot',
            'iomcu_f103_8MHz',
            'luminousbee4',
            'skyviper-v2450',
            'skyviper-f412-rev1',
            'skyviper-journey',
            'Pixhawk1-1M-bdshot',
            'Pixhawk1-bdshot',
            'RADIX2HD',
            'canzero',
            't3-gem-o1',
            'CUAV-Pixhack-v3',  # uses USE_BOOTLOADER_FROM_BOARD
            'kha_eth',  # no hwdef-bl.dat
            'TBS-L431-Airspeed',  # uses USE_BOOTLOADER_FROM_BOARD
            'TBS-L431-BattMon',  # uses USE_BOOTLOADER_FROM_BOARD
            'TBS-L431-CurrMon',  # uses USE_BOOTLOADER_FROM_BOARD
            'TBS-L431-PWM',  # uses USE_BOOTLOADER_FROM_BOARD
            'ARKV6X-bdshot',  # uses USE_BOOTLOADER_FROM_BOARD

            'MatekL431-ADSB',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-Airspeed',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-APDTelem',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-AUAV',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-BatteryTag',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-BattMon',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-bdshot',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-DShot',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-EFI',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-GPS',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-HWTelem',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-MagHiRes',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-Periph',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-Proximity',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-Rangefinder',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-RC',  # uses USE_BOOTLOADER_FROM_BOARD
            'MatekL431-Serial',  # uses USE_BOOTLOADER_FROM_BOARD
        ])

        for board in self.boards_by_name.values():
            if board.hal in ["Linux", "ESP32", "SITL"]:
                ret.add(board.name)

        return ret

    def vehicles_to_build_for_board_info(self, board_info):
        vehicles_to_build = []
        for vehicle in self.vehicle:
            if vehicle == 'AP_Periph':
                if not board_info.is_ap_periph:
                    continue
            elif vehicle == 'bootloader':
                # we generally build bootloaders
                pass
            else:
                if board_info.is_ap_periph:
                    continue
                # Map vehicle name to autobuild target name
                # antennatracker (waf target) -> Tracker (autobuild target)
                vehicle_for_autobuild = vehicle
                if vehicle.lower() == 'antennatracker':
                    vehicle_for_autobuild = 'tracker'
                if vehicle_for_autobuild.lower() not in [x.lower() for x in board_info.autobuild_targets]:
                    continue
            vehicles_to_build.append(vehicle)

        return vehicles_to_build

    def extra_hwdef_file(self, more):
        # create a combined list of hwdefs:
        extra_hwdefs = []
        extra_hwdefs.extend(self.extra_hwdef)
        extra_hwdefs.extend(more)
        extra_hwdefs = list(filter(lambda x : x is not None, extra_hwdefs))
        if len(extra_hwdefs) == 0:
            return None

        # slurp all content into a variable:
        content = bytearray()
        for extra_hwdef in extra_hwdefs:
            with open(extra_hwdef, "r+b") as in_file:
                content += in_file.read()

        # spew content to single file:
        with tempfile.NamedTemporaryFile(delete=False) as out_file:
            out_file.write(content)
            return out_file.name

    def parallel_thread_main(self, thread_number):
        # initialisation; make a copy of the source directory
        my_source_dir = os.path.join(self.tmpdir, f"thread-{thread_number}-source")
        self.run_program("rsync", [
            "rsync",
            "--exclude=build/",
            "-ap",
            "./",
            my_source_dir
        ])

        while True:
            try:
                task = self.parallel_tasks.pop(0)
            except IndexError:
                break
            jobs = None
            if self.jobs is not None:
                jobs = int(self.jobs / self.n_threads)
                if jobs <= 0:
                    jobs = 1
            try:
                self.run_build_task(task, source_dir=my_source_dir, jobs=jobs)
            except Exception as ex:
                self.thread_exit_result_queue.put(f"{task}")
                raise ex

    def check_result_queue(self):
        while True:
            try:
                result = self.thread_exit_result_queue.get_nowait()
            except queue.Empty:
                break
            if result is None:
                continue
            self.failure_exceptions.append(result)

    def parallel_progress_hook(self, tasks):
        '''called periodically while parallel build tasks run; subclasses
        may override to emit progress information'''
        pass

    def run_build_tasks_in_parallel(self, tasks):
        '''runs run_build_task for each of tasks across self.parallel_copies
        source-directory copies; failed tasks are collected (as strings) in
        self.failure_exceptions'''
        self.n_threads = self.parallel_copies

        # shared list for the threads:
        self.parallel_tasks = copy.copy(tasks)  # make this an argument instead?!
        threads = []
        self.thread_exit_result_queue = queue.Queue()
        tstart = time.time()
        self.failure_exceptions = []

        thread_number = 0
        while len(self.parallel_tasks) or len(threads):
            if len(self.parallel_tasks) < self.n_threads:
                self.n_threads = len(self.parallel_tasks)
            while len(threads) < self.n_threads:
                self.progress(f"Starting thread {thread_number}")
                t = threading.Thread(
                    target=self.parallel_thread_main,
                    name=f'task-builder-{thread_number}',
                    args=[thread_number],
                )
                t.start()
                threads.append(t)
                thread_number += 1

            self.check_result_queue()

            new_threads = []
            for thread in threads:
                thread.join(0)
                if thread.is_alive():
                    new_threads.append(thread)
            threads = new_threads
            self.progress(
                f"remaining-tasks={len(self.parallel_tasks)} " +
                f"failed-threads={len(self.failure_exceptions)} elapsed={int(time.time() - tstart)}s")  # noqa

            self.parallel_progress_hook(tasks)

            time.sleep(1)
        self.progress("All threads returned")

        self.check_result_queue()

        if len(self.failure_exceptions):
            self.progress("Some threads failed:")
        for ex in self.failure_exceptions:
            print("Thread failure: %s" % str(ex))

    def run_program(self, prefix, cmd_list, show_output=True, env=None, show_output_on_error=True, show_command=None, cwd="."):
        if show_command is None:
            show_command = True

        cmd = " ".join(cmd_list)
        if cwd is None:
            cwd = "."
        command_debug = f"Running ({cmd}) in ({cwd})"
        process_failure_content = command_debug + "\n"
        if show_command:
            self.progress(command_debug)

        p = subprocess.Popen(
            cmd_list,
            stdin=None,
            stdout=subprocess.PIPE,
            close_fds=True,
            stderr=subprocess.STDOUT,
            cwd=cwd,
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
            x = bytearray(x)
            x = filter(lambda x : chr(x) in string.printable, x)
            x = "".join([chr(c) for c in x])
            output += x
            process_failure_content += x
            x = x.rstrip()
            some_output = "%s: %s" % (prefix, x)
            if show_output:
                print(some_output)
        (_, status) = returncode
        if status != 0:
            if not show_output and show_output_on_error:
                # we were told not to show output, but we just
                # failed... so show output...
                print(output)
            self.progress("Process failed (%s)" %
                          str(returncode))
            try:
                path = pathlib.Path(self.tmpdir, f"process-failure-{int(time.time())}")
                path.write_text(process_failure_content)
                self.progress("Wrote process failure file (%s)" % path)
            except Exception:  # noqa: BLE001 — best-effort debug-file write
                self.progress("Writing process failure file failed")
            raise subprocess.CalledProcessError(
                status, cmd_list)
        return output

    def find_current_git_branch_or_sha1(self):
        try:
            output = self.run_git(["symbolic-ref", "--short", "HEAD"])
            output = output.strip()
            return output
        except subprocess.CalledProcessError:
            pass

        # probably in a detached-head state.  Get a sha1 instead:
        output = self.run_git(["rev-parse", "--short", "HEAD"])
        output = output.strip()
        return output

    def find_git_branch_merge_base(self, branch, master_branch):
        output = self.run_git(["merge-base", branch, master_branch])
        output = output.strip()
        return output

    def run_git(self, args, show_output=True, source_dir=None):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("SCB-GIT", cmd_list, show_output=show_output, cwd=source_dir)

    def get_added_paths_for_commit(self, commit: str) -> list:
        '''return the list of paths added (created) in a single commit'''
        output = self.run_git(
            ['diff-tree', '--no-commit-id', '-r', '--name-only',
             '--diff-filter=A', commit],
            show_output=False,
        )
        return [line.strip() for line in output.splitlines() if line.strip()]

    def created_library_dirs(self, commit: str) -> set:
        '''libraries/<X> subsystem names introduced by files added in commit'''
        created = set()
        for path in self.get_added_paths_for_commit(commit):
            parts = path.split('/')
            if parts[0] == 'libraries' and len(parts) >= 3:
                created.add(parts[1])
        return created

    def run_waf(self, args, compiler=None, show_output=True, source_dir=None):
        # try to modify the environment so we can consistent builds:
        consistent_build_envs = {
            "CHIBIOS_GIT_VERSION": "12345678",
            "GIT_VERSION": "abcdef",
            "GIT_VERSION_EXTENDED": "0123456789abcdef",
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
        self.run_program("SCB-WAF", cmd_list, env=env, show_output=show_output, cwd=source_dir)

    def get_modified_hwdef_paths(self, branch, master_branch, use_merge_base=True):
        '''returns set of hwdef filepaths modified between branch and
        master, relative to the repository root'''
        if use_merge_base:
            base_commit = self.find_git_branch_merge_base(branch, master_branch)
        else:
            base_commit = master_branch

        delta_files = self.run_git([
            'diff',
            '--name-only',
            base_commit,
            branch,
        ], show_output=False)

        ret = set()
        for f in delta_files.split("\n"):
            f = f.strip()
            if not f:
                continue
            if "/hwdef/" not in f:
                continue
            if not f.endswith(("hwdef.dat", "hwdef.inc", "hwdef-bl.dat", "hwdef-bl.inc")):
                continue
            ret.add(f)
        return ret

    def collect_hwdef_includes(self, filepath, seen=None):
        '''recursively collect all hwdef file paths included from filepath'''
        if seen is None:
            seen = set()
        filepath = os.path.realpath(filepath)
        if filepath in seen:
            return set()
        seen.add(filepath)
        paths = {filepath}
        try:
            with open(filepath) as fh:
                for line in fh:
                    m = re.match(r"^\s*include\s+(.+)\s*$", line)
                    if m is not None:
                        inc_path = os.path.join(os.path.dirname(filepath), m.group(1))
                        paths.update(self.collect_hwdef_includes(inc_path, seen))
        except FileNotFoundError:
            pass
        return paths

    def find_modified_boards(self, branch, master_branch, use_merge_base=True):
        '''find boards whose hwdef files have been modified between
        branch and master'''
        modified_paths = self.get_modified_hwdef_paths(
            branch, master_branch, use_merge_base)
        if not modified_paths:
            return []

        for p in modified_paths:
            self.progress("Modified hwdef: %s" % p)

        # convert to absolute paths for comparison
        modified_abs = set()
        for p in modified_paths:
            modified_abs.add(os.path.realpath(p))

        bl = board_list.BoardList()
        modified_board_names = []

        for board_obj in bl.boards:
            for hwdef_dir in bl.hwdef_dir:
                hwdef_path = os.path.join(hwdef_dir, board_obj.name, "hwdef.dat")
                if not os.path.exists(hwdef_path):
                    continue
                includes = self.collect_hwdef_includes(hwdef_path)
                hwdef_bl_path = os.path.join(hwdef_dir, board_obj.name, "hwdef-bl.dat")
                if os.path.exists(hwdef_bl_path):
                    includes.update(self.collect_hwdef_includes(hwdef_bl_path))
                if includes & modified_abs:
                    modified_board_names.append(board_obj.name)
                    self.progress("Board %s uses modified hwdef" % board_obj.name)
                    break

        return sorted(modified_board_names, key=lambda x: x.lower())

    @abstractmethod
    def progress_prefix(self) -> str:
        '''return a short prefix string identifying this script in log output'''

    def progress(self, string):
        '''pretty-print progress'''
        print(f"{self.progress_prefix()}: {string}", file=sys.stderr)

    def size_for_elf(self, elf_path, toolchain="arm-none-eabi"):
        '''run size on an ELF file and return parsed results dict.

        Returns dict with keys: size_text, size_data, size_bss,
        size_total, size_free_flash, ext_flash_used.
        Uses parsing logic compatible with build_summary._parse_size_output.
        '''
        size_tool = f"{toolchain}-size"

        # run `size <elf>`
        size_output = self.run_program(
            "SIZE", [size_tool, elf_path], show_output=False)

        # run `size -A <elf>` for section details
        size_a_output = self.run_program(
            "SIZE", [size_tool, "-A", elf_path], show_output=False)

        # parse size -A output for .crash_log, .heap, .extflash
        crash_log_size = None
        heap_size = 0
        ext_flash_used = 0
        for line in size_a_output.splitlines()[1:]:
            if ".crash_log" in line:
                row = line.strip().split()
                crash_log_size = int(row[1])
            if ".heap" in line:
                row = line.strip().split()
                heap_size = int(row[1])
            if ".extflash" in line:
                row = line.strip().split()
                if int(row[1]) > 0:
                    ext_flash_used = int(row[1])

        # parse standard size output
        lines = size_output.splitlines()[1:]
        # use first data line (non-TOTALS)
        for line in lines:
            row = line.strip().split()
            if len(row) < 3:
                continue

            if crash_log_size is None:
                size_bss = int(row[2])
                size_free_flash = None
            else:
                size_bss = int(row[2]) - crash_log_size
                size_free_flash = crash_log_size
            size_bss -= heap_size

            return dict(
                size_text=int(row[0]),
                size_data=int(row[1]),
                size_bss=size_bss,
                size_total=int(row[0]) + int(row[1]) - ext_flash_used,
                size_free_flash=size_free_flash,
                ext_flash_used=ext_flash_used if ext_flash_used else None,
            )

        raise ValueError(f"Could not parse size output for {elf_path}")
