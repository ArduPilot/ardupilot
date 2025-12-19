#!/usr/bin/env python3

from __future__ import annotations

'''
Wrapper around elf_diff (https://github.com/noseglasses/elf_diff)
to create a html report comparing an ArduPilot build across two
branches

python3 -m pip install --user elf_diff weasyprint

AP_FLAKE8_CLEAN

How to use?
Starting in the ardupilot directory.
~/ardupilot $ python Tools/scripts/size_compare_branches.py --branch=[PR_BRANCH_NAME] --vehicle=copter

Output is placed into ELF_DIFF_[VEHICLE_NAME]
'''

import copy
import fnmatch
import optparse
import os
import pathlib
import queue
import shutil
import tempfile
import threading
import time
import board_list
from build_script_base import BuildScriptBase


class SizeCompareBranchesResult(object):
    '''object to return results from a comparison'''

    def __init__(self, board, vehicle, bytes_delta, identical):
        self.board = board
        self.vehicle = vehicle
        self.bytes_delta = bytes_delta
        self.identical = identical


class FeatureCompareBranchesResult(object):
    '''object to return results from a comparison'''

    def __init__(self, board, vehicle, delta_features_in, delta_features_out):
        self.board = board
        self.vehicle = vehicle
        self.delta_features_in = delta_features_in
        self.delta_features_out = delta_features_out


class SizeCompareBranches(BuildScriptBase):
    '''script to build and compare branches using elf_diff'''

    def __init__(self,
                 branch=None,
                 master_branch="master",
                 board: list | None = None,
                 vehicle: list | None = None,
                 bin_dir=None,
                 run_elf_diff=True,
                 all_vehicles=False,
                 exclude_board_glob: list | None = None,
                 all_boards=False,
                 use_merge_base=True,
                 waf_consistent_builds=True,
                 show_empty=True,
                 show_unchanged=True,
                 extra_hwdef: list | None = None,
                 extra_hwdef_branch: list | None = None,
                 extra_hwdef_master: list | None = None,
                 parallel_copies=None,
                 jobs=None,
                 features=False,
                 ):
        super().__init__()

        if board is None:
            board = ["MatekF405-Wing"]
        if vehicle is None:
            vehicle = ["plane"]
        if exclude_board_glob is None:
            exclude_board_glob = []
        if extra_hwdef is None:
            extra_hwdef = []
        if extra_hwdef_branch is None:
            extra_hwdef_branch = []
        if extra_hwdef_master is None:
            extra_hwdef_master = []

        if branch is None:
            branch = self.find_current_git_branch_or_sha1()

        self.master_branch = master_branch
        self.branch = branch
        self.board = board
        self.vehicle = vehicle
        self.bin_dir = bin_dir
        self.run_elf_diff = run_elf_diff
        self.extra_hwdef = extra_hwdef
        self.extra_hwdef_branch = extra_hwdef_branch
        self.extra_hwdef_master = extra_hwdef_master
        self.all_vehicles = all_vehicles
        self.all_boards = all_boards
        self.use_merge_base = use_merge_base
        self.waf_consistent_builds = waf_consistent_builds
        self.show_empty = show_empty
        self.show_unchanged = show_unchanged
        self.parallel_copies = parallel_copies
        self.jobs = jobs
        self.features = features

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
            "bootloader": "AP_Bootloader",
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

        # some boards we don't have a -bl.dat for, so skip them.
        # TODO: find a way to get this information from board_list:
        self.bootloader_blacklist = set([
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
            'SITL_arm_linux_gnueabihf',
            'SITL_x86_64_linux_gnu',
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

        for board_name in self.board:
            board = self.boards_by_name[board_name]
            if board.hal in ["Linux", "ESP32"]:
                self.bootloader_blacklist.add(board.name)

    def find_bin_dir(self, toolchain_prefix="arm-none-eabi-"):
        '''attempt to find where the arm-none-eabi tools are'''
        binary = shutil.which(toolchain_prefix + "g++")
        if binary is None:
            return None
        return os.path.dirname(binary)

    def progress_prefix(self):
        return 'SCB'

    def build_branch_into_dir(self, board, branch, vehicle, outdir, source_dir=None, extra_hwdef=None, jobs=None):
        self.run_git(["checkout", branch], show_output=False, source_dir=source_dir)
        self.run_git(["submodule", "update", "--recursive"], show_output=False, source_dir=source_dir)
        build_dir = "build"
        if source_dir is not None:
            build_dir = os.path.join(source_dir, "build")
        shutil.rmtree(build_dir, ignore_errors=True)
        waf_configure_args = ["configure", "--board", board]
        if self.waf_consistent_builds:
            waf_configure_args.append("--consistent-builds")

        if extra_hwdef is not None:
            waf_configure_args.extend(["--extra-hwdef", extra_hwdef])

        if self.run_elf_diff:
            waf_configure_args.extend(["--debug-symbols"])

        if jobs is None:
            jobs = self.jobs
        if jobs is not None:
            waf_configure_args.extend(["-j", str(jobs)])

        self.run_waf(waf_configure_args, show_output=False, source_dir=source_dir)
        # we can't run `./waf copter blimp plane` without error, so do
        # them one-at-a-time:
        for v in vehicle:
            if v == 'bootloader':
                # need special configuration directive
                continue
            self.run_waf([v], show_output=False, source_dir=source_dir)
        for v in vehicle:
            if v != 'bootloader':
                continue
            if board in self.bootloader_blacklist:
                continue
            # need special configuration directive
            bootloader_waf_configure_args = copy.copy(waf_configure_args)
            bootloader_waf_configure_args.append('--bootloader')
            if not self.boards_by_name[board].is_ap_periph:
                # hopefully temporary hack so you can build bootloader
                # after building other vehicles without a clean:
                dsdl_generated_path = os.path.join('build', board, "modules", "DroneCAN", "libcanard", "dsdlc_generated")
                self.progress("HACK: Removing (%s)" % dsdl_generated_path)
                if source_dir is not None:
                    dsdl_generated_path = os.path.join(source_dir, dsdl_generated_path)
                shutil.rmtree(dsdl_generated_path, ignore_errors=True)
            self.run_waf(bootloader_waf_configure_args, show_output=False, source_dir=source_dir)
            self.run_waf([v], show_output=False, source_dir=source_dir)
        self.run_program("rsync", ["rsync", "-ap", "build/", outdir], cwd=source_dir)
        if source_dir is not None:
            pathlib.Path(outdir, "scb_sourcepath.txt").write_text(source_dir)

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

    def run_build_tasks_in_parallel(self, tasks):
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

            # write out a progress CSV:
            task_results = []
            for task in tasks:
                task_results.append(self.gather_results_for_task(task))
            # progress CSV:
            pairs = self.pairs_from_task_results(task_results)
            csv_for_results = self.csv_for_results(self.compare_task_results_sizes(pairs))
            path = pathlib.Path("/tmp/some.csv")
            path.write_text(csv_for_results)

            time.sleep(1)
        self.progress("All threads returned")

        self.check_result_queue()

        if len(self.failure_exceptions):
            self.progress("Some threads failed:")
        for ex in self.failure_exceptions:
            print("Thread failure: %s" % str(ex))

    class Task():
        def __init__(self,
                     board: str,
                     commitish: str,
                     outdir: str,
                     vehicles_to_build: str,
                     extra_hwdef: str = None,
                     toolchain: str = None,
                     ) -> None:
            self.board = board
            self.commitish = commitish
            self.outdir = outdir
            self.vehicles_to_build = vehicles_to_build
            self.extra_hwdef_file = extra_hwdef
            self.toolchain: str = toolchain

        def __str__(self):
            return f"Task({self.board}, {self.commitish}, {self.outdir}, {self.vehicles_to_build}, {self.extra_hwdef_file} {self.toolchain})"  # NOQA:E501

    def run(self):
        '''run tests for boards and vehicles passed in constructor'''

        tmpdir = tempfile.mkdtemp()
        self.tmpdir = tmpdir

        self.master_commit = self.master_branch
        if self.use_merge_base:
            self.master_commit = self.find_git_branch_merge_base(self.branch, self.master_branch)
            self.progress("Using merge base (%s)" % self.master_commit)

        # create an array of tasks to run:
        tasks = []
        for board in self.board:
            board_info = self.boards_by_name[board]

            vehicles_to_build = self.vehicles_to_build_for_board_info(board_info)

            outdir_1 = os.path.join(tmpdir, "out-master-%s" % (board,))
            tasks.append(SizeCompareBranches.Task(
                board,
                self.master_commit,
                outdir_1,
                vehicles_to_build,
                extra_hwdef=self.extra_hwdef_master,
                toolchain=board_info.toolchain,
            ))
            outdir_2 = os.path.join(tmpdir, "out-branch-%s" % (board,))
            tasks.append(SizeCompareBranches.Task(
                board,
                self.branch,
                outdir_2,
                vehicles_to_build,
                extra_hwdef=self.extra_hwdef_branch,
                toolchain=board_info.toolchain,
            ))
        self.tasks = tasks

        if self.parallel_copies is not None:
            self.run_build_tasks_in_parallel(tasks)
            task_results = []
            for task in tasks:
                task_results.append(self.gather_results_for_task(task))
        else:
            # traditional build everything in sequence:
            task_results = []
            for task in tasks:
                self.run_build_task(task)
                task_results.append(self.gather_results_for_task(task))

                # progress CSV:
                with open("/tmp/some.csv", "w") as f:
                    pairs = self.pairs_from_task_results(task_results)
                    f.write(self.csv_for_results(self.compare_task_results_sizes(pairs)))

        return self.compare_task_results(task_results)

    def elf_diff_results(self, result_master, result_branch):
        master_branch = result_master.branch
        branch = result_branch.branch
        for vehicle_name in result_master.vehicle.keys():
            master_vehicle = result_master.vehicle[vehicle_name]
            elf_filename = master_vehicle["elf_filename"]
            master_elf_dir = master_vehicle["elf_dir"]
            branch_vehicle = result_branch.vehicle[vehicle_name]
            new_elf_dir = branch_vehicle["elf_dir"]
            board = result_master.board
            self.progress("Starting compare (~10 minutes!)")
            toolchain = result_master.toolchain
            if toolchain is None:
                toolchain = ""
            else:
                toolchain += "-"

            if self.bin_dir is None:
                self.bin_dir = self.find_bin_dir(toolchain_prefix=toolchain)
            if self.bin_dir is None:
                self.progress(f"Gtoolchain: {self.toolchain}")
                raise ValueError("Crap")

            elf_diff_commandline = [
                "time",
                "python3",
                "-m", "elf_diff",
                "--bin_dir", self.bin_dir,
                f'--bin_prefix={toolchain}',
                "--old_alias", "%s %s" % (master_branch, elf_filename),
                "--new_alias", "%s %s" % (branch, elf_filename),
                "--html_dir", "ELF_DIFF_%s_%s" % (board, vehicle_name),
            ]

            try:
                master_source_prefix = master_vehicle["source_path"]
                branch_source_prefix = branch_vehicle["source_path"]
                elf_diff_commandline.extend([
                    "--old_source_prefix", master_source_prefix,
                    "--new_source_prefix", branch_source_prefix,
                ])
            except KeyError:
                pass

            elf_diff_commandline.extend([
                os.path.join(master_elf_dir, elf_filename),
                os.path.join(new_elf_dir, elf_filename)
            ])

            self.run_program("SCB", elf_diff_commandline)

    def pairs_from_task_results(self, task_results: list):
        pairs = {}
        for res in task_results:
            board = res.board
            if board not in pairs:
                pairs[board] = {}
            if res.branch == self.master_commit:
                pairs[board]["master"] = res
            elif res.branch == self.branch:
                pairs[board]["branch"] = res
            else:
                raise ValueError(res["branch"])
        return pairs

    def compare_task_results(self, task_results):
        # pair off results, master and branch:
        pairs = self.pairs_from_task_results(task_results)

        self.emit_csv_for_results(self.compare_task_results_sizes(pairs))

        if self.run_elf_diff:
            self.compare_task_results_elf_diff(pairs)

        if self.features:
            self.compare_task_results_features(pairs)

    def compare_task_results_sizes(self, pairs):
        results = {}
        for pair in pairs.values():
            if "master" not in pair or "branch" not in pair:
                # probably incomplete:
                continue
            master = pair["master"]
            board = master.board
            try:
                results[board] = self.compare_results_sizes(master, pair["branch"])
            except FileNotFoundError:
                pass

        return results

    def compare_task_results_elf_diff(self, pairs):
        for pair in pairs.values():
            if "master" not in pair or "branch" not in pair:
                # probably incomplete:
                continue
            self.elf_diff_results(pair["master"], pair["branch"])

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
        headings = ["Board"] + sorted_all_vehicles
        ret += ",".join(headings) + "\n"
        for board in boards:
            line = [board]
            board_results = results[board]
            for vehicle in sorted_all_vehicles:
                cell_value = ""
                if vehicle in board_results:
                    result = board_results[vehicle]
                    if isinstance(result, FeatureCompareBranchesResult):
                        cell_value = '"' + "\n".join(result.delta_features_in + result.delta_features_out) + '"'
                    else:
                        if result.identical:
                            bytes_delta = "*"
                        else:
                            bytes_delta = result.bytes_delta
                        cell_value = bytes_delta
                line.append(str(cell_value))
            # do not add to ret value if we're not showing empty results:
            if not self.show_empty:
                if len(list(filter(lambda x : x != "", line[1:]))) == 0:
                    continue
            # do not add to ret value if all output binaries are identical:
            if not self.show_unchanged:
                starcount = len(list(filter(lambda x : x == "*", line[1:])))
                if len(line[1:]) == starcount:
                    continue
            ret += ",".join(line) + "\n"
        return ret

    def files_are_identical(self, file1, file2):
        '''returns true if the files have the same content'''
        return open(file1, "rb").read() == open(file2, "rb").read()

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
            with open(extra_hwdef, "r+b") as f:
                content += f.read()

        # spew content to single file:
        f = tempfile.NamedTemporaryFile(delete=False)
        f.write(content)
        f.close()

        return f.name

    def run_build_task(self, task, source_dir=None, jobs=None):
        self.progress(f"Building {task}")
        shutil.rmtree(task.outdir, ignore_errors=True)
        self.build_branch_into_dir(
            task.board,
            task.commitish,
            task.vehicles_to_build,
            task.outdir,
            source_dir=source_dir,
            extra_hwdef=self.extra_hwdef_file(task.extra_hwdef_file),
            jobs=jobs,
        )

    class Result():
        def __init__(self, board, branch, toolchain=None):
            self.board = board
            self.branch = branch
            self.toolchain = toolchain

            self.vehicle = {}

    def gather_results_for_task(self, task) -> Result:
        result = SizeCompareBranches.Result(
            task.board,
            task.commitish,
            toolchain=task.toolchain,
        )

        have_source_trees = self.parallel_copies is not None and len(self.tasks) <= self.parallel_copies

        for vehicle in task.vehicles_to_build:
            if vehicle == 'bootloader' and task.board in self.bootloader_blacklist:
                continue

            result.vehicle[vehicle] = {}
            v = result.vehicle[vehicle]

            elf_dirname = "bin"
            if vehicle == 'bootloader':
                # elfs for bootloaders are in the bootloader directory...
                elf_dirname = "bootloader"
            elf_basedir = task.outdir
            if have_source_trees:
                try:
                    v["source_path"] = pathlib.Path(task.outdir, "scb_sourcepath.txt").read_text()
                    elf_basedir = os.path.join(v["source_path"], 'build')
                    self.progress("Have source trees")
                except FileNotFoundError:
                    pass
            bin_dirname = "bin"
            bin_filename = self.vehicle_map[vehicle] + '.bin'
            elf_filename = self.vehicle_map[vehicle]
            esp32_elf_dirname = "esp-idf_build"
            if os.path.exists(os.path.join(elf_basedir, task.board, esp32_elf_dirname)):
                bin_filename = "ardupilot.bin"
                elf_dirname = esp32_elf_dirname
                bin_dirname = elf_dirname
                elf_filename = "ardupilot.elf"
            v["bin_dir"] = os.path.join(elf_basedir, task.board, bin_dirname)
            v["bin_filename"] = bin_filename
            v["elf_dir"] = os.path.join(elf_basedir, task.board, elf_dirname)
            v["elf_filename"] = elf_filename

        return result

    def create_stripped_elf(self, path, toolchain="arm-none-eabi"):
        stripped_path = f"{path}-stripped"
        shutil.copy(path, stripped_path)

        strip = "strip"
        if toolchain is not None:
            strip = "-".join([toolchain, strip])

        self.run_program("strip", [strip, stripped_path], show_command=False)

        return stripped_path

    def get_features(self, path):
        from extract_features import ExtractFeatures
        x = ExtractFeatures(path)
        return x.extract()

    def compare_results_features(self, result_master: Result, result_branch: Result):
        ret = {}
        for vehicle in result_master.vehicle.keys():
            # check for the difference in size (and identicality)
            # of the two binaries:
            master_elf_dir = result_master.vehicle[vehicle]["elf_dir"]
            new_elf_dir = result_branch.vehicle[vehicle]["elf_dir"]

            elf_filename = result_master.vehicle[vehicle]["elf_filename"]
            master_path = os.path.join(master_elf_dir, elf_filename)
            new_path = os.path.join(new_elf_dir, elf_filename)

            if not os.path.exists(master_path):
                continue
            if not os.path.exists(new_path):
                continue
            (master_features_in, master_features_out) = self.get_features(master_path)
            (new_features_in, new_features_out) = self.get_features(new_path)

            board = result_master.board
            in_delta = []
            for master_feature_in in sorted(master_features_in):
                if master_feature_in not in new_features_in:
                    in_delta.append("-" + master_feature_in)
            for new_feature_in in sorted(new_features_in):
                if new_feature_in not in master_features_in:
                    in_delta.append("+" + new_feature_in)

            out_delta = []
            for master_feature_out in sorted(master_features_out):
                if master_feature_out not in new_features_out:
                    out_delta.append("-!" + master_feature_out)
            for new_feature_out in sorted(new_features_out):
                if new_feature_out not in master_features_out:
                    out_delta.append("+!" + new_feature_out)

            ret[vehicle] = FeatureCompareBranchesResult(board, vehicle, in_delta, out_delta)

        return ret

    def compare_task_results_features(self, pairs):
        results = {}
        for pair in pairs.values():
            if "master" not in pair or "branch" not in pair:
                # probably incomplete:
                continue
            results[pair["master"].board] = self.compare_results_features(pair["master"], pair["branch"])
        print(self.csv_for_results(results))

    def compare_results_sizes(self, result_master, result_branch):
        ret = {}
        for vehicle in result_master.vehicle.keys():
            # check for the difference in size (and identicality)
            # of the two binaries:
            master_bin_dir = result_master.vehicle[vehicle]["bin_dir"]
            new_bin_dir = result_branch.vehicle[vehicle]["bin_dir"]

            try:
                bin_filename = result_master.vehicle[vehicle]["bin_filename"]
                master_path = os.path.join(master_bin_dir, bin_filename)
                new_path = os.path.join(new_bin_dir, bin_filename)
                master_size = os.path.getsize(master_path)
                new_size = os.path.getsize(new_path)
                identical = self.files_are_identical(master_path, new_path)
            except FileNotFoundError:
                elf_filename = result_master.vehicle[vehicle]["elf_filename"]
                master_path = os.path.join(master_bin_dir, elf_filename)
                new_path = os.path.join(new_bin_dir, elf_filename)
                master_size = os.path.getsize(master_path)
                new_size = os.path.getsize(new_path)

                identical = self.files_are_identical(master_path, new_path)
                if not identical:
                    # try stripping the files and *then* comparing.
                    # This treats symbol renames as then "identical".
                    master_path_stripped = self.create_stripped_elf(
                        master_path,
                        toolchain=result_master.toolchain,
                    )
                    new_path_stripped = self.create_stripped_elf(
                        new_path,
                        toolchain=result_branch.toolchain,
                    )
                    identical = self.files_are_identical(master_path_stripped, new_path_stripped)

            board = result_master.board
            ret[vehicle] = SizeCompareBranchesResult(board, vehicle, new_size - master_size, identical)

        return ret


def main():
    parser = optparse.OptionParser("size_compare_branches.py")
    parser.add_option("",
                      "--elf-diff",
                      action="store_true",
                      default=False,
                      help="run elf_diff on output files")
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
                      "--no-waf-consistent-builds",
                      action="store_true",
                      default=False,
                      help="do not use the --consistent-builds waf command-line option (for older branches)")
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
                      "--show-empty",
                      action='store_true',
                      default=False,
                      help="Show result lines even if no builds were done for the board")
    parser.add_option("",
                      "--hide-unchanged",
                      action='store_true',
                      default=False,
                      help="Hide binary-size-change results for any board where output binary is unchanged")
    parser.add_option("",
                      "--board",
                      action='append',
                      default=[],
                      help="board to build for")
    parser.add_option("",
                      "--extra-hwdef",
                      default=[],
                      action="append",
                      help="configure with this extra hwdef file")
    parser.add_option("",
                      "--extra-hwdef-branch",
                      default=[],
                      action="append",
                      help="configure with this extra hwdef file only on new branch")
    parser.add_option("",
                      "--extra-hwdef-master",
                      default=[],
                      action="append",
                      help="configure with this extra hwdef file only on merge/master branch")
    parser.add_option("",
                      "--all-boards",
                      action='store_true',
                      default=False,
                      help="Build all boards")
    parser.add_option("",
                      "--exclude-board-glob",
                      default=[],
                      action="append",
                      help="exclude any board which matches this pattern")
    parser.add_option(
        "",
        "--features",
        default=False,
        action="store_true",
        help="compare features",
    )
    parser.add_option("",
                      "--all-vehicles",
                      action='store_true',
                      default=False,
                      help="Build all vehicles")
    parser.add_option("",
                      "--parallel-copies",
                      type=int,
                      default=None,
                      help="Copy source dir this many times, build from those copies in parallel")
    parser.add_option("-j",
                      "--jobs",
                      type=int,
                      default=None,
                      help="Passed to waf configure -j; number of build jobs.  If running with --parallel-copies, this is divided by the number of remaining threads before being passed.")  # noqa
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
        extra_hwdef_branch=cmd_opts.extra_hwdef_branch,
        extra_hwdef_master=cmd_opts.extra_hwdef_master,
        run_elf_diff=(cmd_opts.elf_diff),
        all_vehicles=cmd_opts.all_vehicles,
        all_boards=cmd_opts.all_boards,
        exclude_board_glob=cmd_opts.exclude_board_glob,
        use_merge_base=not cmd_opts.no_merge_base,
        waf_consistent_builds=not cmd_opts.no_waf_consistent_builds,
        show_empty=cmd_opts.show_empty,
        show_unchanged=not cmd_opts.hide_unchanged,
        parallel_copies=cmd_opts.parallel_copies,
        jobs=cmd_opts.jobs,
        features=cmd_opts.features,
    )
    x.run()


if __name__ == '__main__':
    main()
