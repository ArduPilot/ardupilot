#!/usr/bin/env python3

'''
Script to build the current checkout for a set of boards - potentially
all of them.  Builds the working tree as-is; no git operations are
done.  A summary of failed builds is printed at exit.

AP_FLAKE8_CLEAN

How to use?
Starting in the ardupilot directory.
~/ardupilot $ python Tools/scripts/build_boards.py --board=MatekF405-Wing --vehicle=plane
~/ardupilot $ python Tools/scripts/build_boards.py --all-boards --all-vehicles --parallel-copies=4
'''

from __future__ import annotations

import argparse
import copy
import os
import shutil
import sys
import tempfile

from build_script_base import BuildScriptBase


class BuildBoards(BuildScriptBase):
    '''script to build a set of boards in the current checkout'''

    def __init__(self,
                 board: list | None = None,
                 vehicle: list | None = None,
                 all_boards=False,
                 all_vehicles=False,
                 exclude_board_glob: list | None = None,
                 extra_hwdef: list | None = None,
                 parallel_copies=None,
                 jobs=None,
                 configure_only=False,
                 progress_file=None,
                 ):
        super().__init__(progress_file=progress_file)

        if board is None:
            board = []
        if configure_only and vehicle is None and not all_vehicles:
            raise ValueError("configure-only requires vehicles to be specified")
        if vehicle is None:
            vehicle = ["plane"]
        if exclude_board_glob is None:
            exclude_board_glob = []
        if extra_hwdef is None:
            extra_hwdef = []

        if not board and not all_boards:
            raise ValueError("Must supply --board or --all-boards")

        self.board = board
        self.vehicle = vehicle
        self.extra_hwdef = extra_hwdef
        self.parallel_copies = parallel_copies
        self.jobs = jobs
        self.configure_only = configure_only

        self.resolve_board_and_vehicle_lists(
            all_boards=all_boards,
            all_vehicles=all_vehicles,
            exclude_board_glob=exclude_board_glob,
        )

        self.bootloader_blacklist = self.make_bootloader_blacklist()

    def progress_prefix(self):
        return 'BB'

    def write_task_progress(self, all_tasks, remaining_tasks):
        '''write task counts, remaining tasks and failures to the
        progress file, if one was specified'''
        if self.progress_file is None:
            return
        content = (f"total-tasks={len(all_tasks)} " +
                   f"remaining-tasks={len(remaining_tasks)} " +
                   f"failures={len(self.failure_exceptions)}\n")
        for t in remaining_tasks:
            content += f"remaining: {t}\n"
        for f in self.failure_exceptions:
            content += f"failure: {f}\n"
        self.write_progress_file(content)

    def parallel_progress_hook(self, tasks):
        self.write_task_progress(tasks, copy.copy(self.parallel_tasks))

    class Task():
        def __init__(self, board: str, vehicles_to_build: list) -> None:
            self.board = board
            self.vehicles_to_build = vehicles_to_build

        def __str__(self):
            return f"Task({self.board}, {self.vehicles_to_build})"

    def build_vehicles_for_board(self, board, vehicles, source_dir=None, jobs=None):
        waf_configure_args = ["configure", "--board", board]

        extra_hwdef = self.extra_hwdef_file([])
        if extra_hwdef is not None:
            waf_configure_args.extend(["--extra-hwdef", extra_hwdef])

        if jobs is None:
            jobs = self.jobs
        if jobs is not None:
            waf_configure_args.extend(["-j", str(jobs)])

        # we can't run `./waf copter blimp plane` without error, so do
        # them one-at-a-time:
        non_bootloader_configure_done : bool = False
        for v in vehicles:
            if v == 'bootloader':
                # need special configuration directive
                continue
            if not non_bootloader_configure_done:
                self.run_waf(waf_configure_args, show_output=False, source_dir=source_dir)
                non_bootloader_configure_done = True
            if self.configure_only:
                continue
            self.run_waf([v], show_output=False, source_dir=source_dir)
        for v in vehicles:
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
            if self.configure_only:
                continue
            self.run_waf([v], show_output=False, source_dir=source_dir)

    def run_build_task(self, task, source_dir=None, jobs=None):
        self.progress(f"Building {task}")
        self.build_vehicles_for_board(
            task.board,
            task.vehicles_to_build,
            source_dir=source_dir,
            jobs=jobs,
        )

    def run(self):
        '''build for boards and vehicles passed in constructor; returns
        a list of failed tasks (as strings)'''

        self.tmpdir = tempfile.mkdtemp()

        tasks = []
        for board in self.board:
            board_info = self.boards_by_name[board]
            vehicles_to_build = self.vehicles_to_build_for_board_info(board_info)
            if not vehicles_to_build:
                self.progress(f"Skipping {board}: no requested vehicle is built for this board")
                continue
            tasks.append(BuildBoards.Task(board, vehicles_to_build))

        if self.parallel_copies is not None:
            self.run_build_tasks_in_parallel(tasks)
            failures = self.failure_exceptions
        else:
            # traditional build everything in sequence:
            self.failure_exceptions = []
            remaining = copy.copy(tasks)
            for task in tasks:
                try:
                    self.run_build_task(task)
                except Exception as ex:  # noqa: BLE001 — record failure, continue with remaining boards
                    self.progress(f"Build failed: {task} ({ex})")
                    self.failure_exceptions.append(f"{task}")
                remaining.pop(0)
                self.write_task_progress(tasks, remaining)
            failures = self.failure_exceptions

        return failures


def main():
    parser = argparse.ArgumentParser(description='build ArduPilot for a set of boards')
    parser.add_argument("--board",
                        action='append',
                        default=[],
                        help="board to build for; comma-separated and may be passed multiple times")
    parser.add_argument("--all-boards",
                        action='store_true',
                        default=False,
                        help="build all boards")
    parser.add_argument("--vehicle",
                        action='append',
                        default=[],
                        help="vehicle to build for; comma-separated and may be passed multiple times (default plane)")
    parser.add_argument("--all-vehicles",
                        action='store_true',
                        default=False,
                        help="build all vehicles")
    parser.add_argument("--exclude-board-glob",
                        action='append',
                        default=[],
                        help="exclude any board which matches this pattern")
    parser.add_argument("--extra-hwdef",
                        action='append',
                        default=[],
                        help="configure with this extra hwdef file")
    parser.add_argument("--parallel-copies",
                        type=int,
                        default=None,
                        help="copy source dir this many times, build from those copies in parallel")
    parser.add_argument("-j", "--jobs",
                        type=int,
                        default=None,
                        help="passed to waf configure -j; number of build jobs.  If running with --parallel-copies, this is divided by the number of remaining threads before being passed.")  # noqa
    parser.add_argument("--configure-only",
                        action='store_true',
                        default=False,
                        help="only run waf configure for each board, do not build")
    parser.add_argument("--progress-file",
                        type=str,
                        default=None,
                        help="file to write task progress and failures to as builds complete")
    args = parser.parse_args()

    if args.configure_only and len(args.vehicle) == 0 and not args.all_vehicles:
        parser.error("--configure-only requires --vehicle or --all-vehicles to be specified")

    vehicle = []
    for v in args.vehicle:
        vehicle.extend(v.split(','))
    if len(vehicle) == 0:
        vehicle.append("plane")

    board = []
    for b in args.board:
        board.extend(b.split(','))

    if len(board) == 0 and not args.all_boards:
        parser.error("Must supply --board or --all-boards")

    bb = BuildBoards(
        board=board,
        vehicle=vehicle,
        all_boards=args.all_boards,
        all_vehicles=args.all_vehicles,
        exclude_board_glob=args.exclude_board_glob,
        extra_hwdef=args.extra_hwdef,
        parallel_copies=args.parallel_copies,
        jobs=args.jobs,
        configure_only=args.configure_only,
        progress_file=args.progress_file,
    )
    failures = bb.run()
    if failures:
        print("Failed builds:")
        for failure in failures:
            print(f"  {failure}")
        sys.exit(1)


if __name__ == '__main__':
    main()
