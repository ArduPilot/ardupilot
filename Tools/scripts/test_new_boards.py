#!/usr/bin/env python3

from __future__ import annotations

'''
Look at the difference between this and its merge-base with master

for each new hwdef file, build the board

AP_FLAKE8_CLEAN
'''

import optparse
import os
import shutil

from dataclasses import dataclass
from typing import Set

import board_list
from build_script_base import BuildScriptBase


class TestNewBoards(BuildScriptBase):
    def __init__(self, master_branch: str = "master") -> None:
        super().__init__()
        self.master_branch: str = master_branch

    def progress_prefix(self) -> str:
        '''pretty-print progress'''
        return 'TNB'

    def get_new_hwdef_paths(self) -> Set[str]:
        '''returns list of newly added hwdef filepaths relative to the root.
        Only .dat files for both main and bootloader firmwares'''
        current_commitish = self.find_current_git_branch_or_sha1()
        merge_base = self.find_git_branch_merge_base(current_commitish, self.master_branch)
        delta_files = self.run_git([
            'diff',
            '--name-status',
            f"{merge_base}",
        ], show_output=False)

        ret = set()

        for line in delta_files.split("\n"):
            if not line:
                continue
            parts = line.split(None, 1)
            if len(parts) != 2:
                continue
            status, filepath = parts
            # Only process added files (status 'A')
            if status != 'A':
                continue
            if "/hwdef/" not in filepath:
                continue
            if not filepath.endswith(("hwdef.dat", "hwdef-bl.dat")):
                continue

            ret.add(filepath)

        return ret

    def build_board(self, board : board_list.Board):
        self.run_waf(["configure", "--board", board.name], show_output=False)
        if board.is_ap_periph:
            self.run_waf(["AP_Periph"], show_output=False)
        else:
            for t in board.autobuild_targets:
                t = t.lower()
                if t == 'tracker':
                    t = 'antennatracker'
                self.run_waf([t], show_output=False)

    def build_bootloader(self, board : board_list.Board) -> None:
        if not board.is_ap_periph:
            # hopefully temporary hack so you can build bootloader
            # after building other vehicles without a clean:
            dsdl_generated_path = os.path.join('build', board.name, "modules", "DroneCAN", "libcanard", "dsdlc_generated")
            self.progress(f"HACK: Removing ({dsdl_generated_path})")
            shutil.rmtree(dsdl_generated_path, ignore_errors=True)
        self.run_waf([
            "configure",
            "--board",
            board.name,
            "--bootloader"
        ], show_output=False)
        self.run_waf(["bootloader"], show_output=False)

    def run(self) -> None:
        hwdef_filepaths = self.get_new_hwdef_paths()

        @dataclass
        class BoardToTest():
            test_bootloader: bool = False
            test_vehicles: bool = False

        # Extract unique board names and track if bootloader hwdef was added
        boards_to_test = {}  # board_name -> has_bootloader
        for hwdef_filepath in hwdef_filepaths:
            self.progress(f"New hwdef: {hwdef_filepath}")
            # Extract board name from path like libraries/AP_HAL_ChibiOS/hwdef/BoardName/hwdef.dat
            parts = hwdef_filepath.split('/')
            if 'hwdef' not in parts:
                raise ValueError(f"hwdef not found in path: {hwdef_filepath}")
            hwdef_idx = parts.index('hwdef')
            if hwdef_idx + 1 >= len(parts):
                raise ValueError(f"No board name after hwdef in path: {hwdef_filepath}")
            board_name = parts[hwdef_idx + 1]

            if board_name not in boards_to_test:
                boards_to_test[board_name] = BoardToTest()
            if hwdef_filepath.endswith('hwdef-bl.dat'):
                boards_to_test[board_name].test_bootloader = True
            else:
                boards_to_test[board_name].test_vehicles = True

        # Build each unique board (find it in board list as an additional check)
        bl = board_list.BoardList()

        # First pass: check for README.md for non-AP_Periph boards
        for hwdef_filepath in hwdef_filepaths:
            # Extract board name from path like libraries/AP_HAL_ChibiOS/hwdef/BoardName/hwdef.dat
            parts = hwdef_filepath.split('/')
            hwdef_idx = parts.index('hwdef')
            board_name = parts[hwdef_idx + 1]

            # Find the board object to check if it's AP_Periph
            board = None
            for b in bl.boards:
                if b.name == board_name:
                    board = b
                    break

            if board is None:
                raise ValueError(f"Board {board_name} not found in board list")

            # Only require README.md for non-AP_Periph boards
            if not board.is_ap_periph:
                hwdef_dir = os.path.join(*parts[:hwdef_idx + 2])
                readme_path = os.path.join(hwdef_dir, 'README.md')
                if not os.path.exists(readme_path):
                    raise ValueError(f"Missing README.md for new board: {readme_path} does not exist")
        for board_name in sorted(boards_to_test.keys()):
            # Find the board object
            board = None
            for b in bl.boards:
                if b.name == board_name:
                    board = b
                    break

            if board is None:
                raise ValueError(f"Board {board_name} not found in board list")

            # Skip ESP32 boards - CI machine can't build them
            if board.hal == "ESP32":
                self.progress(f"Skipping ESP32 board {board.name}")
                continue

            self.progress(f"Building board {board.name}")
            if boards_to_test[board_name].test_vehicles:
                self.build_board(board)

            # Build bootloader if a bootloader hwdef file was added
            if boards_to_test[board_name].test_bootloader:
                self.progress(f"Building bootloader for {board.name}")
                self.build_bootloader(board)


def main():
    parser = optparse.OptionParser("test_new_boards.py")
    parser.add_option("",
                      "--master-branch",
                      type="string",
                      default="master",
                      help="master branch to use")
    cmd_opts, cmd_args = parser.parse_args()

    tnb = TestNewBoards(
        master_branch=cmd_opts.master_branch,
    )
    tnb.run()


if __name__ == '__main__':
    main()
