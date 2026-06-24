#!/usr/bin/env python3

'''
Look at the difference between this and its merge-base with master

for each new hwdef file, build the board.

Also validates each new board directory:
  - it contains a README.md (except ODID variants, which are exempt)
  - that README.md references at least one image added in this branch
    (except AP_Periph boards, which need only the README, no image)
  - a new bootloader hwdef (hwdef-bl.dat) has a matching prebuilt binary
    committed at Tools/bootloaders/<board>_bl.bin
  - defaults.parm does not set parameters that belong in hwdef.dat

AP_FLAKE8_CLEAN
'''

from __future__ import annotations

import optparse
import os
import re
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

    def get_added_files(self) -> Set[str]:
        '''returns the set of files newly added (git status 'A') relative to the
        merge-base with master, as paths relative to the repo root'''
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
            ret.add(filepath)

        return ret

    def get_added_hwdef_paths(self) -> Set[str]:
        '''returns the newly added hwdef filepaths relative to the repo root.
        Only .dat files for both main and bootloader firmwares'''
        ret = set()
        for filepath in self.get_added_files():
            if "/hwdef/" not in filepath:
                continue
            if not filepath.endswith(("hwdef.dat", "hwdef-bl.dat")):
                continue
            ret.add(filepath)

        return ret

    # Each tuple: (param name regex, re.sub template producing the hwdef define name)
    DEFAULTS_PARM_CHECKS = [
        (r'^SERIAL(\d+)_(PROTOCOL|BAUD|OPTIONS)$', r'DEFAULT_SERIAL\1_\2'),
        (r'^ADSB_TYPE$',                           r'AP_ADSB_TYPE_DEFAULT'),
        (r'^NTF_LED_TYPES$',                       r'NTF_LED_TYPES'),
        (r'^BRD_TYPE$',                            r'BOARD_TYPE_DEFAULT'),
    ]

    def check_defaults_parm(self, defaults_parm_path: str, board_name: str) -> None:
        issues = []
        with open(defaults_parm_path) as f:
            for line in f:
                line = line.split('#')[0].strip()
                if not line:
                    continue
                m = re.match(r'^(\w+)[\s,]+(\S+)', line)
                if not m:
                    continue
                param, value = m.group(1), m.group(2)
                for pattern, template in self.DEFAULTS_PARM_CHECKS:
                    if re.match(pattern, param):
                        hwdef_name = re.sub(pattern, template, param)
                        issues.append(
                            f"  {param} {value}"
                            f"  ->  add to hwdef.dat: define {hwdef_name} {value}"
                        )
                        break
        if issues:
            raise ValueError(
                f"defaults.parm for board {board_name} contains parameters that must be "
                f"defined in hwdef.dat instead:\n" + "\n".join(issues)
            )

    # markdown image reference: ![alt](path), capturing the path
    IMG_MD_RE = re.compile(r'!\[[^\]]*\]\(([^)#?\s]+)')

    BOOTLOADER_DIR = 'Tools/bootloaders'

    def check_new_bootloader_binary(self, board_name: str, added_files: Set[str]) -> None:
        '''A board that introduces a new bootloader hwdef (hwdef-bl.dat) must also
        commit the matching prebuilt bootloader binary into Tools/bootloaders/.'''
        binary = f"{self.BOOTLOADER_DIR}/{board_name}_bl.bin"
        if binary not in added_files:
            raise ValueError(
                f"new bootloader hwdef for board {board_name} requires its prebuilt "
                f"bootloader binary to be committed at {binary}; build it with "
                f"Tools/scripts/build_bootloaders.py {board_name}"
            )

    def is_odid_board(self, board: board_list.Board) -> bool:
        '''True if the board enables OpenDroneID in its hwdef'''
        hwdef = board.get_hwdef()
        return hwdef.intdefines.get('AP_OPENDRONEID_ENABLED', 0) == 1

    def check_new_board_readme(self, board_name: str, hwdef_dir: str, added_files: Set[str],
                               image_required: bool = True) -> None:
        '''A new board directory must contain a README.md.  Unless image_required
        is False, that README must also reference at least one local image that is
        also added in this branch (AP_Periph boards need only the README).'''
        readme_path = os.path.join(hwdef_dir, 'README.md')
        if not os.path.exists(readme_path):
            raise ValueError(f"Missing README.md for new board: {readme_path} does not exist")

        if not image_required:
            return

        with open(readme_path, encoding="utf-8", errors="replace") as fh:
            content = fh.read()

        local_refs = []
        for m in self.IMG_MD_RE.finditer(content):
            src = m.group(1).strip()
            if not src.startswith(("http://", "https://", "//")):
                local_refs.append(src)

        if not local_refs:
            raise ValueError(
                f"README.md for new board {board_name} contains no local image references"
            )

        found = any(
            os.path.normpath(os.path.join(hwdef_dir, src)) in added_files
            for src in local_refs
        )
        if not found:
            resolved = "\n".join(
                f"  {src!r} -> {os.path.normpath(os.path.join(hwdef_dir, src))}"
                for src in local_refs
            )
            raise ValueError(
                f"README.md for new board {board_name} references no image added in "
                f"this branch:\n{resolved}"
            )

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
        added_files = self.get_added_files()
        hwdef_filepaths = self.get_added_hwdef_paths()

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

        # First pass: validate README.md and defaults.parm for each new board.
        # A board directory may contribute both hwdef.dat and hwdef-bl.dat, so
        # only validate each directory once.
        checked_board_dirs = set()
        for hwdef_filepath in hwdef_filepaths:
            # Extract board name from path like libraries/AP_HAL_ChibiOS/hwdef/BoardName/hwdef.dat
            parts = hwdef_filepath.split('/')
            hwdef_idx = parts.index('hwdef')
            board_name = parts[hwdef_idx + 1]

            # Find the board object to check if it's AP_Periph / ODID
            board = None
            for b in bl.boards:
                if b.name == board_name:
                    board = b
                    break

            if board is None:
                raise ValueError(f"Board {board_name} not found in board list")

            hwdef_dir = os.path.join(*parts[:hwdef_idx + 2])
            if hwdef_dir in checked_board_dirs:
                continue
            checked_board_dirs.add(hwdef_dir)

            # ODID variants are exempt from the README requirement entirely.
            # AP_Periph boards must ship a README but are not required to embed
            # an image.  All other boards require both a README and an image.
            if self.is_odid_board(board):
                self.progress(f"README.md not required for {board_name} (ODID)")
            else:
                self.check_new_board_readme(
                    board_name, hwdef_dir, added_files,
                    image_required=not board.is_ap_periph,
                )

            defaults_parm_path = os.path.join(hwdef_dir, 'defaults.parm')
            if os.path.exists(defaults_parm_path):
                self.check_defaults_parm(defaults_parm_path, board_name)

        # A board that adds a new bootloader hwdef must also commit its prebuilt
        # bootloader binary into Tools/bootloaders/.
        for board_name, board_to_test in sorted(boards_to_test.items()):
            if board_to_test.test_bootloader:
                self.check_new_bootloader_binary(board_name, added_files)

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

            # Skip QURT boards - CI machine can't build them
            if board.hal == "QURT":
                self.progress(f"Skipping QURT board {board.name}")
                continue

            # Skip arms board - CI machine can't build it
            if board.toolchain == "arm-linux-gnueabihf":
                self.progress(f"Skipping arm-linux board {board.name}")
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
