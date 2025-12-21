#!/usr/bin/env python3

import os
import re
import fnmatch
from collections.abc import Collection

'''
list of boards for build_binaries.py and custom build server

AP_FLAKE8_CLEAN
'''


class Board(object):
    def __init__(self, name):
        self.name = name
        self.is_ap_periph = False
        self.toolchain = 'arm-none-eabi'  # FIXME: try to remove this?
        self.hal = None  # filled in below
        self.autobuild_targets = [
            'Tracker',
            'Blimp',
            'Copter',
            'Heli',
            'Plane',
            'Rover',
            'Sub',
        ]
        SITL_toolchain = {
            "SITL_x86_64_linux_gnu": "x86_64-linux-gnu",
            "SITL_arm_linux_gnueabihf": "arm-linux-gnueabihf",
        }
        if name in SITL_toolchain:
            self.toolchain = SITL_toolchain[name]


def in_boardlist(boards : Collection[str], board : str) -> bool:
    '''return true if board is in a collection of wildcard patterns'''
    for pattern in boards:
        if fnmatch.fnmatch(board, pattern):
            return True
    return False


class BoardList(object):

    def set_hwdef_dir(self):
        # work out wheer the hwdef files exist.  This file
        # (board_list.py) is copied into place on the autotest server,
        # so it isn't always in the same relative position to the
        # hwdef directories!
        found = False
        for relpath_bit in [
                os.path.join("..", "..", "libraries"),
                'libraries',
        ]:
            probe = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                relpath_bit, "AP_HAL_ChibiOS", "hwdef"
            )
            if os.path.exists(probe):
                found = True
                break

        if not found:
            raise ValueError("Did not find hwdef_dir")

        realpath = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            relpath_bit
        )

        self.hwdef_dir = []
        for haldir in 'AP_HAL_ChibiOS', 'AP_HAL_Linux', 'AP_HAL_ESP32':
            self.hwdef_dir.append(os.path.join(realpath, haldir, "hwdef"))

    def __init__(self):
        self.set_hwdef_dir()

        # no hwdefs for Linux boards - yet?
        self.boards = [
            Board("SITL_x86_64_linux_gnu"),
            Board("SITL_arm_linux_gnueabihf"),
        ]
        for b in self.boards:
            b.hal = "AP_HAL_SITL"

        for hwdef_dir in self.hwdef_dir:
            self.add_hwdefs_from_hwdef_dir(hwdef_dir)

    def add_hwdefs_from_hwdef_dir(self, hwdef_dir):
        for adir in os.listdir(hwdef_dir):
            if adir is None:
                continue
            if not os.path.isdir(os.path.join(hwdef_dir, adir)):
                continue
            if adir in ["scripts", "common", "STM32CubeConf"]:
                continue
            filepath = os.path.join(hwdef_dir, adir, "hwdef.dat")
            if not os.path.exists(filepath):
                continue
            filepath = os.path.join(hwdef_dir, adir, "hwdef.dat")

            # FIXME: we really should be using hwdef.py to parse
            # these, but it's too slow.  We use board_list in some
            # places we can't afford to be slow.
            text = self.read_hwdef(filepath)

            board = Board(adir)
            self.boards.append(board)
            board_toolchain_set = False
            for line in text:
                if re.match(r"^\s*env AP_PERIPH 1", line):
                    board.is_ap_periph = 1
                if re.match(r"^\s*env AP_PERIPH_HEAVY 1", line):
                    board.is_ap_periph = 1

                # a hwdef can specify which vehicles this target is valid for:
                match = re.match(r"AUTOBUILD_TARGETS\s*(.*)", line)
                if match is not None:
                    mname = match.group(1)
                    if mname.lower() == 'none':
                        board.autobuild_targets = []
                    else:
                        board.autobuild_targets = [
                            x.rstrip().lstrip().lower() for x in mname.split(",")
                        ]

                m = re.match(r"\s*env\s*TOOLCHAIN\s*([-\w]+)\s*", line)
                if m is not None:
                    board.toolchain = m.group(1)
                    board_toolchain_set = True
                    if board.toolchain == 'native':
                        board.toolchain = None

            # toolchain not in hwdef; make up some defaults:
            if not board_toolchain_set:
                if "Linux" in hwdef_dir:
                    board.toolchain = 'arm-linux-gnueabihf'
                elif "ChibiOS" in hwdef_dir:
                    board.toolchain = 'arm-none-eabi'
                elif "ESP32" in hwdef_dir:
                    board.toolchain = 'xtensa-esp32-elf'
                else:
                    raise ValueError(f"Unable to determine toolchain for {hwdef_dir}")

            if "Linux" in hwdef_dir:
                board.hal = "Linux"
            elif "ChibiOS" in hwdef_dir:
                board.hal = "ChibiOS"
            elif "ESP32" in hwdef_dir:
                board.hal = "ESP32"
            else:
                raise ValueError(f"Unable to determine HAL for {hwdef_dir}")

    def read_hwdef(self, filepath):
        fh = open(filepath)
        ret = []
        text = fh.readlines()
        for line in text:
            m = re.match(r"^\s*include\s+(.+)\s*$", line)
            if m is not None:
                ret += self.read_hwdef(os.path.join(os.path.dirname(filepath), m.group(1)))
            else:
                ret += [line]
        return ret

    def find_autobuild_boards(self, build_target=None, skip : Collection[str] = None):
        ret = []
        for board in self.boards:
            if board.is_ap_periph:
                continue
            ret.append(board.name)

        # these were missing in the original list for unknown reasons.
        # Omitting them for backwards-compatability here - but we
        # should probably have a line in the hwdef indicating they
        # shouldn't be auto-built...
        if skip is None:
            skip = [
                # IOMCU:
                "iomcu",
                'iomcu_f103_8MHz',

                # bdshot
                "fmuv3-bdshot",

                # renamed to KakuteH7Mini-Nand
                "KakuteH7Miniv2",

                # renamed to AtomRCF405NAVI
                "AtomRCF405"

                # other
                "crazyflie2",
                "CubeOrange-joey",
                "luminousbee4",
                "MazzyStarDrone",
                "omnibusf4pro-one",
                "skyviper-f412-rev1",
                "SkystarsH7HD",
                "*-ODID",
                "*-ODID-heli",
            ]

        ret = filter(lambda x : not in_boardlist(skip, x), ret)

        # if the caller has supplied a vehicle to limit to then we do that here:
        if build_target is not None:
            # Slow down: n^2 algorithm ahead
            newret = []
            for x in ret:
                for b in self.boards:
                    if b.name.lower() != x.lower():
                        continue
                    if build_target.lower() not in [y.lower() for y in b.autobuild_targets]:
                        continue
                    newret.append(x)
            ret = newret

        return sorted(list(ret))

    def find_ap_periph_boards(self, skip : Collection[str] = None):
        if skip is None:
            skip = [
                "CubeOrange-periph-heavy",
                "f103-HWESC",
                "f103-Trigger",
                "G4-ESC",
            ]
        ret = []
        for x in self.boards:
            if not x.is_ap_periph:
                continue
            if in_boardlist(skip, x.name):
                continue
            ret.append(x.name)
        return sorted(list(ret))


AUTOBUILD_BOARDS = BoardList().find_autobuild_boards()
AP_PERIPH_BOARDS = BoardList().find_ap_periph_boards()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='list boards to build')

    parser.add_argument('target')
    parser.add_argument('--per-line', action='store_true', default=False, help='list one per line for use with xargs')
    args = parser.parse_args()
    board_list = BoardList()
    target = args.target
    if target == "AP_Periph":
        blist = board_list.find_ap_periph_boards()
    else:
        blist = board_list.find_autobuild_boards(target)
    blist = sorted(blist)
    if args.per_line:
        for b in blist:
            print(b)
    else:
        print(blist)
