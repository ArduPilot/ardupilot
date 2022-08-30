#!/usr/bin/env python

import os
import re
import fnmatch

'''
list of boards for build_binaries.py and custom build server

AP_FLAKE8_CLEAN
'''


class Board(object):
    def __init__(self, name):
        self.name = name
        self.is_ap_periph = False
        self.autobuild_targets = [
            'Tracker',
            'Blimp',
            'Copter',
            'Heli',
            'Plane',
            'Rover',
            'Sub',
        ]


def in_blacklist(blacklist, b):
    '''return true if board b is in the blacklist, including wildcards'''
    for bl in blacklist:
        if fnmatch.fnmatch(b, bl):
            return True
    return False


class BoardList(object):

    def set_hwdef_dir(self):
        self.hwdef_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "..", "libraries", "AP_HAL_ChibiOS", "hwdef")

        if os.path.exists(self.hwdef_dir):
            return

        self.hwdef_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "libraries", "AP_HAL_ChibiOS", "hwdef")

        if os.path.exists(self.hwdef_dir):
            # we're on the autotest server and have been copied in
            # to the APM root directory
            return

        raise ValueError("Did not find hwdef_dir")

    def __init__(self):
        self.set_hwdef_dir()

        # no hwdefs for Linux boards - yet?
        self.boards = [
            Board("erlebrain2"),
            Board("navigator"),
            Board("navio"),
            Board("navio2"),
            Board("edge"),
            Board("obal"),
            Board("pxf"),
            Board("bbbmini"),
            Board("blue"),
            Board("pxfmini"),
            Board("SITL_x86_64_linux_gnu"),
            Board("SITL_arm_linux_gnueabihf"),
            Board("SITL_webassembly"),
        ]

        for adir in os.listdir(self.hwdef_dir):
            if adir is None:
                continue
            if not os.path.isdir(os.path.join(self.hwdef_dir, adir)):
                continue
            if adir in ["scripts", "common", "STM32CubeConf"]:
                continue
            filepath = os.path.join(self.hwdef_dir, adir, "hwdef.dat")
            if not os.path.exists(filepath):
                continue
            filepath = os.path.join(self.hwdef_dir, adir, "hwdef.dat")
            text = self.read_hwdef(filepath)

            board = Board(adir)
            self.boards.append(board)
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

    def find_autobuild_boards(self, build_target=None):
        ret = []
        for board in self.boards:
            if board.is_ap_periph:
                continue
            ret.append(board.name)

        # these were missing in the original list for unknown reasons.
        # Omitting them for backwards-compatability here - but we
        # should probably have a line in the hwdef indicating they
        # shouldn't be auto-built...
        blacklist = [
            # IOMCU:
            "iomcu",
            'iomcu_f103_8MHz',

            # bdshot
            "fmuv3-bdshot",
            "OMNIBUSF7V2-bdshot",

            # renamed to KakuteH7Mini-Nand
            "KakuteH7Miniv2",

            # other
            "crazyflie2",
            "CubeOrange-joey",
            "luminousbee4",
            "MazzyStarDrone",
            "omnibusf4pro-one",
            "skyviper-f412-rev1",
            "*-ODID",
            "*-ODID-heli",
        ]

        ret = filter(lambda x : not in_blacklist(blacklist, x), ret)

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

    def find_ap_periph_boards(self):
        blacklist = [
            "CubeOrange-periph-heavy",
            "f103-HWESC",
            "f103-Trigger",
            "G4-ESC",
            "HereID",
            "HerePro",
        ]
        ret = []
        for x in self.boards:
            if not x.is_ap_periph:
                continue
            if x.name in blacklist:
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
