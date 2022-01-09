#!/usr/bin/env python

import os
import re

'''
list of boards for build_binaries.py and custom build server

AP_FLAKE8_CLEAN
'''


class Board(object):
    def __init__(self, name):
        self.name = name
        self.is_ap_periph = False


class BoardList(object):

    def __init__(self):
        self.hwdef_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "..", "libraries", "AP_HAL_ChibiOS", "hwdef")

        if not os.path.exists(self.hwdef_dir):
            raise ValueError("%s does not exist" % self.hwdef_dir)

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

    def find_autobuild_boards(self):
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
            "skyviper-journey",
            "skyviper-v2450",
            "CubeGreen-solo",
            "omnibusf4pro-one",
            "CubeSolo",
            "MazzyStarDrone",
            "fmuv3-bdshot",
            "CubeYellow-bdshot",
            "crazyflie2",
            "NucleoH743",
            "Pixhawk1-1M-bdshot",
            "Nucleo-G491",
            "fmuv5-bdshot",
            "KakuteF7-bdshot",
            "iomcu",
            "luminousbee4",
            "skyviper-f412-rev1",
            "CubeOrange-joey",
            "OMNIBUSF7V2-bdshot",
            'H757I_EVAL',
            'H757I_EVAL_intf',
            'iomcu_f103_8MHz',
        ]

        ret = filter(lambda x : x not in blacklist, ret)

        return list(ret)

    def find_ap_periph_boards(self):
        blacklist = [
            "Pixracer-periph",
            "f103-Trigger",
            "H757I_EVAL",
            "HerePro",
            "HereID",
            "G4-ESC",
            "CubeOrange-periph-heavy",
            "f103-HWESC",
            "Nucleo-L476",
            "Nucleo-L496",
        ]
        ret = []
        for x in self.boards:
            if not x.is_ap_periph:
                continue
            if x.name in blacklist:
                continue
            ret.append(x.name)
        return list(ret)


AUTOBUILD_BOARDS = BoardList().find_autobuild_boards()
AP_PERIPH_BOARDS = BoardList().find_ap_periph_boards()
