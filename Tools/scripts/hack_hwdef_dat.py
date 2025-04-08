#!/usr/bin/env python3

"""
An example script for reading in a hwdef.dat and munging it a little

AP_FLAKE8_CLEAN
"""

import re


class HackHWDef():
    def __init__(self, filepath):
        self.filepath = filepath

    def run(self):
        f = open(self.filepath, "r")
        lines = f.readlines()

        VTX_PWR_pin = None

        for line in lines:
            # print(f"line: {line}", end="")
            if re.match(" *#.*", line):
                # all comment
                continue
            if re.match(".*VTX_PWR.*", line) is None:
                continue
            m = re.match(".*VTX_PWR.*GPIO[(]([0-9]+)[)]", line)
            if m is None:
                raise ValueError(f"Did not find GPIO on line {line}")
            VTX_PWR_pin = m.group(1)

        if VTX_PWR_pin is None:
            return

        # see if we have (eg.) define RELAY2_PIN_DEFAULT 81
        RELAYn = None
        for line in lines:
            if re.match(" *#.*", line):
                # all comment
                continue
            m = re.match("define RELAY([0-9]+)_PIN_DEFAULT %s" % VTX_PWR_pin, line)
            if m is None:
                continue
            RELAYn = m.group(1)

        if RELAYn is None:
            raise ValueError("No RELAYn_PIN_DEFAULT for VTX_PWR=%s" % VTX_PWR_pin)

        # see if we have (eg.) define RELAY2_FUNCTION 2
        print(f"relay pin is {RELAYn=}")
        for line in lines:
            if re.match(" *#.*", line):
                # all comment
                continue
            m = re.match("define RELAY%s_FUNCTION.*" % RELAYn, line)
            if m is not None:
                # we have the function line
                return

        # we need the line...
        outlines = []
        for line in lines:
            outlines.append(line)
            m = re.match("define RELAY([0-9]+)_PIN_DEFAULT %s" % VTX_PWR_pin, line)
            if m is not None:
                # append our relay function line
                outlines.append(f"define RELAY{RELAYn}_FUNCTION 1\n")

        f = open(self.filepath, "w")
        f.write("".join(outlines))
        f.close()


if __name__ == '__main__':

    from argparse import ArgumentParser
    parser = ArgumentParser(description='Hack HWDef')
    parser.add_argument("filepath", default=None, type=str, help="hwdef file")

    args = parser.parse_args()

    hh = HackHWDef(args.filepath)
    hh.run()
