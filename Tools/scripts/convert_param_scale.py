#!/usr/bin/env python3

# flake8: noqa

'''
tool to convert parameter names and scales, useful for conversion for cm -> m and cdeg -> deg

this looks for files called *.param or *.parm

example:
  Tools/scripts/convert_param_scale.py --scale 0.1 TRIM_ARSPD_CM AIRSPEED_CRUISE
'''

import os
import sys

from argparse import ArgumentParser

parser = ArgumentParser(description="parameter conversion tool")

parser.add_argument("--scale", default=1.0, type=float, help="scale factor")
parser.add_argument("--directory", default=".", help="directory to search")
parser.add_argument("old_name", default=None, help="old parameter name")
parser.add_argument("new_name", default=None, help="new parameter name")

args = parser.parse_args()

def process_file(fname):
    needs_write = False
    with open(fname, "r") as f:
        lines = f.readlines()
    for i in range(len(lines)):
        line = lines[i]
        if line.startswith(args.old_name):
            a = line.split()
            if len(a) == 2:
                sep = ' '
            else:
                a = line.split(',')
                if len(a) == 2:
                    sep = ','
                else:
                    continue
            v = float(a[1])
            v *= args.scale
            lines[i] = "%s%s%.2f\n" % (args.new_name, sep, v)
            needs_write = True
    if not needs_write:
        return
    print("Updating %s" % fname)
    with open(fname, "w") as f:
        for line in lines:
            f.write(line)

for root, dirs, files in os.walk(args.directory):
    for file in files:
        if file.endswith(".parm") or file.endswith(".param"):
            process_file(os.path.join(root, file))
