#!/usr/bin/env python3

# flake8: noqa

'''
convert fixed wing PIDs in a parameter file from old system to ACPID system
'''

import os
import re

found_line = False
changed_param = False

def process_file(fname):
    global found_line
    global changed_param

    found_line = False
    changed_param = False

    print("Processing %s" % fname)
    lines = open(fname).readlines()
    for i in range(len(lines)):
        lines[i] = lines[i].rstrip()
    while lines[len(lines)-1].strip() == "":
        lines = lines[:-1]

    def find_line(pname):
        global found_line
        for i in range(len(lines)):
            if lines[i].startswith(pname) and not lines[i][len(pname)].isalpha():
                found_line = True
                return i
        return None

    def find_param(pname, dvalue):
        i = find_line(pname)
        if i is None:
            return dvalue
        s = lines[i].replace(","," ")
        a = s.split()
        return float(a[1])

    def set_param(pname, value, old_name):
        global changed_param
        i = find_line(pname)
        if i is None:
            i = find_line(old_name)
        if i is None:
            changed_param = True
            lines.append("%s %f" % (pname, value))
            return
        # maintain separator if possible
        sep = " "
        m = re.match("[A-Z_0-9]+([\s,]+)[0-9.-]+", lines[i])
        if m is not None:
            sep = m.group(1)
        changed_param = True
        lines[i] = "%s%s%f" % (pname, sep, value)

    print("Converting pitch")
    FF = find_param("PTCH2SRV_FF", 0)
    P = find_param("PTCH2SRV_P", 1.0)
    I = find_param("PTCH2SRV_I", 0.3)
    D = find_param("PTCH2SRV_D", 0.04)
    IMAX = find_param("PTCH2SRV_IMAX", 3000)
    TCONST = find_param("PTCH2SRV_TCONST", 0.5)

    if FF <= 0:
        I = max(I,0.3)

    kp_ff = max((P - I * TCONST) * TCONST  - D, 0)
    if found_line:
        set_param("PTCH_RATE_FF", FF + kp_ff, "PTCH2SRV_FF")
        set_param("PTCH_RATE_P", D, "PTCH2SRV_P")
        set_param("PTCH_RATE_I", I * TCONST, "PTCH2SRV_I")
        set_param("PTCH_RATE_D", 0, "PTCH2SRV_D")
        set_param("PTCH_RATE_IMAX", IMAX/4500.0, "PTCH2SRV_IMAX")

    found_line = False

    print("Converting roll")
    FF = find_param("RLL2SRV_FF", 0)
    P = find_param("RLL2SRV_P", 1.0)
    I = find_param("RLL2SRV_I", 0.3)
    D = find_param("RLL2SRV_D", 0.08)
    IMAX = find_param("RLL2SRV_IMAX", 3000)
    TCONST = find_param("RLL2SRV_TCONST", 0.5)

    kp_ff = max((P - I * TCONST) * TCONST  - D, 0)
    if found_line:
        set_param("RLL_RATE_FF", FF + kp_ff, "RLL2SRV_FF")
        set_param("RLL_RATE_P", D, "RLL2SRV_P")
        set_param("RLL_RATE_I", I * TCONST, "RLL2SRV_I")
        set_param("RLL_RATE_D", 0, "RLL2SRV_D")
        set_param("RLL_RATE_IMAX", IMAX/4500.0, "RLL2SRV_IMAX")

    if not changed_param:
        print("No fixed wing PID params")
        return

    print("Writing")
    tfile = fname + ".tmp"
    f = open(tfile,"w")
    for i in range(len(lines)):
        f.write("%s\n" % lines[i])
    f.close()
    os.rename(tfile, fname)

import argparse
parser = argparse.ArgumentParser(description='convert plane PIDs from old to new system')

parser.add_argument('param_file', nargs='+')
args = parser.parse_args()

for f in args.param_file:
    process_file(f)
