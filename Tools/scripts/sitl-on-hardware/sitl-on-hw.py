#!/usr/bin/env python3
'''
script to build a firmware for SITL-on-hardware
see https://ardupilot.org/dev/docs/sim-on-hardware.html
'''

import subprocess
import sys
import os
import tempfile

from argparse import ArgumentParser
parser = ArgumentParser("SITL on hardware builder")
parser.add_argument("--board", default=None, help="board type")
parser.add_argument("--vehicle", default=None, help="vehicle type")
parser.add_argument("--frame", default=None, help="frame type")
parser.add_argument("--simclass", default=None, help="simulation class")
parser.add_argument("--defaults", default=None, help="extra defaults file")
parser.add_argument("--upload", action='store_true', default=False, help="upload firmware")

args, unknown_args = parser.parse_known_args()

extra_hwdef = None

def run_program(cmd_list):
    '''run a program from a command list'''
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
       print("FAILED: %s" % (' '.join(cmd_list)))
       global extra_hwdef
       if extra_hwdef is not None:
           extra_hwdef.close()
           os.unlink(extra_hwdef.name)
       sys.exit(1)

extra_hwdef = tempfile.NamedTemporaryFile(mode='w')
extra_defaults = tempfile.NamedTemporaryFile(mode='w')

def hwdef_write(s):
    '''write to the hwdef temp file'''
    extra_hwdef.write(s)

def defaults_write(s):
    '''write to the hwdef temp file'''
    extra_defaults.write(s)

def sohw_path(fname):
    '''get path to a file in on-hardware directory'''
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), fname)

if args.vehicle == "plane":
    extra_hwdef_base = "plane-extra-hwdef-sitl-on-hw.dat"
    defaults_base = "plane-default.param"
else:
    extra_hwdef_base = "extra-hwdef-sitl-on-hw.dat"
    defaults_base = "default.param"

# add base hwdef to extra_hwdef
hwdef_write(open(sohw_path(extra_hwdef_base), "r").read() + "\n")

# add base defaults to extra_defaults
defaults_write(open(sohw_path(defaults_base), "r").read() + "\n")

if args.defaults:
    defaults_write(open(args.defaults,"r").read() + "\n")

if args.simclass:
    hwdef_write("define AP_SIM_FRAME_CLASS %s\n" % args.simclass)
if args.frame:
    hwdef_write('define AP_SIM_FRAME_STRING "%s"\n' % args.frame)

extra_hwdef.flush()
extra_defaults.flush()

configure_args = ["./waf", "configure",
                  "--board=%s" % args.board,
                  "--extra-hwdef=%s" % extra_hwdef.name,
                  "--default-param=%s" % extra_defaults.name]
configure_args.extend(unknown_args)
run_program(configure_args)

build_cmd = ["./waf", args.vehicle]
if args.upload:
   build_cmd.append("--upload")

run_program(build_cmd)

# cleanup
extra_hwdef.close()

