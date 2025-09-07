#!/usr/bin/env python3
'''
script to build a firmware for SITL-on-hardware
see https://ardupilot.org/dev/docs/sim-on-hardware.html

AP_FLAKE8_CLEAN
'''

import subprocess
import sys
import os
import tempfile
from argparse import ArgumentParser

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../Tools', 'autotest'))
from pysim import vehicleinfo  # noqa: E402

vinfo = vehicleinfo.VehicleInfo()

vehicle_map = {
    "APMrover2": "Rover",
    "Copter": "ArduCopter",
    "Heli": "Helicopter",
    "Plane": "ArduPlane",
    "Sub": "ArduSub",
    "Blimp": "Blimp",
    "Rover": "Rover",
    "AntennaTracker": "AntennaTracker",
}
# add lower-case equivalents too
for k in list(vehicle_map.keys()):
    vehicle_map[k.lower()] = vehicle_map[k]

vehicle_choices = list(vinfo.options.keys())
# add vehicle aliases to argument parser options:
for c in vehicle_map.keys():
    vehicle_choices.append(c)

parser = ArgumentParser("SITL on hardware builder")
parser.add_argument("--board", default=None, help="board type")
parser.add_argument("-v", "--vehicle", choices=vehicle_choices, required=True, help="vehicle type")
parser.add_argument("-f", "--frame", default=None, help="frame type")
parser.add_argument("--simclass", default=None, help="simulation class")
parser.add_argument("--defaults", default=None, help="extra defaults file")
parser.add_argument("--upload", action='store_true', default=False, help="upload firmware")
parser.add_argument("--debug", action='store_true', default=False, help="create debug build")
parser.add_argument("--extra-hwdef", action='append', default=[], help="extra hwdef files")

args, unknown_args = parser.parse_known_args()

extra_hwdef = None


def run_program(cmd_list):
    '''run a program from a command list'''
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("FAILED: %s" % (' '.join(cmd_list)))
        sys.exit(1)


frame_options = sorted(vinfo.options[vehicle_map[args.vehicle]]["frames"].keys())
frame_options_string = ' '.join(frame_options)
if args.frame and args.frame not in frame_options and not not args.simclass.startswith('json:'):
    print(f"ERROR: invalid frame {args.frame}; must be one of {frame_options_string}")
    sys.exit(1)


extra_hwdef = tempfile.NamedTemporaryFile(mode='w', delete=False)
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

for f in args.extra_hwdef:
    hwdef_write(open(f, "r").read() + "\n")

# add base defaults to extra_defaults
defaults_write(open(sohw_path(defaults_base), "r").read() + "\n")

if args.defaults:
    for d in args.defaults.split(","):
        defaults_write(open(d, "r").read() + "\n")

if args.simclass:
    if args.simclass == 'Glider':
        hwdef_write("define AP_SIM_GLIDER_ENABLED 1\n")
    elif args.simclass == 'JSON':
        hwdef_write("define AP_SIM_JSON_ENABLED 1\n")
    hwdef_write("define AP_SIM_FRAME_CLASS %s\n" % args.simclass)
if args.frame:
    hwdef_write('define AP_SIM_FRAME_STRING "%s"\n' % args.frame)
    if vehicle_map[args.vehicle] == "ArduCopter" or args.simclass == "MultiCopter":
        frame_found = False
        frame_defines = {
            "quad": "AP_MOTORS_FRAME_QUAD_ENABLED",
            "+": "AP_MOTORS_FRAME_QUAD_ENABLED",
            "X": "AP_MOTORS_FRAME_QUAD_ENABLED",
            "cwx": "AP_MOTORS_FRAME_QUAD_ENABLED",
            "djix": "AP_MOTORS_FRAME_QUAD_ENABLED",
            "quad-can": "AP_MOTORS_FRAME_QUAD_ENABLED",
            "hexa": "AP_MOTORS_FRAME_HEXA_ENABLED",
            "hexa-cwx": "AP_MOTORS_FRAME_HEXA_ENABLED",
            "hexa-dji": "AP_MOTORS_FRAME_HEXA_ENABLED",
            "hexax": "AP_MOTORS_FRAME_HEXA_ENABLED",
            "deca": "AP_MOTORS_FRAME_DECA_ENABLED",
            "deca-cwx": "AP_MOTORS_FRAME_DECA_ENABLED",
            "dodeca-hexa": "AP_MOTORS_FRAME_DODECAHEXA_ENABLED",
            "octa": "AP_MOTORS_FRAME_OCTA_ENABLED",
            "octa-dji": "AP_MOTORS_FRAME_OCTA_ENABLED",
            "octa-cwx": "AP_MOTORS_FRAME_OCTA_ENABLED",
            "octa-quad": "AP_MOTORS_FRAME_OCTAQUAD_ENABLED",
            "octa-quad-cwx": "AP_MOTORS_FRAME_OCTAQUAD_ENABLED",
            "y6": "AP_MOTORS_FRAME_Y6_ENABLED"
        }
        for frame, define in frame_defines.items():
            if args.frame == frame:
                print(f"Auto enabling {define} for frame {args.frame}")
                hwdef_write(f'define {define} 1')
                frame_found = True
                break
        if not frame_found:
            print(f"Error: frame {args.frame} not found in frame_defines")
            sys.exit(1)


extra_hwdef.flush()
extra_defaults.flush()

configure_args = ["./waf", "configure",
                  "--board=%s" % args.board,
                  "--extra-hwdef=%s" % extra_hwdef.name,
                  "--default-param=%s" % extra_defaults.name]
if args.debug:
    configure_args.append("--debug")

configure_args.extend(unknown_args)
run_program(configure_args)


def get_key_from_value(d, target_value):
    for key, value in d.items():
        if value == target_value:
            return key
    return None


if args.vehicle in ["APMrover2", "apmrover2"]:  # Double map, but waf only accepts rover.
    args.vehicle = "Rover"
waf_vehicle = args.vehicle if args.vehicle in vehicle_map.keys() else get_key_from_value(vehicle_map, args.vehicle)
build_cmd = ["./waf", waf_vehicle.lower()]
if args.upload:
    build_cmd.append("--upload")

run_program(build_cmd)

# cleanup
extra_hwdef.close()
