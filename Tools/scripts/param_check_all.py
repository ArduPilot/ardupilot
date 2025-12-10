#!/usr/bin/env python3

'''
Check all default parameter files for all hwdef boards and sitl targets.

AP_FLAKE8_CLEAN
'''

from __future__ import annotations
import os
import sys
import glob
from board_list import BoardList
from param_check import get_metadata, check_file, SkippedChecks

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../Tools', 'autotest'))
from pysim.vehicleinfo import VehicleInfo  # noqa: E402


periph_hwdefs_to_skip = set([  # Most of these fail due to NET_ params
    'BotBloxDroneNet',
    'CubeNode-ETH',
    'CubeRedPrimary-PPPGW',
    'MatekG474-Periph',  # Fails for RC_PORT
    'Pixhawk6X-PPPGW',
    'kha_eth',
    'sw-boom-f407',  # Fails for ESC_NUM_POLES*
])

sitl_params_to_skip = set([
    'default_params/blimp.parm',
    'default_params/stratoblimp.parm',
    'default_params/motorboat.parm',
    'default_params/sub.parm',
    'default_params/sub-6dof.parm',
])

frame_params_to_skip = set([
    'EFlight_Convergence.param',
    'WLToys_V383_HeliQuad.param',
])


VEHICLE_BOARDS = BoardList().find_autobuild_boards(skip=set())
PERIPH_BOARDS = BoardList().find_ap_periph_boards(skip=periph_hwdefs_to_skip)

# List of all firmware that isn't AP_Periph
ALL_VEHICLE_FIRMWARES = ['Sub', 'Plane', 'Blimp', 'Copter', 'Tracker', 'Rover']


def check_boards(boards, firmwares, skip: SkippedChecks | None = None):
    '''Check parameter files for ChibiOS hwdef boards.'''
    if skip is None:
        skip = SkippedChecks()

    hwdef_dir = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', '..', 'libraries', 'AP_HAL_ChibiOS', 'hwdef'
    )
    hwdef_dir = os.path.abspath(hwdef_dir)
    param_files = [
        os.path.join(hwdef_dir, board, '**', f'*.{ext}')
        for board in boards
        for ext in ['parm', 'param']
    ]

    # Find all parameter files
    param_files = [f for pattern in param_files for f in glob.glob(pattern, recursive=True)]

    return check_files(param_files, firmwares, skip=skip)


def check_sitl(skip: SkippedChecks | None = None):
    '''Check every parameter file that shows up in vehicleinfo.py

    Because this also provides us information about the intended firmware, we
    can check it strictly against the parameter metadata for that specific
    firmware, unlike the non-periph hwdef boards.
    '''
    if skip is None:
        skip = SkippedChecks()
    vinfo = VehicleInfo()
    vehicle_name = {
        'ArduCopter': 'Copter',
        'Helicopter': 'Copter',
        'Blimp': 'Blimp',
        'ArduPlane': 'Plane',
        'Rover': 'Rover',
        'ArduSub': 'Sub',
        'AntennaTracker': 'Tracker',
        'sitl_periph_universal': 'AP_Periph'
    }
    directory = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', '..', 'Tools', 'autotest'
    )
    success = True
    for key, opts in vinfo.options.items():
        print(f"Checking {key} SITL boards...")
        vehicle = vehicle_name[key]
        files = []
        for frame_opts in opts['frames'].values():
            if not frame_opts.get('default_params_filename'):
                continue
            defaults = frame_opts['default_params_filename']
            # defaults is a string or a list of strings
            if isinstance(defaults, str):
                defaults = [defaults]
            for default in defaults:
                if default in files:
                    continue
                if default in sitl_params_to_skip:
                    continue
                default = os.path.relpath(os.path.join(directory, default))
                if not os.path.exists(default):
                    raise FileNotFoundError(f"Default parameter file {default} does not exist")
                files.append(default)
        if not files:
            print(f"No SITL default files for {vehicle}")
            continue
        success &= check_files(files, [vehicle], skip=skip)

    return success


def check_frame_params(skip: SkippedChecks | None = None):
    '''Check all files within Tools/Frame_params

    These don't contain any information about firmware (e.g. Copter, Plane,
    etc.), so we check them against all vehicles, but it would be a good idea
    to strictly check them against the intended firmware in the future.
    '''
    if skip is None:
        skip = SkippedChecks()
    frame_params_dir = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', '..', 'Tools', 'Frame_params'
    )
    frame_params_dir = os.path.abspath(frame_params_dir)
    param_files_raw = glob.glob(os.path.join(frame_params_dir, '**', '*.parm'), recursive=True)
    param_files_raw += glob.glob(os.path.join(frame_params_dir, '**', '*.param'), recursive=True)

    # Filter out files that are in the skiplist
    param_files = []
    for file in param_files_raw:
        if os.path.relpath(file, frame_params_dir) in frame_params_to_skip:
            continue
        param_files.append(file)

    return check_files(param_files, ALL_VEHICLE_FIRMWARES, skip=skip)


def check_files(files, firmwares, skip: SkippedChecks | None = None):
    '''Check a list of parameter files against the metadata'''
    if skip is None:
        skip = SkippedChecks()

    metadata = get_metadata(firmwares)
    success = True
    for file in files:
        msgs = check_file(file, metadata, skip=skip)
        if msgs:
            success = False
            print(f"Errors in {file}:")
            for msg in msgs:
                print(f"  {msg}")
    return success


def main():
    skip = SkippedChecks(
        no_range=True,
        no_values=True,
        no_bitmask=True,
    )
    success = True
    print("Checking peripheral boards...")
    success &= check_boards(PERIPH_BOARDS, ['AP_Periph'], skip=skip)
    print("Checking vehicle boards...")
    success &= check_boards(VEHICLE_BOARDS, ALL_VEHICLE_FIRMWARES, skip=skip)
    print("Checking SITL...")
    success &= check_sitl(skip=skip)
    print("Checking frame parameters...")
    success &= check_frame_params(skip=skip)
    if not success:
        print("Parameter checks failed!")
        sys.exit(1)
    print("All parameter checks passed!")


if __name__ == '__main__':
    main()
