#!/usr/bin/env python3

'''
Convert ArduPilot 4.6 parameter files to the 4.7 parameter names and units.

ArduPilot 4.7 renamed many Copter and QuadPlane parameters to SI units (for
example ATC_ACCEL_R_MAX in cdeg/s/s became ATC_ACC_R_MAX in deg/s/s, WPNAV_SPEED
in cm/s became WP_SPD in m/s) and moved a few others (SRn_ stream rates to MAVn_,
SYSID_THISMAV to MAV_SYSID, ARMING_CHECK to ARMING_SKIPCHK, two SERIALn_OPTIONS
bits to MAVn_OPTIONS). The firmware converts the parameters stored on the
vehicle, but not saved parameter files. This tool rewrites such files.

Mission Planner (NAME,VALUE), MAVProxy (NAME VALUE) and QGroundControl
(SYSID COMPID NAME VALUE TYPE) files are supported. Files are converted in
place unless --output-dir or --dry-run is given. Directories are searched
recursively for *.param, *.params and *.parm files.

Examples:
  Tools/scripts/convert_params_4_7.py my_copter.param
  Tools/scripts/convert_params_4_7.py --dry-run --vehicle plane params/
  Tools/scripts/convert_params_4_7.py --patch 0 --output-dir converted/ params/

Parameters of Lua applets are not covered, e.g. ARM_C_RTL_ALT of arming-checks.lua
became ARM_C_RTL_ALT_M in metres.

AP_FLAKE8_CLEAN
'''

from __future__ import annotations

import argparse
import dataclasses
import logging
import os
import re
import shutil

from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

import convert_param_scale as cps

COPTER = 'copter'
PLANE = 'plane'
VEHICLES = (COPTER, PLANE)

LATEST_PATCH = 1

# Suffixes of the AC_PID_Basic (PSC_VELZ_) and AC_PID_2D (PSC_VELXY_) controllers.
PID_BASIC_SUFFIXES = ('P', 'I', 'IMAX', 'FLTE', 'D', 'FLTD', 'FF')
# Suffixes of the AC_PID (PSC_ACCZ_) controller.
PID_FULL_SUFFIXES = ('P', 'I', 'D', 'FF', 'IMAX', 'FLTT', 'FLTE', 'FLTD', 'SMAX', 'PDMX', 'D_FF', 'NTF', 'NEF')
# Scale factors of the PSC_ACCZ_ parameters that changed units.
ACC_SCALE = {'P': 0.1, 'I': 0.1, 'D': 0.1, 'IMAX': 0.001}

STREAM_RATE_SUFFIXES = ('RAW_SENS', 'EXT_STAT', 'RC_CHAN', 'RAW_CTRL', 'POSITION',
                        'EXTRA1', 'EXTRA2', 'EXTRA3', 'PARAMS', 'ADSB')
NUM_STREAM_RATE_GROUPS = 7

# AP_SerialManager protocol numbers that create a MAVLink channel.
MAVLINK_PROTOCOLS = (1, 2, 43)
NUM_SERIAL_PORTS = 10
# Default SERIALn_PROTOCOL values of AP_SerialManager, boards may override them.
DEFAULT_SERIAL_PROTOCOLS = {0: 2, 1: 2, 2: 2, 3: 5, 4: 5}

# SERIALn_OPTIONS bits moved to MAVn_OPTIONS in 4.7.
SERIAL_OPTION_NO_FORWARD = 1 << 10
SERIAL_OPTION_NO_STREAM_OVERRIDE = 1 << 12
MAV_OPTION_NO_FORWARD = 1 << 1
MAV_OPTION_NO_STREAM_OVERRIDE = 1 << 2

# Bits of ARMING_CHECK known when the ARMING_SKIPCHK conversion was written.
ARMING_CHECK_MASK = ((1 << 21) - 1) & ~1

# Rangefinder instance characters used in the RNGFNDx_ prefix.
RANGEFINDER_INSTANCES = '123456789A'

# Parameters removed or restructured in 4.7 that cannot be converted, on all vehicles and per vehicle.
REMOVED_PREFIXES = ('CAM_RC_', 'VIEP_')
REMOVED_NAMES = {
    COPTER: ('FLOW_HGT_OVR', 'PLND_ORIENT', 'ARSPD_OFF_PCNT', 'PSC_ACC_XY_FILT',
             'AROT_AS_ACC_MAX', 'AROT_FW_V_FF', 'AROT_FW_V_P', 'AROT_TARG_SP'),
    PLANE: ('FLOW_HGT_OVR', 'PLND_ORIENT', 'Q_P_ACC_XY_FILT', 'FS_SHORT_TIMEOUT'),
}

RE_SERIAL_PROTOCOL = re.compile(r'^SERIAL(\d)_PROTOCOL$')
RE_SERIAL_OPTIONS = re.compile(r'^SERIAL(\d)_OPTIONS$')

# Parameter names that only exist on one of Copter and Plane, used to guess the vehicle of a file.
PLANE_PREFIXES = ('Q_', 'TECS_', 'NAVL1_', 'PTCH_RATE_', 'RLL_RATE_', 'YAW_RATE_', 'KFF_', 'AIRSPEED_',
                  'STEER2SRV_', 'GLIDE_SLOPE_', 'ALT_SLOPE_', 'LAND_FLARE_', 'LAND_PF_')
PLANE_NAMES = ('STALL_PREVENTION', 'RTL_AUTOLAND', 'RTL_RADIUS', 'ACRO_ROLL_RATE', 'ACRO_PITCH_RATE', 'MIXING_GAIN')
COPTER_PREFIXES = ('WPNAV_', 'LOIT_', 'CIRCLE_', 'ATC_', 'PSC_', 'PHLD_', 'AVOID_', 'H_', 'MOT_', 'ACRO_RP_',
                   'ACRO_Y_', 'ACRO_BAL_', 'SPRAY_', 'PILOT_')
COPTER_NAMES = ('FRAME_CLASS', 'ANGLE_MAX', 'RTL_ALT_TYPE', 'RTL_CONE_SLOPE', 'RTL_LOIT_TIME', 'LAND_REPOSITION',
                'FS_THR_ENABLE', 'RTL_ALT', 'RTL_SPEED', 'RTL_ALT_FINAL', 'LAND_SPEED', 'LAND_SPEED_HIGH',
                'LAND_ALT_LOW')
# Parameter names of the vehicles this tool does not support (Sub, Rover, Blimp, Tracker).
UNSUPPORTED_PREFIXES = ('JS_', 'ATC_STR_', 'ATC_SPEED_', 'ATC_BAL_', 'SAIL_', 'POSXY_', 'POSZ_', 'POSYAW_',
                        'MAX_POS_', 'MAX_VEL_', 'PITCH2SRV_')
UNSUPPORTED_NAMES = ('SURFACE_DEPTH', 'CRUISE_SPEED', 'CRUISE_THROTTLE', 'TURN_RADIUS', 'MODE_CH', 'WP_SPEED',
                     'SERVO_PITCH_TYPE', 'STARTUP_DELAY')
# Sub joystick button parameters, BTNn_FUNCTION, as opposed to the BTN_ parameters of AP_Button.
RE_UNSUPPORTED = re.compile(r'^BTN\d+_')


class ConversionError(Exception):
    pass


@dataclasses.dataclass
class Step:
    '''One set of renames, applied in order of firmware version.'''
    name: str
    renames: Dict[str, cps.Rename]
    serial_options: bool = False


def _table(entries: List[cps.Rename]) -> Dict[str, cps.Rename]:
    '''Build a lookup table from rename entries, checking for mistakes.'''
    table = {}
    for entry in entries:
        assert entry.old not in table, f"duplicate rename of {entry.old}"
        assert len(entry.new) <= cps.AP_MAX_NAME_SIZE, f"{entry.new} is too long"
        table[entry.old] = entry
    for entry in entries:
        assert entry.new not in table, f"{entry.new} is both an old and a new name"
    return table


def atc_entries(prefix: str) -> List[cps.Rename]:
    return [
        cps.Rename(prefix + 'SLEW_YAW', prefix + 'RATE_WPY_MAX', 0.01),
        cps.Rename(prefix + 'ACCEL_Y_MAX', prefix + 'ACC_Y_MAX', 0.01),
        cps.Rename(prefix + 'ACCEL_R_MAX', prefix + 'ACC_R_MAX', 0.01),
        cps.Rename(prefix + 'ACCEL_P_MAX', prefix + 'ACC_P_MAX', 0.01),
    ]


def psc_entries(prefix: str) -> List[cps.Rename]:
    entries = [
        cps.Rename(prefix + 'POSZ_P', prefix + 'D_POS_P'),
        cps.Rename(prefix + 'POSXY_P', prefix + 'NE_POS_P'),
        cps.Rename(prefix + 'JERK_XY', prefix + 'JERK_NE'),
        cps.Rename(prefix + 'JERK_Z', prefix + 'JERK_D'),
    ]
    for suffix in PID_BASIC_SUFFIXES:
        factor = 0.01 if suffix == 'IMAX' else 1.0
        entries.append(cps.Rename(prefix + 'VELZ_' + suffix, prefix + 'D_VEL_' + suffix, factor))
        entries.append(cps.Rename(prefix + 'VELXY_' + suffix, prefix + 'NE_VEL_' + suffix, factor))
    for suffix in PID_FULL_SUFFIXES:
        entries.append(cps.Rename(prefix + 'ACCZ_' + suffix, prefix + 'D_ACC_' + suffix, ACC_SCALE.get(suffix, 1.0)))
    return entries


def psc_471_entries(prefix: str) -> List[cps.Rename]:
    return [
        cps.Rename(prefix + 'JERK_NE', prefix + 'NE_JERK'),
        cps.Rename(prefix + 'JERK_D', prefix + 'D_JERK'),
    ]


def wpnav_entries(old_prefix: str, new_prefix: str) -> List[cps.Rename]:
    entries = [
        cps.Rename(old_prefix + 'SPEED', new_prefix + 'SPD', 0.01),
        cps.Rename(old_prefix + 'RADIUS', new_prefix + 'RADIUS_M', 0.01),
        cps.Rename(old_prefix + 'SPEED_UP', new_prefix + 'SPD_UP', 0.01),
        cps.Rename(old_prefix + 'SPEED_DN', new_prefix + 'SPD_DN', 0.01),
        cps.Rename(old_prefix + 'ACCEL', new_prefix + 'ACC', 0.01),
        cps.Rename(old_prefix + 'ACCEL_Z', new_prefix + 'ACC_Z', 0.01),
        cps.Rename(old_prefix + 'ACCEL_C', new_prefix + 'ACC_CNR', 0.01),
    ]
    if old_prefix != new_prefix:
        for suffix in ('RFND_USE', 'JERK', 'TER_MARGIN'):
            entries.append(cps.Rename(old_prefix + suffix, new_prefix + suffix))
    return entries


def loiter_entries(prefix: str) -> List[cps.Rename]:
    return [
        cps.Rename(prefix + 'SPEED', prefix + 'SPEED_MS', 0.01),
        cps.Rename(prefix + 'ACC_MAX', prefix + 'ACC_MAX_M', 0.01),
        cps.Rename(prefix + 'BRK_ACCEL', prefix + 'BRK_ACC_M', 0.01),
        cps.Rename(prefix + 'BRK_JERK', prefix + 'BRK_JRK_M', 0.01),
    ]


def stream_rate_entries() -> List[cps.Rename]:
    entries = []
    for n in range(NUM_STREAM_RATE_GROUPS):
        for suffix in STREAM_RATE_SUFFIXES:
            entries.append(cps.Rename(f'SR{n}_{suffix}', f'MAV{n + 1}_{suffix}'))
    return entries


def rangefinder_entries() -> List[cps.Rename]:
    entries = []
    for instance in RANGEFINDER_INSTANCES:
        prefix = 'RNGFND' + instance + '_'
        entries.append(cps.Rename(prefix + 'MIN_CM', prefix + 'MIN', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32))
        entries.append(cps.Rename(prefix + 'MAX_CM', prefix + 'MAX', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32))
        entries.append(cps.Rename(prefix + 'GNDCLEAR', prefix + 'GNDCLR', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32))
    return entries


def arming_check_to_skipchk(checks: int) -> int:
    '''Convert an ARMING_CHECK bitmask to the equivalent ARMING_SKIPCHK bitmask.'''
    if checks == 0:
        # All checks were disabled, skip all current and future checks.
        return -1
    if not (checks & 1):
        # The ALL bit is not set, skip the checks that were not enabled.
        return (~checks) & ARMING_CHECK_MASK
    return 0


def common_entries() -> List[cps.Rename]:
    return [
        cps.Rename('SYSID_THISMAV', 'MAV_SYSID'),
        cps.Rename('SYSID_MYGCS', 'MAV_GCS_SYSID'),
        cps.Rename('TELEM_DELAY', 'MAV_TELEM_DELAY'),
        cps.Rename('SYSID_ENFORCE', 'MAV_OPTIONS', new_type=cps.MAV_PARAM_TYPE_INT16),
        cps.Rename('EK3_MAX_FLOW', 'EK3_FLOW_MAX'),
        cps.Rename('ARMING_CHECK', 'ARMING_SKIPCHK', func=arming_check_to_skipchk),
    ] + stream_rate_entries() + rangefinder_entries()


def copter_entries() -> List[cps.Rename]:
    return (
        [cps.Rename('ANGLE_MAX', 'ATC_ANGLE_MAX', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32)] +
        atc_entries('ATC_') +
        psc_entries('PSC_') +
        wpnav_entries('WPNAV_', 'WP_') +
        loiter_entries('LOIT_') +
        [
            cps.Rename('CIRCLE_RADIUS', 'CIRCLE_RADIUS_M', 0.01),
            cps.Rename('AVOID_ANGLE_MAX', 'AVOID_ANG_MAX', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('RTL_ALT', 'RTL_ALT_M', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('RTL_SPEED', 'RTL_SPEED_MS', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('RTL_ALT_FINAL', 'RTL_ALT_FINAL_M', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('RTL_CLIMB_MIN', 'RTL_CLIMB_MIN_M', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('LAND_SPEED', 'LAND_SPD_MS', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('LAND_SPEED_HIGH', 'LAND_SPD_HIGH_MS', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('LAND_ALT_LOW', 'LAND_ALT_LOW_M', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('PHLD_BRAKE_ANGLE', 'PHLD_BRK_ANGLE', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('PHLD_BRAKE_RATE', 'PHLD_BRK_RATE'),
            cps.Rename('PILOT_SPEED_UP', 'PILOT_SPD_UP', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('PILOT_SPEED_DN', 'PILOT_SPD_DN', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('PILOT_ACCEL_Z', 'PILOT_ACC_Z', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32),
            cps.Rename('PILOT_TKOFF_ALT', 'PILOT_TKO_ALT_M', 0.01),
        ]
    )


def plane_entries() -> List[cps.Rename]:
    return (
        [cps.Rename('Q_ANGLE_MAX', 'Q_A_ANGLE_MAX', 0.01, new_type=cps.MAV_PARAM_TYPE_REAL32)] +
        atc_entries('Q_A_') +
        psc_entries('Q_P_') +
        wpnav_entries('Q_WP_', 'Q_WP_') +
        loiter_entries('Q_LOIT_') +
        [
            cps.Rename('GLIDE_SLOPE_MIN', 'ALT_SLOPE_MIN'),
            cps.Rename('GLIDE_SLOPE_THR', 'ALT_SLOPE_MAXHGT'),
        ]
    )


def vehicle_entries(vehicle: Optional[str]) -> List[cps.Rename]:
    if vehicle == COPTER:
        return copter_entries()
    if vehicle == PLANE:
        return plane_entries()
    return []


def build_steps(vehicle: Optional[str], patch: int) -> List[Step]:
    '''Return the conversion steps for a vehicle (None for vehicle-independent renames only).'''
    steps = [Step('4.7.0', _table(common_entries() + vehicle_entries(vehicle)), serial_options=True)]
    if patch >= 1:
        prefix = {COPTER: 'PSC_', PLANE: 'Q_P_'}.get(vehicle)
        steps.append(Step('4.7.1', _table(psc_471_entries(prefix) if prefix else [])))
    return steps


def detect_vehicle(lines: List[cps.Line]) -> Optional[str]:
    '''Guess the vehicle a parameter file belongs to from vehicle-specific parameter names.'''
    plane = copter = 0
    for line in lines:
        name = line.name
        if name is None:
            continue
        if name.startswith(UNSUPPORTED_PREFIXES) or name in UNSUPPORTED_NAMES or RE_UNSUPPORTED.match(name):
            raise ConversionError(f"contains {name}, only Copter and Plane files are supported")
        if name.startswith(PLANE_PREFIXES) or name in PLANE_NAMES:
            plane += 1
        elif name.startswith(COPTER_PREFIXES) or name in COPTER_NAMES:
            copter += 1
    if plane and copter:
        raise ConversionError("contains both Plane and Copter parameters, use --vehicle")
    if plane:
        return PLANE
    if copter:
        return COPTER
    return None


def dominant_eol(lines: List[cps.Line]) -> str:
    crlf = sum(1 for line in lines if line.render().endswith('\r\n'))
    lf = sum(1 for line in lines if line.render().endswith('\n')) - crlf
    return '\r\n' if crlf > lf else '\n'


def mavlink_channel_of_serial(port: int, protocols: Dict[int, int]) -> Optional[int]:
    '''Return the MAVLink channel of a serial port, or None if it does not run MAVLink.'''
    channel = 0
    for n in range(NUM_SERIAL_PORTS):
        protocol = protocols.get(n, DEFAULT_SERIAL_PROTOCOLS.get(n, -1))
        if protocol not in MAVLINK_PROTOCOLS:
            continue
        if n == port:
            return channel
        channel += 1
    return None


def convert_serial_options(lines: List[cps.Line], path: str = '') -> List[cps.Change]:
    '''Move the MAVLink bits of SERIALn_OPTIONS into MAVm_OPTIONS.'''
    protocols = {}
    for line in lines:
        if line.name is None:
            continue
        m = RE_SERIAL_PROTOCOL.match(line.name)
        if m:
            protocols[int(m.group(1))] = int(cps.parse_value(line.value))

    changes = []
    i = 0
    while i < len(lines):
        line = lines[i]
        i += 1
        if not isinstance(line, cps.ParamLine):
            continue
        m = RE_SERIAL_OPTIONS.match(line.name)
        if not m:
            continue
        options = int(cps.parse_value(line.value))
        moved = options & (SERIAL_OPTION_NO_FORWARD | SERIAL_OPTION_NO_STREAM_OVERRIDE)
        if not moved:
            continue
        port = int(m.group(1))
        channel = mavlink_channel_of_serial(port, protocols)
        if channel is None:
            logging.warning("%s: %s has MAVLink option bits set but SERIAL%u_PROTOCOL is not MAVLink, not converted",
                            path, line.name, port)
            continue
        mav_name = f'MAV{channel + 1}_OPTIONS'
        mav_bits = 0
        if moved & SERIAL_OPTION_NO_FORWARD:
            mav_bits |= MAV_OPTION_NO_FORWARD
        if moved & SERIAL_OPTION_NO_STREAM_OVERRIDE:
            mav_bits |= MAV_OPTION_NO_STREAM_OVERRIDE
        logging.warning("%s: moved bits of %s into %s; the MAVLink channel index is derived from the SERIALn_PROTOCOL "
                        "values in the file and the board defaults, check that it matches the target board",
                        path, line.name, mav_name)

        new_line = line.replace(value=cps.format_int(options & ~moved, line.value), ptype=cps.MAV_PARAM_TYPE_INT32)
        changes.append(cps.Change(i, line.render(), new_line.render()))
        lines[i - 1] = new_line

        existing = next((j for j, other in enumerate(lines) if other.name == mav_name), None)
        if existing is not None:
            other = lines[existing]
            if isinstance(other, cps.ParamLine):
                new_other = other.replace(value=cps.format_int(int(cps.parse_value(other.value)) | mav_bits, other.value))
                changes.append(cps.Change(existing + 1, other.render(), new_other.render()))
                lines[existing] = new_other
                continue
        inserted = new_line.clone_for(mav_name, str(mav_bits), cps.MAV_PARAM_TYPE_INT16)
        if not new_line.eol:
            # Keep the file without a trailing newline.
            lines[i - 1] = new_line.replace(eol=dominant_eol(lines))
            inserted = inserted.replace(eol='')
        lines.insert(i, inserted)
        changes.append(cps.Change(i + 1, '', inserted.render()))
        i += 1
    return changes


def convert_lines(lines: List[cps.Line], vehicle: Optional[str], patch: int, sig_digits: int,
                  path: str = '') -> List[cps.Change]:
    '''Convert parsed lines in place and return the list of changes.'''
    steps = build_steps(vehicle, patch)
    changes = []
    for step in steps:
        changes += cps.apply_renames(lines, step.renames, sig_digits, path)
        if step.serial_options:
            changes += convert_serial_options(lines, path)
    old_names = set()
    for step in steps:
        old_names.update(step.renames)
    removed_names = REMOVED_NAMES.get(vehicle, ())
    for i, line in enumerate(lines):
        if isinstance(line, cps.RawLine) and line.first_token() in old_names:
            logging.warning("%s: line %u: could not parse line with %s, left untouched", path, i + 1, line.first_token())
        elif line.name is not None and (line.name.startswith(REMOVED_PREFIXES) or line.name in removed_names):
            logging.warning("%s: line %u: %s was removed or restructured in 4.7, left untouched", path, i + 1, line.name)
    return changes


def vehicle_specific_names_present(lines: List[cps.Line], patch: int) -> List[str]:
    '''Return the old parameter names in lines that only a Copter or a Plane table converts.

    Used when the vehicle cannot be detected: an empty result means the vehicle-independent
    conversions are enough, otherwise the caller must ask for --vehicle.
    '''
    names = set()
    for vehicle in VEHICLES:
        for step in build_steps(vehicle, patch):
            names.update(step.renames)
    for step in build_steps(None, patch):
        names.difference_update(step.renames)
    return sorted(name for name in {line.name for line in lines} if name in names)


def convert_file(path: str, vehicle: str, patch: int, sig_digits: int, dry_run: bool,
                 out_path: Optional[str]) -> None:
    '''Convert one parameter file, raising ConversionError on failure.'''
    lines = cps.read_param_file(path)
    if vehicle == 'auto':
        detected = detect_vehicle(lines)
        if detected is None:
            present = vehicle_specific_names_present(lines, patch)
            if present:
                raise ConversionError(f"cannot determine vehicle (found {', '.join(present)}), use --vehicle")
            logging.info("%s: vehicle not detected, applying only vehicle-independent conversions", path)
    else:
        detected = vehicle
    changes = convert_lines(lines, detected, patch, sig_digits, path)
    if dry_run:
        for change in changes:
            print(f"{path}: {change}")
    elif out_path is not None:
        if changes:
            cps.write_param_file(out_path, lines, mode_from=path)
        else:
            os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
            shutil.copy2(path, out_path)
    elif changes:
        cps.write_param_file(path, lines)
    logging.info("%s: %u parameters converted", path, len(changes))


def collect_files(paths: List[str]) -> List[Tuple[str, str]]:
    '''Return (file, root) pairs for all parameter files under the given paths.'''
    files = []
    for path in paths:
        if os.path.isdir(path):
            found = list(cps.iter_param_files(path))
            if not found:
                logging.warning("%s: no parameter files found", path)
            files += [(f, path) for f in found]
        elif os.path.isfile(path):
            files.append((path, os.path.dirname(path)))
        else:
            raise ConversionError(f"{path}: no such file or directory")
    return files


def parse_arguments(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="convert 4.6 parameter files to 4.7 names and units")
    parser.add_argument('--patch', type=int, default=LATEST_PATCH, choices=range(LATEST_PATCH + 1),
                        help="4.7 patch release to convert to (default %(default)s)")
    parser.add_argument('--vehicle', default='auto', choices=('auto',) + VEHICLES,
                        help="vehicle the files belong to (default: detect from content)")
    parser.add_argument('--sig-digits', type=int, default=cps.DEFAULT_SIG_DIGITS,
                        help="significant digits kept in rescaled values (default %(default)s)")
    parser.add_argument('--dry-run', action='store_true', help="print the changes without writing any file")
    parser.add_argument('--output-dir', help="write converted files below this directory instead of in place")
    parser.add_argument('-v', '--verbose', action='store_true', help="print a summary for every file")
    parser.add_argument('paths', nargs='+', metavar='PATH', help="parameter file or directory to convert")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_arguments(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING, format='%(levelname)s: %(message)s')
    try:
        files = collect_files(args.paths)
    except ConversionError as e:
        logging.error("%s", e)
        return 1
    failed = 0
    for path, root in files:
        out_path = None
        if args.output_dir is not None:
            out_path = os.path.join(args.output_dir, os.path.relpath(path, root))
        try:
            convert_file(path, args.vehicle, args.patch, args.sig_digits, args.dry_run, out_path)
        except (ConversionError, OSError, ValueError, OverflowError) as e:
            logging.error("%s: %s", path, e)
            failed += 1
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
