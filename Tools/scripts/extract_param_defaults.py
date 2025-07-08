#!/usr/bin/env python3

'''
Extracts parameter default values from an ArduPilot .bin log file.

Supports Mission Planner, MAVProxy and QGCS file format output

Currently has 95% unit test coverage

AP_FLAKE8_CLEAN

Amilcar do Carmo Lucas, IAV GmbH
'''

import argparse
import re
from typing import Dict, Tuple
from pymavlink import mavutil

NO_DEFAULT_VALUES_MESSAGE = "The .bin file contained no parameter default values. Update to a newer ArduPilot firmware version"
PARAM_NAME_REGEX = r'^[A-Z][A-Z_0-9]*$'
PARAM_NAME_MAX_LEN = 16
MAVLINK_SYSID_MAX = 2**24
MAVLINK_COMPID_MAX = 2**8


def parse_arguments(args=None):
    """
    Parses command line arguments for the script.

    Args:
        args: List of command line arguments. Defaults to None, which means it uses sys.argv.

    Returns:
        Namespace object containing the parsed arguments.
    """
    parser = argparse.ArgumentParser(description='Extracts parameter default values from an ArduPilot .bin log file.')
    parser.add_argument('-f', '--format',
                        choices=['missionplanner', 'mavproxy', 'qgcs'],
                        default='missionplanner', help='Output file format. Defaults to %(default)s.',
                        )
    parser.add_argument('-s', '--sort',
                        choices=['none', 'missionplanner', 'mavproxy', 'qgcs'],
                        default='', help='Sort the parameters in the file. Defaults to the same as the format.',
                        )
    parser.add_argument('-v', '--version', action='version', version='%(prog)s 1.0',
                        help='Display version information and exit.',
                        )
    parser.add_argument('-i', '--sysid', type=int, default=-1,
                        help='System ID for qgcs output format. Defaults to MAV_SYSID if defined else 1.',
                        )
    parser.add_argument('-c', '--compid', type=int, default=-1,
                        help='Component ID for qgcs output format. Defaults to 1.',
                        )
    parser.add_argument('bin_file', help='The ArduPilot .bin log file to read')
    args, _ = parser.parse_known_args(args)

    if args.sort == '':
        args.sort = args.format

    if args.format != 'qgcs':
        if args.sysid != -1:
            raise SystemExit("--sysid parameter is only relevant if --format is qgcs")
        if args.compid != -1:
            raise SystemExit("--compid parameter is only relevant if --format is qgcs")

    return args


def extract_parameter_default_values(logfile: str) -> Dict[str, float]:
    """
    Extracts the parameter default values from an ArduPilot .bin log file.

    Args:
        logfile: The path to the ArduPilot .bin log file.

    Returns:
        A dictionary with parameter names as keys and their default values as float.
    """
    try:
        mlog = mavutil.mavlink_connection(logfile)
    except Exception as e:
        raise SystemExit("Error opening the %s logfile: %s" % (logfile, str(e))) from e
    defaults = {}
    while True:
        m = mlog.recv_match(type=['PARM'])
        if m is None:
            if not defaults:
                raise SystemExit(NO_DEFAULT_VALUES_MESSAGE)
            return defaults
        pname = m.Name
        if len(pname) > PARAM_NAME_MAX_LEN:
            raise SystemExit("Too long parameter name: %s" % pname)
        if not re.match(PARAM_NAME_REGEX, pname):
            raise SystemExit("Invalid parameter name %s" % pname)
        # parameter names are supposed to be unique
        if pname not in defaults and hasattr(m, 'Default'):
            defaults[pname] = m.Default  # the first time default is declared is enough


def missionplanner_sort(item: str) -> Tuple[str, ...]:
    """
    Sorts a parameter name according to the rules defined in the Mission Planner software.

    Args:
        item: The parameter name to sort.

    Returns:
        A tuple representing the sorted parameter name.
    """
    parts = item.split("_")  # Split the parameter name by underscore
    # Compare the parts separately
    return tuple(parts)


def mavproxy_sort(item: str) -> str:
    """
    Sorts a parameter name according to the rules defined in the MAVProxy software.

    Args:
        item: The parameter name to sort.

    Returns:
        The sorted parameter name.
    """
    return item


def sort_params(defaults: Dict[str, float], sort_type: str = 'none') -> Dict[str, float]:
    """
    Sorts parameter names according to sort_type

    Args:
        defaults: A dictionary with parameter names as keys and their default values as float.
        sort_type: The type of sorting to apply. Can be 'none', 'missionplanner', 'mavproxy' or 'qgcs'.

    Returns:
        A dictionary with parameter names as keys and their default values as float.
    """
    if sort_type == "missionplanner":
        defaults = dict(sorted(defaults.items(), key=lambda x: missionplanner_sort(x[0])))
    elif sort_type == "mavproxy":
        defaults = dict(sorted(defaults.items(), key=lambda x: mavproxy_sort(x[0])))
    elif sort_type == "qgcs":
        defaults = {k: defaults[k] for k in sorted(defaults)}
    return defaults


def output_params(defaults: Dict[str, float], format_type: str = 'missionplanner',
                  sysid: int = -1, compid: int = -1) -> None:
    """
    Outputs parameters names and their default values to the console

    Args:
        defaults: A dictionary with parameter names as keys and their default values as float.
        format_type: The output file format. Can be 'missionplanner', 'mavproxy' or 'qgcs'.

    Returns:
        None
    """
    if format_type == "qgcs":
        if sysid == -1:
            if 'MAV_SYSID' in defaults:
                sysid = defaults['MAV_SYSID']
            else:
                sysid = 1  # if unspecified, default to 1
        if compid == -1:
            compid = 1  # if unspecified, default to 1
        if sysid < 0:
            raise SystemExit("Invalid system ID parameter %i must not be negative" % sysid)
        if sysid > MAVLINK_SYSID_MAX-1:
            raise SystemExit("Invalid system ID parameter %i must be smaller than %i" % (sysid, MAVLINK_SYSID_MAX))
        if compid < 0:
            raise SystemExit("Invalid component ID parameter %i must not be negative" % compid)
        if compid > MAVLINK_COMPID_MAX-1:
            raise SystemExit("Invalid component ID parameter %i must be smaller than %i" % (compid, MAVLINK_COMPID_MAX))
        # see https://dev.qgroundcontrol.com/master/en/file_formats/parameters.html
        print("""
# # Vehicle-Id Component-Id Name Value Type
""")

    for param_name, default_value in defaults.items():
        if format_type == "missionplanner":
            try:
                default_value = format(default_value, '.6f').rstrip('0').rstrip('.')
            except ValueError:
                pass # preserve non-floating point strings, if present
            print(f"{param_name},{default_value}")
        elif format_type == "mavproxy":
            print("%-15s %.6f" % (param_name, default_value))
        elif format_type == "qgcs":
            MAV_PARAM_TYPE_REAL32 = 9
            print("%u %u %-15s %.6f %u" %
                  (sysid, compid, param_name, default_value, MAV_PARAM_TYPE_REAL32))


def main():
    args = parse_arguments()
    parameter_default_values = extract_parameter_default_values(args.bin_file)
    parameter_default_values = sort_params(parameter_default_values, args.sort)
    output_params(parameter_default_values, args.format, args.sysid, args.compid)


if __name__ == "__main__":
    main()
