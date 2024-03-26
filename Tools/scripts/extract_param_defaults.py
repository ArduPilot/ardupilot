#!/usr/bin/python3

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
    parser.add_argument('-v', '--version', action='version', version='%(prog)s 1.1',
                        help='Display version information and exit.',
                        )
    parser.add_argument('-i', '--sysid', type=int, default=-1,
                        help='System ID for qgcs output format. Defaults to SYSID_THISMAV if defined else 1.',
                        )
    parser.add_argument('-c', '--compid', type=int, default=-1,
                        help='Component ID for qgcs output format. Defaults to 1.',
                        )
    parser.add_argument('-t', '--type',
                        choices=['defaults', 'values', 'non_default_values'],
                        default='defaults', help='Type of parameter values to extract. Defaults to %(default)s.',
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


def extract_parameter_values(logfile: str, type: str = 'defaults') -> Dict[str, float]:
    """
    Extracts the parameter values from an ArduPilot .bin log file.

    Args:
        logfile: The path to the ArduPilot .bin log file.
        type: The type of parameter values to extract. Can be 'defaults', 'values' or 'non_default_values'.

    Returns:
        A dictionary with parameter names as keys and their values as float.
    """
    try:
        mlog = mavutil.mavlink_connection(logfile)
    except Exception as e:
        raise SystemExit("Error opening the %s logfile: %s" % (logfile, str(e))) from e
    values = {}
    while True:
        m = mlog.recv_match(type=['PARM'])
        if m is None:
            if not values:
                raise SystemExit(NO_DEFAULT_VALUES_MESSAGE)
            return values
        pname = m.Name
        if len(pname) > PARAM_NAME_MAX_LEN:
            raise SystemExit("Too long parameter name: %s" % pname)
        if not re.match(PARAM_NAME_REGEX, pname):
            raise SystemExit("Invalid parameter name %s" % pname)
        # parameter names are supposed to be unique
        if pname in values:
            continue
        if type == 'defaults':
            if hasattr(m, 'Default'):
                values[pname] = m.Default
        elif type == 'values':
            if hasattr(m, 'Value'):
                values[pname] = m.Value
        elif type == 'non_default_values':
            if hasattr(m, 'Value') and hasattr(m, 'Default') and m.Value != m.Default:
                values[pname] = m.Value
        else:
            raise SystemExit("Invalid type %s" % type)


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


def sort_params(params: Dict[str, float], sort_type: str = 'none') -> Dict[str, float]:
    """
    Sorts parameter names according to sort_type

    Args:
        params: A dictionary with parameter names as keys and their values as float.
        sort_type: The type of sorting to apply. Can be 'none', 'missionplanner', 'mavproxy' or 'qgcs'.

    Returns:
        A dictionary with parameter names as keys and their values as float.
    """
    if sort_type == "missionplanner":
        params = dict(sorted(params.items(), key=lambda x: missionplanner_sort(x[0])))
    elif sort_type == "mavproxy":
        params = dict(sorted(params.items(), key=lambda x: mavproxy_sort(x[0])))
    elif sort_type == "qgcs":
        params = {k: params[k] for k in sorted(params)}
    return params


def output_params(params: Dict[str, float], format_type: str = 'missionplanner',
                  sysid: int = -1, compid: int = -1) -> None:
    """
    Outputs parameters names and their values to the console

    Args:
        params: A dictionary with parameter names as keys and their values as float.
        format_type: The output file format. Can be 'missionplanner', 'mavproxy' or 'qgcs'.

    Returns:
        None
    """
    if format_type == "qgcs":
        if sysid == -1:
            if 'SYSID_THISMAV' in params:
                sysid = params['SYSID_THISMAV']
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

    for param_name, param_value in params.items():
        if format_type == "missionplanner":
            try:
                param_value = format(param_value, '.6f').rstrip('0').rstrip('.')
            except ValueError:
                pass # preserve non-floating point strings, if present
            print(f"{param_name},{param_value}")
        elif format_type == "mavproxy":
            print("%-15s %.6f" % (param_name, param_value))
        elif format_type == "qgcs":
            MAV_PARAM_TYPE_REAL32 = 9
            print("%u %u %-15s %.6f %u" %
                  (sysid, compid, param_name, param_value, MAV_PARAM_TYPE_REAL32))


def main():
    args = parse_arguments()
    parameter_values = extract_parameter_values(args.bin_file, args.type)
    parameter_values = sort_params(parameter_values, args.sort)
    output_params(parameter_values, args.format, args.sysid, args.compid)


if __name__ == "__main__":
    main()
