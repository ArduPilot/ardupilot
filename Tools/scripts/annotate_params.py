#!/usr/bin/env python3

"""
This script fetches online ArduPilot parameter documentation (if not cached) and adds it to the specified file
 or to all *.param and *.parm files in the specified directory.

1. Checks if a local cache of the XML file exists in the target directory or on the directory of the target file:
 - If it does, the script loads the file content.
 - If it doesn't, the script sends a GET request to the URL to fetch the XML data for the requested vehicle type.
2. Parses the XML data and creates a dictionary of parameter documentation.
3. DELETES all comments that start at the beginning of a line
4. Adds the parameter documentation to the target file or to all *.param,*.parm files in the target directory.

Supports AP_Periph, AntennaTracker, ArduCopter, ArduPlane, ArduSub, Blimp, Heli, Rover and SITL vehicle types
Supports both Mission Planner and MAVProxy file formats
Supports sorting the parameters
Has unit tests with 88% coverage

AP_FLAKE8_CLEAN

Author: Amilcar do Carmo Lucas, IAV GmbH
"""

import os
import glob
import re
from typing import Any, Dict, List, Tuple
import xml.etree.ElementTree as ET
import argparse
import logging

# URL of the XML file
BASE_URL = "https://autotest.ardupilot.org/Parameters/"

PARAM_DEFINITION_XML_FILE = "apm.pdef.xml"

# ArduPilot parameter names start with a capital letter and can have capital letters, numbers and _
PARAM_NAME_REGEX = r'^[A-Z][A-Z_0-9]*'
PARAM_NAME_MAX_LEN = 16
VERSION = '1.0'


def arg_parser():
    parser = argparse.ArgumentParser(description='Fetches on-line ArduPilot parameter documentation and adds it to the '
                                     'specified file or to all *.param and *.parm files in the specified directory.')
    parser.add_argument('target',
                        help='The target file or directory.',
                        )
    parser.add_argument('-s', '--sort',
                        choices=['none', 'missionplanner', 'mavproxy'],
                        default='none',
                        help='Sort the parameters in the file. Defaults to not sorting.',
                        )
    parser.add_argument('-t', '--vehicle-type',
                        choices=['AP_Periph', 'AntennaTracker', 'ArduCopter', 'ArduPlane',
                                 'ArduSub', 'Blimp', 'Heli', 'Rover', 'SITL'],
                        default='ArduCopter',
                        help='The type of the vehicle. Defaults to ArduCopter',
                        )
    parser.add_argument('--verbose', action='store_true',
                        help='Increase output verbosity, print ReadOnly parameter list. Defaults to false',
                        )
    parser.add_argument('-v', '--version', action='version', version=f'%(prog)s {VERSION}',
                        help='Display version information and exit.',
                        )
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    else:
        logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

    return args


def get_xml_data(base_url: str, directory: str, filename: str) -> ET.Element:
    """
    Fetches XML data from a local file or a URL.

    Args:
        base_url (str): The base URL for fetching the XML file.
        directory (str): The directory where the XML file is expected.
        filename (str): The name of the XML file.

    Returns:
        ET.Element: The root element of the parsed XML data.
    """
    file_path = os.path.join(directory, filename)
    # Check if the locally cached file exists
    if os.path.isfile(file_path):
        # Load the file content relative to the script location
        with open(file_path, "r", encoding="utf-8") as file:
            xml_data = file.read()
    else:
        # No locally cached file exists, get it from the internet
        try:
            import requests  # pylint: disable=C0415
        except ImportError:
            logging.error("The requests package was not found")
            logging.error("Please install it by running 'pip install requests' in your terminal.")
            raise SystemExit("requests package is not installed")  # pylint: disable=W0707
        try:
            # Send a GET request to the URL
            response = requests.get(base_url + filename, timeout=5)
        except requests.exceptions.RequestException as e:
            logging.error("Unable to fetch XML data: %s", e)
            raise SystemExit("unable to fetch online XML documentation")  # pylint: disable=W0707
        # Get the text content of the response
        xml_data = response.text
        # Write the content to a file
        with open(os.path.join(directory, filename), "w", encoding="utf-8") as file:
            file.write(xml_data)

    # Parse the XML data
    root = ET.fromstring(xml_data)
    return root


def remove_prefix(text: str, prefix: str) -> str:
    """
    Removes a prefix from a string.

    Args:
        text (str): The original string.
        prefix (str): The prefix to remove.

    Returns:
        str: The string without the prefix.
    """
    if text.startswith(prefix):
        return text[len(prefix):]
    return text


def split_into_lines(string_to_split: str, maximum_line_length: int) -> List[str]:
    """
    Splits a string into lines of a maximum length.

    Args:
        string_to_split (str): The string to split.
        maximum_line_length (int): The maximum length of a line.

    Returns:
        List[str]: The list of lines.
    """
    doc_lines = re.findall(
        r".{1," + str(maximum_line_length) + r"}(?:\s|$)", string_to_split
    )
    # Remove trailing whitespace from each line
    return [line.rstrip() for line in doc_lines]


def create_doc_dict(root: ET.Element, vehicle_type: str) -> Dict[str, Any]:
    """
    Create a dictionary of parameter documentation from the root element of the parsed XML data.

    Args:
        root (ET.Element): The root element of the parsed XML data.

    Returns:
        Dict[str, Any]: A dictionary of parameter documentation.
    """
    # Dictionary to store the parameter documentation
    doc = {}

    # Use the findall method with an XPath expression to find all "param" elements
    for param in root.findall(".//param"):
        name = param.get("name")
        # Remove the <vehicle_type>: prefix from the name if it exists
        name = remove_prefix(name, vehicle_type + ":")

        human_name = param.get("humanName")
        documentation = split_into_lines(param.get("documentation"), 100)
        # the keys are the "name" attribute of the "field" sub-elements
        # the values are the text content of the "field" sub-elements
        fields = {field.get("name"): field.text for field in param.findall("field")}
        # if Units and UnitText exist, combine them into a single element
        delete_unit_text = False
        for key, value in fields.items():
            if key == "Units" and "UnitText" in fields:
                fields[key] = f"{value} ({fields['UnitText']})"
                delete_unit_text = True
        if delete_unit_text:
            del fields['UnitText']
        # the keys are the "code" attribute of the "values/value" sub-elements
        # the values are the text content of the "values/value" sub-elements
        values = {value.get("code"): value.text for value in param.findall("values/value")}

        # Dictionary with "Parameter names" as keys and the values is a
        # dictionary with "humanName", "documentation" attributes and
        # "fields", "values" sub-elements.
        doc[name] = {
            "humanName": human_name,
            "documentation": documentation,
            "fields": fields,
            "values": values,
        }

    return doc


def format_columns(values: Dict[str, Any], max_width: int = 105, max_columns: int = 4) -> List[str]:
    """
    Formats a dictionary of values into column-major horizontally aligned columns.
    It uses at most max_columns columns

    Args:
        values (Dict[str, Any]): The dictionary of values to format.
        max_width (int, optional): The maximum number of characters on all columns. Defaults to 105.

    Returns:
        List[str]: The list of formatted strings.
    """
    # Convert the dictionary into a list of strings
    strings = [f"{k}: {v}" for k, v in values.items()]

    if (not strings) or (len(strings) == 0):
        return []

    # Calculate the maximum length of the strings
    max_len = max(len(s) for s in strings)

    # Determine the number of columns
    # Column distribution will only happen if it results in more than 5 rows
    # The strings will be distributed evenly across up-to max_columns columns.
    for num_cols in range(max_columns, 0, -1):
        if len(strings) // num_cols > 5 and (max_len + 2) * num_cols < max_width:
            break

    # Calculate the column width
    col_width = max_width // num_cols

    num_rows = (len(strings) + num_cols - 1) // num_cols

    formatted_strings = []
    for j in range(num_rows):
        row = []
        for i in range(num_cols):
            if i*num_rows + j < len(strings):
                if i < num_cols - 1 and ((i+1)*num_rows + j < len(strings)):
                    row.append(strings[i*num_rows + j].ljust(col_width))
                else:
                    row.append(strings[i*num_rows + j])
        formatted_strings.append(" ".join(row))

    return formatted_strings


def extract_parameter_name(item: str) -> str:
    """
    Extract the parameter name from a line. Very simple to be used in sorting
    """
    item = item.strip()
    match = re.match(PARAM_NAME_REGEX, item)
    return match.group(0) if match else item


def missionplanner_sort(item: str) -> Tuple[str, ...]:
    """
    MissionPlanner parameter sorting function
    """
    # Split the parameter name by underscore
    parts = extract_parameter_name(item).split("_")
    # Compare the parts separately
    return tuple(parts)


def extract_parameter_name_and_validate(line: str, filename: str, line_nr: int) -> str:
    """
    Extracts the parameter name from a line and validates it.
    Args:
        line (str): The line to extract the parameter name from.
    Returns:
        str: The extracted parameter name.
    Raises:
        SystemExit: If the line is invalid or the parameter name is too long or invalid.
    """
    # Extract the parameter name
    match = re.match(PARAM_NAME_REGEX, line)
    if match:
        param_name = match.group(0)
    else:
        logging.error("Invalid line %d in file %s: %s", line_nr, filename, line)
        raise SystemExit("Invalid line in input file")
    param_len = len(param_name)
    param_sep = line[param_len] # the character following the parameter name must be a separator
    if param_sep not in {',', ' ', '\t'}:
        logging.error("Invalid parameter name %s on line %d in file %s", param_name, line_nr,
                      filename)
        raise SystemExit("Invalid parameter name")
    if param_len > PARAM_NAME_MAX_LEN:
        logging.error("Too long parameter name on line %d in file %s", line_nr, filename)
        raise SystemExit("Too long parameter name")
    return param_name


def update_parameter_documentation(doc: Dict[str, Any], target: str = '.', sort_type: str = 'none') -> None:
    """
    Updates the parameter documentation in the target file or in all *.param,*.parm files of the target directory.

    This function iterates over all the ArduPilot parameter files in the target directory or file.
    For each file, it DELETES all comments that start at the beginning of a line, optionally sorts the
    parameter names and checks if the parameter name is in the dictionary of parameter documentation.
    If it is, it prefixes the line with comment derived from the dictionary element.
    If it's not, it copies the parameter line 1-to-1.
    After processing all the parameters in a file, it writes the new lines back to the file.

    Args:
        doc (Dict[str, Any]): A dictionary of parameter documentation.
        target (str, optional): The target directory or file. Defaults to '.'.
        sort_type (str, optional): The type of sorting to apply to the parameters.
                                   Can be 'none', 'missionplanner', or 'mavproxy'. Defaults to 'none'.
    """
    # Check if the target is a file or a directory
    if os.path.isfile(target):
        # If it's a file, process only that file
        param_files = [target]
    elif os.path.isdir(target):
        # If it's a directory, process all .param and .parm files in that directory
        param_files = glob.glob(os.path.join(target, "*.param")) \
                    + glob.glob(os.path.join(target, "*.parm"))
    else:
        raise ValueError(f"Target '{target}' is neither a file nor a directory.")

    # Iterate over all the target ArduPilot parameter files
    for param_file in param_files:

        # Read the entire file contents
        with open(param_file, "r", encoding="utf-8") as file:
            lines = file.readlines()

        new_lines = []
        total_params = 0
        documented_params = 0
        undocumented_params = []
        is_first_param_in_file = True  # pylint: disable=C0103
        if sort_type == "missionplanner":
            lines.sort(key=missionplanner_sort)
        if sort_type == "mavproxy":
            lines.sort(key=extract_parameter_name)
        for n, line in enumerate(lines, start=1):
            line = line.strip()
            if not line.startswith("#") and line:
                param_name = extract_parameter_name_and_validate(line, param_file, n)

                if param_name in doc:
                    # If the parameter name is in the dictionary,
                    #  prefix the line with comment derived from the dictionary element
                    data = doc[param_name]
                    prefix_parts = [
                        f"{data['humanName']}",
                    ]
                    prefix_parts += data["documentation"]
                    for key, value in data["fields"].items():
                        prefix_parts.append(f"{key}: {value}")
                    prefix_parts += format_columns(data["values"])
                    doc_text = "\n# ".join(prefix_parts)  # pylint: disable=C0103
                    if not is_first_param_in_file:
                        new_lines.append("\n")
                    new_lines.append(f"# {doc_text}\n{line}\n")
                    documented_params += 1
                else:
                    # If the parameter name is in not the dictionary, copy the parameter line 1-to-1
                    new_lines.append(f"{line}\n")
                    undocumented_params.append(param_name)
                total_params += 1
                is_first_param_in_file = False

        if total_params == documented_params:
            logging.info("Read file %s with %d parameters, all got documented",
                         param_file, total_params)
        else:
            logging.warning("Read file %s with %d parameters, but only %s of which got documented",
                            param_file, total_params, documented_params)
            logging.warning("No documentation found for: %s", ", ".join(undocumented_params))

        # Write the new file contents to the file
        with open(param_file, "w", encoding="utf-8") as file:
            file.writelines(new_lines)


def print_read_only_params(doc):
    """
    Print the names of read-only parameters.

    Args:
        doc (dict): A dictionary of parameter documentation.
    """
    logging.info("ReadOnly parameters:")
    for param_name, param_value in doc.items():
        if 'ReadOnly' in param_value['fields'] and param_value['fields']['ReadOnly']:
            logging.info(param_name)


def main():
    args = arg_parser()
    try:
        xml_dir = args.target if os.path.isdir(args.target) else os.path.dirname(os.path.realpath(args.target))
        xml_root = get_xml_data(BASE_URL + args.vehicle_type + "/", xml_dir, PARAM_DEFINITION_XML_FILE)
        doc_dict = create_doc_dict(xml_root, args.vehicle_type)
        update_parameter_documentation(doc_dict, args.target, args.sort)
        if args.verbose:
            print_read_only_params(doc_dict)
    except Exception as exp:  # pylint: disable=W0718
        logging.fatal(exp)
        exit(1)  # pylint: disable=R1722


if __name__ == "__main__":
    main()
