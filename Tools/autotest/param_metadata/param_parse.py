#!/usr/bin/env python
from __future__ import print_function
import glob
import os
import re
import sys
from argparse import ArgumentParser

from param import (Library, Parameter, Vehicle, known_group_fields,
                   known_param_fields, required_param_fields, known_units)
from htmlemit import HtmlEmit
from rstemit import RSTEmit
from wikiemit import WikiEmit
from xmlemit import XmlEmit
from mdemit import MDEmit

parser = ArgumentParser(description="Parse ArduPilot parameters.")
parser.add_argument("-v", "--verbose", dest='verbose', action='store_true', default=False, help="show debugging output")
parser.add_argument("--vehicle", required=True, help="Vehicle type to generate for")
parser.add_argument("--no-emit",
                    dest='emit_params',
                    action='store_false',
                    default=True,
                    help="don't emit parameter documention, just validate")
parser.add_argument("--format",
                    dest='output_format',
                    action='store',
                    default='all',
                    choices=['all', 'html', 'rst', 'wiki', 'xml', 'edn', 'md'],
                    help="what output format to use")
args = parser.parse_args()


# Regular expressions for parsing the parameter metadata

prog_param = re.compile(r"@Param: (\w+).*((?:\n[ \t]*// @(\w+)(?:{([^}]+)})?: (.*))+)(?:\n\n|\n[ \t]+[A-Z])", re.MULTILINE)

# match e.g @Value: 0=Unity, 1=Koala, 17=Liability
prog_param_fields = re.compile(r"[ \t]*// @(\w+): (.*)")
# match e.g @Value{Copter}: 0=Volcano, 1=Peppermint
prog_param_tagged_fields = re.compile(r"[ \t]*// @(\w+){([^}]+)}: (.*)")

prog_groups = re.compile(r"@Group: *(\w+).*((?:\n[ \t]*// @(Path): (\S+))+)", re.MULTILINE)

apm_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../")
vehicle_path = glob.glob(apm_path + "%s/Parameters.cpp" % args.vehicle)[0]

libraries_list = []

error_count = 0
current_param = None
current_file = None


def debug(str_to_print):
    """Debug output if verbose is set."""
    if args.verbose:
        print(str_to_print)


def error(str_to_print):
    """Show errors."""
    global error_count
    error_count += 1
    if current_file is not None:
        print("In %s" % current_file)
    if current_param is not None:
        print("At param %s" % current_param)
    print(str_to_print)


truename_map = {
    "APMrover2": "Rover",
    "ArduSub": "Sub",
    "ArduCopter": "Copter",
    "ArduPlane": "Plane",
    "AntennaTracker": "Tracker",
}

vehicle_name = os.path.basename(os.path.dirname(vehicle_path))
vehicle_path = os.path.normpath(os.path.dirname(vehicle_path))
vehicle = Vehicle(vehicle_name, vehicle_path, truename_map[vehicle_name])
debug("Found vehicle type %s" % vehicle_name)


def process_vehicle():
    debug("===\n\n\nProcessing %s" % vehicle.name)
    global current_file, current_param
    current_file = vehicle.path + "/Parameters.cpp"

    f = open(current_file)
    p_text = f.read()
    f.close()
    param_matches = prog_param.findall(p_text)
    group_matches = prog_groups.findall(p_text)
    debug("Found %u groups" % len(group_matches))
    debug(group_matches)
    for group_match in group_matches:
        group = group_match[0]
        debug("Group: %s" % group)
        group_lib = Library(group)
        fields = prog_param_fields.findall(group_match[1])
        for field in fields:
            if field[0] in known_group_fields:
                setattr(group_lib, field[0], field[1])
            else:
                error("group: unknown parameter metadata field '%s'" % field[0])
        if not any(group_lib.name == parsed_l.name for parsed_l in libraries_list):
            libraries_list.append(group_lib)

    debug("Found %u documented parameters" % len(param_matches))
    for param_match in param_matches:
        p = Parameter(vehicle.name + ":" + param_match[0], current_file)
        debug(p.name + ' ')
        current_param = p.name
        field_text = param_match[1]
        fields = prog_param_fields.findall(field_text)
        field_list = []
        for field in fields:
            field_list.append(field[0])
            if field[0] in known_param_fields:
                value = re.sub("@PREFIX@", "", field[1])
                setattr(p, field[0], value)
            else:
                error("param: unknown parameter metadata field '%s'" % field[0])
        for req_field in required_param_fields:
            if req_field not in field_list:
                error("missing parameter metadata field '%s' in %s" % (req_field, field_text))

        vehicle.params.append(p)
    current_file = None
    debug("Processed %u params" % len(vehicle.params))


process_vehicle()
debug("Found %u documented libraries" % len(libraries_list))
alllibs = libraries_list[:]


def process_library(library, pathprefix=None):
    """process one library"""
    paths = library.Path.split(',')
    for path in paths:
        path = path.strip()
        global current_file
        current_file = path
        debug("\n Processing file '%s'" % path)
        if pathprefix is not None:
            libraryfname = os.path.join(pathprefix, path)
        elif path.find('/') == -1:
            libraryfname = os.path.join(vehicle.path, path)
        else:
            libraryfname = os.path.normpath(os.path.join(apm_path + "/libraries/" + path))
        if path and os.path.exists(libraryfname):
            f = open(libraryfname)
            p_text = f.read()
            f.close()
        else:
            error("Path %s not found for library %s" % (path, library.name))
            continue

        param_matches = prog_param.findall(p_text)
        debug("Found %u documented parameters" % len(param_matches))
        for param_match in param_matches:
            p = Parameter(library.name + param_match[0], current_file)
            debug(p.name + ' ')
            global current_param
            current_param = p.name
            field_text = param_match[1]
            fields = prog_param_fields.findall(field_text)
            non_vehicle_specific_values_seen = False
            for field in fields:
                if field[0] in known_param_fields:
                    value = re.sub("@PREFIX@", library.name, field[1])
                    setattr(p, field[0], value)
                    if field[0] == "Values":
                        non_vehicle_specific_values_seen = True
                else:
                    error("param: unknown parameter metadata field %s" % field[0])
            debug("matching %s" % field_text)
            fields = prog_param_tagged_fields.findall(field_text)
            this_vehicle_values_seen = False
            this_vehicle_value = None
            other_vehicle_values_seen = False
            for field in fields:
                only_for_vehicles = field[1].split(",")
                only_for_vehicles = [x.rstrip().lstrip() for x in only_for_vehicles]
                delta = set(only_for_vehicles) - set(truename_map.values())
                if len(delta):
                    error("Unknown vehicles (%s)" % delta)
                debug("field[0]=%s vehicle=%s truename=%s field[1]=%s only_for_vehicles=%s\n" %
                      (field[0], vehicle.name, vehicle.truename, field[1], str(only_for_vehicles)))
                value = re.sub("@PREFIX@", library.name, field[2])
                if field[0] == "Values":
                    if vehicle.truename in only_for_vehicles:
                        this_vehicle_values_seen = True
                        this_vehicle_value = value
                        if len(only_for_vehicles) > 1:
                            other_vehicle_values_seen = True
                    elif len(only_for_vehicles):
                        other_vehicle_values_seen = True
                if field[0] in known_param_fields:
                    setattr(p, field[0], value)
                else:
                    error("tagged param<: unknown parameter metadata field '%s'" % field[0])
            if (non_vehicle_specific_values_seen or not other_vehicle_values_seen) or this_vehicle_values_seen:
                if this_vehicle_values_seen:
                    debug("Setting vehicle-specific value (%s)" % str(this_vehicle_value))
                    setattr(p, field[0], this_vehicle_value)
#                debug("Appending (non_vehicle_specific_values_seen=%u "
#                      "other_vehicle_values_seen=%u this_vehicle_values_seen=%u)" %
#                      (non_vehicle_specific_values_seen, other_vehicle_values_seen, this_vehicle_values_seen))
                p.path = path # Add path. Later deleted - only used for duplicates
                library.params.append(p)

        group_matches = prog_groups.findall(p_text)
        debug("Found %u groups" % len(group_matches))
        debug(group_matches)
        for group_match in group_matches:
            group = group_match[0]
            debug("Group: %s" % group)
            group_lib = Library(group)
            fields = prog_param_fields.findall(group_match[1])
            for field in fields:
                if field[0] in known_group_fields:
                    setattr(group_lib, field[0], field[1])
                else:
                    error("group: unknown parameter metadata field '%s'" % field[0])
            if not any(group_lib.name == parsed_l.name for parsed_l in libraries_list):
                group_lib.name = library.name + group_lib.name
                debug("Group name: %s" % group_lib.name)
                if hasattr(group_lib, "Path"):
                    process_library(group_lib, os.path.dirname(libraryfname))
                else:
                    error("Skipped: no Path found")
                alllibs.append(group_lib)

    current_file = None


for lib in libraries_list:
    debug("===\n\n\nProcessing library %s" % lib.name)

    if hasattr(lib, "Path"):
        process_library(lib)
    else:
        error("Skipped: no Path found")

    debug("Processed %u documented parameters" % len(lib.params))

# sort libraries by name
alllibs = sorted(alllibs, key=lambda x: x.name)

libraries_list = alllibs


def is_number(numberString):
    try:
        float(numberString)
        return True
    except ValueError:
        return False


def validate(parameter):
    """
    Validates the parameter meta data.
    """
    global current_file
    current_file = parameter.real_path
    global current_param
    current_param = parameter.name
    # Validate values
    if hasattr(parameter, "Range"):
        rangeValues = parameter.__dict__["Range"].split(" ")
        if len(rangeValues) != 2:
            error("Invalid Range values for %s" % parameter.name)
            return
        min_value = rangeValues[0]
        max_value = rangeValues[1]
        if not is_number(min_value):
            error("Min value not number: %s %s" % (parameter.name, min_value))
            return
        if not is_number(max_value):
            error("Max value not number: %s %s" % (parameter.name, max_value))
            return
    # Validate units
    if hasattr(parameter, "Units"):
        if (parameter.__dict__["Units"] != "") and (parameter.__dict__["Units"] not in known_units):
            error("unknown units field '%s'" % parameter.__dict__["Units"])


for param in vehicle.params:
    validate(param)

# Find duplicate names in library and fix up path
for lib in libraries_list:
    param_names_seen = set()
    param_names_duplicate = set()
    # Find duplicates:
    for param in lib.params:
        if param.name in param_names_seen:  # is duplicate
            param_names_duplicate.add(param.name)
        param_names_seen.add(param.name)
    # Fix up path for duplicates
    for param in lib.params:
        if param.name in param_names_duplicate:
            param.path = param.path.rsplit('/')[-1].rsplit('.')[0]
        else:
            # not a duplicate, so delete attribute.
            delattr(param, "path")

for lib in libraries_list:
    for param in lib.params:
        validate(param)


def do_emit(emit):
    emit.set_annotate_with_vehicle(False)
    emit.emit(vehicle)

    emit.start_libraries()

    for library in libraries_list:
        if library.params:
            emit.emit(library)

    emit.close()


if args.emit_params:
    if args.output_format == 'all' or args.output_format == 'xml':
        do_emit(XmlEmit())
    if args.output_format == 'all' or args.output_format == 'wiki':
        do_emit(WikiEmit())
    if args.output_format == 'all' or args.output_format == 'html':
        do_emit(HtmlEmit())
    if args.output_format == 'all' or args.output_format == 'rst':
        do_emit(RSTEmit())
    if args.output_format == 'all' or args.output_format == 'md':
        do_emit(MDEmit())
    if args.output_format == 'all' or args.output_format == 'edn':
        try:
            from ednemit import EDNEmit
            do_emit(EDNEmit())
        except ImportError:
            # if the user wanted edn only then don't hide any errors
            if args.output_format == 'edn':
                raise

            if args.verbose:
                print("Unable to emit EDN, install edn_format and pytz if edn is desired")

sys.exit(error_count)
