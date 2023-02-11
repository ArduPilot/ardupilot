#!/usr/bin/env python

'''Generates parameter metadata files suitable for consumption by
  ground control stations and various web services

  AP_FLAKE8_CLEAN

'''

from __future__ import print_function
import copy
import os
import re
import sys
import glob
from argparse import ArgumentParser

from param import (Library, Parameter, Vehicle, known_group_fields,
                   known_param_fields, required_param_fields, required_library_param_fields, known_units)
from htmlemit import HtmlEmit
from rstemit import RSTEmit
from rstlatexpdfemit import RSTLATEXPDFEmit
from xmlemit import XmlEmit
from mdemit import MDEmit
from jsonemit import JSONEmit
from xmlemit_mp import XmlEmitMP

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
                    choices=['all', 'html', 'rst', 'rstlatexpdf', 'wiki', 'xml', 'json', 'edn', 'md', 'xml_mp'],
                    help="what output format to use")

args = parser.parse_args()


# Regular expressions for parsing the parameter metadata

prog_param = re.compile(r"@Param(?:{([^}]+)})?: (\w+).*((?:\n[ \t]*// @(\w+)(?:{([^}]+)})?: ?(.*))+)(?:\n[ \t\r]*\n|\n[ \t]+[A-Z]|\n\-\-\]\])", re.MULTILINE)  # noqa

# match e.g @Value: 0=Unity, 1=Koala, 17=Liability
prog_param_fields = re.compile(r"[ \t]*// @(\w+): ?([^\r\n]*)")
# match e.g @Value{Copter}: 0=Volcano, 1=Peppermint
prog_param_tagged_fields = re.compile(r"[ \t]*// @(\w+){([^}]+)}: ([^\r\n]*)")

prog_groups = re.compile(r"@Group: *(\w+).*((?:\n[ \t]*// @(Path): (\S+))+)", re.MULTILINE)

apm_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')


def find_vehicle_parameter_filepath(vehicle_name):
    apm_tools_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../Tools/')

    vehicle_name_to_dir_name_map = {
        "Copter": "ArduCopter",
        "Plane": "ArduPlane",
        "Tracker": "AntennaTracker",
        "Sub": "ArduSub",
    }

    # first try ArduCopter/Parmameters.cpp
    for top_dir in apm_path, apm_tools_path:
        path = os.path.join(top_dir, vehicle_name, "Parameters.cpp")
        if os.path.exists(path):
            return path

        # then see if we can map e.g. Copter -> ArduCopter
        if vehicle_name in vehicle_name_to_dir_name_map:
            path = os.path.join(top_dir, vehicle_name_to_dir_name_map[vehicle_name], "Parameters.cpp")
            if os.path.exists(path):
                return path

    raise ValueError("Unable to find parameters file for (%s)" % vehicle_name)


def debug(str_to_print):
    """Debug output if verbose is set."""
    if args.verbose:
        print(str_to_print)


def lua_applets():
    '''return list of Library objects for lua applets and drivers'''
    lua_lib = Library("", reference="Lua Script", not_rst=True, check_duplicates=True)
    patterns = ["libraries/AP_Scripting/applets/*.lua", "libraries/AP_Scripting/drivers/*.lua"]
    paths = []
    for p in patterns:
        debug("Adding lua paths %s" % p)
        luafiles = glob.glob(os.path.join(apm_path, p))
        for f in luafiles:
            # the library is expected to have the path as a relative path from within
            # a vehicle directory
            f = f.replace(apm_path, "../")
            paths.append(f)
    setattr(lua_lib, "Path", ','.join(paths))
    return lua_lib


libraries = []

# AP_Vehicle also has parameters rooted at "", but isn't referenced
# from the vehicle in any way:
ap_vehicle_lib = Library("") # the "" is tacked onto the front of param name
setattr(ap_vehicle_lib, "Path", os.path.join('..', 'libraries', 'AP_Vehicle', 'AP_Vehicle.cpp'))
libraries.append(ap_vehicle_lib)

libraries.append(lua_applets())

error_count = 0
current_param = None
current_file = None


def error(str_to_print):
    """Show errors."""
    global error_count
    error_count += 1
    if current_file is not None:
        print("Error in %s" % current_file)
    if current_param is not None:
        print("At param %s" % current_param)
    print(str_to_print)


truename_map = {
    "Rover": "Rover",
    "ArduSub": "Sub",
    "ArduCopter": "Copter",
    "ArduPlane": "Plane",
    "AntennaTracker": "Tracker",
    "AP_Periph": "AP_Periph",
    "Blimp": "Blimp",
}
valid_truenames = frozenset(truename_map.values())
truename = truename_map.get(args.vehicle, args.vehicle)

documentation_tags_which_are_comma_separated_nv_pairs = frozenset([
    'Values',
    'Bitmask',
])

vehicle_path = find_vehicle_parameter_filepath(args.vehicle)

basename = os.path.basename(os.path.dirname(vehicle_path))
path = os.path.normpath(os.path.dirname(vehicle_path))
reference = basename  # so links don't break we use ArduCopter
vehicle = Vehicle(truename, path, reference=reference)
debug('Found vehicle type %s' % vehicle.name)


def process_vehicle(vehicle):
    debug("===\n\n\nProcessing %s" % vehicle.name)
    current_file = vehicle.path+'/Parameters.cpp'

    f = open(current_file)
    p_text = f.read()
    f.close()
    group_matches = prog_groups.findall(p_text)

    debug(group_matches)
    for group_match in group_matches:
        lib = Library(group_match[0])
        fields = prog_param_fields.findall(group_match[1])
        for field in fields:
            if field[0] in known_group_fields:
                setattr(lib, field[0], field[1])
            else:
                error("group: unknown parameter metadata field '%s'" % field[0])
        if not any(lib.name == parsed_l.name for parsed_l in libraries):
            libraries.append(lib)

    param_matches = []
    param_matches = prog_param.findall(p_text)

    for param_match in param_matches:
        (only_vehicles, param_name, field_text) = (param_match[0],
                                                   param_match[1],
                                                   param_match[2])
        if len(only_vehicles):
            only_vehicles_list = [x.strip() for x in only_vehicles.split(",")]
            for only_vehicle in only_vehicles_list:
                if only_vehicle not in valid_truenames:
                    raise ValueError("Invalid only_vehicle %s" % only_vehicle)
            if vehicle.truename not in only_vehicles_list:
                continue
        p = Parameter(vehicle.reference+":"+param_name, current_file)
        debug(p.name + ' ')
        global current_param
        current_param = p.name
        fields = prog_param_fields.findall(field_text)
        p.__field_text = field_text
        field_list = []
        for field in fields:
            (field_name, field_value) = field
            field_list.append(field[0])
            if field[0] in known_param_fields:
                value = re.sub('@PREFIX@', "", field[1]).rstrip()
                if hasattr(p, field_name):
                    if field_name in documentation_tags_which_are_comma_separated_nv_pairs:
                        # allow concatenation of (e.g.) bitmask fields
                        x = eval("p.%s" % field_name)
                        x += ", "
                        x += value
                        value = x
                    else:
                        error("%s already has field %s" % (p.name, field_name))
                setattr(p, field[0], value)
            elif field[0] in frozenset(["CopyFieldsFrom", "CopyValuesFrom"]):
                setattr(p, field[0], field[1])
            else:
                error("param: unknown parameter metadata field '%s'" % field[0])

        vehicle.params.append(p)
    current_file = None
    debug("Processed %u params" % len(vehicle.params))


process_vehicle(vehicle)

debug("Found %u documented libraries" % len(libraries))

libraries = list(libraries)

alllibs = libraries[:]


def process_library(vehicle, library, pathprefix=None):
    '''process one library'''
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
            libraryfname = os.path.normpath(os.path.join(apm_path + '/libraries/' + path))
        if path and os.path.exists(libraryfname):
            f = open(libraryfname)
            p_text = f.read()
            f.close()
        else:
            error("Path %s not found for library %s (fname=%s)" % (path, library.name, libraryfname))
            continue

        param_matches = prog_param.findall(p_text)
        debug("Found %u documented parameters" % len(param_matches))
        for param_match in param_matches:
            (only_vehicles, param_name, field_text) = (param_match[0],
                                                       param_match[1],
                                                       param_match[2])
            if len(only_vehicles):
                only_vehicles_list = [x.strip() for x in only_vehicles.split(",")]
                for only_vehicle in only_vehicles_list:
                    if only_vehicle not in valid_truenames:
                        raise ValueError("Invalid only_vehicle %s" % only_vehicle)
                if vehicle.name not in only_vehicles_list:
                    continue
            p = Parameter(library.name+param_name, current_file)
            debug(p.name + ' ')
            global current_param
            current_param = p.name
            fields = prog_param_fields.findall(field_text)
            p.__field_text = field_text
            field_list = []
            for field in fields:
                (field_name, field_value) = field
                field_list.append(field[0])
                if field[0] in known_param_fields:
                    value = re.sub('@PREFIX@', library.name, field[1])
                    if hasattr(p, field_name):
                        if field_name in documentation_tags_which_are_comma_separated_nv_pairs:
                            # allow concatenation of (e.g.) bitmask fields
                            x = eval("p.%s" % field_name)
                            x += ", "
                            x += value
                            value = x
                        else:
                            error("%s already has field %s" % (p.name, field_name))
                    setattr(p, field[0], value)
                elif field[0] in frozenset(["CopyFieldsFrom", "CopyValuesFrom"]):
                    setattr(p, field[0], field[1])
                else:
                    error("param: unknown parameter metadata field %s" % field[0])

            debug("matching %s" % field_text)
            fields = prog_param_tagged_fields.findall(field_text)
            # a parameter is considered to be vehicle-specific if
            # there does not exist a Values: or Values{VehicleName}
            # for that vehicle but @Values{OtherVehicle} exists.
            seen_values_or_bitmask_for_other_vehicle = False
            for field in fields:
                only_for_vehicles = field[1].split(",")
                only_for_vehicles = [some_vehicle.rstrip().lstrip() for some_vehicle in only_for_vehicles]
                delta = set(only_for_vehicles) - set(truename_map.values())
                if len(delta):
                    error("Unknown vehicles (%s)" % delta)
                debug("field[0]=%s vehicle=%s field[1]=%s only_for_vehicles=%s\n" %
                      (field[0], vehicle.name, field[1], str(only_for_vehicles)))
                if field[0] not in known_param_fields:
                    error("tagged param: unknown parameter metadata field '%s'" % field[0])
                    continue
                if vehicle.name not in only_for_vehicles:
                    if len(only_for_vehicles) and field[0] in ['Values', 'Bitmask']:
                        seen_values_or_bitmask_for_other_vehicle = True
                    continue
                value = re.sub('@PREFIX@', library.name, field[2])
                setattr(p, field[0], value)

            if (getattr(p, 'Values', None) is not None and
                    getattr(p, 'Bitmask', None) is not None):
                error("Both @Values and @Bitmask present")

            if (getattr(p, 'Values', None) is None and
                    getattr(p, 'Bitmask', None) is None):
                # values and Bitmask available for this vehicle
                if seen_values_or_bitmask_for_other_vehicle:
                    # we've (e.g.) seen @Values{Copter} when we're
                    # processing for Rover, and haven't seen either
                    # @Values: or @Vales{Rover} - so we omit this
                    # parameter on the assumption that it is not
                    # applicable for this vehicle.
                    continue

            p.path = path # Add path. Later deleted - only used for duplicates
            if library.check_duplicates and library.has_param(p.name):
                error("Duplicate parameter %s in %s" % (p.name, library.name))
                continue
            library.params.append(p)

        group_matches = prog_groups.findall(p_text)
        debug("Found %u groups" % len(group_matches))
        debug(group_matches)
        done_groups = dict()
        for group_match in group_matches:
            group = group_match[0]
            debug("Group: %s" % group)
            do_append = True
            if group in done_groups:
                # this is to handle cases like the RangeFinder
                # parameters, where the wasp stuff gets tack into the
                # same RNGFND1_ group
                lib = done_groups[group]
                do_append = False
            else:
                lib = Library(group)
                done_groups[group] = lib

            fields = prog_param_fields.findall(group_match[1])
            for field in fields:
                if field[0] in known_group_fields:
                    setattr(lib, field[0], field[1])
                elif field[0] in ["CopyFieldsFrom", "CopyValuesFrom"]:
                    setattr(p, field[0], field[1])
                else:
                    error("unknown parameter metadata field '%s'" % field[0])
            if not any(lib.name == parsed_l.name for parsed_l in libraries):
                if do_append:
                    lib.set_name(library.name + lib.name)
                debug("Group name: %s" % lib.name)
                process_library(vehicle, lib, os.path.dirname(libraryfname))
                if do_append:
                    alllibs.append(lib)

    current_file = None


for library in libraries:
    debug("===\n\n\nProcessing library %s" % library.name)

    if hasattr(library, 'Path'):
        process_library(vehicle, library)
    else:
        error("Skipped: no Path found")

    debug("Processed %u documented parameters" % len(library.params))

# sort libraries by name
alllibs = sorted(alllibs, key=lambda x: x.name)

libraries = alllibs


def is_number(numberString):
    try:
        float(numberString)
        return True
    except ValueError:
        return False


def clean_param(param):
    if (hasattr(param, "Values")):
        valueList = param.Values.split(",")
        new_valueList = []
        for i in valueList:
            (start, sep, end) = i.partition(":")
            if sep != ":":
                raise ValueError("Expected a colon seperator in (%s)" % (i,))
            if len(end) == 0:
                raise ValueError("Expected a colon-separated string, got (%s)" % i)
            end = end.strip()
            start = start.strip()
            new_valueList.append(":".join([start, end]))
        param.Values = ",".join(new_valueList)


def do_copy_values(vehicle_params, libraries, param):
    if not hasattr(param, "CopyValuesFrom"):
        return

    # so go and find the values...
    wanted_name = param.CopyValuesFrom
    del param.CopyValuesFrom
    for x in vehicle_params:
        name = x.name
        (v, name) = name.split(":")
        if name != wanted_name:
            continue
        param.Values = x.Values
        return

    for lib in libraries:
        for x in lib.params:
            if x.name != wanted_name:
                continue
            param.Values = x.Values
            return

    error("Did not find value to copy (%s wants %s)" %
          (param.name, wanted_name))


def do_copy_fields(vehicle_params, libraries, param):
    do_copy_values(vehicle_params, libraries, param)

    if not hasattr(param, 'CopyFieldsFrom'):
        return

    # so go and find the values...
    wanted_name = param.CopyFieldsFrom
    del param.CopyFieldsFrom
    for x in vehicle_params:
        name = x.name
        (v, name) = name.split(":")
        if name != wanted_name:
            continue
        for field in dir(x):
            if hasattr(param, field):
                # override
                continue
            if field.startswith("__") or field in frozenset(["name", "real_path"]):
                # internal methods like __ne__
                continue
            setattr(param, field, getattr(x, field))
        return

    for lib in libraries:
        for x in lib.params:
            if x.name != wanted_name:
                continue
            for field in dir(x):
                if hasattr(param, field):
                    # override
                    continue
                if field.startswith("__") or field in frozenset(["name", "real_path"]):
                    # internal methods like __ne__
                    continue
                setattr(param, field, getattr(x, field))
            return

    error("Did not find value to copy (%s wants %s)" %
          (param.name, wanted_name))


def validate(param, is_library=False):
    """
    Validates the parameter meta data.
    """
    global current_file
    current_file = param.real_path
    global current_param
    current_param = param.name
    # Validate values
    if (hasattr(param, "Range")):
        rangeValues = param.__dict__["Range"].split(" ")
        if (len(rangeValues) != 2):
            error("Invalid Range values for %s (%s)" %
                  (param.name, param.__dict__["Range"]))
            return
        min_value = rangeValues[0]
        max_value = rangeValues[1]
        if not is_number(min_value):
            error("Min value not number: %s %s" % (param.name, min_value))
            return
        if not is_number(max_value):
            error("Max value not number: %s %s" % (param.name, max_value))
            return
    # Check for duplicate in @value field
    if (hasattr(param, "Values")):
        valueList = param.__dict__["Values"].split(",")
        values = []
        for i in valueList:
            i = i.replace(" ", "")
            values.append(i.partition(":")[0])
        if (len(values) != len(set(values))):
            error("Duplicate values found")
    # Validate units
    if (hasattr(param, "Units")):
        if (param.__dict__["Units"] != "") and (param.__dict__["Units"] not in known_units):
            error("unknown units field '%s'" % param.__dict__["Units"])
    # Validate User
    if (hasattr(param, "User")):
        if param.User.strip() not in ["Standard", "Advanced"]:
            error("unknown user (%s)" % param.User.strip())

    if (hasattr(param, "Description")):
        if not param.Description or not param.Description.strip():
            error("Empty Description (%s)" % param)

    required_fields = required_param_fields
    if is_library:
        required_fields = required_library_param_fields
    for req_field in required_fields:
        if not getattr(param, req_field, False):
            error("missing parameter metadata field '%s' in %s" % (req_field, param.__field_text))


# handle CopyFieldsFrom and CopyValuesFrom:
for param in vehicle.params:
    do_copy_fields(vehicle.params, libraries, param)
for library in libraries:
    for param in library.params:
        do_copy_fields(vehicle.params, libraries, param)

for param in vehicle.params:
    clean_param(param)

for param in vehicle.params:
    validate(param)

# Find duplicate names in library and fix up path
for library in libraries:
    param_names_seen = set()
    param_names_duplicate = set()
    # Find duplicates:
    for param in library.params:
        if param.name in param_names_seen:  # is duplicate
            param_names_duplicate.add(param.name)
        param_names_seen.add(param.name)
    # Fix up path for duplicates
    for param in library.params:
        if param.name in param_names_duplicate:
            param.path = param.path.rsplit('/')[-1].rsplit('.')[0]
        else:
            # not a duplicate, so delete attribute.
            delattr(param, "path")

for library in libraries:
    for param in library.params:
        clean_param(param)

for library in libraries:
    for param in library.params:
        validate(param, is_library=True)

if not args.emit_params:
    sys.exit(error_count)

all_emitters = {
    'json': JSONEmit,
    'xml': XmlEmit,
    'html': HtmlEmit,
    'rst': RSTEmit,
    'rstlatexpdf': RSTLATEXPDFEmit,
    'md': MDEmit,
    'xml_mp': XmlEmitMP,
}

try:
    from ednemit import EDNEmit
    all_emitters['edn'] = EDNEmit
except ImportError:
    # if the user wanted edn only then don't hide any errors
    if args.output_format == 'edn':
        raise

    if args.verbose:
        print("Unable to emit EDN, install edn_format and pytz if edn is desired")

# filter to just the ones we want to emit:
emitters_to_use = []
for emitter_name in all_emitters.keys():
    if args.output_format == 'all' or args.output_format == emitter_name:
        emitters_to_use.append(emitter_name)

# actually invoke each emitter:
for emitter_name in emitters_to_use:
    emit = all_emitters[emitter_name]()

    emit.emit(vehicle)

    emit.start_libraries()

    # create a single parameter list for all SIM_ parameters (for rst to use)
    sim_params = []
    for library in libraries:
        if library.name.startswith("SIM_"):
            sim_params.extend(library.params)
    sim_params = sorted(sim_params, key=lambda x : x.name)

    for library in libraries:
        if library.params:
            # we sort the parameters in the SITL library to avoid
            # rename, and on the assumption that an asciibetical sort
            # gives a good layout:
            if emitter_name == 'rst':
                if library.not_rst:
                    continue
                if library.name == 'SIM_':
                    library = copy.deepcopy(library)
                    library.params = sim_params
                elif library.name.startswith('SIM_'):
                    continue
            emit.emit(library)

    emit.close()

sys.exit(error_count)
