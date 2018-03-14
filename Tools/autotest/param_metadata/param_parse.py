#!/usr/bin/env python
from __future__ import print_function
import glob
import os
import re
import sys
from optparse import OptionParser

from param import (Library, Parameter, Vehicle, known_group_fields,
                   known_param_fields, required_param_fields, known_units)
from htmlemit import HtmlEmit
from rstemit import RSTEmit
from wikiemit import WikiEmit
from xmlemit import XmlEmit
from mdemit import MDEmit

parser = OptionParser("param_parse.py [options]")
parser.add_option("-v", "--verbose", dest='verbose', action='store_true', default=False, help="show debugging output")
parser.add_option("--vehicle", default='*',  help="Vehicle type to generate for")
parser.add_option("--no-emit", dest='emit_params', action='store_false', default=True, help="don't emit parameter documention, just validate")
(opts, args) = parser.parse_args()


# Regular expressions for parsing the parameter metadata

prog_param = re.compile(r"@Param: (\w+).*((?:\n[ \t]*// @(\w+)(?:{([^}]+)})?: (.*))+)(?:\n\n|\n[ \t]+[A-Z])", re.MULTILINE)

# match e.g @Value: 0=Unity, 1=Koala, 17=Liability
prog_param_fields = re.compile(r"[ \t]*// @(\w+): (.*)")
# match e.g @Value{ArduCopter}: 0=Volcano, 1=Peppermint
prog_param_tagged_fields = re.compile(r"[ \t]*// @(\w+){([^}]+)}: (.*)")

prog_groups = re.compile(r"@Group: *(\w+).*((?:\n[ \t]*// @(Path): (\S+))+)", re.MULTILINE)

apm_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')
vehicle_paths = glob.glob(apm_path + "%s/Parameters.cpp" % opts.vehicle)
extension = 'cpp'
if len(vehicle_paths) == 0:
    vehicle_paths = glob.glob(apm_path + "%s/Parameters.pde" % opts.vehicle)
    extension = 'pde'
vehicle_paths.sort(reverse=True)

vehicles = []
libraries = []

error_count = 0


def debug(str_to_print):
    """Debug output if verbose is set."""
    if opts.verbose:
        print(str_to_print)


def error(str_to_print):
    """Show errors."""
    global error_count
    error_count += 1
    print(str_to_print)

truename_map = {
    "APMrover2": "Rover",
    "ArduSub": "Sub",
    "ArduCopter": "Copter",
    "ArduPlane": "Plane",
    "AntennaTracker": "Tracker",
}
for vehicle_path in vehicle_paths:
    name = os.path.basename(os.path.dirname(vehicle_path))
    path = os.path.normpath(os.path.dirname(vehicle_path))
    vehicles.append(Vehicle(name, path, truename_map[name]))
    debug('Found vehicle type %s' % name)

if len(vehicles) > 1:
    print("Single vehicle only, please")
    sys.exit(1)

for vehicle in vehicles:
    debug("===\n\n\nProcessing %s" % vehicle.name)

    f = open(vehicle.path+'/Parameters.' + extension)
    p_text = f.read()
    f.close()

    param_matches = prog_param.findall(p_text)
    group_matches = prog_groups.findall(p_text)

    debug(group_matches)
    for group_match in group_matches:
        l = Library(group_match[0])
        fields = prog_param_fields.findall(group_match[1])
        for field in fields:
            if field[0] in known_group_fields:
                setattr(l, field[0], field[1])
            else:
                error("group: unknown parameter metadata field '%s'" % field[0])
        if not any(l.name == parsed_l.name for parsed_l in libraries):
            libraries.append(l)

    for param_match in param_matches:
        p = Parameter(vehicle.name+":"+param_match[0])
        debug(p.name + ' ')
        field_text = param_match[1]
        fields = prog_param_fields.findall(field_text)
        field_list = []
        for field in fields:
            field_list.append(field[0])
            if field[0] in known_param_fields:
                value = re.sub('@PREFIX@', "", field[1])
                setattr(p, field[0], value)
            else:
                error("param: unknown parameter metadata field '%s'" % field[0])
        for req_field in required_param_fields:
            if req_field not in field_list:
                error("missing parameter metadata field '%s' in %s" % (req_field, field_text))

        vehicle.params.append(p)

    debug("Processed %u params" % len(vehicle.params))

debug("Found %u documented libraries" % len(libraries))

alllibs = libraries[:]

vehicle = vehicles[0]

def process_library(vehicle, library, pathprefix=None):
    '''process one library'''
    paths = library.Path.split(',')
    for path in paths:
        path = path.strip()
        debug("\n Processing file '%s'" % path)
        if pathprefix is not None:
            libraryfname = os.path.join(pathprefix, path)            
        elif path.find('/') == -1:
            if len(vehicles) != 1:
                print("Unable to handle multiple vehicles with .pde library")
                continue
            libraryfname = os.path.join(vehicles[0].path, path)
        else:
            libraryfname = os.path.normpath(os.path.join(apm_path + '/libraries/' + path))
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
            p = Parameter(library.name+param_match[0])
            debug(p.name + ' ')
            field_text = param_match[1]
            fields = prog_param_fields.findall(field_text)
            for field in fields:
                if field[0] in known_param_fields:
                    value = re.sub('@PREFIX@', library.name, field[1])
                    setattr(p, field[0], value)
                else:
                    error("param: unknown parameter metadata field %s" % field[0])
            debug("matching %s" % field_text)
            fields = prog_param_tagged_fields.findall(field_text)
            for field in fields:
                only_for_vehicles = field[1].split(",")
                only_for_vehicles = [ x.rstrip().lstrip() for x in only_for_vehicles ]
                delta = set(only_for_vehicles) - set(truename_map.values())
                if len(delta):
                    error("Unknown vehicles (%s)" % delta)
                debug("field[0]=%s vehicle=%s truename=%s field[1]=%s only_for_vehicles=%s\n" % (field[0], vehicle.name,vehicle.truename,field[1], str(only_for_vehicles)))
                if vehicle.truename not in only_for_vehicles:
                    continue;
                if field[0] in known_param_fields:
                    value = re.sub('@PREFIX@', library.name, field[2])
                    setattr(p, field[0], value)
                else:
                    error("tagged param: unknown parameter metadata field '%s'" % field[0])
            library.params.append(p)

        group_matches = prog_groups.findall(p_text)
        debug("Found %u groups" % len(group_matches))
        debug(group_matches)
        for group_match in group_matches:
            group = group_match[0]
            debug("Group: %s" % group)
            l = Library(group)
            fields = prog_param_fields.findall(group_match[1])
            for field in fields:
                if field[0] in known_group_fields:
                    setattr(l, field[0], field[1])
                else:
                    error("unknown parameter metadata field '%s'" % field[0])
            if not any(l.name == parsed_l.name for parsed_l in libraries):
                l.name = library.name + l.name
                debug("Group name: %s" % l.name)
                process_library(vehicle, l, os.path.dirname(libraryfname))
                alllibs.append(l)

for library in libraries:
    debug("===\n\n\nProcessing library %s" % library.name)

    if hasattr(library, 'Path'):
        process_library(vehicle, library)
    else:
        error("Skipped: no Path found")

    debug("Processed %u documented parameters" % len(library.params))

# sort libraries by name
alllibs = sorted(alllibs, key=lambda x : x.name)

libraries = alllibs
    
def is_number(numberString):
    try:
        float(numberString)
        return True
    except ValueError:
        return False

def validate(param):
    """
    Validates the parameter meta data.
    """
    # Validate values
    if (hasattr(param, "Range")):
        rangeValues = param.__dict__["Range"].split(" ")
        if (len(rangeValues) != 2):
            error("Invalid Range values for %s" % (param.name))
            return
        min_value = rangeValues[0]
        max_value = rangeValues[1]
        if not is_number(min_value):
            error("Min value not number: %s %s" % (param.name, min_value))
            return
        if not is_number(max_value):
            error("Max value not number: %s %s" % (param.name, max_value))
            return
    # Validate units
    if (hasattr(param, "Units")):
        if (param.__dict__["Units"] != "") and (param.__dict__["Units"] not in known_units):
            error("unknown units field '%s'" % param.__dict__["Units"])

for vehicle in vehicles:
    for param in vehicle.params:
        validate(param)

for library in libraries:
    for param in library.params:
        validate(param)

def do_emit(emit):
    emit.set_annotate_with_vehicle(len(vehicles) > 1)
    for vehicle in vehicles:
        emit.emit(vehicle, f)

    emit.start_libraries()

    for library in libraries:
        if library.params:
            emit.emit(library, f)

    emit.close()

if opts.emit_params:
    do_emit(XmlEmit())
    do_emit(WikiEmit())
    do_emit(HtmlEmit())
    do_emit(RSTEmit())
    do_emit(MDEmit())

sys.exit(error_count)
