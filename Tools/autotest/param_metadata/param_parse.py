#!/usr/bin/env python

import copy
import glob
import os
import re
import sys
from optparse import OptionParser

from param import (ParamNode, Parameter, Vehicle, known_group_fields,
                   known_param_fields, required_param_fields)
from htmlemit import HtmlEmit
from rstemit import RSTEmit
from wikiemit import WikiEmit
from xmlemit import XmlEmit

parser = OptionParser("param_parse.py [options]")
parser.add_option("-v", "--verbose", dest='verbose', action='store_true', default=False, help="show debugging output")
parser.add_option("--vehicle", default='*',  help="Vehicle type to generate for")
(opts, args) = parser.parse_args()


# Regular expressions for parsing the parameter metadata

prog_param = re.compile(r"@Param: *(\w+).*((?:\n[ \t]*// @(\w+): (.*))+)(?:\n\n|\n[ \t]+[A-Z])", re.MULTILINE)

prog_param_fields = re.compile(r"[ \t]*// @(\w+): (.*)")

prog_groups = re.compile(r"@Group: *(\w+).*((?:\n[ \t]*// @(Path): (\S+))+)", re.MULTILINE)

prog_group_param = re.compile(r"@Param: (\w+).*((?:\n[ \t]*// @(\w+): (.*))+)(?:\n\n|\n[ \t]+[A-Z])", re.MULTILINE)

apm_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')
vehicle_paths = glob.glob(apm_path + "%s/Parameters.cpp" % opts.vehicle)
extension = 'cpp'
if len(vehicle_paths) == 0:
    vehicle_paths = glob.glob(apm_path + "%s/Parameters.pde" % opts.vehicle)
    extension = 'pde'
vehicle_paths.sort(reverse=True)

vehicles = []

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

def fill_node(node, parent=None, prefix=""):
    p_text = ""
    debug("===\n\n\nProcessing %s" % node.Path)
    for path in node.Path.split(","):
        p_text += open(path).read()

    param_matches = prog_param.findall(p_text)
    group_matches = prog_groups.findall(p_text)

    debug(group_matches)
    for group_match in group_matches:
        if node.name is None:
            childname = group_match[0]
        else:
            childname = node.name + group_match[0]
        l = ParamNode(childname)
        fields = prog_param_fields.findall(group_match[1])
        for field in fields:
            if field[0] in known_group_fields:
                setattr(l, field[0], field[1])
            else:
                error("unknown parameter metadata field '%s'" % field[0])
        if hasattr(l, 'Path'):
            # make this path relative to the file it was found
            # in.  This is required when e.g. ArduCopter's
            # Parameters.cpp uses the Motors library and the
            # Motors library uses the RC_Channel library, as
            # Motors references RC_Channel relative to the Motors
            # directory
            l.Path = ','.join([ os.path.join(os.path.dirname(node.Path),x) for x in l.Path.split(',') ])

        node.add_child(l)

    for param_match in param_matches:
        if node.name is None:
            paramname = param_match[0]
        else:
            paramname = node.name + param_match[0]
        p = Parameter(paramname)
        debug(p.name + ' ')
        field_text = param_match[1]
        fields = prog_param_fields.findall(field_text)
        field_list = []
        for field in fields:
            field_list.append(field[0])
            if field[0] in known_param_fields:
                if getattr(p, field[0], None) is not None:
                    error("parameter metadata field '%s' specified twice for (%s) (file=%s)" % (field[0], p.name,os.path.realpath(node.Path)))
                setattr(p, field[0], field[1])
            else:
                error("unknown parameter metadata field '%s'" % field[0])
        for req_field in required_param_fields:
            if req_field not in field_list:
                error("missing parameter metadata field '%s' in %s" % (req_field, field_text))
        debug("Appending parameter (%s)" % p.name)
        node.params.append(p)

    debug("Found (%u) params and (%u) children" % (len(node.params),len(node.children)))

    for child in node.children:
        fill_node(child)

def do_emit_single(emit, root_nodes, flattened_nodes):
    print("vehicle_path count is %d" % len(vehicle_paths))
    # emit a libraries section to attempt to keep old links working:
    emit.set_annotate_with_vehicle(False)
    print("vehicle_paths=%s" % vehicle_paths)
    if len(vehicle_paths) > 1:
        emit.set_annotate_with_vehicle(True)
        for flat_node in flattened_nodes:
            print("**** %s" % flat_node.name)
            emit.emit(flat_node)
    else:
        for node in root_nodes:
            print("node is %s" % node.name)
            emit.emit(node)

    emit.close()

root_nodes = []

for vehicle_path in vehicle_paths:
    name = os.path.basename(os.path.dirname(vehicle_path))
#    path = os.path.normpath(os.path.dirname(vehicle_path))
#    vehicles.append(Vehicle(name, path))
#    debug('Found vehicle type %s' % name)

    root = ParamNode()
    root._vehicle = name
    root.Path = vehicle_path
    fill_node(root)
    root.validate()
    root_nodes.append(root)


def flatten_nodes(root_nodes):
    todo = root_nodes[:]
    flattened_nodes = []
    library_nodes = []
    while len(todo):
        node = todo.pop()
        todo.extend(node.children)

        my_copy = copy.deepcopy(node)
        if my_copy.name is None:
            # these are the root nodes
            continue
        have_node = None
        for flat_node in flattened_nodes:
            if flat_node.name == my_copy.name:
                have_node = flat_node
        if have_node is not None:
            print("Already have a a node called %s" % my_copy.name)
            # do some quick checks to warn of absolute insanity
            in_both = True
            for param in node.params:
                trimmed = re.sub('.*:','',param.name)
                if trimmed not in [ x.name for x in have_node.params ]:
                    print("  Parameter %s present in %s but not in %s" % (param.name, node.vehicle(), have_node.vehicle(),))
                    # try to merge this parameter into the libraries
                    print("   Merging parameter in")
                    x = copy.deepcopy(param)
                    x.name = re.sub('.*:','',x.name)
                    have_node.params.append(x)
                    in_both = False
            for param in have_node.params:
                trimmed_names = [ re.sub('.*:','',x.name) for x in node.params ]
                if param.name not in trimmed_names:
                    print("  Parameter %s present in %s but not in %s" % (param.name, have_node.name,node.name))
                    in_both = False;
            if in_both:
                # could make sanity checks here....
                pass
            continue
        my_copy.children = []
        for param in my_copy.params:
            param.name = re.sub('.*:','',param.name)
        flattened_nodes.append(my_copy)
    return sorted(flattened_nodes, key=lambda x : x.name)

if len(vehicle_paths) == 1:
    emit.set_annotate_with_vehicle(False)
else:
    nodes_to_emit = flatten_nodes(root_nodes)
    emit.set_annotate_with_vehicle(True)

do_emit(XmlEmit(), nodes_to_emit)
do_emit(WikiEmit(), nodes_to_emit)
do_emit(HtmlEmit(), nodes_to_emit)
do_emit(RSTEmit(), nodes_to_emit)

sys.exit(error_count)
