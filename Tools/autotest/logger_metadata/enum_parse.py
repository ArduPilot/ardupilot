#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import re
import sys

import enum_emit_rst
import enum_emit_python
# import enum_emit_xml

topdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')
topdir = os.path.realpath(topdir)

# TODO: validate URLS actually return 200
# TODO: augment with other information from log definitions; type and units...


class EnumDocco(object):

    vehicle_map = {
        "Rover": "Rover",
        "Sub": "ArduSub",
        "Copter": "ArduCopter",
        "Plane": "ArduPlane",
        "Tracker": "AntennaTracker",
    }

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.enumerations = []
        self.emitters = [
            enum_emit_rst.RSTEnumEmitter(),
            enum_emit_python.PythonEnumEmitter(),
#            emit_xml.XMLEmitter(),
        ]

        self.whitelist = {
            'RC_Channels::AUX_FUNC',
        }

    class EnumEntry(object):
        def __init__(self, name, value, comment):
            self.name = name
            self.value = value
            self.comment = comment

    def enumerations_from_file(self, source_file):
        state_outside = "outside"
        state_inside = "inside"

        state = state_outside

        enumerations = []
        with open(source_file) as f:
            enum_name = None
            in_class = None
            while state != "done":
                line = f.readline()
                if line == "":
                    break
                line = line.rstrip()
                #        print("state=%s line: %s" % (state, line))
                if re.match(" *//.*", line):
                    continue
                if state == "outside":
                    m = re.match("class .*;", line)
                    if m is not None:
                        # forward-declaration of a class
                        continue
                    m = re.match("class *(\w+)", line)
                    if m is not None:
                        in_class = m.group(1)
                        continue
                    m = re.match("namespace *(\w+)", line)
                    if m is not None:
                        in_class = m.group(1)
                        continue
                    m = re.match(".*enum\s*(class)? *([\w]+)\s*(?::.*_t)? *{(.*)};", line)
                    if m is not None:
                        # all one one line!  Thanks!
                        enum_name = m.group(2)
                        entries_string = m.group(3)
                        entry_names = [x.strip() for x in entries_string.split(",")]
                        count = 0
                        entries = []
                        for entry in entry_names:
                            entries.append(EnumDocco.EnumEntry(enum_name, count, None))
                            count += 1
                        new_enumeration = EnumDocco.Enumeration(enum_name, entries)
                        enumerations.append(new_enumeration)
                        continue

                    m = re.match(".*enum\s*(class)? *([\w]+)\s*(?::.*_t)? *{", line)
                    if m is not None:
                        enum_name = m.group(2)
                        print("%s: %s" % (source_file, enum_name))
                        entries = []
                        last_value = None
                        state = state_inside
                    continue
                if state == "inside":
                    if re.match("\s*$", line):
                        continue
                    if re.match("#if", line):
                        continue
                    if re.match("#endif", line):
                        continue
                    if re.match("#else", line):
                        continue
                    if re.match(".*}\s*\w*(\s*=\s*[\w:]+)?;", line):
                        if enum_name is None:
                            raise Exception("WT??")
                        if in_class is not None:
                            enum_name = "::".join([in_class, enum_name])
                        new_enumeration = EnumDocco.Enumeration(enum_name, entries)
                        enumerations.append(new_enumeration)
                        print("Got enum (%s)" % enum_name)
#                        for entry in new_enumeration.entries:
#                            print("   %s: %u (%s)" % (entry.name, entry.value, entry.comment))
                        state = state_outside
                        continue
                    m = re.match("\s*([A-Z0-9_a-z]+)(?: *= *([-0-9]+))?(?:, *// *(.*) *)?",
                                 line)
                    if m is None:
                        continue
                        raise ValueError("Line (%s) does not match" % (line))
                    name = m.group(1)
                    if m.group(2) is None:
                        if last_value is None:
                            value = 0
                            last_value = 0
                        else:
                            last_value += 1
                            value = last_value
                    else:
                        value = int(m.group(2))
                        last_value = value
                    comment = m.group(3)
#                    print("entry=%s value=%s comment=%s" % (name, value, comment))
                    entries.append(EnumDocco.EnumEntry(name, value, comment))
        return enumerations

    class Enumeration(object):

        def __init__(self, name, entries):
            self.name = name
            self.entries = entries

    def search_for_files(self, dirs_to_search):
        _next = []
        for _dir in dirs_to_search:
            for entry in os.listdir(_dir):
                if "AP_Scripting/lua" in _dir:
                    continue
                if "modules" in _dir:
                    continue
                if "examples" in _dir:
                    continue
                filepath = os.path.join(_dir, entry)
                if os.path.isdir(filepath):
                    _next.append(filepath)
                    continue
                (name, extension) = os.path.splitext(filepath)
                if extension not in [".cpp", ".h"]:
                    continue
                self.files.append(filepath)
        if len(_next):
            self.search_for_files(_next)

    def parse_files(self):
        for _file in self.files:
            self.enumerations.extend(self.enumerations_from_file(_file))

    def should_emit_enumeration(self, name):
        return True
        if name in self.whitelist:
            return True
        return False

    def emit_output(self):
        # expand things like PIDR,PIDQ,PIDA into multiple doccos
        new_enumerations = []
        for enumeration in self.enumerations:
            if self.should_emit_enumeration(enumeration):
                new_enumerations.append(enumeration)

        for emitter in self.emitters:
            emitter.emit(self.vehicle_map[self.vehicle], new_enumerations)

    def get_enumerations(self):
        self.files = []
        self.search_for_files([self.vehicle_map[self.vehicle], "libraries"])
        self.parse_files()
        return self.enumerations

    def run(self):
        self.get_enumerations()
        self.emit_output()

    def finalise_docco(self, docco):
        self.doccos.append(docco)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Parse parameters.")
    parser.add_argument("-v", "--verbose", dest='verbose', action='store_true', default=False, help="show debugging output")
    parser.add_argument("--vehicle", required=True, help="Vehicle type to generate for")

    args = parser.parse_args()

    s = EnumDocco(args.vehicle)

    if args.vehicle not in s.vehicle_map:
        print("Invalid vehicle (choose from: %s)" % str(s.vehicle_map.keys()))
        sys.exit(1)

    s.run()
