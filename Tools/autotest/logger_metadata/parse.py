#!/usr/bin/env python

from __future__ import print_function

import argparse
import copy
import os
import re
import sys

import emit_html
import emit_rst
import emit_xml
import emit_md

import enum_parse
from enum_parse import EnumDocco

topdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')
topdir = os.path.realpath(topdir)

re_loggermessage = re.compile(r"@LoggerMessage\s*:\s*([\w,]+)", re.MULTILINE)
re_commentline = re.compile(r"\s*//")
re_description = re.compile(r"\s*//\s*@Description\s*:\s*(.*)")
re_url = re.compile(r"\s*//\s*@URL\s*:\s*(.*)")
re_field = re.compile(r"\s*//\s*@Field\s*:\s*(\w+):\s*(.*)")
re_fieldbits = re.compile(r"\s*//\s*@FieldBits\s*:\s*(\w+):\s*(.*)")
re_fieldbitmaskenum = re.compile(r"\s*//\s*@FieldBitmaskEnum\s*:\s*(\w+):\s*(.*)")
re_fieldvalueenum = re.compile(r"\s*//\s*@FieldValueEnum\s*:\s*(\w+):\s*(.*)")
re_vehicles = re.compile(r"\s*//\s*@Vehicles\s*:\s*(.*)")

# TODO: validate URLS actually return 200
# TODO: augment with other information from log definitions; type and units...


class LoggerDocco(object):

    vehicle_map = {
        "Rover": "Rover",
        "Sub": "ArduSub",
        "Copter": "ArduCopter",
        "Plane": "ArduPlane",
        "Tracker": "AntennaTracker",
        "Blimp": "Blimp",
    }

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.doccos = []
        self.emitters = [
            emit_html.HTMLEmitter(),
            emit_rst.RSTEmitter(),
            emit_xml.XMLEmitter(),
            emit_md.MDEmitter(),
        ]

    class Docco(object):

        def __init__(self, name):
            self.name = name
            self.url = None
            self.description = None
            self.fields = {}
            self.fields_order = []
            self.vehicles = None
            self.bits_enums = []

        def set_description(self, desc):
            self.description = desc

        def set_url(self, url):
            self.url = url

        def ensure_field(self, field):
            if field not in self.fields:
                self.fields[field] = {}
                self.fields_order.append(field)

        def set_field_description(self, field, description):
            if field in self.fields:
                raise ValueError("Already have field %s in %s" %
                                 (field, self.name))
            self.ensure_field(field)
            self.fields[field]["description"] = description

        def set_field_bits(self, field, bits):
            bits = bits.split(",")
            count = 0
            entries = []
            for bit in bits:
                entries.append(EnumDocco.EnumEntry(bit, 1<<count, None))
                count += 1
            bitmask_name = self.name + field
            self.bits_enums.append(EnumDocco.Enumeration(bitmask_name, entries))
            self.ensure_field(field)
            self.fields[field]["bitmaskenum"] = bitmask_name

        def set_fieldbitmaskenum(self, field, bits):
            self.ensure_field(field)
            self.fields[field]["bitmaskenum"] = bits

        def set_fieldbitmaskvalue(self, field, bits):
            self.ensure_field(field)
            self.fields[field]["valueenum"] = bits

        def set_vehicles(self, vehicles):
            self.vehicles = vehicles

    def search_for_files(self, dirs_to_search):
        _next = []
        for _dir in dirs_to_search:
            _dir = os.path.join(topdir, _dir)
            for entry in os.listdir(_dir):
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

    def parse_file(self, filepath):
        with open(filepath) as f:
            #            print("Opened (%s)" % filepath)
            lines = f.readlines()
            f.close()
            state_outside = "outside"
            state_inside = "inside"
            state = state_outside
            docco = None
        for line in lines:
            if state == state_outside:
                m = re_loggermessage.search(line)
                if m is None:
                    continue
                name = m.group(1)
                if "," in name:
                    name = name.split(",")
                state = state_inside
                docco = LoggerDocco.Docco(name)
            elif state == state_inside:
                if not re_commentline.match(line):
                    state = state_outside
                    if docco.vehicles is None or self.vehicle in docco.vehicles:
                        self.finalise_docco(docco)
                    continue
                m = re_description.match(line)
                if m is not None:
                    docco.set_description(m.group(1))
                    continue
                m = re_url.match(line)
                if m is not None:
                    docco.set_url(m.group(1))
                    continue
                m = re_field.match(line)
                if m is not None:
                    docco.set_field_description(m.group(1), m.group(2))
                    continue
                m = re_fieldbits.match(line)
                if m is not None:
                    docco.set_field_bits(m.group(1), m.group(2))
                    continue
                m = re_fieldbitmaskenum.match(line)
                if m is not None:
                    docco.set_fieldbitmaskenum(m.group(1), m.group(2))
                    continue
                m = re_fieldvalueenum.match(line)
                if m is not None:
                    docco.set_fieldbitmaskvalue(m.group(1), m.group(2))
                    continue
                m = re_vehicles.match(line)
                if m is not None:
                    docco.set_vehicles([x.strip() for x in m.group(1).split(',')])
                    continue
                print("Unknown field (%s)" % str(line))
                sys.exit(1)

    def parse_files(self):
        for _file in self.files:
            self.parse_file(_file)

    def emit_output(self):
        # expand things like PIDR,PIDQ,PIDA into multiple doccos
        new_doccos = []
        for docco in self.doccos:
            if isinstance(docco.name, list):
                for name in docco.name:
                    tmpdocco = copy.copy(docco)
                    tmpdocco.name = name
                    new_doccos.append(tmpdocco)
            else:
                new_doccos.append(docco)
        new_doccos = sorted(new_doccos, key=lambda x : x.name)

        enums_by_name = {}
        for enum in self.enumerations:
            enums_by_name[enum.name] = enum
        for emitter in self.emitters:
            emitter.emit(new_doccos, enums_by_name)

    def run(self):
        self.enumerations = enum_parse.EnumDocco(self.vehicle).get_enumerations()
        self.files = []
        self.search_for_files([self.vehicle_map[self.vehicle], "libraries"])
        self.parse_files()
        self.emit_output()

    def finalise_docco(self, docco):
        self.doccos.append(docco)
        self.enumerations += docco.bits_enums


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Parse parameters.")
    parser.add_argument("-v", "--verbose", dest='verbose', action='store_true', default=False, help="show debugging output")
    parser.add_argument("--vehicle", required=True, help="Vehicle type to generate for")

    args = parser.parse_args()

    s = LoggerDocco(args.vehicle)

    if args.vehicle not in s.vehicle_map:
        print("Invalid vehicle (choose from: %s)" % str(s.vehicle_map.keys()))
        sys.exit(1)

    s.run()
