#!/usr/bin/env python3

'''
AP_FLAKE8_CLEAN
'''

import argparse
import os
import re
import sys

topdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')
topdir = os.path.realpath(topdir)


class EnumDocco(object):

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
        self.enumerations = []

    class EnumEntry(object):
        def __init__(self, name, value, comment):
            self.name = name
            self.value = value
            self.comment = comment

    def match_enum_line(self, line):
        # attempts to extract name, value and comment from line.

        # Match:  "            FRED,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+)\s*,? *(?://[/>]* *(.*) *)?$", line)
        if m is not None:
            return (m.group(1), None, m.group(2))

        # Match:  "            FRED,  /* optional comment */"
        m = re.match(r"\s*([A-Z0-9_a-z]+)\s*,? *(?:/[*] *(.*) *[*]/ *)?$", line)
        if m is not None:
            return (m.group(1), None, m.group(2))

        # Match:  "            FRED  = 17,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+)\s*=\s*([-0-9]+)\s*,?(?:\s*//[/<]*\s*(.*) *)?$",
                     line)
        if m is not None:
            return (m.group(1), m.group(2), m.group(3))

        # Match:  "            FRED  = 17,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *([-0-9]+) *,?(?: */* *(.*) *)? *[*]/ *$",
                     line)
        if m is not None:
            return (m.group(1), m.group(2), m.group(3))

        # Match:  "            FRED  = 1U<<0,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *[(]?1U? *[<][<] *(\d+)(?:, *// *(.*) *)?",
                     line)
        if m is not None:
            return (m.group(1), 1 << int(m.group(2)), m.group(3))

        # Match:  "            FRED  = 0xabc,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *(?:0[xX]([0-9A-Fa-f]+))(?:, *// *(.*) *)?",
                     line)
        if m is not None:
            return (m.group(1), int(m.group(2), 16), m.group(3))

        '''start discarded matches - lines we understand but can't do anything
        with:'''
        # Match:  "            FRED  = 17,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *(\w+) *,?(?: *// *(.*) *)?$",
                     line)
        if m is not None:
            return (None, None, None)
        # Match:  "            FRED  = FOO(17),  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *(\w+) *\\( *(\w+) *\\) *,?(?: *// *(.*) *)?$",
                     line)
        if m is not None:
            return (None, None, None)

        # Match:  "            FRED  = 1U<<0,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *[(]?3U? *[<][<] *(\d+)(?:, *// *(.*) *)?",
                     line)
        if m is not None:
            return (m.group(1), 1 << int(m.group(2)), m.group(3))

        # Match:  "#define FRED 1  // optional comment"
        m = re.match(r"#define\s*([A-Z0-9_a-z]+)\s+(-?\d+) *(// *(.*) *)?$", line)
        if m is not None:
            return (m.group(1), m.group(2), m.group(4))

        if m is None:
            raise ValueError("Failed to match (%s)" % line)

    def enumerations_from_file(self, source_file):
        def debug(x):
            pass
        # if source_file == "/home/pbarker/rc/ardupilot/libraries/AP_HAL/AnalogIn.h":
        #     debug = print
        state_outside = "outside"
        state_inside = "inside"

        state = state_outside

        enumerations = []
        with open(source_file) as f:
            enum_name = None
            in_class = None
            while True:
                line = f.readline()
                #  debug(f"{state} line: {line}")
                if line == "":
                    break
                line = line.rstrip()
                #        print("state=%s line: %s" % (state, line))
                # Skip single-line comments - unless they contain LoggerEnum tags
                if re.match(r"\s*//.*", line) and "LoggerEnum" not in line:
                    continue
                # Skip multi-line comments
                if re.match(r"\s*/\*.*", line):
                    while "*/" not in line:
                        line = f.readline()
                    continue
                if state == "outside":
                    if re.match("class .*;", line) is not None:
                        # forward-declaration of a class
                        continue
                    m = re.match(r"class *([:\w]+)", line)
                    if m is not None:
                        in_class = m.group(1)
                        continue
                    m = re.match(r"namespace *(\w+)", line)
                    if m is not None:
                        in_class = m.group(1)
                        continue
                    m = re.match(r".*enum\s*(class)? *([\w]+)\s*(?::.*_t)? *{(.*)};", line)
                    if m is not None:
                        # all one one line!  Thanks!
                        enum_name = m.group(2)
                        debug("ol: %s: %s" % (source_file, enum_name))
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

                    m = re.match(r".*enum\s*(class)? *([\w]+)\s*(?::.*_t)? *{", line)
                    if m is not None:
                        enum_name = m.group(2)
                        debug("%s: %s" % (source_file, enum_name))
                        entries = []
                        last_value = None
                        state = state_inside
                        skip_enumeration = False
                        continue

                    # // @LoggerEnum: NAME  -  can be used around for #define sets
                    m = re.match(r".*@LoggerEnum: *([\w:]+)", line)
                    if m is not None:
                        enum_name = m.group(1)
                        debug("%s: %s" % (source_file, enum_name))
                        entries = []
                        last_value = None
                        state = state_inside
                        skip_enumeration = False
                        continue

                    continue
                if state == "inside":
                    if re.match(r"\s*enum.*$", line):
                        # Allow @LoggerEnum around Enum for name override
                        continue
                    if re.match(r"\s*$", line):
                        continue
                    if re.match(r"#if", line):
                        continue
                    if re.match(r"#endif", line):
                        continue
                    if re.match(r"#else", line):
                        continue
                    if re.match(r".*}\s*\w*(\s*=\s*[\w:]+)?;", line) or "@LoggerEnumEnd" in line:
                        # potential end of enumeration
                        if not skip_enumeration:
                            if enum_name is None:
                                raise Exception("WT??")
                            if in_class is not None:
                                enum_name = "::".join([in_class, enum_name])
                            new_enumeration = EnumDocco.Enumeration(enum_name, entries)
                            enumerations.append(new_enumeration)
                            # print("Got enum (%s)" % enum_name)
                            #                        for entry in new_enumeration.entries:
                            #                            print("   %s: %u (%s)" % (entry.name, entry.value, entry.comment))
                        state = state_outside
                        continue
                    (name, value, comment) = self.match_enum_line(line)
                    if name is None:
                        skip_enumeration = True
                        continue
                    debug(" name=(%s) value=(%s) comment=(%s)\n" % (name, value, comment))
                    if value is None:
                        if last_value is None:
                            value = 0
                            last_value = 0
                        else:
                            last_value += 1
                            value = last_value
                    else:
                        value = int(value)
                        last_value = value
                        # print("entry=%s value=%s comment=%s" % (name, value, comment))
                    entries.append(EnumDocco.EnumEntry(name, value, comment))
        return enumerations

    class Enumeration(object):

        def __init__(self, name, entries):
            # print("creating enum %s" % name)
            self.name = name
            self.entries = entries

        def __str__(self):
            return f"EnumDocco.Enumeration: {self.name} [{len(self.entries)} entries]"

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
                if filepath.endswith("libraries/AP_HAL/utility/getopt_cpp.h"):
                    continue
                # Failed to match (    IOEVENT_PWM = EVENT_MASK(1),)
                if filepath.endswith("libraries/AP_IOMCU/iofirmware/iofirmware.cpp"):
                    continue
                self.files.append(filepath)
        if len(_next):
            self.search_for_files(_next)

    def parse_files(self):
        for _file in self.files:
            self.enumerations.extend(self.enumerations_from_file(_file))

    def get_enumerations(self):
        self.files = []
        self.search_for_files([os.path.join(topdir, x) for x in [
            self.vehicle_map[self.vehicle],
            "libraries"]])
        self.parse_files()
        return self.enumerations

    def run(self):
        self.get_enumerations()


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

    if args.verbose:
        for e in s.enumerations:
            print(e)
