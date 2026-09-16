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

    # C++ integer literal suffix: u, l or ll, in either order and either case
    INTEGER_SUFFIX = r"(?:[uU](?:ll|LL|[lL])?|(?:ll|LL|[lL])[uU]?)?"

    # decimal integer; a leading zero would make it octal, which is not handled
    DECIMAL = r"(?:0|[1-9][0-9]*)"

    # optional trailing comment, captured without a doxygen marker such as
    # ///< or //!< (only when followed by whitespace, so "//!important" is kept)
    COMMENT = r"(?:\s*//(?:(?:[/!]<?|<)(?=\s|$))?\s*(.*))?$"

    def match_enum_line(self, line):
        # attempts to extract name, value and comment from line.
        suffix = self.INTEGER_SUFFIX
        decimal = self.DECIMAL
        comment = self.COMMENT

        # Match:  "            FRED,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+)\s*,?" + comment, line)
        if m is not None:
            return (m.group(1), None, m.group(2))

        # Match:  "            FRED  = 17,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+)\s*=\s*(-?" + decimal + r")(" + suffix + r")\s*,?" + comment,
                     line)
        if m is not None:
            if m.group(2).startswith("-") and "u" in m.group(3).lower():
                # e.g. -1U is UINT_MAX, not -1
                raise ValueError("Negative unsigned value (%s)" % line)
            return (m.group(1), m.group(2), m.group(4))

        # Match:  "            FRED  = 1U<<0,  // optional comment"
        # Match:  "            FRED  = (3U << 6U),  // optional comment"
        # the whole line must match so that e.g. "(1U<<4) | 8" is not taken as 1<<4
        shift = "(" + decimal + ")(" + suffix + r") *<< *(" + decimal + ")" + suffix
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *(?:" + shift + r"|\( *" + shift + r" *\))\s*,?" + comment,
                     line)
        if m is not None:
            if m.group(2) is not None:
                (base, base_suffix, count) = (m.group(2), m.group(3), m.group(4))
            else:
                (base, base_suffix, count) = (m.group(5), m.group(6), m.group(7))
            value = int(base) << int(count)
            width = 64 if "ll" in base_suffix.lower() else 32
            if value >= 1 << width:
                # the result would wrap, or depends on the width of the base's type
                raise ValueError("Shift may overflow its type (%s)" % line)
            return (m.group(1), value, m.group(8))

        # Match:  "            FRED  = 0xabc,  // optional comment"
        # Match:  "            FRED  = 0xabcULL,  // optional comment"
        # the whole line must match so that e.g. "0x18 + 1" is not taken as 0x18
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *(?:0[xX]([0-9A-Fa-f]+))" + suffix + r"\s*,?" + comment,
                     line)
        if m is not None:
            return (m.group(1), int(m.group(2), 16), m.group(3))

        '''start discarded matches - lines we understand but can't do anything
        with.  These only match identifiers, so a numeric value which none of
        the patterns above understood raises rather than being discarded:'''
        # Match:  "            FRED  = BARNEY,  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *([A-Za-z_]\w*) *,?(?: *// *(.*) *)?$",
                     line)
        if m is not None:
            return (None, None, None)
        # Match:  "            FRED  = FOO(17, BAR),  // optional comment"
        m = re.match(r"\s*([A-Z0-9_a-z]+) *= *([A-Za-z_]\w*) *\([^()]*\) *,?(?: *// *(.*) *)?$",
                     line)
        if m is not None:
            return (None, None, None)

        # Match:  "#define FRED 1  // optional comment"
        m = re.match(r"#define\s*([A-Z0-9_a-z]+)\s+(-?\d+) *(// *(.*) *)?$", line)
        if m is not None:
            return (m.group(1), m.group(2), m.group(4))

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
            lineno = 0
            while True:
                line = f.readline()
                lineno += 1
                #  debug(f"{state} line: {line}")
                if line == "":
                    break
                line = line.rstrip()
                #        print("state=%s line: %s" % (state, line))
                # Skip single-line comments - unless they contain LoggerEnum tags
                if re.match(r"\s*//.*", line) and "LoggerEnum" not in line:
                    continue
                # Skip block comments starting the line, keeping any code
                # which follows them on the line where they end
                if re.match(r"\s*/\*", line):
                    comment_lineno = lineno
                    end = line.find("*/", line.index("/*") + 2)
                    while end == -1:
                        line = f.readline()
                        lineno += 1
                        if line == "":
                            raise ValueError("%s:%u: unterminated /* comment" % (source_file, comment_lineno))
                        line = line.rstrip()
                        end = line.find("*/")
                    line = line[end+2:]
                    if re.match(r"\s*(//.*)?$", line):
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
                        # all on one line
                        enum_name = m.group(2)
                        debug("ol: %s: %s" % (source_file, enum_name))
                        entries = []
                        last_value = None
                        skip_enumeration = False
                        for item in self.split_entries(m.group(3)):
                            if item.strip() == "":
                                continue
                            (name, value, comment) = self.match_enum_entry(item, source_file, lineno)
                            if name is None:
                                skip_enumeration = True
                                break
                            last_value = self.entry_value(value, last_value)
                            entries.append(EnumDocco.EnumEntry(name, last_value, comment))
                        if not skip_enumeration:
                            if in_class is not None:
                                enum_name = "::".join([in_class, enum_name])
                            enumerations.append(EnumDocco.Enumeration(enum_name, entries))
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
                    # ignore any trailing comment, so that a comment
                    # containing "};" does not end the enumeration early
                    code = re.sub(r"//.*", "", line)
                    if re.match(r".*}\s*\w*(\s*=\s*[\w:]+)?;", code) or "@LoggerEnumEnd" in line:
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
                    (name, value, comment) = self.match_enum_entry(line, source_file, lineno)
                    if name is None:
                        skip_enumeration = True
                        continue
                    debug(" name=(%s) value=(%s) comment=(%s)\n" % (name, value, comment))
                    last_value = self.entry_value(value, last_value)
                    entries.append(EnumDocco.EnumEntry(name, last_value, comment))
        return enumerations

    def match_enum_entry(self, line, source_file, lineno):
        '''match_enum_line, adding the location to any error'''
        try:
            return self.match_enum_line(line)
        except ValueError as ex:
            hint = ""
            if "/*" in re.sub(r"//.*", "", line):
                hint = "; use // rather than /* */ for comments on enumeration entries"
            raise ValueError("%s:%u: %s%s" % (source_file, lineno, ex, hint)) from None

    @staticmethod
    def split_entries(body):
        '''split the body of a single-line enumeration on the commas which are not inside brackets'''
        items = []
        depth = 0
        start = 0
        for (i, c) in enumerate(body):
            if c in "([{":
                depth += 1
            elif c in ")]}":
                depth -= 1
            elif c == "," and depth == 0:
                items.append(body[start:i])
                start = i + 1
        items.append(body[start:])
        return items

    @staticmethod
    def entry_value(value, last_value):
        '''value of an entry, given its explicit value (or None) and the previous entry's value'''
        if value is not None:
            return int(value)
        if last_value is None:
            return 0
        return last_value + 1

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
                if filepath.endswith("libraries/AP_GPS/AP_GPS_UBLOX_CFG_Keys.h"):
                    continue
                if filepath.endswith("libraries/AP_GPS/AP_GPS_UBLOX_CFGv2.cpp"):
                    continue
                if filepath.endswith("libraries/AP_GPS/AP_GPS_UBLOX_CFGv2.h"):
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
