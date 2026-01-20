#!/usr/bin/env python3

'''
AP_FLAKE8_CLEAN
'''

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

# Regular expressions for finding message information in code comments
re_loggermessage = re.compile(r"@LoggerMessage\s*:\s*([\w,]+)", re.MULTILINE)
re_commentline = re.compile(r"\s*//")
re_description = re.compile(r"\s*//\s*@Description\s*:\s*(.*)")
re_url = re.compile(r"\s*//\s*@URL\s*:\s*(.*)")
re_field = re.compile(r"\s*//\s*@Field\s*:\s*(\w+):\s*(.*)")
re_fieldbits = re.compile(r"\s*//\s*@FieldBits\s*:\s*(\w+):\s*(.*)")
re_fieldbitmaskenum = re.compile(r"\s*//\s*@FieldBitmaskEnum\s*:\s*(\w+):\s*(.*)")
re_fieldvalueenum = re.compile(r"\s*//\s*@FieldValueEnum\s*:\s*(\w+):\s*(.*)")
re_vehicles = re.compile(r"\s*//\s*@Vehicles\s*:\s*(.*)")

# Regular expressions for finding message definitions in structure format
re_start_messagedef = re.compile(r"^\s*{?\s*LOG_[A-Z0-9_]+_[MSGTA]+[A-Z0-9_]*\s*,")
re_deffield = r'[\s\\]*"?([\w\-#?%]+)"?\s*'
re_full_messagedef = re.compile(r'\s*LOG_\w+\s*,\s*\w+\([^)]+\)[\s\\]*,' +
                                f'{re_deffield},{re_deffield},' +
                                r'[\s\\]*"?([\w,]+)"?[\s\\]*,' +
                                f'{re_deffield},{re_deffield}',
                                re.MULTILINE)
re_names_define = re.compile(r'#define\s+(\w+_LABELS)\s+"([\w,]+)"')
re_fmt_define = re.compile(r'#define\s+(\w+_FMT)\s+"([\w\-#?%]+)"')
re_units_define = re.compile(r'#define\s+(\w+_UNITS)\s+"([\w\-#?%]+)"')
re_mults_define = re.compile(r'#define\s+(\w+_MULTS)\s+"([\w\-#?%]+)"')

# Regular expressions for finding message definitions in Write calls
re_start_writecall = re.compile(r"\s*[AP:]*logger[\(\)]*.Write[StreamingCrcl]*\(")
re_writefield = r'\s*"([\w\-#?%,]+)"\s*'
re_full_writecall = re.compile(r'\s*[AP:]*logger[\(\)]*.Write[StreamingCrcl]*\(' +
                               f'{re_writefield},{re_writefield},{re_writefield},({re_writefield},{re_writefield})?',
                               re.MULTILINE)

# Regular expression for extracting unit and multipliers from structure
re_units_mults_struct = re.compile(r"^\s*{\s*'([\w\-#?%!/])',"+r'\s*"?([\w\-#?%./]*)"?\s*}')

# TODO: validate URLS actually return 200

# Lookup tables are populated by reading LogStructure.h
log_fmt_lookup = {}
log_units_lookup = {}
log_mult_lookup = {}

# Lookup table to convert multiplier to prefix
mult_prefix_lookup = {
    0: "",
    1: "",
    1e-1: "d", # deci-
    1e-2: "c", # centi-
    1e-3: "m", # milli-
    1e-6: "μ", # micro-
    1e-9: "n"  # nano-
}


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
        self.msg_fmts_list = {}
        self.msg_names_list = {}
        self.msg_units_list = {}
        self.msg_mults_list = {}

    class Docco(object):

        def __init__(self, name):
            self.name = name
            self.url = None
            if isinstance(name, list):
                self.description = [None] * len(name)
            else:
                self.description = None
            self.fields = {}
            self.fields_order = []
            self.vehicles = None
            self.bits_enums = []

        def add_name(self, name):
            # If self.name/description aren't lists, convert them
            if isinstance(self.name, str):
                self.name = [self.name]
                self.description = [self.description]
            # Replace any existing empty descriptions with empty strings
            for i in range(0, len(self.description)):
                if self.description[i] is None:
                    self.description[i] = ""
            # Extend the name and description lists
            if isinstance(name, list):
                self.name.extend(name)
                self.description.extend([None] * len(name))
            else:
                self.name.append(name)
                self.description.append(None)

        def set_description(self, desc):
            if isinstance(self.description, list):
                for i in range(0, len(self.description)):
                    if self.description[i] is None:
                        self.description[i] = desc
            else:
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

        def get_field_description(self, field):
            if field not in self.fields:
                return None
            return self.fields[field].get('description', None)

        def set_field_bits(self, field, bits):
            bits = bits.split(",")
            count = 0
            entries = []
            for bit in bits:
                entries.append(EnumDocco.EnumEntry(bit, 1 << count, None))
                count += 1
            bitmask_name = self.name + field
            self.bits_enums.append(EnumDocco.Enumeration(bitmask_name, entries))
            self.ensure_field(field)
            self.fields[field]["bitmaskenum"] = bitmask_name

        def set_fieldbitmaskenum(self, field, bits):
            self.ensure_field(field)
            self.fields[field]["bitmaskenum"] = bits

        def set_fieldvalueenum(self, field, bits):
            self.ensure_field(field)
            self.fields[field]["valueenum"] = bits

        def set_vehicles(self, vehicles):
            self.vehicles = vehicles

        def set_field_names(self, fields):
            ''' Check that the field ordering matches the defined fields '''
            fields = fields.split(",")
            # First check that the number of fields match
            if len(fields) != len(self.fields_order):
                print(f"Error: Mismatch in number of fields in message {self.name}: ", file=sys.stderr, end='')
                print(f"{len(self.fields_order)} vs {len(fields)}", file=sys.stderr)
                sys.exit(1)
            # Now check that each field name matches
            err = False
            for idx in range(0, len(fields)):
                if fields[idx] != self.fields_order[idx]:
                    print(f"Error: Field order mismatch in log message {self.name}: ", file=sys.stderr, end='')
                    print(f"field={idx+1}: {fields[idx]} vs {self.fields_order[idx]}", file=sys.stderr)
                    err = True
            # Exit if we had any name mismatch errors
            if err:
                sys.exit(1)

        def set_fmts(self, fmts):
            # If no fields are defined, do nothing
            if len(self.fields_order) == 0:
                return
            # Make sure lengths match up
            if len(fmts) != len(self.fields_order):
                print(f"Number of fmts don't match fields: msg={self.name} fmts={fmts} num_fields={len(self.fields_order)} {self.fields_order}")  # noqa:E501
                return
            # Loop through the list
            for idx in range(0, len(fmts)):
                if fmts[idx] in log_fmt_lookup:
                    self.fields[self.fields_order[idx]]["fmt"] = log_fmt_lookup[fmts[idx]]
                else:
                    print(f"Unrecognised format character: {fmts[idx]} in message {self.name}")

        def set_units(self, units, mults):
            # If no fields are defined, do nothing
            if len(self.fields_order) == 0:
                return
            # Make sure lengths match up
            if len(units) != len(self.fields_order) or len(units) != len(mults):
                print(f"Number of units/mults/fields don't match: msg={self.name} units={units} mults={mults} num_fields={len(self.fields_order)}")  # noqa:E501
                return
            # Loop through the list
            for idx in range(0, len(units)):
                # Get the index into fields from field_order
                f = self.fields_order[idx]
                # Convert unit char to base unit
                if units[idx] in log_units_lookup:
                    baseunit = log_units_lookup[units[idx]]
                else:
                    print(f"Unrecognised units character: {units[idx]} in message {self.name}")
                    continue
                # Do nothing if this field has no unit defined
                if baseunit == "":
                    continue
                # Convert mult char to value
                if mults[idx] in log_mult_lookup:
                    mult = log_mult_lookup[mults[idx]]
                    mult_num = float(mult)
                else:
                    print(f"Unrecognised multiplier character: {mults[idx]} in message {self.name}")
                    continue
                # Check if the defined format for this field contains its own multiplier
                # If so, the presented value will be the base-unit directly
                if 'fmt' in self.fields[f] and self.fields[f]['fmt'].endswith("* 100"):
                    self.fields[f]["units"] = baseunit
                elif 'fmt' in self.fields[f] and "latitude/longitude" in self.fields[f]['fmt']:
                    self.fields[f]["units"] = baseunit
                # Check if we have a defined prefix for this multiplier
                elif mult_num in mult_prefix_lookup:
                    self.fields[f]["units"] = f"{mult_prefix_lookup[mult_num]}{baseunit}"
                # If all else fails, set the unit as the multiplier and base unit together
                else:
                    self.fields[f]["units"] = f"{mult} {baseunit}"

    def populate_lookups(self):
        # Initialise the lookup tables
        # Read the contents of the LogStructure.h file
        structfile = os.path.join(topdir, "libraries", "AP_Logger", "LogStructure.h")
        with open(structfile) as f:
            lines = f.readlines()
            f.close()
        # Initialise current section to none
        section = "none"
        # Loop through the lines in the file
        for line in lines:
            # Look for the start of fmt/unit/mult info
            if line.startswith("Format characters"):
                section = "fmt"
            elif line.startswith("const struct UnitStructure"):
                section = "units"
            elif line.startswith("const struct MultiplierStructure"):
                section = "mult"
            # Read formats from code comment, e.g.:
            #    b   : int8_t
            elif section == "fmt":
                if "*/" in line:
                    section = "none"
                else:
                    parts = line.split(":")
                    log_fmt_lookup[parts[0].strip()] = parts[1].strip()
            # Read units or multipliers from C struct definition, e.g.:
            #    { '2', 1e2 },  or   { 'J', "W.s" },
            elif section != "none":
                if "};" in line:
                    section = "none"
                else:
                    u = re_units_mults_struct.search(line)
                    if u is not None and section == "units":
                        log_units_lookup[u.group(1)] = u.group(2)
                    if u is not None and section == "mult":
                        log_mult_lookup[u.group(1)] = u.group(2)

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

    def parse_messagedef(self, messagedef):
        # Merge concatenated strings and remove comments
        messagedef = re.sub(r'"\s+"', '', messagedef)
        messagedef = re.sub(r'//[^\n]*', '', messagedef)
        # Extract details from a structure definition
        d = re_full_messagedef.search(messagedef)
        if d is not None:
            self.msg_fmts_list[d.group(1)] = d.group(2)
            self.msg_names_list[d.group(1)] = d.group(3)
            self.msg_units_list[d.group(1)] = d.group(4)
            self.msg_mults_list[d.group(1)] = d.group(5)
            return
        # Extract details from a WriteStreaming call
        d = re_full_writecall.search(messagedef)
        if d is not None:
            if d.group(1) in self.msg_fmts_list:
                return
            if d.group(5) is None:
                self.msg_names_list[d.group(1)] = d.group(2)
                self.msg_fmts_list[d.group(1)] = d.group(3)
            else:
                self.msg_names_list[d.group(1)] = d.group(2)
                self.msg_fmts_list[d.group(1)] = d.group(6)
                self.msg_units_list[d.group(1)] = d.group(3)
                self.msg_mults_list[d.group(1)] = d.group(5)
            return
        # Didn't parse
        # print(f"Unable to parse: {messagedef}")

    def search_messagedef_start(self, line, prevmessagedef=""):
        # Look for the start of a structure definition
        d = re_start_messagedef.search(line)
        if d is not None:
            messagedef = line
            if "}" in line:
                self.parse_messagedef(messagedef)
                return ""
            else:
                return messagedef
        # Look for a new call to WriteStreaming
        d = re_start_writecall.search(line)
        if d is not None:
            messagedef = line
            if ";" in line:
                self.parse_messagedef(messagedef)
                return ""
            else:
                return messagedef
        # If we didn't find a new one, continue with any previous state
        return prevmessagedef

    def parse_file(self, filepath):
        with open(filepath) as f:
            # print("Opened (%s)" % filepath)
            lines = f.readlines()
            f.close()

        def debug(x):
            pass
        #        if filepath == "/home/pbarker/rc/ardupilot/libraries/AP_HAL/AnalogIn.h":
        #            debug = print
        state_outside = "outside"
        state_inside = "inside"
        messagedef = ""
        state = state_outside
        docco = None
        for line in lines:
            debug(f"{state}: {line}")
            if messagedef:
                messagedef = messagedef + line
                if "}" in line or ";" in line:
                    self.parse_messagedef(messagedef)
                    messagedef = ""
            if state == state_outside:
                # Check for start of a message definition
                messagedef = self.search_messagedef_start(line, messagedef)

                # Check for fmt/unit/mult #define
                u = re_names_define.search(line)
                if u is not None:
                    self.msg_names_list[u.group(1)] = u.group(2)
                u = re_fmt_define.search(line)
                if u is not None:
                    self.msg_fmts_list[u.group(1)] = u.group(2)
                u = re_units_define.search(line)
                if u is not None:
                    self.msg_units_list[u.group(1)] = u.group(2)
                u = re_mults_define.search(line)
                if u is not None:
                    self.msg_mults_list[u.group(1)] = u.group(2)

                # Check for the @LoggerMessage tag indicating the start of the docco block
                m = re_loggermessage.search(line)
                if m is None:
                    continue
                name = m.group(1)
                if "," in name:
                    name = name.split(",")
                state = state_inside
                docco = LoggerDocco.Docco(name)
            elif state == state_inside:
                # If this line is not a comment, then this is the end of the docco block
                if not re_commentline.match(line):
                    state = state_outside
                    if docco.vehicles is None or self.vehicle in docco.vehicles:
                        self.finalise_docco(docco)
                    messagedef = self.search_messagedef_start(line)
                    continue
                # Check for an multiple @LoggerMessage lines in this docco block
                m = re_loggermessage.search(line)
                if m is not None:
                    name = m.group(1)
                    if "," in name:
                        name = name.split(",")
                    docco.add_name(name)
                    continue
                # Find and extract data from the various docco fields
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
                    docco.set_fieldvalueenum(m.group(1), m.group(2))
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
                for name, desc in zip(docco.name, docco.description):
                    tmpdocco = copy.copy(docco)
                    tmpdocco.name = name
                    tmpdocco.description = desc
                    new_doccos.append(tmpdocco)
            else:
                new_doccos.append(docco)
        new_doccos = sorted(new_doccos, key=lambda x : x.name)

        # Try to attach the formats/units/multipliers
        for docco in new_doccos:
            # Check that the field names are correctly ordered
            if docco.name in self.msg_names_list:
                if "LABELS" in self.msg_names_list[docco.name]:
                    if self.msg_names_list[docco.name] in self.msg_names_list:
                        docco.set_field_names(self.msg_names_list[self.msg_names_list[docco.name]])
                else:
                    docco.set_field_names(self.msg_names_list[docco.name])
            else:
                print(f"No field names found for message {docco.name}")
            # Apply the Formats to the docco
            if docco.name in self.msg_fmts_list:
                if "FMT" in self.msg_fmts_list[docco.name]:
                    if self.msg_fmts_list[docco.name] in self.msg_fmts_list:
                        docco.set_fmts(self.msg_fmts_list[self.msg_fmts_list[docco.name]])
                else:
                    docco.set_fmts(self.msg_fmts_list[docco.name])
            else:
                print(f"No formats found for message {docco.name}")
            # Get the Units
            units = None
            if docco.name in self.msg_units_list:
                if "UNITS" in self.msg_units_list[docco.name]:
                    if self.msg_units_list[docco.name] in self.msg_units_list:
                        units = self.msg_units_list[self.msg_units_list[docco.name]]
                else:
                    units = self.msg_units_list[docco.name]
            # Get the Multipliers
            mults = None
            if docco.name in self.msg_mults_list:
                if "MULTS" in self.msg_mults_list[docco.name]:
                    if self.msg_mults_list[docco.name] in self.msg_mults_list:
                        mults = self.msg_mults_list[self.msg_mults_list[docco.name]]
                else:
                    mults = self.msg_mults_list[docco.name]
            # Apply the units/mults to the docco
            if units is not None and mults is not None:
                docco.set_units(units, mults)
            elif units is not None or mults is not None:
                print(f"Cannot find matching units/mults for message {docco.name}")

        # every field must have a description.  Things like
        # FieldBitmaskEnum can create the field object but not fill
        # description in.
        for docco in new_doccos:
            for field in docco.fields:
                if docco.get_field_description(field) is None:
                    raise ValueError(f"{docco.name}.{field} missing description")

        enums_by_name = {}
        for enum in self.enumerations:
            enums_by_name[enum.name] = enum
        for emitter in self.emitters:
            emitter.emit(new_doccos, enums_by_name)

    def run(self):
        self.populate_lookups()
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
