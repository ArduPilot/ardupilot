#!/usr/bin/env python3

'''
generate the MAVLink events interface metadata (COMP_METADATA_TYPE_EVENTS)
from the annotated LogEvent, LogErrorSubsystem and LogErrorCode
enumerations in libraries/AP_Logger/AP_Logger.h

The output is a JSON file in the format GCSs expect for MAVLink
events (as defined by the libevents project), optionally xz-compressed.

Annotations are comments immediately above an enumerator:

  LogEvent:
    // @Message: one line shown to the user (required)
    // @Description: longer text (optional)

  LogErrorSubsystem: each subsystem is a single event with one uint8_t
  argument, the error code
    // @Message: text, "{1}" is replaced with the code (required)
    // @Description: longer text (optional)
    // @Codes: LogErrorCode names this subsystem uses, comma separated
  or, where the code is data rather than a LogErrorCode:
    // @Argument: name of the argument

  LogErrorCode: entries named in a @Codes annotation
    // @Message: text used in place of "{1}" (required)

AP_FLAKE8_CLEAN
'''

import argparse
import json
import lzma
import os
import re
import sys

# the libevents component ID the ArduPilot events are defined in;
# component 0 is reserved for the libevents common definitions.  Must
# match the top byte of the ids AP_Logger sends
COMPONENT_ID = 1
NAMESPACE = "ardupilot"

# lower 24 bits of the event ID; LogEvent values are used directly
ERROR_SUB_ID_BASE = 0x10000

# romfs name of the compressed metadata
ROMFS_NAME = "events.json.xz"

LOGGER_HEADER = os.path.join("libraries", "AP_Logger", "AP_Logger.h")

MAX_MESSAGE_LEN = 120


class EventsMetadataError(Exception):
    pass


class Enumerator(object):
    def __init__(self, name, value, annotations, linenum):
        self.name = name
        self.value = value
        self.annotations = annotations
        self.linenum = linenum


def parse_enum(text, enum_name):
    '''return a list of Enumerator in enum_name in header text'''
    m = re.search(r'enum class %s\s*:\s*\w+\s*{(.*?)^};' % enum_name, text, re.MULTILINE | re.DOTALL)
    if m is None:
        raise EventsMetadataError("enum %s not found" % enum_name)
    first_line = text.count('\n', 0, m.start(1)) + 1

    ret = []
    annotations = {}
    for offset, line in enumerate(m.group(1).split('\n')):
        a = re.match(r'\s*//\s*@(\w+):\s*(.*?)\s*$', line)
        if a is not None:
            key = a.group(1)
            if key in annotations:
                raise EventsMetadataError("%s:%u: duplicate @%s" % (LOGGER_HEADER, first_line+offset, key))
            annotations[key] = a.group(2)
            continue
        e = re.match(r'\s*([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*,?', line)
        if e is not None:
            ret.append(Enumerator(e.group(1), int(e.group(2)), annotations, first_line+offset))
            annotations = {}
    if annotations:
        raise EventsMetadataError("%s: %s ends with unattached annotations" % (LOGGER_HEADER, enum_name))
    return ret


def check_message(where, message):
    if len(message) == 0 or len(message) > MAX_MESSAGE_LEN:
        raise EventsMetadataError("%s: @Message must be 1 to %u characters" % (where, MAX_MESSAGE_LEN))


def where(e):
    return "%s:%u: %s" % (LOGGER_HEADER, e.linenum, e.name)


def event_entry(e, name, arguments=None):
    message = e.annotations.get("Message")
    if message is None:
        raise EventsMetadataError("%s: missing @Message" % where(e))
    check_message(where(e), message)
    entry = {
        "name": name,
        "message": message,
    }
    if "Description" in e.annotations:
        entry["description"] = e.annotations["Description"]
    if arguments is not None:
        entry["arguments"] = arguments
    return entry


def generate(header_text):
    '''return the events metadata as a dictionary'''
    log_events = parse_enum(header_text, "LogEvent")
    subsystems = parse_enum(header_text, "LogErrorSubsystem")
    codes = {e.name: e for e in parse_enum(header_text, "LogErrorCode")}

    events = {}
    enums = {}
    for e in log_events:
        if e.value >= ERROR_SUB_ID_BASE:
            raise EventsMetadataError("%s: value too large" % where(e))
        unknown = set(e.annotations.keys()) - set(["Message", "Description"])
        if unknown:
            raise EventsMetadataError("%s: unknown annotations %s" % (where(e), sorted(unknown)))
        events[str(e.value)] = event_entry(e, e.name.lower())

    for s in subsystems:
        unknown = set(s.annotations.keys()) - set(["Message", "Description", "Codes", "Argument"])
        if unknown:
            raise EventsMetadataError("%s: unknown annotations %s" % (where(s), sorted(unknown)))
        if ("Codes" in s.annotations) == ("Argument" in s.annotations):
            raise EventsMetadataError("%s: need exactly one of @Codes and @Argument" % where(s))
        if "Argument" in s.annotations:
            argument = {"type": "uint8_t", "name": s.annotations["Argument"]}
        else:
            enum_name = "%s_error_t" % s.name.lower()
            entries = {}
            for code_name in [x.strip() for x in s.annotations["Codes"].split(",")]:
                code = codes.get(code_name)
                if code is None:
                    raise EventsMetadataError("%s: unknown LogErrorCode %s" % (where(s), code_name))
                if str(code.value) in entries:
                    raise EventsMetadataError("%s: %s duplicates the value of %s" %
                                              (where(s), code_name, entries[str(code.value)]["name"]))
                message = code.annotations.get("Message")
                if message is None:
                    raise EventsMetadataError("%s: missing @Message" % where(code))
                entries[str(code.value)] = {
                    "name": code.name.lower(),
                    "description": message,
                }
            enums[enum_name] = {
                "type": "uint8_t",
                "description": "%s error codes" % s.name,
                "entries": entries,
            }
            argument = {"type": enum_name, "name": "code"}
        events[str(ERROR_SUB_ID_BASE + s.value)] = event_entry(s, "error_" + s.name.lower(), arguments=[argument])

    return {
        "version": 2,
        "components": {
            str(COMPONENT_ID): {
                "namespace": NAMESPACE,
                "enums": enums,
                "event_groups": {
                    "default": {
                        "events": events,
                    },
                },
            },
        },
    }


def generate_json(srcdir):
    '''return the events metadata as bytes of JSON'''
    with open(os.path.join(srcdir, LOGGER_HEADER), encoding="utf-8") as f:
        header_text = f.read()
    metadata = generate(header_text)
    return json.dumps(metadata, sort_keys=True, separators=(',', ':')).encode("utf-8")


def compress(contents):
    '''xz-compress contents.  CRC32 integrity checks are used as they
    are supported by every xz decoder'''
    return lzma.compress(contents, format=lzma.FORMAT_XZ, check=lzma.CHECK_CRC32,
                         preset=9 | lzma.PRESET_EXTREME)


def write_if_changed(path, contents):
    '''write contents to path, leaving it untouched if unchanged so its
    timestamp doesn't change'''
    try:
        with open(path, 'rb') as f:
            if f.read() == contents:
                return
    except FileNotFoundError:
        pass
    with open(path, 'wb') as f:
        f.write(contents)


def write_romfs_file(srcdir, path):
    '''write the compressed events metadata to path'''
    write_if_changed(path, compress(generate_json(srcdir)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument("--srcdir", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", ".."),
                        help="ArduPilot source directory")
    parser.add_argument("--xz", action='store_true', help="xz-compress the output")
    parser.add_argument("output", help="output file")
    args = parser.parse_args()

    try:
        contents = generate_json(args.srcdir)
    except EventsMetadataError as ex:
        print("events_metadata: %s" % ex, file=sys.stderr)
        sys.exit(1)
    if args.xz:
        contents = compress(contents)
    write_if_changed(args.output, contents)
