#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
translate a MAVLink telemetry log (.tlog) file between incompatible versions
of the protocol, mapping messages and fields by name between the two.

Copyright Marek S. Łukasiewicz 2025
Released under GNU GPL version 3 or later
'''

from argparse import ArgumentParser
import importlib
import os
import struct
import subprocess
import sys
import tempfile

import pymavlink
from pymavlink.generator import mavgen
from pymavlink.mavutil import mavlink_connection

FROM_TO_REQUIRED = "Specify at least 'from' or 'to' definition, with the unspecified one assumed to be installed pymavlink."
parser = ArgumentParser(description=__doc__, epilog=FROM_TO_REQUIRED)

parser.add_argument("-o", "--output", help="output matching packets to given file, by default will append '_translated' to input file name", default=None)
parser.add_argument("-f", "--from", help="git reference or xml file with definitions for input messages", default=None)
parser.add_argument("-F", "--from-repository", help="local path or git clone URL for definitions, default github.com/mavlink/mavlink", default="https://github.com/mavlink/mavlink.git")
parser.add_argument("-t", "--to", help="output messages, see '--from'", default=None)
parser.add_argument("-T", "--to-repository", help="repository with output definitions, see '--from-repository'", default="https://github.com/mavlink/mavlink.git")
parser.add_argument("--dialect", default="all", help="MAVLink dialect, ignored when specifying definitions in XML file, default all")
parser.add_argument("--mav10", action="store_true", help="parse as MAVLink1")
parser.add_argument("log", metavar="LOG")
parser.add_argument("-q", "--quiet", action="count", help="display less output", default=0)
parser.add_argument("-v", "--verbose", action="count", help="display more output", default=0)

args = parser.parse_args()

# Assign to typed variables to avoid mistakes (type comments for backward compatibility with python < 3.5)
args_output = args.output  # type: str | None
# from is a python keyword
args_from = getattr(args, "from")  # type: str | None
args_from_repository = args.from_repository  # type: str
args_to = args.to  # type: str | None
args_to_repository = args.to_repository  # type: str
args_dialect = args.dialect  # type: str
args_mav10 = args.mav10  # type: bool
args_log = args.log  # type: str
args_verbosity = args.verbose - args.quiet  # type: int

if args_from is None and args_to is None:
    parser.error(FROM_TO_REQUIRED)

if not os.path.isfile(args_log):
    parser.error("LOG argument must be a path to a file")

if (args_from is not None and args_from_repository.endswith(".git")) or \
    (args_to is not None and args_to_repository.endswith(".git")):
    try:
        git_version = subprocess.check_output(['git', '--version'], universal_newlines=True)
        if args_verbosity >= 2:
            print(git_version)
    except:
        print("Unable to run 'git' command required for checking out definitions", file=sys.stderr)

# Don't check out the same repository twice
repository_checkouts = {}

def protocol_module(identifier, repository, dialect):
    # type: (str | None, str, str) -> pymavlink.dialects.v20.all

    if identifier is None:
        return importlib.import_module("pymavlink.dialects.v20." + dialect)
    else:
        dialect_path = identifier
        if not os.path.isfile(dialect_path) or not dialect_path.endswith(".xml"):
            repo_path = repository
            if repository.endswith(".git"):
                if repository in repository_checkouts:
                    repo_path = repository_checkouts[repository]
                else:
                    # We actually need to clone it to a temporary directory
                    repo_path = tempfile.mkdtemp()
                    if args_verbosity >= 1:
                        print("Cloning from", repository, "to", repo_path)
                    subprocess.check_call(['git', 'clone', '--recursive', repository, repo_path])

            assert os.path.isdir(repo_path), "Has a local copy of mavlink repository"
            try:
                subprocess.check_call(['git', 'fetch'], cwd=repo_path)
            except:
                if args_verbosity >= 0:
                    print("Could not fetch updates to mavlink repository")

            subprocess.check_call(['git', 'checkout', identifier], cwd=repo_path)

            dialect_path = os.path.join(repo_path, "message_definitions", "v1.0", dialect + ".xml")

        out_path = tempfile.mkdtemp()
        module_name = "mavlink_" + identifier.replace("-", "_")
        mavgen.mavgen(mavgen.Opts(os.path.join(out_path, module_name), language="Python3", validate=False, wire_protocol="1.0" if args_mav10 else "2.0"), [dialect_path])

        sys.path.insert(1, out_path)
        return importlib.import_module(module_name)

module_from = protocol_module(args_from, args_from_repository, args_dialect)
module_to = protocol_module(args_to, args_to_repository, args_dialect)

output_path = args_output if args_output is not None else \
    (args_log.replace(".tlog", "") + "_translated.tlog")

mav_from = mavlink_connection(args_log, write=False)

# create an empty file to prevent mavutil from treating the path as serial
with open(output_path, "w"):
    pass
mav_to = mavlink_connection(output_path, write=True)

# switch protocol like in mavutil.mavfile.auto_mavlink_version
mav_from.mav = module_from.MAVLink(mav_from)
mav_to.mav = module_to.MAVLink(mav_to)

translated_names = {}
dropped_messages = {}

if args_verbosity >= 1:
    print("Translating messages...")

while True:
    message_from = mav_from.recv_msg()
    if not message_from:
        break

    # get message by name from target module
    mtype_from = message_from.get_type()
    try:
        msg_id_to = getattr(module_to, "MAVLINK_MSG_ID_" + mtype_from)
    except AttributeError:
        dropped_messages[mtype_from] = dropped_messages.get(mtype_from, 0) + 1
        continue

    # HACK: For some reason was getting STATUSTEXT with id 0
    if message_from.id != msg_id_to and message_from.id != 0:
        translated_names[mtype_from] = (message_from.id, msg_id_to)

    MAVLink_translated_message = module_to.mavlink_map[msg_id_to]

    # map fields by names, otherwise replace with zeros or empty string
    fields = []
    for fname in MAVLink_translated_message.fieldnames:
        ftype = MAVLink_translated_message.fieldtypes[MAVLink_translated_message.fieldnames.index(fname)]
        flength = MAVLink_translated_message.array_lengths[MAVLink_translated_message.ordered_fieldnames.index(fname)]
        try:
            value = getattr(message_from, fname)
            if flength > 0 and ftype == "char":
                value = value.encode("ascii")
            fields.append(value)
        except AttributeError:
            if flength > 0 and ftype == "char":
                fields.append(bytes([0]))
            elif flength > 0:
                fields.append([0] * flength)
            else:
                fields.append(0)

    message_to = MAVLink_translated_message(*fields)

    mav_to.write(struct.pack(">Q", round(message_from._timestamp * 1.0e6)))
    mav_to.write(message_to.pack(mav_to.mav))


if args_verbosity >= 0:
    if len(translated_names) > 0:
        print("Message ids translated by name:")
        for name, (id_from, id_to) in translated_names.items():
            print("  ", name, id_from, "to", id_to)
    if len(dropped_messages) > 0:
        print("Did not translate the following messages:")
        for mtype, count in dropped_messages.items():
            print("  ", mtype, "found", count, "times")
