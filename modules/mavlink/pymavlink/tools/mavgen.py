#!/usr/bin/env python3

'''
parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

'''

# allow running mavgen from within the tree without installing
if __name__ == "__main__" and __package__ is None:
    from os import sys, path
    sys.path.insert(0, path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))

from pymavlink.generator import mavgen
from pymavlink.generator import mavparse

from argparse import ArgumentParser

parser = ArgumentParser(description="This tool generate implementations from MAVLink message definitions")
parser.add_argument("-o", "--output", default="mavlink", help="output directory.")
parser.add_argument("--lang", dest="language", choices=mavgen.supportedLanguages, default=mavgen.DEFAULT_LANGUAGE, help="language of generated code [default: %(default)s]")
parser.add_argument("--wire-protocol", choices=[mavparse.PROTOCOL_0_9, mavparse.PROTOCOL_1_0, mavparse.PROTOCOL_2_0], default=mavgen.DEFAULT_WIRE_PROTOCOL, help="MAVLink protocol version. [default: %(default)s]")
parser.add_argument("--no-validate", action="store_false", dest="validate", default=mavgen.DEFAULT_VALIDATE, help="Do not perform XML validation. Can speed up code generation if XML files are known to be correct.")
parser.add_argument("--strict-units", action="store_true", dest="strict_units", default=mavgen.DEFAULT_STRICT_UNITS, help="Perform validation of units attributes.")
parser.add_argument("definitions", metavar="XML", nargs="+", help="MAVLink definitions")
args = parser.parse_args()

ok = mavgen.mavgen(args, args.definitions)
# report failure in the exit code when no exception is thrown
if not ok:
    exit(1)
