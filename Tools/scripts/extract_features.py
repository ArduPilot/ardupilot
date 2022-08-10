#!/usr/bin/env python

"""
script to determine what features have been built into an ArduPilot binary

AP_FLAKE8_CLEAN
"""

import optparse
import os
import re
import string
import subprocess
import sys
import time
import build_options


if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


class ExtractFeatures(object):

    def __init__(self, filename):
        self.filename = filename
        self.nm = 'arm-none-eabi-nm'

        # feature_name should match the equivalent feature in
        # build_options.py ('FEATURE_NAME', 'EXPECTED_SYMBOL').
        # EXPECTED_SYMBOL is a regular expression which will be matched
        # against "define" in build_options's feature list, and
        # FEATURE_NAME will have substitutions made from the match.
        # the substitutions will be upper-cased
        self.features = [
            ('AP_AIRSPEED_ENABLED', 'AP_Airspeed::AP_Airspeed',),
            ('AP_AIRSPEED_{type}_ENABLED', 'AP_Airspeed_(?P<type>.*)::AP_Airspeed_(?P=type)',),
            ('HAL_ADSB_ENABLED', 'AP_ADSB::AP_ADSB',),
            ('HAL_EFI_ENABLED', 'AP_EFI::AP_EFI',),
            ('BEACON_ENABLED', 'AP_Beacon::AP_Beacon',),
            ('HAL_TORQEEDO_ENABLED', 'AP_Torqeedo::AP_Torqeedo'),
        ]

    def progress(self, string):
        '''pretty-print progress'''
        print("EF: %s" % string)

    def run_program(self, prefix, cmd_list, show_output=True, env=None):
        '''swiped from build_binaries.py'''
        if show_output:
            self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(cmd_list, bufsize=1, stdin=None,
                             stdout=subprocess.PIPE, close_fds=True,
                             stderr=subprocess.STDOUT, env=env)
        output = ""
        while True:
            x = p.stdout.readline()
            if len(x) == 0:
                returncode = os.waitpid(p.pid, 0)
                if returncode:
                    break
                    # select not available on Windows... probably...
                time.sleep(0.1)
                continue
            if running_python3:
                x = bytearray(x)
                x = filter(lambda x : chr(x) in string.printable, x)
                x = "".join([chr(c) for c in x])
            output += x
            x = x.rstrip()
            if show_output:
                print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0 and show_output:
            self.progress("Process failed (%s)" %
                          str(returncode))
            raise subprocess.CalledProcessError(
                returncode, cmd_list)
        return output

    class Symbols(object):
        def __init__(self):
            self.symbols = dict()
            self.symbols_without_arguments = dict()

        def add(self, key, attributes):
            self.symbols[key] = attributes

            # also keep around the same symbol name without arguments.
            # if the key is already present then the attributes become
            # None as there are multiple possible answers...
            m = re.match("^([^(]+).*", key)
            if m is None:
                extracted_symbol_name = key
            else:
                extracted_symbol_name = m.group(1)
            # print("Adding (%s)" % str(extracted_symbol_name))
            if extracted_symbol_name in self.symbols_without_arguments:
                self.symbols_without_arguments[extracted_symbol_name] = None
            else:
                self.symbols_without_arguments[extracted_symbol_name] = attributes

        def dict_for_symbol(self, symbol):
            if '(' not in symbol:
                some_dict = self.symbols_without_arguments
            else:
                some_dict = self.symbols
            return some_dict

    def extract_symbols_from_elf(self, filename):
        '''parses ELF in filename, returns dict of symbols=>attributes'''
        text_output = self.run_program('EF', [
            self.nm,
            '--demangle',
            '--print-size',
            filename
        ], show_output=False)
        ret = ExtractFeatures.Symbols()
        for line in text_output.split("\n"):
            m = re.match("^([^ ]+) ([^ ]+) ([^ ]) (.*)", line.rstrip())
            if m is None:
                m = re.match("^([^ ]+) ([^ ]) (.*)", line.rstrip())
                if m is None:
                    # raise ValueError("Did not match (%s)" % line)
                    # e.g. Did not match (         U _errno)
                    continue
                (offset, symbol_type, symbol_name) = m.groups()
                size = "0"
            else:
                (offset, size, symbol_type, symbol_name) = m.groups()
            size = int(size, 16)
            # print("symbol (%s) size %u" % (str(symbol_name), size))
            ret.add(symbol_name, {
                "size": size,
            })

        return ret

    def run(self):

        build_options_defines = set([x.define for x in build_options.BUILD_OPTIONS])

        symbols = self.extract_symbols_from_elf(filename)

        for (feature_define, symbol) in self.features:
            some_dict = symbols.dict_for_symbol(symbol)
            # look for symbols without arguments
            # print("Looking for (%s)" % str(name))
            for s in some_dict.keys():
                m = re.match(symbol, s)
                # print("matching %s with %s" % (symbol, s))
                if m is None:
                    continue
                d = m.groupdict()
                for key in d.keys():
                    d[key] = d[key].upper()
                # filter to just the defines present in
                # build_options.py - otherwise we end up with (e.g.)
                # AP_AIRSPEED_BACKEND_ENABLED, even 'though that
                # doesn't exist in the ArduPilot codebase.
                some_define = feature_define.format(**d)
                if some_define not in build_options_defines:
                    continue
                print(some_define)


if __name__ == '__main__':

    parser = optparse.OptionParser("extract_features.py FILENAME")

    cmd_opts, cmd_args = parser.parse_args()

    if len(cmd_args) < 1:
        parser.print_help()
        sys.exit(1)

    filename = cmd_args[0]

    ef = ExtractFeatures(filename)
    ef.run()
