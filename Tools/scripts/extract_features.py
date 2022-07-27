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


if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


class ExtractFeatures(object):

    def __init__(self, filename):
        self.filename = filename
        self.nm = 'arm-none-eabi-nm'

        # feature_name should match the equivalent feature in build_options.py
        # ('FEATURE_NAME', 'EXPECTED_SYMBOL')
        self.features = [
            ('AIRSPEED', 'AP_Airspeed::AP_Airspeed',),
            ('AP_ADSB', 'AP_ADSB::AP_ADSB',),
            ('AP_EFI', 'AP_EFI::AP_EFI',),
            ('BEACON', 'AP_Beacon::AP_Beacon',),
            ('TORQEEDO', 'AP_Torqeedo::AP_Torqeedo'),
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

        def find(self, name):
            if '(' not in name:
                # look for symbols without arguments
                # print("Looking for (%s)" % str(name))
                return self.symbols_without_arguments[name]

            return self.symbols[name]

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

        symbols = self.extract_symbols_from_elf(filename)

        for (feature_name, symbol) in self.features:
            try:
                symbols.find(symbol)
            except KeyError:
                continue

            print("%s" % str(feature_name))


if __name__ == '__main__':

    parser = optparse.OptionParser("extract_features.py FILENAME")

    cmd_opts, cmd_args = parser.parse_args()

    if len(cmd_args) < 1:
        parser.print_help()
        sys.exit(1)

    filename = cmd_args[0]

    ef = ExtractFeatures(filename)
    ef.run()
