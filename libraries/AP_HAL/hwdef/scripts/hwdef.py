'''
base class for hwdef processing

AP_FLAKE8_CLEAN
'''

import filecmp
import os
import re
import shlex
import sys


class IncludeNotFoundException(Exception):
    def __init__(self, hwdef, includer):
        self.hwdef = hwdef
        self.includer = includer


class HWDef:
    def __init__(self, quiet=False, outdir=None, hwdef=[]):
        self.outdir = outdir
        self.hwdef = hwdef
        self.quiet = quiet

        # dictionary of all config lines, indexed by first word
        self.config = {}

        # all config lines in order
        self.alllines = []

        # allow for extra env vars
        self.env_vars = {}

        # output lines:
        self.all_lines = []

        # integer defines
        self.intdefines = {}

        # boolean indicating whether we have read and processed self.hwdef
        self.processed_hwdefs = False

    def is_int(self, str):
        '''check if a string is an integer'''
        try:
            int(str)
        except Exception:
            return False
        return True

    def error(self, str):
        '''show an error and exit'''
        print("Error: " + str)
        sys.exit(1)

    def load_file_with_include(self, fname):
        '''load a file as an array of lines, processing any include lines'''
        lines = open(fname, 'r').readlines()
        ret = []
        for line in lines:
            if line.startswith("include"):
                a = shlex.split(line)
                if len(a) > 1 and a[0] == "include":
                    fname2 = os.path.relpath(os.path.join(os.path.dirname(fname), a[1]))
                    ret.extend(self.load_file_with_include(fname2))
                    continue
            ret.append(line)
        return ret

    def get_numeric_board_id(self):
        '''return a numeric board ID, which may require mapping a string to a
        number via board_list.txt'''
        some_id = self.get_config('APJ_BOARD_ID')
        if some_id.isnumeric():
            return some_id

        board_types_filename = "board_types.txt"
        topdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../..')
        board_types_dirpath = os.path.join(topdir, "Tools", "AP_Bootloader")
        board_types_filepath = os.path.join(board_types_dirpath, board_types_filename)
        for line in open(board_types_filepath, 'r'):
            m = re.match(r"(?P<name>[-\w]+)\s+(?P<board_id>\d+)", line)
            if m is None:
                continue
            if m.group('name') == some_id:
                return m.group('board_id')

        raise ValueError("Unable to map (%s) to a board ID using %s" %
                         (some_id, board_types_filepath))

    def write_all_lines(self, hwdat):
        f = open(hwdat, 'w')
        f.write('\n'.join(self.all_lines))
        f.close()

    def write_defaulting_define(self, f, name, value):
        f.write(f"#ifndef {name}\n")
        f.write(f"#define {name} {value}\n")
        f.write("#endif\n")

    def write_define(self, f, name, value):
        f.write(f"#define {name} {value}\n")

    def write_hwdef_header(self, outfilename):
        '''write hwdef header file'''
        self.progress("Writing hwdef setup in %s" % outfilename)
        tmpfile = outfilename + ".tmp"
        f = open(tmpfile, 'w')

        f.write('''/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

''')

        self.write_hwdef_header_content(f)

        f.close()
        # see if we ended up with the same file, on an unnecessary reconfigure
        try:
            if filecmp.cmp(outfilename, tmpfile):
                self.progress("No change in hwdef.h")
                os.unlink(tmpfile)
                return
        except Exception:
            pass
        try:
            os.unlink(outfilename)
        except Exception:
            pass
        os.rename(tmpfile, outfilename)

    def progress(self, message):
        if self.quiet:
            return
        print(message)

    def process_file(self, filename, depth=0):
        '''process a hwdef.dat file'''
        try:
            f = open(filename, "r")
        except Exception:
            self.error("Unable to open file %s" % filename)
        for line in f.readlines():
            line = line.split('#')[0] # ensure we discard the comments
            line = line.strip()
            if len(line) == 0 or line[0] == '#':
                continue
            a = shlex.split(line)
            if a[0] == "include" and len(a) > 1:
                include_file = a[1]
                if include_file[0] != '/':
                    dir = os.path.dirname(filename)
                    include_file = os.path.normpath(
                        os.path.join(dir, include_file))
                self.progress("Including %s" % include_file)
                self.process_file(include_file, depth+1)
            else:
                self.process_line(line, depth)

    def process_hwdefs(self):
        for fname in self.hwdef:
            self.process_file(fname)
        self.processed_hwdefs = True

    def run(self):
        # process input file
        self.process_hwdefs()

        # write out hwdef.h
        self.write_hwdef_header(os.path.join(self.outdir, "hwdef.h"))

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        a = shlex.split(line, posix=False)

        if a[0] == 'undef':
            return self.process_line_undef(line, depth, a)

        if a[0] == 'env':
            return self.process_line_env(line, depth, a)

        if a[0] == 'define':
            return self.process_line_define(line, depth, a)

    def process_line_undef(self, line, depth, a):
        for u in a[1:]:
            self.progress("Removing %s" % u)
            self.config.pop(u, '')
            self.intdefines.pop(u, '')
            # also remove all occurences of defines in previous lines if any
            for line in self.alllines[:]:
                if line.startswith('define') and u == line.split()[1]:
                    self.alllines.remove(line)

    def process_line_env(self, line, depth, a):
        self.progress("Adding environment %s" % ' '.join(a[1:]))
        if len(a[1:]) < 2:
            self.error("Bad env line for %s" % a[0])
        name = a[1]
        value = ' '.join(a[2:])
        self.env_vars[name] = value

    def process_line_define(self, line, depth, a):
        # extract numerical defines for processing by other parts of the script
        result = re.match(r'define\s*([A-Z_0-9]+)\s+([0-9]+)', line)
        if result:
            (name, intvalue) = (result.group(1), int(result.group(2)))
            if name in self.intdefines and self.intdefines[name] == intvalue:
                msg = f"{name} already in defines with same value"
                if depth == 0:
                    print(msg)
                    # raise ValueError(msg)

            self.intdefines[name] = intvalue
