#!/usr/bin/python

# Copyright (c) 2008,2010,2011,2012,2013 Alexander Belchenko
# All rights reserved.
#
# Redistribution and use in source and binary forms,
# with or without modification, are permitted provided
# that the following conditions are met:
#
# * Redistributions of source code must retain
#   the above copyright notice, this list of conditions
#   and the following disclaimer.
# * Redistributions in binary form must reproduce
#   the above copyright notice, this list of conditions
#   and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the author nor the names
#   of its contributors may be used to endorse
#   or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''Intel HEX file format bin2hex convertor utility.'''

VERSION = '1.5'

if __name__ == '__main__':
    import getopt
    import os
    import sys

    from intelhex import bin2hex

    usage = '''Bin2Hex convertor utility.
Usage:
    python bin2hex.py [options] INFILE [OUTFILE]

Arguments:
    INFILE      name of bin file for processing.
                Use '-' for reading from stdin.

    OUTFILE     name of output file. If omitted then output
                will be writing to stdout.

Options:
    -h, --help              this help message.
    -v, --version           version info.
    --offset=N              offset for loading bin file (default: 0).
'''

    offset = 0

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hv",
                                  ["help", "version", "offset="])

        for o, a in opts:
            if o in ("-h", "--help"):
                print(usage)
                sys.exit(0)
            elif o in ("-v", "--version"):
                print(VERSION)
                sys.exit(0)
            elif o in ("--offset"):
                base = 10
                if a[:2].lower() == '0x':
                    base = 16
                try:
                    offset = int(a, base)
                except:
                    raise getopt.GetoptError('Bad offset value')

        if not args:
            raise getopt.GetoptError('Input file is not specified')

        if len(args) > 2:
            raise getopt.GetoptError('Too many arguments')

    except getopt.GetoptError as msg:
        txt = 'ERROR: '+str(msg)    # that's required to get not-so-dumb result from 2to3 tool
        print(txt)
        print(usage)
        sys.exit(2)

    def force_stream_binary(stream):
        """Force binary mode for stream on Windows."""
        if os.name == 'nt':
            f_fileno = getattr(stream, 'fileno', None)
            if f_fileno:
                fileno = f_fileno()
                if fileno >= 0:
                    import msvcrt
                    msvcrt.setmode(fileno, os.O_BINARY)

    fin = args[0]
    if fin == '-':
        # read from stdin
        fin = sys.stdin
        force_stream_binary(fin)
    elif not os.path.isfile(fin):
        txt = "ERROR: File not found: %s" % fin   # that's required to get not-so-dumb result from 2to3 tool
        print(txt)
        sys.exit(1)

    if len(args) == 2:
        fout = args[1]
    else:
        # write to stdout
        fout = sys.stdout
        force_stream_binary(fout)

    sys.exit(bin2hex(fin, fout, offset))
