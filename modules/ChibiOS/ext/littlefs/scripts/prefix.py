#!/usr/bin/env python2

# This script replaces prefixes of files, and symbols in that file.
# Useful for creating different versions of the codebase that don't
# conflict at compile time.
#
# example:
# $ ./scripts/prefix.py lfs2

import os
import os.path
import re
import glob
import itertools
import tempfile
import shutil
import subprocess

DEFAULT_PREFIX = "lfs"

def subn(from_prefix, to_prefix, name):
    name, count1 = re.subn('\\b'+from_prefix, to_prefix, name)
    name, count2 = re.subn('\\b'+from_prefix.upper(), to_prefix.upper(), name)
    name, count3 = re.subn('\\B-D'+from_prefix.upper(),
            '-D'+to_prefix.upper(), name)
    return name, count1+count2+count3

def main(from_prefix, to_prefix=None, files=None):
    if not to_prefix:
        from_prefix, to_prefix = DEFAULT_PREFIX, from_prefix

    if not files:
        files = subprocess.check_output([
                'git', 'ls-tree', '-r', '--name-only', 'HEAD']).split()

    for oldname in files:
        # Rename any matching file names
        newname, namecount = subn(from_prefix, to_prefix, oldname)
        if namecount:
            subprocess.check_call(['git', 'mv', oldname, newname])

        # Rename any prefixes in file
        count = 0
        with open(newname+'~', 'w') as tempf:
            with open(newname) as newf:
                for line in newf:
                    line, n = subn(from_prefix, to_prefix, line)
                    count += n
                    tempf.write(line)
        shutil.copystat(newname, newname+'~')
        os.rename(newname+'~', newname)
        subprocess.check_call(['git', 'add', newname])

        # Summary
        print '%s: %d replacements' % (
                '%s -> %s' % (oldname, newname) if namecount else oldname,
                count)

if __name__ == "__main__":
    import sys
    sys.exit(main(*sys.argv[1:]))
