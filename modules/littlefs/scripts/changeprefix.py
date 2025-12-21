#!/usr/bin/env python3
#
# Change prefixes in files/filenames. Useful for creating different versions
# of a codebase that don't conflict at compile time.
#
# Example:
# $ ./scripts/changeprefix.py lfs lfs3
#
# Copyright (c) 2022, The littlefs authors.
# Copyright (c) 2019, Arm Limited. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import glob
import itertools
import os
import os.path
import re
import shlex
import shutil
import subprocess
import tempfile

GIT_PATH = ['git']


def openio(path, mode='r', buffering=-1):
    # allow '-' for stdin/stdout
    if path == '-':
        if mode == 'r':
            return os.fdopen(os.dup(sys.stdin.fileno()), mode, buffering)
        else:
            return os.fdopen(os.dup(sys.stdout.fileno()), mode, buffering)
    else:
        return open(path, mode, buffering)

def changeprefix(from_prefix, to_prefix, line):
    line, count1 = re.subn(
        '\\b'+from_prefix,
        to_prefix,
        line)
    line, count2 = re.subn(
        '\\b'+from_prefix.upper(),
        to_prefix.upper(),
        line)
    line, count3 = re.subn(
        '\\B-D'+from_prefix.upper(),
        '-D'+to_prefix.upper(),
        line)
    return line, count1+count2+count3

def changefile(from_prefix, to_prefix, from_path, to_path, *,
        no_replacements=False):
    # rename any prefixes in file
    count = 0

    # create a temporary file to avoid overwriting ourself
    if from_path == to_path and to_path != '-':
        to_path_temp = tempfile.NamedTemporaryFile('w', delete=False)
        to_path = to_path_temp.name
    else:
        to_path_temp = None

    with openio(from_path) as from_f:
        with openio(to_path, 'w') as to_f:
            for line in from_f:
                if not no_replacements:
                    line, n = changeprefix(from_prefix, to_prefix, line)
                    count += n
                to_f.write(line)

    if from_path != '-' and to_path != '-':
        shutil.copystat(from_path, to_path)

    if to_path_temp:
        os.rename(to_path, from_path)
    elif from_path != '-':
        os.remove(from_path)

    # Summary
    print('%s: %d replacements' % (
        '%s -> %s' % (from_path, to_path) if not to_path_temp else from_path,
        count))

def main(from_prefix, to_prefix, paths=[], *,
        verbose=False,
        output=None,
        no_replacements=False,
        no_renames=False,
        git=False,
        no_stage=False,
        git_path=GIT_PATH):
    if not paths:
        if git:
            cmd = git_path + ['ls-tree', '-r', '--name-only', 'HEAD']
            if verbose:
                print(' '.join(shlex.quote(c) for c in cmd))
            paths = subprocess.check_output(cmd, encoding='utf8').split()
        else:
            print('no paths?', file=sys.stderr)
            sys.exit(1)
        
    for from_path in paths:
        # rename filename?
        if output:
            to_path = output
        elif no_renames:
            to_path = from_path
        else:
            to_path = os.path.join(
                os.path.dirname(from_path),
                changeprefix(from_prefix, to_prefix,
                    os.path.basename(from_path))[0])

        # rename contents
        changefile(from_prefix, to_prefix, from_path, to_path,
            no_replacements=no_replacements)

        # stage?
        if git and not no_stage:
            if from_path != to_path:
                cmd = git_path + ['rm', '-q', from_path]
                if verbose:
                    print(' '.join(shlex.quote(c) for c in cmd))
                subprocess.check_call(cmd)
            cmd = git_path + ['add', to_path]
            if verbose:
                print(' '.join(shlex.quote(c) for c in cmd))
            subprocess.check_call(cmd)


if __name__ == "__main__":
    import argparse
    import sys
    parser = argparse.ArgumentParser(
        description="Change prefixes in files/filenames. Useful for creating "
            "different versions of a codebase that don't conflict at compile "
            "time.",
        allow_abbrev=False)
    parser.add_argument(
        'from_prefix',
        help="Prefix to replace.")
    parser.add_argument(
        'to_prefix',
        help="Prefix to replace with.")
    parser.add_argument(
        'paths',
        nargs='*',
        help="Files to operate on.")
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help="Output commands that run behind the scenes.")
    parser.add_argument(
        '-o', '--output',
        help="Output file.")
    parser.add_argument(
        '-N', '--no-replacements',
        action='store_true',
        help="Don't change prefixes in files")
    parser.add_argument(
        '-R', '--no-renames',
        action='store_true',
        help="Don't rename files")
    parser.add_argument(
        '--git',
        action='store_true',
        help="Use git to find/update files.")
    parser.add_argument(
        '--no-stage',
        action='store_true',
        help="Don't stage changes with git.")
    parser.add_argument(
        '--git-path',
        type=lambda x: x.split(),
        default=GIT_PATH,
        help="Path to git executable, may include flags. "
            "Defaults to %r." % GIT_PATH)
    sys.exit(main(**{k: v
        for k, v in vars(parser.parse_intermixed_args()).items()
        if v is not None}))
