#!/usr/bin/env python

"""
Finds all makefile projects in the given directories and allows to exclude
overlaps.

To get help on usage, possible options and their descriptions, use
the following command:

    find.py --help
"""

import argparse
import os
import sys


def find(args):
    result = []
    for path in args.dirs:
        for root, dirs, files in os.walk(path):
            if args.makefile and 'Makefile' in files:
                if args.no_overlaps and args.make and 'make' in dirs:
                    # Skip 'Makefile' because there is the 'make' subdir with
                    # more specific '*.make' makefiles.
                    continue
                result.append(os.path.join(root, 'Makefile'))
            if args.make and os.path.basename(root) == 'make':
                result.extend(
                    os.path.join(root, make)
                    for make in files
                    if make.endswith('.make')
                )
    return result


def main():
    parser = argparse.ArgumentParser(description='Find makefiles')
    parser.add_argument('--makefile', action='store_true',
                        help='Search for "Makefile" files.')
    parser.add_argument('--make', action='store_true',
                        help='Search for "*.make" files.')
    parser.add_argument('--no-overlaps', action='store_true',
                        help=('Exclude "Makefile" if there is the "make" '
                              'subdir which includes more specific "*.make" '
                              'files'))
    parser.add_argument('dirs', metavar='dir', nargs='+',
                        help='Directories in which to search for makefiles.')
    args = parser.parse_args()

    makefiles = find(args)
    sys.stdout.write('\n'.join(makefiles))
    sys.stdout.write('\n')


if __name__ == '__main__':
    main()
