#!/usr/bin/env python3
'''
compare two MAVLink parameter files
'''

from pymavlink import mavparm

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("file1", metavar="FILE1")
parser.add_argument("file2", metavar="FILE2")
parser.add_argument("-t",
                    help="use tabs delimiter between columns for the output",
                    default=False,
                    action='store_true',
                    dest='use_tabs')
parser.add_argument("--full-diff",
                    help="include volatile and similar parameters",
                    default=True,
                    action='store_false',
                    dest='use_excludes')
parser.add_argument("--hide-only1",
                    help="hide params only in first file",
                    default=False,
                    action='store_true')
parser.add_argument("--hide-only2",
                    help="hide params only in second file",
                    default=False,
                    action='store_true')

args = parser.parse_args()

file1 = args.file1
file2 = args.file2

p1 = mavparm.MAVParmDict()
p2 = mavparm.MAVParmDict()
p1.load(file2, use_excludes=args.use_excludes)
p1.diff(file1, use_excludes=args.use_excludes, use_tabs=args.use_tabs,
        show_only1=not args.hide_only1,
        show_only2=not args.hide_only2)
