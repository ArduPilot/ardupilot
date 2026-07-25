#!/usr/bin/env python3

'''
merge two tlogs
'''

import os
import struct

from pymavlink import mavutil


class MAVMerge(object):
    def __init__(self, log1, log2, output_filepath=None, quiet=False):
        self.filename1 = log1
        self.filename2 = log2
        self.output_filepath = output_filepath
        self.output_fh = None
        self.quiet = quiet

    def emit_message(self, m):
        '''emit message to stdout, and possibly to output file'''
        if not self.quiet:
            print(str(m))
        if self.output_fh is not None:
            self.output_fh.write(struct.pack('>Q', int(m._timestamp*1.0e6)))
            self.output_fh.write(m.get_msgbuf())

    def run(self):
        mlog1 = mavutil.mavlink_connection(self.filename1, dialect='all')
        mlog2 = mavutil.mavlink_connection(self.filename2, dialect='all')

        if self.output_filepath is not None:
            self.output_fh = open(self.output_filepath, mode='wb')

        m1 = mlog1.recv_match()
        m2 = mlog2.recv_match()
        while m1 is not None or m2 is not None:
            if m1 is None:
                self.emit_message(m2)
                m2 = mlog2.recv_match()
                continue
            if m2 is None:
                self.emit_message(m1)
                m1 = mlog1.recv_match()
                continue

            if m1._timestamp < m2._timestamp:
                self.emit_message(m1)
                m1 = mlog1.recv_match()
            else:
                self.emit_message(m2)
                m2 = mlog1.recv_match()


if __name__ == '__main__':

    os.environ['MAVLINK20'] = '1'

    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("log1", metavar="LOG1")
    parser.add_argument("log2", metavar="LOG2")
    parser.add_argument("-o", "--output", default=None, help="output to given file")
    parser.add_argument("-q", "--quiet", action='store_true', help="don't display packets")

    args = parser.parse_args()

    filename1 = args.log1
    filename2 = args.log2

    merger = MAVMerge(
        filename1,
        filename2,
        output_filepath=args.output,
        quiet=args.quiet,
    )
    merger.run()
