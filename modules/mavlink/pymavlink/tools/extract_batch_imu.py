#!/usr/bin/env python3

'''
extract ISBH and ISBD messages from AP_Logging files into a bin GYR and ACC log
'''
from pymavlink import DFReader
from argparse import ArgumentParser

parser = ArgumentParser('Parse bin log into GYR+ACC bin log')
parser.add_argument("inlog")
parser.add_argument("outlog")

args = parser.parse_args()

from pymavlink import mavutil

class DF_logger:
    '''write to a DF format log'''
    def __init__(self, filename):
        self.outf = open(filename,'wb')
        self.outf.write(bytes([0]))
        self.outf.flush()
        self.mlog = DFReader.DFReader_binary(filename)
        self.outf.seek(0)
        self.formats = {}

    def write(self, name, fmt, fields, *args):
        if not name in self.formats:
            self.formats[name] = self.mlog.add_format(DFReader.DFFormat(0, name, 0, fmt, fields))
            self.outf.write(self.mlog.make_format_msgbuf(self.formats[name]))
        self.outf.write(self.mlog.make_msgbuf(self.formats[name], args))
        self.outf.flush()

def extract_batch(inlog, outlog):
    print("Processing log %s" % inlog)
    mlog = mavutil.mavlink_connection(inlog)
    dflog = DF_logger(outlog)

    ISBH = None
    count = 0

    while True:
        m = mlog.recv_match(type=['ISBH','ISBD'])
        if m is None:
            break
        msg_type = m.get_type()
        if msg_type == "ISBH":
            ISBH = m
            count += 1
        elif msg_type == "ISBD" and ISBH is not None:
            if m.N != ISBH.N:
                continue
            stype = 'ACC' if ISBH.type == 0 else 'GYR'
            stype += '%u' % ISBH.instance
            sample_us = int(1.0e6 / ISBH.smp_rate)
            tbase = ISBH.SampleUS + (m.seqno * 32) * sample_us
            mul = 1.0/ISBH.mul
            for i in range(32):
                dflog.write(stype, 'Qfff', 'TimeUS,X,Y,Z',
                            tbase+i*sample_us,
                            m.x[i]*mul, m.y[i]*mul, m.z[i]*mul)

    print("Extracted %u frames" % count)
extract_batch(args.inlog, args.outlog)

