#!/usr/bin/env python

'''
extract ISBH and ISBD messages from AP_Logging files and produce C++ arrays for consumption by the DSP subsystem
'''
from __future__ import print_function

import os
import sys
import time
import copy
import numpy

from argparse import ArgumentParser

parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

def isb_parser(logfile):
    '''display fft for raw ACC data in logfile'''

    '''object to store data about a single FFT plot'''
    class ISBData(object):
        def __init__(self, ffth):
            self.seqno = -1
            self.fftnum = ffth.N
            self.sensor_type = ffth.type
            self.instance = ffth.instance
            self.sample_rate_hz = ffth.smp_rate
            self.multiplier = ffth.mul
            self.data = {}
            self.data["X"] = []
            self.data["Y"] = []
            self.data["Z"] = []
            self.holes = False
            self.freq = None

        def add_isb(self, fftd):
            if fftd.N != self.fftnum:
                print("Skipping ISBD with wrong fftnum (%u vs %u)\n" % (fftd.fftnum, self.fftnum), file=sys.stderr)
                return
            if self.holes:
                print("Skipping ISBD(%u) for ISBH(%u) with holes in it" % (fftd.seqno, self.fftnum), file=sys.stderr)
                return
            if fftd.seqno != self.seqno+1:
                print("ISBH(%u) has holes in it" % fftd.N, file=sys.stderr)
                self.holes = True
                return
            self.seqno += 1
            self.data["X"].extend(fftd.x)
            self.data["Y"].extend(fftd.y)
            self.data["Z"].extend(fftd.z)

        def prefix(self):
            if self.sensor_type == 0:
                return "Accel"
            elif self.sensor_type == 1:
                return "Gyro"
            else:
                return "?Unknown Sensor Type?"

        def tag(self):
            return str(self)

        def __str__(self):
            return "%s[%u]" % (self.prefix(), self.instance)

    mlog = mavutil.mavlink_connection(logfile)

    isb_frames = []
    isbdata = None
    msgcount = 0
    start_time = time.time()
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        msgcount += 1
        if msgcount % 1000 == 0:
            sys.stderr.write(".")
        msg_type = m.get_type()
        if msg_type == "ISBH":
            if isbdata is not None:
                sensor = isbdata.tag()
                isb_frames.append(isbdata)
            isbdata = ISBData(m)
            continue

        if msg_type == "ISBD":
            if isbdata is None:
                sys.stderr.write("?(fftnum=%u)" % m.N)
                continue
            isbdata.add_isb(m)

    print("", file=sys.stderr)
    time_delta = time.time() - start_time
    print("Extracted %u fft data sets" % len(isb_frames), file=sys.stderr)

    sample_rates = {}
    counts = {}

    print("#include \"GyroFrame.h\"")
    print("const uint32_t NUM_FRAMES = %d;" % len(isb_frames))
    print("const GyroFrame gyro_frames[] = {")

    sample_rate = None
    for isb_frame in isb_frames:
        if isb_frame.sensor_type == 1 and isb_frame.instance == 0:  # gyro data
            print("    {")

            for axis in [ "X","Y","Z" ]:
                # normalize data
                d = numpy.array(isb_frame.data[axis]) # / float(isb_frame.multiplier)
                print("        { " + ", ".join(d.astype(numpy.dtype(str))) + " },")

                if len(d) == 0:
                    print("No data?!?!?!", file=sys.stderr)
            if sample_rate is None:
                sample_rate = isb_frame.sample_rate_hz
            print("    },")

    print("};")
    print("const uint16_t SAMPLE_RATE = %d;" % sample_rate)    



for filename in args.logs:
    isb_parser(filename)
