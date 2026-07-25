#!/usr/bin/env python3

'''
extract ISBH and ISBD messages from AP_Logging files into a csv file
'''
import numpy
import os
import sys
import time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

def mavfft_fttd(logfile):
    '''display fft for raw ACC data in logfile'''

    '''object to store data about a single FFT plot'''
    class PlotData(object):
        def __init__(self, ffth):
            self.seqno = -1
            self.fftnum = ffth.N
            self.sensor_type = ffth.type
            self.instance = ffth.instance
            self.sample_rate_hz = ffth.smp_rate
            self.multiplier = ffth.mul
            self.SampleUS = ffth.SampleUS
            self.data = {}
            self.data["X"] = []
            self.data["Y"] = []
            self.data["Z"] = []
            self.holes = False
            self.freq = None

        def add_fftd(self, fftd):
            if fftd.N != self.fftnum:
                print("Skipping ISBD with wrong fftnum (%u vs %u)\n" % (fftd.fftnum, self.fftnum))
                return
            if self.holes:
                print("Skipping ISBD(%u) for ISBH(%u) with holes in it" % (fftd.seqno, self.fftnum))
                return
            if fftd.seqno != self.seqno+1:
                print("ISBH(%u) has holes in it" % fftd.N)
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

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    things_to_plot = []
    plotdata = None
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
            if plotdata is not None:
                # close off previous data collection
                things_to_plot.append(plotdata)
            # initialise plot-data collection object
            plotdata = PlotData(m)
            continue

        if msg_type == "ISBD":
            if plotdata is None:
                print("Skipping ISBD outside ISBH (fftnum=%u)\n" % m.N)
                continue
            plotdata.add_fftd(m)

    print("", file=sys.stderr)
    time_delta = time.time() - start_time
    print("%us messages  %u messages/second  %u kB/second" % (msgcount, msgcount/time_delta, os.stat(filename).st_size/time_delta))
    print("Extracted %u fft data sets" % len(things_to_plot), file=sys.stderr)

    sum_fft = {}
    freqmap = {}
    count = 0

    first_freq = None
    files = {}

    for p in things_to_plot:
        fname = p.prefix() + str(p.instance) + ".csv"
        if not fname in files:
            files[fname] = open(fname, "w")
            f = files[fname]
            f.write("SampleUS,X,Y,Z\n")
        f = files[fname]
        dt = 1.0e6 / p.sample_rate_hz
        for i in range(len(p.data['X'])):
            x = p.data['X'][i]/p.multiplier
            y = p.data['Y'][i]/p.multiplier
            z = p.data['Z'][i]/p.multiplier
            f.write("%u,%.5f,%.5f,%.5f\n" % (int(p.SampleUS+i*dt), x, y, z))
    for f in files:
        files[f].close()

for filename in args.logs:
    mavfft_fttd(filename)
