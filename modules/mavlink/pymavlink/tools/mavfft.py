#!/usr/bin/env python3

'''
fit best estimate of magnetometer offsets
'''
import numpy
import pylab

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--sample-length", type=int, default=0, help="number of samples to run FFT over")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

def fft(logfile):
    '''display fft for raw ACC data in logfile'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    data = {'ACC1.rate' : 1000,
            'ACC2.rate' : 1600,
            'ACC3.rate' : 1000,
            'GYR1.rate' : 1000,
            'GYR2.rate' :  800,
            'GYR3.rate' : 1000}
    for acc in ['ACC1','ACC2','ACC3']:
        for ax in ['AccX', 'AccY', 'AccZ']:
            data[acc+'.'+ax] = []
    for gyr in ['GYR1','GYR2','GYR3']:
        for ax in ['GyrX', 'GyrY', 'GyrZ']:
            data[gyr+'.'+ax] = []

    # now gather all the data
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        type = m.get_type()
        if type.startswith("ACC"):
            data[type+'.AccX'].append(m.AccX)
            data[type+'.AccY'].append(m.AccY)
            data[type+'.AccZ'].append(m.AccZ)
        if type.startswith("GYR"):
            data[type+'.GyrX'].append(m.GyrX)
            data[type+'.GyrY'].append(m.GyrY)
            data[type+'.GyrZ'].append(m.GyrZ)

    print("Extracted %u data points" % len(data['ACC1.AccX']))

    for msg in ['ACC1', 'ACC2', 'ACC3', 'GYR1', 'GYR2', 'GYR3']:
        pylab.figure()

        if msg.startswith('ACC'):
            prefix = 'Acc'
        else:
            prefix = 'Gyr'
        for axis in ['X', 'Y', 'Z']:
            field = msg + '.' + prefix + axis
            d = data[field]
            if args.sample_length != 0:
                d = d[0:args.sample_length]
            d = numpy.array(d)
            if len(d) == 0:
                continue
            avg = numpy.sum(d) / len(d)
            d -= avg
            d_fft = numpy.fft.rfft(d)
            freq  = numpy.fft.rfftfreq(len(d), 1.0 / data[msg+'.rate'])
            pylab.plot( freq, numpy.abs(d_fft), label=field )
        pylab.legend(loc='upper right')

for filename in args.logs:
    fft(filename)

pylab.show()
