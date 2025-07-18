#!/usr/bin/env python3

# flake8: noqa

'''
decode a Saleae logic CSV export file as PWM values. Used for
testing output values on PWM channels
'''

import csv, sys

filename = sys.argv[1]

c = open(filename, 'r')
data = csv.reader(c)

pulse_start = []
prev_values = []
pwm = []
nchannels = 0

for row in data:
    # skip header
    if row[0].startswith('Time'):
        continue
    changed = False

    if len(row) > nchannels+1:
        nchannels = len(row)-1
        while len(pwm) < nchannels:
            pwm.append(0)
            pulse_start.append(0)
            prev_values.append(0)

    # time in microseconds
    t = int(float(row[0])*1e6)

    # current value of each channel
    values = [int(row[i]) for i in range(1,len(row))]
    
    for c in range(nchannels):
        if values[c] == 0 and prev_values[c] == 1 and pulse_start[c] != 0:
            pulse = t - pulse_start[c]
            if pulse < 10000:
                pwm[c] = pulse
                changed = True
        if values[c] == 1 and prev_values[c] == 0:
            pulse_start[c] = t
        prev_values[c] = values[c]
        
    if changed:
        for c in range(nchannels):
            sys.stdout.write("%4u " % pwm[c])
        sys.stdout.write("%.3f\n" % float(row[0]))

