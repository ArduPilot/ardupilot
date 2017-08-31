#!/usr/bin/env python
'''
decode a Saleae logic CSV export file as PWM values. Used for
testing output values on PWM channels
'''

import csv, sys
import matplotlib.pyplot as plt
import numpy as np;

filename = sys.argv[1]

c = open(filename, 'r')
data = csv.reader(c,delimiter=';')

pulse_start = []
prev_values = []
pwm = []
nchannels = 0
x = []
y = []
min_frame_gap = 1e6
min_gap_t = 0
trailing_edge0 = 0
sbus_latency_max = 0
sbus_latency_max_t = 0
last_frame = 0
frame_start = 0
frame_interval = 0
min_frame_interval = 1e6
max_frame_interval = 0
min_interval_t = 0
max_interval_t = 0
rising_edge5 = 0
new_frame = False
frame_intervals = []
maxwidth5 = 0
maxwidth5_t = 0

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

    # time in seconds
    tfields = row[0].split()
    if len(tfields) > 1:
        tval = float(''.join(tfields[:-1]))
        units = tfields[-1]
        if (units == 'us'):
            t = tval * 1e-6
        elif (units == 'ms'):
            t = tval * 1e-3
        elif (units == 's'):
            t = tval
    else:
        t = float(row[0])

    # current value of each channel
    values = [int(row[i]) for i in range(1,len(row))]
    
    # channel 5 is a pulse bracketing the call to _write_fd
    # measure delay from ch5 rising edge to start of sbus frame
    if values[5] == 1 and prev_values[5] == 0:
        rising_edge5 = t
        new_frame = True
    if values[5] == 0 and prev_values[5] == 1 and rising_edge5 != 0:
        width5 = t - rising_edge5;
        if frame_start < rising_edge5:
            print("aux6 TE before frame_start at %.6f" % t)
        if width5 > maxwidth5:
            maxwidth5 = width5
            maxwidth5_t = t
    prev_values[5] = values[5]
                
    # channel 0 is the uart output
    # look for start of frame at each rising edge
    if values[0] == 1 and prev_values[0] == 0 and trailing_edge0 != 0:
        # require interframe gap of at least 12 bits)
        if (t - trailing_edge0) > .000120:
            frame_interval = t - frame_start
            frame_intervals.append(frame_interval)
            if frame_interval > .003 and frame_interval < min_frame_interval: 
                min_frame_interval = frame_interval
                min_interval_t = t
            if (frame_interval < .008) and (frame_interval > max_frame_interval): 
                max_frame_interval = frame_interval
                max_interval_t = t
            frame_gap = t - trailing_edge0
            frame_start = t
            if new_frame:
                new_frame = False
                sbus_latency = frame_start - rising_edge5
                if (sbus_latency > sbus_latency_max):
                    sbus_latency_max = sbus_latency
                    sbus_latency_max_t = frame_start
                
#             print("frame_start: %.6f, gap: %.6f" % (frame_start, frame_gap))
            if frame_gap < min_frame_gap:
                min_frame_gap = frame_gap
                min_gap_t = t
    
    # record time of last trailing edge
    if values[0] == 0 and prev_values[0] == 1:
        trailing_edge0 = t
        
    prev_values[0] = values[0]
    
    # channels 1-4 are the sbus decoder outputs
    channelset = range(1,5)
    for c in channelset:
        if values[c] == 0 and prev_values[c] == 1 and pulse_start[c] != 0:
            pulse = t - pulse_start[c]
            if pulse < .003:
                pwm[c] = pulse
                changed = True
        if values[c] == 1 and prev_values[c] == 0:
            pulse_start[c] = t
        prev_values[c] = values[c]
        
    if changed:
        sys.stdout.write("%.7f" % t)
        for c in channelset:
            sys.stdout.write(" %.1f" % (1e6*pwm[c]))
        sys.stdout.write("\n")
            
            
        x.append(t)
        y.append(pwm[1:nchannels])

sys.stdout.write("min_frame_gap: %.6f at %.6f\n" % (min_frame_gap, min_gap_t))
sys.stdout.write("min_frame_interval: %.6f, at: %.6f\n" % (min_frame_interval, min_interval_t))
sys.stdout.write("max_frame_interval: %.6f, at: %.6f\n" % (max_frame_interval, max_interval_t))
sys.stdout.write("sbus_latency_max: %.6f, at: %.6f\n" % (sbus_latency_max, sbus_latency_max_t))
sys.stdout.write("maxwidth5: %.6f, at: %.6f\n" % (maxwidth5, maxwidth5_t))

intervals = np.array(frame_intervals)

xa = np.array(x)
ya = np.array(y)
plt.figure(1)
plt.plot(xa, ya)
plt.grid()

plt.figure(2)
plt.hist(intervals,25)

plt.show()

