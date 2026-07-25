#!/usr/bin/env python3

'''
interactively select accel and gyro data for FFT analysis
'''
import numpy
import pylab
import matplotlib
import matplotlib.pyplot as pyplot

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

try:
    raw_input          # Python 2
except NameError:
    raw_input = input  # Python 3

def plot_input(data, msg, prefix, start, end):
    preview = pylab.figure()
    preview.set_size_inches(12, 3, forward=True)
    for axis in ['X', 'Y', 'Z']:
        field = msg + '.' + prefix + axis
        d = numpy.array(data[field][start:end])
        pylab.plot( d, marker='.', label=field )
    pylab.legend(loc='upper right')
#     pylab.ylabel('m/sec/sec')
    pylab.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.16)
    preview.canvas.set_window_title('FFT input: ' + msg)
    pylab.show()

def check_drops(data, msg, start, end):
    ts = 1e-6 * numpy.array(data[msg + '.TimeUS'])
    seqcnt = numpy.array(data[msg + '.SampleC'])

    deltas = numpy.diff(seqcnt[start:end])
#     print('ndeltas: ', len(deltas))
    duration = ts[end] - ts[start]
    print(msg + ' duration: {0:.3f} seconds'.format(duration))
    avg_rate = float(end - start - 1) / duration
    print('average logging rate: {0:.0f} Hz'.format(avg_rate))
    ts_mean = numpy.mean(deltas) 
    dmin = numpy.min(deltas)
    dmax = numpy.max(deltas)
    print('sample count delta min: {0}, max: {1}'.format(dmin, dmax))
    if (dmin != dmax):
        print('sample count delta mean: ', '{0:.2f}, std: {0:.2f}'.format(ts_mean, numpy.std(deltas)))
    print('sensor sample rate: {0:.0f} Hz'.format(ts_mean * avg_rate))

    drop_lens = []
    drop_times = []
    intvl_count = [0]
    for i in range(0, len(deltas)):
        if (deltas[i] > 1.5 * ts_mean):
            drop_lens.append(deltas[i])
            drop_times.append(ts[start+i])
            print('dropout at sample {0}: length {1}'.format(start+i, deltas[i]))
    
    print('{0:d} sample intervals > {1:.3f}'.format(len(drop_lens), 1.5 * ts_mean))
    return avg_rate
    
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
        for ax in ['AccX', 'AccY', 'AccZ', 'SampleC', 'TimeUS']:
            data[acc+'.'+ax] = []
    for gyr in ['GYR1','GYR2','GYR3']:
        for ax in ['GyrX', 'GyrY', 'GyrZ', 'SampleC', 'TimeUS']:
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
            data[type+'.SampleC'].append(m.SampleC)
            data[type+'.TimeUS'].append(m.TimeUS)
        if type.startswith("GYR"):
            data[type+'.GyrX'].append(m.GyrX)
            data[type+'.GyrY'].append(m.GyrY)
            data[type+'.GyrZ'].append(m.GyrZ)
            data[type+'.SampleC'].append(m.SampleC)
            data[type+'.TimeUS'].append(m.TimeUS)

    # SampleC is just a sample counter
    ts = 1e-6 * numpy.array(data['ACC1.TimeUS'])
    seqcnt = numpy.array(data['ACC1.SampleC'])

    print("Extracted %u data points" % len(data['ACC1.AccX']))
    
    # interactive selection of analysis window
    preview = pylab.figure()
    preview.set_size_inches(12, 3, forward=True)
    msg = 'ACC1'
    for axis in ['X', 'Y', 'Z']:
        field = msg + '.Acc' + axis
        d = numpy.array(data[field])
        pylab.plot( d, marker='.', label=field )
    pylab.legend(loc='upper right')
    pylab.ylabel('m/sec/sec')
    pylab.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.16)
    pylab.show()
    currentAxes = preview.gca()
    s_start = 0
    s_end = len(ts)-1
    n_samp = s_end - s_start
    currentAxes.set_xlim(s_start, s_end)

    # outer loop for repeating time window selection
    while True:
        
        while True:
            print('select sample range for fft analysis')
            preview.canvas.set_window_title('select sample range')
            try:
                s_start = input('start sample: ')
                s_end = input('end sample: ')
                currentAxes.set_xlim(s_start, s_end)
            except:
                break
            
        # process selected samples
        s_start = int(currentAxes.get_xlim()[0])
        s_end = int(currentAxes.get_xlim()[1])
        n_samp = s_end - s_start
        print('sample range: ', s_start, s_end)
        print('N samples: ', n_samp)
        
        # check for dropouts: (delta > 1)
        avg_rate = check_drops(data, 'ACC1', s_start, s_end)
        
        title = 'FFT input: {0:s} ACC1[{1:d}:{2:d}], {3:d} samples'.format(logfile, s_start, s_end, n_samp)
        currentAxes.set_xlabel('sample index : nsamples: {0:d}, avg rate: {1:.0f} Hz'.format(n_samp, avg_rate))
        preview.canvas.set_window_title(title)
        preview.savefig('acc1z.png')
            
        for msg in ['ACC1', 'GYR1', 'ACC2', 'GYR2']:
            if msg.startswith('ACC'):
                prefix = 'Acc'
                title = '{2} FFT [{0:d}:{1:d}]'.format(s_start, s_end, msg)
            else:
                prefix = 'Gyr'
                title = '{2} FFT [{0:d}:{1:d}]'.format(s_start, s_end, msg)
            
            # check for dropouts    
            data[msg+'.rate'] = check_drops(data, msg, s_start, s_end)
            plot_input(data, msg, prefix, s_start, s_end)
            
            fftwin = pylab.figure()
            fftwin.set_size_inches(12, 3, forward=True)
            f_res = float(data[msg+'.rate']) / n_samp
    
            max_fft = 0
            abs_fft = {}
            index = 0
            avg = {'X':0, 'Y':0, 'Z':0}
            for axis in ['X', 'Y', 'Z']:
                field = msg + '.' + prefix + axis
                d = data[field][s_start:s_end]
                if len(d) == 0:
                    continue
    
                d = numpy.array(d)
                freq  = numpy.fft.rfftfreq(len(d), 1.0 / data[msg+'.rate'])
                # remove mean
                avg[axis] = numpy.mean(d)
                d -= avg[axis]
                print('{1} DC component: {0:.3f}'.format(avg[axis], field))
                # transform
                d_fft = numpy.fft.rfft(d)
                abs_fft[axis] = numpy.abs(d_fft)
                # remember the max over all axes
                thismax = numpy.max(abs_fft[axis])
                if (max_fft < thismax):
                    max_fft = thismax
                index += 1
                
            for axis in ['X', 'Y', 'Z']:
                # scale to 0dB = max
                field = msg + '.' + prefix + axis
                db_fft = 20 * numpy.log10(abs_fft[axis] / max_fft)
                pylab.plot( freq, db_fft, label=field )
    
            fftwin.canvas.set_window_title(title)
            fftwin.gca().set_ylim(-90, 0)
            pylab.legend(loc='upper right')
            pylab.xlabel('Hz : res: ' + '{0:.3f}'.format(f_res))
            pylab.ylabel('dB X {0:.3f} Y {1:.3f} Z {2:.3f}\n'.format(avg['X'], avg['Y'], avg['Z']))
            pylab.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.16)
            fftwin.savefig(msg + '_fft.png')
        
        try:
            selection = raw_input('q to proceed to next file, anything else to select a new range: ')
            print(selection)
        except:
            continue

        if (selection == 'q'): 
            break

pylab.ion()
for filename in args.logs:
    fft(filename)

print('type ctrl-c to close windows and exit')
pylab.ioff()
pylab.show()

