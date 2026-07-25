#!/usr/bin/env python3
'''
find signs of aliasing on IMU3
'''

from pymavlink import mavutil
from pymavlink import mavextra
import math, sys, multiprocessing, random, time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("infile", default=None, nargs='+', help="input file")
parser.add_argument("--parallel", default=4, type=int)
parser.add_argument("--threshold", type=float, default=3.0)
args = parser.parse_args()

def process_log(fname):
    '''process one log'''
    mlog = mavutil.mavlink_connection(fname)

    count_imu1 = 0
    count_imu3 = 0
    err_count = {}
    lp1 = {}
    lp3 = {}
    last_err = {}
    max_err = 0.0
    max_err_axis = None

    axes = ['AccX', 'AccY', 'AccZ']
    for a in axes:
        err_count[a] = 0
        last_err[a] = 0.0

    alpha = 0.9
    msg_types = set(['IMU', 'IMU3'])

    while True:
        m = mlog.recv_match(type=msg_types)
        if m is None:
            break

        mtype = m.get_type()
        if mtype == 'IMU':
            if count_imu1 == 0:
                for a in axes:
                    lp1[a] = getattr(m,a)
            for a in axes:
                lp1[a] = alpha * lp1[a] + (1.0-alpha) * getattr(m,a)
            count_imu1 += 1

        if mtype == 'IMU3':
            if count_imu3 == 0:
                for a in axes:
                    lp3[a] = getattr(m,a)
            for a in axes:
                lp3[a] = alpha * lp3[a] + (1.0-alpha) * getattr(m,a)
            count_imu3 += 1

        if count_imu1 > 100 and count_imu3 > 100:
            for a in axes:
                err = lp1[a] - lp3[a]
                # look for errors above threshold where it is not changing sign
                if abs(err) > args.threshold and err * last_err[a] > 0 and abs(lp1[a]) < 60.0:
                    err_count[a] += 1
                else:
                    err_count[a] = 0
                last_err[a] = err
                if err_count[a] > 30:
                    if abs(err) > max_err:
                        max_err = abs(err)
                        max_err_axis = a

        if count_imu1 > 100 and count_imu3 == 0:
            # we don't have both IMUs logged
            break

    if max_err_axis is not None:
        print("%s aliasing on %s %.1f" % (fname, max_err_axis, max_err))
        try:
            open(fname + ".aliasing", "w").write("aliasing on %s %.1f : %s\n" % (max_err_axis, max_err, fname))
        except Exception:
            pass

files = args.infile
nfiles = len(args.infile)

procs = []
done = set()

while len(done) < nfiles:
    r = random.randint(0, nfiles-1)
    fname = files[r]
    if fname in done:
        continue
    done.add(fname)
    print("Processing %s (%u/%u)" % (fname, len(done), nfiles))
    p = multiprocessing.Process(target=process_log, args=(fname,))
    p.start()
    procs.append(p)
    while len(procs) >= args.parallel:
        for p in procs:
            if p.exitcode is not None:
                p.join()
                procs.remove(p)
        time.sleep(0.1)
