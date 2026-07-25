#!/usr/bin/env python3

'''
estimate COMPASS_MOT_* parameters for throttle based compensation
'''
import math

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

def mag_error(p, data):
    cx,cy,cz,r = p
    errout = 0
    # cruse bounds mechanism
    if r < 0.3:
        errout = (0.3 - r) * 100
        r = 0.3
    if r > 2.0:
        errout = (r - 2.0) * 100
        r = 2.0
    corr = Vector3(cx,cy,cz)
    (baseline_field,baseline_throttle) = data[0]
    ret = []
    for d in data:
        (mag,throttle) = d
        cmag = mag + corr * math.pow(throttle,r)
        err = (cmag - baseline_field).length()
        err += errout
        ret.append(err)
    return ret

def fit_data(data):
    from scipy import optimize

    p0 = [0.0, 0.0, 0.0, 1.0]
    p1, ier = optimize.leastsq(mag_error, p0[:], args=(data))
    if not ier in [1, 2, 3, 4]:
        raise RuntimeError("Unable to find solution")
    return (Vector3(p1[0], p1[1], p1[2]), p1[3])

def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    data = []
    throttle = 0

    # now gather all the data
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        if m.get_type() == "MAG":
            mag = Vector3(m.MagX, m.MagY, m.MagZ)
            data.append((mag, throttle))
        #if m.get_type() == "RCOU":
        #    throttle = math.sqrt(m.C1/500.0)
        if m.get_type() == "CTUN":
            throttle = m.ThO

    print("Extracted %u data points" % len(data))

    (cmot,r) = fit_data(data)
    print("Fit    : %s  r: %s" % (cmot,r))

    x = range(len(data))
    errors = mag_error((cmot.x,cmot.y,cmot.z,r), data)

    import matplotlib.pyplot as plt
    plt.plot(x, errors, 'bo-')
    x1,x2,y1,y2 = plt.axis()
    plt.axis((x1,x2,0,y2))
    plt.ylabel('Error')
    plt.xlabel('Sample')
    plt.show()

for filename in args.logs:
    magfit(filename)
