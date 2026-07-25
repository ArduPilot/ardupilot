#!/usr/bin/env python3

'''
fit best estimate of magnetometer offsets
'''

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--noise", type=float, default=0, help="noise to add")
parser.add_argument("--mag2", action='store_true', help="use 2nd mag from DF log")
parser.add_argument("--radius", default=None, type=float, help="target radius")
parser.add_argument("--plot", action='store_true', help="plot points in 3D")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3


def noise():
    '''a noise vector'''
    from random import gauss
    v = Vector3(gauss(0, 1), gauss(0, 1), gauss(0, 1))
    v.normalize()
    return v * args.noise

def select_data(data):
    ret = []
    counts = {}
    for d in data:
        mag = d
        key = "%u:%u:%u" % (mag.x/20,mag.y/20,mag.z/20)
        if key in counts:
            counts[key] += 1
        else:
            counts[key] = 1
        if counts[key] < 3:
            ret.append(d)
    print(len(data), len(ret))
    return ret

def radius(mag, offsets):
    '''return radius give data point and offsets'''
    return (mag + offsets).length()

def radius_cmp(a, b, offsets):
    '''return +1 or -1 for for sorting'''
    diff = radius(a, offsets) - radius(b, offsets)
    if diff > 0:
        return 1
    if diff < 0:
        return -1
    return 0

def sphere_error(p, data):
    x,y,z,r = p
    if args.radius is not None:
        r = args.radius
    ofs = Vector3(x,y,z)
    ret = []
    for d in data:
        mag = d
        err = r - radius(mag, ofs)
        ret.append(err)
    return ret

def fit_data(data):
    from scipy import optimize

    p0 = [0.0, 0.0, 0.0, 0.0]
    p1, ier = optimize.leastsq(sphere_error, p0[:], args=(data))
    if not ier in [1, 2, 3, 4]:
        raise RuntimeError("Unable to find solution")
    if args.radius is not None:
        r = args.radius
    else:
        r = p1[3]
    return (Vector3(p1[0], p1[1], p1[2]), r)

def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps)

    data = []

    last_t = 0
    offsets = Vector3(0,0,0)

    # now gather all the data
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        if m.get_type() == "SENSOR_OFFSETS":
            # update current offsets
            offsets = Vector3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)
        if m.get_type() == "RAW_IMU":
            mag = Vector3(m.xmag, m.ymag, m.zmag)
            # add data point after subtracting the current offsets
            data.append(mag - offsets + noise())
        if m.get_type() == "MAG" and not args.mag2:
            offsets = Vector3(m.OfsX,m.OfsY,m.OfsZ)
            mag = Vector3(m.MagX,m.MagY,m.MagZ)
            data.append(mag - offsets + noise())
        if m.get_type() == "MAG2" and args.mag2:
            offsets = Vector3(m.OfsX,m.OfsY,m.OfsZ)
            mag = Vector3(m.MagX,m.MagY,m.MagZ)
            data.append(mag - offsets + noise())

    print("Extracted %u data points" % len(data))
    print("Current offsets: %s" % offsets)

    orig_data = data

    data = select_data(data)

    # remove initial outliers
    data.sort(lambda a,b : radius_cmp(a,b,offsets))
    data = data[len(data)/16:-len(data)/16]

    # do an initial fit
    (offsets, field_strength) = fit_data(data)

    for count in range(3):
        # sort the data by the radius
        data.sort(lambda a,b : radius_cmp(a,b,offsets))

        print("Fit %u    : %s  field_strength=%6.1f to %6.1f" % (
            count, offsets,
            radius(data[0], offsets), radius(data[-1], offsets)))

        # discard outliers, keep the middle 3/4
        data = data[len(data)/8:-len(data)/8]

        # fit again
        (offsets, field_strength) = fit_data(data)

    print("Final    : %s  field_strength=%6.1f to %6.1f" % (
        offsets,
        radius(data[0], offsets), radius(data[-1], offsets)))

    if args.plot:
        plot_data(orig_data, data)

def plot_data(orig_data, data):
    '''plot data in 3D'''
    import matplotlib.pyplot as plt

    for dd, c in [(orig_data, 'r'), (data, 'b')]:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        xs = [ d.x for d in dd ]
        ys = [ d.y for d in dd ]
        zs = [ d.z for d in dd ]
        ax.scatter(xs, ys, zs, c=c, marker='o')

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
    plt.show()

total = 0.0
for filename in args.logs:
    magfit(filename)
