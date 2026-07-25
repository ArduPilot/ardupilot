#!/usr/bin/env python3

'''
fit best estimate of magnetometer offsets
'''
import sys, time, os, math

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--noise", type=float, default=0, help="noise to add")
parser.add_argument("--mag2", action='store_true', help="use 2nd mag from DF log")
parser.add_argument("--radius", default=500.0, type=float, help="target radius")
parser.add_argument("--plot", action='store_true', help="plot points in 3D")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3
from pymavlink.rotmat import Matrix3


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
    print((len(data), len(ret)))
    return ret

def constrain(v, min, max):
    if v < min:
        return min
    if v > max:
        return max
    return v

def correct(mag, offsets, diag, offdiag):
    '''correct a mag sample'''
    diag.x = 1.0
    mat = Matrix3(
        Vector3(diag.x,    offdiag.x,  offdiag.y),
        Vector3(offdiag.x,    diag.y,  offdiag.z),
        Vector3(offdiag.y, offdiag.z,  diag.z))
    mag = mag + offsets
    mag = mat * mag
    return mag

def radius(mag, offsets, diag, offdiag):
    '''return radius give data point and offsets'''
    mag = correct(mag, offsets, diag, offdiag)
    return mag.length()

def radius_cmp(a, b, offsets, diag, offdiag):
    '''return +1 or -1 for for sorting'''
    diff = radius(a, offsets, diag, offdiag) - radius(b, offsets, diag, offdiag)
    if diff > 0:
        return 1
    if diff < 0:
        return -1
    return 0

def sphere_error(p, data):
    x,y,z,r,dx,dy,dz,odx,ody,odz = p
    if args.radius is not None:
        r = args.radius
    ofs = Vector3(x,y,z)
    diag = Vector3(dx, dy, dz)
    offdiag = Vector3(odx, ody, odz)
        
    ret = []
    for d in data:
        mag = correct(d, ofs, diag, offdiag)
        err = r - mag.length()
        ret.append(err)
    return ret

def fit_data(data):
    from scipy import optimize

    p0 = [0.0, 0.0, 0.0, # offsets
          500.0, # radius
          1.0, 1.0, 1.0, # diagonals
          0.0, 0.0, 0.0  # offdiagonals
          ]
    if args.radius is not None:
        p0[3] = args.radius
    p1, ier = optimize.leastsq(sphere_error, p0[:], args=(data))
    if not ier in [1, 2, 3, 4]:
        print(p1)
        raise RuntimeError("Unable to find solution: %u" % ier)
    if args.radius is not None:
        r = args.radius
    else:
        r = p1[3]
    return (Vector3(p1[0], p1[1], p1[2]),
            r,
            Vector3(p1[4], p1[5], p1[6]),
            Vector3(p1[7], p1[8], p1[9]))

def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps)

    data = []

    last_t = 0
    offsets = None

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
            if offsets is not None:
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

    # find average values
    avg = Vector3()
    count = 0
    for d in data:
        avg += d
        count += 1
    avg /= count

    # subtract average
    data = []
    for d in orig_data:
        data.append(d - avg)
    print("Average %s" % avg)

    #data = select_data(data)

    diag = Vector3(1,1,1)
    offdiag = Vector3(0,0,0)

    # remove initial outliers
    if False:
        data.sort(lambda a,b : radius_cmp(a,b,offsets,diag,offdiag))
        data = data[len(data)/16:-len(data)/16]

    # do an initial fit
    (offsets, field_strength, diag, offdiag) = fit_data(data)

    for count in range(3):
        # sort the data by the radius
        data.sort(lambda a,b : radius_cmp(a,b,offsets,diag,offdiag))

        print("Fit %u    : %s %s %s  field_strength=%6.1f to %6.1f" % (
            count, offsets, diag, offdiag,
            radius(data[0], offsets,diag,offdiag), radius(data[-1], offsets,diag,offdiag)))

        # discard outliers, keep the middle
        data = data[len(data)/32:-len(data)/32]

        # fit again
        (offsets, field_strength, diag, offdiag) = fit_data(data)

    print("Final    : %s %s %s field_strength=%6.1f to %6.1f" % (
        offsets, diag, offdiag,
        radius(data[0], offsets, diag, offdiag), radius(data[-1], offsets, diag, offdiag)))

    offsets -= avg
    print("With average     : %s" % offsets)

    if args.plot:
        data2 = [correct(d,offsets,diag,offdiag) for d in orig_data]
        plot_data(orig_data, data2)

def plot_data(orig_data, data):
    '''plot data in 3D'''
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt

    fig = plt.figure()
    for dd, c, p in [(orig_data, 'r', 1), (data, 'b', 2)]:
        ax = fig.add_subplot(1, 2, p, projection='3d')

        xs = [ d.x for d in dd ]
        ys = [ d.y for d in dd ]
        zs = [ d.z for d in dd ]
        ax.scatter(xs, ys, zs, c=c, marker='o')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    plt.show()

total = 0.0
for filename in args.logs:
    magfit(filename)
