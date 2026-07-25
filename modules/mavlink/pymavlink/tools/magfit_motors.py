#!/usr/bin/env python3

'''
fit best estimate of magnetometer offsets, trying to take into account motor interference
'''

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_argument("--noise", type=float, default=0, help="noise to add")
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
        (mag,motor) = d
        key = "%u:%u:%u" % (mag.x/20,mag.y/20,mag.z/20)
        if key in counts:
            counts[key] += 1
        else:
            counts[key] = 1
        if counts[key] < 3:
            ret.append(d)
    print(len(data), len(ret))
    return ret

def radius(d, offsets, motor_ofs):
    '''return radius give data point and offsets'''
    (mag, motor) = d
    return (mag + offsets + motor*motor_ofs).length()

def radius_cmp(a, b, offsets, motor_ofs):
    '''return radius give data point and offsets'''
    diff = radius(a, offsets, motor_ofs) - radius(b, offsets, motor_ofs)
    if diff > 0:
        return 1
    if diff < 0:
        return -1
    return 0

def sphere_error(p, data):
    x,y,z,mx,my,mz,r = p
    ofs = Vector3(x,y,z)
    motor_ofs = Vector3(mx,my,mz)
    ret = []
    for d in data:
        (mag,motor) = d
        err = r - radius((mag,motor), ofs, motor_ofs)
        ret.append(err)
    return ret

def fit_data(data):
    from scipy import optimize

    p0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    p1, ier = optimize.leastsq(sphere_error, p0[:], args=(data))
    if not ier in [1, 2, 3, 4]:
        raise RuntimeError("Unable to find solution")
    return (Vector3(p1[0], p1[1], p1[2]), Vector3(p1[3], p1[4], p1[5]), p1[6])

def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps)

    data = []

    last_t = 0
    offsets = Vector3(0,0,0)
    motor_ofs = Vector3(0,0,0)
    motor = 0.0

    # now gather all the data
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        if m.get_type() == "PARAM_VALUE" and m.param_id == "RC3_MIN":
            rc3_min = float(m.param_value)
        if m.get_type() == "SENSOR_OFFSETS":
            # update current offsets
            offsets = Vector3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)
        if m.get_type() == "SERVO_OUTPUT_RAW":
            motor_pwm = m.servo1_raw + m.servo2_raw + m.servo3_raw + m.servo4_raw
            motor_pwm *= 0.25
            rc3_min = mlog.param('RC3_MIN', 1100)
            rc3_max = mlog.param('RC3_MAX', 1900)
            motor = (motor_pwm - rc3_min) / (rc3_max - rc3_min)
            if motor > 1.0:
                motor = 1.0
            if motor < 0.0:
                motor = 0.0
        if m.get_type() == "RAW_IMU":
            mag = Vector3(m.xmag, m.ymag, m.zmag)
            # add data point after subtracting the current offsets
            data.append((mag - offsets + noise(), motor))

    print("Extracted %u data points" % len(data))
    print("Current offsets: %s" % offsets)

    data = select_data(data)

    # do an initial fit with all data
    (offsets, motor_ofs, field_strength) = fit_data(data)

    for count in range(3):
        # sort the data by the radius
        data.sort(lambda a,b : radius_cmp(a,b,offsets,motor_ofs))

        print("Fit %u    : %s  %s field_strength=%6.1f to %6.1f" % (
            count, offsets, motor_ofs,
            radius(data[0], offsets, motor_ofs), radius(data[-1], offsets, motor_ofs)))

        # discard outliers, keep the middle 3/4
        data = data[len(data)/8:-len(data)/8]

        # fit again
        (offsets, motor_ofs, field_strength) = fit_data(data)

    print("Final    : %s  %s field_strength=%6.1f to %6.1f" % (
        offsets, motor_ofs,
        radius(data[0], offsets, motor_ofs), radius(data[-1], offsets, motor_ofs)))
    print("mavgraph.py '%s' 'mag_field(RAW_IMU)' 'mag_field_motors(RAW_IMU,SENSOR_OFFSETS,(%f,%f,%f),SERVO_OUTPUT_RAW,(%f,%f,%f))'" % (
        filename,
        offsets.x,offsets.y,offsets.z,
        motor_ofs.x, motor_ofs.y, motor_ofs.z))

total = 0.0
for filename in args.logs:
    magfit(filename)
