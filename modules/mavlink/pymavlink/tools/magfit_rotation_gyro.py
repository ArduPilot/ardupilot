#!/usr/bin/env python3

'''
fit best estimate of magnetometer rotation to gyro data
'''

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--verbose", action='store_true', help="verbose output")
parser.add_argument("--min-rotation", default=5.0, type=float, help="min rotation to add point")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3, Matrix3
from math import radians


class Rotation(object):
    def __init__(self, name, roll, pitch, yaw):
        self.name = name
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.r = Matrix3()
        self.r.from_euler(self.roll, self.pitch, self.yaw)

    def is_90_degrees(self):
        return (self.roll % 90 == 0) and (self.pitch % 90 == 0) and (self.yaw % 90 == 0)

    def __str__(self):
        return self.name

# the rotations used in APM
rotations = [
    Rotation("ROTATION_NONE",                      0,   0,   0),
    Rotation("ROTATION_YAW_45",                    0,   0,  45),
    Rotation("ROTATION_YAW_90",                    0,   0,  90),
    Rotation("ROTATION_YAW_135",                   0,   0, 135),
    Rotation("ROTATION_YAW_180",                   0,   0, 180),
    Rotation("ROTATION_YAW_225",                   0,   0, 225),
    Rotation("ROTATION_YAW_270",                   0,   0, 270),
    Rotation("ROTATION_YAW_315",                   0,   0, 315),
    Rotation("ROTATION_ROLL_180",                180,   0,   0),
    Rotation("ROTATION_ROLL_180_YAW_45",         180,   0,  45),
    Rotation("ROTATION_ROLL_180_YAW_90",         180,   0,  90),
    Rotation("ROTATION_ROLL_180_YAW_135",        180,   0, 135),
    Rotation("ROTATION_PITCH_180",                 0, 180,   0),
    Rotation("ROTATION_ROLL_180_YAW_225",        180,   0, 225),
    Rotation("ROTATION_ROLL_180_YAW_270",        180,   0, 270),
    Rotation("ROTATION_ROLL_180_YAW_315",        180,   0, 315),
    Rotation("ROTATION_ROLL_90",                  90,   0,   0),
    Rotation("ROTATION_ROLL_90_YAW_45",           90,   0,  45),
    Rotation("ROTATION_ROLL_90_YAW_90",           90,   0,  90),
    Rotation("ROTATION_ROLL_90_YAW_135",          90,   0, 135),
    Rotation("ROTATION_ROLL_270",                270,   0,   0),
    Rotation("ROTATION_ROLL_270_YAW_45",         270,   0,  45),
    Rotation("ROTATION_ROLL_270_YAW_90",         270,   0,  90),
    Rotation("ROTATION_ROLL_270_YAW_135",        270,   0, 135),
    Rotation("ROTATION_PITCH_90",                  0,  90,   0),
    Rotation("ROTATION_PITCH_270",                 0, 270,   0),
    Rotation("ROTATION_PITCH_180_YAW_90",          0, 180,  90),
    Rotation("ROTATION_PITCH_180_YAW_270",         0, 180, 270),
    Rotation("ROTATION_ROLL_90_PITCH_90",         90,  90,   0),
    Rotation("ROTATION_ROLL_180_PITCH_90",       180,  90,   0),
    Rotation("ROTATION_ROLL_270_PITCH_90",       270,  90,   0),
    Rotation("ROTATION_ROLL_90_PITCH_180",        90, 180,   0),
    Rotation("ROTATION_ROLL_270_PITCH_180",      270, 180,   0),
    Rotation("ROTATION_ROLL_90_PITCH_270",        90, 270,   0),
    Rotation("ROTATION_ROLL_180_PITCH_270",      180, 270,   0),
    Rotation("ROTATION_ROLL_270_PITCH_270",      270, 270,   0),
    Rotation("ROTATION_ROLL_90_PITCH_180_YAW_90", 90, 180,  90),
    Rotation("ROTATION_ROLL_90_YAW_270",          90,   0, 270)
    ]

def mag_fixup(mag, AHRS_ORIENTATION, COMPASS_ORIENT, COMPASS_EXTERNAL):
    '''fixup a mag vector back to original value using AHRS and Compass orientation parameters'''
    if COMPASS_EXTERNAL == 0 and AHRS_ORIENTATION != 0:
        # undo any board orientation
        mag = rotations[AHRS_ORIENTATION].r.transposed() * mag
    # undo any compass orientation
    if COMPASS_ORIENT != 0:
        mag = rotations[COMPASS_ORIENT].r.transposed() * mag
    return mag

def add_errors(mag, gyr, last_mag, deltat, total_error, rotations):
    for i in range(len(rotations)):
        if not rotations[i].is_90_degrees():
            continue
        r = rotations[i].r
        m = Matrix3()
        m.rotate(gyr * deltat)
        rmag1 = r * last_mag
        rmag2 = r * mag
        rmag3 = m.transposed() * rmag1
        err = rmag3 - rmag2
        total_error[i] += err.length()


def magfit(logfile):
    '''find best magnetometer rotation fit to a log file'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps)

    last_mag = None
    last_usec = 0
    count = 0
    total_error = [0]*len(rotations)

    AHRS_ORIENTATION = 0
    COMPASS_ORIENT = 0
    COMPASS_EXTERNAL = 0
    last_gyr = None

    # now gather all the data
    while True:
        m = mlog.recv_match()
        if m is None:
            break
        if m.get_type() in ["PARAM_VALUE", "PARM"]:
            if m.get_type() == "PARM":
                name = str(m.Name)
                value = int(m.Value)
            else:
                name = str(m.param_id)
                value = int(m.param_value)
            if name == "AHRS_ORIENTATION":
                AHRS_ORIENTATION = value
            if name == 'COMPASS_ORIENT':
                COMPASS_ORIENT = value
            if name == 'COMPASS_EXTERNAL':
                COMPASS_EXTERNAL = value
        if m.get_type() in ["RAW_IMU", "MAG","IMU"]:
            if m.get_type() == "RAW_IMU":
                mag = Vector3(m.xmag, m.ymag, m.zmag)
                gyr = Vector3(m.xgyro, m.ygyro, m.zgyro) * 0.001
                usec = m.time_usec
            elif m.get_type() == "IMU":
                last_gyr = Vector3(m.GyrX,m.GyrY,m.GyrZ)
                continue
            elif last_gyr is not None:
                mag = Vector3(m.MagX,m.MagY,m.MagZ)
                gyr = last_gyr
                usec = m.TimeMS * 1000
            mag = mag_fixup(mag, AHRS_ORIENTATION, COMPASS_ORIENT, COMPASS_EXTERNAL)
            if last_mag is not None and gyr.length() > radians(args.min_rotation):
                add_errors(mag, gyr, last_mag, (usec - last_usec)*1.0e-6, total_error, rotations)
                count += 1
            last_mag = mag
            last_usec = usec

    best_i = 0
    best_err = total_error[0]
    for i in range(len(rotations)):
        r = rotations[i]
        if not r.is_90_degrees():
            continue
        if args.verbose:
            print("%s err=%.2f" % (r, total_error[i]/count))
        if total_error[i] < best_err:
            best_i = i
            best_err = total_error[i]
    r = rotations[best_i]
    print("Current rotation is AHRS_ORIENTATION=%s COMPASS_ORIENT=%s COMPASS_EXTERNAL=%u" % (
        rotations[AHRS_ORIENTATION],
        rotations[COMPASS_ORIENT],
        COMPASS_EXTERNAL))
    print("Best rotation is %s err=%.2f from %u points" % (r, best_err/count, count))
    print("Please set AHRS_ORIENTATION=%s COMPASS_ORIENT=%s COMPASS_EXTERNAL=1" % (
        rotations[AHRS_ORIENTATION],
        r))

for filename in args.logs:
    magfit(filename)
