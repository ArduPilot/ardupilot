#!/usr/bin/env python3

'''
calculate GPS lag from DF log

The DF log file needs to be generated with ATT, GPS and IMU bits:
-  On copter (bit0, bit2 and bit18) set on the LOG_BITMASK parameters
-  On rover  (bit0, bit2 and bit7) set on the LOG_BITMASK parameters
-  On plane  (bit0, bit2 and bit7) set on the LOG_BITMASK parameters
Make sure IMU_RAW (bit 19) is not set

For this to work, the vehicle must move at speeds above the --minspeed parameter (defaults to 6m/s)
The code really only works when there is significant acceleration as well.
You'll need to fly quite aggressively on a copter to get a result.

'''

import os

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--plot", action='store_true', default=False, help="plot errors")
parser.add_argument("--minspeed", type=float, default=6, help="minimum speed")
parser.add_argument("--gps", type=int, default=0, help="GPS number")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.mavextra import *
from pymavlink.rotmat import Vector3

'''
Support having a $HOME/.pymavlink/mavextra.py for extra graphing functions
'''
home = os.getenv('HOME')
if home is not None:
    extra = os.path.join(home, '.pymavlink', 'mavextra.py')
    if os.path.exists(extra):
        import imp
        mavuser = imp.load_source('pymavlink.mavuser', extra)
        from pymavlink.mavuser import *


def velocity_error(timestamps, vel, gaccel, accel_indexes, imu_dt, shift=0):
    '''return summed velocity error'''
    sum = 0
    count = 0
    for i in range(0, len(vel)-1):
        dv = vel[i+1] - vel[i]
        da = Vector3()
        for idx in range(1+accel_indexes[i]-shift, 1+accel_indexes[i+1]-shift):
            da += gaccel[idx]
        dt1 = timestamps[i+1] - timestamps[i]
        dt2 = (accel_indexes[i+1] - accel_indexes[i]) * imu_dt
        da *= imu_dt
        da *= dt1/dt2
        #print(accel_indexes[i+1] - accel_indexes[i])
        ex = abs(dv.x - da.x)
        ey = abs(dv.y - da.y)
        sum += 0.5*(ex+ey)
        count += 1
    if count == 0:
        return None
    return sum/count

def gps_lag(logfile):
    '''work out gps velocity lag times for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    timestamps = []
    vel = []
    gaccel = []
    accel_indexes = []
    ATT = None
    IMU = None
    GPS = None

    dtsum = 0
    dtcount = 0

    while True:
        m = mlog.recv_match(type=['GPS','IMU','ATT','GPA'])
        if m is None:
            break
        t = m.get_type()
        if t.startswith('GPS') and m.Status >= 3 and m.Spd>args.minspeed:
            if m.I != args.gps:
                continue
            GPS = m;
        elif t.startswith('GPA') and GPS is not None and GPS.TimeUS == m.TimeUS:
            if m.I != args.gps:
                continue
            v = Vector3(GPS.Spd*cos(radians(GPS.GCrs)), GPS.Spd*sin(radians(GPS.GCrs)), GPS.VZ)
            vel.append(v)
            timestamps.append(m.SMS*0.001)
            accel_indexes.append(max(len(gaccel)-1,0))
        elif t == 'ATT':
            ATT = m
        elif t == 'IMU':
            if m.I != 0:
                continue
            if ATT is not None:
                ga = earth_accel_df(m, ATT)
                ga.z += 9.80665
                gaccel.append(ga)
                if IMU is not None:
                    dt = (m.TimeUS - IMU.TimeUS) * 1.0e-6
                    dtsum += dt
                    dtcount += 1
                IMU = m

    imu_dt = dtsum / dtcount

    print("Loaded %u samples imu_dt=%.3f" % (len(vel), imu_dt))
    besti = -1
    besterr = 0
    delays = []
    errors = []
    for i in range(0,100):
        err = velocity_error(timestamps, vel, gaccel, accel_indexes, imu_dt, shift=i)
        if err is None:
            break
        errors.append(err)
        delays.append(i*imu_dt)
        if besti == -1 or err < besterr:
            besti = i
            besterr = err
    print("Best %u (%.3fs) %f" % (besti, besti*imu_dt, besterr))

    if args.plot:
        import matplotlib.pyplot as plt
        plt.plot(delays, errors, 'bo-')
        x1,x2,y1,y2 = plt.axis()
        plt.axis((x1,x2,0,y2))
        plt.ylabel('Error')
        plt.xlabel('Delay(s)')
        plt.show()


for filename in args.logs:
    gps_lag(filename)
