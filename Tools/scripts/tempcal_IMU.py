#!/usr/bin/env python
'''
Create temperature calibration parameters for IMUs based on log data.
'''

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--outfile", default="tcal.parm")
parser.add_argument("--no-graph", action='store_true', default=False)
parser.add_argument("log", metavar="LOG")

args = parser.parse_args()

import sys
import math
import re
from pymavlink import mavutil
import numpy as np
import matplotlib.pyplot as pyplot
from scipy import signal
from pymavlink.rotmat import Vector3, Matrix3

POLY_ORDER = 3

# we scale the parameters so the values work nicely in
# parameter editors and parameter files that don't
# use exponential notation
SCALE_FACTOR = 1.0e6

def moving_average(data, w):
    '''apply a moving average filter over a window of width w'''
    ret = np.cumsum(data)
    ret[w:] = ret[w:] - ret[:-w]
    return ret[w - 1:] / w

def constrain(value, minv, maxv):
    """Constrain a value to a range."""
    if value < minv:
        value = minv
    if value > maxv:
        value = maxv
    return value

def correction_parm(enable, tmin, tmax, coeff, temperature, cal_temp, axis):
    '''calculate correction from temperature calibration from log data using parameters'''
    if enable != 1.0:
        return 0.0
    tmid = 0.5 * (tmax + tmin)
    if tmid <= 0:
        return 0.0
    if cal_temp <= -80:
        return 0.0
    if not axis in coeff:
        return 0.0
    temperature = constrain(temperature, tmin, tmax)
    cal_temp = constrain(cal_temp, tmin, tmax)
    poly = np.poly1d(coeff[axis])
    ret = poly(cal_temp-tmid) - poly(temperature-tmid)
    return ret

def IMUfit(logfile):
    '''find IMU calibration parameters from a log file'''
    print("Processing log %s" % logfile)
    mlog = mavutil.mavlink_connection(logfile)

    accel = {}
    gyro = {}
    acoef = {}
    gcoef = {}
    enable = [0]*3
    tmin = [-100]*3
    tmax = [-100]*3
    gtcal = {}
    atcal = {}
    orientation = 0
    axes = ['X','Y','Z']
    axesT = ['X','Y','Z','T','time']

    while True:
        msg = mlog.recv_match(type=['IMU','PARM'])
        if msg is None:
            break

        if msg.get_type() == 'PARM':
            # build up the old coefficients so we can remove the impact of
            # existing coefficients from the data
            m = re.match("^INS_TCAL(\d)_([AG]..)([1-3])_([XYZ])$", msg.Name)
            if m:
                imu = int(m.group(1))-1
                stype = m.group(2)
                p = int(m.group(3))
                axis = m.group(4)
                if stype == 'ACC':
                    if not imu in acoef:
                        acoef[imu] = {}
                    if not axis in acoef[imu]:
                        acoef[imu][axis] = [0.0]*4
                    acoef[imu][axis][POLY_ORDER-p] = msg.Value/SCALE_FACTOR
                if stype == 'GYR':
                    if not imu in gcoef:
                        gcoef[imu] = {}
                    if not axis in gcoef[imu]:
                        gcoef[imu][axis] = [0.0]*4
                    gcoef[imu][axis][POLY_ORDER-p] = msg.Value/SCALE_FACTOR
            m = re.match("^INS_TCAL(\d)_ENABLE$", msg.Name)
            if m:
                imu = int(m.group(1))-1
                if msg.Value == 1 and enable[imu] == 2:
                    print("TCAL[%u] enabled" % imu)
                    break
                enable[imu] = msg.Value
            m = re.match("^INS_TCAL(\d)_TMIN$", msg.Name)
            if m:
                imu = int(m.group(1))-1
                tmin[imu] = msg.Value
            m = re.match("^INS_TCAL(\d)_TMAX", msg.Name)
            if m:
                imu = int(m.group(1))-1
                tmax[imu] = msg.Value
            m = re.match("^INS_GYR_CALTEMP(\d)", msg.Name)
            if m:
                imu = int(m.group(1))-1
                gtcal[imu] = msg.Value
            m = re.match("^INS_ACC_CALTEMP(\d)", msg.Name)
            if m:
                imu = int(m.group(1))-1
                atcal[imu] = msg.Value
            if msg.Name == 'AHRS_ORIENTATION':
                orientation = int(msg.Value)
                print("Using orientation %d" % orientation)

        if msg.get_type() != 'IMU':
            continue

        imu = msg.I

        if imu not in accel:
            accel[imu] = {}
            gyro[imu] = {}
            for axis in axesT:
                accel[imu][axis] = np.zeros(0,dtype=float)
                gyro[imu][axis] = np.zeros(0,dtype=float)

        T = msg.T
        accel[imu]['T'] = np.append(accel[imu]['T'], T)
        gyro[imu]['T'] = np.append(gyro[imu]['T'], T)
        accel[imu]['time'] = np.append(accel[imu]['time'], msg.TimeUS*1.0e-6)
        gyro[imu]['time'] = np.append(gyro[imu]['time'], msg.TimeUS*1.0e-6)

        acc = Vector3(msg.AccX, msg.AccY, msg.AccZ)
        gyr = Vector3(msg.GyrX, msg.GyrY, msg.GyrZ)

        # invert the board orientation rotation. Corrections are in sensor frame
        if orientation != 0:
            acc = acc.rotate_by_inverse_id(orientation)
            gyr = gyr.rotate_by_inverse_id(orientation)
        if acc is None or gyr is None:
            print("Invalid AHRS_ORIENTATION %u" % orientation)
            sys.exit(1)

        for axis in axes:
            value = getattr(acc, axis.lower())
            if enable[imu] == 1:
                value -= correction_parm(enable[imu], tmin[imu], tmax[imu], acoef[imu],
                                         T, atcal[imu], axis)
            accel[imu][axis] = np.append(accel[imu][axis], value)

            value = getattr(gyr, axis.lower())
            if enable[imu] == 1:
                value -= correction_parm(enable[imu], tmin[imu], tmax[imu], gcoef[imu],
                                         T, gtcal[imu], axis)
            gyro[imu][axis] = np.append(gyro[imu][axis], value)

    # apply moving average filter with 2s width
    for imu in accel:
        nseconds = accel[imu]['time'][-1] - accel[imu]['time'][0]
        nsamples = len(accel[imu]['time'])
        window = int(nsamples / nseconds) * 2

        for axis in axesT:
            accel[imu][axis] = moving_average(accel[imu][axis], window)
            gyro[imu][axis] = moving_average(gyro[imu][axis], window)

    trel = {}

    calfile = open(args.outfile, "w")

    for imu in accel:
        tmin = np.amin(accel[imu]['T'])
        tmax = np.amax(accel[imu]['T'])
        tref = (tmin+tmax)*0.5

        acoef[imu] = {}
        gcoef[imu] = {}

        trel[imu] = accel[imu]['T'] - tref

        for axis in axes:
            acoef[imu][axis] = np.polyfit(trel[imu], accel[imu][axis] - np.median(accel[imu][axis]), POLY_ORDER)
            gcoef[imu][axis] = np.polyfit(trel[imu], gyro[imu][axis], POLY_ORDER)

        params = ''
        params += 'INS_TCAL%u_ENABLE 1\n' % (imu+1)
        params += 'INS_TCAL%u_TMIN %.1f\n' % (imu+1, tmin)
        params += 'INS_TCAL%u_TMAX %.1f\n' % (imu+1, tmax)
        # note that we don't save the first term of the polynomial as that is a
        # constant offset which is already handled by the accel/gyro constant
        # offsets. We only same the temperature dependent part of the
        # calibration
        for p in range(POLY_ORDER):
            for axis in axes:
                params += 'INS_TCAL%u_ACC%u_%s %.9f\n' % (imu+1, p+1, axis, acoef[imu][axis][POLY_ORDER-(p+1)]*SCALE_FACTOR)
        for p in range(POLY_ORDER):
            for axis in axes:
                params += 'INS_TCAL%u_GYR%u_%s %.9f\n' % (imu+1, p+1, axis, gcoef[imu][axis][POLY_ORDER-(p+1)]*SCALE_FACTOR)
        print(params)
        calfile.write(params)

    calfile.close()
    print("Calibration written to %s" % args.outfile)

    if args.no_graph:
        return
    fig, axs = pyplot.subplots(len(gyro), 1, sharex=True)

    if len(gyro) == 1:
        axs = [axs]

    for imu in gyro:
        scale = math.degrees(1)
        for axis in axes:
            axs[imu].plot(gyro[imu]['time'], gyro[imu][axis]*scale, label='Uncorrected %s' % axis)
        for axis in axes:
            poly = np.poly1d(gcoef[imu][axis])
            correction = poly(trel[imu])
            axs[imu].plot(gyro[imu]['time'], (gyro[imu][axis] - correction)*scale, label='Corrected %s' % axis)
        ax2 = axs[imu].twinx()
        ax2.plot(gyro[imu]['time'], gyro[imu]['T'], label='Temperature(C)', color='black')
        ax2.legend(loc='upper right')
        axs[imu].legend(loc='upper left')
        axs[imu].set_title('IMU[%u] Gyro (deg/s)' % imu)

    fig, axs = pyplot.subplots(len(accel), 1, sharex=True)
    if len(accel) == 1:
        axs = [axs]

    for imu in accel:
        mean = {}
        for axis in axes:
            mean[axis] = np.mean(accel[imu][axis])
            axs[imu].plot(accel[imu]['time'], accel[imu][axis] - mean[axis], label='Uncorrected %s' % axis)
        for axis in axes:
            poly = np.poly1d(acoef[imu][axis])
            correction = poly(trel[imu])
            axs[imu].plot(accel[imu]['time'], (accel[imu][axis]-mean[axis]) - correction, label='Corrected %s' % axis)
        ax2 = axs[imu].twinx()
        ax2.plot(accel[imu]['time'], accel[imu]['T'], label='Temperature(C)', color='black')
        ax2.legend(loc='upper right')
        axs[imu].legend(loc='upper left')
        axs[imu].set_title('IMU[%u] Accel (m/s^2)' % imu)
        
    pyplot.show()



IMUfit(args.log)
sys.exit(1)

