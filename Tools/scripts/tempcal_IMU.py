#!/usr/bin/env python3

# flake8: noqa

'''
Create temperature calibration parameters for IMUs based on log data.
'''

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--outfile", default="tcal.parm", help='set output file')
parser.add_argument("--no-graph", action='store_true', default=False, help='disable graph display')
parser.add_argument("--log-parm", action='store_true', default=False, help='show corrections using coefficients from log file')
parser.add_argument("--online", action='store_true', default=False, help='use online polynomial fitting')
parser.add_argument("--tclr", action='store_true', default=False, help='use TCLR messages from log instead of IMU messages')
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

# fit an order 3 polynomial
POLY_ORDER = 3

# we use a fixed reference temperature of 35C. This has the advantage that
# we don't need to know the final temperature when doing an online calibration
# which allows us to have a calibration timeout
TEMP_REF = 35.0

# we scale the parameters so the values work nicely in
# parameter editors and parameter files that don't
# use exponential notation
SCALE_FACTOR = 1.0e6

AXES = ['X','Y','Z']
AXEST = ['X','Y','Z','T','time']

class Coefficients:
    '''class representing a set of coefficients'''
    def __init__(self):
        self.acoef = {}
        self.gcoef = {}
        self.enable = [0]*3
        self.tmin = [-100]*3
        self.tmax = [-100]*3
        self.gtcal = {}
        self.atcal = {}
        self.gofs = {}
        self.aofs = {}

    def set_accel_poly(self, imu, axis, values):
        if imu not in self.acoef:
            self.acoef[imu] = {}
        self.acoef[imu][axis] = values

    def set_gyro_poly(self, imu, axis, values):
        if imu not in self.gcoef:
            self.gcoef[imu] = {}
        self.gcoef[imu][axis] = values
        
    def set_acoeff(self, imu, axis, order, value):
        if imu not in self.acoef:
            self.acoef[imu] = {}
        if axis not in self.acoef[imu]:
            self.acoef[imu][axis] = [0]*4
        self.acoef[imu][axis][POLY_ORDER-order] = value

    def set_gcoeff(self, imu, axis, order, value):
        if imu not in self.gcoef:
            self.gcoef[imu] = {}
        if axis not in self.gcoef[imu]:
            self.gcoef[imu][axis] = [0]*4
        self.gcoef[imu][axis][POLY_ORDER-order] = value

    def set_aoffset(self, imu, axis, value):
        if imu not in self.aofs:
            self.aofs[imu] = {}
        self.aofs[imu][axis] = value

    def set_goffset(self, imu, axis, value):
        if imu not in self.gofs:
            self.gofs[imu] = {}
        self.gofs[imu][axis] = value
        
    def set_tmin(self, imu, tmin):
        self.tmin[imu] = tmin

    def set_tmax(self, imu, tmax):
        self.tmax[imu] = tmax

    def set_gyro_tcal(self, imu, value):
        self.gtcal[imu] = value

    def set_accel_tcal(self, imu, value):
        self.atcal[imu] = value

    def set_enable(self, imu, value):
        self.enable[imu] = value

    def correction(self, coeff, imu, temperature, axis, cal_temp):
        '''calculate correction from temperature calibration from log data using parameters'''
        if self.enable[imu] != 1.0:
            return 0.0
        if cal_temp < -80:
            return 0.0
        if axis not in coeff:
            return 0.0
        temperature = constrain(temperature, self.tmin[imu], self.tmax[imu])
        cal_temp = constrain(cal_temp, self.tmin[imu], self.tmax[imu])
        poly = np.poly1d(coeff[axis])
        return poly(cal_temp - TEMP_REF) - poly(temperature - TEMP_REF)

    def correction_accel(self, imu, temperature):
        '''calculate accel correction from temperature calibration from
        log data using parameters'''
        cal_temp = self.atcal.get(imu, TEMP_REF)
        return Vector3(self.correction(self.acoef[imu], imu, temperature, 'X', cal_temp),
                       self.correction(self.acoef[imu], imu, temperature, 'Y', cal_temp),
                       self.correction(self.acoef[imu], imu, temperature, 'Z', cal_temp))


    def correction_gyro(self, imu, temperature):
        '''calculate gyro correction from temperature calibration from
        log data using parameters'''
        cal_temp = self.gtcal.get(imu, TEMP_REF)
        return Vector3(self.correction(self.gcoef[imu], imu, temperature, 'X', cal_temp),
                       self.correction(self.gcoef[imu], imu, temperature, 'Y', cal_temp),
                       self.correction(self.gcoef[imu], imu, temperature, 'Z', cal_temp))

    def param_string(self, imu):
        params = ''
        params += 'INS_TCAL%u_ENABLE 1\n' % (imu+1)
        params += 'INS_TCAL%u_TMIN %.1f\n' % (imu+1, self.tmin[imu])
        params += 'INS_TCAL%u_TMAX %.1f\n' % (imu+1, self.tmax[imu])
        # note that we don't save the first term of the polynomial as that is a
        # constant offset which is already handled by the accel/gyro constant
        # offsets. We only same the temperature dependent part of the
        # calibration
        for p in range(POLY_ORDER):
            for axis in AXES:
                params += 'INS_TCAL%u_ACC%u_%s %.9f\n' % (imu+1, p+1, axis, self.acoef[imu][axis][POLY_ORDER-(p+1)]*SCALE_FACTOR)
        for p in range(POLY_ORDER):
            for axis in AXES:
                params += 'INS_TCAL%u_GYR%u_%s %.9f\n' % (imu+1, p+1, axis, self.gcoef[imu][axis][POLY_ORDER-(p+1)]*SCALE_FACTOR)
        return params
    

class OnlineIMUfit:
    '''implement the online learning used in ArduPilot'''
    def __init__(self):
        pass

    def update(self, x, y):
        temp = 1.0

        for i in range(2*(self.porder - 1), -1, -1):
            k = 0 if (i < self.porder) else (i - self.porder + 1)
            for j in range(i - k, k-1, -1):
                self.mat[j][i-j] += temp
            temp *= x
    
        temp = 1.0
        for i in range(self.porder-1, -1, -1):
            self.vec[i] += y * temp
            temp *= x

    def get_polynomial(self):
        inv_mat = np.linalg.inv(self.mat)
        res = np.zeros(self.porder)
        for i in range(self.porder):
            for j in range(self.porder):
                res[i] += inv_mat[i][j] * self.vec[j]
        return res

    def polyfit(self, x, y, order):
        self.porder = order + 1
        self.mat = np.zeros((self.porder, self.porder))
        self.vec = np.zeros(self.porder)
        for i in range(len(x)):
            self.update(x[i], y[i])
        return self.get_polynomial()

class IMUData:
    def __init__(self):
        self.accel = {}
        self.gyro = {}

    def IMUs(self):
        '''return list of IMUs'''
        if len(self.accel.keys()) != len(self.gyro.keys()):
            print("accel and gyro data doesn't match")
            sys.exit(1)
        return self.accel.keys()

    def add_accel(self, imu, temperature, time, value):
        if imu not in self.accel:
            self.accel[imu] = {}
            for axis in AXEST:
                self.accel[imu][axis] = np.zeros(0,dtype=float)
        self.accel[imu]['T'] = np.append(self.accel[imu]['T'], temperature)
        self.accel[imu]['X'] = np.append(self.accel[imu]['X'], value.x)
        self.accel[imu]['Y'] = np.append(self.accel[imu]['Y'], value.y)
        self.accel[imu]['Z'] = np.append(self.accel[imu]['Z'], value.z)
        self.accel[imu]['time'] = np.append(self.accel[imu]['time'], time)

    def add_gyro(self, imu, temperature, time, value):
        if imu not in self.gyro:
            self.gyro[imu] = {}
            for axis in AXEST:
                self.gyro[imu][axis] = np.zeros(0,dtype=float)
        self.gyro[imu]['T'] = np.append(self.gyro[imu]['T'], temperature)
        self.gyro[imu]['X'] = np.append(self.gyro[imu]['X'], value.x)
        self.gyro[imu]['Y'] = np.append(self.gyro[imu]['Y'], value.y)
        self.gyro[imu]['Z'] = np.append(self.gyro[imu]['Z'], value.z)
        self.gyro[imu]['time'] = np.append(self.gyro[imu]['time'], time)

    def moving_average(self, data, w):
        '''apply a moving average filter over a window of width w'''
        ret = np.cumsum(data)
        ret[w:] = ret[w:] - ret[:-w]
        return ret[w - 1:] / w

    def FilterArray(self, data, width_s):
        '''apply moving average filter of width width_s seconds'''
        nseconds = data['time'][-1] - data['time'][0]
        nsamples = len(data['time'])
        window = int(nsamples / nseconds) * width_s
        if window > 1:
            for axis in AXEST:
                data[axis] = self.moving_average(data[axis], window)
        return data

    def Filter(self, width_s):
        '''apply moving average filter of width width_s seconds'''
        for imu in self.IMUs():
            self.accel[imu] = self.FilterArray(self.accel[imu], width_s)
            self.gyro[imu] = self.FilterArray(self.gyro[imu], width_s)

    def accel_at_temp(self, imu, axis, temperature):
        '''return the accel value closest to the given temperature'''
        if temperature < self.accel[imu]['T'][0]:
            return self.accel[imu][axis][0]
        for i in range(len(self.accel[imu]['T'])-1):
            if temperature >= self.accel[imu]['T'][i] and temperature <= self.accel[imu]['T'][i+1]:
                v1 = self.accel[imu][axis][i]
                v2 = self.accel[imu][axis][i+1]
                p = (temperature - self.accel[imu]['T'][i]) / (self.accel[imu]['T'][i+1]-self.accel[imu]['T'][i])
                return v1 + (v2-v1) * p
        return self.accel[imu][axis][-1]

    def gyro_at_temp(self, imu, axis, temperature):
        '''return the gyro value closest to the given temperature'''
        if temperature < self.gyro[imu]['T'][0]:
            return self.gyro[imu][axis][0]
        for i in range(len(self.gyro[imu]['T'])-1):
            if temperature >= self.gyro[imu]['T'][i] and temperature <= self.gyro[imu]['T'][i+1]:
                v1 = self.gyro[imu][axis][i]
                v2 = self.gyro[imu][axis][i+1]
                p = (temperature - self.gyro[imu]['T'][i]) / (self.gyro[imu]['T'][i+1]-self.gyro[imu]['T'][i])
                return v1 + (v2-v1) * p
        return self.gyro[imu][axis][-1]


def constrain(value, minv, maxv):
    """Constrain a value to a range."""
    if value < minv:
        value = minv
    if value > maxv:
        value = maxv
    return value

def IMUfit(logfile):
    '''find IMU calibration parameters from a log file'''
    print("Processing log %s" % logfile)
    mlog = mavutil.mavlink_connection(logfile)

    data = IMUData()

    c = Coefficients()
    orientation = 0

    stop_capture = [ False ] * 3

    if args.tclr:
        messages = ['PARM','TCLR']
    else:
        messages = ['PARM','IMU']

    while True:
        msg = mlog.recv_match(type=messages)
        if msg is None:
            break

        if msg.get_type() == 'PARM':
            # build up the old coefficients so we can remove the impact of
            # existing coefficients from the data
            m = re.match("^INS_TCAL(\d)_ENABLE$", msg.Name)
            if m:
                imu = int(m.group(1))-1
                if stop_capture[imu]:
                    continue
                if msg.Value == 1 and c.enable[imu] == 2:
                    print("TCAL[%u] enabled" % imu)
                    stop_capture[imu] = True
                    continue
                if msg.Value == 0 and c.enable[imu] == 1:
                    print("TCAL[%u] disabled" % imu)
                    stop_capture[imu] = True
                    continue
                c.set_enable(imu, msg.Value)
            m = re.match("^INS_TCAL(\d)_(ACC|GYR)([1-3])_([XYZ])$", msg.Name)
            if m:
                imu = int(m.group(1))-1
                stype = m.group(2)
                p = int(m.group(3))
                axis = m.group(4)
                if stop_capture[imu]:
                    continue
                if stype == 'ACC':
                    c.set_acoeff(imu, axis, p, msg.Value/SCALE_FACTOR)
                if stype == 'GYR':
                    c.set_gcoeff(imu, axis, p, msg.Value/SCALE_FACTOR)
            m = re.match("^INS_TCAL(\d)_TMIN$", msg.Name)
            if m:
                imu = int(m.group(1))-1
                if stop_capture[imu]:
                    continue
                c.set_tmin(imu, msg.Value)
            m = re.match("^INS_TCAL(\d)_TMAX", msg.Name)
            if m:
                imu = int(m.group(1))-1
                if stop_capture[imu]:
                    continue
                c.set_tmax(imu, msg.Value)
            m = re.match("^INS_GYR(\d)_CALTEMP", msg.Name)
            if m:
                imu = int(m.group(1))-1
                if stop_capture[imu]:
                    continue
                c.set_gyro_tcal(imu, msg.Value)
            m = re.match("^INS_ACC(\d)_CALTEMP", msg.Name)
            if m:
                imu = int(m.group(1))-1
                if stop_capture[imu]:
                    continue
                c.set_accel_tcal(imu, msg.Value)
            m = re.match("^INS_(ACC|GYR)(\d?)OFFS_([XYZ])$", msg.Name)
            if m:
                stype = m.group(1)
                if m.group(2) == "":
                    imu = 0
                else:
                    imu = int(m.group(2))-1
                axis = m.group(3)
                if stop_capture[imu]:
                    continue
                if stype == 'ACC':
                    c.set_aoffset(imu, axis, msg.Value)
                if stype == 'GYR':
                    c.set_goffset(imu, axis, msg.Value)
            if msg.Name == 'AHRS_ORIENTATION':
                orientation = int(msg.Value)
                print("Using orientation %d" % orientation)

        if msg.get_type() == 'TCLR' and args.tclr:
            imu = msg.I

            T = msg.Temp
            if msg.SType == 0:
                # accel
                acc = Vector3(msg.X, msg.Y, msg.Z)
                time = msg.TimeUS*1.0e-6
                data.add_accel(imu, T, time, acc)
            elif msg.SType == 1:
                # gyro
                gyr = Vector3(msg.X, msg.Y, msg.Z)
                time = msg.TimeUS*1.0e-6
                data.add_gyro(imu, T, time, gyr)

        if msg.get_type() == 'IMU' and not args.tclr:
            imu = msg.I

            if stop_capture[imu]:
                continue

            T = msg.T
            acc = Vector3(msg.AccX, msg.AccY, msg.AccZ)
            gyr = Vector3(msg.GyrX, msg.GyrY, msg.GyrZ)

            # invert the board orientation rotation. Corrections are in sensor frame
            if orientation != 0:
                acc = acc.rotate_by_inverse_id(orientation)
                gyr = gyr.rotate_by_inverse_id(orientation)
            if acc is None or gyr is None:
                print("Invalid AHRS_ORIENTATION %u" % orientation)
                sys.exit(1)

            if c.enable[imu] == 1:
                acc -= c.correction_accel(imu, T)
                gyr -= c.correction_gyro(imu, T)

            time = msg.TimeUS*1.0e-6
            data.add_accel(imu, T, time, acc)
            data.add_gyro (imu, T, time, gyr)

    if len(data.IMUs()) == 0:
        print("No data found")
        sys.exit(1)

    print("Loaded %u accel and %u gyro samples" % (len(data.accel[0]['T']),len(data.gyro[0]['T'])))

    if not args.tclr:
        # apply moving average filter with 2s width
        data.Filter(2)

    clog = c
    c = Coefficients()

    calfile = open(args.outfile, "w")

    for imu in data.IMUs():
        tmin = np.amin(data.accel[imu]['T'])
        tmax = np.amax(data.accel[imu]['T'])

        c.set_tmin(imu, tmin)
        c.set_tmax(imu, tmax)

        for axis in AXES:
            if args.online:
                fit = OnlineIMUfit()
                trel = data.accel[imu]['T'] - TEMP_REF
                ofs = data.accel_at_temp(imu, axis, clog.atcal[imu])
                c.set_accel_poly(imu, axis, fit.polyfit(trel, data.accel[imu][axis] - ofs, POLY_ORDER))
                trel = data.gyro[imu]['T'] - TEMP_REF
                c.set_gyro_poly(imu, axis, fit.polyfit(trel, data.gyro[imu][axis], POLY_ORDER))
            else:
                trel = data.accel[imu]['T'] - TEMP_REF
                if imu in clog.atcal:
                    ofs = data.accel_at_temp(imu, axis, clog.atcal[imu])
                else:
                    ofs = np.mean(data.accel[imu][axis])
                c.set_accel_poly(imu, axis, np.polyfit(trel, data.accel[imu][axis] - ofs, POLY_ORDER))
                trel = data.gyro[imu]['T'] - TEMP_REF
                c.set_gyro_poly(imu, axis, np.polyfit(trel, data.gyro[imu][axis], POLY_ORDER))

        params = c.param_string(imu)
        print(params)
        calfile.write(params)

    calfile.close()
    print("Calibration written to %s" % args.outfile)

    if args.no_graph:
        return
    fig, axs = pyplot.subplots(len(data.IMUs()), 1, sharex=True)

    num_imus = len(data.IMUs())
    if num_imus == 1:
        axs = [axs]

    for imu in data.IMUs():
        scale = math.degrees(1)
        for axis in AXES:
            axs[imu].plot(data.gyro[imu]['time'], data.gyro[imu][axis]*scale, label='Uncorrected %s' % axis)
        for axis in AXES:
            poly = np.poly1d(c.gcoef[imu][axis])
            trel = data.gyro[imu]['T'] - TEMP_REF
            correction = poly(trel)
            axs[imu].plot(data.gyro[imu]['time'], (data.gyro[imu][axis] - correction)*scale, label='Corrected %s' % axis)
        if args.log_parm:
            for axis in AXES:
                if clog.enable[imu] == 0.0:
                    print("IMU[%u] disabled in log parms" % imu)
                    continue
                poly = np.poly1d(clog.gcoef[imu][axis])
                correction = poly(data.gyro[imu]['T'] - TEMP_REF) - poly(clog.gtcal[imu] - TEMP_REF) + clog.gofs[imu][axis]
                axs[imu].plot(data.gyro[imu]['time'], (data.gyro[imu][axis] - correction)*scale, label='Corrected %s (log parm)' % axis)
        ax2 = axs[imu].twinx()
        ax2.plot(data.gyro[imu]['time'], data.gyro[imu]['T'], label='Temperature(C)', color='black')
        ax2.legend(loc='upper right')
        axs[imu].legend(loc='upper left')
        axs[imu].set_title('IMU[%u] Gyro (deg/s)' % imu)

    fig, axs = pyplot.subplots(num_imus, 1, sharex=True)
    if num_imus == 1:
        axs = [axs]

    for imu in data.IMUs():
        for axis in AXES:
            ofs = data.accel_at_temp(imu, axis, clog.atcal.get(imu, TEMP_REF))
            axs[imu].plot(data.accel[imu]['time'], data.accel[imu][axis] - ofs, label='Uncorrected %s' % axis)
        for axis in AXES:
            poly = np.poly1d(c.acoef[imu][axis])
            trel = data.accel[imu]['T'] - TEMP_REF
            correction = poly(trel)
            ofs = data.accel_at_temp(imu, axis, clog.atcal.get(imu, TEMP_REF))
            axs[imu].plot(data.accel[imu]['time'], (data.accel[imu][axis] - ofs) - correction, label='Corrected %s' % axis)
        if args.log_parm:
            for axis in AXES:
                if clog.enable[imu] == 0.0:
                    print("IMU[%u] disabled in log parms" % imu)
                    continue
                poly = np.poly1d(clog.acoef[imu][axis])
                ofs = data.accel_at_temp(imu, axis, clog.atcal[imu])
                correction = poly(data.accel[imu]['T'] - TEMP_REF) - poly(clog.atcal[imu] - TEMP_REF)
                axs[imu].plot(data.accel[imu]['time'], (data.accel[imu][axis] - ofs) - correction, label='Corrected %s (log parm)' % axis)
        ax2 = axs[imu].twinx()
        ax2.plot(data.accel[imu]['time'], data.accel[imu]['T'], label='Temperature(C)', color='black')
        ax2.legend(loc='upper right')
        axs[imu].legend(loc='upper left')
        axs[imu].set_title('IMU[%u] Accel (m/s^2)' % imu)
        
    pyplot.show()



IMUfit(args.log)


