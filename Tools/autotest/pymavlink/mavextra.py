#!/usr/bin/env python
'''
useful extra functions for use by mavlink clients

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

from math import *


def kmh(mps):
    '''convert m/s to Km/h'''
    return mps*3.6

def altitude(SCALED_PRESSURE):
    '''calculate barometric altitude'''
    import mavutil
    self = mavutil.mavfile_global
    if self.ground_pressure is None or self.ground_temperature is None:
        return 0
    scaling = self.ground_pressure / (SCALED_PRESSURE.press_abs*100.0)
    temp = self.ground_temperature + 273.15
    return log(scaling) * temp * 29271.267 * 0.001


def mag_heading(RAW_IMU, ATTITUDE, declination=0, SENSOR_OFFSETS=None, ofs=None):
    '''calculate heading from raw magnetometer'''
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z

    headX = mag_x*cos(ATTITUDE.pitch) + mag_y*sin(ATTITUDE.roll)*sin(ATTITUDE.pitch) + mag_z*cos(ATTITUDE.roll)*sin(ATTITUDE.pitch)
    headY = mag_y*cos(ATTITUDE.roll) - mag_z*sin(ATTITUDE.roll)
    heading = degrees(atan2(-headY,headX)) + declination
    if heading < 0:
        heading += 360
    return heading

def mag_field(RAW_IMU, SENSOR_OFFSETS=None, ofs=None):
    '''calculate magnetic field strength from raw magnetometer'''
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z
    return sqrt(mag_x**2 + mag_y**2 + mag_z**2)

def angle_diff(angle1, angle2):
    '''show the difference between two angles in degrees'''
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret


lowpass_data = {}

def lowpass(var, key, factor):
    '''a simple lowpass filter'''
    global lowpass_data
    if not key in lowpass_data:
        lowpass_data[key] = var
    else:
        lowpass_data[key] = factor*lowpass_data[key] + (1.0 - factor)*var
    return lowpass_data[key]

last_delta = {}

def delta(var, key):
    '''calculate slope'''
    global last_delta
    dv = 0
    if key in last_delta:
        dv = var - last_delta[key]
    last_delta[key] = var
    return dv

def delta_angle(var, key):
    '''calculate slope of an angle'''
    global last_delta
    dv = 0
    if key in last_delta:
        dv = var - last_delta[key]
    last_delta[key] = var
    if dv > 180:
        dv -= 360
    if dv < -180:
        dv += 360
    return dv

def roll_estimate(RAW_IMU,SENSOR_OFFSETS=None, ofs=None, mul=None,smooth=0.7):
    '''estimate roll from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(degrees(-asin(ry/sqrt(rx**2+ry**2+rz**2))),'_roll',smooth)

def pitch_estimate(RAW_IMU, SENSOR_OFFSETS=None, ofs=None, mul=None, smooth=0.7):
    '''estimate pitch from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(degrees(asin(rx/sqrt(rx**2+ry**2+rz**2))),'_pitch',smooth)

def gravity(RAW_IMU, SENSOR_OFFSETS=None, ofs=None, mul=None, smooth=0.7):
    '''estimate pitch from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(sqrt(rx**2+ry**2+rz**2),'_gravity',smooth)



def pitch_sim(SIMSTATE, GPS_RAW):
    '''estimate pitch from SIMSTATE accels'''
    xacc = SIMSTATE.xacc - lowpass(delta(GPS_RAW.v,"v")*6.6, "v", 0.9)
    zacc = SIMSTATE.zacc
    zacc += SIMSTATE.ygyro * GPS_RAW.v;
    if xacc/zacc >= 1:
        return 0
    if xacc/zacc <= -1:
        return -0
    return degrees(-asin(xacc/zacc))

def distance_two(GPS_RAW1, GPS_RAW2):
    '''distance between two points'''
    lat1 = radians(GPS_RAW1.lat)
    lat2 = radians(GPS_RAW2.lat)
    lon1 = radians(GPS_RAW1.lon)
    lon2 = radians(GPS_RAW2.lon)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
    return 6371 * 1000 * c


first_fix = None

def distance_home(GPS_RAW):
    '''distance from first fix point'''
    global first_fix
    if GPS_RAW.fix_type < 2:
        return 0
    if first_fix == None or first_fix.fix_type < 2:
        first_fix = GPS_RAW
        return 0
    return distance_two(GPS_RAW, first_fix)

def sawtooth(ATTITUDE, amplitude=2.0, period=5.0):
    '''sawtooth pattern based on uptime'''
    mins = (ATTITUDE.usec * 1.0e-6)/60
    p = fmod(mins, period*2)
    if p < period:
        return amplitude * (p/period)
    return amplitude * (period - (p-period))/period
