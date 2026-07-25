#!/usr/bin/env python3

'''
estimate attitude from an ArduPilot replay log using a python state estimator
'''

import os

from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument("log", metavar="LOG")
parser.add_argument("--debug", action='store_true')
parser.add_argument("--graphs", default="EK3.Roll,EK3.Pitch,EST.Roll,EST.Pitch")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3, Matrix3
from math import degrees

GRAVITY_MSS = 9.80665

class Estimator(object):
    '''state estimator'''
    def __init__(self):
        self.r = Matrix3()
        self.vel = Vector3()
        self.pos = Vector3()

    def update_GPS(self, position, velocity):
        '''handle new GPS sample'''
        if args.debug:
            print('GPS: ', position, velocity)

    def update_MAG(self, field):
        '''handle new magnetometer sample'''
        if args.debug:
            print('MAG: ', field)

    def update_BARO(self, altitude):
        '''handle new barometer sample'''
        if args.debug:
            print('BARO: ', altitude)

    def update_IMU(self, delta_velocity, dv_dt, delta_angle, da_dt):
        '''handle new IMU sample'''

        # rotate using delta_angle
        self.r.rotate(delta_angle)

        # inertial update with accelerometer. This will diverge very quickly
        # without corrections
        earth_dvel = self.r * delta_velocity
        earth_dvel.z += GRAVITY_MSS*dv_dt

        self.vel += earth_dvel
        self.pos += self.vel * dv_dt

        # normalise the rotation matrix to stop numerical errors creeping in
        self.r.normalize()

def estimate(filename):
    '''run estimator over a replay log'''
    print("Processing log %s" % filename)

    mlog = mavutil.mavlink_connection(filename)

    est = Estimator()

    output = { 'SIM.Roll' : [],
               'SIM.Pitch' : [],
               'SIM.Yaw' : [],
               'EK3.Roll' : [],
               'EK3.Pitch' : [],
               'EK3.Yaw' : [],
               'EST.Roll' : [],
               'EST.Pitch' : [],
               'EST.Yaw' : [],
               'EK3.PN' : [],
               'EK3.PE' : [],
               'EK3.PD' : [],
               'EST.PN' : [],
               'EST.PE' : [],
               'EST.PD' : [],
                }

    RGPI = None
    RFRH = None

    while True:
        # we want replay sensor data, plus EKF3 result and SITL data
        m = mlog.recv_match(type=['XKF1','SIM','RFRH','RFRF','RISH','RISI','RGPH','RGPI','RGPJ','RFRH','RBRH','RBRI','RMGH','RMGI'])
        if m is None:
            break
        t = m.get_type()

        if t == 'XKF1' and m.C == 0:
            # output attitude of first EKF3 lane
            tsec = m.TimeUS*1.0e-6
            output['EK3.Roll'].append((tsec, m.Roll))
            output['EK3.Pitch'].append((tsec, m.Pitch))
            output['EK3.Yaw'].append((tsec, m.Yaw))
            output['EK3.PN'].append((tsec, m.PN))
            output['EK3.PE'].append((tsec, m.PE))
            output['EK3.PD'].append((tsec, m.PD))

        if t == 'SIM':
            # output SITL attitude
            tsec = m.TimeUS*1.0e-6
            output['SIM.Roll'].append((tsec, m.Roll))
            output['SIM.Pitch'].append((tsec, m.Pitch))
            output['SIM.Yaw'].append((tsec, m.Yaw))

        if t == 'RFRH':
            # replay frame header, used for timestamp of the frame
            RFRH = m

        if t == 'RGPI' and m.I == 0:
            # GPS0 status info, remember it so we know if we have a 3D fix
            RGPI = m

        if t == 'RGPJ' and m.I == 0 and RGPI.Stat >= 3:
            # update on GPS0, with 3D fix
            pos = Vector3(m.Lat*1.0e-7, m.Lon*1.0e-7, m.Alt*1.0e-2)
            vel = Vector3(m.VX, m.VY, m.VZ)
            est.update_GPS(pos, vel)

        if t == 'RMGI' and m.I == 0 and m.H == 1:
            # first compass sample, healthy compass
            field = Vector3(m.FX, m.FY, m.FZ)
            est.update_MAG(field)

        if t == 'RBRI' and m.I == 0:
            # first baro sample
            est.update_BARO(m.Alt)
            
        if t == 'RISI' and m.I == 0:
            # update on IMU0
            dvel = Vector3(m.DVX, m.DVY, m.DVZ)
            dang = Vector3(m.DAX, m.DAY, m.DAZ)
            est.update_IMU(dvel, m.DVDT, dang, m.DADT)

            # output euler roll/pitch
            r,p,y = est.r.to_euler()
            tsec = RFRH.TimeUS*1.0e-6
            output['EST.Roll'].append((tsec, degrees(r)))
            output['EST.Pitch'].append((tsec, degrees(p)))
            output['EST.Yaw'].append((tsec, degrees(y)))
            output['EST.PN'].append((tsec, est.pos.x))
            output['EST.PE'].append((tsec, est.pos.y))
            output['EST.PD'].append((tsec, est.pos.z))

    # graph all the fields we've output
    import matplotlib.pyplot as plt
    for k in args.graphs.split(','):
        t = [ v[0] for v in output[k] ]
        y = [ v[1] for v in output[k] ]
        plt.plot(t, y, label=k)
        plt.legend(loc='upper left')
    plt.show()

estimate(args.log)

