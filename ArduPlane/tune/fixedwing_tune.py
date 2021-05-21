#!/usr/bin/env python

from __future__ import print_function

'''
roll accel A = ail * dP * K1
roll damping B = roll_rate * (K2 / Vtas) * dP

steady state when A == B

roll_rate = ail * K1 * Vtas / K2

tune for I = 0.75 * P

FF = median 5

'''

'''
framework for developing fixed wing tuning systems
'''
from builtins import range

import os
import math

from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument("log", metavar="LOG")
parser.add_argument("--debug", action='store_true')
parser.add_argument("--loop-rate", type=float, default=50.0, help="main loop rate")
parser.add_argument("--servo-filter", type=float, default=5.0, help="filter freq for servo model")
parser.add_argument("--servo-speed", type=float, default=0.15, help="servo speed in seconds for 60 degrees")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3, Matrix3
from math import degrees

class LowPassFilter2p:
    '''two pole butterworth filter'''
    def __init__(self, sample_rate_hz, cutoff_hz):
        self.sample_rate_hz = sample_rate_hz
        self.cutoff_hz = cutoff_hz

        fr = sample_rate_hz/cutoff_hz
        ohm = math.tan(math.pi/fr)
        c = 1.0+2.0*math.cos(math.pi/4.0)*ohm + ohm*ohm

        self.b0 = ohm*ohm/c
        self.b1 = 2.0*self.b0
        self.b2 = self.b0
        self.a1 = 2.0*(ohm*ohm-1.0)/c
        self.a2 = (1.0-2.0*math.cos(math.pi/4.0)*ohm+ohm*ohm)/c
        self.output = 0.0
        self.initialised = False

    def reset(self, value):
        '''reset filter to given value'''
        self.delay_element_1 = value * (1.0 / (1 + self.a1 + self.a2))
        self.delay_element_2 = self.delay_element_1
        self.output = value
        self.initialised = True

    def apply(self, value):
        '''apply filter'''
        if not self.initialised:
            self.reset(value)

        delay_element_0 = value - self.delay_element_1 * self.a1 - self.delay_element_2 * self.a2
        self.output = delay_element_0 * self.b0 + self.delay_element_1 * self.b1 + self.delay_element_2 * self.b2

        self.delay_element_2 = self.delay_element_1
        self.delay_element_1 = delay_element_0

        return self.output

    def get(self):
        '''get current value'''
        return self.output


class AileronModel:
    def __init__(self, initial_deflection):
        # deflection in degrees
        self.deflection = initial_deflection
        self.servo_model = LowPassFilter2p(args.loop_rate, args.servo_filter)

    def update(self, demanded_angle):
        '''update model with new demanded angle, called at loop-rate'''
        return self.servo_model.apply(demanded_angle)

class RollModel:
    '''numerical model of roll of aircraft'''
    def __init__(self, initial_roll, initial_rate, initial_deflection):
        self.roll = initial_roll
        self.roll_rate = initial_rate
        self.aileron = AileronModel(initial_deflection)

    def update(self, aileron_demand_deg, speed_scalar):
        '''update roll model'''

        # update aileron model, getting filtered aileron position
        aileron = self.aileron.update(aileron_demand_deg)

        # here is where we apply the model parameters, calculating roll acceleration
        # roll drag etc

class FWState(object):
    '''object containing roll state of aircraft'''
    def __init__(self, ATT, PIDR, AETR):
        self.timestamp = ATT.TimeUS*1.0e-6
        # demanded rate
        self.dem_rate_dps = PIDR.Tar
        # actual rate
        self.act_rate_dps = PIDR.Act
        # speed scalar
        self.speed_scalar = AETR.SS
        # aileron deflection demand, notional degrees assuming 45 degrees max total deflection
        self.aileron = AETR.Ail*0.01
        # roll in degrees
        self.roll = ATT.Roll
        
def process_log(filename):
    '''process one log'''
    print("Processing log %s" % filename)

    mlog = mavutil.mavlink_connection(filename)

    # ATT and PIDR messages come before AETR, so keep it around for time alignment
    PIDR = None
    ATT = None

    data = []

    while True:
        # we want AETR, PIDR and ATT
        m = mlog.recv_match(type=['AETR','PIDR','ATT'])
        if m is None:
            break
        t = m.get_type()

        if t == 'PIDR':
            PIDR = m
            continue

        if t == 'ATT':
            ATT = m
            continue
        
        if t == 'AETR' and PIDR is not None and ATT is not None:
            data.append(FWState(ATT,PIDR,m))

    print("Extracted %u points" % len(data))

    # create model of roll of aircraft
    model = RollModel(data[0].roll, data[0].act_rate_dps, data[0].aileron)

    # run the data through the model
    for d in data:
        model.update(d.aileron, d.speed_scalar)

    # graph all the fields we've output
    import matplotlib.pyplot as plt
    t = [ d.timestamp for d in data ]
    y = [ d.roll for d in data ]
    plt.plot(t, y, label='Roll')
    plt.legend(loc='upper left')
    plt.show()
        
process_log(args.log)
