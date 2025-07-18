#!/usr/bin/env python3

# flake8: noqa

'''
fit coefficients for battery percentate from resting voltage

See AP_Scripting/applets/BattEstimate.lua
'''

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-graph", action='store_true', default=False, help='disable graph display')
parser.add_argument("--num-cells", type=int, default=0, help='cell count, zero for auto-detection')
parser.add_argument("--batidx", type=int, default=1, help='battery index')
parser.add_argument("--condition", default=None, help='match condition')
parser.add_argument("--final-pct", type=float, default=100.0, help='set final percentage in log')
parser.add_argument("--comparison", type=str, default=None, help='comparison coefficients')
parser.add_argument("log", metavar="LOG")

args = parser.parse_args()

import sys
import math
from pymavlink import mavutil
import numpy as np
import matplotlib.pyplot as pyplot

def constrain(value, minv, maxv):
    """Constrain a value to a range."""
    return max(min(value,maxv),minv)

def SOC_model(cell_volt, c):
    '''simple model of state of charge versus resting voltage.
    With thanks to Roho for the form of the equation
    https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery

    Adjusted to also fit other battery chemistries such as Li-ion.
    '''
    p0 = c[3]
    p1 = c[2]

    return constrain(c[0] * (1.0 - 1.0 / math.pow(1 + math.pow(cell_volt / c[1], p0), p1)), 0, 100)

def fit_batt(data):
    '''fit a set of battery data to the SOC model'''
    from scipy import optimize

    def fit_error(p):
        p = list(p)
        ret = 0
        for (voltR,pct) in data:
            error = pct - SOC_model(voltR, p)
            ret += abs(error)

        ret /= len(data)
        return ret

    p = [123.0, 3.7, 0.165, 80.0]
    bounds = [(100.0, 10000.0), (3.0, 3.9), (0.001, 0.4), (5.0, 100.0)]

    (p,err,iterations,imode,smode) = optimize.fmin_slsqp(fit_error, p, bounds=bounds, iter=10000, full_output=True)
    if imode != 0:
        print("Fit failed: %s" % smode)
        sys.exit(1)
    return p

def ExtractDataLog(logfile):
    '''find battery fit parameters from a log file'''
    print("Processing log %s" % logfile)
    mlog = mavutil.mavlink_connection(logfile)

    Wh_total = 0.0
    last_t = None
    data = []
    last_voltR = None

    while True:
        msg = mlog.recv_match(type=['BAT'], condition=args.condition)
        if msg is None:
            break

        if (
            msg.get_type() == 'BAT'
            and (getattr(msg, 'Inst', None) == args.batidx - 1
                 or getattr(msg, 'Instance', None) == args.batidx - 1)
            and msg.VoltR > 1
        ):
            current = msg.Curr
            voltR = msg.VoltR
            if last_voltR is not None and voltR > last_voltR:
                continue
            last_voltR = voltR
            power = current*voltR
            t = msg.TimeUS*1.0e-6

            if last_t is None:
                last_t = t
                continue

            dt = t - last_t
            if dt < 0.5:
                # 2Hz data is plenty
                continue
            last_t = t
            Wh_total += (power*dt)/3600.0

            data.append((voltR,Wh_total))

    if len(data) == 0:
        print("No data found")
        sys.exit(1)

    # calculate total pack capacity based on final percentage
    Wh_max = data[-1][1]/(args.final_pct*0.01)

    fit_data = []

    for i in range(len(data)):
        (voltR,Wh) = data[i]
        SOC = 100-100*Wh/Wh_max
        fit_data.append((voltR, SOC))

    print("Loaded %u samples" % len(data))
    return fit_data

def ExtractDataCSV(logfile):
    '''find battery fit parameters from a CSV file'''
    print("Processing CSV %s" % logfile)
    lines = open(logfile,'r').readlines()
    fit_data = []
    for line in lines:
        line = line.strip()
        if line.startswith("#"):
            continue
        v = line.split(',')
        if len(v) != 2:
            continue
        if not v[0][0].isnumeric() or not v[1][0].isnumeric():
            continue
        fit_data.append((float(v[1]),float(v[0])))
    return fit_data

def BattFit(fit_data, num_cells):

    fit_data = [ (v/num_cells,p) for (v,p) in fit_data ]

    c = fit_batt(fit_data)
    print("Coefficients C1=%.4f C2=%.4f C3=%.4f C4=%.4f" % (c[0], c[1], c[2], c[3]))

    if args.no_graph:
        return

    fig, axs = pyplot.subplots()

    np_volt = np.array([v for (v,p) in fit_data])
    np_pct = np.array([p for (v,p) in fit_data])
    axs.invert_xaxis()
    axs.plot(np_volt, np_pct, label='SOC')
    np_rem = np.zeros(0,dtype=float)

    # pad down to 3.2V to make it easier to visualise for logs that don't go to a low voltage
    low_volt = np_volt[-1]
    while low_volt > 3.2:
        low_volt -= 0.1
        np_volt = np.append(np_volt, low_volt)

    for i in range(np_volt.size):
        voltR = np_volt[i]
        np_rem = np.append(np_rem, SOC_model(voltR, c))

    axs.plot(np_volt, np_rem, label='SOC Fit')

    if args.comparison:
        c2 = args.comparison.split(',')
        c2 = [ float(x) for x in c2 ]
        np_rem2 = np.zeros(0,dtype=float)
        for i in range(np_volt.size):
            voltR = np_volt[i]
            np_rem2 = np.append(np_rem2, SOC_model(voltR, c2))
        axs.plot(np_volt, np_rem2, label='SOC Fit2')

    axs.legend(loc='upper left')
    axs.set_title('Battery Fit')

    pyplot.show()

def get_cell_count(data):
    if args.num_cells != 0:
        return args.num_cells
    volts = [ v[0] for v in data ]
    volts = sorted(volts)
    num_cells = round(volts[-1]/4.2)
    print("Max voltags %.1f num_cells %u" % (volts[-1], num_cells))
    return num_cells

if args.log.upper().endswith(".CSV"):
    fit_data = ExtractDataCSV(args.log)
else:
    fit_data = ExtractDataLog(args.log)

num_cells = get_cell_count(fit_data)
BattFit(fit_data, num_cells)
