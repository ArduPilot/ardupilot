#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" ArduPilot IMU Filter Test Tool

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

__author__ = "Guglielmo Cassinelli"
__contact__ = "gdguglie@gmail.com"

try: # Python 3.x
    from tkinter import Tk
    from tkinter.filedialog import askopenfilename
except ImportError: # Python 2.x
    from Tkinter import Tk
    from tkFileDialog import askopenfilename

import argparse
import ntpath
import numpy as np
from pymavlink import mavutil


"""
read command line parameters
"""

parser = argparse.ArgumentParser(description='ArduPilot IMU Filter Tester Tool. Input one log file from ')
parser.add_argument('file', nargs='?', default=None, help='bin log file containing raw IMU logs')
parser.add_argument('--begin-time', '-b', type=int, default=0, help='start from second')
parser.add_argument('--end-time', '-e', type=int, default=-1, help='end to second')

args = parser.parse_args()

log_file = args.file
begin_time = args.begin_time
end_time = args.end_time

# if log not input by command line
if not log_file:
    # GUI log file chooser
    root = Tk()
    root.withdraw()
    root.focus_force()
    log_file = askopenfilename(title="Select log file", filetypes=(("log files", ".BIN .bin .log"), ("all files", "*.*")))
    root.update()
    root.destroy()

if log_file is None or log_file == "":
    print("No log file to open")
    quit()

log_name = ntpath.basename(log_file)

"""
default settings
"""
POST_FILTER_LOGGING_BIT = 2 ** 1

RAW_IMU_LOG_BIT = 2 ** 19

PREVENT_POST_FILTER_LOGS = False

PARAMS_TO_CHECK = [
    "INS_LOG_BAT_OPT", "INS_GYRO_FILTER", "INS_ACCEL_FILTER",
    "INS_NOTCH_ENABLE", "INS_NOTCH_FREQ", "INS_NOTCH_BW", "INS_NOTCH_ATT",
    "INS_NOTCA_ENABLE", "INS_NOTCA_FREQ", "INS_NOTCA_BW", "INS_NOTCA_ATT",
    "LOG_BITMASK"
]

DEFAULT_ACC_FILTER = 80  # hz
DEFAULT_GYR_FILTER = 80  # hz

DEFAULT_ACC_NOTCH_FREQ = 150  # hz
DEFAULT_ACC_NOTCH_ATTENUATION = 30  # db
DEFAULT_ACC_NOTCH_BANDWIDTH = 100  # hz

DEFAULT_GYR_NOTCH_FREQ = 145
DEFAULT_GYR_NOTCH_ATTENUATION = 30  # db
DEFAULT_GYR_NOTCH_BANDWIDTH = 100  # hz

ACCEL_NOTCH_FILTER = True

"""
load LOG
"""
print("Loading %s...\n" % log_name)

mlog = mavutil.mavlink_connection(log_file)

log_start_time = 0
log_end_time = 0

ACC_t = []
ACC_x = []
ACC_y = []
ACC_z = []

GYR_t = []
GYR_x = []
GYR_y = []
GYR_z = []

params = {}

while True:
    m = mlog._parse_next()
    """
    @type m DFMessage
    """

    if m is None:
        break

    if m.fmt.name == "PARM":
        # check param value

        if m.Name in PARAMS_TO_CHECK:
            print(m.Name, ", ", m.Value)
            params[m.Name] = m.Value

    try:
        m_time_sec = m.TimeUS / 1000000.

        if log_start_time == 0:
            log_start_time = m_time_sec

        if m_time_sec < begin_time:
            continue

        if end_time > 0 and m_time_sec > end_time:
            continue
    except AttributeError:
        pass

    if m.fmt.name == "ACC1":
        ACC_t.append(m_time_sec)
        ACC_x.append(m.AccX)
        ACC_y.append(m.AccY)
        ACC_z.append(m.AccZ)

    elif m.fmt.name == "GYR1":
        GYR_t.append(m_time_sec)
        GYR_x.append(m.GyrX)
        GYR_y.append(m.GyrY)
        GYR_z.append(m.GyrZ)


def print_log_msg_stats(log_time_list, msg_name):
    msg_count = len(log_time_list)

    if msg_count > 0:
        msg_total_time = log_time_list[-1] - log_time_list[0]
        msg_freq = msg_count / msg_total_time
    else:
        msg_total_time = 0
        msg_freq = 0

    print("\n{} {} logs for a duration of {:.1f} secs".format(msg_count, msg_name, msg_total_time))
    print(msg_name + " frequency = {:.2f} hz".format(msg_freq))

    return msg_freq


def get_mean_and_std(np_arr):
    mean = np.mean(np_arr)
    std = np.std(np_arr)
    return mean, std


def print_mean_and_std(np_arr, name=""):
    mean, std = get_mean_and_std(np_arr)
    print("{} mean {:.3f}  std {:.3f}".format(name, mean, std))


def set_bit(number, bit_index, bit_value):
    """Set the index:th bit of v to 1 if x is truthy, else to 0, and return the new value."""
    mask = 1 << bit_index  # Compute mask, an integer with just bit 'index' set.
    number &= ~mask  # Clear the bit indicated by the mask (if x is False)
    if bit_value:
        number |= mask  # If x was True, set the bit indicated by the mask.
    return number  # Return the result, we're done.


ACC_freq = print_log_msg_stats(ACC_t, "ACC")
GYR_freq = print_log_msg_stats(GYR_t, "GYR")

if not ACC_t or not GYR_t:
    print("\nNo RAW IMU logs to analyze")
    quit()

if "INS_LOG_BAT_OPT" in params:
    log_bat_opt = int(params["INS_LOG_BAT_OPT"])
    if log_bat_opt & POST_FILTER_LOGGING_BIT:
        print("\nINS_LOG_BAT_OPT was set to {} which enables post filter logging,"
              "use pre filter logging to not sum multiple filter passes.".format(log_bat_opt))
        print("(set INS_LOG_BAT_OPT = {})".format(set_bit(log_bat_opt, 1, 0)))

        if PREVENT_POST_FILTER_LOGS:
            quit()
else:
    print("couldn't check ")

if "LOG_BITMASK" in params:
    log_bitmask = int(params["LOG_BITMASK"])
    if not log_bitmask & RAW_IMU_LOG_BIT:
        print("\nWARNING: LOG_BITMASK was not set to enable RAW_IMU logging, please enable it to have best resolution")
else:
    print("\nWARNING: Cannot read LOG_BITMASK, please ensure to have enabled RAW_IMU logging")

# set filter parameters
print("Reading filter parameters to set initial filter values...")

if "INS_GYRO_FILTER" in params:
    DEFAULT_GYR_FILTER = params["INS_GYRO_FILTER"]

if "INS_ACCEL_FILTER" in params:
    DEFAULT_ACC_FILTER = params["INS_ACCEL_FILTER"]

if "INS_NOTCH_ENABLE" in params:
    if params["INS_NOTCH_ENABLE"] != 0:
        if "INS_NOTCH_ATT" in params:
            DEFAULT_GYR_NOTCH_ATTENUATION = params["INS_NOTCH_ATT"]
    else:
        DEFAULT_GYR_NOTCH_ATTENUATION = 0

    if "INS_NOTCH_BW" in params:
        DEFAULT_GYR_NOTCH_BANDWIDTH = params["INS_NOTCH_BW"]

    if "INS_NOTCH_FREQ" in params:
        DEFAULT_GYR_NOTCH_FREQ = params["INS_NOTCH_FREQ"]

if "INS_NOTCA_ENABLE" in params:
    if params["INS_NOTCA_ENABLE"] != 0:
        if "INS_NOTCA_ATT" in params:
            DEFAULT_ACC_NOTCH_ATTENUATION = params["INS_NOTCA_ATT"]
    else:
        DEFAULT_ACC_NOTCH_ATTENUATION = 0

    if "INS_NOTCA_BW" in params:
        DEFAULT_ACC_NOTCH_BANDWIDTH = params["INS_NOTCA_BW"]

    if "INS_NOTCA_FREQ" in params:
        DEFAULT_ACC_NOTCH_FREQ = params["INS_NOTCA_FREQ"]

else:
    print("The firmware that produced this log does not support notch filter on accelerometer")
    ACCEL_NOTCH_FILTER = False


"""
run filter tet
"""
from FilterTest import FilterTest

filter_test = FilterTest(ACC_t, ACC_x, ACC_y, ACC_z, GYR_t, GYR_x, GYR_y, GYR_z, ACC_freq, GYR_freq,
                         DEFAULT_ACC_FILTER, DEFAULT_GYR_FILTER,
                         DEFAULT_ACC_NOTCH_FREQ, DEFAULT_ACC_NOTCH_ATTENUATION, DEFAULT_ACC_NOTCH_BANDWIDTH,
                         DEFAULT_GYR_NOTCH_FREQ, DEFAULT_GYR_NOTCH_ATTENUATION, DEFAULT_GYR_NOTCH_BANDWIDTH,
                         log_name, ACCEL_NOTCH_FILTER)
