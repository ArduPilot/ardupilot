#!/usr/bin/env python3
"""
Create temperature calibration parameters for IMUs based on log data.


SPDX-FileCopyrightText: 2021 Andrew Tridgell <andrew@tridgell.net>, 2024-2025 Amilcar do Carmo Lucas <amilcar.lucas@iav.de>

SPDX-License-Identifier: GPL-3.0-or-later
"""

import logging
import math
import os
import re
import sys
from argparse import ArgumentParser
from typing import Callable, Union

import numpy as np
from matplotlib import pyplot as plt
from pymavlink import mavutil
from pymavlink.rotmat import Vector3

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

AXES = ["X", "Y", "Z"]
AXEST = ["X", "Y", "Z", "T", "time"]


class Coefficients:  # pylint: disable=too-many-instance-attributes
    """class representing a set of coefficients."""

    def __init__(self) -> None:
        self.acoef: dict = {}
        self.gcoef: dict = {}
        self.enable: list[int] = [0] * 3
        self.tmin: list[float] = [-100] * 3
        self.tmax: list[float] = [-100] * 3
        self.gtcal: dict = {}
        self.atcal: dict = {}
        self.gofs: dict = {}
        self.aofs: dict = {}

    def set_accel_poly(self, imu: int, axis: str, values) -> None:
        if imu not in self.acoef:
            self.acoef[imu] = {}
        self.acoef[imu][axis] = values

    def set_gyro_poly(self, imu: int, axis: str, values) -> None:
        if imu not in self.gcoef:
            self.gcoef[imu] = {}
        self.gcoef[imu][axis] = values

    def set_acoeff(self, imu: int, axis: str, order: int, value: dict) -> None:
        if imu not in self.acoef:
            self.acoef[imu] = {}
        if axis not in self.acoef[imu]:
            self.acoef[imu][axis] = [0] * 4
        self.acoef[imu][axis][POLY_ORDER - order] = value

    def set_gcoeff(self, imu: int, axis: str, order: int, value: dict) -> None:
        if imu not in self.gcoef:
            self.gcoef[imu] = {}
        if axis not in self.gcoef[imu]:
            self.gcoef[imu][axis] = [0] * 4
        self.gcoef[imu][axis][POLY_ORDER - order] = value

    def set_aoffset(self, imu: int, axis: str, value: dict) -> None:
        if imu not in self.aofs:
            self.aofs[imu] = {}
        self.aofs[imu][axis] = value

    def set_goffset(self, imu: int, axis: str, value: dict) -> None:
        if imu not in self.gofs:
            self.gofs[imu] = {}
        self.gofs[imu][axis] = value

    def set_tmin(self, imu: int, tmin: float) -> None:
        self.tmin[imu] = tmin

    def set_tmax(self, imu: int, tmax: float) -> None:
        self.tmax[imu] = tmax

    def set_gyro_tcal(self, imu: int, value: dict) -> None:
        self.gtcal[imu] = value

    def set_accel_tcal(self, imu: int, value: dict) -> None:
        self.atcal[imu] = value

    def set_enable(self, imu: int, value: int) -> None:
        self.enable[imu] = value

    def correction(self, coeff: dict, imu: int, temperature: float, axis: str, cal_temp: float) -> float:  # pylint: disable=too-many-arguments, too-many-positional-arguments
        """Calculate correction from temperature calibration from log data using parameters."""
        if self.enable[imu] != 1.0:
            return 0.0
        if cal_temp < -80:
            return 0.0
        if axis not in coeff:
            return 0.0
        temperature = constrain(temperature, self.tmin[imu], self.tmax[imu])
        cal_temp = constrain(cal_temp, self.tmin[imu], self.tmax[imu])
        poly = np.poly1d(coeff[axis])
        return poly(cal_temp - TEMP_REF) - poly(temperature - TEMP_REF)  # type: ignore[no-any-return]

    def correction_accel(self, imu: int, temperature: float) -> Vector3:
        """
        Calculate accel correction from temperature calibration from
        log data using parameters.
        """
        cal_temp = self.atcal.get(imu, TEMP_REF)
        return Vector3(
            self.correction(self.acoef[imu], imu, temperature, "X", cal_temp),
            self.correction(self.acoef[imu], imu, temperature, "Y", cal_temp),
            self.correction(self.acoef[imu], imu, temperature, "Z", cal_temp),
        )

    def correction_gyro(self, imu: int, temperature: float) -> Vector3:
        """
        Calculate gyro correction from temperature calibration from
        log data using parameters.
        """
        cal_temp = self.gtcal.get(imu, TEMP_REF)
        return Vector3(
            self.correction(self.gcoef[imu], imu, temperature, "X", cal_temp),
            self.correction(self.gcoef[imu], imu, temperature, "Y", cal_temp),
            self.correction(self.gcoef[imu], imu, temperature, "Z", cal_temp),
        )

    def param_string(self, imu: int) -> str:
        params = ""
        params += f"INS_TCAL{imu + 1}_ENABLE 1\n"
        params += f"INS_TCAL{imu + 1}_TMIN {self.tmin[imu]:.1f}\n"
        params += f"INS_TCAL{imu + 1}_TMAX {self.tmax[imu]:.1f}\n"
        for p in range(POLY_ORDER):
            for axis in AXES:
                params += (
                    f"INS_TCAL{imu + 1}_ACC{p + 1}_{axis} {self.acoef[imu][axis][POLY_ORDER - (p + 1)] * SCALE_FACTOR:.9f}\n"
                )
        for p in range(POLY_ORDER):
            for axis in AXES:
                params += (
                    f"INS_TCAL{imu + 1}_GYR{p + 1}_{axis} {self.gcoef[imu][axis][POLY_ORDER - (p + 1)] * SCALE_FACTOR:.9f}\n"
                )
        return params


class OnlineIMUfit:  # pylint: disable=too-few-public-methods
    """implement the online learning used in ArduPilot."""

    def __init__(self) -> None:
        self.porder: int = 0
        self.mat = np.zeros((1, 1))
        self.vec = np.zeros(1)

    def __update(self, x, y) -> None:
        temp = 1.0

        for i in range(2 * (self.porder - 1), -1, -1):
            k = 0 if (i < self.porder) else (i - self.porder + 1)
            for j in range(i - k, k - 1, -1):
                self.mat[j][i - j] += temp
            temp *= x

        temp = 1.0
        for i in range(self.porder - 1, -1, -1):
            self.vec[i] += y * temp
            temp *= x

    def __get_polynomial(self) -> np.ndarray:
        inv_mat = np.linalg.inv(self.mat)
        res = np.zeros(self.porder)
        for i in range(self.porder):
            for j in range(self.porder):
                res[i] += inv_mat[i][j] * self.vec[j]
        return res

    def polyfit(self, x, y, order: int) -> np.ndarray:
        self.porder = order + 1
        self.mat = np.zeros((self.porder, self.porder))
        self.vec = np.zeros(self.porder)
        for i, value in enumerate(x):
            self.__update(value, y[i])
        return self.__get_polynomial()


# pylint: disable=invalid-name
class IMUData:
    """
    A class to manage IMU data, including acceleration and gyroscope readings.

    This class provides methods to add acceleration and gyroscope data, apply
    moving average filters, and retrieve data for specific IMUs and temperatures.
    """

    def __init__(self) -> None:
        self.accel: dict[int, dict[str, np.ndarray]] = {}
        self.gyro: dict[int, dict[str, np.ndarray]] = {}

    def IMUs(self) -> list[int]:  # noqa: N802
        """Return list of IMUs."""
        if len(self.accel.keys()) != len(self.gyro.keys()):
            logging.critical("accel and gyro data doesn't match")
            sys.exit(1)
        return self.accel.keys()  # type: ignore[return-value]

    def add_accel(self, imu: int, temperature: float, time: float, value: Vector3) -> None:
        if imu not in self.accel:
            self.accel[imu] = {}
            for axis in AXEST:
                self.accel[imu][axis] = np.zeros(0, dtype=float)
        self.accel[imu]["T"] = np.append(self.accel[imu]["T"], temperature)
        self.accel[imu]["X"] = np.append(self.accel[imu]["X"], value.x)
        self.accel[imu]["Y"] = np.append(self.accel[imu]["Y"], value.y)
        self.accel[imu]["Z"] = np.append(self.accel[imu]["Z"], value.z)
        self.accel[imu]["time"] = np.append(self.accel[imu]["time"], time)

    def add_gyro(self, imu: int, temperature: float, time: float, value: Vector3) -> None:
        if imu not in self.gyro:
            self.gyro[imu] = {}
            for axis in AXEST:
                self.gyro[imu][axis] = np.zeros(0, dtype=float)
        self.gyro[imu]["T"] = np.append(self.gyro[imu]["T"], temperature)
        self.gyro[imu]["X"] = np.append(self.gyro[imu]["X"], value.x)
        self.gyro[imu]["Y"] = np.append(self.gyro[imu]["Y"], value.y)
        self.gyro[imu]["Z"] = np.append(self.gyro[imu]["Z"], value.z)
        self.gyro[imu]["time"] = np.append(self.gyro[imu]["time"], time)

    def moving_average(self, data: np.ndarray, w: int) -> np.ndarray:
        """Apply a moving average filter over a window of width w."""
        ret: np.ndarray = np.cumsum(data)
        ret[w:] = ret[w:] - ret[:-w]
        return ret[w - 1 :] / w

    def filter_array(self, data: dict[str, np.ndarray], width_s: int) -> dict[str, np.ndarray]:
        """Apply moving average filter of width width_s seconds."""
        nseconds = data["time"][-1] - data["time"][0]
        nsamples = len(data["time"])
        window = int(nsamples / nseconds) * width_s
        if window > 1:
            for axis in AXEST:
                data[axis] = self.moving_average(data[axis], window)
        return data

    def filter(self, width_s: int) -> None:
        """Apply moving average filter of width width_s seconds."""
        for imu in self.IMUs():
            self.accel[imu] = self.filter_array(self.accel[imu], width_s)
            self.gyro[imu] = self.filter_array(self.gyro[imu], width_s)

    def accel_at_temp(self, imu: int, axis: str, temperature: float) -> float:
        """Return the accel value closest to the given temperature."""
        if temperature < self.accel[imu]["T"][0]:
            return self.accel[imu][axis][0]  # type: ignore[no-any-return]
        for i in range(len(self.accel[imu]["T"]) - 1):
            if self.accel[imu]["T"][i] <= temperature <= self.accel[imu]["T"][i + 1]:
                v1 = self.accel[imu][axis][i]
                v2 = self.accel[imu][axis][i + 1]
                p = (temperature - self.accel[imu]["T"][i]) / (self.accel[imu]["T"][i + 1] - self.accel[imu]["T"][i])
                return v1 + (v2 - v1) * p  # type: ignore[no-any-return]
        return self.accel[imu][axis][-1]  # type: ignore[no-any-return]

    def gyro_at_temp(self, imu: int, axis: str, temperature: float) -> float:
        """Return the gyro value closest to the given temperature."""
        if temperature < self.gyro[imu]["T"][0]:
            return self.gyro[imu][axis][0]  # type: ignore[no-any-return]
        for i in range(len(self.gyro[imu]["T"]) - 1):
            if self.gyro[imu]["T"][i] <= temperature <= self.gyro[imu]["T"][i + 1]:
                v1 = self.gyro[imu][axis][i]
                v2 = self.gyro[imu][axis][i + 1]
                p = (temperature - self.gyro[imu]["T"][i]) / (self.gyro[imu]["T"][i + 1] - self.gyro[imu]["T"][i])
                return v1 + (v2 - v1) * p  # type: ignore[no-any-return]
        return self.gyro[imu][axis][-1]  # type: ignore[no-any-return]


def constrain(value: float, minv: float, maxv: float) -> Union[float, int]:
    """Constrain a value to a range."""
    value = max(minv, value)
    return min(maxv, value)


def IMUfit(  # noqa: PLR0915, N802, pylint: disable=too-many-locals, too-many-branches, too-many-statements, too-many-arguments, too-many-positional-arguments
    logfile: str,
    outfile: str,
    no_graph: bool,
    log_parm: bool,
    online: bool,
    tclr: bool,
    figpath: Union[str, None],
    progress_callback: Union[Callable[[int], None], None],
) -> None:
    """Find IMU calibration parameters from a log file."""
    logging.info("Processing log %s", logfile)
    mlog = mavutil.mavlink_connection(logfile, progress_callback=progress_callback)

    data = IMUData()

    c = Coefficients()
    orientation = 0

    stop_capture = [False] * 3

    messages = ["PARM", "TCLR"] if tclr else ["PARM", "IMU"]

    # Pre-compile regular expressions used frequently inside the loop
    enable_pattern = re.compile(r"^INS_TCAL(\d)_ENABLE$")
    coeff_pattern = re.compile(r"^INS_TCAL(\d)_(ACC|GYR)([1-3])_([XYZ])$")
    tmin_pattern = re.compile(r"^INS_TCAL(\d)_TMIN$")
    tmax_pattern = re.compile(r"^INS_TCAL(\d)_TMAX$")
    gyr_caltemp_pattern = re.compile(r"^INS_GYR(\d)_CALTEMP")
    acc_caltemp_pattern = re.compile(r"^INS_ACC(\d)_CALTEMP")
    offset_pattern = re.compile(r"^INS_(ACC|GYR)(\d?)OFFS_([XYZ])$")

    total_msgs = 0
    for mtype in messages:
        total_msgs += mlog.counts[mlog.name_to_id[mtype]]

    logging.info("Found %d messages", total_msgs)

    pct = 0
    msgcnt = 0
    while True:
        msg = mlog.recv_match(type=messages)
        if msg is None:
            break

        if progress_callback is not None:
            msgcnt += 1
            new_pct = (100 * msgcnt) // total_msgs
            if new_pct != pct:
                progress_callback(100 + new_pct)
                pct = new_pct

        msg_type = msg.get_type()
        if msg_type == "PARM":
            # build up the old coefficients so we can remove the impact of
            # existing coefficients from the data
            m = enable_pattern.match(msg.Name)
            if m:
                imu = int(m.group(1)) - 1
                if stop_capture[imu]:
                    continue
                if msg.Value == 1 and c.enable[imu] == 2:
                    logging.info("TCAL[%d] enabled", imu)
                    stop_capture[imu] = True
                    continue
                if msg.Value == 0 and c.enable[imu] == 1:
                    logging.info("TCAL[%d] disabled", imu)
                    stop_capture[imu] = True
                    continue
                c.set_enable(imu, msg.Value)
                continue
            m = coeff_pattern.match(msg.Name)
            if m:
                imu = int(m.group(1)) - 1
                stype = m.group(2)
                p = int(m.group(3))
                axis = m.group(4)
                if stop_capture[imu]:
                    continue
                if stype == "ACC":
                    c.set_acoeff(imu, axis, p, msg.Value / SCALE_FACTOR)
                if stype == "GYR":
                    c.set_gcoeff(imu, axis, p, msg.Value / SCALE_FACTOR)
                continue
            m = tmin_pattern.match(msg.Name)
            if m:
                imu = int(m.group(1)) - 1
                if stop_capture[imu]:
                    continue
                c.set_tmin(imu, msg.Value)
                continue
            m = tmax_pattern.match(msg.Name)
            if m:
                imu = int(m.group(1)) - 1
                if stop_capture[imu]:
                    continue
                c.set_tmax(imu, msg.Value)
                continue
            m = gyr_caltemp_pattern.match(msg.Name)
            if m:
                imu = int(m.group(1)) - 1
                if stop_capture[imu]:
                    continue
                c.set_gyro_tcal(imu, msg.Value)
                continue
            m = acc_caltemp_pattern.match(msg.Name)
            if m:
                imu = int(m.group(1)) - 1
                if stop_capture[imu]:
                    continue
                c.set_accel_tcal(imu, msg.Value)
                continue
            m = offset_pattern.match(msg.Name)
            if m:
                stype = m.group(1)
                imu = 0 if m.group(2) == "" else int(m.group(2)) - 1
                axis = m.group(3)
                if stop_capture[imu]:
                    continue
                if stype == "ACC":
                    c.set_aoffset(imu, axis, msg.Value)
                if stype == "GYR":
                    c.set_goffset(imu, axis, msg.Value)
                continue
            if msg.Name == "AHRS_ORIENTATION":
                orientation = int(msg.Value)
                logging.info("Using orientation %d", orientation)
                continue

        if msg_type == "TCLR" and tclr:
            imu = msg.I

            temp = msg.Temp
            if msg.SType == 0:
                # accel
                acc = Vector3(msg.X, msg.Y, msg.Z)
                time = msg.TimeUS * 1.0e-6
                data.add_accel(imu, temp, time, acc)
            elif msg.SType == 1:
                # gyro
                gyr = Vector3(msg.X, msg.Y, msg.Z)
                time = msg.TimeUS * 1.0e-6
                data.add_gyro(imu, temp, time, gyr)
            continue

        if msg_type == "IMU" and not tclr:
            imu = msg.I

            if stop_capture[imu]:
                continue

            temp = msg.T
            acc = Vector3(msg.AccX, msg.AccY, msg.AccZ)
            gyr = Vector3(msg.GyrX, msg.GyrY, msg.GyrZ)

            # invert the board orientation rotation. Corrections are in sensor frame
            if orientation != 0:
                acc = acc.rotate_by_inverse_id(orientation)
                gyr = gyr.rotate_by_inverse_id(orientation)
            if acc is None or gyr is None:
                logging.critical("Invalid AHRS_ORIENTATION %d", orientation)
                sys.exit(1)

            if c.enable[imu] == 1:
                acc -= c.correction_accel(imu, temp)
                gyr -= c.correction_gyro(imu, temp)

            time = msg.TimeUS * 1.0e-6
            data.add_accel(imu, temp, time, acc)
            data.add_gyro(imu, temp, time, gyr)

    if len(data.IMUs()) == 0:
        logging.critical("No data found")
        sys.exit(1)

    info_msg = f"Loaded {len(data.accel[0]['T'])} accel and {len(data.gyro[0]['T'])} gyro samples"
    logging.info(info_msg)

    if progress_callback:
        progress_callback(210)
    if not tclr:
        # apply moving average filter with 2s width
        data.filter(2)

    c, clog = generate_calibration_file(outfile, progress_callback, data, c, online)

    if no_graph:
        return
    num_imus = len(data.IMUs())

    generate_tempcal_gyro_figures(figpath, data, c, clog, num_imus, log_parm)

    if progress_callback:
        progress_callback(290)

    generate_tempcal_accel_figures(figpath, data, c, clog, num_imus, log_parm)

    if progress_callback:
        progress_callback(300)

    plt.show()


def generate_calibration_file(  # pylint: disable=too-many-locals
    outfile: str, progress_callback: Union[Callable[[int], None], None], data: IMUData, c: Coefficients, online: bool
) -> tuple[Coefficients, Coefficients]:
    clog = c
    c = Coefficients()

    if progress_callback:
        progress = 220
        progress_callback(progress)
        progress_delta = 60 / (len(data.IMUs()) * len(AXES))
    with open(outfile, "w", encoding="utf-8") as calfile:
        for imu in data.IMUs():
            tmin = np.amin(data.accel[imu]["T"])
            tmax = np.amax(data.accel[imu]["T"])

            c.set_tmin(imu, tmin)
            c.set_tmax(imu, tmax)

            for axis in AXES:
                if online:
                    fit = OnlineIMUfit()
                    trel = data.accel[imu]["T"] - TEMP_REF
                    ofs = data.accel_at_temp(imu, axis, clog.atcal[imu])
                    c.set_accel_poly(imu, axis, fit.polyfit(trel, data.accel[imu][axis] - ofs, POLY_ORDER))
                    trel = data.gyro[imu]["T"] - TEMP_REF
                    c.set_gyro_poly(imu, axis, fit.polyfit(trel, data.gyro[imu][axis], POLY_ORDER))
                else:
                    trel = data.accel[imu]["T"] - TEMP_REF
                    if imu in clog.atcal:
                        ofs = data.accel_at_temp(imu, axis, clog.atcal[imu])
                    else:
                        ofs = np.mean(data.accel[imu][axis])
                    c.set_accel_poly(imu, axis, np.polyfit(trel, data.accel[imu][axis] - ofs, POLY_ORDER))
                    trel = data.gyro[imu]["T"] - TEMP_REF
                    c.set_gyro_poly(imu, axis, np.polyfit(trel, data.gyro[imu][axis], POLY_ORDER))
                if progress_callback:
                    progress += int(progress_delta)
                    progress_callback(progress)

            params = c.param_string(imu)
            print(params)  # noqa: T201
            calfile.write(params)
        # ensure all data is written to disk, so that other processes can safely read the result without race conditions
        calfile.flush()
        if hasattr(os, "fdatasync"):
            os.fdatasync(calfile.fileno())
        else:
            os.fsync(calfile.fileno())  # Use fsync as a fallback

    logging.info("Calibration written to %s", outfile)
    return c, clog


def generate_tempcal_gyro_figures(  # pylint: disable=too-many-arguments, too-many-positional-arguments
    figpath: Union[str, None], data: IMUData, c: Coefficients, clog: Coefficients, num_imus: int, log_parm: bool
) -> None:
    _fig, axs = plt.subplots(len(data.IMUs()), 1, sharex=True)
    if num_imus == 1:
        axs = [axs]

    for imu in data.IMUs():
        scale = math.degrees(1)
        for axis in AXES:
            axs[imu].plot(data.gyro[imu]["time"], data.gyro[imu][axis] * scale, label=f"Uncorrected {axis}")
        for axis in AXES:
            poly = np.poly1d(c.gcoef[imu][axis])
            trel = data.gyro[imu]["T"] - TEMP_REF
            correction = poly(trel)
            axs[imu].plot(data.gyro[imu]["time"], (data.gyro[imu][axis] - correction) * scale, label=f"Corrected {axis}")
        if log_parm:
            for axis in AXES:
                if clog.enable[imu] == 0.0:
                    logging.info("IMU[%d] disabled in log parms", imu)
                    continue
                poly = np.poly1d(clog.gcoef[imu][axis])
                correction = poly(data.gyro[imu]["T"] - TEMP_REF) - poly(clog.gtcal[imu] - TEMP_REF) + clog.gofs[imu][axis]
                axs[imu].plot(
                    data.gyro[imu]["time"], (data.gyro[imu][axis] - correction) * scale, label=f"Corrected {axis} (log parm)"
                )
        ax2 = axs[imu].twinx()
        ax2.plot(data.gyro[imu]["time"], data.gyro[imu]["T"], label="Temperature(C)", color="black")
        ax2.legend(loc="upper right")
        axs[imu].legend(loc="upper left")
        axs[imu].set_title(f"IMU[{imu}] Gyro (deg/s)")

    if figpath:
        _fig.savefig(os.path.join(figpath, "tempcal_gyro.png"))


def generate_tempcal_accel_figures(  # pylint: disable=too-many-arguments, too-many-positional-arguments
    figpath: Union[str, None], data: IMUData, c: Coefficients, clog: Coefficients, num_imus: int, log_parm: bool
) -> None:
    _fig, axs = plt.subplots(num_imus, 1, sharex=True)
    if num_imus == 1:
        axs = [axs]

    for imu in data.IMUs():
        for axis in AXES:
            ofs = data.accel_at_temp(imu, axis, clog.atcal.get(imu, TEMP_REF))
            axs[imu].plot(data.accel[imu]["time"], data.accel[imu][axis] - ofs, label=f"Uncorrected {axis}")
        for axis in AXES:
            poly = np.poly1d(c.acoef[imu][axis])
            trel = data.accel[imu]["T"] - TEMP_REF
            correction = poly(trel)
            ofs = data.accel_at_temp(imu, axis, clog.atcal.get(imu, TEMP_REF))
            axs[imu].plot(data.accel[imu]["time"], (data.accel[imu][axis] - ofs) - correction, label=f"Corrected {axis}")
        if log_parm:
            for axis in AXES:
                if clog.enable[imu] == 0.0:
                    logging.info("IMU[%d] disabled in log parms", imu)
                    continue
                poly = np.poly1d(clog.acoef[imu][axis])
                ofs = data.accel_at_temp(imu, axis, clog.atcal[imu])
                correction = poly(data.accel[imu]["T"] - TEMP_REF) - poly(clog.atcal[imu] - TEMP_REF)
                axs[imu].plot(
                    data.accel[imu]["time"], (data.accel[imu][axis] - ofs) - correction, label=f"Corrected {axis} (log parm)"
                )
        ax2 = axs[imu].twinx()
        ax2.plot(data.accel[imu]["time"], data.accel[imu]["T"], label="Temperature(C)", color="black")
        ax2.legend(loc="upper right")
        axs[imu].legend(loc="upper left")
        axs[imu].set_title(f"IMU[{imu}] Accel (m/s^2)")

    if figpath:
        _fig.savefig(os.path.join(figpath, "tempcal_acc.png"))


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--outfile", default="tcal.parm", help="set output file")
    parser.add_argument("--no-graph", action="store_true", default=False, help="disable graph display")
    parser.add_argument(
        "--log-parm", action="store_true", default=False, help="show corrections using coefficients from log file"
    )
    parser.add_argument("--online", action="store_true", default=False, help="use online polynomial fitting")
    parser.add_argument(
        "--tclr", action="store_true", default=False, help="use TCLR messages from log instead of IMU messages"
    )
    parser.add_argument("log", metavar="LOG")

    args = parser.parse_args()

    logging.basicConfig(level="INFO", format="%(asctime)s - %(levelname)s - %(message)s")
    IMUfit(args.log, args.outfile, args.no_graph, args.log_parm, args.online, args.tclr, None, None)


if __name__ == "__main__":
    main()
