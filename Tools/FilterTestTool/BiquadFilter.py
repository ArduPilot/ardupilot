#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" ArduPilot BiquadFilter

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

import numpy as np


class DigitalLPF:
    def __init__(self, cutoff_freq, sample_freq):
        self._cutoff_freq = cutoff_freq
        self._sample_freq = sample_freq

        self._output = 0

        self.compute_alpha()

    def compute_alpha(self):

        if self._cutoff_freq <= 0 or self._sample_freq <= 0:
            self.alpha = 1.
        else:
            dt = 1. / self._sample_freq
            rc = 1. / (np.pi * 2 * self._cutoff_freq)
            a = dt / (dt + rc)
            self.alpha = np.clip(a, 0, 1)

    def apply(self, sample):
        self._output += (sample - self._output) * self.alpha
        return self._output


class BiquadFilterType:
    LPF = 0
    PEAK = 1
    NOTCH = 2


class BiquadFilter:

    def __init__(self, center_freq, sample_freq, type=BiquadFilterType.LPF, attenuation=10, bandwidth=15):
        self._center_freq = int(center_freq)
        self._attenuation_db = int(attenuation)  # used only by notch, use setter
        self._bandwidth_hz = int(bandwidth)  # used only by notch, use setter

        self._sample_freq = sample_freq
        self._type = type

        self._delayed_sample1 = 0
        self._delayed_sample2 = 0
        self._delayed_output1 = 0
        self._delayed_output2 = 0

        self.b0 = 0.
        self.b1 = 0.
        self.b2 = 0.
        self.a0 = 1
        self.a1 = 0.
        self.a2 = 0.

        self.compute_params()

    def get_sample_freq(self):
        return self._sample_freq

    def reset(self):
        self._delayed_sample1 = 0
        self._delayed_sample2 = 0
        self._delayed_output1 = 0
        self._delayed_output2 = 0

    def get_type(self):
        return self._type

    def set_attenuation(self, attenuation_db):
        self._attenuation_db = int(attenuation_db)
        self.compute_params()

    def set_bandwidth(self, bandwidth_hz):
        self._bandwidth_hz = int(bandwidth_hz)
        self.compute_params()

    def set_center_freq(self, cutoff_freq):
        self._center_freq = int(cutoff_freq)
        self.compute_params()

    def compute_params(self):

        omega = 2 * np.pi * self._center_freq / self._sample_freq
        sin_om = np.sin(omega)
        cos_om = np.cos(omega)

        if self._type == BiquadFilterType.LPF:

            if self._center_freq > 0:
                Q = 1 / np.sqrt(2)
                alpha = sin_om / (2 * Q)

                self.b0 = (1 - cos_om) / 2
                self.b1 = 1 - cos_om
                self.b2 = self.b0
                self.a0 = 1 + alpha
                self.a1 = -2 * cos_om
                self.a2 = 1 - alpha

        elif self._type == BiquadFilterType.PEAK:

            A = 10 ** (-self._attenuation_db / 40)

            # why not the formula below? It prevents a division by 0 when bandwidth = 2*frequency
            octaves = np.log2(self._center_freq / (self._center_freq - self._bandwidth_hz / 2)) * 2
            Q = np.sqrt(2 ** octaves) / (2 ** octaves - 1)

            # Q = self._center_freq / self._bandwidth_hz

            alpha = sin_om / (2 * Q / A)

            self.b0 = 1.0 + alpha * A
            self.b1 = -2.0 * cos_om
            self.b2 = 1.0 - alpha * A
            self.a0 = 1.0 + alpha / A
            self.a1 = -2.0 * cos_om
            self.a2 = 1.0 - alpha / A

        elif self._type == BiquadFilterType.NOTCH:
            alpha = sin_om * np.sinh(np.log(2) / 2 * self._bandwidth_hz * omega * sin_om)

            self.b0 = 1
            self.b1 = -2 * cos_om
            self.b2 = self.b0
            self.a0 = 1 + alpha
            self.a1 = -2 * cos_om
            self.a2 = 1 - alpha

        self.b0 /= self.a0
        self.b1 /= self.a0
        self.b2 /= self.a0
        self.a1 /= self.a0
        self.a2 /= self.a0

    def apply(self, sample):

        if self._center_freq <= 0:
            return sample

        output = (self.b0 * sample + self.b1 * self._delayed_sample1 + self.b2 * self._delayed_sample2 - self.a1
                  * self._delayed_output1 - self.a2 * self._delayed_output2)

        self._delayed_sample2 = self._delayed_sample1
        self._delayed_sample1 = sample

        self._delayed_output2 = self._delayed_output1
        self._delayed_output1 = output

        return output

    def get_params(self):

        return {
            "a1": self.a1,
            "a2": self.a2,
            "b0": self.b0,
            "b1": self.b1,
            "b2": self.b2,
        }

    def get_center_freq(self):
        return self._center_freq

    def get_attenuation(self):
        return self._attenuation_db

    def get_bandwidth(self):
        return self._bandwidth_hz

    def freq_response(self, f):
        if self._center_freq <= 0:
            return 1

        phi = (np.sin(np.pi * f * 2 / (2 * self._sample_freq))) ** 2
        r = (((self.b0 + self.b1 + self.b2) ** 2 - 4 * (self.b0 * self.b1 + 4 * self.b0 * self.b2 + self.b1 * self.b2)
              * phi + 16 * self.b0 * self.b2 * phi * phi)
             / ((1 + self.a1 + self.a2) ** 2 - 4 * (self.a1 + 4 * self.a2 + self.a1 * self.a2) * phi + 16
                * self.a2 * phi * phi))
        # if r < 0:
        #    r = 0
        return r ** .5
