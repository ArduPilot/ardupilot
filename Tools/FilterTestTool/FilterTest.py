# -*- coding: utf-8 -*-

# flake8: noqa

""" ArduPilot IMU Filter Test Class

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
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
from scipy import signal
from BiquadFilter import BiquadFilterType, BiquadFilter

sliders = []  # matplotlib sliders must be global
anim = None  # matplotlib animations must be global


class FilterTest:
    FILTER_DEBOUNCE = 10  # ms

    FILT_SHAPE_DT_FACTOR = 1  # increase to reduce filter shape size

    FFT_N = 512

    filters = {}

    def __init__(self, acc_t, acc_x, acc_y, acc_z, gyr_t, gyr_x, gyr_y, gyr_z, acc_freq, gyr_freq,
                 acc_lpf_cutoff, gyr_lpf_cutoff,
                 acc_notch_freq, acc_notch_att, acc_notch_band,
                 gyr_notch_freq, gyr_notch_att, gyr_notch_band,
                 log_name, accel_notch=False, second_notch=False):

        self.filter_color_map = plt.get_cmap('summer')

        self.filters["acc"] = [
            BiquadFilter(acc_lpf_cutoff, acc_freq)
        ]

        if accel_notch:
            self.filters["acc"].append(
                BiquadFilter(acc_notch_freq, acc_freq, BiquadFilterType.PEAK, acc_notch_att, acc_notch_band),
            )

        self.filters["gyr"] = [
            BiquadFilter(gyr_lpf_cutoff, gyr_freq),
            BiquadFilter(gyr_notch_freq, gyr_freq, BiquadFilterType.PEAK, gyr_notch_att, gyr_notch_band)
        ]

        if second_notch:
            self.filters["acc"].append(
                BiquadFilter(acc_notch_freq * 2, acc_freq, BiquadFilterType.PEAK, acc_notch_att, acc_notch_band)
            )
            self.filters["gyr"].append(
                BiquadFilter(gyr_notch_freq * 2, gyr_freq, BiquadFilterType.PEAK, gyr_notch_att, gyr_notch_band)
            )

        self.ACC_t = acc_t
        self.ACC_x = acc_x
        self.ACC_y = acc_y
        self.ACC_z = acc_z

        self.GYR_t = gyr_t
        self.GYR_x = gyr_x
        self.GYR_y = gyr_y
        self.GYR_z = gyr_z

        self.GYR_freq = gyr_freq
        self.ACC_freq = acc_freq

        self.gyr_dt = 1. / gyr_freq
        self.acc_dt = 1. / acc_freq

        self.timer = None

        self.updated_artists = []

        # INIT
        self.init_plot(log_name)

    def test_acc_filters(self):
        filt_xs = self.test_filters(self.filters["acc"], self.ACC_t, self.ACC_x)
        filt_ys = self.test_filters(self.filters["acc"], self.ACC_t, self.ACC_y)
        filt_zs = self.test_filters(self.filters["acc"], self.ACC_t, self.ACC_z)
        return filt_xs, filt_ys, filt_zs

    def test_gyr_filters(self):
        filt_xs = self.test_filters(self.filters["gyr"], self.GYR_t, self.GYR_x)
        filt_ys = self.test_filters(self.filters["gyr"], self.GYR_t, self.GYR_y)
        filt_zs = self.test_filters(self.filters["gyr"], self.GYR_t, self.GYR_z)
        return filt_xs, filt_ys, filt_zs

    def test_filters(self, filters, Ts, Xs):
        for f in filters:
            f.reset()

        x_filtered = []

        for i, t in enumerate(Ts):
            x = Xs[i]

            x_f = x
            for filt in filters:
                x_f = filt.apply(x_f)

            x_filtered.append(x_f)

        return x_filtered

    def get_filter_shape(self, filter):
        samples = int(filter.get_sample_freq())  # resolution of filter shape based on sample rate
        x_space = np.linspace(0.0, samples // 2, samples // int(2 * self.FILT_SHAPE_DT_FACTOR))
        return x_space, filter.freq_response(x_space)

    def init_signal_plot(self, ax, Ts, Xs, Ys, Zs, Xs_filtered, Ys_filtered, Zs_filtered, label):
        ax.plot(Ts, Xs, linewidth=1, label="{}X".format(label), alpha=0.5)
        ax.plot(Ts, Ys, linewidth=1, label="{}Y".format(label), alpha=0.5)
        ax.plot(Ts, Zs, linewidth=1, label="{}Z".format(label), alpha=0.5)
        filtered_x_ax, = ax.plot(Ts, Xs_filtered, linewidth=1, label="{}X filtered".format(label), alpha=1)
        filtered_y_ax, = ax.plot(Ts, Ys_filtered, linewidth=1, label="{}Y filtered".format(label), alpha=1)
        filtered_z_ax, = ax.plot(Ts, Zs_filtered, linewidth=1, label="{}Z filtered".format(label), alpha=1)
        ax.legend(prop={'size': 8})
        return filtered_x_ax, filtered_y_ax, filtered_z_ax

    def fft_to_xdata(self, fft):
        n = len(fft)
        norm_factor = 2. / n
        return norm_factor * np.abs(fft[:n // 2])

    def plot_fft(self, ax, x, fft, label):
        fft_ax, = ax.plot(x, self.fft_to_xdata(fft), label=label)
        return fft_ax

    def init_fft(self, ax, Ts, Xs, Ys, Zs, sample_rate, dt, Xs_filtered, Ys_filtered, Zs_filtered, label):

        _freqs_raw_x, _times_raw_x, _stft_raw_x = signal.stft(Xs, sample_rate, window='hann', nperseg=self.FFT_N)
        raw_fft_x = np.average(np.abs(_stft_raw_x), axis=1)

        _freqs_raw_y, _times_raw_y, _stft_raw_y = signal.stft(Ys, sample_rate, window='hann', nperseg=self.FFT_N)
        raw_fft_y = np.average(np.abs(_stft_raw_y), axis=1)

        _freqs_raw_z, _times_raw_z, _stft_raw_z = signal.stft(Zs, sample_rate, window='hann', nperseg=self.FFT_N)
        raw_fft_z = np.average(np.abs(_stft_raw_z), axis=1)

        _freqs_x, _times_x, _stft_x = signal.stft(Xs_filtered, sample_rate, window='hann', nperseg=self.FFT_N)
        filtered_fft_x = np.average(np.abs(_stft_x), axis=1)

        _freqs_y, _times_y, _stft_y = signal.stft(Ys_filtered, sample_rate, window='hann', nperseg=self.FFT_N)
        filtered_fft_y = np.average(np.abs(_stft_y), axis=1)

        _freqs_z, _times_z, _stft_z = signal.stft(Zs_filtered, sample_rate, window='hann', nperseg=self.FFT_N)
        filtered_fft_z = np.average(np.abs(_stft_z), axis=1)

        ax.plot(_freqs_raw_x, raw_fft_x, alpha=0.5, linewidth=1, label="{}x FFT".format(label))
        ax.plot(_freqs_raw_y, raw_fft_y, alpha=0.5, linewidth=1, label="{}y FFT".format(label))
        ax.plot(_freqs_raw_z, raw_fft_z, alpha=0.5, linewidth=1, label="{}z FFT".format(label))

        filtered_fft_ax_x, = ax.plot(_freqs_x, filtered_fft_x, label="filt. {}x FFT".format(label))
        filtered_fft_ax_y, = ax.plot(_freqs_y, filtered_fft_y, label="filt. {}y FFT".format(label))
        filtered_fft_ax_z, = ax.plot(_freqs_z, filtered_fft_z, label="filt. {}z FFT".format(label))

        # FFT
        # samples = len(Ts)
        # x_space = np.linspace(0.0, 1.0 / (2.0 * dt), samples // 2)
        # filtered_data = np.hanning(len(Xs_filtered)) * Xs_filtered
        # raw_fft = np.fft.fft(np.hanning(len(Xs)) * Xs)
        # filtered_fft = np.fft.fft(filtered_data, n=self.FFT_N)
        # self.plot_fft(ax, x_space, raw_fft, "{} FFT".format(label))
        # fft_freq = np.fft.fftfreq(self.FFT_N, d=dt)
        # x_space
        # filtered_fft_ax = self.plot_fft(ax, fft_freq[:self.FFT_N // 2], filtered_fft, "filtered {} FFT".format(label))

        ax.set_xlabel("frequency")
        # ax.set_xscale("log")
        # ax.xaxis.set_major_formatter(ScalarFormatter())
        ax.legend(prop={'size': 8})

        return filtered_fft_ax_x, filtered_fft_ax_y, filtered_fft_ax_z

    def init_filter_shape(self, ax, filter, color):
        center = filter.get_center_freq()
        x_space, lpf_shape = self.get_filter_shape(filter)

        plot_slpf_shape, = ax.plot(x_space, lpf_shape, c=color, label="LPF shape")
        xvline_lpf_cutoff = ax.axvline(x=center, linestyle="--", c=color)  # LPF cutoff freq

        return plot_slpf_shape, xvline_lpf_cutoff

    def create_slider(self, name, rect, max, value, color, callback):
        global sliders
        ax_slider = self.fig.add_axes(rect, facecolor='lightgoldenrodyellow')
        slider = Slider(ax_slider, name, 0, max, valinit=np.sqrt(max * value), valstep=1, color=color)
        slider.valtext.set_text(value)

        # slider.drawon = False

        def changed(val, cbk, max, slider):
            # non linear slider to better control small values
            val = int(val ** 2 / max)
            slider.valtext.set_text(val)
            cbk(val)

        slider.on_changed(lambda val, cbk=callback, max=max, s=slider: changed(val, cbk, max, s))
        sliders.append(slider)

    def delay_update(self, update_cbk):
        def _delayed_update(self, cbk):
            self.timer.stop()
            cbk()

        # delay actual filtering
        if self.fig:
            if self.timer:
                self.timer.stop()
            self.timer = self.fig.canvas.new_timer(interval=self.FILTER_DEBOUNCE)
            self.timer.add_callback(lambda self=self: _delayed_update(self, update_cbk))
            self.timer.start()

    def update_filter_shape(self, filter, shape, center_line):
        x_data, new_shape = self.get_filter_shape(filter)

        shape.set_ydata(new_shape)
        center_line.set_xdata(filter.get_center_freq())

        self.updated_artists.extend([
            shape,
            center_line,
        ])

    def update_signal_and_fft_plot(self, filters_key, time_list, sample_lists, signal_shapes, fft_shapes, shape,
                                   center_line, sample_rate):
        # print("update_signal_and_fft_plot", self.filters[filters_key][0].get_center_freq())
        Xs, Ys, Zs = sample_lists
        signal_shape_x, signal_shape_y, signal_shape_z = signal_shapes
        fft_shape_x, fft_shape_y, fft_shape_z = fft_shapes

        Xs_filtered = self.test_filters(self.filters[filters_key], time_list, Xs)
        Ys_filtered = self.test_filters(self.filters[filters_key], time_list, Ys)
        Zs_filtered = self.test_filters(self.filters[filters_key], time_list, Zs)

        signal_shape_x.set_ydata(Xs_filtered)
        signal_shape_y.set_ydata(Ys_filtered)
        signal_shape_z.set_ydata(Zs_filtered)

        self.updated_artists.extend([signal_shape_x, signal_shape_y, signal_shape_z])

        _freqs_x, _times_x, _stft_x = signal.stft(Xs_filtered, sample_rate, window='hann', nperseg=self.FFT_N)
        filtered_fft_x = np.average(np.abs(_stft_x), axis=1)

        _freqs_y, _times_y, _stft_y = signal.stft(Ys_filtered, sample_rate, window='hann', nperseg=self.FFT_N)
        filtered_fft_y = np.average(np.abs(_stft_y), axis=1)

        _freqs_z, _times_z, _stft_z = signal.stft(Zs_filtered, sample_rate, window='hann', nperseg=self.FFT_N)
        filtered_fft_z = np.average(np.abs(_stft_z), axis=1)

        fft_shape_x.set_ydata(filtered_fft_x)
        fft_shape_y.set_ydata(filtered_fft_y)
        fft_shape_z.set_ydata(filtered_fft_z)

        self.updated_artists.extend([
            fft_shape_x, fft_shape_y, fft_shape_z,
            shape, center_line,
        ])

        # self.fig.canvas.draw()

    def animation_update(self):
        updated_artists = self.updated_artists.copy()

        # if updated_artists:
        #    print("animation update")

        # reset updated artists
        self.updated_artists = []

        return updated_artists

    def update_filter(self, val, cbk, filter, shape, center_line, filters_key, time_list, sample_lists, signal_shapes,
                      fft_shapes):
        # this callback sets the parameter controlled by the slider
        cbk(val)
        # print("filter update",val)
        # update filter shape and delay fft update
        self.update_filter_shape(filter, shape, center_line)
        sample_freq = filter.get_sample_freq()
        self.delay_update(
            lambda self=self: self.update_signal_and_fft_plot(filters_key, time_list, sample_lists, signal_shapes,
                                                              fft_shapes, shape, center_line, sample_freq))

    def create_filter_control(self, name, filter, rect, max, default, shape, center_line, cbk, filters_key, time_list,
                              sample_lists, signal_shapes, fft_shapes, filt_color):
        self.create_slider(name, rect, max, default, filt_color, lambda val, cbk=cbk, self=self, filter=filter, shape=shape,
                                                                        center_line=center_line, filters_key=filters_key,
                                                                        time_list=time_list, sample_list=sample_lists,
                                                                        signal_shape=signal_shapes, fft_shape=fft_shapes:
                                                        self.update_filter(val, cbk, filter, shape, center_line, filters_key,
                                                                           time_list, sample_list, signal_shape, fft_shape))

    def create_controls(self, filters_key, base_rect, padding, ax_fft, time_list, sample_lists, signal_shapes,
                        fft_shapes):
        ax_filter = ax_fft.twinx()
        ax_filter.set_navigate(False)
        ax_filter.set_yticks([])

        num_filters = len(self.filters[filters_key])

        for i, filter in enumerate(self.filters[filters_key]):
            filt_type = filter.get_type()
            filt_color = self.filter_color_map(i / num_filters)
            filt_shape, filt_cutoff = self.init_filter_shape(ax_filter, filter, filt_color)

            if filt_type == BiquadFilterType.PEAK:
                name = "Notch"
            else:
                name = "LPF"

            # control for center freq is common to all filters
            self.create_filter_control("{} freq".format(name), filter, base_rect, 500, filter.get_center_freq(),
                                       filt_shape, filt_cutoff,
                                       lambda val, filter=filter: filter.set_center_freq(val),
                                       filters_key, time_list, sample_lists, signal_shapes, fft_shapes, filt_color)
            # move down of control height + padding
            base_rect[1] -= (base_rect[3] + padding)

            if filt_type == BiquadFilterType.PEAK:
                self.create_filter_control("{} att (db)".format(name), filter, base_rect, 100, filter.get_attenuation(),
                                           filt_shape, filt_cutoff,
                                           lambda val, filter=filter: filter.set_attenuation(val),
                                           filters_key, time_list, sample_lists, signal_shapes, fft_shapes, filt_color)
                base_rect[1] -= (base_rect[3] + padding)
                self.create_filter_control("{} band".format(name), filter, base_rect, 300, filter.get_bandwidth(),
                                           filt_shape, filt_cutoff,
                                           lambda val, filter=filter: filter.set_bandwidth(val),
                                           filters_key, time_list, sample_lists, signal_shapes, fft_shapes, filt_color)
                base_rect[1] -= (base_rect[3] + padding)

    def create_spectrogram(self, data, name, sample_rate):
        freqs, times, Sx = signal.spectrogram(np.array(data), fs=sample_rate, window='hanning',
                                              nperseg=self.FFT_N, noverlap=self.FFT_N - self.FFT_N // 10,
                                              detrend=False, scaling='spectrum')

        f, ax = plt.subplots(figsize=(4.8, 2.4))
        ax.pcolormesh(times, freqs, 10 * np.log10(Sx), cmap='viridis')
        ax.set_title(name)
        ax.set_ylabel('Frequency (Hz)')
        ax.set_xlabel('Time (s)')

    def init_plot(self, log_name):

        self.fig = plt.figure(figsize=(14, 9))
        self.fig.canvas.set_window_title("ArduPilot Filter Test Tool - {}".format(log_name))
        self.fig.canvas.draw()

        rows = 2
        cols = 3
        raw_acc_index = 1
        fft_acc_index = raw_acc_index + 1
        raw_gyr_index = cols + 1
        fft_gyr_index = raw_gyr_index + 1

        # signal
        self.ax_acc = self.fig.add_subplot(rows, cols, raw_acc_index)
        self.ax_gyr = self.fig.add_subplot(rows, cols, raw_gyr_index, sharex=self.ax_acc)

        accx_filtered, accy_filtered, accz_filtered = self.test_acc_filters()
        self.ax_filtered_accx, self.ax_filtered_accy, self.ax_filtered_accz = self.init_signal_plot(self.ax_acc,
                                                                                                    self.ACC_t,
                                                                                                    self.ACC_x,
                                                                                                    self.ACC_y,
                                                                                                    self.ACC_z,
                                                                                                    accx_filtered,
                                                                                                    accy_filtered,
                                                                                                    accz_filtered,
                                                                                                    "AccX")

        gyrx_filtered, gyry_filtered, gyrz_filtered = self.test_gyr_filters()
        self.ax_filtered_gyrx, self.ax_filtered_gyry, self.ax_filtered_gyrz = self.init_signal_plot(self.ax_gyr,
                                                                                                    self.GYR_t,
                                                                                                    self.GYR_x,
                                                                                                    self.GYR_y,
                                                                                                    self.GYR_z,
                                                                                                    gyrx_filtered,
                                                                                                    gyry_filtered,
                                                                                                    gyrz_filtered,
                                                                                                    "GyrX")

        # FFT
        self.ax_acc_fft = self.fig.add_subplot(rows, cols, fft_acc_index)
        self.ax_gyr_fft = self.fig.add_subplot(rows, cols, fft_gyr_index)

        self.acc_filtered_fft_ax_x, self.acc_filtered_fft_ax_y, self.acc_filtered_fft_ax_z = self.init_fft(
            self.ax_acc_fft, self.ACC_t, self.ACC_x, self.ACC_y, self.ACC_z, self.ACC_freq, self.acc_dt, accx_filtered,
            accy_filtered, accz_filtered, "AccX")
        self.gyr_filtered_fft_ax_x, self.gyr_filtered_fft_ax_y, self.gyr_filtered_fft_ax_z = self.init_fft(
            self.ax_gyr_fft, self.GYR_t, self.GYR_x, self.GYR_y, self.GYR_z, self.GYR_freq, self.gyr_dt, gyrx_filtered,
            gyry_filtered, gyrz_filtered, "GyrX")

        self.fig.tight_layout()

        # TODO add y z
        self.create_controls("acc", [0.75, 0.95, 0.2, 0.02], 0.01, self.ax_acc_fft, self.ACC_t,
                             (self.ACC_x, self.ACC_y, self.ACC_z),
                             (self.ax_filtered_accx, self.ax_filtered_accy, self.ax_filtered_accz),
                             (self.acc_filtered_fft_ax_x, self.acc_filtered_fft_ax_y, self.acc_filtered_fft_ax_z))
        self.create_controls("gyr", [0.75, 0.45, 0.2, 0.02], 0.01, self.ax_gyr_fft, self.GYR_t,
                             (self.GYR_x, self.GYR_y, self.GYR_z),
                             (self.ax_filtered_gyrx, self.ax_filtered_gyry, self.ax_filtered_gyrz),
                             (self.gyr_filtered_fft_ax_x, self.gyr_filtered_fft_ax_y, self.gyr_filtered_fft_ax_z))

        # setup animation for continuous update
        global anim
        anim = FuncAnimation(self.fig, lambda frame, self=self: self.animation_update(), interval=1, blit=False)

        # Work in progress here...
        # self.create_spectrogram(self.GYR_x, "GyrX", self.GYR_freq)
        # self.create_spectrogram(gyrx_filtered, "GyrX filtered", self.GYR_freq)
        # self.create_spectrogram(self.ACC_x, "AccX", self.ACC_freq)
        # self.create_spectrogram(accx_filtered, "AccX filtered", self.ACC_freq)

        plt.show()

        self.print_filter_param_info()

    def print_filter_param_info(self):
        if len(self.filters["acc"]) > 2 or len(self.filters["gyr"]) > 2:
            print("Testing too many filters unsupported from firmware, cannot calculate parameters to set them")
            return

        print("To have the last filter settings in the graphs set the following parameters:\n")

        for f in self.filters["acc"]:
            filt_type = f.get_type()

            if filt_type == BiquadFilterType.PEAK:  # NOTCH
                print("INS_NOTCA_ENABLE,", 1)
                print("INS_NOTCA_FREQ,", f.get_center_freq())
                print("INS_NOTCA_BW,", f.get_bandwidth())
                print("INS_NOTCA_ATT,", f.get_attenuation())
            else:  # LPF
                print("INS_ACCEL_FILTER,", f.get_center_freq())

        for f in self.filters["gyr"]:
            filt_type = f.get_type()

            if filt_type == BiquadFilterType.PEAK:  # NOTCH
                print("INS_HNTC2_ENABLE,", 1)
                print("INS_HNTC2_FREQ,", f.get_center_freq())
                print("INS_HNTC2_BW,", f.get_bandwidth())
                print("INS_HNTC2_ATT,", f.get_attenuation())
            else:  # LPF
                print("INS_GYRO_FILTER,", f.get_center_freq())

        print("\n+---------+")
        print("| WARNING |")
        print("+---------+")
        print("Always check the onboard FFT to setup filters, this tool only simulate effects of filtering.")
