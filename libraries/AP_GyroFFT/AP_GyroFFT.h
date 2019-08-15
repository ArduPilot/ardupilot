/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Code by Andy Piper
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED HAL_WITH_DSP
#endif

#if HAL_GYROFFT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <Filter/LowPassFilter.h>

#define DEBUG_FFT   0

// a library that leverages the HAL DSP support to perform FFT analysis on gyro samples
class AP_GyroFFT
{
public:
    AP_GyroFFT();

    // Do not allow copies
    AP_GyroFFT(const AP_GyroFFT &other) = delete;
    AP_GyroFFT &operator=(const AP_GyroFFT&) = delete;

    void init(uint32_t target_looptime);

    // cycle through the FFT steps - runs at 400Hz
    uint16_t update();
    // capture gyro values at the appropriate update rate - runs at fast loop rate
    void sample_gyros();
    // update calculated values of dynamic parameters - runs at 1Hz
    void update_parameters();
    // thread for processing gyro data via FFT
    void update_thread();
    // start the update thread
    void start_update_thread();

    // check at startup that standard frequencies can be detected
    bool calibration_check();
    // called when hovering to determine the average peak frequency and reference value
    void update_freq_hover(float dt, float throttle_out);
    // called to save the average peak frequency and reference value
    void save_params_on_disarm();
    // dynamically enable or disable the analysis through the aux switch
    void set_analysis_enabled(bool enabled) { _analysis_enabled = enabled; };

    // detected peak frequency filtered at 1/3 the update rate
    Vector3f get_noise_center_freq_hz() const { return _global_state._center_freq_hz_filtered; }
    // energy of the background noise at the detected center frequency
    Vector3f get_noise_signal_to_noise_db() const { return _global_state._center_snr; }
    // detected peak frequency weighted by energy
    float get_weighted_noise_center_freq_hz();
    // detected peak frequency
    Vector3f get_raw_noise_center_freq_hz() const { return _global_state._center_freq_hz; }
    // match between first and second harmonics
    Vector3f get_raw_noise_harmonic_fit() const { return _global_state._harmonic_fit; }
    // energy of the detected peak frequency
    Vector3f get_center_freq_energy() const { return _global_state._center_freq_energy; }
    // index of the FFT bin containing the detected peak frequency
    Vector3<uint16_t> get_center_freq_bin() const { return _global_state._center_freq_bin; }
    // detected peak bandwidth
    Vector3f get_noise_center_bandwidth_hz() const { return _global_state._center_bandwidth_hz; };
    // weighted detected peak bandwidth
    float get_weighted_noise_center_bandwidth_hz();
    // log gyro fft messages
    void write_log_messages();

    static const struct AP_Param::GroupInfo var_info[];
    static AP_GyroFFT *get_singleton() { return _singleton; }

private:
    // calculate the peak noise frequency
    void calculate_noise(uint16_t max_bin);
    // update the estimation of the background noise energy
    void update_ref_energy(uint16_t max_bin);
    // test frequency detection for all of the allowable bins
    float self_test_bin_frequencies();
    // detect the provided frequency
    float self_test(float frequency, GyroWindow test_window);
    // whether to run analysis or not
    bool analysis_enabled() const { return _initialized && _analysis_enabled && _thread_created; };
    // whether analysis can be run again or not
    bool start_analysis();
    // the size of the ring buffer in both the IMU backend
    uint16_t get_buffer_size() const { return _state->_window_size + INS_MAX_GYRO_WINDOW_SAMPLES; }
    // semaphore for access to shared FFT data
    HAL_Semaphore _sem;

    // data accessible from the main thread protected by the semaphore
    struct EngineState {
        // energy of the detected peak frequency
        Vector3f _center_freq_energy;
        // energy of the detected peak frequency in dB
        Vector3f _center_freq_energy_db;
        // detected peak frequency
        Vector3f _center_freq_hz;
        // fit between first and second harmonics
        Vector3f _harmonic_fit;
        // bin of detected poeak frequency
        Vector3<uint16_t> _center_freq_bin;
        // filtered version of the peak frequency
        Vector3f _center_freq_hz_filtered;
        // signal to noise ratio of PSD at the detected centre frequency
        Vector3f _center_snr;
        // detected peak width
        Vector3f _center_bandwidth_hz;
        // axes that still require noise calibration
        uint8_t _noise_needs_calibration : 3;
        // whether the analyzer is mid-cycle
        bool _analysis_started = false;
    };

    // Shared FFT engine state local to the FFT thread
    EngineState _thread_state;
    // Shared FFT engine state accessible by the main thread
    EngineState _global_state;

    // configuration data local to the FFT thread but set from the main thread
    struct EngineConfig {
        // whether the analyzer should be run
        bool _analysis_enabled;
        // minimum frequency of the detection window
        uint16_t _fft_min_hz;
        // maximum frequency of the detection window
        uint16_t _fft_max_hz;
        // configured start bin based on min hz
        uint16_t _fft_start_bin;
        // configured end bin based on max hz
        uint16_t _fft_end_bin;
        // attenuation cutoff for calculation of hover bandwidth
        float _attenuation_cutoff;
        // SNR Threshold
        float _snr_threshold_db;
    } _config;

    // number of sampeles needed before a new frame can be processed
    uint16_t _samples_per_frame;
    // downsampled gyro data circular buffer index frequency analysis
    uint16_t _circular_buffer_idx;
    // number of collected unprocessed gyro samples
    uint16_t _sample_count;

    // downsampled gyro data circular buffer for frequency analysis
    float* _downsampled_gyro_data[XYZ_AXIS_COUNT];
    // accumulator for sampled gyro data
    Vector3f _oversampled_gyro_accum;
    // count of oversamples
    uint16_t _oversampled_gyro_count;
    // current write index in the buffer
    uint16_t _downsampled_gyro_idx;

    // state of the FFT engine
    AP_HAL::DSP::FFTWindowState* _state;
    // update state machine step information
    uint8_t _update_axis;
    // noise base of the gyros
    Vector3f* _ref_energy;
    // the number of cycles required to have a proper noise reference
    uint16_t _noise_cycles;
    // number of cycles over which to generate noise ensemble averages
    uint16_t _noise_calibration_cycles[XYZ_AXIS_COUNT];
    // current _sample_mode
    uint8_t _current_sample_mode : 3;
    // harmonic multiplier for two highest peaks
    float _harmonic_multiplier;
    // searched harmonics - inferred from harmonic notch harmoncis
    uint8_t _harmonics;

    // smoothing filter on the output
    LowPassFilterFloat _center_freq_filter[XYZ_AXIS_COUNT];
    // smoothing filter on the bandwidth
    LowPassFilterFloat _center_bandwidth_filter[XYZ_AXIS_COUNT];

    // configured sampling rate
    uint16_t _fft_sampling_rate_hz;
    // number of cycles without a detected signal
    uint8_t _missed_cycles;
    // whether the analyzer initialized correctly
    bool _initialized;

    // whether the analyzer should be run
    bool _analysis_enabled = true;
    // whether the update thread was created
    bool _thread_created = false;
    // minimum frequency of the detection window
    AP_Int16 _fft_min_hz;
    // maximum frequency of the detection window
    AP_Int16 _fft_max_hz;
    // size of the FFT window
    AP_Int16 _window_size;
    // percentage overlap of FFT windows
    AP_Float _window_overlap;
    // overall enablement of the feature
    AP_Int8 _enable;
    // gyro rate sampling or cycle divider
    AP_Int8 _sample_mode;
    // learned throttle reference for the hover frequency
    AP_Float _throttle_ref;
    // learned hover filter frequency
    AP_Float _freq_hover_hz;
    // SNR Threshold
    AP_Float _snr_threshold_db;
    // attenuation to use for calculating the peak bandwidth at hover
    AP_Float _attenuation_power_db;
    // learned peak bandwidth at configured attenuation at hover
    AP_Float _bandwidth_hover_hz;
    AP_InertialSensor* _ins;
#if DEBUG_FFT
    uint32_t _last_output_ms;
    EngineState _debug_state;
    float _debug_max_bin_freq;
    float _debug_max_freq_bin;
    uint16_t _debug_max_bin;
    float _debug_snr;
#endif

    static AP_GyroFFT *_singleton;
};

namespace AP {
    AP_GyroFFT *fft();
};

#endif // HAL_GYROFFT_ENABLED