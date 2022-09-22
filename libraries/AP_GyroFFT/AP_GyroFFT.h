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

#if HAL_GYROFFT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <Filter/LowPassFilter.h>
#include <Filter/FilterWithBuffer.h>

#define DEBUG_FFT   0

// a library that leverages the HAL DSP support to perform FFT analysis on gyro samples
class AP_GyroFFT
{
    friend class ReplayGyroFFT;

public:
    typedef AP_HAL::DSP::FrequencyPeak FrequencyPeak;

    enum class Options : uint32_t {
        FFTPostFilter = 1 << 0,
        ESCNoiseCheck = 1 << 1
    };

    AP_GyroFFT();

    // Do not allow copies
    CLASS_NO_COPY(AP_GyroFFT);

    void init(uint16_t loop_rate_hz);

    // cycle through the FFT steps - runs in the FFT thread
    uint16_t run_cycle();
    // capture gyro values at the appropriate update rate - runs at fast loop rate
    void sample_gyros();
    // update the engine state - runs at 400Hz
    void update();
    // update calculated values of dynamic parameters - runs at 1Hz
    void update_parameters() { update_parameters(false); }
    // thread for processing gyro data via FFT
    void update_thread();
    // start the update thread
    bool start_update_thread();
    // is the subsystem enabled
    bool enabled() const { return _enable; }

    // check at startup that standard frequencies can be detected
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len);
    // make sure calibration is set done
    bool prepare_for_arming();
    // called when hovering to determine the average peak frequency and reference value
    void update_freq_hover(float dt, float throttle_out);
    // called to save the average peak frequency and reference value
    void save_params_on_disarm();
    // dynamically enable or disable the analysis through the aux switch
    void set_analysis_enabled(bool enabled) { _analysis_enabled = enabled; };
    // notch tuning
    void start_notch_tune();
    void stop_notch_tune();

    // detected peak frequency filtered at 1/3 the update rate
    const Vector3f& get_noise_center_freq_hz() const { return get_noise_center_freq_hz(FrequencyPeak::CENTER); }
    const Vector3f& get_noise_center_freq_hz(FrequencyPeak peak) const { return _global_state._center_freq_hz_filtered[peak]; }
    // frequency values
    float get_weighted_freq_hz(FrequencyPeak peak) const;
    // energy of the background noise at the detected center frequency
    const Vector3f& get_noise_signal_to_noise_db() const { return get_noise_signal_to_noise_db(FrequencyPeak::CENTER); }
    const Vector3f& get_noise_signal_to_noise_db(FrequencyPeak peak) const { return _global_state._center_freq_snr[peak];; }
    // detected peak frequency weighted by energy
    float get_weighted_noise_center_freq_hz() const;
    // all detected peak frequencies weighted by energy
    uint8_t get_weighted_noise_center_frequencies_hz(uint8_t num_freqs, float* freqs) const;
    // detected peak frequency
    const Vector3f& get_raw_noise_center_freq_hz() const { return _global_state._center_freq_hz; }
    // match between first and second harmonics
    const Vector3f& get_raw_noise_harmonic_fit() const { return _global_state._harmonic_fit; }
    // energy of the detected peak frequency
    const Vector3f& get_center_freq_energy() const { return get_center_freq_energy(FrequencyPeak::CENTER); }
    const Vector3f& get_center_freq_energy(FrequencyPeak peak) const { return _global_state._center_freq_energy_filtered[peak]; }
    // index of the FFT bin containing the detected peak frequency
    const Vector3<uint16_t>& get_center_freq_bin() const { return _global_state._center_freq_bin; }
    // detected peak bandwidth
    const Vector3f& get_noise_center_bandwidth_hz() const { return get_noise_center_bandwidth_hz(FrequencyPeak::CENTER); }
    const Vector3f& get_noise_center_bandwidth_hz(FrequencyPeak peak) const { return _global_state._center_bandwidth_hz_filtered[peak]; };
    // weighted detected peak bandwidth
    float get_weighted_noise_center_bandwidth_hz() const;
    // log gyro fft messages
    void write_log_messages();
    // post filter mask of IMUs
    bool using_post_filter_samples() const { return (_options & uint32_t(Options::FFTPostFilter)) != 0; }
    // post filter mask of IMUs
    bool check_esc_noise() const { return (_options & uint32_t(Options::ESCNoiseCheck)) != 0; }
    // look for a frequency in the detected noise
    float has_noise_at_frequency_hz(float freq) const;

    static const struct AP_Param::GroupInfo var_info[];
    static AP_GyroFFT *get_singleton() { return _singleton; }

private:
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

    // smoothing filter that first takes the median from a sliding window and then
    // applies a low pass filter to the result
    class MedianLowPassFilter3dFloat {
    public:
        MedianLowPassFilter3dFloat() { }

        float apply(uint8_t axis, float sample);
        float get(uint8_t axis) const { return _lowpass_filter[axis].get(); }

        void set_cutoff_frequency(float sample_freq, float cutoff_freq) {
            for (uint8_t i = 0; i < XYZ_AXIS_COUNT; i++) {
                _lowpass_filter[i].set_cutoff_frequency(sample_freq, cutoff_freq);
            }
        }

    private:
        LowPassFilterFloat _lowpass_filter[XYZ_AXIS_COUNT];
        FilterWithBuffer<float,3> _median_filter[XYZ_AXIS_COUNT];
    };

    // structure for holding noise peak data while calculating swaps
    class FrequencyData {
    public:
        FrequencyData(const AP_GyroFFT& gyrofft, const EngineConfig& config);
        float get_weighted_frequency(FrequencyPeak i) const { return frequency[i]; }
        float get_signal_to_noise(FrequencyPeak i) const { return snr[i]; }
        bool is_valid(FrequencyPeak i) const { return valid[i]; }
    private:
        float frequency[FrequencyPeak::MAX_TRACKED_PEAKS];
        float snr[FrequencyPeak::MAX_TRACKED_PEAKS];
        bool valid[FrequencyPeak::MAX_TRACKED_PEAKS];
    };
    // distance matrix between filtered and instantaneous peaks
    typedef float DistanceMatrix[FrequencyPeak::MAX_TRACKED_PEAKS][FrequencyPeak::MAX_TRACKED_PEAKS];

    // thread-local accessors of filtered state
    float get_tl_noise_center_freq_hz(FrequencyPeak peak, uint8_t axis) const { return _thread_state._center_freq_hz_filtered[peak][axis]; }
    float get_tl_center_freq_energy(FrequencyPeak peak, uint8_t axis) const { return _thread_state._center_freq_energy_filtered[peak][axis]; }
    float get_tl_noise_center_bandwidth_hz(FrequencyPeak peak, uint8_t axis) const { return _thread_state._center_bandwidth_hz_filtered[peak][axis]; };
    // thread-local mutators of filtered state
    float update_tl_noise_center_freq_hz(FrequencyPeak peak, uint8_t axis, float value) {
        return (_thread_state._center_freq_hz_filtered[peak][axis] = _center_freq_filter[peak].apply(axis, value));
    }
    float update_tl_center_freq_energy(FrequencyPeak peak, uint8_t axis, float value) {
        return (_thread_state._center_freq_energy_filtered[peak][axis] = _center_freq_energy_filter[peak].apply(axis, value));
    }
    float update_tl_noise_center_bandwidth_hz(FrequencyPeak peak, uint8_t axis, float value) {
        return (_thread_state._center_bandwidth_hz_filtered[peak][axis] = _center_bandwidth_filter[peak].apply(axis, value));
    }
    // write single log mesages
    void log_noise_peak(uint8_t id, FrequencyPeak peak) const;
    // calculate the peak noise frequency
    void calculate_noise(bool calibrating, const EngineConfig& config);
    // calculate noise peaks based on energy and history
    uint8_t calculate_tracking_peaks(float& weighted_peak_freq_hz, bool calibrating, const EngineConfig& config);
    uint8_t calculate_tracking_peaks(float& weighted_center_freq_hz, const FrequencyData& freqs, const EngineConfig& config);
    // calculate noise peak frequency characteristics
    bool calculate_filtered_noise(FrequencyPeak target_peak, FrequencyPeak source_peak, const FrequencyData& freqs, const EngineConfig& config);
    void update_snr_values(const FrequencyData& freqs);
    // get the weighted frequency
    bool get_weighted_frequency(FrequencyPeak peak, float& weighted_peak_freq_hz, float& snr, const EngineConfig& config) const;
    // return the tracked noise peak
    FrequencyPeak get_tracked_noise_peak() const;
    // calculate the distance matrix between the current estimates and the current cycle
    void find_distance_matrix(DistanceMatrix& distance_matrix, const FrequencyData& freqs, const EngineConfig& config) const;
    // return the instantaneous peak that is closest to the target estimate peak
    FrequencyPeak find_closest_peak(const FrequencyPeak target, const DistanceMatrix& distance_matrix, uint8_t ignore = 0) const;
    // detected peak frequency weighted by energy
    float calculate_weighted_freq_hz(const Vector3f& energy, const Vector3f& freq) const;
    // update the estimation of the background noise energy
    void update_ref_energy(uint16_t max_bin);
    // test frequency detection for all of the allowable bins
    float self_test_bin_frequencies();
    // detect the provided frequency
    float self_test(float frequency, FloatBuffer& test_window);
    // whether to run analysis or not
    bool analysis_enabled() const { return _initialized && _analysis_enabled && _thread_created; };
    // whether analysis can be run again or not
    bool start_analysis();
    // return samples available in the gyro window
    uint16_t get_available_samples(uint8_t axis) {
        return _sample_mode == 0 ?_ins->get_raw_gyro_window(axis).available() : _downsampled_gyro_data[axis].available();
    }
    void update_parameters(bool force);
    // semaphore for access to shared FFT data
    HAL_Semaphore _sem;

    // data set from the FFT thread but accessible from the main thread protected by the semaphore
    struct EngineState {
        // energy of the detected peak frequency in dB
        Vector3f _center_freq_energy_db;
        // detected peak frequency
        Vector3f _center_freq_hz;
        // fit between first and second harmonics
        Vector3f _harmonic_fit;
        // bin of detected peak frequency
        Vector3ui _center_freq_bin;
        // fft engine health
        Vector3<uint8_t> _health;
        Vector3ul _health_ms;
        // fft engine output rate
        uint32_t _output_cycle_ms;
        // tracked frequency peak for the purposes of notching
        Vector3<uint8_t> _tracked_peak;
        // center frequency peak ignoring temporary energy changes / order switching
        Vector3<uint8_t> _center_peak;
        // signal to noise ratio of PSD at each of the detected centre frequencies
        Vector3f _center_freq_snr[FrequencyPeak::MAX_TRACKED_PEAKS];
        // filtered version of the peak frequency
        Vector3f _center_freq_hz_filtered[FrequencyPeak::MAX_TRACKED_PEAKS];
        // when we last calculated a value
        Vector3ul _last_output_us;
        // filtered energy of the detected peak frequency
        Vector3f _center_freq_energy_filtered[FrequencyPeak::MAX_TRACKED_PEAKS];
        // filtered detected peak width
        Vector3f _center_bandwidth_hz_filtered[FrequencyPeak::MAX_TRACKED_PEAKS];
        // axes that still require noise calibration
        uint8_t _noise_needs_calibration : 3;
        // whether the analyzer is mid-cycle
        bool _analysis_started;
    };

    // Shared FFT engine state local to the FFT thread
    EngineState _thread_state;
    // Shared FFT engine state accessible by the main thread
    EngineState _global_state;

    // number of samples needed before a new frame can be processed
    uint16_t _samples_per_frame;
    // number of ms that a frame should take to process to sustain output rate
    uint16_t _frame_time_ms;
    // last cycle time
    uint32_t _output_cycle_micros;
    // downsampled gyro data circular buffer for frequency analysis
    FloatBuffer _downsampled_gyro_data[XYZ_AXIS_COUNT];
    // accumulator for sampled gyro data
    Vector3f _oversampled_gyro_accum;
    // count of oversamples
    uint16_t _oversampled_gyro_count;

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
    // number of tracked peaks
    uint8_t _tracked_peaks;
    // engine health in tracked peaks per axis
    Vector3<uint8_t> _health;
    // engine health on roll/pitch/yaw
    Vector3<uint8_t> _rpy_health;
    // averaged throttle output over averaging period
    float _avg_throttle_out;

    // smoothing filter on the output
    MedianLowPassFilter3dFloat _center_freq_filter[FrequencyPeak::MAX_TRACKED_PEAKS];
    // smoothing filter on the energy
    MedianLowPassFilter3dFloat _center_freq_energy_filter[FrequencyPeak::MAX_TRACKED_PEAKS];
    // smoothing filter on the bandwidth
    MedianLowPassFilter3dFloat _center_bandwidth_filter[FrequencyPeak::MAX_TRACKED_PEAKS];
    // smoothing filter on the frequency fit
    LowPassFilterFloat _harmonic_fit_filter[XYZ_AXIS_COUNT];

    // configured sampling rate
    uint16_t _fft_sampling_rate_hz;
    // number of cycles without a detected signal
    uint8_t _missed_cycles[XYZ_AXIS_COUNT][FrequencyPeak::MAX_TRACKED_PEAKS];
    // number of cycles where peaks have swapped places
    uint8_t _distorted_cycles[XYZ_AXIS_COUNT];
    // whether the analyzer initialized correctly
    bool _initialized;

    // whether the analyzer should be run
    bool _analysis_enabled ;
    // whether the update thread was created
    bool _thread_created ;
    // whether the pre-arm check has successfully completed
    bool _calibrated;
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
    // harmonic fit percentage
    AP_Int8 _harmonic_fit;
    // harmonic peak target
    AP_Int8 _harmonic_peak;
    // number of output frames to retain for averaging
    AP_Int8 _num_frames;
    // mask of IMUs to record gyro frames after the filter bank
    AP_Int32 _options;
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
