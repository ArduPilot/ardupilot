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

   Code by Andy Piper with help from betaflight
 */

#include "AP_GyroFFT.h"

#if HAL_GYROFFT_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <Filter/HarmonicNotchFilter.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Vehicle/AP_Vehicle.h>
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include <AP_Motors/AP_Motors.h>
#endif
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#ifndef FFT_DEFAULT_WINDOW_SIZE
// the H7 can cope with a longer length and these boards generally have BMI088 which needs a longer length
#if defined(STM32H7)
#define FFT_DEFAULT_WINDOW_SIZE     64
#else
#define FFT_DEFAULT_WINDOW_SIZE     32
#endif
#endif
#ifndef FFT_DEFAULT_WINDOW_OVERLAP
#if defined(STM32H7)
#define FFT_DEFAULT_WINDOW_OVERLAP  0.75f
#else
#define FFT_DEFAULT_WINDOW_OVERLAP  0.5f
#endif
#endif
#define FFT_THR_REF_DEFAULT         0.35f   // the estimated throttle reference, 0 ~ 1
#define FFT_SNR_DEFAULT            25.0f // a higher SNR is safer and this works quite well on a Pixracer
#define FFT_STACK_SIZE              1024
#define FFT_MIN_SAMPLES_PER_FRAME   16
#define FFT_HARMONIC_FIT_DEFAULT    10
#define FFT_HARMONIC_FIT_FILTER_HZ  15.0f
#define FFT_HARMONIC_FIT_MULT       200.0f
#define FFT_HARMONIC_FIT_TRACK_ROLL    4
#define FFT_HARMONIC_FIT_TRACK_PITCH   5

// table of user settable parameters
const AP_Param::GroupInfo AP_GyroFFT::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable Gyro FFT analyser
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_GyroFFT, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MINHZ
    // @DisplayName: Minimum Frequency
    // @Description: Lower bound of FFT frequency detection in Hz. On larger vehicles the minimum motor frequency is likely to be significantly lower than for smaller vehicles.
    // @Range: 20 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MINHZ", 2, AP_GyroFFT, _fft_min_hz, 80),

    // @Param: MAXHZ
    // @DisplayName: Maximum Frequency
    // @Description: Upper bound of FFT frequency detection in Hz. On smaller vehicles the maximum motor frequency is likely to be significantly higher than for larger vehicles.
    // @Range: 20 495
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MAXHZ", 3, AP_GyroFFT, _fft_max_hz, 200),

    // @Param: SAMPLE_MODE
    // @DisplayName: Sample Mode
    // @Description: Sampling mode (and therefore rate). 0: Gyro rate sampling, 1: Fast loop rate sampling, 2: Fast loop rate / 2 sampling, 3: Fast loop rate / 3 sampling. Takes effect on reboot.
    // @Range: 0 4
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SAMPLE_MODE", 4, AP_GyroFFT, _sample_mode, 0),

    // @Param: WINDOW_SIZE
    // @DisplayName: FFT window size
    // @Description: Size of window to be used in FFT calculations. Takes effect on reboot. Must be a power of 2 and between 32 and 512. Larger windows give greater frequency resolution but poorer time resolution, consume more CPU time and may not be appropriate for all vehicles. Time and frequency resolution are given by the sample-rate / window-size. Windows of 256 are only really recommended for F7 class boards, windows of 512 or more H7 class.
    // @Range: 32 1024
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("WINDOW_SIZE", 5, AP_GyroFFT, _window_size, FFT_DEFAULT_WINDOW_SIZE),

    // @Param: WINDOW_OLAP
    // @DisplayName: FFT window overlap
    // @Description: Percentage of window to be overlapped before another frame is process. Takes effect on reboot. A good default is 50% overlap. Higher overlap results in more processed frames but not necessarily more temporal resolution. Lower overlap results in lost information at the frame edges.
    // @Range: 0 0.9
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("WINDOW_OLAP", 6, AP_GyroFFT, _window_overlap, FFT_DEFAULT_WINDOW_OVERLAP),

    // @Param: FREQ_HOVER
    // @DisplayName: FFT learned hover frequency
    // @Description: The learned hover noise frequency
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("FREQ_HOVER", 7, AP_GyroFFT, _freq_hover_hz, 80.0f),

    // @Param: THR_REF
    // @DisplayName: FFT learned thrust reference
    // @Description: FFT learned thrust reference for the hover frequency and FFT minimum frequency.
    // @Range: 0.01 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_REF", 8, AP_GyroFFT, _throttle_ref, FFT_THR_REF_DEFAULT),

    // @Param: SNR_REF
    // @DisplayName: FFT SNR reference threshold
    // @Description: FFT SNR reference threshold in dB at which a signal is determined to be present.
    // @Range: 0.0 100.0
    // @User: Advanced
    AP_GROUPINFO("SNR_REF", 9, AP_GyroFFT, _snr_threshold_db, FFT_SNR_DEFAULT),

    // @Param: ATT_REF
    // @DisplayName: FFT attenuation for bandwidth calculation
    // @Description: FFT attenuation level in dB for bandwidth calculation and peak detection. The bandwidth is calculated by comparing peak power output with the attenuated version. The default of 15 has shown to be a good compromise in both simulations and real flight.
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("ATT_REF", 10, AP_GyroFFT, _attenuation_power_db, 15),

    // @Param: BW_HOVER
    // @DisplayName: FFT learned bandwidth at hover
    // @Description: FFT learned bandwidth at hover for the attenuation frequencies.
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("BW_HOVER", 11, AP_GyroFFT, _bandwidth_hover_hz, 20),

    // @Param: HMNC_FIT
    // @DisplayName: FFT harmonic fit frequency threshold
    // @Description: FFT harmonic fit frequency threshold percentage at which a signal of the appropriate frequency is determined to be the harmonic of another. Signals that have a harmonic relationship that varies at most by this percentage are considered harmonics of each other for the purpose of selecting the harmonic notch frequency. If a match is found then the lower frequency harmonic is always used as the basis for the dynamic harmonic notch. A value of zero completely disables harmonic matching.
    // @Range: 0 100
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HMNC_FIT", 12, AP_GyroFFT, _harmonic_fit, FFT_HARMONIC_FIT_DEFAULT),

    // @Param: HMNC_PEAK
    // @DisplayName: FFT harmonic peak target
    // @Description: The FFT harmonic peak target that should be returned by FTN1.PkAvg. The resulting value will be used by the harmonic notch if configured to track the FFT frequency. By default the appropriate peak is auto-detected based on the harmonic fit between peaks and the energy-weighted average frequency on roll on pitch is used. Setting this to 1 will always target the highest energy peak. Setting this to 2 will target the highest energy peak that is lower in frequency than the highest energy peak. Setting this to 3 will target the highest energy peak that is higher in frequency than the highest energy peak. Setting this to 4 will target the highest energy peak on the roll axis only and only the roll frequency will be used (some vehicles have a much more pronounced peak on roll). Setting this to 5 will target the highest energy peak on the pitch axis only and only the pitch frequency will be used (some vehicles have a much more pronounced peak on roll).
    // @Values: 0:Auto,1:Center Frequency,2:Lower-Shoulder Frequency,3:Upper-Shoulder Frequency,4:Roll-Axis,5:Pitch-Axis
    // @User: Advanced
    AP_GROUPINFO("HMNC_PEAK", 13, AP_GyroFFT, _harmonic_peak, 0),

    // @Param: NUM_FRAMES
    // @DisplayName: FFT output frames to retain and average
    // @Description: Number of output frequency frames to retain and average in order to calculate final frequencies. Averaging output frames can drastically reduce noise and jitter at the cost of latency as long as the input is stable. The default is to perform no averaging. For rapidly changing frequencies (e.g. smaller aircraft) fewer frames should be averaged.
    // @Range: 0 8
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("NUM_FRAMES", 14, AP_GyroFFT, _num_frames, 0),

    AP_GROUPEND
};

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency (Nyquist) of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// The first bin is used to store the DC and Nyquist values combined.
// Eg [DC/Nyquist], [16,47), [47,78), [78,109) etc
// For a loop rate of 800Hz, 16 bins each 25Hz wide
// Eg X[0]=[DC/Nyquist], X[1]=[12,37), X[2]=[37,62), X[3]=[62,87), X[4]=[87,112)
// So middle frequency is X[n] * 25 and the range is X[n] * 25 - 12 < f < X[n] * 25 + 12

const extern AP_HAL::HAL& hal;

AP_GyroFFT::AP_GyroFFT()
{
    _thread_state._noise_needs_calibration = 0x07; // all axes need calibration
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_GyroFFT must be singleton");
    }
    _singleton = this;
}

// initialize the FFT parameters and engine
void AP_GyroFFT::init(uint16_t loop_rate_hz)
{
    // if FFT analysis is not enabled we don't want to allocate any of the associated resources
    if (!_enable) {
        return;
    }

    _ins = &AP::ins();

    // sanity check
    if (_ins->get_raw_gyro_rate_hz() == 0) {
        AP_HAL::panic("AP_GyroFFT must be initialized after AP_InertialSensor");
    }

    // check that we support the window size requested and it is a power of 2
    _window_size.set(1 << lrintf(log2f(_window_size.get())));
#if defined(STM32H7) || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _window_size.set(constrain_int16(_window_size, 32, 512));
#else
    _window_size.set(constrain_int16(_window_size, 32, 256));
#endif
    // number of samples needed before a new frame can be processed
    _window_overlap.set(constrain_float(_window_overlap, 0.0f, 0.9f));
    _samples_per_frame = (1.0f - _window_overlap) * _window_size;
    // if we allow too small a number of samples per frame the output rate gets very high
    // this is particularly a problem on IMUs with higher sample rates (e.g. BMI088)
    // 16 gives a maximum output rate of 2Khz / 16 = 125Hz per axis or 375Hz in aggregate
    _samples_per_frame = MAX(FFT_MIN_SAMPLES_PER_FRAME, 1 << lrintf(log2f(_samples_per_frame)));
    if (_num_frames > 0) {
        _num_frames.set(constrain_int16(_num_frames, 2, AP_HAL::DSP::MAX_SLIDING_WINDOW_SIZE));
    }

    // check that we have enough memory for the window size requested
    // INS: XYZ_AXIS_COUNT * INS_MAX_INSTANCES * _window_size, DSP: 3 * _window_size, FFT: XYZ_AXIS_COUNT + 3 * _window_size
    const uint32_t allocation_count = (XYZ_AXIS_COUNT * INS_MAX_INSTANCES + 3 + XYZ_AXIS_COUNT + 3 + _num_frames) * sizeof(float);
    if (allocation_count * FFT_DEFAULT_WINDOW_SIZE > hal.util->available_memory() / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AP_GyroFFT: disabled, required %u bytes", (unsigned int)allocation_count * FFT_DEFAULT_WINDOW_SIZE);
        return;
    } else if (allocation_count * _window_size > hal.util->available_memory() / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AP_GyroFFT: req alloc %u bytes (free=%u)", (unsigned int)allocation_count * _window_size, (unsigned int)hal.util->available_memory());
        _window_size.set(FFT_DEFAULT_WINDOW_SIZE);
    }
    // save any changes that were made
    _window_size.save();

    // determine the FFT sample rate based on the gyro rate, loop rate and configuration
    if (_sample_mode == 0) {
        _fft_sampling_rate_hz = _ins->get_raw_gyro_rate_hz();
    } else {
        _fft_sampling_rate_hz = loop_rate_hz / _sample_mode;
        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            if (!_downsampled_gyro_data[axis].set_size(_window_size + _samples_per_frame)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate window for AP_GyroFFT");
                return;
            }
        }
    }
    _current_sample_mode = _sample_mode;

    _ref_energy = new Vector3f[_window_size];
    if (_ref_energy == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate window for AP_GyroFFT");
        return;
    }

    // make the gyro window match the window size plus a buffer to cope with the backend
    // getting too far ahead.
    if (!_ins->set_gyro_window_size(_window_size + _samples_per_frame)) {
        return;
    }

    // check for harmonics across all harmonic notch filters
    // note that we only allow one harmonic notch filter linked to the FFT code
    uint8_t harmonics = 0;
    uint8_t num_notches = 0;
    for (auto &notch : _ins->harmonic_notches) {
        if (notch.params.enabled()) {
            harmonics |= notch.params.harmonics();
            num_notches = MAX(num_notches, notch.num_dynamic_notches);
        }
    }
    if (harmonics == 0) {
        // this allows use of FFT to find peaks with all notch filters disabled
        harmonics = 3;
    }
    // count the number of active harmonics or dynamic notchs
    _tracked_peaks = constrain_int16(MAX(__builtin_popcount(harmonics),
                                         num_notches), 1, FrequencyPeak::MAX_TRACKED_PEAKS);

    // calculate harmonic multiplier. this assumes the harmonics configured on the 
    // harmonic notch reflect the multiples of the fundamental harmonic that should be tracked
    if (_harmonic_fit > 0) {
        uint8_t first_harmonic = 0;
        for (uint8_t i = 0; i < HNF_MAX_HARMONICS; i++) {
            if (harmonics & (1<<i)) {
                if (first_harmonic == 0) {
                    first_harmonic = i + 1;
                } else {
                    _harmonic_multiplier = float(i + 1) / first_harmonic;
                    break;
                }
            }
        }
        // if no harmonic specified then select a simple 2x multiple
        if (is_zero(_harmonic_multiplier)) {
            _harmonic_multiplier = 2.0f;
        }
    }

    // initialise the HAL DSP subsystem
    _state = hal.dsp->fft_init(_window_size, _fft_sampling_rate_hz, _num_frames);
    if (_state == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to initialize DSP engine");
        return;
    }

    // per-axis frame time
    _frame_time_ms = _samples_per_frame * 1000 / _fft_sampling_rate_hz;
    // The update rate for the output, defaults are 1Khz / (1 - 0.5) * 32 == 62hz
    const float output_rate = static_cast<float>(_fft_sampling_rate_hz) / static_cast<float>(_samples_per_frame);
    // establish suitable defaults for the detected values
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        _thread_state._center_freq_hz[axis] = _fft_min_hz;

        for (uint8_t peak = 0; peak < FrequencyPeak::MAX_TRACKED_PEAKS; peak++) {
            _thread_state._center_freq_hz_filtered[axis][peak] = _fft_min_hz;
        }
        // number of cycles to average over, two complete windows to be sure
        _noise_calibration_cycles[axis] = (_window_size / _samples_per_frame) * 2;
        // harmonic frequency fit should change relatively slowly
        _harmonic_fit_filter[axis].set_cutoff_frequency(output_rate, MIN(output_rate * 0.48f, FFT_HARMONIC_FIT_FILTER_HZ));
    }

    // configure a filter for frequency, bandwidth and energy for each of the three tracked noise peaks
    for (uint8_t peak = 0; peak < FrequencyPeak::MAX_TRACKED_PEAKS; peak++) {
        // calculate low-pass filter characteristics based on window size and overlap
        _center_freq_filter[peak].set_cutoff_frequency(output_rate, output_rate * 0.48f);
        // the bin energy jumps around a lot so requires more filtering
        _center_freq_energy_filter[peak].set_cutoff_frequency(output_rate, output_rate * 0.25f);
        // smooth the bandwidth output more aggressively
        _center_bandwidth_filter[peak].set_cutoff_frequency(output_rate, output_rate * 0.25f);
    }

    // the number of cycles required to have a proper noise reference
    _noise_cycles = (_window_size / _samples_per_frame) * XYZ_AXIS_COUNT;

    // finally we are done
    _initialized = true;
    update_parameters(true);
    // start running FFTs
    if (start_update_thread()) {
        set_analysis_enabled(true);
    }
}

// sample the gyros either by using a gyro window sampled at the gyro rate or making invdividual samples
// called from fast_loop thread - this function does not take out a sempahore to avoid waiting on the FFT thread
void AP_GyroFFT::sample_gyros()
{
    if (!analysis_enabled()) {
        return;
    }

    // update counters for gyro window
    if (_current_sample_mode > 0) {
        // for loop rate sampling accumulate and average gyro samples
        _oversampled_gyro_accum += _ins->get_raw_gyro();
        _oversampled_gyro_count++;

        if ((_oversampled_gyro_count % _current_sample_mode) == 0) {
            // calculate mean value of accumulated samples
            Vector3f sample = _oversampled_gyro_accum / _current_sample_mode;
            // fast sampling means that the raw gyro values have already been averaged over 8 samples
            _downsampled_gyro_data[0].push(sample.x);
            _downsampled_gyro_data[1].push(sample.y);
            _downsampled_gyro_data[2].push(sample.z);

            _oversampled_gyro_accum.zero();
            _oversampled_gyro_count = 0;
        }
    }
}

// update the state as as required
// called from main thread at 400Hz - anything that requires atomic access to IMU data needs to be done here
void AP_GyroFFT::update()
{
    if (!analysis_enabled()) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    _config._analysis_enabled = _analysis_enabled;
    _global_state = _thread_state;

    // calculate health based on being 5 frames behind, SITL needs longer
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const uint32_t output_delay = _frame_time_ms * FFT_MAX_MISSED_UPDATES * 2;
#else
    const uint32_t output_delay = _frame_time_ms * FFT_MAX_MISSED_UPDATES;
#endif
    uint32_t now = AP_HAL::millis();
    _rpy_health.x = (now - _global_state._health_ms.x <= output_delay);
    _rpy_health.y = (now - _global_state._health_ms.y <= output_delay);
    _rpy_health.z = (now - _global_state._health_ms.z <= output_delay);

    if (!_rpy_health.x && !_rpy_health.y) {
        _health = 0;
    } else {
        uint8_t num_notches = 1;
        for (auto &notch : _ins->harmonic_notches) {
            if (notch.params.enabled()) {
                num_notches = MAX(num_notches, notch.num_dynamic_notches);
            }
        }
        _health = MIN(_global_state._health, num_notches);
    }
}

// analyse gyro data using FFT, returns number of samples still held
// called from FFT thread
uint16_t AP_GyroFFT::run_cycle()
{
    if (!analysis_enabled()) {
        return 0;
    }

    if (!_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return 0;
    }

    // do we have enough samples for another pass?
    if (!start_analysis()) {
        uint16_t new_sample_count =  get_available_samples(_update_axis);
        _sem.give();
        return new_sample_count;
    }

    // take a copy of the config inside the semaphore
    EngineConfig config = _config;

    _sem.give();

    uint32_t now = AP_HAL::micros();

    // get the appropriate gyro buffer
    FloatBuffer& gyro_buffer = (_sample_mode == 0 ?_ins->get_raw_gyro_window(_update_axis) : _downsampled_gyro_data[_update_axis]);
    // if we have many more samples than the window size then we are struggling to 
    // stay ahead of the gyro loop so drop samples so that this cycle will use all available samples
    if (gyro_buffer.available() > uint32_t(_state->_window_size + uint16_t(_samples_per_frame >> 1))) { // half the frame size is a heuristic
        gyro_buffer.advance(gyro_buffer.available() - _state->_window_size);
    }
    // let's go!
    hal.dsp->fft_start(_state, gyro_buffer, _samples_per_frame);

    // calculate FFT and update filters outside the semaphore
    uint16_t bin_max = hal.dsp->fft_analyse(_state, config._fft_start_bin, config._fft_end_bin, config._attenuation_cutoff);

    // something has been detected, update the peak frequency and associated metrics
    update_ref_energy(bin_max);
    calculate_noise(false, config);

    // record how we are doing
    _thread_state._last_output_us[_update_axis] = AP_HAL::micros();
    _output_cycle_micros = _thread_state._last_output_us[_update_axis] - now;

    // move onto the next axis
    _update_axis = (_update_axis + 1) % XYZ_AXIS_COUNT;

    // ready to receive another frame, because lock contention is so expensive we don't lock
    // around this flag but rather rely on the semaphore at the beginning of the loop to
    // ensure eventual visibility to the main loop
    _thread_state._analysis_started = false;

    // samples remaining in the next axis
    return get_available_samples(_update_axis);
}

// whether analysis can be run again or not
// called from FFT thread with the semaphore held
bool AP_GyroFFT::start_analysis() {
    if (_thread_state._analysis_started) {
        return false;
    }
    // don't run any more gyro cycles once noise is calibrated and the self-test is running
    if (!_thread_state._noise_needs_calibration && !_calibrated) {
        return false;
    }

    if (get_available_samples(_update_axis) >= _state->_window_size) {
        _thread_state._analysis_started = true;
        return true;
    }
    return false;
}

// update calculated values of dynamic parameters - runs at 1Hz
void AP_GyroFFT::update_parameters(bool force)
{
    // lock contention is very costly, so don't allow configuration updates while flying
    if ((!_initialized || AP::arming().is_armed()) && !force) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    // don't allow MAXHZ to go to Nyquist
    _fft_max_hz.set(MIN(_fft_max_hz, _fft_sampling_rate_hz * 0.48));
    _config._snr_threshold_db = _snr_threshold_db;
    _config._fft_min_hz = _fft_min_hz;
    _config._fft_max_hz = _fft_max_hz;
    // determine the start FFT bin for all frequency detection
    _config._fft_start_bin = MAX(floorf(_fft_min_hz.get() / _state->_bin_resolution), 1);
    // determine the endt FFT bin for all frequency detection
    _config._fft_end_bin = MIN(ceilf(_fft_max_hz.get() / _state->_bin_resolution), _state->_bin_count);
    // actual attenuation from the db value
    _config._attenuation_cutoff = powf(10.0f, -_attenuation_power_db * 0.1f);
}

// thread for processing gyro data via FFT
void AP_GyroFFT::update_thread(void)
{
    while (true) {
        uint16_t remaining_samples = run_cycle();
        // this is to stop us burning CPU while waiting for samples, the reduction by _samples_per_frame is a heuristic to prevent waiting too long
        // and missing frames (easy to see in SITL because the noise will keep calibrating)
        // we always delay by at least 1us to give logging a chance to run at the same priority
        uint32_t delay = constrain_int32((int16_t)_state->_window_size - (int16_t)remaining_samples, 0, _samples_per_frame)
            * 1e6 / _fft_sampling_rate_hz;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // in SITL the gyros do not run in a different thread
        if (delay > 0) {
            hal.scheduler->delay_microseconds(delay);
        }
#else
        hal.scheduler->delay_microseconds(MAX(delay, 1U));
#endif
    }
}

// start the update thread
bool AP_GyroFFT::start_update_thread(void)
{
    WITH_SEMAPHORE(_sem);

    if (_thread_created) {
        return true;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_GyroFFT::update_thread, void), "apm_fft", FFT_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        AP_HAL::panic("Failed to start AP_GyroFFT update thread");
        return false;
    }

    _thread_created = true;
    return true;
}

// self-test the FFT analyser - can only be done while samples are not being taken
// called from main thread
bool AP_GyroFFT::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len)
{
    if (!analysis_enabled()) {
        return true;
    }

    // already calibrated
    if (_calibrated) {
        return true;
    }

    // analysis is started in the main thread, don't trample on in-flight analysis
    if (_global_state._analysis_started) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FFT still analyzing");
        return false;
    }

    // still calibrating noise so not ready
    if (_global_state._noise_needs_calibration) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FFT calibrating noise");
        return false;
    }

    // make sure the frequency maximum is below Nyquist
    if (_fft_max_hz > _fft_sampling_rate_hz * 0.5f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FFT config MAXHZ %dHz > %dHz", _fft_max_hz.get(), _fft_sampling_rate_hz / 2);
        return false;
    }

    // check for sane frequency resolution - for 1k backends with length 32 this will be 32Hz
    if (_state->_bin_resolution > 50.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: resolution is %.1fHz, increase length", _state->_bin_resolution);
        return true; // a low resolution is not fatal
    }
#if 0 // these calculations do not result in a long enough expected delay
    // in order to test all frequencies we need to endure a long pause for higher windows lengths
    // see timings AP_HAL_ChibiOS/DSP.cpp for per step timings on different hardware
#if defined(STM32H7)
    const uint32_t cycle_time = (16 * (1 << (_window_size / 32)) + 5) * (_config._fft_end_bin - _config._fft_start_bin + 1) * 2 / 1000; // H7
#else
    const uint32_t cycle_time = (29 * (1 << (_window_size / 32)) + 5) * (_config._fft_end_bin - _config._fft_start_bin + 1) * 2 / 1000; // F4
#endif
#endif
    EXPECT_DELAY_MS(2000); // tested on an H7 at 1024 window

    float max_divergence = self_test_bin_frequencies();
    // for longer FFT lengths the resolution gets below 1Hz
    if (max_divergence > MAX(_state->_bin_resolution * 0.5f, 1)) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FFT self-test failed, max error %fHz", max_divergence);
    }

    _calibrated =  max_divergence <= MAX(_state->_bin_resolution * 0.5f, 1);

    if (_calibrated) {
        // provide the user with some useful information about what they have configured
        gcs().send_text(MAV_SEVERITY_INFO, "FFT: calibrated %.1fKHz/%.1fHz/%.1fHz", _fft_sampling_rate_hz * 0.001f,
             _state->_bin_resolution * 0.5, 1000.0f * XYZ_AXIS_COUNT / _frame_time_ms);
    }

    return _calibrated;
}

// we may have disabled the FFT arming check, in which case make sure the engine can still run
bool AP_GyroFFT::prepare_for_arming()
{
    _calibrated = true;
    return true;
}

// update the hover frequency input filter. should be called at 100hz when in a stable hover
// called from main thread
void AP_GyroFFT::update_freq_hover(float dt, float throttle_out)
{
    if (!analysis_enabled()) {
        return;
    }

    // throttle averaging for average fft calculation
    if (is_zero(_avg_throttle_out)) {
        _avg_throttle_out = throttle_out;
    } else {
        _avg_throttle_out = constrain_float(_avg_throttle_out + (dt / (10.0f + dt)) * (throttle_out - _avg_throttle_out), 0.01f, 0.9f);
    }

    // we have chosen to constrain the hover frequency to be within the range reachable by the third order expo polynomial.
    _freq_hover_hz.set(constrain_float(_freq_hover_hz + (dt / (10.0f + dt)) * (get_weighted_noise_center_freq_hz() - _freq_hover_hz), _fft_min_hz, _fft_max_hz));
    _bandwidth_hover_hz.set(constrain_float(_bandwidth_hover_hz + (dt / (10.0f + dt)) * (get_weighted_noise_center_bandwidth_hz() - _bandwidth_hover_hz), 0, _fft_max_hz * 0.5f));
    _throttle_ref.set(constrain_float(_throttle_ref + (dt / (10.0f + dt)) * (throttle_out * sq((float)_fft_min_hz.get() / _freq_hover_hz.get()) - _throttle_ref), 0.01f, 0.9f));
}

// save parameters as part of disarming
// called from main thread
void AP_GyroFFT::save_params_on_disarm()
{
    if (!analysis_enabled()) {
        return;
    }

    _freq_hover_hz.save();
    _bandwidth_hover_hz.save();
    _throttle_ref.save();
}

    // notch tuning
void AP_GyroFFT::start_notch_tune()
{
    if (!analysis_enabled()) {
        return;
    }

    if (!hal.dsp->fft_start_average(_state)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: Unable to start FFT averaging");
    }
    _avg_throttle_out = 0.0f;
}

void AP_GyroFFT::stop_notch_tune()
{
    if (!analysis_enabled()) {
        return;
    }

    float freqs[FrequencyPeak::MAX_TRACKED_PEAKS] {};

    uint16_t numpeaks = hal.dsp->fft_stop_average(_state, _config._fft_start_bin, _config._fft_end_bin, freqs);

    if (numpeaks == 0) {
        return;
    }

    float harmonic = freqs[0];
    float harmonic_fit = 100.0f;

    for (uint8_t i = 1; i < numpeaks; i++) {
        if (freqs[i] < harmonic) {
            const float fit = 100.0f * fabsf(harmonic - freqs[i] * _harmonic_multiplier) / harmonic;
            if (isfinite(fit) && fit < _harmonic_fit && fit < harmonic_fit) {
                harmonic = freqs[i];
                harmonic_fit = fit;
            }
        }
    }

    gcs().send_text(MAV_SEVERITY_INFO, "FFT: Found peaks at %.1f/%.1f/%.1fHz", freqs[0], freqs[1], freqs[2]);
    gcs().send_text(MAV_SEVERITY_NOTICE, "FFT: Selected %.1fHz with fit %.1f%%\n", harmonic, is_equal(harmonic_fit, 100.0f) ? 100.0f : 100.0f - harmonic_fit);

    // if we don't have a throttle value then all bets are off
    if (is_zero(_avg_throttle_out) || is_zero(harmonic)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: Unable to calculate notch: need stable hover");
        return;
    }

    // pick a ref which means the notch covers all the way down to FFT_MINHZ
    const float thr_ref = _avg_throttle_out * sq((float)_fft_min_hz.get() / harmonic);

    if (!_ins->setup_throttle_gyro_harmonic_notch((float)_fft_min_hz.get(), thr_ref)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: Unable to set throttle notch with %.1fHz/%.2f",
            (float)_fft_min_hz.get(), thr_ref);
        // save results to FFT slots
        _throttle_ref.set(thr_ref);
        _freq_hover_hz.set(harmonic);
    } else {
        gcs().send_text(MAV_SEVERITY_NOTICE, "FFT: Notch frequency %.1fHz and ref %.2f selected", (float)_fft_min_hz.get(), thr_ref);
    }
}

// return the noise peak that is being tracked
// called from main thread
AP_GyroFFT::FrequencyPeak AP_GyroFFT::get_tracked_noise_peak() const
{
    // if the user has specified a specific axis to track then use that
    if (_harmonic_peak > FrequencyPeak::MAX_TRACKED_PEAKS) {
        switch (_harmonic_peak) {
        case FFT_HARMONIC_FIT_TRACK_ROLL:
            if (_global_state._harmonic_fit.x < _harmonic_fit) {
                return FrequencyPeak(_global_state._tracked_peak.x);
            }
            break;
        case FFT_HARMONIC_FIT_TRACK_PITCH:
            if (_global_state._harmonic_fit.y < _harmonic_fit) {
                return FrequencyPeak(_global_state._tracked_peak.y);
            }
            break;
        default:
            break;
        }
        return FrequencyPeak::CENTER;
    }
    // if the user has specified a specific peak to track then use that
    if (_harmonic_peak > 0) {
        return FrequencyPeak(constrain_int16(_harmonic_peak - 1, FrequencyPeak::CENTER, FrequencyPeak::UPPER_SHOULDER));
    }

    // required fit of 10% is fairly conservative when testing in SITL, testing shows that it's safer to
    // require both tracked axes to fit - biasing towards the highest energy peak
    if (_global_state._harmonic_fit.x < _harmonic_fit && _global_state._harmonic_fit.y < _harmonic_fit) {
        return FrequencyPeak(_global_state._tracked_peak.x);
    }

    return FrequencyPeak::CENTER;
}

// center frequency slewed from previous to current value at the output rate
float AP_GyroFFT::get_slewed_noise_center_freq_hz(FrequencyPeak peak, uint8_t axis) const
{
    uint32_t now = AP_HAL::micros();
    const float slew = MIN(1.0f, (now - _global_state._last_output_us[axis])
        * (static_cast<float>(_fft_sampling_rate_hz) / static_cast<float>(_samples_per_frame)) * 1e-6f);
    return (_global_state._prev_center_freq_hz_filtered[peak][axis]
        + (_global_state._center_freq_hz_filtered[peak][axis] - _global_state._prev_center_freq_hz_filtered[peak][axis]) * slew);
}

// weighted center frequency slewed from previous to current value at the output rate
float AP_GyroFFT::get_slewed_weighted_freq_hz(FrequencyPeak peak) const
{
    const Vector3f& energy = get_center_freq_energy(peak);
    const float freq_x = get_slewed_noise_center_freq_hz(peak, 0);
    const float freq_y = get_slewed_noise_center_freq_hz(peak, 1);

    if (!energy.is_nan() && !is_zero(energy.x) && !is_zero(energy.y)) {
        return (freq_x * energy.x + freq_y * energy.y) / (energy.x + energy.y);
    } else {
        return (freq_x + freq_y) * 0.5f;
    }
}

// return an average center frequency weighted by bin energy
// called from main thread
float AP_GyroFFT::get_weighted_noise_center_freq_hz() const
{
    if (!analysis_enabled()) {
        return _fft_min_hz;
    }

    if (!_health) {
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        AP_Motors* motors = AP::motors();
        if (motors != nullptr && !is_zero(_throttle_ref)) {
            // FFT is not healthy, fallback to throttle-based estimate
            return constrain_float(_fft_min_hz * MAX(1.0f, sqrtf(motors->get_throttle_out() / _throttle_ref)), _fft_min_hz, _fft_max_hz);
        }
#endif
    }

    const FrequencyPeak peak = get_tracked_noise_peak();
    // pitch was good or required, roll was not, use pitch only
    if (!_rpy_health.x || _harmonic_peak == FFT_HARMONIC_FIT_TRACK_PITCH) {
        return get_slewed_noise_center_freq_hz(peak, 1);    // Y-axis
    }
    // roll was good or required, pitch was not, use roll only
    if (!_rpy_health.y || _harmonic_peak == FFT_HARMONIC_FIT_TRACK_ROLL) {
        return get_slewed_noise_center_freq_hz(peak, 0);    // X-axis
    }

    return get_slewed_weighted_freq_hz(peak);
}

// return all the center frequencies weighted by bin energy
// called from main thread
uint8_t AP_GyroFFT::get_weighted_noise_center_frequencies_hz(uint8_t num_freqs, float* freqs) const
{
    if (!analysis_enabled()) {
        freqs[0] = _fft_min_hz;
        return 1;
    }

    if (!_health) {
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        AP_Motors* motors = AP::motors();
        if (motors != nullptr) {
            // FFT is not healthy, fallback to throttle-based estimate
            freqs[0] = constrain_float(_fft_min_hz * MAX(1.0f, sqrtf(motors->get_throttle_out() / _throttle_ref)), _fft_min_hz, _fft_max_hz);
            return 1;
        }
#endif
    }

    const uint8_t tracked_peaks = MIN(_health, num_freqs);
    // pitch was good or required, roll was not, use pitch only
    if (!_rpy_health.x || _harmonic_peak == FFT_HARMONIC_FIT_TRACK_PITCH) {
        for (uint8_t i = 0; i < tracked_peaks; i++) {
            freqs[i] = get_slewed_noise_center_freq_hz(FrequencyPeak(i), 1);    // Y-axis
        }
        return tracked_peaks;
    }
    // roll was good or required, pitch was not, use roll only
    if (!_rpy_health.y || _harmonic_peak == FFT_HARMONIC_FIT_TRACK_ROLL) {
        for (uint8_t i = 0; i < tracked_peaks; i++) {
            freqs[i] = get_slewed_noise_center_freq_hz(FrequencyPeak(i), 0);    // X-axis
        }
        return tracked_peaks;
    }

    for (uint8_t i = 0; i < tracked_peaks; i++) {
        freqs[i] = get_slewed_weighted_freq_hz(FrequencyPeak(i));
    }
    return tracked_peaks;
}

float AP_GyroFFT::calculate_weighted_freq_hz(const Vector3f& energy, const Vector3f& freq) const
{
    // there is generally a lot of high-energy, slightly lower frequency noise on yaw, however this
    // appears to be a second-order effect as only targetting pitch and roll (x & y) produces much cleaner output all round
    if (!energy.is_nan() && !is_zero(energy.x) && !is_zero(energy.y)) {
        return (freq.x * energy.x + freq.y * energy.y)
            / (energy.x + energy.y);
    }
    else {
        return (freq.x + freq.y) * 0.5f;
    }
}

// @LoggerMessage: FTN1
// @Description: FFT Filter Tuning
// @Field: TimeUS: microseconds since system startup
// @Field: PkAvg: peak noise frequency as an energy-weighted average of roll and pitch peak frequencies
// @Field: BwAvg: bandwidth of weighted peak frequency where edges are determined by FFT_ATT_REF
// @Field: SnX: signal-to-noise ratio on the roll axis
// @Field: SnY: signal-to-noise ratio on the pitch axis
// @Field: SnZ: signal-to-noise ratio on the yaw axis
// @Field: FtX: harmonic fit on roll of the highest noise peak to the second highest noise peak
// @Field: FtY: harmonic fit on pitch of the highest noise peak to the second highest noise peak
// @Field: FtZ: harmonic fit on yaw of the highest noise peak to the second highest noise peak
// @Field: FH: FFT health
// @Field: Tc: FFT cycle time

// log gyro fft messages
void AP_GyroFFT::write_log_messages()
{
    if (!analysis_enabled()) {
        return;
    }

    AP::logger().WriteStreaming(
        "FTN1",
        "TimeUS,PkAvg,BwAvg,SnX,SnY,SnZ,FtX,FtY,FtZ,FH,Tc",
        "szz---%%%-s",
        "F---------F",
        "QffffffffBI",
        AP_HAL::micros64(),
        get_weighted_noise_center_freq_hz(),
        get_weighted_noise_center_bandwidth_hz(),
        get_noise_signal_to_noise_db().x,
        get_noise_signal_to_noise_db().y,
        get_noise_signal_to_noise_db().z,
        get_raw_noise_harmonic_fit().x,
        get_raw_noise_harmonic_fit().y,
        get_raw_noise_harmonic_fit().z,
        _health, _output_cycle_micros);

    log_noise_peak(0, FrequencyPeak::CENTER);
    if (_tracked_peaks> 1) {
        log_noise_peak(1, FrequencyPeak::LOWER_SHOULDER);
        log_noise_peak(2, FrequencyPeak::UPPER_SHOULDER);
    }

#if DEBUG_FFT
    const uint32_t now = AP_HAL::millis();
    // output at 1hz
    if ((now - _last_output_ms) > 1000) {
        // doing this from the update thread overflows the stack
        WITH_SEMAPHORE(_sem);
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: f:%.1f, fr:%.1f, b:%u, fd:%.1f",
                        _debug_state._center_freq_hz_filtered[FrequencyPeak::CENTER][_update_axis], _debug_state._center_freq_hz[_update_axis], _debug_max_bin, _debug_max_bin_freq);
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: bw:%.1f, e:%.1f, r:%.1f, snr:%.1f",
                        _debug_state._center_bandwidth_hz_filtered[FrequencyPeak::CENTER][_update_axis], _debug_max_freq_bin, _ref_energy[_debug_max_bin][_update_axis], _debug_snr);
        _last_output_ms = now;
    }
#endif
}

// @LoggerMessage: FTN2
// @Description: FFT Noise Frequency Peak
// @Field: TimeUS: microseconds since system startup
// @Field: Id: peak id where 0 is the centre peak, 1 is the lower shoulder and 2 is the upper shoulder
// @Field: PkX: noise frequency of the peak on roll
// @Field: PkY: noise frequency of the peak on pitch
// @Field: PkZ: noise frequency of the peak on yaw
// @Field: BwX: bandwidth of the peak frequency on roll where edges are determined by FFT_ATT_REF
// @Field: BwY: bandwidth of the peak frequency on pitch where edges are determined by FFT_ATT_REF
// @Field: BwZ: bandwidth of the peak frequency on yaw where edges are determined by FFT_ATT_REF
// @Field: EnX: power spectral density bin energy of the peak on roll
// @Field: EnY: power spectral density bin energy of the peak on roll
// @Field: EnZ: power spectral density bin energy of the peak on roll

// write a single log message
void AP_GyroFFT::log_noise_peak(uint8_t id, FrequencyPeak peak) const
{
    AP::logger().WriteStreaming("FTN2", "TimeUS,Id,PkX,PkY,PkZ,BwX,BwY,BwZ,EnX,EnY,EnZ", "s#zzzzzz---", "F----------", "QBfffffffff",
        AP_HAL::micros64(),
        id,
        get_noise_center_freq_hz(peak).x,
        get_noise_center_freq_hz(peak).y,
        get_noise_center_freq_hz(peak).z,
        get_noise_center_bandwidth_hz(peak).x,
        get_noise_center_bandwidth_hz(peak).y,
        get_noise_center_bandwidth_hz(peak).z,
        get_center_freq_energy(peak).x,
        get_center_freq_energy(peak).y,
        get_center_freq_energy(peak).z);
}

// return an average noise bandwidth weighted by bin energy
// called from main thread
float AP_GyroFFT::get_weighted_noise_center_bandwidth_hz() const
{
    if (!analysis_enabled()) {
        return 0.0f;
    }

    const FrequencyPeak peak = get_tracked_noise_peak();

    return calculate_weighted_freq_hz(get_center_freq_energy(peak), get_noise_center_bandwidth_hz(peak));
}

// calculate noise frequencies from FFT data provided by the HAL subsystem
// called from FFT thread
void AP_GyroFFT::calculate_noise(bool calibrating, const EngineConfig& config)
{
    // calculate the SNR and center frequency energy
    float weighted_center_freq_hz = 0.0f;
    float snr = 0.0f;

    uint8_t num_peaks = calculate_tracking_peaks(weighted_center_freq_hz, snr, calibrating, config);

    _thread_state._center_freq_bin[_update_axis] = _state->_peak_data[FrequencyPeak::CENTER]._bin;
    _thread_state._center_freq_hz[_update_axis] = weighted_center_freq_hz;
    _thread_state._center_snr[_update_axis] = snr;
    // record the last time we had a good signal on this axis
    if (num_peaks > 0) {
        _thread_state._health_ms[_update_axis] = AP_HAL::millis();
    } else {
        _thread_state._health_ms[_update_axis] = 0;
    }
    _thread_state._health = num_peaks;
    FrequencyPeak tracked_peak = FrequencyPeak::CENTER;

    // record the tracked peak for harmonic fit, but only if we have more than one noise peak
    if (num_peaks > 1 && _tracked_peaks > 1 && !is_zero(get_tl_noise_center_freq_hz(FrequencyPeak::CENTER, _update_axis))) {
        if (get_tl_noise_center_freq_hz(FrequencyPeak::CENTER, _update_axis) > get_tl_noise_center_freq_hz(FrequencyPeak::LOWER_SHOULDER, _update_axis)
            // ignore the fit if there is too big a discrepancy between the energies
            && get_tl_center_freq_energy(FrequencyPeak::CENTER, _update_axis) < get_tl_center_freq_energy(FrequencyPeak::LOWER_SHOULDER, _update_axis) * FFT_HARMONIC_FIT_MULT) {
            tracked_peak = FrequencyPeak::LOWER_SHOULDER;
        } else if (num_peaks > 2 && get_tl_noise_center_freq_hz(FrequencyPeak::CENTER, _update_axis) > get_tl_noise_center_freq_hz(FrequencyPeak::UPPER_SHOULDER, _update_axis)
            // ignore the fit if there is too big a discrepancy between the energies
            && get_tl_center_freq_energy(FrequencyPeak::CENTER, _update_axis) < get_tl_center_freq_energy(FrequencyPeak::UPPER_SHOULDER, _update_axis) * FFT_HARMONIC_FIT_MULT) {
            tracked_peak = FrequencyPeak::UPPER_SHOULDER;
        }
    }

    _thread_state._tracked_peak[_update_axis] = tracked_peak;

    // if targetting more than one harmonic then make sure we get the fundamental
    // on larger copters the second harmonic often has more energy
    // if the highest peak is above the second highest then check for harmonic fit
    if (_thread_state._tracked_peak[_update_axis] != FrequencyPeak::CENTER) {
        // calculate the fit and filter at 10hz
        const float harmonic_fit = 100.0f * fabsf(get_tl_noise_center_freq_hz(FrequencyPeak::CENTER, _update_axis)
            - get_tl_noise_center_freq_hz(tracked_peak, _update_axis) * _harmonic_multiplier)
            / get_tl_noise_center_freq_hz(FrequencyPeak::CENTER, _update_axis);

        // calculate the fit and filter at 10hz
        if (isfinite(harmonic_fit)) {
            _thread_state._harmonic_fit[_update_axis] = _harmonic_fit_filter[_update_axis].apply(harmonic_fit);
        }
    } else {
        _thread_state._harmonic_fit[_update_axis] = 100.0f;
    }
#if DEBUG_FFT
    WITH_SEMAPHORE(_sem);
    _debug_state = _thread_state;
    _debug_max_freq_bin = _state->get_freq_bin(_state->_peak_data[FrequencyPeak::CENTER]._bin);
    _debug_max_bin_freq = _state->_peak_data[FrequencyPeak::CENTER]._freq_hz;
    _debug_snr = snr;
    _debug_max_bin = _state->_peak_data[FrequencyPeak::CENTER]._bin;
#endif
}


// calculate noise peaks based on the frequencies closest to the recent historical average, switching peaks around as necessary
uint8_t AP_GyroFFT::calculate_tracking_peaks(float& weighted_center_freq_hz, float& snr, bool calibrating, const EngineConfig& config)
{
    uint8_t num_peaks = 0;
    FrequencyData freqs(*this, config);

    // the noise peaks are returned by the HAL in decreasing order of magnitude, however each peak can temporarily
    // switch places with another depending on a whole host of hardware and software factors
    // thus we must be able to temporarily reassign the peaks so that the filtered values track
    // a continuous frequency
    DistanceMatrix distance_matrix;
    find_distance_matrix(distance_matrix, freqs, config);

    FrequencyPeak center = find_closest_peak(FrequencyPeak::CENTER, distance_matrix);
    FrequencyPeak lower = find_closest_peak(FrequencyPeak::LOWER_SHOULDER, distance_matrix, 1 << center);
    FrequencyPeak upper = find_closest_peak(FrequencyPeak::UPPER_SHOULDER, distance_matrix, 1 << center | 1 << lower);

    if (calibrating || _distorted_cycles[_update_axis] == 0) {
        calculate_filtered_noise(FrequencyPeak::LOWER_SHOULDER, FrequencyPeak::LOWER_SHOULDER, freqs, config) && num_peaks++;
        calculate_filtered_noise(FrequencyPeak::UPPER_SHOULDER, FrequencyPeak::UPPER_SHOULDER, freqs, config) && num_peaks++;
        calculate_filtered_noise(FrequencyPeak::CENTER, FrequencyPeak::CENTER, freqs, config) && num_peaks++;
        _distorted_cycles[_update_axis] = constrain_int16(_distorted_cycles[_update_axis] + 1, 0, FFT_MAX_MISSED_UPDATES);
        _thread_state._tracked_peak[_update_axis] = FrequencyPeak::CENTER;
        weighted_center_freq_hz = freqs.get_weighted_frequency(FrequencyPeak::CENTER);
        snr = freqs.get_signal_to_noise(FrequencyPeak::CENTER);
#if DEBUG_FFT
        printf("Skipped update, order would have been is %d/%.1f(%.1f) %d/%.1f(%.1f) %d/%.1f(%.1f) n = %d\n",
            center, _state->_peak_data[center]._freq_hz, get_tl_noise_center_freq_hz(FrequencyPeak::CENTER, _update_axis),
            lower, _state->_peak_data[lower]._freq_hz, get_tl_noise_center_freq_hz(FrequencyPeak::LOWER_SHOULDER, _update_axis),
            upper, _state->_peak_data[upper]._freq_hz, get_tl_noise_center_freq_hz(FrequencyPeak::UPPER_SHOULDER, _update_axis), num_peaks);
#endif
        return num_peaks;
    }

    // another peak is closer to what is currently considered the center frequency
    if (center != FrequencyPeak::CENTER || lower != FrequencyPeak::LOWER_SHOULDER || upper != FrequencyPeak::UPPER_SHOULDER) {
        if (lower != FrequencyPeak::NONE) {
            calculate_filtered_noise(FrequencyPeak::LOWER_SHOULDER, lower, freqs, config) && num_peaks++;
        }
        if (upper != FrequencyPeak::NONE) {
            calculate_filtered_noise(FrequencyPeak::UPPER_SHOULDER, upper, freqs, config) && num_peaks++;
        }
        if (center != FrequencyPeak::NONE) {
            calculate_filtered_noise(FrequencyPeak::CENTER,  center, freqs, config) && num_peaks++;
        }
        weighted_center_freq_hz = freqs.get_weighted_frequency(center);
        snr = freqs.get_signal_to_noise(center);
        _thread_state._tracked_peak[_update_axis] = center;
        _distorted_cycles[_update_axis]--;
        return num_peaks;
    }

    calculate_filtered_noise(FrequencyPeak::LOWER_SHOULDER, FrequencyPeak::LOWER_SHOULDER, freqs, config) && num_peaks++;
    calculate_filtered_noise(FrequencyPeak::UPPER_SHOULDER, FrequencyPeak::UPPER_SHOULDER, freqs, config) && num_peaks++;
    calculate_filtered_noise(FrequencyPeak::CENTER, FrequencyPeak::CENTER, freqs, config) && num_peaks++;
    _distorted_cycles[_update_axis] = constrain_int16(_distorted_cycles[_update_axis] + 1, 0, FFT_MAX_MISSED_UPDATES);
    weighted_center_freq_hz = freqs.get_weighted_frequency(FrequencyPeak::CENTER);
    snr = freqs.get_signal_to_noise(FrequencyPeak::CENTER);
    _thread_state._tracked_peak[_update_axis] = FrequencyPeak::CENTER;

    return num_peaks;
}

// calculate noise frequencies from FFT data provided by the HAL subsystem
// target_peak is the filtered record we want to apply the new fft data to, source peak is where the fft data is coming from
// called from FFT thread
bool AP_GyroFFT::calculate_filtered_noise(FrequencyPeak target_peak, FrequencyPeak source_peak, const FrequencyData& freqs, const EngineConfig& config)
{
    if (source_peak > FrequencyPeak::MAX_TRACKED_PEAKS) {
        // if we failed to find a signal, carry on using the previous readings
        if (_missed_cycles[_update_axis][target_peak]++ < FFT_MAX_MISSED_UPDATES) {
            return true; // the peak is synthetic
        }
        update_tl_center_freq_energy(target_peak, _update_axis, 0.0f);
        update_tl_noise_center_bandwidth_hz(target_peak, _update_axis, _bandwidth_hover_hz);
        update_tl_noise_center_freq_hz(target_peak, _update_axis, config._fft_min_hz);
        return false;
    }

    AP_HAL::DSP::FrequencyPeakData* peak_data = &_state->_peak_data[source_peak];

    const uint16_t nb = peak_data->_bin;

    if (freqs.is_valid(FrequencyPeak(source_peak))) {
        // total peak energy requires an integration, as an approximation use amplitude * noise width * 5/6
        update_tl_center_freq_energy(target_peak, _update_axis, _state->get_freq_bin(nb) * peak_data->_noise_width_hz * 0.8333f);
        update_tl_noise_center_bandwidth_hz(target_peak, _update_axis, peak_data->_noise_width_hz);
        update_tl_noise_center_freq_hz(target_peak, _update_axis, freqs.get_weighted_frequency(FrequencyPeak(source_peak)));
        _missed_cycles[_update_axis][target_peak] = 0;
        return true;
    }

    // if we failed to find a signal, carry on using the previous readings
    if (_missed_cycles[_update_axis][target_peak]++ < FFT_MAX_MISSED_UPDATES) {
        return true; // the peak is synthetic
    }

    // we failed to find a signal for more than FFT_MAX_MISSED_UPDATES cycles
    update_tl_center_freq_energy(target_peak, _update_axis, _state->get_freq_bin(nb) * peak_data->_noise_width_hz * 0.8333f);     // use the actual energy detected rather than 0
    update_tl_noise_center_bandwidth_hz(target_peak, _update_axis, _bandwidth_hover_hz);
    update_tl_noise_center_freq_hz(target_peak, _update_axis, config._fft_min_hz);

    return false;
}

// filter values through a median sliding window followed by low pass filter
// this eliminates temporary spikes in the detected frequency that are either pure noise
// or a different peak that will erroneously bias the peak we are tracking
float AP_GyroFFT::MedianLowPassFilter3dFloat::apply(uint8_t axis, float sample)
{
    _median_filter[axis].apply(sample);
    const float a = _median_filter[axis].get_sample(0);
    const float b = _median_filter[axis].get_sample(1);
    const float c = _median_filter[axis].get_sample(2);
    float median = MAX(MIN(a, b), MIN(MAX(a, b), c));
    return _lowpass_filter[axis].apply(median);
}

// initialize a FrequencyData structure with peak frequency information for use in the swapping algorithm
AP_GyroFFT::FrequencyData::FrequencyData(const AP_GyroFFT& gyrofft, const EngineConfig& config)
{
    for (uint8_t i = 0; i < FrequencyPeak::MAX_TRACKED_PEAKS; i++) {
        valid[i] = gyrofft.get_weighted_frequency(FrequencyPeak(i), frequency[i], snr[i], config);
    }
}

// calculate noise frequencies from FFT data provided by the HAL subsystem
bool AP_GyroFFT::get_weighted_frequency(FrequencyPeak peak, float& weighted_peak_freq_hz, float& snr, const EngineConfig& config) const
{
    AP_HAL::DSP::FrequencyPeakData* peak_data = &_state->_peak_data[peak];

    const uint16_t bin = peak_data->_bin;

    // calculate the SNR and center frequency energy
    const float max_energy = MAX(1.0f, _state->get_freq_bin(bin));
    const float ref_energy = MAX(1.0f, _ref_energy[bin][_update_axis]);
    snr = 10.f * (log10f(max_energy) - log10f(ref_energy));

    // if the bin energy is above the noise threshold then we have a signal
    if (!_thread_state._noise_needs_calibration && isfinite(_state->get_freq_bin(bin)) && snr > config._snr_threshold_db) {
        weighted_peak_freq_hz = constrain_float(peak_data->_freq_hz, (float)config._fft_min_hz, (float)config._fft_max_hz);
        return true;
    }

    weighted_peak_freq_hz = (float)config._fft_min_hz;

    return false;
}

// calculate a matrix of distances between the current filtered estimates and instantaneous values from the current cycle
void AP_GyroFFT::find_distance_matrix(DistanceMatrix& distance_matrix, const FrequencyData& freqs, const EngineConfig& config) const
{
    float curr_freqs[FrequencyPeak::MAX_TRACKED_PEAKS];
    // get the current frequency estimate for all peaks
    for (uint8_t i = 0; i < FrequencyPeak::MAX_TRACKED_PEAKS; i++) {
        curr_freqs[i] = get_tl_noise_center_freq_hz(FrequencyPeak(i), _update_axis);
    }
    // calculate the matrix
    for (uint8_t i = 0; i < FrequencyPeak::MAX_TRACKED_PEAKS; i++) {
        for (uint8_t j = 0; j < FrequencyPeak::MAX_TRACKED_PEAKS; j++) {
            distance_matrix[i][j] = fabsf((freqs.is_valid(FrequencyPeak(i)) ?
                freqs.get_weighted_frequency(FrequencyPeak(i)) : FLT_MAX) - curr_freqs[j]);
        }
    }
}

// return the instantaneous peak that is closest to the target estimate peak
AP_GyroFFT::FrequencyPeak AP_GyroFFT::find_closest_peak(const FrequencyPeak target, const DistanceMatrix& distance_matrix, uint8_t ignore) const
{
    // find the closest peak to target
    uint8_t closest = target;
    for (uint8_t i = 0; i < FrequencyPeak::MAX_TRACKED_PEAKS; i++) {
        if (distance_matrix[i][target] < distance_matrix[closest][target] && (1 << i & ~ignore)) {
            closest = i;
        }
    }
    // didn't find anything
    if (!(1<<closest & ~ignore)) {
        return FrequencyPeak::NONE;
    }
    return FrequencyPeak(closest);
}

// calculate noise baseline from FFT data provided by the HAL subsystem
// called from FFT thread
void AP_GyroFFT::update_ref_energy(uint16_t max_bin)
{
    if (!_thread_state._noise_needs_calibration) {
        return;
    }

    // according to https://www.tcd.ie/Physics/research/groups/magnetism/files/lectures/py5021/MagneticSensors3.pdf sensor noise is not necessarily gaussian
    // determine a PS noise reference at each of the possible center frequencies
    if (_noise_cycles == 0 && _noise_calibration_cycles[_update_axis] > 0) {
        for (uint16_t i = 1; i < _state->_bin_count; i++) {
            _ref_energy[i][_update_axis] += _state->get_freq_bin(i);
        }
        if (--_noise_calibration_cycles[_update_axis] == 0) {
            for (uint16_t i = 1; i < _state->_bin_count; i++) {
                const float cycles = (static_cast<float>(_window_size) / static_cast<float>(_samples_per_frame)) * 2;
                // overall random noise is reduced by sqrt(N) when averaging periodigrams so adjust for that
                _ref_energy[i][_update_axis] = (_ref_energy[i][_update_axis] / cycles) * sqrtf(cycles);
            }

            WITH_SEMAPHORE(_sem);
            _thread_state._noise_needs_calibration &= ~(1 << _update_axis);
        }
    }
    else if (_noise_cycles > 0) {
        _noise_cycles--;
    }
}

// perform FFT analysis on the range of frequencies supported by the analyser
// called from main thread
float AP_GyroFFT::self_test_bin_frequencies()
{
    if (_state->_window_size * sizeof(float) > hal.util->available_memory() / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: unable to run self-test, required %u bytes", (unsigned int)(_state->_window_size * sizeof(float)));
        return 0.0f;
    }

    FloatBuffer test_window(_state->_window_size);
    // in the unlikely event we can't allocate a test window, skip the checks
    if (test_window.get_size() == 0) {
        return 0.0f;
    }

    float max_divergence = 0;

    for (uint16_t bin = _config._fft_start_bin; bin <= _config._fft_end_bin; bin++) {
        // the algorithm will only ever return values in this range
        float frequency = constrain_float(bin * _state->_bin_resolution, _fft_min_hz, _fft_max_hz);
        max_divergence = MAX(max_divergence, self_test(frequency, test_window)); // test bin centers
        frequency = constrain_float(bin * _state->_bin_resolution - _state->_bin_resolution / 4, _fft_min_hz, _fft_max_hz);
        max_divergence = MAX(max_divergence, self_test(frequency, test_window)); // test bin off-centers
    }

    return max_divergence;
}

// perform FFT analysis of a single sine wave at the selected frequency
// called from main thread
float AP_GyroFFT::self_test(float frequency, FloatBuffer& test_window)
{
    test_window.clear();
    for(uint16_t i = 0; i < _state->_window_size; i++) {
        if (!test_window.push(sinf(2.0f * M_PI * frequency * i / _fft_sampling_rate_hz) * ToRad(20) * 2000)) {
            AP_HAL::panic("Could not create FFT test window");
        }
    }

    _update_axis = 0;

    // if using averaging we need to process _num_frames in order to not bias the result
    for (uint8_t i = 1; i < _num_frames; i++) {
        hal.dsp->fft_start(_state, test_window, 0);
        hal.dsp->fft_analyse(_state, _config._fft_start_bin, _config._fft_end_bin, _config._attenuation_cutoff);
    }
    // final cycle is the one we want
    hal.dsp->fft_start(_state, test_window, 0);
    uint16_t max_bin = hal.dsp->fft_analyse(_state, _config._fft_start_bin, _config._fft_end_bin, _config._attenuation_cutoff);

    if (max_bin == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed, failed to find frequency %.1f", frequency);
    }

    calculate_noise(true, _config);

    float max_divergence = 0;
    // make sure the selected frequencies are in the right bin
    max_divergence = MAX(max_divergence, fabsf(frequency - _thread_state._center_freq_hz[0]));
    if (_thread_state._center_freq_hz[0] < (frequency - MAX(_state->_bin_resolution * 0.5f, 1)) || _thread_state._center_freq_hz[0] > (frequency + MAX(_state->_bin_resolution * 0.5f, 1))) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed: wanted %.1f, had %.1f", frequency, _thread_state._center_freq_hz[0]);
    }
#if DEBUG_FFT
    else {
        gcs().send_text(MAV_SEVERITY_INFO, "FFT: self-test succeeded: wanted %.1f, had %.1f", frequency, _thread_state._center_freq_hz[0]);
    }
#endif

    return max_divergence;
}

// singleton instance
AP_GyroFFT *AP_GyroFFT::_singleton;

namespace AP {

AP_GyroFFT *fft()
{
    return AP_GyroFFT::get_singleton();
}

}

#endif // HAL_GYROFFT_ENABLED
