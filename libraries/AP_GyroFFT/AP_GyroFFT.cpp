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

extern const AP_HAL::HAL& hal;

#ifndef FFT_DEFAULT_WINDOW_SIZE
// the H7 can cope with a longer length and these boards generally have BMI088 which needs a longer length
#if defined(STM32H7)
#define FFT_DEFAULT_WINDOW_SIZE     64
#else
#define FFT_DEFAULT_WINDOW_SIZE     32
#endif
#endif
#define FFT_DEFAULT_WINDOW_OVERLAP  0.5f
#define FFT_THR_REF_DEFAULT         0.35f   // the estimated throttle reference, 0 ~ 1
#define FFT_SNR_DEFAULT            25.0f // a higher SNR is safer and this works quite well on a Pixracer
#define FFT_STACK_SIZE              1024
#define FFT_REQUIRED_HARMONIC_FIT  10.0f

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
    // @Description: Lower bound of FFT frequency detection in Hz.
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MINHZ", 2, AP_GyroFFT, _fft_min_hz, 80),

    // @Param: MAXHZ
    // @DisplayName: Maximum Frequency
    // @Description: Upper bound of FFT frequency detection in Hz.
    // @Range: 10 400
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
    // @Description: Size of window to be used in FFT calculations. Takes effect on reboot. Must be a power of 2 and between 32 and 1024. Larger windows give greater frequency resolution but poorer time resolution, consume more CPU time and may not be appropriate for all vehicles. Time and frequency resolution are given by the sample-rate / window-size. Windows of 256 are only really recommended for F7 class boards, windows of 512 or more H7 class.
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

    AP_GROUPEND
};

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency (Nyquist) of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// The first bin is used to store the DC and Nyquist values combined.
// Eg [DC/Nyquist], [16,47), [47,78), [78,109) etc
// For a loop rate of 800Hz, 16 bins each 25Hz wide
// Eg X[0]=[DC/Nyquist], X[1]=[12,37), X[2]=[37,62), X[3]=[62,87), X[4]=[87,112)
// So middle frequency is X[n] * 25 and the range is X[n] * 25 - 12 < f < X[n] * 25 + 12

// Maximum tolerated number of cycles with missing signal
#define FFT_MAX_MISSED_UPDATES 3

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
void AP_GyroFFT::init(uint32_t target_looptime_us)
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
    _window_size = 1 << lrintf(log2f(_window_size.get()));
#if defined(STM32H7) || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _window_size = constrain_int16(_window_size, 32, 1024);
#else
    _window_size = constrain_int16(_window_size, 32, 256);
#endif

    // check that we have enough memory for the window size requested
    // INS: XYZ_AXIS_COUNT * INS_MAX_INSTANCES * _window_size, DSP: 3 * _window_size, FFT: XYZ_AXIS_COUNT + 3 * _window_size
    const uint32_t allocation_count = (XYZ_AXIS_COUNT * INS_MAX_INSTANCES + 3 + XYZ_AXIS_COUNT + 3) * sizeof(float);
    if (allocation_count * FFT_DEFAULT_WINDOW_SIZE > hal.util->available_memory() / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AP_GyroFFT: disabled, required %u bytes", (unsigned int)allocation_count * FFT_DEFAULT_WINDOW_SIZE);
        return;
    } else if (allocation_count * _window_size > hal.util->available_memory() / 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AP_GyroFFT: req alloc %u bytes (free=%u)", (unsigned int)allocation_count * _window_size, (unsigned int)hal.util->available_memory());
        _window_size = FFT_DEFAULT_WINDOW_SIZE;
    }
    // save any changes that were made
    _window_size.save();

    // determine the FFT sample rate based on the gyro rate, loop rate and configuration
    if (_sample_mode == 0) {
        _fft_sampling_rate_hz = _ins->get_raw_gyro_rate_hz();
    } else {
        const uint16_t loop_rate_hz = 1000*1000UL / target_looptime_us;
        _fft_sampling_rate_hz = loop_rate_hz / _sample_mode;
        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            _downsampled_gyro_data[axis] = (float*)hal.util->malloc_type(sizeof(float) * _window_size, DSP_MEM_REGION);
            if (_downsampled_gyro_data[axis] == nullptr) {
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

    // make the gyro window match the window size plus the maximum that can be in play from the backend
    if (!_ins->set_gyro_window_size(_window_size + INS_MAX_GYRO_WINDOW_SAMPLES)) {
        return;
    }
    // current read marker is the beginning of the window
    if (_sample_mode == 0) {
        _circular_buffer_idx = _ins->get_raw_gyro_window_index();
    } else {
        _circular_buffer_idx = 0;
    }

    // initialise the HAL DSP subsystem
    _state = hal.dsp->fft_init(_window_size, _fft_sampling_rate_hz);
    if (_state == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to initialize DSP engine");
        return;
    }

    // number of samples needed before a new frame can be processed
    _window_overlap = constrain_float(_window_overlap, 0.0f, 0.9f);
    _window_overlap.save();
    _samples_per_frame = (1.0f - _window_overlap) * _window_size;
    _samples_per_frame = 1 << lrintf(log2f(_samples_per_frame));

    // The update rate for the output
    const float output_rate = _fft_sampling_rate_hz / _samples_per_frame;
    // establish suitable defaults for the detected values
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        _thread_state._center_freq_hz[axis] = _fft_min_hz;
        _thread_state._center_freq_hz_filtered[axis] = _fft_min_hz;
        // calculate low-pass filter characteristics based on overlap size
        _center_freq_filter[axis].set_cutoff_frequency(output_rate, output_rate * 0.48f);
        // smooth the bandwidth output more aggressively
        _center_bandwidth_filter[axis].set_cutoff_frequency(output_rate, output_rate * 0.25f);
        // number of cycles to average over, two complete windows to be sure
        _noise_calibration_cycles[axis] = (_window_size / _samples_per_frame) * 2;
    }

    // the number of cycles required to have a proper noise reference
    _noise_cycles = (_window_size / _samples_per_frame) * XYZ_AXIS_COUNT;
    // calculate harmonic multiplier
    uint8_t first_harmonic = 0;
    _harmonics = 1; // always search for 1
    for (uint8_t i = 0; i < HNF_MAX_HARMONICS; i++) {
        if (_ins->get_gyro_harmonic_notch_harmonics() & (1<<i)) {
            if (first_harmonic == 0) {
                first_harmonic = i + 1;
            } else {
                _harmonics++;
                _harmonic_multiplier = float(i + 1) / first_harmonic;
                break;
            }
        }
    }

    // finally we are done
    _initialized = true;
    update_parameters();
    // start running FFTs
    start_update_thread();
}

// sample the gyros either by using a gyro window sampled at the gyro rate or making invdividual samples
// called from fast_loop thread - anything that requires atomic access to IMU data needs to be done here
void AP_GyroFFT::sample_gyros()
{
    if (!analysis_enabled()) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    // update counters for gyro window
    if (_current_sample_mode == 0) {
        // number of available samples are those in the IMU buffer less those we have already consumed
        _sample_count = ((_ins->get_raw_gyro_window_index() - _circular_buffer_idx + get_buffer_size()) % get_buffer_size());

        if (start_analysis()) {
            hal.dsp->fft_start(_state, _ins->get_raw_gyro_window(_update_axis), _circular_buffer_idx, get_buffer_size());
            // we have pushed a frame into the FFT loop, move the index to the beginning of the next frame
            _circular_buffer_idx = (_circular_buffer_idx + _samples_per_frame) % get_buffer_size();
            _sample_count -= _samples_per_frame;
        }
    }
    // for loop rate sampling accumulate and average gyro samples
    else {
        _oversampled_gyro_accum += _ins->get_raw_gyro();
        _oversampled_gyro_count++;

        if ((_oversampled_gyro_count % _current_sample_mode) == 0) {
            // calculate mean value of accumulated samples
            Vector3f sample = _oversampled_gyro_accum / _current_sample_mode;
            // fast sampling means that the raw gyro values have already been averaged over 8 samples
            _downsampled_gyro_data[0][_downsampled_gyro_idx] = sample.x;
            _downsampled_gyro_data[1][_downsampled_gyro_idx] = sample.y;
            _downsampled_gyro_data[2][_downsampled_gyro_idx] = sample.z;

            _oversampled_gyro_accum.zero();
            _oversampled_gyro_count = 0;
            _downsampled_gyro_idx = (_downsampled_gyro_idx + 1) % _state->_window_size;
            _sample_count++;

            if (start_analysis()) {
                hal.dsp->fft_start(_state, _downsampled_gyro_data[_update_axis], _circular_buffer_idx, _state->_window_size);
                _circular_buffer_idx = (_circular_buffer_idx + _samples_per_frame) % _state->_window_size;
                _sample_count -= _samples_per_frame;
            }
        }
    }

    _global_state = _thread_state;
}

// analyse gyro data using FFT, returns number of samples still held
// called from FFT thread
uint16_t AP_GyroFFT::update()
{
    if (!analysis_enabled()) {
        return 0;
    }

    if (!_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return 0;
    }

    // only proceeed if a full frame has been pushed into the dsp
    if (!_thread_state._analysis_started) {
        uint16_t new_sample_count = _sample_count;
        _sem.give();
        return new_sample_count;
    }

    uint16_t start_bin = _config._fft_start_bin;
    uint16_t end_bin = _config._fft_end_bin;

    _sem.give();

    // calculate FFT and update filters outside the semaphore
    uint16_t bin_max = hal.dsp->fft_analyse(_state, start_bin, end_bin, _harmonics, _config._attenuation_cutoff);

    // in order to access _config state we need the semaphore again
    WITH_SEMAPHORE(_sem);

    // something has been detected, update the peak frequency and associated metrics
    update_ref_energy(bin_max);
    calculate_noise(bin_max);

    // ready to receive another frame
    _thread_state._analysis_started = false;

    return _sample_count;
}

    // whether analysis can be run again or not
bool AP_GyroFFT::start_analysis() {
    if (_thread_state._analysis_started) {
        return false;
    }
    if (_sample_count >= _state->_window_size) {
        _thread_state._analysis_started = true;
        return true;
    }
    return false;
}

// update calculated values of dynamic parameters - runs at 1Hz
void AP_GyroFFT::update_parameters()
{
    if (!_initialized) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    // don't allow MAXHZ to go to Nyquist
    _fft_max_hz = MIN(_fft_max_hz, _fft_sampling_rate_hz * 0.48);
    _config._snr_threshold_db = _snr_threshold_db;
    _config._fft_min_hz = _fft_min_hz;
    _config._fft_max_hz = _fft_max_hz;
    // determine the start FFT bin for all frequency detection
    _config._fft_start_bin = MAX(floorf(_fft_min_hz.get() / _state->_bin_resolution), 1);
    // determine the endt FFT bin for all frequency detection
    _config._fft_end_bin = MIN(ceilf(_fft_max_hz.get() / _state->_bin_resolution), _state->_bin_count);
    // actual attenuation from the db value
    _config._attenuation_cutoff = powf(10.0f, -_attenuation_power_db / 10.0f);
    _config._analysis_enabled = _analysis_enabled;
}

// thread for processing gyro data via FFT
void AP_GyroFFT::update_thread(void)
{
    while (true) {
        uint16_t remaining_samples = update();
        // wait approximately until we are likely to have another frame ready
        uint32_t delay = constrain_int32((int32_t(_window_size) - remaining_samples) * 1e6 / _fft_sampling_rate_hz, 0, 100000);
        if (delay > 0) {
            hal.scheduler->delay_microseconds(delay);
        }
    }
}

// start the update thread
void AP_GyroFFT::start_update_thread(void)
{
    WITH_SEMAPHORE(_sem);

    if (_thread_created) {
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_GyroFFT::update_thread, void), "apm_fft", FFT_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        AP_HAL::panic("Failed to start AP_GyroFFT update thread");
    }

    _thread_created = true;
}

// self-test the FFT analyser - can only be done while samples are not being taken
// called from main thread
bool AP_GyroFFT::calibration_check()
{
    if (!analysis_enabled()) {
        return true;
    }

    // analysis is started in the main thread, don't trample on in-flight analysis
    if (_global_state._analysis_started) {
        return true;
    }

    // still calibrating noise so not ready
    if (_global_state._noise_needs_calibration) {
        return false;
    }

    // make sure the frequency maxium is below Nyquist
    if (_fft_max_hz > _fft_sampling_rate_hz * 0.5f) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "FFT: config MAXHZ %dHz > %dHz", _fft_max_hz.get(), _fft_sampling_rate_hz / 2);
        return false;
    }

    // check for sane frequency resolution - for 1k backends with length 32 this will be 32Hz
    if (_state->_bin_resolution > 50.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: resolution is %.1fHz, increase length", _state->_bin_resolution);
        return true; // a low resolution is not fatal
    }

    // larger windows make the the self-test run too long, triggering the watchdog
    if (AP_Logger::get_singleton()->log_while_disarmed()
        || _window_size > FFT_DEFAULT_WINDOW_SIZE * 2) {
        return true;
    }
    float max_divergence = self_test_bin_frequencies();

    if (max_divergence > _state->_bin_resolution * 0.5f) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test FAILED, max error %fHz", max_divergence);
    }
    return max_divergence <= _state->_bin_resolution * 0.5f;
}

// update the hover frequency input filter. should be called at 100hz when in a stable hover
// called from main thread
void AP_GyroFFT::update_freq_hover(float dt, float throttle_out)
{
    if (!analysis_enabled()) {
        return;
    }
    // we have chosen to constrain the hover frequency to be within the range reachable by the third order expo polynomial.
    _freq_hover_hz = constrain_float(_freq_hover_hz + (dt / (10.0f + dt)) * (get_weighted_noise_center_freq_hz() - _freq_hover_hz), _fft_min_hz, _fft_max_hz);
    _bandwidth_hover_hz = constrain_float(_bandwidth_hover_hz + (dt / (10.0f + dt)) * (get_weighted_noise_center_bandwidth_hz() - _bandwidth_hover_hz), 0, _fft_max_hz * 0.5f);
    _throttle_ref = constrain_float(_throttle_ref + (dt / (10.0f + dt)) * (throttle_out * sq((float)_fft_min_hz.get() / _freq_hover_hz.get()) - _throttle_ref), 0.01f, 0.9f);
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

// return an average center frequency weighted by bin energy
// called from main thread
float AP_GyroFFT::get_weighted_noise_center_freq_hz()
{
    if (!analysis_enabled()) {
        return _fft_min_hz;
    }

    // there is generally a lot of high-energy, slightly lower frequency noise on yaw, however this
    // appears to be a second-order effect as only targetting pitch and roll (x & y) produces much cleaner output all round
    if (!_global_state._center_freq_energy.is_nan()
        && !is_zero(_global_state._center_freq_energy.x)
        && !is_zero(_global_state._center_freq_energy.y)) {
        return (_global_state._center_freq_hz_filtered.x * _global_state._center_freq_energy.x
            + _global_state._center_freq_hz_filtered.y * _global_state._center_freq_energy.y)
            / (_global_state._center_freq_energy.x + _global_state._center_freq_energy.y);
    }
    else {
        return (_global_state._center_freq_hz_filtered.x + _global_state._center_freq_hz_filtered.y) * 0.5f;
    }
}

static const char* LOG_FTN1_NAME = "FTN1";
static const char* LOG_FTN1_LABELS = "TimeUS,PkAvg,PkX,PkY,PkZ,BwAvg,BwX,BwY,BwZ,DnF";
static const char* LOG_FTN1_UNITS = "szzzzzzzzz";
static const char* LOG_FTN1_SCALE = "F---------";
static const char* LOG_FTN1_FORMAT = "Qfffffffff";

static const char* LOG_FTN2_NAME = "FTN2";
static const char* LOG_FTN2_LABELS = "TimeUS,FtX,FtY,FtZ,EnX,EnY,EnZ,SnX,SnY,SnZ,Bin";
static const char* LOG_FTN2_UNITS = "s%%%-------";
static const char* LOG_FTN2_SCALE = "F----------";
static const char* LOG_FTN2_FORMAT = "QfffffffffB";

// log gyro fft messages
void AP_GyroFFT::write_log_messages()
{
    if (!analysis_enabled()) {
        return;
    }

    AP::logger().Write(LOG_FTN1_NAME, LOG_FTN1_LABELS, LOG_FTN1_UNITS, LOG_FTN1_SCALE, LOG_FTN1_FORMAT,
        AP_HAL::micros64(),
        get_weighted_noise_center_freq_hz(),
        get_noise_center_freq_hz().x,
        get_noise_center_freq_hz().y,
        get_noise_center_freq_hz().z,
        get_weighted_noise_center_bandwidth_hz(),
        get_noise_center_bandwidth_hz().x,
        get_noise_center_bandwidth_hz().y,
        get_noise_center_bandwidth_hz().z,
        _ins->get_gyro_dynamic_notch_center_freq_hz());

    AP::logger().Write(LOG_FTN2_NAME, LOG_FTN2_LABELS, LOG_FTN2_UNITS, LOG_FTN2_SCALE, LOG_FTN2_FORMAT,
        AP_HAL::micros64(),
        get_raw_noise_harmonic_fit().x,
        get_raw_noise_harmonic_fit().y,
        get_raw_noise_harmonic_fit().z,
        get_center_freq_energy().x,
        get_center_freq_energy().y,
        get_center_freq_energy().z,
        get_noise_signal_to_noise_db().x,
        get_noise_signal_to_noise_db().y,
        get_noise_signal_to_noise_db().z,
        get_center_freq_bin().z);

#if DEBUG_FFT
    const uint32_t now = AP_HAL::millis();
    // output at 1hz
    if ((now - _last_output_ms) > 1000) {
        // doing this from the update thread overflows the stack
        WITH_SEMAPHORE(_sem);
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: f:%.1f, fr:%.1f, b:%u, fd:%.1f",
                        _debug_state._center_freq_hz_filtered[_update_axis], _debug_state._center_freq_hz[_update_axis], _debug_max_bin, _debug_max_bin_freq);
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: bw:%.1f, e:%.1f, r:%.1f, snr:%.1f",
                        _debug_state._center_bandwidth_hz[_update_axis], _debug_max_freq_bin, _ref_energy[_update_axis][_debug_max_bin], _debug_snr);
        _last_output_ms = now;
    }
#endif
}

// return an average noise bandwidth weighted by bin energy
// called from main thread
float AP_GyroFFT::get_weighted_noise_center_bandwidth_hz()
{
    if (!analysis_enabled()) {
        return 0.0f;
    }

    if (!_global_state._center_freq_energy.is_nan()
        && !is_zero(_global_state._center_freq_energy.x)
        && !is_zero(_global_state._center_freq_energy.y)) {
        return (_global_state._center_bandwidth_hz.x * _global_state._center_freq_energy.x
            + _global_state._center_bandwidth_hz.y * _global_state._center_freq_energy.y)
            / (_global_state._center_freq_energy.x + _global_state._center_freq_energy.y);
    }
    else {
        return (_global_state._center_bandwidth_hz.x + _global_state._center_bandwidth_hz.y) * 0.5f;
    }
}

// calculate noise frequencies from FFT data provided by the HAL subsystem
// called from FFT thread
void AP_GyroFFT::calculate_noise(uint16_t max_bin)
{
    _thread_state._center_freq_bin[_update_axis] = max_bin;

    float weighted_center_freq_hz = 0;

    // cacluate the SNR and center frequency energy
    const float max_energy = MAX(1.0f, _state->_freq_bins[max_bin]);
    const float ref_energy = MAX(1.0f, _ref_energy[_update_axis][max_bin]);
    float snr = 10.f * log10f(max_energy) - log10f(ref_energy);
    // if the bin energy is above the noise threshold then we have a signal
    if (!_thread_state._noise_needs_calibration && !isnan(_state->_freq_bins[max_bin]) && snr > _config._snr_threshold_db) {
        // if targetting more than one harmonic then make sure we get the fundamental
        // on larger copters the second harmonic often has more energy
        const float peak_freq_hz = constrain_float(_state->_max_bin_freq, (float)_config._fft_min_hz, (float)_config._fft_max_hz);
        const float second_peak_freq_hz = constrain_float(_state->_second_bin_freq, (float)_config._fft_min_hz, (float)_config._fft_max_hz);
        const float harmonic_fit = 100.0f * fabsf(peak_freq_hz - second_peak_freq_hz * _harmonic_multiplier) / peak_freq_hz;

        // required fit of 10% is fairly conservative when testing in SITL
        if (_harmonics > 1 && peak_freq_hz > second_peak_freq_hz && harmonic_fit < FFT_REQUIRED_HARMONIC_FIT) {
            weighted_center_freq_hz = second_peak_freq_hz;
            _thread_state._center_freq_energy[_update_axis] = _state->_freq_bins[_state->_second_energy_bin];
            _thread_state._center_bandwidth_hz[_update_axis] = _center_bandwidth_filter[_update_axis].apply(_state->_second_noise_width_hz);
        } else {
            weighted_center_freq_hz = peak_freq_hz;
            _thread_state._center_freq_energy[_update_axis] = _state->_freq_bins[max_bin];
            _thread_state._center_bandwidth_hz[_update_axis] = _center_bandwidth_filter[_update_axis].apply(_state->_max_noise_width_hz);
        }
        // record how good the fit was
        if (peak_freq_hz > second_peak_freq_hz) {
            _thread_state._harmonic_fit[_update_axis] = harmonic_fit;
        } else {
            _thread_state._harmonic_fit[_update_axis] = 0.0f;
        }
        _thread_state._center_freq_hz[_update_axis] = weighted_center_freq_hz;
        _thread_state._center_snr[_update_axis] = snr;
        _missed_cycles = 0;
    }
    // if we failed to find a signal, carry on using the previous reading
    else if (_missed_cycles++ < FFT_MAX_MISSED_UPDATES) {
        weighted_center_freq_hz = _thread_state._center_freq_hz[_update_axis];
        // Keep the previous center frequency and energy
    }
    // we failed to find a signal for more than FFT_MAX_MISSED_UPDATES cycles
    else {
        weighted_center_freq_hz = _config._fft_min_hz;
        _thread_state._center_freq_hz[_update_axis] = _config._fft_min_hz;
        _thread_state._center_freq_energy[_update_axis] = 0.0f;
        _thread_state._center_snr[_update_axis] = 0.0f;
        _thread_state._center_bandwidth_hz[_update_axis] = _center_bandwidth_filter[_update_axis].apply(_bandwidth_hover_hz);
    }

    _thread_state._center_freq_hz_filtered[_update_axis] = _center_freq_filter[_update_axis].apply(weighted_center_freq_hz);

#if DEBUG_FFT
    WITH_SEMAPHORE(_sem);
    _debug_state = _thread_state;
    _debug_max_freq_bin = _state->_freq_bins[max_bin];
    _debug_max_bin_freq = _state->_max_bin_freq;
    _debug_snr = snr;
    _debug_max_bin = max_bin;
#endif
    _update_axis = (_update_axis + 1) % XYZ_AXIS_COUNT;
}

// calculate noise baseline from FFT data provided by the HAL subsystem
// called from FFT thread
void AP_GyroFFT::update_ref_energy(uint16_t max_bin)
{
    if (!_thread_state._noise_needs_calibration) {
        return;
    }
    // according to https://www.tcd.ie/Physics/research/groups/magnetism/files/lectures/py5021/MagneticSensors3.pdf sensor noise is not necessarily gaussian
    // determine a PS noise reference at each of the possble center frequencies
    if (_noise_cycles == 0 && _noise_calibration_cycles[_update_axis] > 0) {
        for (uint16_t i = 1; i < _state->_bin_count; i++) {
            _ref_energy[_update_axis][i] += _state->_freq_bins[i];
        }
        if (--_noise_calibration_cycles[_update_axis] == 0) {
            for (uint16_t i = 1; i < _state->_bin_count; i++) {
                const float cycles = (_window_size / _samples_per_frame) * 2;
                // overall random noise is reduced by sqrt(N) when averaging periodigrams so adjust for that
                _ref_energy[_update_axis][i] = (_ref_energy[_update_axis][i] / cycles) * sqrtf(cycles);
            }
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

    GyroWindow test_window = (float*)hal.util->malloc_type(sizeof(float) * _state->_window_size, DSP_MEM_REGION);
    // in the unlikely event we can't allocate a test window, skip the checks
    if (test_window == nullptr) {
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

    hal.util->free_type(test_window, sizeof(float) * _window_size, DSP_MEM_REGION);
    return max_divergence;
}

// perform FFT analysis of a single sine wave at the selected frequency
// called from main thread
float AP_GyroFFT::self_test(float frequency, GyroWindow test_window)
{
    for(uint16_t i = 0; i < _state->_window_size; i++) {
        test_window[i]= sinf(2.0f * M_PI * frequency * i / _fft_sampling_rate_hz) * ToRad(20) * 2000;
    }

    _update_axis = 0;

    hal.dsp->fft_start(_state, test_window, 0, _state->_window_size);
    uint16_t max_bin = hal.dsp->fft_analyse(_state, _config._fft_start_bin, _config._fft_end_bin, _harmonics, _config._attenuation_cutoff);

    if (max_bin <= 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed, failed to find frequency %f", frequency);
    }

    calculate_noise(max_bin);

    float max_divergence = 0;
    // make sure the selected frequencies are in the right bin
    max_divergence = MAX(max_divergence, fabsf(frequency - _thread_state._center_freq_hz[0]));
    if (_thread_state._center_freq_hz[0] < (frequency - _state->_bin_resolution * 0.5f) || _thread_state._center_freq_hz[0] > (frequency + _state->_bin_resolution * 0.5f)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FFT: self-test failed: wanted %f, had %f", frequency, _thread_state._center_freq_hz[0]);
    }
#if DEBUG_FFT
    else {
        gcs().send_text(MAV_SEVERITY_INFO, "FFT: self-test succeeded: wanted %f, had %f", frequency, _thread_state._center_freq_hz[0]);
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
