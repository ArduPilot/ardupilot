/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper and the betaflight team
 */

#include <hal.h>
#include "AP_HAL_ChibiOS.h"

#if HAL_WITH_DSP

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "DSP.h"
#include <cmath>

using namespace ChibiOS;

#if DEBUG_FFT
#define TIMER_START(timer) \
void *istate = hal.scheduler->disable_interrupts_save(); \
uint32_t timer##now = AP_HAL::micros()
#define TIMER_END(timer) timer.time(timer##now); \
hal.scheduler->restore_interrupts(istate)
#else
#define TIMER_START(timer)
#define TIMER_END(timer)
#endif

#define TICK_CYCLE 10

extern const AP_HAL::HAL& hal;

// The algorithms originally came from betaflight but are now substantially modified based on theory and experiment.
// https://holometer.fnal.gov/GH_FFT.pdf "Spectrum and spectral density estimation by the Discrete Fourier transform (DFT),
// including a comprehensive list of window functions and some new flat-top windows." - Heinzel et. al is a great reference
// for understanding the underlying theory although we do not use spectral density here since time resolution is equally
// important as frequency resolution. Referred to as [Heinz] throughout the code.

// initialize the FFT state machine
AP_HAL::DSP::FFTWindowState* DSP::fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
{
    DSP::FFTWindowStateARM* fft = new DSP::FFTWindowStateARM(window_size, sample_rate, sliding_window_size);
    if (fft == nullptr || fft->_hanning_window == nullptr || fft->_rfft_data == nullptr || fft->_freq_bins == nullptr || fft->_derivative_freq_bins == nullptr) {
        delete fft;
        return nullptr;
    }
    return fft;
}

// start an FFT analysis
void DSP::fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance)
{
    step_hanning((FFTWindowStateARM*)state, samples, advance);
}

// perform remaining steps of an FFT analysis
uint16_t DSP::fft_analyse(AP_HAL::DSP::FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    FFTWindowStateARM* fft = (FFTWindowStateARM*)state;
    step_arm_cfft_f32(fft);
    step_bitreversal(fft);
    step_stage_rfft_f32(fft);
    step_arm_cmplx_mag_f32(fft, start_bin, end_bin, noise_att_cutoff);
    return step_calc_frequencies_f32(fft, start_bin, end_bin);
}

// create an instance of the FFT state machine
DSP::FFTWindowStateARM::FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
    : AP_HAL::DSP::FFTWindowState::FFTWindowState(window_size, sample_rate, sliding_window_size)
{
    if (_freq_bins == nullptr || _hanning_window == nullptr || _rfft_data == nullptr || _derivative_freq_bins == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to allocate %u bytes for window %u for DSP",
            unsigned(sizeof(float) * (window_size * 3 + 2)), unsigned(window_size));
        return;
    }

    // initialize the ARM data structure.
    // it's important not to use arm_rfft_fast_init_f32() as this links all of the twiddle tables
    // by being selective we save 70k in text space

    switch (window_size) {
    case 32:
        arm_rfft_32_fast_init_f32(&_fft_instance);
        break;
    case 64:
        arm_rfft_64_fast_init_f32(&_fft_instance);
        break;
    case 128:
        arm_rfft_128_fast_init_f32(&_fft_instance);
        break;
    case 256:
        arm_rfft_256_fast_init_f32(&_fft_instance);
        break;
#if defined(STM32H7)
// Don't pull in the larger FFT tables unless we have to
    case 512:
        arm_rfft_512_fast_init_f32(&_fft_instance);
        break;
    case 1024:
        arm_rfft_1024_fast_init_f32(&_fft_instance);
        break;
#endif
    }
}

DSP::FFTWindowStateARM::~FFTWindowStateARM() {}

extern "C" {
    void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
    void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
    void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);
}

// step 1: filter the incoming samples through a Hanning window
void DSP::step_hanning(FFTWindowStateARM* fft, FloatBuffer& samples, uint16_t advance)
{
    TIMER_START(_hanning_timer);

    // 5us
    // apply hanning window to gyro samples and store result in _freq_bins
    // hanning starts and ends with 0, could be skipped for minor speed improvement
    samples.peek(&fft->_freq_bins[0], fft->_window_size); // the caller ensures we get a full buffer of samples
    samples.advance(advance);
    arm_mult_f32(&fft->_freq_bins[0], &fft->_hanning_window[0], &fft->_freq_bins[0], fft->_window_size);

    TIMER_END(_hanning_timer);
}

// step 2: guts of complex fft processing
void DSP::step_arm_cfft_f32(FFTWindowStateARM* fft)
{
    arm_cfft_instance_f32 *Sint = &(fft->_fft_instance.Sint);
    Sint->fftLen = fft->_fft_instance.fftLenRFFT / 2;

    TIMER_START(_arm_cfft_f32_timer);

    switch (fft->_bin_count) {
    case 16: // window 32
        // 16us (BF)
        //  5us F7,  7us F4, 8us H7
    case 128: // window 256
        // 37us F7, 81us F4, 17us H7
        arm_cfft_radix8by2_f32(Sint, fft->_freq_bins);
        break;
    case 32: // window 64
        // 35us (BF)
        // 10us F7,  24us F4
    case 256: // window 512
        // 66us F7, 174us F4, 37us H7
        arm_cfft_radix8by4_f32(Sint, fft->_freq_bins);
        break;
    case 64: // window 128
        // 70us BF
        // 21us F7, 34us F4
    case 512: // window 1024
        // 152us F7, 73us H7
        arm_radix8_butterfly_f32(fft->_freq_bins, fft->_bin_count, Sint->pTwiddle, 1);
        break;
    }

    TIMER_END(_arm_cfft_f32_timer);
}

// step 3: reverse the bits of the output
void DSP::step_bitreversal(FFTWindowStateARM* fft)
{
    TIMER_START(_bitreversal_timer);
    // 6us (BF)
    // 32   -  2us F7,  3us F4, 1us H7
    // 64   -  3us F7,  6us F4
    // 128  -  4us F7,  9us F4
    // 256  - 10us F7, 20us F4, 5us H7
    // 512  - 22us F7, 54us F4, 15us H7
    // 1024 - 42us F7,          15us H7
    arm_bitreversal_32((uint32_t *)fft->_freq_bins, fft->_fft_instance.Sint.bitRevLength, fft->_fft_instance.Sint.pBitRevTable);

    TIMER_END(_bitreversal_timer);
}

// step 4: convert from complex to real data
void DSP::step_stage_rfft_f32(FFTWindowStateARM* fft)
{
    TIMER_START(_stage_rfft_f32_timer);
    // 14us (BF)
    // 32   -  2us F7,  5us F4,  2us H7
    // 64   -  5us F7, 16us F4
    // 128  - 17us F7, 26us F4
    // 256  - 21us F7, 70us F4,  9us H7
    // 512  - 35us F7, 71us F4, 17us H7
    // 1024 - 76us F7,          33us H7
    // this does not work in place => _freq_bins AND _rfft_data needed
    stage_rfft_f32(&fft->_fft_instance, fft->_freq_bins, fft->_rfft_data);

    TIMER_END(_stage_rfft_f32_timer);
}

// step 5: find the magnitudes of the complex data
void DSP::step_arm_cmplx_mag_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    TIMER_START(_arm_cmplx_mag_f32_timer);

    // 8us (BF)
    // 32   -   4us F7,  5us F4,  5us H7
    // 64   -   7us F7, 13us F4
    // 128  -  14us F7, 17us F4
    // 256  -  29us F7, 28us F4,  7us H7
    // 512  -  55us F7, 93us F4, 13us H7
    // 1024 - 131us F7,          25us H7
    // General case for the magnitudes - see https://stackoverflow.com/questions/42299932/dsp-libraries-rfft-strange-results
    // The frequency of each of those frequency components are given by k*fs/N

    arm_cmplx_mag_squared_f32(&fft->_rfft_data[2], &fft->_freq_bins[1], fft->_bin_count - 1);
    fft->_freq_bins[0] = sq(fft->_rfft_data[0]);               // DC
    fft->_freq_bins[fft->_bin_count] = sq(fft->_rfft_data[1]); // Nyquist
    fft->_rfft_data[fft->_window_size] = fft->_rfft_data[1]; // Nyquist for the interpolator
    fft->_rfft_data[fft->_window_size + 1] = 0;

    step_cmplx_mag(fft, start_bin, end_bin, noise_att_cutoff);

    TIMER_END(_arm_cmplx_mag_f32_timer);
}

// step 6: find the bin with the highest energy and interpolate the required frequency
uint16_t DSP::step_calc_frequencies_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin)
{
    TIMER_START(_step_calc_frequencies);
    // 4us H7

    step_calc_frequencies(fft, start_bin, end_bin);

    TIMER_END(_step_calc_frequencies);

#if DEBUG_FFT
    _output_count++;
    // outputs at approx 1hz
    if (_output_count % 400 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FFT(us): t1:%lu,t2:%lu,t3:%lu,t4:%lu,t5:%lu,t6:%lu",
                        _hanning_timer._timer_avg, _arm_cfft_f32_timer._timer_avg, _bitreversal_timer._timer_avg, _stage_rfft_f32_timer._timer_avg, _arm_cmplx_mag_f32_timer._timer_avg, _step_calc_frequencies._timer_avg);
    }
#endif

    return fft->_peak_data[CENTER]._bin;
}

static const float PI_N = M_PI / 32.0f;
static const float CANDAN_FACTOR = tanf(PI_N) / PI_N;

// Interpolate center frequency using http://users.metu.edu.tr/ccandan//pub_dir/FineDopplerEst_IEEE_SPL_June2011.pdf
// This is slightly less accurate than Quinn, but much cheaper to calculate
float DSP::calculate_candans_estimator(const FFTWindowStateARM* fft, uint16_t k_max) const
{
    if (k_max <= 1 || k_max == fft->_bin_count) {
        return 0.0f;
    }

    const uint16_t k_m1 = (k_max - 1) * 2;
    const uint16_t k_p1 = (k_max + 1) * 2;
    const uint16_t k = k_max * 2;

    const float npr = fft->_rfft_data[k_m1] - fft->_rfft_data[k_p1];
    const float npc = fft->_rfft_data[k_m1 + 1] - fft->_rfft_data[k_p1 + 1];
    const float dpr = 2.0f * fft->_rfft_data[k] - fft->_rfft_data[k_m1] - fft->_rfft_data[k_p1];
    const float dpc = 2.0f * fft->_rfft_data[k + 1] - fft->_rfft_data[k_m1 + 1] - fft->_rfft_data[k_p1 + 1];

    const float realn = npr * dpr + npc * dpc;
    const float reald = dpr * dpr + dpc * dpc;

    // sanity check
    if (is_zero(reald)) {
        return 0.0f;
    }

    float d = CANDAN_FACTOR * (realn / reald);

    // -0.5 < d < 0.5 which is the fraction of the sample spacing about the center element
    return constrain_float(d, -0.5f, 0.5f);
}

#if DEBUG_FFT
 void DSP::StepTimer::time(uint32_t start)
 {
    _timer_total += (AP_HAL::micros() - start);
    _time_ticks = (_time_ticks + 1) % TICK_CYCLE;
    if (_time_ticks == 0) {
        _timer_avg = _timer_total / TICK_CYCLE;
        _timer_total = 0;
    }
}
#endif

#endif
