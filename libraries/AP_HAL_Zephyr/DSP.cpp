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
 * Ported from AP_HAL_ChibiOS/DSP.cpp (Andy Piper and the betaflight team).
 *
 * PORTING NOTES (why this isn't a byte-for-byte copy):
 *
 * ChibiOS's version calls arm_rfft_32_fast_init_f32()/_64_/_128_/_256_/
 * _512_/_1024_ (size-first naming) and, for the complex-FFT stage, reaches
 * directly into CMSIS-DSP's internal radix-8 fast paths
 * (arm_cfft_radix8by2_f32/arm_cfft_radix8by4_f32/arm_radix8_butterfly_f32,
 * forward-declared extern "C" since the public header doesn't expose them)
 * plus a manual bit-reversal and stage-conversion step, explicitly to avoid
 * arm_rfft_fast_init_f32()'s generic dispatcher, which links every size's
 * twiddle table ("by being selective we save 70k in text space").
 *
 * Zephyr's vendored CMSIS-DSP (modules/zephyr/modules/lib/cmsis-dsp,
 * v1.10.0) has the size-specific init functions too, but under
 * fast_init_SIZE naming (arm_rfft_fast_init_32_f32() etc.) - and its
 * arm_cfft_radix8by2_f32()/arm_cfft_radix8by4_f32() are `static` in
 * arm_cfft_f32.c (internal linkage, called only from arm_cfft_f32()'s own
 * dispatch) - the extern "C" forward-declare trick ChibiOS uses would
 * compile but fail to LINK here, since a static function emits no external
 * symbol.
 *
 * So: steps 2-4 of ChibiOS's six-step breakdown (complex FFT, bit-reversal,
 * real-from-complex conversion) collapse into one call to CMSIS-DSP's
 * public arm_rfft_fast_f32(), which does the same work internally. This is
 * the standard, documented CMSIS-DSP entry point - less hand-optimized
 * than ChibiOS's approach, but correct, and the only path this vendored
 * copy actually exposes. RT1176's flash budget (64 MiB external NOR, 1.54%
 * used as of this port) has no reason to chase ChibiOS's 70k text-space
 * saving, so all six window sizes (32-1024) are initialised unconditionally
 * here rather than gating 512/1024 behind an STM32H7-equivalent check.
 *
 * Confirmed present and correct in this CMSIS-DSP tree before writing this:
 * arm_rfft_fast_init_{32,64,128,256,512,1024}_f32(), arm_rfft_fast_f32(),
 * arm_cmplx_mag_squared_f32(), arm_mult_f32(), arm_max_f32(),
 * arm_scale_f32(), arm_mean_f32(), arm_add_f32() - all public API, all
 * externally linkable.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "DSP.h"

#if HAL_WITH_DSP

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>

using namespace Zephyr;

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

extern const AP_HAL::HAL &hal;

// initialize the FFT state machine
AP_HAL::DSP::FFTWindowState *DSP::fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
{
    DSP::FFTWindowStateARM *fft = NEW_NOTHROW DSP::FFTWindowStateARM(window_size, sample_rate, sliding_window_size);
    if (fft == nullptr || fft->_hanning_window == nullptr || fft->_rfft_data == nullptr ||
        fft->_freq_bins == nullptr || fft->_derivative_freq_bins == nullptr) {
        delete fft;
        return nullptr;
    }
    return fft;
}

// start an FFT analysis
void DSP::fft_start(FFTWindowState *state, FloatBuffer &samples, uint16_t advance)
{
    step_hanning((FFTWindowStateARM *)state, samples, advance);
}

// perform remaining steps of an FFT analysis
uint16_t DSP::fft_analyse(AP_HAL::DSP::FFTWindowState *state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    FFTWindowStateARM *fft = (FFTWindowStateARM *)state;
    step_rfft_f32(fft);
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

    // See this file's header comment: fast_init_SIZE naming, not
    // ChibiOS's SIZE_fast_init, is what this CMSIS-DSP tree exposes.
    switch (window_size) {
    case 32:
        arm_rfft_fast_init_32_f32(&_fft_instance);
        break;
    case 64:
        arm_rfft_fast_init_64_f32(&_fft_instance);
        break;
    case 128:
        arm_rfft_fast_init_128_f32(&_fft_instance);
        break;
    case 256:
        arm_rfft_fast_init_256_f32(&_fft_instance);
        break;
    case 512:
        arm_rfft_fast_init_512_f32(&_fft_instance);
        break;
    case 1024:
        arm_rfft_fast_init_1024_f32(&_fft_instance);
        break;
    }
}

DSP::FFTWindowStateARM::~FFTWindowStateARM() {}

// step 1: filter the incoming samples through a Hanning window
void DSP::step_hanning(FFTWindowStateARM *fft, FloatBuffer &samples, uint16_t advance)
{
    TIMER_START(_hanning_timer);

    // apply hanning window to gyro samples and store result in _freq_bins
    samples.peek(&fft->_freq_bins[0], fft->_window_size);  // the caller ensures we get a full buffer of samples
    samples.advance(advance);
    arm_mult_f32(&fft->_freq_bins[0], &fft->_hanning_window[0], &fft->_freq_bins[0], fft->_window_size);

    TIMER_END(_hanning_timer);
}

// step 2: real FFT via CMSIS-DSP's public entry point - see this file's
// header comment for why this replaces ChibiOS's separate cfft/bitreversal/
// stage_rfft steps.
void DSP::step_rfft_f32(FFTWindowStateARM *fft)
{
    TIMER_START(_rfft_f32_timer);

    // this does not work in place => _freq_bins AND _rfft_data needed
    arm_rfft_fast_f32(&fft->_fft_instance, fft->_freq_bins, fft->_rfft_data, 0);

    TIMER_END(_rfft_f32_timer);
}

// step 3: find the magnitudes of the complex data
void DSP::step_arm_cmplx_mag_f32(FFTWindowStateARM *fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    TIMER_START(_arm_cmplx_mag_f32_timer);

    // General case for the magnitudes - see https://stackoverflow.com/questions/42299932/dsp-libraries-rfft-strange-results
    // The frequency of each of those frequency components are given by k*fs/N
    arm_cmplx_mag_squared_f32(&fft->_rfft_data[2], &fft->_freq_bins[1], fft->_bin_count - 1);
    fft->_freq_bins[0] = sq(fft->_rfft_data[0]);                // DC
    fft->_freq_bins[fft->_bin_count] = sq(fft->_rfft_data[1]);  // Nyquist
    fft->_rfft_data[fft->_window_size] = fft->_rfft_data[1];    // Nyquist for the interpolator
    fft->_rfft_data[fft->_window_size + 1] = 0;

    step_cmplx_mag(fft, start_bin, end_bin, noise_att_cutoff);

    TIMER_END(_arm_cmplx_mag_f32_timer);
}

// step 4: find the bin with the highest energy and interpolate the required frequency
uint16_t DSP::step_calc_frequencies_f32(FFTWindowStateARM *fft, uint16_t start_bin, uint16_t end_bin)
{
    TIMER_START(_step_calc_frequencies);

    step_calc_frequencies(fft, start_bin, end_bin);

    TIMER_END(_step_calc_frequencies);

#if DEBUG_FFT
    _output_count++;
    // outputs at approx 1hz
    if (_output_count % 400 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FFT(us): t1:%lu,t2:%lu,t3:%lu,t4:%lu",
                      _hanning_timer._timer_avg, _rfft_f32_timer._timer_avg,
                      _arm_cmplx_mag_f32_timer._timer_avg, _step_calc_frequencies._timer_avg);
    }
#endif

    return fft->_peak_data[CENTER]._bin;
}

static const float PI_N = M_PI / 32.0f;
static const float CANDAN_FACTOR = tanf(PI_N) / PI_N;

// Interpolate center frequency using http://users.metu.edu.tr/ccandan//pub_dir/FineDopplerEst_IEEE_SPL_June2011.pdf
// This is slightly less accurate than Quinn, but much cheaper to calculate.
// Ported for parity; unused here just as in AP_HAL_ChibiOS/DSP.cpp - see DSP.h.
float DSP::calculate_candans_estimator(const FFTWindowStateARM *fft, uint16_t k_max) const
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

#endif  // HAL_WITH_DSP

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
