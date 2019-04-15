/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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
 * Modified for Ardupilot by Siddharth Bharat Purohit
 */

#include "AP_HAL_ChibiOS.h"

#if HAL_WITH_UAVCAN
#include "CANClock.h"
#include "CANThread.h"
#include "CANInternal.h"

#ifndef UAVCAN_STM32_TIMER_NUMBER
#define UAVCAN_STM32_TIMER_NUMBER 0
#endif

#if UAVCAN_STM32_TIMER_NUMBER

#include <cassert>
#include <cmath>

/*
 * Timer instance
 */
# if (CH_KERNEL_MAJOR == 2)
#  define TIMX                    UAVCAN_STM32_GLUE2(TIM, UAVCAN_STM32_TIMER_NUMBER)
#  define TIMX_IRQn               UAVCAN_STM32_GLUE3(TIM, UAVCAN_STM32_TIMER_NUMBER, _IRQn)
#  define TIMX_INPUT_CLOCK        STM32_TIMCLK1
# endif

# if (CH_KERNEL_MAJOR == 3 || CH_KERNEL_MAJOR == 4)
#  define TIMX                    UAVCAN_STM32_GLUE2(STM32_TIM, UAVCAN_STM32_TIMER_NUMBER)
#  define TIMX_IRQn               UAVCAN_STM32_GLUE3(STM32_TIM, UAVCAN_STM32_TIMER_NUMBER, _NUMBER)
#  define TIMX_IRQHandler         UAVCAN_STM32_GLUE3(STM32_TIM, UAVCAN_STM32_TIMER_NUMBER, _HANDLER)
#  define TIMX_INPUT_CLOCK        STM32_TIMCLK1
# else
#  define TIMX_IRQHandler         UAVCAN_STM32_GLUE3(TIM, UAVCAN_STM32_TIMER_NUMBER, _IRQHandler)
# endif

# if UAVCAN_STM32_TIMER_NUMBER >= 2 && UAVCAN_STM32_TIMER_NUMBER <= 7
#  define TIMX_RCC_ENR           RCC->APB1ENR
#  define TIMX_RCC_RSTR          RCC->APB1RSTR
#  define TIMX_RCC_ENR_MASK      UAVCAN_STM32_GLUE3(RCC_APB1ENR_TIM,  UAVCAN_STM32_TIMER_NUMBER, EN)
#  define TIMX_RCC_RSTR_MASK     UAVCAN_STM32_GLUE3(RCC_APB1RSTR_TIM, UAVCAN_STM32_TIMER_NUMBER, RST)
# else
#  error "This UAVCAN_STM32_TIMER_NUMBER is not supported yet"
# endif

# if (TIMX_INPUT_CLOCK % 1000000) != 0
#  error "No way, timer clock must be divisible to 1e6. FIXME!"
# endif

extern "C" UAVCAN_STM32_IRQ_HANDLER(TIMX_IRQHandler);

namespace ChibiOS_CAN {
namespace clock {
namespace {

const uavcan::uint32_t USecPerOverflow = 65536;

Mutex mutex;

bool initialized = false;

bool utc_set = false;
bool utc_locked = false;
uavcan::uint32_t utc_jump_cnt = 0;
UtcSyncParams utc_sync_params;
float utc_prev_adj = 0;
float utc_rel_rate_ppm = 0;
float utc_rel_rate_error_integral = 0;
uavcan::int32_t utc_accumulated_correction_nsec = 0;
uavcan::int32_t utc_correction_nsec_per_overflow = 0;
uavcan::MonotonicTime prev_utc_adj_at;

uavcan::uint64_t time_mono = 0;
uavcan::uint64_t time_utc = 0;

}

void init()
{
    CriticalSectionLocker lock;
    if (initialized) {
        return;
    }
    initialized = true;


    // Power-on and reset
    TIMX_RCC_ENR |= TIMX_RCC_ENR_MASK;
    TIMX_RCC_RSTR |=  TIMX_RCC_RSTR_MASK;
    TIMX_RCC_RSTR &= ~TIMX_RCC_RSTR_MASK;

    // Enable IRQ
    nvicEnableVector(TIMX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);

# if (TIMX_INPUT_CLOCK % 1000000) != 0
#  error "No way, timer clock must be divisible to 1e6. FIXME!"
# endif

    // Start the timer
    TIMX->ARR  = 0xFFFF;
    TIMX->PSC  = (TIMX_INPUT_CLOCK / 1000000) - 1;  // 1 tick == 1 microsecond
    TIMX->CR1  = TIM_CR1_URS;
    TIMX->SR   = 0;
    TIMX->EGR  = TIM_EGR_UG;     // Reload immediately
    TIMX->DIER = TIM_DIER_UIE;
    TIMX->CR1  = TIM_CR1_CEN;    // Start

}

void setUtc(uavcan::UtcTime time)
{
    MutexLocker mlocker(mutex);
    UAVCAN_ASSERT(initialized);

    {
        CriticalSectionLocker locker;
        time_utc = time.toUSec();
    }

    utc_set = true;
    utc_locked = false;
    utc_jump_cnt++;
    utc_prev_adj = 0;
    utc_rel_rate_ppm = 0;
}

static uavcan::uint64_t sampleUtcFromCriticalSection()
{
    UAVCAN_ASSERT(initialized);
    UAVCAN_ASSERT(TIMX->DIER & TIM_DIER_UIE);

    volatile uavcan::uint64_t time = time_utc;
    volatile uavcan::uint32_t cnt = TIMX->CNT;

    if (TIMX->SR & TIM_SR_UIF) {
        cnt = TIMX->CNT;
        const uavcan::int32_t add = uavcan::int32_t(USecPerOverflow) +
                                    (utc_accumulated_correction_nsec + utc_correction_nsec_per_overflow) / 1000;
        time = uavcan::uint64_t(uavcan::int64_t(time) + add);
    }
    return time + cnt;
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
    return utc_set ? sampleUtcFromCriticalSection() : 0;
}

uavcan::MonotonicTime getMonotonic()
{
    uavcan::uint64_t usec = 0;
    // Scope Critical section
    {
        CriticalSectionLocker locker;

        volatile uavcan::uint64_t time = time_mono;


        volatile uavcan::uint32_t cnt = TIMX->CNT;
        if (TIMX->SR & TIM_SR_UIF) {
            cnt = TIMX->CNT;
            time += USecPerOverflow;
        }
        usec = time + cnt;

# ifndef NDEBUG
        static uavcan::uint64_t prev_usec = 0;      // Self-test
        UAVCAN_ASSERT(prev_usec <= usec);
        (void)prev_usec;
        prev_usec = usec;
# endif
    } // End Scope Critical section

    return uavcan::MonotonicTime::fromUSec(usec);
}

uavcan::UtcTime getUtc()
{
    if (utc_set) {
        uavcan::uint64_t usec = 0;
        {
            CriticalSectionLocker locker;
            usec = sampleUtcFromCriticalSection();
        }
        return uavcan::UtcTime::fromUSec(usec);
    }
    return uavcan::UtcTime();
}

static float lowpass(float xold, float xnew, float corner, float dt)
{
    const float tau = 1.F / corner;
    return (dt * xnew + tau * xold) / (dt + tau);
}

static void updateRatePID(uavcan::UtcDuration adjustment)
{
    const uavcan::MonotonicTime ts = getMonotonic();
    const float dt = float((ts - prev_utc_adj_at).toUSec()) / 1e6F;
    prev_utc_adj_at = ts;
    const float adj_usec = float(adjustment.toUSec());

    /*
     * Target relative rate in PPM
     * Positive to go faster
     */
    const float target_rel_rate_ppm = adj_usec * utc_sync_params.offset_p;

    /*
     * Current relative rate in PPM
     * Positive if the local clock is faster
     */
    const float new_rel_rate_ppm = (utc_prev_adj - adj_usec) / dt; // rate error in [usec/sec], which is PPM
    utc_prev_adj = adj_usec;
    utc_rel_rate_ppm = lowpass(utc_rel_rate_ppm, new_rel_rate_ppm, utc_sync_params.rate_error_corner_freq, dt);

    const float rel_rate_error = target_rel_rate_ppm - utc_rel_rate_ppm;

    if (dt > 10) {
        utc_rel_rate_error_integral = 0;
    }
    else {
        utc_rel_rate_error_integral += rel_rate_error * dt * utc_sync_params.rate_i;
        utc_rel_rate_error_integral =
            uavcan::max(utc_rel_rate_error_integral, -utc_sync_params.max_rate_correction_ppm);
        utc_rel_rate_error_integral =
            uavcan::min(utc_rel_rate_error_integral, utc_sync_params.max_rate_correction_ppm);
    }

    /*
     * Rate controller
     */
    float total_rate_correction_ppm = rel_rate_error + utc_rel_rate_error_integral;
    total_rate_correction_ppm = uavcan::max(total_rate_correction_ppm, -utc_sync_params.max_rate_correction_ppm);
    total_rate_correction_ppm = uavcan::min(total_rate_correction_ppm, utc_sync_params.max_rate_correction_ppm);

    utc_correction_nsec_per_overflow = uavcan::int32_t((USecPerOverflow * 1000) * (total_rate_correction_ppm / 1e6F));

    //    syslog("$ adj=%f   rel_rate=%f   rel_rate_eint=%f   tgt_rel_rate=%f   ppm=%f\n",
    //              adj_usec, utc_rel_rate_ppm, utc_rel_rate_error_integral, target_rel_rate_ppm,
    // total_rate_correction_ppm);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
    MutexLocker mlocker(mutex);
    UAVCAN_ASSERT(initialized);

    if (adjustment.getAbs() > utc_sync_params.min_jump || !utc_set) {
        const uavcan::int64_t adj_usec = adjustment.toUSec();

        {
            CriticalSectionLocker locker;
            if ((adj_usec < 0) && uavcan::uint64_t(-adj_usec) > time_utc) {
                time_utc = 1;
            }
            else {
                time_utc = uavcan::uint64_t(uavcan::int64_t(time_utc) + adj_usec);
            }
        }

        utc_set = true;
        utc_locked = false;
        utc_jump_cnt++;
        utc_prev_adj = 0;
        utc_rel_rate_ppm = 0;
    }
    else {
        updateRatePID(adjustment);

        if (!utc_locked) {
            utc_locked =
                (std::abs(utc_rel_rate_ppm) < utc_sync_params.lock_thres_rate_ppm) &&
                (std::abs(utc_prev_adj) < utc_sync_params.lock_thres_offset.toUSec());
        }
    }
}

float getUtcRateCorrectionPPM()
{
    MutexLocker mlocker(mutex);
    const float rate_correction_mult = float(utc_correction_nsec_per_overflow) / float(USecPerOverflow * 1000);
    return 1e6F * rate_correction_mult;
}

uavcan::uint32_t getUtcJumpCount()
{
    MutexLocker mlocker(mutex);
    return utc_jump_cnt;
}

bool isUtcLocked()
{
    MutexLocker mlocker(mutex);
    return utc_locked;
}

UtcSyncParams getUtcSyncParams()
{
    MutexLocker mlocker(mutex);
    return utc_sync_params;
}

void setUtcSyncParams(const UtcSyncParams& params)
{
    MutexLocker mlocker(mutex);
    // Add some sanity check
    utc_sync_params = params;
}

} // namespace clock

SystemClock& SystemClock::get_singleton()
{
    static union SystemClockStorage {
        uavcan::uint8_t buffer[sizeof(SystemClock)];
        long long _aligner_1;
        long double _aligner_2;
    } storage;

    SystemClock* const ptr = reinterpret_cast<SystemClock*>(storage.buffer);

    if (!clock::initialized) {
        MutexLocker mlocker(clock::mutex);
        clock::init();
        new (ptr)SystemClock();
    }
    return *ptr;
}

} // namespace uavcan_stm32


/**
 * Timer interrupt handler
 */

extern "C"
UAVCAN_STM32_IRQ_HANDLER(TIMX_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();

    TIMX->SR = 0;

    using namespace uavcan_stm32::clock;
    UAVCAN_ASSERT(initialized);

    time_mono += USecPerOverflow;

    if (utc_set) {
        time_utc += USecPerOverflow;
        utc_accumulated_correction_nsec += utc_correction_nsec_per_overflow;
        if (std::abs(utc_accumulated_correction_nsec) >= 1000) {
            time_utc = uavcan::uint64_t(uavcan::int64_t(time_utc) + utc_accumulated_correction_nsec / 1000);
            utc_accumulated_correction_nsec %= 1000;
        }

        // Correction decay - 1 nsec per 65536 usec
        if (utc_correction_nsec_per_overflow > 0) {
            utc_correction_nsec_per_overflow--;
        }
        else if (utc_correction_nsec_per_overflow < 0) {
            utc_correction_nsec_per_overflow++;
        }
        else {
            ; // Zero
        }
    }

    UAVCAN_STM32_IRQ_EPILOGUE();
}

#endif
#endif //HAL_WITH_UAVCAN