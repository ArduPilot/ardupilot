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
 * Code by @davidbuzz and Claude
 */
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include <AP_Math/div1000.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
#include <zephyr/kernel.h>
#if defined(CONFIG_SOC_SERIES_STM32H7X)
#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#endif
#else
#include <time.h>
#endif

namespace AP_HAL {

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SOC_SERIES_STM32H7X)
/* High-resolution time base: TIM5 free-running 32-bit at exactly 1 MHz, as
 * AP_HAL_ChibiOS does. CNT *is* microseconds, so micros() is a single volatile
 * load. micros64() extends it across the 71.6-minute wrap under a spinlock. */
static struct k_spinlock hrt_lock;
static uint32_t hrt_high32;
static uint32_t hrt_last_cnt;

static void hrt_init()
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

    /* APB1 timer kernel clock: PCLK1, doubled unless the APB1
       prescaler is /1 (RM0433 clock tree) */
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    uint32_t timclk = clocks.PCLK1_Frequency;
    if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1) {
        timclk *= 2U;
    }

    TIM5->CR1 = 0;
    /* The ChibiOS bootloader uses TIM5 as its tick timer and leaves
       DIER.CC1IE and NVIC IRQ50 enabled. Zephyr installs no TIM5
       handler, so the first stale compare match after we start the
       counter lands in z_irq_spurious -> fatal (~minutes after boot).
       Sanitize interrupt state completely before counting. */
    TIM5->DIER = 0;
    TIM5->SR   = 0;
    NVIC_DisableIRQ(TIM5_IRQn);
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    TIM5->PSC = (timclk / 1000000U) - 1U;   /* count at 1 MHz */
    TIM5->ARR = 0xFFFFFFFFU;               /* free-run, 32-bit wrap */
    TIM5->EGR = TIM_EGR_UG;                 /* latch PSC */
    TIM5->CNT = 0;
    TIM5->CR1 = TIM_CR1_CEN;
}
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(__XTENSA__)
/* Xtensa time base: CCOUNT is a 32-bit cycle counter at the CPU clock, so at
 * 240 MHz it wraps every 17.9 s - scaling the raw value made micros() itself
 * restart, silencing every non-fast task. Extend to 64 bits under a spinlock. */
static struct k_spinlock ccount_lock;
static uint32_t ccount_high32;
static uint32_t ccount_last;

static uint64_t xtensa_cycles64()
{
    k_spinlock_key_t key = k_spin_lock(&ccount_lock);
    uint32_t now;
    __asm__ volatile("rsr.ccount %0" : "=a"(now));
    if (now < ccount_last) {
        ccount_high32++;
    }
    ccount_last = now;
    const uint64_t ret = ((uint64_t)ccount_high32 << 32) | now;
    k_spin_unlock(&ccount_lock, key);
    return ret;
}
#endif

#if CONFIG_HAL_BOARD != HAL_BOARD_ZEPHYR
static struct {
    uint64_t start_time_ns;
} state;

static uint64_t ts_to_nsec(const struct timespec &ts)
{
    return uint64_t(ts.tv_sec) * 1000000000ULL + uint64_t(ts.tv_nsec);
}
#endif

void init()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_ZEPHYR
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    state.start_time_ns = ts_to_nsec(ts);
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
    hrt_init();
#endif
    /* Other Zephyr targets: k_cycle_get_64() counts from boot — no init needed. */
}

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
/* Set by panic(); the scheduler monitor thread checks this and refrains
 * from rebooting so the panic message stays visible on the console. */
volatile bool _hal_zephyr_panicked;
#endif

void WEAK panic(const char *errormsg, ...)
{
    va_list ap;
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
    _hal_zephyr_panicked = true;
    /* Re-print forever (like AP_HAL_ChibiOS) so a console attached after
     * the fact — e.g. USB CDC opened later — still sees the message. */
    uint32_t delay_ms = 10000;
    while (true) {
        va_start(ap, errormsg);
        vfprintf(stderr, errormsg, ap);
        va_end(ap);
        fputc('\n', stderr);
        fflush(stderr);
        k_msleep(delay_ms);
        delay_ms = 500;
    }
#else
    va_start(ap, errormsg);
    vfprintf(stderr, errormsg, ap);
    va_end(ap);
    fputc('\n', stderr);
    exit(1);
#endif
}

uint32_t micros()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
#if defined(__XTENSA__)
    /* Full-range uint32 µs from the wrap-extended cycle counter - see
     * xtensa_cycles64() above for why the raw CCOUNT read this used to be
     * is wrong (it restarted micros() every 17.9 s). Fixed-point
     * reciprocal: 1/240 ≈ 17895697 × 2^-32 (error < 1 PPM at 240 MHz). */
    return uint32_t((xtensa_cycles64() * 17895697ULL) >> 32);
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
    /* TIM5 counts microseconds directly — single volatile load */
    return TIM5->CNT;
#else
    /* Other arches: derive from the 64-bit cycle counter and truncate.
     * NOT k_cyc_to_us_floor32(k_cycle_get_32()): that counts CPU cycles, so on a
     * 1 GHz part micros() restarts every 4.29 s and `micros() - start` breaks. */
    return uint32_t(micros64());
#endif
#else
    return uint32_t(micros64());
#endif
}

uint32_t millis()
{
    return uint32_t(millis64());
}

uint64_t micros64()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
#if defined(__XTENSA__)
    /* Wrap-extended CCOUNT scaled to µs - true µs resolution. (NOT
     * k_cycle_get_64(): it uses the ESP32 16 MHz system timer while
     * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=240000000, so k_cyc_to_us_floor64()
     * misscales; and NOT k_uptime_get()*1000, this function's previous body,
     * whose ms granularity quantised every µs-level interval measurement.) */
    return (xtensa_cycles64() * 17895697ULL) >> 32;
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
    /* TIM5 µs counter, wrap-extended to 64 bits (wraps every 71.6 min;
       we are called far more often than that) */
    k_spinlock_key_t key = k_spin_lock(&hrt_lock);
    const uint32_t now = TIM5->CNT;
    if (now < hrt_last_cnt) {
        hrt_high32++;
    }
    hrt_last_cnt = now;
    const uint64_t ret = ((uint64_t)hrt_high32 << 32) | now;
    k_spin_unlock(&hrt_lock, key);
    return ret;
#else
    /* other arches: k_cycle_get_64() uses the SysTick/DWT cycle counter
     * at the CPU clock rate (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC). */
    return k_cyc_to_us_floor64(k_cycle_get_64());
#endif
#else
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_div1000(ts_to_nsec(ts) - state.start_time_ns);
#endif
}

uint64_t millis64()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && !defined(CONFIG_SOC_SERIES_STM32H7X)
    return k_uptime_get();
#else
    /* H7 + non-Zephyr: derive from the same time base as micros64() */
    return uint64_div1000(micros64());
#endif
}

} // namespace AP_HAL

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(__XTENSA__)
/* picolibc's sqrtf is generic C soft-float, measured 1.10 us/call on ESP32-S3.
 * libgcc in this SDK already has __ieee754_sqrtf using the LX7 hardware
 * sqrt-assist, but gcc lowers sqrtf() to libm and picolibc wins the link. */
extern "C" {
extern float __ieee754_sqrtf(float);
float sqrtf(float x)
{
    return __ieee754_sqrtf(x);
}
}
#endif
