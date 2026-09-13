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
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
#include <soc.h>
#include <fsl_clock.h>
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

    /* Keep TIM5 clocked while the core sleeps.
     *
     * THIS IS WHY TIM5 IS THE TIME SOURCE: with this bit set it keeps
     * counting forward when everything else on the chip has stopped for WFI.
     * A time base is only useful if it still advances while the core is
     * asleep - one that stops when the CPU stops cannot measure the thing it
     * is being asked to measure, which is how much time passed while nothing
     * was running. Every clock this HAL tried before failed exactly there.
     *
     * RCC_APB1LENR (above) gates the peripheral in Run mode; RCC_APB1LLPENR is
     * a SEPARATE gate that applies in Sleep mode, and it is NOT implied by the
     * Run-mode one. TIM5LPEN comes out of reset set on the H7, but a
     * bootloader or an earlier init can clear it, and this HAL's time base is
     * TIM5->CNT: if the timer stops during WFI then micros() and millis() -
     * which is derived from micros64() on this SoC - simply lose the idle
     * time, and every interval measured across a sleep comes out short by the
     * idle fraction. Measured 2026-09-08 with the counter gated:
     * time_boot_ms advanced at 0.106x wall.
     *
     * That is one of the two reasons CONFIG_AP_NO_WFI_IDLE exists. Setting
     * this bit removes it for this board: SysTick is driven by FCLK, which is
     * free-running by architecture and already survives Sleep, so with TIM5
     * ungated both the kernel's clock and AP's clock keep counting through
     * WFI. It does NOT by itself make WFI safe to re-enable - see the
     * CONFIG_AP_NO_WFI_IDLE help - and the veto stays on. It removes a reason,
     * not the decision. */
    LL_APB1_GRP1_EnableClockSleep(LL_APB1_GRP1_PERIPH_TIM5);

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

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SOC_SERIES_IMXRT11XX)
/* High-resolution time base: GPT2 free-running 32-bit at exactly 1 MHz, the
 * i.MX counterpart of the STM32H7's TIM5 above. CNT *is* microseconds.
 *
 * THIS IS WHY GPT2 IS THE TIME SOURCE, AND IT IS THE WHOLE REASON: it keeps
 * counting forward when everything else on this chip has stopped for WFI.
 * On the RT1176 that is a much sharper problem than on the H7 - WFI gates the
 * ENTIRE CM7 clock domain, so SysTick VAL and DWT CYCCNT both freeze together,
 * measured 2026-08-08 with the documented ungating knobs set
 * (GPC_CPU_MODE_CTRL_0 in RUN mode, CCM GPR_PRIVATE1 bit 0 = 1,
 * SCR.SLEEPDEEP = 0). That search was not exhaustive - LPCG0_DOMAIN's WAIT
 * dependency level and SysTick's own M7_SYSTICK_CLK_ROOT (STCLK, a clock root
 * distinct from the core's) were never tried - so read it as "no in-core
 * counter survived sleep in the configuration we ran", not as a proof that
 * none can. It does not change what this HAL does: like ChibiOS-ArduPilot, it
 * declines to sleep at all, and k_cycle_get_64() - what micros64() used to
 * read on this board - is one of the things that stops when it does.
 * A time base that halts with the
 * CPU cannot measure the one quantity it is being asked for, which is how
 * much time passed while nothing was running.
 *
 * GPT sits outside the CM7 domain and has explicit low-power run bits, so it
 * is the part of this SoC that can answer that question:
 *
 *   WAITEN  keep counting in Wait mode  <- this is the mode WFI enters
 *   DOZEEN  keep counting in Doze mode
 *   STOPEN  keep counting in Stop mode
 *
 * and it is clocked from the 24 MHz crystal rather than from anything derived
 * from the core clock, so the source itself also survives.
 * Zephyr's own GPT kernel-timer driver (drivers/timer/mcux_gpt_timer.c) sets
 * the same three bits for the same reason; it is not used here because it
 * runs GPT from the 32 kHz low-frequency reference, whose 30.5 us resolution
 * is far too coarse for a flight controller's loop and IMU interval timing.
 *
 * This does NOT make WFI safe to re-enable on this board on its own: Zephyr's
 * kernel clock is still SysTick and still stops. It fixes AP's clock only.
 * See the CONFIG_AP_NO_WFI_IDLE help text.
 *
 * ---- getting the crystal to the timer, measured on silicon 2026-09-13 ----
 *
 * GPT has an input the SDK enum calls the crystal oscillator
 * (kGPT_ClockSource_Osc = CLKSRC 101b, gated by CR[EN_24M], divided by
 * PR[PRESCALER24M]). That enum is shared across the whole i.MX family and it
 * is WRONG for this part: measured with a debugger against the host clock,
 * that input delivers 16.24 MHz, not 24. Two independent PRESCALER24M
 * settings agreed to 0.02 %, and 16.24 MHz is the on-chip 16 MHz RC at the
 * +1.5 % an untrimmed RC gives you.
 *
 * A /3 then /8 chain on a 16.24 MHz input is 677 kHz, so a CNT read as
 * microseconds ran at 0.677x real time - every interval AP measured came out
 * 32 % short and every rate it reported was 1.478x the truth, the loop rate
 * included. Nothing in the GPT or oscillator registers looks wrong: they read
 * back exactly as written, and OSC_24M_CTRL says the crystal is enabled,
 * stable, ungated and unbypassed. The frequency is the only place it shows,
 * which is why this needs a measurement and not a code review.
 *
 * So drive the timer from the CCM clock root instead (CLKSRC 001b), and point
 * that root at the crystal. The root's reset mux is OscRc48MDiv2 - an RC
 * again, measured 24.085 MHz - so the mux is the load-bearing half:
 *
 *   root mux 0 (OscRc48MDiv2, RC)   /24 -> 1 003 545 Hz   +0.35 %
 *   root mux 1 (Osc24MOut, crystal) /24 -> 1 000 014 Hz   +0.0014 %
 *
 * ONE CAVEAT, stated because it is the thing this comment spends 30 lines
 * defending: the WAIT-mode argument above was made for the EN_24M input,
 * which came straight off the analog oscillator. Running from a CCM root
 * means WAIT-mode counting now also depends on that root staying ungated in
 * low-power modes, which has NOT been measured - the board sets
 * CONFIG_AP_NO_WFI_IDLE=y and never sleeps, so there was nothing to measure
 * it against. Measure it before enabling WFI on this board. */
#define AP_HRT_GPT ((GPT_Type *)DT_REG_ADDR(DT_NODELABEL(gpt2)))

static struct k_spinlock hrt_lock;
static uint32_t hrt_high32;
static uint32_t hrt_last_cnt;

static void hrt_init()
{
    GPT_Type *gpt = AP_HRT_GPT;

    /* Ungate the peripheral. Nothing else does: this board builds with
       CONFIG_COUNTER off, so no Zephyr driver binds gpt2 and no clock_control
       call reaches its LPCG. */
    CLOCK_EnableClock(kCLOCK_Gpt2);

    /* Software reset clears CR/PR/SR/IR and the counter. It self-clears. */
    gpt->CR = GPT_CR_SWR_MASK;
    while (gpt->CR & GPT_CR_SWR_MASK) {
    }
    gpt->IR = 0;                 /* no interrupts - this is a counter only */
    gpt->SR = gpt->SR;           /* w1c: clear any latched status */

    /* Point GPT2's CCM clock root at the 24 MHz crystal, undivided. The reset
       value of this root is mux 0 = OscRc48MDiv2, an RC oscillator, so this
       is the line that makes the time base crystal-accurate instead of 0.35 %
       fast - see the measurements at the top of this block. */
    clock_root_config_t root_cfg = {};
    root_cfg.mux = kCLOCK_GPT2_ClockRoot_MuxOsc24MOut;
    root_cfg.div = 1;
    CLOCK_SetRootClock(kCLOCK_Root_Gpt2, &root_cfg);

    /* 24 MHz root -> /24 = 1 MHz. PRESCALER24M is not in this path at all -
       it divides only the CR[EN_24M] input, which on this SoC is the 16 MHz
       RC - so the main 12-bit prescaler carries the whole divide. Both fields
       are "divide by n+1". */
    gpt->PR = GPT_PR_PRESCALER24M(1U - 1U) | GPT_PR_PRESCALER(24U - 1U);

    gpt->CR = GPT_CR_CLKSRC(1U)      /* 001b = the CCM root configured above */
            | GPT_CR_FRR_MASK        /* free-run: wrap at 2^32, no compare reset */
            | GPT_CR_ENMOD_MASK      /* start counting from 0 */
            | GPT_CR_WAITEN_MASK     /* the bit this whole comment is about */
            | GPT_CR_DOZEEN_MASK
            | GPT_CR_STOPEN_MASK
            | GPT_CR_DBGEN_MASK;     /* keep time under a halted debugger */

    gpt->CR |= GPT_CR_EN_MASK;
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
#elif defined(CONFIG_SOC_SERIES_IMXRT11XX)
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
#elif defined(CONFIG_SOC_SERIES_IMXRT11XX)
    /* GPT2 counts microseconds directly, and keeps counting through WFI —
     * see hrt_init() for why nothing in the CM7 core can be used here. */
    return AP_HRT_GPT->CNT;
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
#elif defined(CONFIG_SOC_SERIES_IMXRT11XX)
    /* GPT2 µs counter, wrap-extended to 64 bits (wraps every 71.6 min).
     * NOT k_cycle_get_64(): that is the CM7's own SysTick/DWT counter, which
     * this SoC stops dead in WFI along with the rest of the core domain. */
    k_spinlock_key_t key = k_spin_lock(&hrt_lock);
    const uint32_t now = AP_HRT_GPT->CNT;
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
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && \
    !defined(CONFIG_SOC_SERIES_STM32H7X) && \
    !defined(CONFIG_SOC_SERIES_IMXRT11XX)
    return k_uptime_get();
#else
    /* H7, RT11xx + non-Zephyr: derive from the same time base as micros64().
     * k_uptime_get() is the kernel clock, and on both of those SoCs the kernel
     * clock is exactly the thing that stops in WFI - so millis() and micros()
     * would disagree about how long a sleep took, which is worse than either
     * being wrong on its own. One time base per board. */
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
