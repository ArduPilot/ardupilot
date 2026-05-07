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
 */

#include "hal.h"
#include "usbcfg.h"
#include "stm32_util.h"
#include "watchdog.h"
#include "board.h"
#include "hrt.h"

#if defined(RP2350)

/*
 * RP2350 reset-cause breadcrumb register used for live SWD diagnosis.
 * SCRATCH[0] and SCRATCH[1] are used by fastboot/bootloader handoff,
 * SCRATCH[6] is used by watchdog-reason detection, so use SCRATCH[7].
 */

static volatile uint8_t etype = 0;
static volatile uint8_t e_number = 0;

/*
 * Override ChibiOS weak _unhandled_exception for RP2350.
 * Persist useful breadcrumbs, then force a clean system reset so we recover
 * instead of getting wedged in an infinite fault loop.
 */
void __attribute__((noreturn)) _unhandled_exception(void)
{
    WATCHDOG->SCRATCH[0] = SCB->VTOR;
    WATCHDOG->SCRATCH[2] = e_number = (__get_xPSR() & 0x1FFU) - 16U;
    WATCHDOG->SCRATCH[3] = etype = SCB->ICSR;
    WATCHDOG->SCRATCH[4] = NVIC->ISPR[0];
    WATCHDOG->SCRATCH[5] = NVIC->ISPR[1];
    __asm volatile ("str lr, %0" : "=m"(WATCHDOG->SCRATCH[6]) :: "memory");
    WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX] = RP2350_RESET_DIAG_UNHANDLED_EXCEPTION;
    __DSB();
    __ISB();

    NVIC_SystemReset();
    while (1) {}
}

#if RP_I2C_USE_I2C1 == FALSE
/* Disable and clear I2C1 IRQ when that peripheral is not used. */
OSAL_IRQ_HANDLER(VectorD4) {
    OSAL_IRQ_PROLOGUE();
    nvicDisableVector(37U);
    nvicClearPending(37U);
    OSAL_IRQ_EPILOGUE();
}
#endif

/* Disable all interrupts and alarms for the selected RP2350 timer block. */
static void turn_off_timer(int timerid)
{
    switch (timerid) {
    case 0:
        TIMER0->INTE = 0U;
        TIMER0->INTR = 0xFU;
        TIMER0->ARMED = 0xFU;
        break;
    case 1:
        TIMER1->INTE = 0U;
        TIMER1->INTR = 0xFU;
        TIMER1->ARMED = 0xFU;
        break;
    default:
        break;
    }
}

/*
 * During RP2350 core1 launch, ROM can transiently assert IRQs that we do not
 * use on these targets. Provide explicit no-op handlers so they are disabled
 * and cleared instead of dropping into the weak unhandled-exception path.
 */
/* Handle TIMER1 IRQ0 by silencing timer1 and masking the vector. */
OSAL_IRQ_HANDLER(Vector50) {     
    OSAL_IRQ_PROLOGUE();     turn_off_timer(1);     __DSB();     nvicClearPending(4U);     nvicDisableVector(4U);     OSAL_IRQ_EPILOGUE(); }
/* Handle TIMER1 IRQ1 by silencing timer1 and masking the vector. */
OSAL_IRQ_HANDLER(Vector54) {
         OSAL_IRQ_PROLOGUE();     turn_off_timer(1);     __DSB();     nvicClearPending(5U);     nvicDisableVector(5U);     OSAL_IRQ_EPILOGUE(); }
/* Handle TIMER1 IRQ2 by silencing timer1 and masking the vector. */
OSAL_IRQ_HANDLER(Vector58) {
         OSAL_IRQ_PROLOGUE();     turn_off_timer(1);     __DSB();     nvicClearPending(6U);     nvicDisableVector(6U);     OSAL_IRQ_EPILOGUE(); }
/* Handle TIMER1 IRQ3 by silencing timer1 and masking the vector. */
OSAL_IRQ_HANDLER(Vector5C) {
         OSAL_IRQ_PROLOGUE();     turn_off_timer(1);     __DSB();     nvicClearPending(7U);     nvicDisableVector(7U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear PWM IRQ wrap channel 1. */
OSAL_IRQ_HANDLER(Vector64) {
         OSAL_IRQ_PROLOGUE();     nvicDisableVector(9U);     nvicClearPending(9U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear DMA IRQ channel 2. */
OSAL_IRQ_HANDLER(Vector70) {
         OSAL_IRQ_PROLOGUE();     nvicDisableVector(12U);     nvicClearPending(12U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear DMA IRQ channel 3. */
OSAL_IRQ_HANDLER(Vector74) {
         OSAL_IRQ_PROLOGUE();     nvicDisableVector(13U);     nvicClearPending(13U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear PIO2 IRQ0. */
OSAL_IRQ_HANDLER(Vector8C) {
         OSAL_IRQ_PROLOGUE();     nvicDisableVector(19U);     nvicClearPending(19U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear PIO2 IRQ1. */
OSAL_IRQ_HANDLER(Vector90) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(20U);     nvicClearPending(20U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear IO IRQ BANK0 non-secure line. */
OSAL_IRQ_HANDLER(Vector98) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(22U);     nvicClearPending(22U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear QSPI IO IRQ. */
OSAL_IRQ_HANDLER(Vector9C) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(23U);     nvicClearPending(23U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear QSPI IO IRQ non-secure line. */
OSAL_IRQ_HANDLER(VectorA0) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(24U);     nvicClearPending(24U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SIO FIFO IRQ. */
OSAL_IRQ_HANDLER(VectorA4) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(25U);     nvicClearPending(25U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SIO bell IRQ. */
OSAL_IRQ_HANDLER(VectorA8) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(26U);     nvicClearPending(26U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SIO FIFO non-secure IRQ. */
OSAL_IRQ_HANDLER(VectorAC) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(27U);     nvicClearPending(27U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SIO bell non-secure IRQ. */
OSAL_IRQ_HANDLER(VectorB0) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(28U);     nvicClearPending(28U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SIO mtime compare IRQ. */
OSAL_IRQ_HANDLER(VectorB4) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(29U);     nvicClearPending(29U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear clocks IRQ. */
OSAL_IRQ_HANDLER(VectorB8) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(30U);     nvicClearPending(30U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SPI0 IRQ. */
OSAL_IRQ_HANDLER(VectorBC) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(31U);     nvicClearPending(31U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SPI1 IRQ. */
OSAL_IRQ_HANDLER(VectorC0) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(32U);     nvicClearPending(32U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear ADC FIFO IRQ. */
OSAL_IRQ_HANDLER(VectorCC) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(35U);     nvicClearPending(35U);     OSAL_IRQ_EPILOGUE(); }
#if RP_I2C_USE_I2C0 == FALSE
/* Disable and clear I2C0 IRQ when that peripheral is not used. */
OSAL_IRQ_HANDLER(VectorD0) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(36U);     nvicClearPending(36U);     OSAL_IRQ_EPILOGUE(); }
#endif
/* Disable and clear OTP controller IRQ. */
OSAL_IRQ_HANDLER(VectorD8) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(38U);     nvicClearPending(38U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear TRNG IRQ. */
OSAL_IRQ_HANDLER(VectorDC) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(39U);     nvicClearPending(39U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear processor 0 CTI IRQ. */
OSAL_IRQ_HANDLER(VectorE0) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(40U);     nvicClearPending(40U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear processor 1 CTI IRQ. */
OSAL_IRQ_HANDLER(VectorE4) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(41U);     nvicClearPending(41U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear SYS PLL IRQ. */
OSAL_IRQ_HANDLER(VectorE8) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(42U);     nvicClearPending(42U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear USB PLL IRQ. */
OSAL_IRQ_HANDLER(VectorEC) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(43U);     nvicClearPending(43U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear power manager power IRQ. */
OSAL_IRQ_HANDLER(VectorF0) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(44U);     nvicClearPending(44U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear power manager timer IRQ. */
OSAL_IRQ_HANDLER(VectorF4) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(45U);     nvicClearPending(45U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear spare IRQ 0. */
OSAL_IRQ_HANDLER(VectorF8) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(46U);     nvicClearPending(46U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear spare IRQ 1. */
OSAL_IRQ_HANDLER(VectorFC) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(47U);     nvicClearPending(47U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear spare IRQ 2. */
OSAL_IRQ_HANDLER(Vector100) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(48U);     nvicClearPending(48U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear spare IRQ 3. */
OSAL_IRQ_HANDLER(Vector104) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(49U);     nvicClearPending(49U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear spare IRQ 4. */
OSAL_IRQ_HANDLER(Vector108) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(50U);     nvicClearPending(50U);     OSAL_IRQ_EPILOGUE(); }
/* Disable and clear spare IRQ 5. */
OSAL_IRQ_HANDLER(Vector10C) {
    OSAL_IRQ_PROLOGUE();     nvicDisableVector(51U);     nvicClearPending(51U);     OSAL_IRQ_EPILOGUE(); }

#define RP2350_RAMFUNC __attribute__((noinline, section(".ramtext")))

/*
 * After SWD flashing + SYSRESETREQ, RP2350 can keep stale XIP cache lines.
 * Invalidate from RAM before installing copied vectors so IRQ dispatch uses
 * freshly programmed flash contents.
 */
static RP2350_RAMFUNC void rp2350_invalidate_xip_cache(void)
{
    volatile uint8_t *maint = (volatile uint8_t *)0x18000000U;
    volatile uint32_t *xip = (volatile uint32_t *)0x400C8000U;

    for (uint32_t offset = 0U; offset < (16U * 1024U); offset += 8U) {
        maint[offset] = 0U;
    }

    __DSB();
    __ISB();
    xip[0] = (1U << 0) | (1U << 1);
}

/* Configure RP2350 pin mux and default output states from HAL defines. */
void pico2_gpio_init(void)
{
#if defined(HAL_PWM_GPIO_LINES)
    ioline_t lines[] = {HAL_PWM_GPIO_LINES};
    for (unsigned i = 0; i < sizeof(lines)/sizeof(lines[0]); i++) {
        palSetLineMode(lines[i], PAL_MODE_ALTERNATE_PWM);
    }
#endif

#if defined(HAL_PWM_ALARM_GPIO_LINE)
    palSetLineMode(HAL_PWM_ALARM_GPIO_LINE, PAL_MODE_ALTERNATE_PWM);
#endif

#if defined(HAL_GPIO_PINS)
    typedef struct {
        uint8_t pin_num;
        bool enabled;
        uint8_t pwm_num;
        ioline_t pal_line;
    } rp_gpio_entry_t;
    const rp_gpio_entry_t gpio_lines[] = HAL_GPIO_PINS;
    for (unsigned i = 0; i < sizeof(gpio_lines) / sizeof(gpio_lines[0]); i++) {
        if (!gpio_lines[i].enabled || gpio_lines[i].pwm_num != 0U) {
            continue;
        }
#if defined(HAL_PWM_ALARM) && defined(HAL_BUZZER_PIN)
        if (gpio_lines[i].pin_num == HAL_BUZZER_PIN) {
            continue;
        }
#endif
        palSetLineMode(gpio_lines[i].pal_line, PAL_MODE_OUTPUT_PUSHPULL);
    }
#endif

#if defined(HAL_GPIO_PIN_SPI0_SCK)
    palSetLineMode(HAL_GPIO_PIN_SPI0_SCK, PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI0_RX, PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI0_TX, PAL_MODE_ALTERNATE_SPI);
#endif
#if defined(HAL_GPIO_PIN_SPI1_SCK)
    palSetLineMode(HAL_GPIO_PIN_SPI1_SCK, PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI1_MISO, PAL_MODE_ALTERNATE_SPI);
    palSetLineMode(HAL_GPIO_PIN_SPI1_MOSI, PAL_MODE_ALTERNATE_SPI);
#endif

#if defined(HAL_GPIO_PIN_I2C0_SCL)
    palSetLineMode(HAL_GPIO_PIN_I2C0_SCL, PAL_MODE_ALTERNATE_I2C);
    palSetLineMode(HAL_GPIO_PIN_I2C0_SDA, PAL_MODE_ALTERNATE_I2C);
#endif
#if defined(HAL_GPIO_PIN_I2C1_SCL)
    palSetLineMode(HAL_GPIO_PIN_I2C1_SCL, PAL_MODE_ALTERNATE_I2C);
    palSetLineMode(HAL_GPIO_PIN_I2C1_SDA, PAL_MODE_ALTERNATE_I2C);
#endif

#if defined(HAL_GPIO_PIN_MAG_CS)
    palSetLine(HAL_GPIO_PIN_MAG_CS);
    palSetLineMode(HAL_GPIO_PIN_MAG_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_MPU_CS)
    palSetLine(HAL_GPIO_PIN_MPU_CS);
    palSetLineMode(HAL_GPIO_PIN_MPU_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_BARO_EXT_CS)
    palSetLine(HAL_GPIO_PIN_BARO_EXT_CS);
    palSetLineMode(HAL_GPIO_PIN_BARO_EXT_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_GYRO_EXT_CS)
    palSetLine(HAL_GPIO_PIN_GYRO_EXT_CS);
    palSetLineMode(HAL_GPIO_PIN_GYRO_EXT_CS, PAL_MODE_OUTPUT_PUSHPULL);
#endif

#if defined(HAL_GPIO_PIN_LED_BLUE)
    palSetLineMode(HAL_GPIO_PIN_LED_BLUE, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_LED_GREEN)
    palSetLineMode(HAL_GPIO_PIN_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
#endif

#if defined(HAL_GPIO_PIN_BUZZER)
    palClearLine(HAL_GPIO_PIN_BUZZER);
    palSetLineMode(HAL_GPIO_PIN_BUZZER, PAL_MODE_OUTPUT_PUSHPULL);
#endif

#if defined(HAL_GPIO_PIN_BEC_5V_EN)
    palSetLine(HAL_GPIO_PIN_BEC_5V_EN);
    palSetLineMode(HAL_GPIO_PIN_BEC_5V_EN, PAL_MODE_OUTPUT_PUSHPULL);
#endif
#if defined(HAL_GPIO_PIN_BEC_9V_EN)
    palClearLine(HAL_GPIO_PIN_BEC_9V_EN);
    palSetLineMode(HAL_GPIO_PIN_BEC_9V_EN, PAL_MODE_OUTPUT_PUSHPULL);
#endif
}

static uint32_t rp2350_vectors[16U + 56U] __attribute__((aligned(512)));

/* Perform RP2350 startup setup before generic HAL initialisation. */
void rp2350_board_early_init(void)
{
    extern uint32_t __vectors_base__[];

    SCB->VTOR = (uint32_t)__vectors_base__;
    __DSB();
    __ISB();

#if RP_NO_INIT == FALSE
    rp_peripheral_reset(~(RESETS_ALLREG_IO_QSPI | RESETS_ALLREG_PADS_QSPI |
                          RESETS_ALLREG_PLL_SYS | RESETS_ALLREG_PLL_USB));
    rp_clock_init();
#endif

    pico2_gpio_init();
}

/* Quiesce peripherals and prepare a safe vector-table handover before halInit(). */
void rp2350_board_pre_hal_init(void)
{
    // Ensure USB IRQ is fully quiesced before HAL re-initialises USBCTRL.
    nvicDisableVector(RP_USBCTRL_IRQ_NUMBER);
    rp_peripheral_reset(RESETS_ALLREG_USBCTRL);

    NVIC->ICPR[0] = 0xFFFFFFFFU;
    NVIC->ICPR[1] = 0xFFFFFFFFU;

    TIMER1->INTE = 0U;
    TIMER1->INTR = 0xFU;
    TIMER1->ARMED = 0xFU;
    __DSB();

    rp2350_invalidate_xip_cache();

    extern uint32_t __vectors_base__[];
    const uint32_t *flash_vectors = __vectors_base__;
    // Install vectors in RAM so late IRQs do not depend on stale XIP state.
    for (uint32_t i = 0; i < (16U + 56U); i++) {
        rp2350_vectors[i] = flash_vectors[i];
    }

    PADS_BANK0->GPIO[13] = 0x5AU;
    PADS_BANK0->GPIO[11] = 0x5AU;

#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE
    SIO->FIFO_ST = SIO_FIFO_ST_ROE | SIO_FIFO_ST_WOF;
    while ((SIO->FIFO_ST & SIO_FIFO_ST_VLD) != 0U) {
        (void)SIO->FIFO_RD;
    }
#endif
}

/* Finalise RP2350 board setup immediately after halInit(). */
void rp2350_board_post_hal_init(void)
{
    pico2_gpio_init();
    // Move VTOR to RAM vector table after HAL has finished core init.
    SCB->VTOR = (uint32_t)rp2350_vectors;
    __DSB();
    __ISB();
}

/* Run RP2350 board-level late init hooks used by boardInit(). */
void rp2350_board_init(void)
{
#if defined(RP2350B_QFN80) && (HAL_USE_ADC == TRUE)
    adcRPGpioInit(40U);
    adcRPGpioInit(41U);
    adcRPGpioInit(42U);
#endif
}

#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE
volatile uint32_t c1_startup_result = 0U;

/* Core1 test callback used by startup verification handshake. */
static void c1_startup_ping(void)
{
    c1_startup_result = 0xDEADC1C1U;
}

volatile uint32_t c1_timeout_count;
volatile uint32_t c1_ekf_c1_count;
volatile uint32_t c1_ekf_c0_count;
volatile uint32_t c1_pid_c1_count;
volatile uint32_t c1_pid_c0_count;

static MUTEX_DECL(c1_dispatch_mtx);

/* Serialize synchronous core1 dispatch attempts with a shared mutex. */
void c1_run_sync_locked(void (*fn)(void))
{
    chMtxLock(&c1_dispatch_mtx);
    bool on_c1 = c1_run_sync(fn);
    chMtxUnlock(&c1_dispatch_mtx);
    if (on_c1) { c1_ekf_c1_count++; } else { c1_ekf_c0_count++; }
}

/* Validate core1 boot and schedule a ping task when core1 is ready. */
void c1_startup_verify(void)
{
    extern volatile uint32_t c1_boot_stage;
    // Wait for core1 startup handshake from c1_main.c.
    for (volatile uint32_t i = 0U; i < 1000000UL && c1_boot_stage != 0x4DU; i++) {}
    if (c1_boot_stage == 0x4DU) {
        c1_run_sync(c1_startup_ping);
    } else {
        c1_startup_result = 0xBADC1BADU;
    }
}

volatile bool c1_att_pending;
volatile bool c1_cov_pending;
static volatile bool c1_dispatch_blocked;

volatile uint32_t c1_att_fn_sidechan __attribute__((weak)) = 0U;
volatile uint8_t c1_att_sidechan_done __attribute__((weak)) = 0U;
volatile uint32_t c1_cov_fn_sidechan __attribute__((weak)) = 0U;
volatile uint8_t c1_cov_sidechan_done __attribute__((weak)) = 0U;

/* Try to run a callback on core1; fallback to core0 if dispatch is contended. */
bool c1_try_run_sync(void (*fn)(void))
{
    // Fast path: run on core0 when the shared dispatch mutex is busy.
    if (!chMtxTryLock(&c1_dispatch_mtx)) {
        fn();
        __DMB();
        c1_pid_c0_count++;
        return false;
    }
    bool on_c1 = c1_run_sync(fn);
    chMtxUnlock(&c1_dispatch_mtx);
    if (on_c1) { c1_pid_c1_count++; } else { c1_pid_c0_count++; }
    return on_c1;
}

/* Queue an asynchronous attitude task for core1 side-channel execution. */
bool c1_att_dispatch_async(void (*fn)(void))
{
    if (c1_dispatch_blocked) {
        return false;
    }
    if (c1_att_fn_sidechan != 0U || c1_att_sidechan_done != 0U) {
        return false;
    }
    __DMB();
    c1_att_fn_sidechan = (uint32_t)fn;
    __SEV();
    c1_att_pending = true;
    return true;
}

/* Wait for completion of a pending asynchronous attitude task. */
void c1_att_barrier(void)
{
    if (!c1_att_pending) {
        return;
    }
    const uint32_t t0 = hrt_micros32();
    while (c1_att_sidechan_done == 0U) {
        // Keep bounded latency if core1 is stalled or unavailable.
        if (hrt_micros32() - t0 > 3000U) {
            c1_att_fn_sidechan = 0U;
            c1_timeout_count++;
            c1_att_pending = false;
            return;
        }
        chThdSleep(TIME_US2I(10));
    }
    __DMB();
    c1_att_sidechan_done = 0U;
    c1_att_pending = false;
}

/* Queue an asynchronous covariance task for core1 side-channel execution. */
bool c1_cov_dispatch_async(void (*fn)(void))
{
    if (c1_dispatch_blocked) {
        return false;
    }
    if (c1_cov_fn_sidechan != 0U || c1_cov_sidechan_done != 0U) {
        return false;
    }
    __DMB();
    c1_cov_fn_sidechan = (uint32_t)fn;
    __SEV();
    c1_cov_pending = true;
    return true;
}

/* Wait for completion of a pending asynchronous covariance task. */
void c1_cov_barrier(void)
{
    if (!c1_cov_pending) {
        return;
    }
    const uint32_t t0 = hrt_micros32();
    while (c1_cov_sidechan_done == 0U) {
        // Keep bounded latency if core1 is stalled or unavailable.
        if (hrt_micros32() - t0 > 3000U) {
            c1_cov_fn_sidechan = 0U;
            c1_timeout_count++;
            c1_cov_pending = false;
            return;
        }
        chThdSleep(TIME_US2I(10));
    }
    __DMB();
    c1_cov_sidechan_done = 0U;
    c1_cov_pending = false;
}

/* Run a callback synchronously on core1 using the inter-core FIFO handshake. */
bool c1_run_sync(void (*fn)(void))
{
    if (c1_dispatch_blocked) {
        fn();
        __DMB();
        return false;
    }

    __DMB();

    const uint32_t t0 = hrt_micros32();
    // Wait for FIFO write slot from core1 with bounded timeout.
    while (!(SIO->FIFO_ST & SIO_FIFO_ST_RDY)) {
        if (hrt_micros32() - t0 > 1000U) {
            c1_timeout_count++;
            fn();
            __DMB();
            return false;
        }
        chThdSleep(TIME_US2I(10));
    }
    SIO->FIFO_WR = (uint32_t)fn;
    __SEV();

    const uint32_t t1 = hrt_micros32();
    // Wait for core1 completion ack from FIFO.
    while (!(SIO->FIFO_ST & SIO_FIFO_ST_VLD)) {
        if (hrt_micros32() - t1 > 1000U) {
            c1_timeout_count++;
            fn();
            __DMB();
            return false;
        }
        chThdSleep(TIME_US2I(10));
    }
    (void)SIO->FIFO_RD;
    __DMB();
    return true;
}

/* Enter a flash-critical section by blocking and draining core1 dispatch work. */
void c1_flash_begin(void)
{
    // Stop new core1 jobs and drain pending side-channel jobs before flash IO.
    c1_dispatch_blocked = true;
    __DMB();
    c1_att_barrier();
    c1_cov_barrier();
    chMtxLock(&c1_dispatch_mtx);
}

/* Exit the flash-critical section and re-enable normal core1 dispatch. */
void c1_flash_end(void)
{
    chMtxUnlock(&c1_dispatch_mtx);
    __DMB();
    // Re-enable normal core1 dispatch after flash operation completes.
    c1_dispatch_blocked = false;
}
#endif // RP_CORE1_START

#endif // RP2350
