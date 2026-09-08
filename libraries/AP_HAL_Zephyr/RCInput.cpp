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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "RCInput.h"

#ifdef __ZEPHYR__
#if defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
#include <zephyr/sys/sys_io.h>
#endif
#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)
#include <zephyr/drivers/pwm.h>
#endif
#endif

#include <AP_RCProtocol/AP_RCProtocol.h>

extern const AP_HAL::HAL& hal;

using namespace Zephyr;

#ifdef __ZEPHYR__
/* PPM-SUM input pin, e.g. CubeOrange AUX OUT 6 (PD14) repurposed as a
   capture input. Declared per-board with a zephyr,user node:
       / { zephyr,user { rcin-gpios = <&gpiod 14 GPIO_ACTIVE_HIGH>; }; } */
#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), rcin_gpios)
#define HAVE_RCIN_PULSE_GPIO 1
static const struct gpio_dt_spec rcin_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), rcin_gpios);
#if defined(CONFIG_SOC_SERIES_STM32H7X)
#include <stm32_ll_exti.h>
#endif

#if defined(CONFIG_AP_RCIN_GPIO2_DIRECT_ISR)
/* Zero-latency direct ISRs for gpio2 (see the Kconfig help): the capture edge
 * must not queue behind the kernel's own masking. */
static Zephyr::RCInput *rcin_isr_instance;
#define AP_GPIO2_ISR_STATUS (*(volatile uint32_t *)0x40130018)

ISR_DIRECT_DECLARE(ap_rcin_gpio2_isr_low)
{
    const uint32_t st = AP_GPIO2_ISR_STATUS;
    AP_GPIO2_ISR_STATUS = st;   /* W1C */
    if ((st & BIT(rcin_gpio.pin)) && rcin_isr_instance != nullptr) {
        rcin_isr_instance->_edge();
    }
    return 0;   /* no scheduling decisions from a ZLI ISR */
}

ISR_DIRECT_DECLARE(ap_rcin_gpio2_isr_high)
{
    const uint32_t st = AP_GPIO2_ISR_STATUS;
    AP_GPIO2_ISR_STATUS = st;   /* W1C - nothing uses pins 16-31 IRQs */
    return 0;
}
#endif  /* CONFIG_AP_RCIN_GPIO2_DIRECT_ISR */

#if defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
/* Direct IOMUXC_SW_MUX_CTL_PAD register write - the ONLY way to flip this pad
 * between its UART and capture roles at runtime. */
static constexpr uint32_t RCIN_MUX_CTL_ADDR = 0x400E80B0U;
static constexpr uint32_t RCIN_MUX_ALT_UART = 3U;
#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)
static constexpr uint32_t RCIN_MUX_ALT_PPM = 1U;   /* ALT1 XBAR1_IN12 -> QTMR */
#else
static constexpr uint32_t RCIN_MUX_ALT_PPM = 5U;   /* ALT5 GPIO_MUX2_IO08 */
#endif

void RCInput::_set_pad_mux(bool gpio_mode)
{
    if (_pad_is_gpio == gpio_mode) {
        return;
    }
    uint32_t reg = sys_read32(RCIN_MUX_CTL_ADDR);
    reg &= ~0xFU;   // clear MUX_MODE[3:0] only; SION (bit4) and reserved bits untouched
    reg |= gpio_mode ? RCIN_MUX_ALT_PPM : RCIN_MUX_ALT_UART;
    sys_write32(reg, RCIN_MUX_CTL_ADDR);
    _pad_is_gpio = gpio_mode;
}
#endif  /* RCIN_PULSE_GPIO_SHARES_UART_PAD */
#endif
#endif

void RCInput::init()
{
    if (_init) {
        return;
    }

#if AP_RCPROTOCOL_ENABLED
    AP_RCProtocol &rcprot = AP::RC();
    rcprot.init();

    /* Deliberately no set_rc_protocols() mask and no pre-configured protocol: the
     * autodetect scan tries both polarities and every protocol. */
    AP_HAL::UARTDriver *rcin_uart = hal.serial(7);
    if (rcin_uart != nullptr) {
        rcprot.add_uart(rcin_uart);
    }
#endif

#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)
    /* HARDWARE timer capture via the portable pwm_configure_capture() API. */
    {
        static const struct pwm_dt_spec cap = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));
        if (device_is_ready(cap.dev) &&
            pwm_get_cycles_per_sec(cap.dev, cap.channel, &_cap_cyc_per_sec) == 0 &&
            _cap_cyc_per_sec > 0 &&
            pwm_configure_capture(cap.dev, cap.channel,
                                  PWM_CAPTURE_TYPE_PERIOD | PWM_CAPTURE_MODE_CONTINUOUS,
                                  _pwm_capture_cb, this) == 0) {
            pwm_enable_capture(cap.dev, cap.channel);
        }
    }
#if defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
    /* Deterministic start: the qtmr1 driver's pinctrl put the pad in ALT1
       (capture) at boot, after lpuart6's own pinctrl had set ALT3 - the
       last applier wins and that ordering is init-priority luck. Force the
       arbiter's declared starting state (UART) onto the silicon so state
       variable and register agree from the first probe cycle. */
    {
        uint32_t reg = sys_read32(RCIN_MUX_CTL_ADDR);
        sys_write32((reg & ~0xFU) | RCIN_MUX_ALT_UART, RCIN_MUX_CTL_ADDR);
        _pad_is_gpio = false;
    }
#endif
#elif defined(HAVE_RCIN_PULSE_GPIO)
    if (gpio_is_ready_dt(&rcin_gpio)) {
#if !defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
        /* Dedicated pin (e.g. CubeOrange AUX6): safe to let the GPIO
           driver's own configure() also own pinctrl/pull for it.
           pull-up: PPM idles high, and a floating/disconnected lead must
           sit quietly instead of harvesting noise edges (same lesson
           ChibiOS learned on floating GPIO interrupt inputs). */
        gpio_pin_configure_dt(&rcin_gpio, GPIO_INPUT | GPIO_PULL_UP);
#else
        /* MUST NOT call gpio_pin_configure_dt() here: it reapplies pinctrl and undoes the
         * runtime pad arbitration below. */
#endif
#if defined(CONFIG_CPU_CORTEX_M)
        /* _edge() timestamps from DWT->CYCCNT (see its comment). Make sure
           the counter is actually running rather than assuming some other
           subsystem enabled it: DEMCR.TRCENA then DWT_CTRL.CYCCNTENA. Both
           are set-only here - never cleared - so this cannot disturb other
           DWT users (PCSR profiling, the chain profiler). */
        *(volatile uint32_t *)0xE000EDFC |= (1U << 24);   /* DEMCR.TRCENA */
        *(volatile uint32_t *)0xE0001000 |= 1U;           /* DWT_CTRL.CYCCNTENA */
#endif
#if defined(CONFIG_AP_RCIN_GPIO2_DIRECT_ISR)
        /* Zero-latency path: WE own gpio2's vectors (the driver's claim is
           compiled out - see the gpio_mcux_igpio.c local patch). Priority
           argument is ignored for ZLI (forced to 0 = highest). The driver
           call below still arms IMR/ICR/EDGE_SEL - register writes only,
           independent of vector ownership. */
        rcin_isr_instance = this;
        IRQ_DIRECT_CONNECT(DT_IRQ_BY_IDX(DT_NODELABEL(gpio2), 0, irq), 0,
                           ap_rcin_gpio2_isr_low, IRQ_ZERO_LATENCY);
        IRQ_DIRECT_CONNECT(DT_IRQ_BY_IDX(DT_NODELABEL(gpio2), 1, irq), 0,
                           ap_rcin_gpio2_isr_high, IRQ_ZERO_LATENCY);
        irq_enable(DT_IRQ_BY_IDX(DT_NODELABEL(gpio2), 0, irq));
        irq_enable(DT_IRQ_BY_IDX(DT_NODELABEL(gpio2), 1, irq));
#else
        gpio_init_callback(&_gpio_cb, _edge_isr, BIT(rcin_gpio.pin));
        gpio_add_callback(rcin_gpio.port, &_gpio_cb);
#endif
        gpio_pin_interrupt_configure_dt(&rcin_gpio, GPIO_INT_EDGE_BOTH);
    }
#endif

    _init = true;
}

#ifdef __ZEPHYR__
void RCInput::_edge_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    (void)dev;
    (void)pins;
    RCInput *self = CONTAINER_OF(cb, RCInput, _gpio_cb);
    self->_edge();
}

void RCInput::_edge()
{
    /* Timestamp from the raw DWT cycle counter, NOT AP_HAL::micros(): micros() takes
     * a spinlock, which must not happen in this ISR. */
    volatile uint32_t *const dwt_cyccnt = (volatile uint32_t *)0xE0001004;
    const uint32_t now_cyc = *dwt_cyccnt;
    const uint32_t width = (now_cyc - _last_edge_cyc) / CYCLES_PER_US;
    _last_edge_cyc = now_cyc;

    const uint16_t next = (_pulse_head + 1) % PULSE_BUF_SIZE;
    if (next != _pulse_tail) {
        _pulse_widths[_pulse_head] = width;
        _pulse_head = next;
    }

    /* storm guard — see STORM_* in RCInput.h. A noisy/floating pin
       exceeding any real PPM rate gets the line masked here (in ISR);
       the rcin thread re-arms after a cooldown, re-masking if still
       noisy, so ISR load stays bounded and threads keep running. */
    if (++_burst_count >= STORM_BURST) {
        /* all in the cycle domain - wrap-safe unsigned subtraction; the
           re-arm deadline is set later by _update() in thread context
           (this ISR is ZLI and must not call micros()) */
        if (now_cyc - _burst_start_cyc < STORM_WINDOW_US * CYCLES_PER_US) {
            _mask_pulse_irq();
            _remask_deadline_us = 0;    // _update() computes the real deadline
            _pulse_irq_masked = true;
        }
        _burst_count = 0;
        _burst_start_cyc = now_cyc;
    }
}

void RCInput::_mask_pulse_irq()
{
#if defined(HAVE_RCIN_PULSE_GPIO) && defined(CONFIG_SOC_SERIES_STM32H7X)
    /* single register write — ISR-safe; matches how Zephyr's stm32 EXTI
       enables the line (CPUIMR1). */
    LL_EXTI_DisableIT_0_31(BIT(rcin_gpio.pin));
#elif defined(HAVE_RCIN_PULSE_GPIO)
    /* Portable path via Zephyr's own GPIO API, for boards with no capture timer. */
    gpio_pin_interrupt_configure_dt(&rcin_gpio, GPIO_INT_DISABLE);
#endif
}

void RCInput::_unmask_pulse_irq()
{
#if defined(HAVE_RCIN_PULSE_GPIO) && defined(CONFIG_SOC_SERIES_STM32H7X)
    LL_EXTI_EnableIT_0_31(BIT(rcin_gpio.pin));
#elif defined(HAVE_RCIN_PULSE_GPIO)
    gpio_pin_interrupt_configure_dt(&rcin_gpio, GPIO_INT_EDGE_BOTH);
#endif
}

#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)
/* Hardware-capture callback: runs from the timer driver's ISR with a period
 * already latched by the silicon. */
void RCInput::_pwm_capture_cb(const struct device *dev, uint32_t channel,
                              uint32_t period_cycles, uint32_t pulse_cycles,
                              int status, void *user_data)
{
    (void)dev; (void)channel; (void)pulse_cycles;
    RCInput *self = (RCInput *)user_data;

    uint32_t period_us = 0;
    if (status == 0) {
        period_us = (uint32_t)(((uint64_t)period_cycles * 1000000ULL) /
                               self->_cap_cyc_per_sec);
    }

    const uint16_t next = (self->_pulse_head + 1) % PULSE_BUF_SIZE;
    if (next != self->_pulse_tail) {
        self->_pulse_widths[self->_pulse_head] = period_us;
        self->_pulse_head = next;
    }
}
#endif  /* CONFIG_AP_RCIN_PWM_CAPTURE */
#endif

void RCInput::pulse_input_enable(bool enable)
{
    _pulse_input_enabled = enable;
#if defined(HAVE_RCIN_PULSE_GPIO) && defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
    if (!enable) {
        /* AP_RCProtocol has confirmed a UART-based lock (SBUS/CRSF/etc) -
           UART wins. Latch so _update() stops switching the pad away from
           it, and make sure the pad is ACTUALLY on LPUART6 right now: if
           this fires while a GPIO probe window happened to be open, the
           UART cannot receive anything until the pad is put back. */
        _pad_latched = true;
        _set_pad_mux(false);
    }
#endif
}

void RCInput::_update()
{
    if (!_init || !_pulse_input_enabled) {
        return;
    }

#if AP_RCPROTOCOL_ENABLED
    AP_RCProtocol &rcprot = AP::RC();

#if defined(HAVE_RCIN_PULSE_GPIO)
#if defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
    /* Runtime pad arbitration - see the _set_pad_mux() comment. */
    if (!_pad_latched) {
        const uint32_t now_arb = AP_HAL::micros();
        if (_pad_is_gpio) {
            if ((int32_t)(now_arb - _gpio_probe_until_us) >= 0) {
                // probe window over, no lock yet - give the UART scan a turn
                _set_pad_mux(false);
                _next_gpio_probe_us = now_arb + GPIO_PROBE_INTERVAL_US;
                _pulse_head = _pulse_tail = 0;
                _have_first_width = false;
            }
        } else if ((int32_t)(now_arb - _next_gpio_probe_us) >= 0) {
            // steal the pad for one PPM-frame-length probe window
            _set_pad_mux(true);
            _gpio_probe_until_us = now_arb + GPIO_PROBE_WINDOW_US;
            _pulse_head = _pulse_tail = 0;
            _have_first_width = false;
#if !defined(CONFIG_AP_RCIN_PWM_CAPTURE)
            // discard the switch gap as one bogus width (cycle domain)
            _last_edge_cyc = *(volatile uint32_t *)0xE0001004;
            if (_pulse_irq_masked) {    // guarantee a clean armed state each probe
                _burst_count = 0;
                _burst_start_cyc = _last_edge_cyc;
                _pulse_irq_masked = false;
                _unmask_pulse_irq();
            }
#endif
        }
    }
#endif
#if !defined(CONFIG_AP_RCIN_PWM_CAPTURE)
    /* re-arm the pulse IRQ after a storm cooldown; if the pin is still
       noisy the ISR will mask it again within STORM_BURST edges.
       The ZLI ISR only sets the masked flag with deadline 0; the actual
       µs deadline is computed HERE, in thread context where micros() is
       legal (the ~1 ms lazy-set delay just extends the cooldown, safe). */
    if (_pulse_irq_masked) {
        const uint32_t now_us = AP_HAL::micros();
        if (_remask_deadline_us == 0) {
            _remask_deadline_us = now_us + STORM_COOLDOWN_US;
            if (_remask_deadline_us == 0) {   // avoid the sentinel on exact wrap
                _remask_deadline_us = 1;
            }
        } else if ((int32_t)(now_us - _remask_deadline_us) >= 0) {
            _burst_count = 0;
            _burst_start_cyc = *(volatile uint32_t *)0xE0001004;
            _remask_deadline_us = 0;
            _pulse_irq_masked = false;
            _unmask_pulse_irq();
        }
    }
#endif

#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)
    /* drain hardware-capture PERIODS: each ring entry is already one full
       rising-to-rising channel period, silicon-exact. PPMSum consumes the
       SUM of the two arguments, so pass (period-1, 1); a 0 entry is the
       capture-error marker and (0,0) is PPMSum's own explicit reset. */
    while (_pulse_tail != _pulse_head) {
        const uint32_t w = _pulse_widths[_pulse_tail];
        _pulse_tail = (_pulse_tail + 1) % PULSE_BUF_SIZE;
        if (w > 1) {
            rcprot.process_pulse(w - 1, 1);
        } else {
            rcprot.process_pulse(0, 0);   /* explicit frame reset */
        }
    }
#else
    /* drain the edge-width queue in consecutive pairs — PPMSUM only
       uses the pair sum, so pairing phase doesn't matter */
    while (_pulse_tail != _pulse_head) {
        const uint32_t w = _pulse_widths[_pulse_tail];
        _pulse_tail = (_pulse_tail + 1) % PULSE_BUF_SIZE;
        if (!_have_first_width) {
            _first_width = w;
            _have_first_width = true;
        } else {
            rcprot.process_pulse(_first_width, w);
            _have_first_width = false;
        }
    }
#endif
#endif

    rcprot.update();

    if (rcprot.new_input()) {
#if defined(HAVE_RCIN_PULSE_GPIO) && defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
        /* PPM won: this can only be a pulse-decoded frame, because the UART
           physically cannot be receiving anything right now - MUX_MODE is
           on GPIO's ALT (_pad_is_gpio true precisely means "currently mid
           probe window"), so LPUART6 has no connection to the pad to have
           produced this. Latch permanently; no more switching. */
        if (!_pad_latched && _pad_is_gpio) {
            _pad_latched = true;
        }
#endif
        if (!_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            return;
        }
        _rcin_timestamp_last_signal = AP_HAL::micros();
        _num_channels = rcprot.num_channels();
        if (_num_channels > RC_INPUT_MAX_CHANNELS) {
            _num_channels = RC_INPUT_MAX_CHANNELS;
        }
        rcprot.read(_rc_values, _num_channels);
        _rssi = rcprot.get_RSSI();
        _rx_link_quality = rcprot.get_rx_link_quality();
        _mutex.give();
    }
#endif
}

/* The accessors below are PURE READERS, ChibiOS parity. */
bool RCInput::new_input()
{
    if (!_init) {
        return false;
    }

    bool valid = false;
    if (!_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    valid = (_rcin_timestamp_last_signal != _last_read);
    _last_read = _rcin_timestamp_last_signal;
    _mutex.give();

    return valid;
}

uint8_t RCInput::num_channels()
{
    if (!_init) {
        return 0;
    }

    return _num_channels;
}

uint16_t RCInput::read(uint8_t ch)
{
    if (!_init) {
        return 0;
    }

    if (ch >= _num_channels || ch >= RC_INPUT_MAX_CHANNELS) {
        return 0;
    }

    uint16_t v = 0;
    if (!_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return 0;
    }
    v = _rc_values[ch];
    _mutex.give();
    return v;
}

uint8_t RCInput::read(uint16_t *periods, uint8_t len)
{
    if (!_init || periods == nullptr) {
        return 0;
    }

    if (len > _num_channels) {
        len = _num_channels;
    }
    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }

    if (!_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return 0;
    }
    memcpy(periods, _rc_values, len * sizeof(periods[0]));
    _mutex.give();

    return len;
}

const char *RCInput::protocol() const
{
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().detected_protocol_name();
#else
    return "none";
#endif
}


#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
