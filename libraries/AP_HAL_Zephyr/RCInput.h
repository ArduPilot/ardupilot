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
#pragma once

#include <AP_HAL/RCInput.h>

#include "Semaphores.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#endif

#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

namespace Zephyr {

class RCInput : public AP_HAL::RCInput {
public:
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t *periods, uint8_t len) override;
    void pulse_input_enable(bool enable) override;

    int16_t get_rssi(void) override
    {
        return _rssi;
    }

    int16_t get_rx_link_quality(void) override
    {
        return _rx_link_quality;
    }

    const char *protocol() const override;

    void _update();

    uint16_t _rc_values[RC_INPUT_MAX_CHANNELS] = {};
    uint8_t _num_channels = 0;
    uint32_t _rcin_timestamp_last_signal = 0;
    uint32_t _last_read = 0;
    bool _init = false;
    bool _pulse_input_enabled = true;

    int16_t _rssi = -1;
    int16_t _rx_link_quality = -1;

    Semaphore _mutex;

#ifdef __ZEPHYR__
    /* PPM-SUM pulse capture on a GPIO (zephyr,user rcin-gpios): the edge
       ISR timestamps with micros() and queues edge-to-edge widths; the
       rcin thread pairs them up for AP_RCProtocol::process_pulse(). */
    static void _edge_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    void _edge();
    void _mask_pulse_irq();
    void _unmask_pulse_irq();
    struct gpio_callback _gpio_cb;
    static constexpr uint16_t PULSE_BUF_SIZE = 128;  /* power of two */
    volatile uint32_t _pulse_widths[PULSE_BUF_SIZE];
    volatile uint16_t _pulse_head = 0, _pulse_tail = 0;
    /* Edge timestamps are raw DWT->CYCCNT cycles, not micros() - the edge
       ISR runs zero-latency and may not touch kernel state (see _edge()).
       CYCLES_PER_US comes from the kernel's own cycle-rate config: 1000 on
       RT1176 (1 GHz), 480 on CubeOrangeZephyr (480 MHz H7). */
    static constexpr uint32_t CYCLES_PER_US =
        CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000U;
    uint32_t _last_edge_cyc = 0;
    uint32_t _burst_start_cyc = 0;
    bool _have_first_width = false;
    uint32_t _first_width = 0;

    /* EXTI storm guard: a floating/noisy/disconnected RC lead can raise
       the pin interrupt faster than any real PPM signal and starve every
       thread. Above STORM_BURST edges within STORM_WINDOW_US (a rate far
       higher than real PPM-SUM's <10k edges/s) the ISR masks the line;
       the rcin thread re-arms it after STORM_COOLDOWN_US. */
    static constexpr uint32_t STORM_BURST = 512;
    static constexpr uint32_t STORM_WINDOW_US = 20000;   /* 512 edges in <20ms => >25k/s */
    static constexpr uint32_t STORM_COOLDOWN_US = 50000;  // 50ms
    /* Storm-guard state lives in the CYCLE domain (set from the ZLI ISR,
       which cannot call micros()); the re-arm DEADLINE is µs, but it is
       computed lazily in _update() (thread context) the first time the
       masked flag is observed, never in the ISR. */
    uint32_t _burst_count = 0;
    volatile bool _pulse_irq_masked = false;
    uint32_t _remask_deadline_us = 0;

#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)
    /* HARDWARE timer capture path (portable pwm_configure_capture() API). */
    static void _pwm_capture_cb(const struct device *dev, uint32_t channel,
                                uint32_t period_cycles, uint32_t pulse_cycles,
                                int status, void *user_data);
    uint64_t _cap_cyc_per_sec = 0;
#endif

#if defined(RCIN_PULSE_GPIO_SHARES_UART_PAD)
    /* Runtime MUX_MODE arbitration: a pad cannot passively share the UART and capture
     * roles, so ownership is switched explicitly. */
    void _set_pad_mux(bool gpio_mode);
    bool _pad_latched = false;          // true once EITHER side has a confirmed lock
    bool _pad_is_gpio = false;          // current MUX_MODE selection (false = LPUART6)
    uint32_t _gpio_probe_until_us = 0;  // 0 = not currently in a probe window
    uint32_t _next_gpio_probe_us = 0;
    /* WIDENED 2026-08-13, bench-measured: a 25 ms (one-frame) window proved too tight
     * once the capture path added latency. */
    static constexpr uint32_t GPIO_PROBE_WINDOW_US = 300000;
    static constexpr uint32_t GPIO_PROBE_INTERVAL_US = 500000; // still leaves UART >60% of the time while unlocked
#endif
#endif
};

}  // namespace Zephyr
