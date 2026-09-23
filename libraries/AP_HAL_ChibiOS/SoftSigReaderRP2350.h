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
 * RC pulse capture for RP2350 using ChibiOS PAL GPIO edge callbacks.
 * Both rising and falling edges are detected; successive edge-to-edge
 * intervals are paired into (widths0, widths1) pulse descriptors that
 * match the interface used by the EICU-based SoftSigReaderInt.
 */
#pragma once

#include "AP_HAL_ChibiOS.h"

#if defined(HAL_RCIN_IS_GPIO)

#include <hal.h>

#ifndef SOFTSIG_MAX_SIGNAL_TRANSITIONS
#define SOFTSIG_MAX_SIGNAL_TRANSITIONS 128
#endif

class ChibiOS::SoftSigReaderRP2350 {
public:
    SoftSigReaderRP2350() {}
    CLASS_NO_COPY(SoftSigReaderRP2350);

    void init(ioline_t line);
    void enable(void);

/*
 * Returns true and fills widths0/widths1 with the next pulse pair (in microseconds) from the ring buffer, or returns false when empty.
 * widths0 = time from first edge to second edge widths1 = time from second edge to third edge
 */
    bool read(uint32_t &widths0, uint32_t &widths1);

    void disable(void);

private:
    static void _irq_handler(void *ctx);
    void _edge(void);

    struct PACKED pulse_t {
        uint32_t w0;
        uint32_t w1;
    };

// Single-producer (ISR) / single-consumer (RCIN thread) fixed ring buffer.
// Keeping this structure simple avoids heavy ByteBuffer operations in ISR context and reduces MSP/IRQ stack pressure on RP2350.
    pulse_t _ring[SOFTSIG_MAX_SIGNAL_TRANSITIONS];
    volatile uint16_t _head = 0;
    volatile uint16_t _tail = 0;

    bool _push_isr(const pulse_t &p);
    bool _pop_thread(pulse_t &p);

    ioline_t _line;
    uint32_t _last_tick;
    uint32_t _pending_w0;
    bool _got_first;
    bool _pending_valid;
    bool _irq_enabled;
};

#endif // HAL_RCIN_IS_GPIO
