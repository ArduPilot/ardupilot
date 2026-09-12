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
 */

#include "SoftSigReaderRP2350.h"

#if defined(HAL_RCIN_IS_GPIO)

using namespace ChibiOS;

/*
  ISR-context callback invoked by ChibiOS PAL on every GPIO edge.
  CH_CFG_ST_FREQUENCY == 1000000 on RP2350, so chVTGetSystemTimeX() ticks == µs.
*/
void SoftSigReaderRP2350::_irq_handler(void *ctx)
{
    SoftSigReaderRP2350 *reader = static_cast<SoftSigReaderRP2350 *>(ctx);
    reader->_edge();
}

void SoftSigReaderRP2350::_edge(void)
{
    const uint32_t now = (uint32_t)chVTGetSystemTimeX();

    if (!_got_first) {
        _last_tick = now;
        _got_first = true;
        return;
    }

    const uint32_t delta = now - _last_tick;
    _last_tick = now;

    if (!_pending_valid) {
        _pending_w0 = delta;
        _pending_valid = true;
    } else {
        pulse_t p;
        p.w0 = _pending_w0;
        p.w1 = delta;
        (void)_push_isr(p);
        _pending_valid = false;
    }
}

bool SoftSigReaderRP2350::_push_isr(const pulse_t &p)
{
    const uint16_t head = _head;
    const uint16_t next = (uint16_t)((head + 1U) % SOFTSIG_MAX_SIGNAL_TRANSITIONS);

    // Drop oldest new sample when ring is full; this matches the existing
    // behavior of non-blocking RC pulse capture under overload.
    if (next == _tail) {
        return false;
    }

    _ring[head] = p;
    _head = next;
    return true;
}

bool SoftSigReaderRP2350::_pop_thread(pulse_t &p)
{
    const uint16_t tail = _tail;
    if (tail == _head) {
        return false;
    }

    p = _ring[tail];
    _tail = (uint16_t)((tail + 1U) % SOFTSIG_MAX_SIGNAL_TRANSITIONS);
    return true;
}

void SoftSigReaderRP2350::init(ioline_t line)
{
    _line = line;
    _head = 0;
    _tail = 0;
    _got_first = false;
    _pending_valid = false;
    _last_tick = 0;
    _pending_w0 = 0;
    _irq_enabled = false;

/*
 * Explicitly set the pin to SIO input mode with pull-down before enabling the PAL edge-callback.
 * RP2350: the reset default for PADS_BANK0 is FUNCSEL=NULL (31) with IE=0 and ISO=1 (analog-isolated), which prevents the digital input path from seeing any edges.
 */
    palSetLineMode(_line, PAL_MODE_INPUT_PULLDOWN);

    chSysLock();
    palSetLineCallbackI(_line, _irq_handler, this);
    chSysUnlock();
}

void SoftSigReaderRP2350::enable(void)
{
    if (_irq_enabled) {
        return;
    }

// Arm GPIO edge capture only after the scheduler/timer path is alive.
// Early RC receiver traffic can generate a dense IRQ stream during AP_Vehicle::setup(), which consumes the tiny startup MSP stack on RP2350.
    _got_first = false;
    _pending_valid = false;
    _last_tick = 0;
    _pending_w0 = 0;

    chSysLock();
    palEnableLineEventI(_line, PAL_EVENT_MODE_BOTH_EDGES);
    chSysUnlock();
    _irq_enabled = true;
}

bool SoftSigReaderRP2350::read(uint32_t &widths0, uint32_t &widths1)
{
    pulse_t p;
    if (!_pop_thread(p)) {
        return false;
    }
    widths0 = p.w0;
    widths1 = p.w1;
    return true;
}

void SoftSigReaderRP2350::disable(void)
{
    if (_irq_enabled) {
        palDisableLineEvent(_line);
        _irq_enabled = false;
    }
}

#endif // HAL_RCIN_IS_GPIO
