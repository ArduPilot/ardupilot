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
/*
  ESC serial passthrough on the RP2350 PIO DShot outputs.

  AP_BLHeli configures and flashes ESCs over their signal wire, as a one-wire,
  half-duplex UART: 19200 baud, 8N1, idle high. The STM32 path in RCOutput.cpp
  transmits from a timer with DMA. Here the motor channels leave their PIO
  state machines and the CPU drives the line, and replies are read by
  serial_bit_irq(), the edge-timed reader the STM32 path uses:

  - setup takes every motor channel off DShot and drives each pad high, the
    idle line. Pausing and resuming the state machines follows Ben La Monica's
    implementation.
  - each byte is timed against the microsecond clock with this core's
    interrupts masked for that byte. Betaflight's passthrough loop is the only
    thing running and masks nothing; here the main thread can be preempted,
    which would stretch a bit. No kernel lock is taken, so the other core is
    not held up.
  - the line is released at the start of the last stop bit, which the pad's
    pull-up then holds high, as Betaflight does, so the line is never driven
    while the ESC answers. serial_bit_irq() is attached before that byte and
    the edges the byte latches are cleared at release, so it sees only the
    reply.
  - end hands every channel back to its state machine.

  With thanks to Betaflight, whose 4-way interface and RP2350 port the
  transmit side follows.

  AP_BLHeli calls all of this from the main thread, on core0, and refuses
  passthrough while armed; so does serial_setup_output().
 */

#include <hal.h>
#include "RCOutput.h"

#if defined(RP2350) && (HAL_USE_PWM == TRUE) && HAL_SERIAL_ESC_COMM_ENABLED

#if !HAL_DSHOT_ENABLED
#error "RP2350 ESC passthrough drives the DShot outputs, so it needs HAL_DSHOT_ENABLED"
#endif

#include "RCOutput_pico.h"
#include "GPIO.h"
#include <AP_HAL/AP_HAL.h>

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

namespace
{

// start bit, eight data bits, stop bit
constexpr uint8_t FRAME_BITS = 10;

// the line driven and released; both keep the input on, so switching between
// them is not seen as an edge
constexpr iomode_t LINE_DRIVEN = PAL_RP_IOCTRL_FUNCSEL_SIO | PAL_RP_GPIO_OE | PAL_RP_PAD_IE |
                                 PAL_RP_PAD_SCHMITT | PAL_RP_PAD_DRIVE4;
constexpr iomode_t LINE_RELEASED = PAL_RP_IOCTRL_FUNCSEL_SIO | PAL_RP_PAD_IE |
                                   PAL_RP_PAD_SCHMITT | PAL_RP_PAD_PUE;

// the line in use, whether serial_bit_irq() is attached to it, and the bit time
ioline_t serial_line;
bool line_selected;
bool receiving;
uint16_t bit_time_us;

/*
  Mask this core's interrupts up to the kernel's level, so nothing preempts a
  byte being timed. The kernel lock is not taken: the other core is not held up.
 */
uint32_t mask_core_interrupts()
{
    const uint32_t basepri = __get_BASEPRI();
    __set_BASEPRI_MAX(CORTEX_BASEPRI_KERNEL);
    return basepri;
}

void unmask_core_interrupts(uint32_t basepri)
{
    __set_BASEPRI(basepri);
}

void drive_idle(ioline_t line)
{
    palSetLine(line);
    palSetLineMode(line, LINE_DRIVEN);
}

/*
  Forget the edges a line has latched. IO_BANK0's INTR registers hold four
  event bits a GPIO, eight GPIOs a register; the two edge bits are
  write-one-to-clear. Used on the line in use with interrupts masked, so the
  edge interrupt has not yet seen them.
 */
void clear_latched_edges(ioline_t line)
{
    const uint32_t gpio = (uint32_t)line;
    IO_BANK0->INTR[gpio >> 3] = 0x0CU << (4U * (gpio & 7U));
}

}  // namespace

/*
  Get serial_bit_irq() ready for a reply: fresh decoder state, an empty buffer,
  then the edge interrupt. The caller still drives the line high, so no edge
  reaches the decoder before the line is released.
 */
bool RCOutput::serial_rx_attach(ioline_t line, uint16_t bit_us)
{
    chVTObjectInit(&irq.serial_timeout);
    serial_buffer.clear();
    irq.line = line;
    irq.nbits = 0;
    irq.bitmask = 0;
    irq.bit_time_tick = bit_us;
    irq.last_bit = 0;
    return ((GPIO *)hal.gpio)->_attach_interrupt(line, serial_bit_irq, AP_HAL::GPIO::INTERRUPT_BOTH);
}

void RCOutput::serial_rx_detach(ioline_t line)
{
    chSysLock();
    palDisableLineEventI(line);
    chVTResetI(&irq.serial_timeout);
    chSysUnlock();
}

bool RCOutput::serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t motor_mask)
{
    chan -= chan_offset;
    motor_mask >>= chan_offset;
    if (hal.util->get_soft_armed() || baudrate == 0 || chan >= RCOutput_pico::MAX_CHANNELS ||
        !RCOutput_pico::is_initialised()) {
        return false;
    }
    const uint32_t bound = RCOutput_pico::channel_mask() | RCOutput_pico::paused_mask();
    if (!(bound & (1U << chan))) {
        return false;
    }

    // take every motor off DShot, so none is driven while another ESC is spoken to
    motor_mask |= (1U << chan);
    for (uint8_t i = 0; i < RCOutput_pico::MAX_CHANNELS; i++) {
        if ((motor_mask & (1U << i)) && RCOutput_pico::pause_channel(i)) {
            drive_idle(PAL_LINE(IOPORT1, RCOutput_pico::channel_gpio(i)));
        }
    }

    if (receiving) {
        serial_rx_detach(serial_line);
        receiving = false;
        drive_idle(serial_line);
    }
    serial_line = PAL_LINE(IOPORT1, RCOutput_pico::channel_gpio(chan));
    bit_time_us = 1000000UL / baudrate;
    line_selected = true;
    return true;
}

bool RCOutput::serial_write_bytes(const uint8_t *bytes, uint16_t len)
{
    if (!line_selected || hal.util->get_soft_armed()) {
        return false;
    }
    if (receiving) {
        serial_rx_detach(serial_line);
        receiving = false;
    }
    // driven high before the first bit, so taking the line is not a start bit
    drive_idle(serial_line);

    for (uint16_t n = 0; n < len; n++) {
        const bool last = (n + 1U == len);
        if (last) {
            // the ESC can answer as soon as the line is released
            if (!serial_rx_attach(serial_line, bit_time_us)) {
                return false;
            }
            receiving = true;
        }
        // start bit low, data least significant bit first, stop bit high
        const uint16_t frame = (uint16_t(bytes[n]) << 1) | (1U << (FRAME_BITS - 1U));
        const uint32_t basepri = mask_core_interrupts();
        const uint32_t t0 = AP_HAL::micros();
        for (uint8_t bit = 0; bit < FRAME_BITS; bit++) {
            if (frame & (1U << bit)) {
                palSetLine(serial_line);
            } else {
                palClearLine(serial_line);
            }
            if (last && bit == FRAME_BITS - 1U) {
                // release at the start of the stop bit, which the pull-up holds high
                clear_latched_edges(serial_line);
                palSetLineMode(serial_line, LINE_RELEASED);
                break;
            }
            const uint32_t due = t0 + (bit + 1U) * bit_time_us;
            while (int32_t(AP_HAL::micros() - due) < 0) {
            }
        }
        unmask_core_interrupts(basepri);
    }
    return true;
}

/*
  The same read as the STM32 path's, from serial_bit_irq() through
  serial_read_byte(): timeout_us bounds the whole read.
 */
uint16_t RCOutput::serial_read_bytes(uint8_t *buf, uint16_t len, uint32_t timeout_us)
{
    if (!line_selected) {
        return 0;
    }
    if (!receiving) {
        // a read with no write before it: listen from now
        if (!serial_rx_attach(serial_line, bit_time_us)) {
            return 0;
        }
        receiving = true;
        palSetLineMode(serial_line, LINE_RELEASED);
    }

    uint16_t i;
    const uint32_t start_us = AP_HAL::micros();
    for (i = 0; i < len; i++) {
        if (AP_HAL::micros() - start_us > timeout_us) {
            break;
        }
        if (!serial_read_byte(buf[i], timeout_us)) {
            break;
        }
    }

    // the reply is in; hold the line high until the next write, as the STM32 path does
    serial_rx_detach(serial_line);
    receiving = false;
    drive_idle(serial_line);
    return i;
}

void RCOutput::serial_end(uint32_t chanmask)
{
    // every channel serial_setup_output() took is given back, whatever the mask
    (void)chanmask;
    if (receiving) {
        serial_rx_detach(serial_line);
        receiving = false;
    }
    line_selected = false;
    for (uint8_t i = 0; i < RCOutput_pico::MAX_CHANNELS; i++) {
        RCOutput_pico::resume_channel(i);
    }
}

void RCOutput::serial_reset(uint32_t chanmask)
{
    (void)chanmask;
    if (receiving) {
        serial_rx_detach(serial_line);
        receiving = false;
    }
    if (line_selected) {
        drive_idle(serial_line);
    }
}

#endif  // defined(RP2350) && (HAL_USE_PWM == TRUE) && HAL_SERIAL_ESC_COMM_ENABLED
