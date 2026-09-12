/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * PIO-driven outputs for RP2350: DShot600 (bidirectional or not) on PIO2, and
 * WS2812 / NeoPixel on PIO1.
 */
#pragma once

#include "AP_HAL_ChibiOS.h"

#if defined(RP2350) && (HAL_USE_PWM == TRUE) && (HAL_DSHOT_ENABLED || HAL_SERIALLED_ENABLED)

namespace ChibiOS {

/*
  The two output types have nothing in common but the peripheral: separate PIO
  blocks, separate programs, separate state machines. They share a class only
  because they share the register-level plumbing for talking to a PIO, which is
  otherwise duplicated.

  The block each one lives on is forced, not chosen. See RCOutput_pico.cpp.
 */
class RCOutput_pico {
public:
    // one state machine per channel, four per PIO block
    static constexpr uint8_t MAX_CHANNELS = 4;

#if HAL_DSHOT_ENABLED
    /*
      Claim the PIO block and load the program for the requested direction.
      Safe to call again to change direction; the block is reloaded. Returns
      false if the PIO could not be claimed.
     */
    static bool init(bool bidir);

    // bind a channel to a GPIO. init() must have been called first.
    static bool add_channel(uint8_t chan, uint8_t gpio);

    // queue an already-encoded DShot frame (16 bits, checksum included)
    static void write_frame(uint8_t chan, uint16_t frame);

    /*
      Bidirectional only: pull one telemetry response out of the RX FIFO and
      decode it. Returns false when no complete response is waiting or the
      GCR decode failed. On success 'encoded' is the 16-bit value that
      RCOutput::bdshot_decode_telemetry_from_erpm() expects.
     */
    static bool read_telemetry(uint8_t chan, uint16_t &encoded);

    static bool is_bidir(void) { return _bidir; }
    static bool is_initialised(void) { return _initialised; }

    // channels bound so far, as a mask
    static uint32_t channel_mask(void) { return _chan_mask; }
#endif // HAL_DSHOT_ENABLED

#if HAL_SERIALLED_ENABLED
    // claim the NeoPixel PIO block and load the program. Repeatable.
    static bool neopixel_init(void);

    // bind an LED chain to a GPIO. neopixel_init() must have been called first.
    static bool neopixel_add_channel(uint8_t idx, uint8_t gpio);

    /*
      A frame is streamed a word at a time rather than handed over as a buffer,
      so nothing has to hold 128 LEDs of packed colour anywhere. Call
      neopixel_send_begin(), then neopixel_send_word() per LED, then
      neopixel_send_end().

      neopixel_send_begin() returns false if the chain is not bound or the
      previous frame has not yet cleared its reset gap. neopixel_send_word()
      returns false if the FIFO did not drain inside the timeout, and the
      caller should stop.

      neopixel_send_word() blocks for roughly 30 us per LED once the FIFO is
      full, which is what the eight word depth costs on a longer chain. It is
      called from the LED thread, whose whole purpose is to absorb exactly this.
     */
    static bool neopixel_send_begin(uint8_t idx);
    static bool neopixel_send_word(uint8_t idx, uint32_t word);
    static void neopixel_send_end(uint8_t idx);

    /*
      WS2812 wants the byte order green, red, blue, and the state machine
      shifts out of the top of the word, so the 24 bits sit in bits 31..8.
     */
    static uint32_t neopixel_pack_grb(uint8_t r, uint8_t g, uint8_t b)
    {
        return ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
    }

    // WS2811-style parts take the same frame in red, green, blue order
    static uint32_t neopixel_pack_rgb(uint8_t r, uint8_t g, uint8_t b)
    {
        return ((uint32_t)r << 24) | ((uint32_t)g << 16) | ((uint32_t)b << 8);
    }
#endif // HAL_SERIALLED_ENABLED

private:
#if HAL_DSHOT_ENABLED
    static void load_program(bool bidir);
    static void start_sm(uint8_t chan, uint8_t gpio);

    /*
      A run of n samples is worth n/samples_per_bit bits, but the boundaries
      are not at the half-way points - a run has to be long enough that it
      rounds up. Build the thresholds once so the inner loop is three compares.
     */
    static void set_transitions(float bits_per_sample);

    // put a state machine parked on a wait back at the top of the program
    static void restart_sm(uint8_t chan, uint16_t frame);

    static bool     _initialised;
    static bool     _bidir;
    static uint32_t _chan_mask;
    static uint8_t  _gpio[MAX_CHANNELS];

    // sample counts at which a run becomes 1, 2, 3 or 4 bits long
    static uint8_t  _len_transition[4];

    // consecutive updates found parked on a wait, and how often that has
    // forced a restart - the second is a diagnostic for a flaky ESC or wiring
    static uint8_t  _stall_count[MAX_CHANNELS];
    static uint32_t _stall_restarts[MAX_CHANNELS];
#endif // HAL_DSHOT_ENABLED

#if HAL_SERIALLED_ENABLED
    static void neopixel_start_sm(uint8_t idx, uint8_t gpio);

    static bool     _neop_initialised;
    static uint32_t _neop_chan_mask;
    static uint8_t  _neop_gpio[MAX_CHANNELS];

    // end of the last frame, for the inter-frame reset gap
    static uint32_t _neop_last_send_us[MAX_CHANNELS];
#endif // HAL_SERIALLED_ENABLED
};

} // namespace ChibiOS

#endif // defined(RP2350) && (HAL_USE_PWM == TRUE) && (HAL_DSHOT_ENABLED || HAL_SERIALLED_ENABLED)
