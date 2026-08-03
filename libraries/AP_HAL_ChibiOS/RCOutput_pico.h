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
 * DShot600 output for RP2350 using the PIO, bidirectional or not.
 */
#pragma once

#include "AP_HAL_ChibiOS.h"

#if defined(RP2350) && HAL_DSHOT_ENABLED

namespace ChibiOS {

/*
  One PIO state machine drives one motor. The PIO program is shared by all of
  them, so a block gives four channels; see RCOutput_pico.cpp for why only one
  of the two programs can be resident at a time.
 */
class RCOutput_pico {
public:
    // one state machine per channel, four per PIO block
    static constexpr uint8_t MAX_CHANNELS = 4;

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

private:
    static void load_program(bool bidir);
    static void start_sm(uint8_t chan, uint8_t gpio);

    /*
      A run of n samples is worth n/samples_per_bit bits, but the boundaries
      are not at the half-way points - a run has to be long enough that it
      rounds up. Build the thresholds once so the inner loop is three compares.
     */
    static void set_transitions(float bits_per_sample);

    static bool     _initialised;
    static bool     _bidir;
    static uint32_t _chan_mask;
    static uint8_t  _gpio[MAX_CHANNELS];

    // sample counts at which a run becomes 1, 2, 3 or 4 bits long
    static uint8_t  _len_transition[4];
};

} // namespace ChibiOS

#endif // defined(RP2350) && HAL_DSHOT_ENABLED
