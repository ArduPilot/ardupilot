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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "AP_RCProtocol_PPMSum.h"

/*
  process a PPM-sum pulse of the given width
 */
void AP_RCProtocol_PPMSum::process_pulse(const uint32_t &width_s0, const uint32_t &width_s1, const uint8_t &pulse_id)
{
    if (width_s0 == 0 || width_s1 == 0) {
        //invalid data: reset frame
        ppm_state._channel_counter = -1;
        return;
    }
    uint32_t width_usec = width_s0 + width_s1;
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        if (ppm_state._channel_counter >= MIN_RCIN_CHANNELS) {
            add_input(ppm_state._channel_counter, ppm_state._pulse_capt, false);
        }
        ppm_state._channel_counter = 0;
        return;
    }
    if (ppm_state._channel_counter == -1) {
        // we are not synchronised
        return;
    }

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        // buffer these
        ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

        // move to next channel
        ppm_state._channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (ppm_state._channel_counter >= MAX_RCIN_CHANNELS) {
        add_input(ppm_state._channel_counter, ppm_state._pulse_capt, false);
        ppm_state._channel_counter = -1;
    }
}
