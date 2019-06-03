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
 * Code by Sidhant Goel
 */
/*
  IBUS decoder based on SBUS decoder
 */

#include "AP_RCProtocol_IBUS.h"
#include <AP_Math/AP_Math.h>

// constructor
AP_RCProtocol_IBUS::AP_RCProtocol_IBUS(AP_RCProtocol &_frontend, bool _inverted) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted)
{}

/*
  process a IBUS input pulse of the given width
 */
void AP_RCProtocol_IBUS::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t w0 = width_s0;
    uint32_t w1 = width_s1;
    if (inverted) {
        w0 = saved_width;
        w1 = width_s0;
        saved_width = width_s1;
    }
    uint8_t b;
    if (ss.process_pulse(w0, w1, b)) {
        _process_byte(b);
    }
}

// decode a full IBUS frame
bool AP_RCProtocol_IBUS::ibus_decode(const uint8_t frame[32], uint16_t *values, uint16_t *nvalues, uint16_t max_values)
{
    int i;
    /* check frame boundary markers to avoid out-of-sync cases */
    if ((frame[0] != 0x20)) {
        return false;
    }
    if ((frame[1] != 0x40)) {
        return false;
    }
    uint16_t checksum = (frame[IBUS_FRAME_SIZE - 1] << 8) | frame[IBUS_FRAME_SIZE - 2];
    uint16_t calcsum = 0xFFFF;
    for(i = 0; i < (IBUS_FRAME_SIZE - 2); i++) {
        calcsum -= frame[i];
    }
    if(checksum != calcsum) {
       return false;
    }

    *nvalues = MIN(max_values, IBUS_INPUT_CHANNELS);

    for(i = 0; i < *nvalues; i++) {
        values[i] = (frame[2 + 2 * i + 1] << 8) | frame[2 + 2 * i];
    }

    return true;
}

// support byte input
void AP_RCProtocol_IBUS::_process_byte(uint8_t b)
{
    if (byte_input.ofs == 0 && b != 0x20) {
        // definately not IBUS, missing header byte
        return;
    }
    
    if (byte_input.ofs == 1 && b != 0x40) {
        // definately not IBUS, missing header byte
        byte_input.ofs = 0;
        return;
    }

    byte_input.buf[byte_input.ofs++] = b;

    if (byte_input.ofs == sizeof(byte_input.buf)) {
        uint16_t values[IBUS_INPUT_CHANNELS];
        uint16_t num_values=IBUS_INPUT_CHANNELS;
        if(ibus_decode(byte_input.buf, values, &num_values, IBUS_INPUT_CHANNELS)) {
            add_input(num_values, values, false);
        }
        byte_input.ofs = 0;
    }
}

// support byte input
void AP_RCProtocol_IBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    _process_byte(b);
}
