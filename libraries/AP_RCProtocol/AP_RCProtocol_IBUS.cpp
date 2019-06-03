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

#define IBUS_FRAME_SIZE		32
#define IBUS_INPUT_CHANNELS	14

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
    // uint16_t values[6] = {1500, 1500, 1500, 1500, 1500, 1500};
    //     add_input(6, values, false);
    //     #if HAL_OS_POSIX_IO
    //     #error 1
    //     #endif
    if (ss.process_pulse(w0, w1, b)) {
        //uint16_t values[6] = {1000, 1000, 1000, 1000, 1000, 1000};
        //add_input(6, values, false);
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
    uint16_t checksum = (frame[31] << 8) | frame[30];
    uint16_t calcsum = 0xFFFF;
    for(i = 0; i < 30; i++) calcsum -= frame[i];
    if(checksum != calcsum) {
        uint16_t values1[IBUS_INPUT_CHANNELS] = {1000, 1400, 1000, 1000, 1000, 1000};
        add_input(6, values1, false);
       return false;
    }

    *nvalues = max_values;
    if(*nvalues > 14) *nvalues = 14;

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
    // if (byte_input.ofs == 32 && byte_input.buf[1] == 0x20 && byte_input.buf[2] == 0x40) {
    //     memcpy(byte_input.buf, byte_input.buf + 1, 31);
    // }
    // if (byte_input.ofs == 0 && !have_frame_gap) {
    //     // must have a frame gap before the start of a new SBUS frame
    //     return;
    // }

    byte_input.buf[byte_input.ofs++] = b;

    if (byte_input.ofs == sizeof(byte_input.buf)) {
        uint16_t values[IBUS_INPUT_CHANNELS];
        uint16_t num_values=IBUS_INPUT_CHANNELS;
        bool ibus_failsafe = false;
        //bool sbus_frame_drop = false;
        if(ibus_decode(byte_input.buf, values, &num_values, IBUS_INPUT_CHANNELS)) {
            add_input(num_values, values, ibus_failsafe);
        }
        byte_input.ofs = 0;
    }
}

// support byte input
void AP_RCProtocol_IBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    //if (baudrate != 100000) {
    //    return;
    //}
    
    _process_byte(b);
}
