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
 */

#include "AP_RCProtocol_IBUS.h"

// constructor
AP_RCProtocol_IBUS::AP_RCProtocol_IBUS(AP_RCProtocol &_frontend) :
    AP_RCProtocol_Backend(_frontend)
{}

// decode a full IBUS frame
bool AP_RCProtocol_IBUS::ibus_decode(const uint8_t frame[IBUS_FRAME_SIZE], uint16_t *values, bool *ibus_failsafe)
{
    uint32_t chksum = 96;

    /* check frame boundary markers to avoid out-of-sync cases */
    if ((frame[0] != 0x20) || (frame[1] != 0x40)) {
        return false;
    }

    /* use the decoder matrix to extract channel data */
    for (uint8_t channel = 0, pick=2; channel < IBUS_INPUT_CHANNELS; channel++, pick+=2) {
        values[channel]=frame[pick]|(frame[pick+1] & 0x0F)<<8;
        chksum+=frame[pick]+frame[pick+1];
    }

    chksum += frame[IBUS_FRAME_SIZE-2]|frame[IBUS_FRAME_SIZE-1]<<8;

    if (chksum!=0xFFFF) {
        return false;
    }

    if ((frame[3]&0xF0) || (frame[9]&0xF0)) {
        *ibus_failsafe = true;
    } else {
        *ibus_failsafe = false;
    }

    return true;
}


/*
  process an IBUS input pulse of the given width
 */
void AP_RCProtocol_IBUS::process_pulse(const uint32_t &w0, const uint32_t &w1, const uint8_t &pulse_id)
{
    uint8_t b;
    if (ss_default.process_pulse(w0, w1, pulse_id, b)) {
        _process_byte(ss_default.get_byte_timestamp_us(), b, pulse_id);
    }
}

// support byte input
void AP_RCProtocol_IBUS::_process_byte(uint32_t timestamp_us, uint8_t b, uint8_t byte_id)
{
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= 2000U);
    byte_input.last_byte_us = timestamp_us;

    if (have_frame_gap) {
        // if we have a frame gap then this must be the start of a new
        // frame
        byte_input.ofs = 0;
        set_sync_index(frontend.push_byte(b, byte_id));
        if (get_sync_index() < 0) {
            // buffer overflow
            return;
        }
    }
    if (b != 0x20 && byte_input.ofs == 0) {
        // definately not IBUS, missing header byte
        set_sync_index(-1);
        return;
    }
    if (byte_input.ofs == 0 && !have_frame_gap) {
        // must have a frame gap before the start of a new IBUS frame
        set_sync_index(-1);
        return;
    }

    if (frontend.push_byte(b, byte_id) < 0) {
        // a logic bug in the state machine, this shouldn't happen for first condition
        set_sync_index(-1);
        return;
    }
    byte_input.ofs++;
    byte_input.buf = frontend.buffer_ptr(get_sync_index());
    if (byte_input.buf == nullptr) {
        // there is a bug, we shouldn't be getting here unless something is seriously wrong
        set_sync_index(-1);
        return;
    }

    if (byte_input.ofs == IBUS_FRAME_SIZE) {
        uint16_t values[IBUS_INPUT_CHANNELS];
        bool ibus_failsafe = false;
        log_data(AP_RCProtocol::IBUS, timestamp_us, byte_input.buf, byte_input.ofs);
        if (ibus_decode(byte_input.buf, values, &ibus_failsafe)) {
            add_input(IBUS_INPUT_CHANNELS, values, ibus_failsafe);
        }
        byte_input.ofs = 0;
    }
}

// support byte input
void AP_RCProtocol_IBUS::process_byte(uint8_t b, uint32_t baudrate, uint8_t byte_id)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), b, byte_id);
}
