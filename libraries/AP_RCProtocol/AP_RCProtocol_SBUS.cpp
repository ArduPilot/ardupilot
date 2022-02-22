/*
  SBUS decoder, based on src/modules/px4iofirmware/sbus.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
 */
/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
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

#include "AP_RCProtocol_SBUS.h"

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200
#define SBUS_RANGE_MAX 1800
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

#define SBUS_TARGET_MIN 1000
#define SBUS_TARGET_MAX 2000
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

// this is 875
#define SBUS_SCALE_OFFSET (SBUS_TARGET_MIN - ((SBUS_TARGET_RANGE * SBUS_RANGE_MIN / SBUS_RANGE_RANGE)))

#ifndef HAL_SBUS_FRAME_GAP
#define HAL_SBUS_FRAME_GAP 2000U
#endif

// constructor
AP_RCProtocol_SBUS::AP_RCProtocol_SBUS(AP_RCProtocol &_frontend, bool _inverted, uint32_t configured_baud) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted),
    ss{configured_baud, SoftSerial::SERIAL_CONFIG_8E2I}
{}

// decode a full SBUS frame
bool AP_RCProtocol_SBUS::sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
                                     bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{
    /* check frame boundary markers to avoid out-of-sync cases */
    if ((frame[0] != 0x0f)) {
        return false;
    }

    switch (frame[24]) {
    case 0x00:
        /* this is S.BUS 1 */
        break;
    case 0x03:
        /* S.BUS 2 SLOT0: RX battery and external voltage */
        break;
    case 0x83:
        /* S.BUS 2 SLOT1 */
        break;
    case 0x43:
    case 0xC3:
    case 0x23:
    case 0xA3:
    case 0x63:
    case 0xE3:
        break;
    default:
        /* we expect one of the bits above, but there are some we don't know yet */
        break;
    }

    uint16_t chancount = SBUS_INPUT_CHANNELS;

    decode_11bit_channels((const uint8_t*)(&frame[1]), SBUS_INPUT_CHANNELS, values,
        SBUS_TARGET_RANGE, SBUS_RANGE_RANGE, SBUS_SCALE_OFFSET);

    /* decode switch channels if data fields are wide enough */
    if (max_values > 17 && SBUS_INPUT_CHANNELS > 15) {
        chancount = 18;

        /* channel 17 (index 16) */
        values[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0))?1998:998;
        /* channel 18 (index 17) */
        values[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1))?1998:998;
    }

    /* note the number of channels decoded */
    *num_values = chancount;

    /* decode and handle failsafe and frame-lost flags */
    if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
        /* report that we failed to read anything valid off the receiver */
        *sbus_failsafe = true;
        *sbus_frame_drop = true;
    } else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
        /* set a special warning flag
         *
         * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
         * condition as fail-safe greatly reduces the reliability and range of the radio link,
         * e.g. by prematurely issuing return-to-launch!!! */

        *sbus_failsafe = false;
        *sbus_frame_drop = true;
    } else {
        *sbus_failsafe = false;
        *sbus_frame_drop = false;
    }

    return true;
}


/*
  process a SBUS input pulse of the given width
 */
void AP_RCProtocol_SBUS::process_pulse(uint32_t width_s0, uint32_t width_s1)
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
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

// support byte input
void AP_RCProtocol_SBUS::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= HAL_SBUS_FRAME_GAP);
    byte_input.last_byte_us = timestamp_us;

    if (have_frame_gap) {
        // if we have a frame gap then this must be the start of a new
        // frame
        byte_input.ofs = 0;
    }
    if (b != 0x0F && byte_input.ofs == 0) {
        // definately not SBUS, missing header byte
        return;
    }
    if (byte_input.ofs == 0 && !have_frame_gap) {
        // must have a frame gap before the start of a new SBUS frame
        return;
    }

    byte_input.buf[byte_input.ofs++] = b;

    if (byte_input.ofs == sizeof(byte_input.buf)) {
        log_data(AP_RCProtocol::SBUS, timestamp_us, byte_input.buf, byte_input.ofs);
        uint16_t values[SBUS_INPUT_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe = false;
        bool sbus_frame_drop = false;
        if (sbus_decode(byte_input.buf, values, &num_values,
                        &sbus_failsafe, &sbus_frame_drop, SBUS_INPUT_CHANNELS) &&
            num_values >= MIN_RCIN_CHANNELS) {
            add_input(num_values, values, sbus_failsafe);
        }
        byte_input.ofs = 0;
    }
}

// support byte input
void AP_RCProtocol_SBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    // note that if we're here we're not actually using SoftSerial,
    // but it does record our configured baud rate:
    if (baudrate != ss.baud()) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}
