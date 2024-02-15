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
  soft serial receive implementation, based on pulse width inputs
 */

#include "SoftSerial.h"
#include <AP_Math/crc.h>
#include <stdio.h>

SoftSerial::SoftSerial(uint32_t _baudrate, serial_config _config) :
    baudrate(_baudrate),
    config(_config),
    half_bit((1000000U / baudrate)/2)
{
    switch (config) {
    case SERIAL_CONFIG_8N1:
    case SERIAL_CONFIG_8N1I:
        data_width = 8;
        byte_width = 10;
        stop_mask = 0x200;
        break;
    case SERIAL_CONFIG_8E2I:
        data_width = 9;
        byte_width = 12;
        stop_mask = 0xC00;
        break;
    }
}

/*
  process a pulse made up of a width of values at high voltage
  followed by a width at low voltage
 */
bool SoftSerial::process_pulse(uint32_t width_high, uint32_t width_low, uint8_t &byte)
{
    // convert to bit widths, allowing for a half bit error
    uint16_t bits_high = ((width_high+half_bit)*baudrate) / 1000000;
    uint16_t bits_low = ((width_low+half_bit)*baudrate) / 1000000;

    byte_timestamp_us = timestamp_us;
    timestamp_us += (width_high + width_low);

    if (bits_high == 0 || bits_low == 0) {
        // invalid data
        goto reset;
    }

    if (bits_high >= byte_width) {
        // if we have a start bit and a stop bit then we can have at
        // most 9 bits in high state for data. The rest must be idle
        // bits
        bits_high = byte_width-1;
    }

    if (state.bit_ofs == 0) {
        // we are in idle state, waiting for first low bit. swallow
        // the high bits
        bits_high = 0;
    }

    state.byte |= ((1U<<bits_high)-1) << state.bit_ofs;

    state.bit_ofs += bits_high + bits_low;

    if (state.bit_ofs >= byte_width) {
        // check start bit
        if ((state.byte & 1) != 0) {
            goto reset;
        }
        // check stop bits
        if ((state.byte & stop_mask) != stop_mask) {
            goto reset;
        }
        if (config == SERIAL_CONFIG_8E2I) {
            // check parity
            if (parity((state.byte>>1)&0xFF) != (state.byte&0x200)>>9) {
                goto reset;
            }
        }

        byte = ((state.byte>>1) & 0xFF);
        state.byte >>= byte_width;
        state.bit_ofs -= byte_width;
        if (state.bit_ofs > byte_width) {
            state.byte = 0;
            state.bit_ofs = bits_low;
        }
        // swallow idle bits
        while (state.bit_ofs > 0 && (state.byte & 1)) {
            state.bit_ofs--;
            state.byte >>= 1;
        }
        return true;
    }
    return false;

reset:
    state.byte = 0;
    state.bit_ofs = 0;

    return false;
}

