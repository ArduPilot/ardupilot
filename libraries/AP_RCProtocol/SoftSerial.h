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

#pragma once
#include <stdint.h>

enum SerialProtocolConfig : uint8_t {
    SERIAL_CONFIG_8N1  = 0, // DSM, SRXL, FPORT etc, 8 bit, no parity, 1 stop bit
    SERIAL_CONFIG_8E2  = 1, // SBUS, 8 bit, even parity, 2 stop bits
};

class SoftSerial {
public:
    SoftSerial() : SoftSerial(115200, SERIAL_CONFIG_8N1, false) {}
    SoftSerial(uint32_t baudrate, SerialProtocolConfig protocol, bool inverted);
    bool process_pulse(uint32_t width_s0, uint32_t width_s1, uint8_t& b);
    void configure(uint32_t baudrate, SerialProtocolConfig protocol, bool inverted);

    // get timestamp of the last byte
    uint32_t get_byte_timestamp_us(void) const {
        return byte_timestamp_us;
    }

    uint32_t baud() const { return baudrate; }
    void reset();

private:
    uint32_t baudrate;
    uint8_t half_bit; // width of half a bit in microseconds
    SerialProtocolConfig config;

    uint8_t data_width;
    uint8_t byte_width;
    uint16_t stop_mask;
    uint32_t timestamp_us;
    uint32_t byte_timestamp_us;

    // saved history to support inversion
    bool inverted;
    uint32_t saved_width;

    struct {
        uint32_t byte;
        uint16_t bit_ofs;
    } state;
};
