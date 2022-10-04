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

class SoftSerial {
public:
    enum serial_config {
        SERIAL_CONFIG_8N1  = 0, // DSM, SRXL etc, 8 bit, no parity, 1 stop bit
        SERIAL_CONFIG_8E2I = 1, // SBUS, 8 bit, even parity, 2 stop bits, inverted
        SERIAL_CONFIG_8N1I = 2, // FPort inverted, 8 bit, no parity, 1 stop bit
    };

    SoftSerial(uint32_t baudrate, enum serial_config config);
    bool process_pulse(const uint32_t &width_s0, const uint32_t &width_s1, const uint8_t &pulse_id, uint8_t &b);

    // get timestamp of the last byte
    uint32_t get_byte_timestamp_us(void) const {
        return byte_timestamp_us;
    }

    uint32_t baud() const { return baudrate; }

private:
    const uint32_t baudrate;
    const uint8_t half_bit; // width of half a bit in microseconds
    const enum serial_config config;

    uint8_t data_width;
    uint8_t byte_width;
    uint16_t stop_mask;
    uint32_t timestamp_us;
    uint32_t byte_timestamp_us;

    struct {
        uint32_t byte;
        uint16_t bit_ofs;
        uint8_t last_pulse_id = 255;
        uint8_t last_ret;
        uint8_t last_byte;
    } state;
};
