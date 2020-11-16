/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_GYUS42v2.h"
#include <ctype.h>

extern const AP_HAL::HAL& hal;

bool AP_RangeFinder_GYUS42v2::find_signature_in_buffer(uint8_t start)
{
    for (uint8_t i=start; i<buffer_used; i++) {
        if (buffer[i] == 0x5A) {
            memmove(&buffer[0], &buffer[i], buffer_used-i);
            buffer_used -= i;
            return true;
        }
    }
    // header byte not in buffer
    buffer_used = 0;
    return false;
}

// get_reading - read a value from the sensor
bool AP_RangeFinder_GYUS42v2::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    const uint8_t num_read = uart->read(&buffer[buffer_used], ARRAY_SIZE(buffer)-buffer_used);
    buffer_used += num_read;

    if (buffer_used == 0) {
        return false;
    }

    if (buffer[0] != 0x5A &&
        !find_signature_in_buffer(1)) {
        return false;
    }

    if (buffer_used < ARRAY_SIZE(buffer)) {
        return false;
    }

    uint8_t sum = 0;
    for (uint8_t i=0; i<6; i++) {
        sum += buffer[i];
    }
    if (buffer[6] != sum) {
        find_signature_in_buffer(1);
        return false;
    }

    reading_cm = buffer[4] << 8 | buffer[5];
    buffer_used = 0;

    return true;
}
