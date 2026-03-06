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

#include "AP_IBUS2.h"

#if AP_IBUS2_ENABLED

#include <AP_Math/crc.h>

// CRC8 using polynomial 0x25 (spec notation: 0x125, the leading 1 is implicit).
uint8_t ibus2_crc8(const uint8_t *buf, uint16_t len)
{
    return crc8_generic(buf, len, 0x25);
}

bool ibus2_crc8_ok(const uint8_t *buf, uint16_t len)
{
    if (len < 2) {
        return false;
    }
    return ibus2_crc8(buf, len - 1) == buf[len - 1];
}

void ibus2_crc8_write(uint8_t *buf, uint16_t len)
{
    if (len < 2) {
        return;
    }
    buf[len - 1] = ibus2_crc8(buf, len - 1);
}

#endif  // AP_IBUS2_ENABLED
