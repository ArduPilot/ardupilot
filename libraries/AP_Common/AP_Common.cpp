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
/*
 *   AP_Common.cpp - common utility functions
 */


#include <AP_HAL/AP_HAL.h>
#include "AP_Common.h"

extern const AP_HAL::HAL& hal;

/*
  Return true if value is between lower and upper bound inclusive.
  False otherwise.
*/
bool is_bounded_int32(int32_t value, int32_t lower_bound, int32_t upper_bound)
{
    if ((lower_bound <= upper_bound) &&
        (value >= lower_bound) && (value <= upper_bound)) {
        return true;
    }

    return false;

}

/**
 * return the numeric value of an ascii hex character
 *
 * @param [in]  a   Hexa character
 * @param [out] res uint8 value
 * @retval true  Conversion OK
 * @retval false Input value error
 * @Note Input character is 0-9, A-F, a-f
 *  A 0x41, a 0x61, 0 0x30
 */
bool hex_to_uint8(uint8_t a, uint8_t &res)
{
    uint8_t nibble_low  = a & 0xf;

    switch (a & 0xf0) {
    case 0x30:  // 0-
        if (nibble_low > 9) {
            return false;
        }
        res = nibble_low;
        break;
    case 0x40:  // uppercase A-
    case 0x60:  // lowercase a-
        if (nibble_low == 0 || nibble_low > 6) {
            return false;
        }
        res = nibble_low + 9;
        break;
    default:
        return false;
    }
    return true;
}

/*
  strncpy without the warning for not leaving room for nul termination
 */
void strncpy_noterm(char *dest, const char *src, size_t n)
{
    size_t len = strnlen(src, n);
    if (len < n) {
        // include nul term if it fits
        len++;
    }
    memcpy(dest, src, len);
}

/**
 * return the numeric value of an ascii hex character
 * 
 * @param[in] a Hexadecimal character 
 * @return  Returns a binary value
 */
int16_t char_to_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}
