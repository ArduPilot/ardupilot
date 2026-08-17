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

/* assert that const vals are float, not double. so 100.0 means 100.0f */
static_assert(sizeof(1e6) == sizeof(float), "Compilation needs to use single-precision constants");

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
bool hex_char_to_nibble(uint8_t a, uint8_t &res)
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
  decode two hex characters into a byte.
  e.g. hex_twochars_to_uint8("3F", res) -> res = 0x3F
 */
bool hex_twochars_to_uint8(const char s[2], uint8_t &res)
{
    uint8_t hi, lo;
    if (!hex_char_to_nibble(s[0], hi) || !hex_char_to_nibble(s[1], lo)) {
        return false;
    }
    res = (hi << 4) | lo;
    return true;
}

bool hex_charpairs_to_uint8s(const char *s, uint8_t num_pairs, uint8_t *out)
{
    for (uint8_t i = 0; i < num_pairs; i++) {
        uint8_t hi, lo;
        if (!hex_char_to_nibble(s[i*2], hi) || !hex_char_to_nibble(s[i*2+1], lo)) {
            return false;
        }
        out[i] = (hi << 4) | lo;
    }
    return true;
}

/*
  decode len hex characters into a uint32_t, treating each character
  as a nibble (most-significant first).
  e.g. hex_chars_to_uint32("1A2B", 4, out) -> out = 0x1A2B
 */
bool hex_chars_to_uint32(const char *s, uint8_t len, uint32_t &out)
{
    out = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t nibble;
        if (!hex_char_to_nibble(s[i], nibble)) {
            return false;
        }
        out = (out << 4) | nibble;
    }
    return true;
}

/*
  strncpy without the warning for not leaving room for nul termination
 */
size_t strncpy_noterm(char *dest, const char *src, size_t n)
{
    size_t len = strnlen(src, n);
    size_t ret = len; // return value is length of src
    if (len < n) {
        // include nul term if it fits
        len++;
    }
    memcpy(dest, src, len);
    return ret;
}
