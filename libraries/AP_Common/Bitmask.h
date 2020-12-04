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
  simple bitmask class
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <AP_InternalError/AP_InternalError.h>

template<uint16_t num_bits>
class Bitmask {
public:
    Bitmask() :
        numbits(num_bits),
        numwords((num_bits+31)/32) {
        clearall();
    }

    Bitmask &operator=(const Bitmask&other) {
        memcpy(bits, other.bits, sizeof(bits[0])*other.numwords);
        return *this;
    }

    Bitmask(const Bitmask &other) = delete;

    // set given bitnumber
    void set(uint16_t bit) {
        // ignore an invalid bit number
        if (bit >= numbits) {
            INTERNAL_ERROR(AP_InternalError::error_t::bitmask_range);
            return;
        }
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
        bits[word] |= (1U << ofs);
    }

    // set all bits
    void setall(void) {
        // set all words to 111... except the last one.
        for (uint16_t i=0; i<numwords-1; i++) {
            bits[i] = 0xffffffff;
        }
        // set most of the last word to 111.., leaving out-of-range bits to be 0
        uint16_t num_valid_bits = numbits % 32;
        bits[numwords-1] = (1 << num_valid_bits) - 1;
    }

    // clear given bitnumber
    void clear(uint16_t bit) {
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
        bits[word] &= ~(1U << ofs);
    }

    // clear all bits
    void clearall(void) {
        memset(bits, 0, numwords*sizeof(bits[0]));
    }

    // return true if given bitnumber is set
    bool get(uint16_t bit) const {
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (bit >= numbits) {
            INTERNAL_ERROR(AP_InternalError::error_t::bitmask_range);
            return false;
        }
#endif
        return (bits[word] & (1U << ofs)) != 0;
    }

    // return true if all bits are clear
    bool empty(void) const {
        for (uint16_t i=0; i<numwords; i++) {
            if (bits[i] != 0) {
                return false;
            }
        }
        return true;
    }

    // return number of bits set
    uint16_t count() const {
        uint16_t sum = 0;
        for (uint16_t i=0; i<numwords; i++) {
            if (sizeof(bits[i]) <= sizeof(int)) {
                sum += __builtin_popcount(bits[i]);
            } else if (sizeof(bits[i]) <= sizeof(long)) {
                sum += __builtin_popcountl(bits[i]);
            } else {
                sum += __builtin_popcountll(bits[i]);
            }
        }
        return sum;
    }

    // return first bit set, or -1 if none set
    int16_t first_set() const {
        for (uint16_t i=0; i<numwords; i++) {
            if (bits[i] == 0) {
                continue;
            }
            int fs;
            if (sizeof(bits[i]) <= sizeof(int)) {
                fs = __builtin_ffs(bits[i]);
            } else if (sizeof(bits[i]) <= sizeof(long)) {
                fs = __builtin_ffsl(bits[i]);
            } else {
                fs = __builtin_ffsll(bits[i]);
            }
            return i*32 + fs - 1;
        }
        return -1;
    }

    // return number of bits available
    uint16_t size() const {
        return numbits;
    }

private:
    uint16_t numbits;
    uint16_t numwords;
    uint32_t bits[(num_bits+31)/32];
};
