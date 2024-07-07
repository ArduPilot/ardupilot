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
    static_assert(num_bits > 0, "must store something");
    // for first_set()'s return value
    static_assert(num_bits <= INT16_MAX, "must fit in int16_t");
    // so that 1U << bits is in range
    static_assert(sizeof(unsigned int) >= sizeof(uint32_t), "int too small");

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

    bool operator==(const Bitmask&other) {
        if (other.numbits != numbits) {
            return false;
        } else {
            return memcmp(bits, other.bits, sizeof(bits[0])*numwords) == 0;
        }
    }

    bool operator!=(const Bitmask&other) {
        return !(*this == other);
    }

    Bitmask(const Bitmask &other) = delete;

    // set given bitnumber
    void set(uint16_t bit) {
        if (!validate(bit)) {
            return; // ignore access of invalid bit
        }
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
        bits[word] |= (1U << ofs);
    }

    // set all bits
    void setall(void) {
        // set all words to 111...
        for (uint16_t i=0; i<numwords; i++) {
            bits[i] = 0xffffffff;
        }
        // ensure out-of-range bits in the last word, if any exist, are 0
        uint16_t num_valid_bits = numbits % 32;
        if (num_valid_bits) { // word has out of range bits
            bits[numwords-1] = (1U << num_valid_bits) - 1;
        }
    }

    // clear given bitnumber
    void clear(uint16_t bit) {
        if (!validate(bit)) {
            return; // ignore access of invalid bit
        }
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
        bits[word] &= ~(1U << ofs);
    }

    // set given bitnumber to on/off
    void setonoff(uint16_t bit, bool onoff) {
        if (onoff) {
            set(bit);
        } else {
            clear(bit);
        }
    }

    // clear all bits
    void clearall(void) {
        memset(bits, 0, numwords*sizeof(bits[0]));
    }

    // return true if given bitnumber is set
    bool get(uint16_t bit) const {
        if (!validate(bit)) {
            return false; // pretend invalid bit is not set
        }
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
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
            sum += __builtin_popcount(bits[i]);
        }
        return sum;
    }

    // return first bit set, or -1 if none set
    int16_t first_set() const {
        for (uint16_t i=0; i<numwords; i++) {
            if (bits[i] != 0) {
                return i*32 + __builtin_ffs(bits[i]) - 1;
            }
        }
        return -1;
    }

    // return number of bits available
    uint16_t size() const {
        return numbits;
    }

private:
    bool validate(uint16_t bit) const {
        if (bit >= numbits) {
            INTERNAL_ERROR(AP_InternalError::error_t::bitmask_range);
            return false;
        }
        return true;
    }

    uint16_t numbits;
    uint16_t numwords;
    uint32_t bits[(num_bits+31)/32];
};
