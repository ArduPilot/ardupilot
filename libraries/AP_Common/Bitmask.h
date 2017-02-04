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

class Bitmask {
public:
    Bitmask(uint16_t numbits) :
        numwords((numbits+31)/32) {
        bits = new uint32_t[numwords];
    }
    ~Bitmask(void) {
        delete[] bits;
    }

    // set given bitnumber
    void set(uint16_t bit) {
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
        bits[word] |= (1U << ofs);
    }

    // clear given bitnumber
    void clear(uint16_t bit) {
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
        bits[word] &= ~(1U << ofs);
    }

    // clear all bits
    void clearall(void) {
        for (uint16_t i=0; i<numwords; i++) {
            bits[i] = 0;
        }
    }
    
    // return true if given bitnumber is set
    bool get(uint16_t bit) {
        uint16_t word = bit/32;
        uint8_t ofs = bit & 0x1f;
        return (bits[word] & (1U << ofs)) != 0;
    }

    // return true if all bits are clear
    bool empty(void) {
        for (uint16_t i=0; i<numwords; i++) {
            if (bits[i] != 0) {
                return false;
            }
        }
        return true;
    }
    
private:
    uint16_t numwords;
    uint32_t *bits;
};
