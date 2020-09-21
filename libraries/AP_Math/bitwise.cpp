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
  collection of bitwise and array-wise operations.
 */


#include "bitwise.h"

void loadUint(uint8_t *b, uint16_t v, uint8_t bitCount, bool MSBfirst)
{
    const uint8_t last = bitCount/8;

//    count = 32
//    last = 4
//    MSBfirst = 1;

    for (uint8_t i=0; i<last; i++) {
        const uint8_t idx = MSBfirst ? last-1-i : i;
//        idx = 4-1-0
        b[i] = v >> (8*idx);
    }
}

uint16_t fetchU16(const uint8_t *v, bool MSBfirst)
{
    if (MSBfirst) {
        return v[1] | (v[0]<<8);
    }
    return v[0] | (v[1]<<8);
}

uint32_t fetchU24(const uint8_t *v, bool MSBfirst)
{
    if (MSBfirst) {
        return v[2] | (v[1]<<8) | (v[0]<<16);
    }
    return v[0] | (v[1]<<8) | (v[2]<<16);
}

uint32_t fetchU32(const uint8_t *v, bool MSBfirst)
{
    if (MSBfirst) {
        return v[3] | (v[2]<<8) | (v[1]<<16) | (v[0]<<24);
    }
    return v[0] | (v[1]<<8) | (v[2]<<16) | (v[3]<<24);
}
