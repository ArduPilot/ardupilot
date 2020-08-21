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
  interfaces to ArduPilot collection of bitwise and arraywise operations
 */
#pragma once

#include <stdint.h>

void loadUint(uint8_t *b, uint16_t v, uint8_t bitCount, bool MSBfirst = true);

//void loadU16(uint8_t *b, uint16_t v, bool MSBfirst = true) { loadUx(b, v, 16, MSBfirst); }
//void loadU24(uint8_t *b, uint32_t v, bool MSBfirst = true) { loadUx(b, v, 24, MSBfirst); }
//void loadU32(uint8_t *b, uint32_t v, bool MSBfirst = true) { loadUx(b, v, 32, MSBfirst); }
uint16_t fetchU16(const uint8_t *v, bool MSBfirst = true);
uint32_t fetchU24(const uint8_t *v, bool MSBfirst = true);
uint32_t fetchU32(const uint8_t *v, bool MSBfirst = true);


