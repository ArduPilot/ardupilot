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
  collection of non-CRC checksum calculation functions
 */
#pragma once

#include <stdint.h>

// checksum used by SPORT/FPort.  For each byte, adds it to a 16-bit
// sum, then adds those two bytes together.  Returns the complement of
// the final sum.
uint8_t crc_sum8_with_carry(const uint8_t *p, uint8_t len);

uint16_t crc_fletcher16(const uint8_t * buffer, uint32_t len);

// generate 64bit FNV1a hash from buffer
#define FNV_1_OFFSET_BASIS_64 14695981039346656037UL
void hash_fnv_1a(uint32_t len, const uint8_t* buf, uint64_t* hash);

// return the parity of byte - "1" if there is an odd number of bits
// set, "0" if there is an even number of bits set
uint8_t parity(uint8_t byte);

// sums the bytes in the supplied buffer, returns that sum mod 256
// (i.e. shoved into a uint8_t)
uint8_t crc_sum_of_bytes(const uint8_t *data, uint16_t count);

// sums the bytes in the supplied buffer, returns that sum mod 0xFFFF
uint16_t crc_sum_of_bytes_16(const uint8_t *data, uint16_t count);
