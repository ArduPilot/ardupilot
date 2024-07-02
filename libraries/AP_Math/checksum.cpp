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

#include <stdint.h>
#include "checksum.h"

#include <AP_HAL/AP_HAL_Boards.h>

// fletcher 16 implementation
uint16_t cksum_fletcher16(const uint8_t *buffer, uint32_t len) {
    uint16_t c0 = 0;
    uint16_t c1 = 0;
    for (uint32_t i = 0; i < len; i++) {
        c0 = (c0 + buffer[i]) % 255;
        c1 = (c1 + c0) % 255;
    }

    return (c1 << 8) | c0;
}

// FNV-1a implementation
#define FNV_1_PRIME_64 1099511628211UL
void cksum_fnv_1a(uint32_t len, const uint8_t* buf, uint64_t* hash)
{
    uint32_t i;
    for (i=0; i<len; i++) {
        *hash ^= (uint64_t)buf[i];
        *hash *= FNV_1_PRIME_64;
    }
}

// simple 8 bit checksum used by FPort
uint8_t cksum_fport(const uint8_t *p, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i=0; i<len; i++) {
        sum += p[i];
        sum += sum >> 8;
        sum &= 0xFF;              
    }
    sum = 0xff - ((sum & 0xff) + (sum >> 8));
    return sum;
}

// return the parity of byte - "1" if there is an odd number of bits
// set, "0" if there is an even number of bits set note that
// __builtin_parity causes hardfaults on Pixracer-periph - and is
// slower on 1 byte than this:
uint8_t cksum_byte_parity(uint8_t byte)
{
    uint8_t p = 0;

    p ^= byte & 0x1;
    byte >>= 1;
    p ^= byte & 0x1;
    byte >>= 1;
    p ^= byte & 0x1;
    byte >>= 1;
    p ^= byte & 0x1;
    byte >>= 1;
    p ^= byte & 0x1;
    byte >>= 1;
    p ^= byte & 0x1;
    byte >>= 1;
    p ^= byte & 0x1;
    byte >>= 1;
    p ^= byte & 0x1;

    return p;
}

// sums the bytes in the supplied buffer, returns that sum mod 0xFFFF
uint16_t cksum_sum16(const uint8_t *data, uint16_t count)
{
    uint16_t ret = 0;
    for (uint32_t i=0; i<count; i++) {
        ret += data[i];
    }
    return ret;
}

// sums the bytes in the supplied buffer, returns that sum mod 256
// (i.e. shoved into a uint8_t)
uint8_t cksum_sum8(const uint8_t *data, uint16_t count)
{
    return cksum_sum16(data, count) & 0xFF;
}
