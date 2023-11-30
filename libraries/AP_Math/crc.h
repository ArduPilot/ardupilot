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
  interfaces to ArduPilot collection of CRCs. 
 */
#pragma once

#include <stdint.h>

uint16_t crc_crc4(uint16_t *data);
uint8_t crc_crc8(const uint8_t *p, uint8_t len);
uint8_t crc8_generic(const uint8_t *buf, const uint16_t buf_len, const uint8_t polynomial);     // CRC8 that does not use a lookup table for generic polynomials
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
uint8_t crc8_dvb(uint8_t crc, uint8_t a, uint8_t seed);
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length);
uint8_t crc8_dvb_update(uint8_t crc, const uint8_t* buf, const uint16_t buf_len);
uint8_t crc8_maxim(const uint8_t *data, uint16_t length);
uint8_t crc8_sae(const uint8_t *data, uint16_t length);
uint16_t crc_xmodem_update(uint16_t crc, uint8_t data);
uint16_t crc_xmodem(const uint8_t *data, uint16_t len);
uint32_t crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size);
uint32_t crc32_small(uint32_t crc, const uint8_t *buf, uint32_t size);
uint32_t crc_crc24(const uint8_t *bytes, uint16_t len);
uint16_t crc_crc16_ibm(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

// checksum used by SPORT/FPort.  For each byte, adds it to a 16-bit
// sum, then adds those two bytes together.  Returns the complement of
// the final sum.
uint8_t crc_sum8_with_carry(const uint8_t *p, uint8_t len);

// Copyright (C) 2010 Swift Navigation Inc.
// Contact: Fergus Noble <fergus@swift-nav.com>
uint16_t crc16_ccitt(const uint8_t *buf, uint32_t len, uint16_t crc);
uint16_t crc16_ccitt_r(const uint8_t *buf, uint32_t len, uint16_t crc, uint16_t out);

// CRC16_CCITT algorithm using the GDL90 parser method which is non-standard
// https://www.faa.gov/nextgen/programs/adsb/archival/media/gdl90_public_icd_reva.pdf
uint16_t crc16_ccitt_GDL90(const uint8_t *buf, uint32_t len, uint16_t crc);

uint16_t calc_crc_modbus(const uint8_t *buf, uint16_t len);

uint16_t crc_fletcher16(const uint8_t * buffer, uint32_t len);

// generate 64bit FNV1a hash from buffer
#define FNV_1_OFFSET_BASIS_64 14695981039346656037UL
void hash_fnv_1a(uint32_t len, const uint8_t* buf, uint64_t* hash);

// CRC-64-WE using the polynomial of 0x42F0E1EBA9EA3693
uint64_t crc_crc64(const uint32_t *data, uint16_t num_words);

// return the parity of byte - "1" if there is an odd number of bits
// set, "0" if there is an even number of bits set
uint8_t parity(uint8_t byte);

// sums the bytes in the supplied buffer, returns that sum mod 256
// (i.e. shoved into a uint8_t)
uint8_t crc_sum_of_bytes(const uint8_t *data, uint16_t count);

// sums the bytes in the supplied buffer, returns that sum mod 0xFFFF
uint16_t crc_sum_of_bytes_16(const uint8_t *data, uint16_t count);
