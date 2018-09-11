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

#include <cstdint>

uint8_t crc_crc8(const uint8_t *p, uint8_t len);
uint16_t crc_xmodem_update(uint16_t crc, uint8_t data);
uint16_t crc_xmodem(const uint8_t *data, uint16_t len);
uint32_t crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size);
