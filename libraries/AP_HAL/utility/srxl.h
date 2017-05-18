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
  SRXL protocol decoder
  Andrew Tridgell, September 2016
 */

#pragma once

/*
 * Decoder for SRXL protocol
 *
 * @return 0 for success (a decoded packet), 1 for no packet yet (accumulating), 2 for unknown packet, 4 for checksum error
 */
int srxl_decode(uint64_t timestamp_us, uint8_t byte, uint8_t *num_values, uint16_t *values, uint16_t max_values, bool *failsafe_state);
