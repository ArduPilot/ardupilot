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

#include <AP_HAL/AP_HAL.h>

/*
  formatted print of NMEA message to an allocated string, with
  checksum appended
 */
char *nmea_vaprintf(const char *fmt, va_list ap);

/*
  formatted print of NMEA message to a uart, with checksum appended
 */
bool nmea_printf(AP_HAL::UARTDriver *uart, const char *fmt, ...) FMT_PRINTF(2,3);

/*
  formatted print of NMEA message to a buffer, with checksum appended.
  Returns the length of the string filled into buf. If the NMEA string does not fit in the buffer, returns 0
 */
uint16_t nmea_printf_buffer(char* buf, const uint16_t buf_max_len, const char *fmt, ...);

