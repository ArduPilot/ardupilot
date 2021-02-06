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


#include "NMEA.h"

extern const AP_HAL::HAL &hal;

/*
  formatted print of NMEA message to an allocated string, with
  checksum appended
 */
char *nmea_vaprintf(const char *fmt, va_list ap)
{
    va_list ap_copy;

    // we print once to nullptr to get the length
    va_copy(ap_copy, ap);
    int len = hal.util->vsnprintf(nullptr, 0, fmt, ap_copy);
    va_end(ap_copy);
    if (len <= 0) {
        // can't print this format
        return nullptr;
    }

    // now allocate the right length, including trailer
    char *s = (char *)malloc(len+6);
    if (s == nullptr) {
        // allocation failed
        return nullptr;
    }

    if (hal.util->vsnprintf(s, len+5, fmt, ap) < len) {
        free(s);
        // inconsistent formatting
        return nullptr;
    }

    // calculate the checksum
    uint8_t cs = 0;
    const uint8_t *b = (const uint8_t *)s+1;
    while (*b) {
        cs ^= *b++;
    }

    hal.util->snprintf(s+len, 6, "*%02X\r\n", (unsigned)cs);
    return s;
}

/*
  formatted print of NMEA message to the port, with checksum appended
 */
bool nmea_printf(AP_HAL::UARTDriver *uart, const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    char *s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (s == nullptr) {
        return false;
    }

    size_t len = strlen(s);
    if (uart->txspace() < len) {
        free(s);
        return false;
    }
    uart->write((const uint8_t*)s, len);
    free(s);
    return true;
}
