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
 * datasheet: https://www.maxbotix.com/documents/IRXL-MaxSonar-CS_Datasheet.pdf
 */
#include "AP_RangeFinder_MaxsonarSerialIRXL.h"
#include <ctype.h>

/**
 * Serial Output: The serial output is either RS232 or TTL format (0 to Vcc) with a 1 cm resolution. The output is an
 * ASCII capital “R”, followed by four ASCII character digits representing the range in centimeters, followed by a carriage
 * return (ASCII 13). The maximum distance reported is 1650. The serial output is the most accurate of the range outputs.
 * Serial data sent is 9600 baud, with 8 data bits, no parity, and one stop bit. 
 */

// read - return last value measured by sensor
bool AP_RangeFinder_MaxsonarSerialIRXL::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    int32_t sum = 0;
    int16_t nbytes = uart->available();
    uint16_t count = 0;

    while (nbytes-- > 0) {
        int16_t c = uart->read();
        if (c < 0 || c == 'R') {
            linebuf_len = 0;
            continue;
        }
        if (c == '\r') {
            linebuf[linebuf_len] = 0;
            sum += (int)atoi(linebuf);
            count++;
            linebuf_len = 0;
        } else if (isdigit(c)) {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    if (count == 0) {
        return false;
    }

    reading_cm = sum / count;
    //gcs().send_text(MAV_SEVERITY_DEBUG, "### read %d", reading_cm);

    return true;
}
