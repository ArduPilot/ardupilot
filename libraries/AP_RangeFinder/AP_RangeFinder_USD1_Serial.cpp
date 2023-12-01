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

#include "AP_RangeFinder_USD1_Serial.h"

#if AP_RANGEFINDER_USD1_SERIAL_ENABLED

#include <ctype.h>

#define USD1_HDR 254   // Header Byte from USD1_Serial (0xFE)
#define USD1_HDR_V0 72 // Header Byte for beta V0 of USD1_Serial (0x48)

/*
   detect USD1_Serial Firmware Version
*/
bool AP_RangeFinder_USD1_Serial::detect_version(void)
{
    if (_version_known) {
        // return true if we've already detected the USD1_Serial version
        return true;
    } else if (uart == nullptr) {
        return false;
    }

    bool hdr_found = false;
    uint8_t byte1 = 0;
    uint8_t count = 0;

    // read any available data from USD1_Serial
    for (auto i=0; i<8192; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        if (((c == USD1_HDR_V0) || (c == USD1_HDR)) && !hdr_found) {
            byte1 = c;
            hdr_found = true;
            count++;
        } else if (hdr_found) {
            if (byte1 == USD1_HDR_V0) {
                if (++count < 4) {
                    /* need to collect 4 bytes to check for recurring
                     * header byte in the old 3-byte data format
                     */
                    continue;
                } else {
                    if (c == byte1) {
                        // if header byte is recurring, set USD1_Serial Version
                        _version = 0;
                        _header = USD1_HDR_V0;
                        _version_known = true;
                        return true;
                    } else {
                        /* if V0 header byte didn't occur again on 4th byte,
                         * start the search again for a header byte
                         */
                        count = 0;
                        byte1 = 0;
                        hdr_found = false;
                    }
                }
            } else {
                if ((c & 0x80) || (c == USD1_HDR_V0)) {
                    /* Though unlikely, it is possible we could find USD1_HDR
                     * in a data byte from the old 3-byte format. In this case,
                     * either the next byte is another data byte (which by default
                     * is of the form 0x1xxxxxxx), or the next byte is the old 
                     * header byte (USD1_HDR_V0). In this case, start the search
                     * again for a header byte.
                     */
                    count = 0;
                    byte1 = 0;
                    hdr_found = false;
                } else {
                    /* if this second byte passes the above if statement, this byte
                     * is the version number
                     */
                    _version = c;
                    _header = USD1_HDR;
                    _version_known = true;
                    return true;
                }
            }
        }
    }

    /* return false if we've gone through all available data
     * and haven't detected a USD1_Serial firmware version
     */
    return false;
}


// read - return last value measured by sensor
bool AP_RangeFinder_USD1_Serial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    
    if (!detect_version()) {
        // return false if USD1_Serial version check failed
        return false;
    }

    // read any available lines from the USD1_Serial
    float sum = 0;
    uint16_t count = 0;
    bool hdr_found = false;

    for (auto i=0; i<8192; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        if ((c == _header) && !hdr_found) {
            // located header byte
            _linebuf_len = 0;
            hdr_found   = true;
        }
        // decode index information
        if (hdr_found) {
            _linebuf[_linebuf_len++] = c;

            if ((_linebuf_len < (sizeof(_linebuf)/sizeof(_linebuf[0]))) ||
                (_version == 0 && _linebuf_len < 3)) {
                /* don't process _linebuf until we've collected six bytes of data
                 * (or 3 bytes for Version 0 firmware)
                 */
                continue;
            } else {
                if (_version == 0 && _header != USD1_HDR) {
                    // parse data for Firmware Version #0
                    sum += (_linebuf[2]&0x7F)*128 + (_linebuf[1]&0x7F);
                    count++;
                } else {
                    // evaluate checksum
                    if (((_linebuf[1] + _linebuf[2] + _linebuf[3] + _linebuf[4]) & 0xFF) == _linebuf[5]) {
                        // if checksum passed, parse data for Firmware Version #1
                        sum += _linebuf[3]*256 + _linebuf[2];
                        count++;
                    }
                }

                hdr_found = false;
                _linebuf_len = 0;
            }
        }
    }

    if (count == 0) {
        return false;
    }

    reading_m = (sum * 0.01) / count;

    if (_version == 0 && _header != USD1_HDR) {
        reading_m *= 2.5;
    }

    return true;
}

#endif  // AP_RANGEFINDER_USD1_SERIAL_ENABLED
