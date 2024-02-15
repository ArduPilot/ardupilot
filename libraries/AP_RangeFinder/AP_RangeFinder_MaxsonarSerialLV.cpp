/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_MaxsonarSerialLV.h"

#if AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

#define MAXSONAR_SERIAL_LV_BAUD_RATE 9600

extern const AP_HAL::HAL& hal;

AP_RangeFinder_MaxsonarSerialLV::AP_RangeFinder_MaxsonarSerialLV(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params):
    AP_RangeFinder_Backend_Serial(_state, _params)
{
    params.scaling.set_default(0.0254f);
}

// read - return last value measured by sensor
bool AP_RangeFinder_MaxsonarSerialLV::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    int32_t sum = 0;
    uint16_t count = 0;

    for (auto i=0; i<8192; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
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

    // This sonar gives the metrics in inches, so we have to transform this to meters
    reading_m = params.scaling * (float(sum) / count);

    return true;
}

#endif  // AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
