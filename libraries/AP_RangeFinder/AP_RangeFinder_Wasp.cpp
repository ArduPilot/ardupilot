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

#include "AP_RangeFinder_Wasp.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RangeFinder_Wasp::var_info[] = {
    // @Param: WSP_MAVG
    // @DisplayName: Moving Average Range
    // @Description: Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("WSP_MAVG", 1, AP_RangeFinder_Wasp, mavg, 4),

    // @Param: WSP_MEDF
    // @DisplayName: Moving Median Filter
    // @Description: Sets the window size for the real-time median filter. When MEDF is greater than 0 the median filter is active
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("WSP_MEDF", 2, AP_RangeFinder_Wasp, medf, 4),

    // @Param: WSP_FRQ
    // @DisplayName: Frequency
    // @Description: Sets the repetition frequency of the ranging operation in Hertz. Upon entering the desired frequency the system will calculate the nearest frequency that it can handle according to the resolution of internal timers.
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("WSP_FRQ", 3, AP_RangeFinder_Wasp, frq, 20),

    // @Param: WSP_AVG
    // @DisplayName: Multi-pulse averages
    // @Description: Sets the number of pulses to be used in multi-pulse averaging mode. In this mode, a sequence of rapid fire ranges are taken and then averaged to improve the accuracy of the measurement
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("WSP_AVG", 4, AP_RangeFinder_Wasp, avg, 2),

    // @Param: WSP_THR
    // @DisplayName: Sensitivity threshold
    // @Description: Sets the system sensitivity. Larger values of THR represent higher sensitivity. The system may limit the maximum value of THR to prevent excessive false alarm rates based on settings made at the factory. Set to -1 for automatic threshold adjustments
    // @Range: -1 255
    // @User: Advanced
    AP_GROUPINFO("WSP_THR", 5, AP_RangeFinder_Wasp, thr, -1),

    // @Param: WSP_BAUD
    // @DisplayName: Baud rate
    // @Description: Desired baud rate
    // @Values: 0:Low Speed,1:High Speed
    // @User: Advanced
    AP_GROUPINFO("WSP_BAUD", 6, AP_RangeFinder_Wasp, baud, 0),

    AP_GROUPEND
};

AP_RangeFinder_Wasp::AP_RangeFinder_Wasp(RangeFinder::RangeFinder_State &_state,
                                         AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend_Serial(_state, _params)
{
    AP_Param::setup_object_defaults(this, var_info);

    state.var_info = var_info;
}

// read - return last value measured by sensor
bool AP_RangeFinder_Wasp::get_reading(uint16_t &reading_cm) {
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == '\n') {
            linebuf[linebuf_len] = 0;
            linebuf_len = 0;
            state.last_reading_ms = AP_HAL::millis();
            if (isalpha(linebuf[0])) {
                parse_response();
            } else {
                float read_value = strtof(linebuf, NULL);
                if (read_value > 0) {
                    sum += read_value;
                    count++;
                }
            }
        } else if (isalnum(c) || c == '.' || c == '-') {
            linebuf[linebuf_len++] = c;
        }

        // discard excessively long buffers
        if (linebuf_len == sizeof(linebuf)) {
            linebuf_len = 0;
        }
    }

    if (configuration_state == WASP_CFG_RATE && uart->tx_pending() == 0) {
        configuration_state = WASP_CFG_ENCODING;
    }

    if (count == 0) {
        return false;
    }

    reading_cm = 100 * sum / count;
    set_status(RangeFinder::Status::Good);

    return true;
}

#define COMMAND_BUFFER_LEN 15

void AP_RangeFinder_Wasp::update(void) {
    if (!get_reading(state.distance_cm)) {
        set_status(RangeFinder::Status::NoData);
    }

    if (AP_HAL::millis() - state.last_reading_ms > 500) {
        // attempt to reconfigure on the assumption this was a bad baud setting
        configuration_state = WASP_CFG_RATE;
    }

    char command[COMMAND_BUFFER_LEN] = {};

    switch (configuration_state) {
        case WASP_CFG_RATE:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">BAUD %s\n", baud > 0 ? "HIGH" : "LOW");
            break;
        case WASP_CFG_ENCODING:
            uart->end();
            uart->begin(baud > 0 ? 921600 : 115200);
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">LBE LITTLE\n");
            break;
        case WASP_CFG_PROTOCOL:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">FMT ASCII\n");
            break;
        case WASP_CFG_FRQ:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">FRQ %d\n", constrain_int16(frq, 20, baud > 0 ? 10000 : 1440));
            break;
        case WASP_CFG_GO:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">GO\n");
            break;
        case WASP_CFG_AUT:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">AUT %d\n", thr >= 0 ? 0 : 1);
            break;
        case WASP_CFG_THR:
            if (thr >= 0) {
                hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">THR %d\n", constrain_int16(thr, 0,255));
            } else {
                configuration_state = WASP_CFG_MAVG;
            }
            break;
        case WASP_CFG_MAVG:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">MAVG %d\n", constrain_int16(mavg, 0, 255));
            break;
        case WASP_CFG_MEDF:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">MEDF %d\n", constrain_int16(medf, 0, 255));
            break;
        case WASP_CFG_AVG:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">AVG %d\n", constrain_int16(avg, 0, 255));
            break;
        case WASP_CFG_AUV:
            hal.util->snprintf(command, COMMAND_BUFFER_LEN, ">AUV 1\n");
            break;
        case WASP_CFG_DONE:
            return;
    }

    if (command[0] != 0) {
        uart->write((uint8_t *)command, strlen(command));
    }
}

void AP_RangeFinder_Wasp::parse_response(void) {
    switch (configuration_state) {
        case WASP_CFG_RATE:
            configuration_state = WASP_CFG_ENCODING;
            break;
        case WASP_CFG_ENCODING:
            if (strncmp(linebuf, "LBE", 3) == 0) {
                configuration_state = WASP_CFG_PROTOCOL;
            }
            break;
        case WASP_CFG_PROTOCOL:
            if (strncmp(linebuf, "FMT", 3) == 0) {
                configuration_state = WASP_CFG_FRQ;
            }
            break;
        case WASP_CFG_FRQ:
            if (strncmp(linebuf, "FRQ", 3) == 0) {
                configuration_state = WASP_CFG_GO;
            }
            break;
        case WASP_CFG_GO:
            if (strncmp(linebuf, "GO", 2) == 0) {
                configuration_state = WASP_CFG_AUT;
            }
            break;
        case WASP_CFG_AUT:
            if (strncmp(linebuf, "AUT", 3) == 0) {
                configuration_state = WASP_CFG_THR;
            }
            break;
        case WASP_CFG_THR:
            if (strncmp(linebuf, "THR", 3) == 0) {
                configuration_state = WASP_CFG_MAVG;
            }
            break;
        case WASP_CFG_MAVG:
            if (strncmp(linebuf, "MAVG", 4) == 0) {
                configuration_state = WASP_CFG_MEDF;
            }
            break;
        case WASP_CFG_MEDF:
            if (strncmp(linebuf, "MEDF", 4) == 0) {
                configuration_state = WASP_CFG_AVG;
            }
            break;
        case WASP_CFG_AVG:
            if (strncmp(linebuf, "AVG", 3) == 0) {
                configuration_state = WASP_CFG_AUV;
            }
            break;
        case WASP_CFG_AUV:
            if (strncmp(linebuf, "AUV", 3) == 0) {
                configuration_state = WASP_CFG_DONE;
            }
            break;
        case WASP_CFG_DONE:
            return;
    }
}

