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


   Author: Francisco Ferreira

 */

#pragma once

#include "AP_NMEA_Output_config.h"

#if HAL_NMEA_OUTPUT_ENABLED

#ifndef NMEA_MAX_OUTPUTS
#define NMEA_MAX_OUTPUTS 3
#endif

// maximum length of a fully formatted NMEA message including the trailing
// checksum and CR/LF terminator
#ifndef AP_NMEA_OUTPUT_MAX_MSG_LENGTH
#define AP_NMEA_OUTPUT_MAX_MSG_LENGTH 100
#endif

// worst case lengths of the NMEA sub-strings, including the NUL terminator
#define AP_NMEA_OUTPUT_MAX_TIME_STRING_LENGTH 12     // "HHMMSS.SS"
#define AP_NMEA_OUTPUT_MAX_DATE_STRING_LENGTH 8      // "DDMMYY"
#define AP_NMEA_OUTPUT_MAX_COORD_STRING_LENGTH 16    // "00000.00000,X"

#include <AP_Param/AP_Param.h>

class AP_NMEA_Output {

public:

    AP_NMEA_Output() {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_NMEA_Output);

    void update();

    void init();

    enum class Enabled_Messages {
        GPGGA   = (1<<0),
        GPRMC   = (1<<1),
        PASHR   = (1<<2),
    };

    static const struct AP_Param::GroupInfo var_info[];

private:

    uint8_t _num_outputs;
    AP_HAL::UARTDriver* _uart[NMEA_MAX_OUTPUTS];

    uint32_t _last_run_ms;

    AP_Int16 _interval_ms;
    AP_Int16 _message_enable_bitmask;

    // NMEA sub-strings. These are persistent buffers allocated once and
    // sized to the maximum possible formatted length so that the %s
    // consumers of the message formats always receive a NUL terminated
    // string and snprintf can never truncate into an unterminated string.
    char _time_string[AP_NMEA_OUTPUT_MAX_TIME_STRING_LENGTH];
    char _date_string[AP_NMEA_OUTPUT_MAX_DATE_STRING_LENGTH];
    char _lat_string[AP_NMEA_OUTPUT_MAX_COORD_STRING_LENGTH];
    char _lng_string[AP_NMEA_OUTPUT_MAX_COORD_STRING_LENGTH];

    // persistent message buffers, allocated once and sized for the longest
    // formatted NMEA message, so they never need to be re-allocated
    char _gga[AP_NMEA_OUTPUT_MAX_MSG_LENGTH];
    char _rmc[AP_NMEA_OUTPUT_MAX_MSG_LENGTH];
    char _pashr[AP_NMEA_OUTPUT_MAX_MSG_LENGTH];
};

#endif  // !HAL_NMEA_OUTPUT_ENABLED
