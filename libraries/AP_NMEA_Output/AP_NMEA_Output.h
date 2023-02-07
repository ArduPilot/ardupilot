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

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_NMEA_OUTPUT_ENABLED
#define HAL_NMEA_OUTPUT_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_NMEA_OUTPUT_ENABLED

#ifndef NMEA_MAX_OUTPUTS
#define NMEA_MAX_OUTPUTS 3
#endif

class AP_NMEA_Output {

public:
    static AP_NMEA_Output* probe();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_NMEA_Output);

    void update();

private:
    AP_NMEA_Output();

    uint8_t _nmea_checksum(const char *str);

    uint8_t _num_outputs;
    AP_HAL::UARTDriver* _uart[NMEA_MAX_OUTPUTS];

    uint32_t _last_run_ms;
};

#endif  // !HAL_MINIMIZE_FEATURES
