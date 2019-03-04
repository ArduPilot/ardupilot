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
#include <AP_AHRS/AP_AHRS.h>

#if !HAL_MINIMIZE_FEATURES && AP_AHRS_NAVEKF_AVAILABLE

#include <AP_SerialManager/AP_SerialManager.h>

class AP_NMEA_Output {

public:
    static AP_NMEA_Output* probe();

    /* Do not allow copies */
    AP_NMEA_Output(const AP_NMEA_Output &other) = delete;
    AP_NMEA_Output &operator=(const AP_NMEA_Output&) = delete;

    void update();

private:
    AP_NMEA_Output();

    uint8_t _nmea_checksum(const char *str);

    static AP_NMEA_Output* _singleton;

    uint8_t _num_outputs;
    AP_HAL::UARTDriver* _uart[SERIALMANAGER_NUM_PORTS];

    uint16_t _last_sent;
};

#endif  // !HAL_MINIMIZE_FEATURES && AP_AHRS_NAVEKF_AVAILABLE
