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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#ifndef HAL_HOTT_TELEM_ENABLED
#define HAL_HOTT_TELEM_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_HOTT_TELEM_ENABLED
class AP_Hott_Telem {
public:
    AP_Hott_Telem();

    /* Do not allow copies */
    AP_Hott_Telem(const AP_Hott_Telem &other) = delete;
    AP_Hott_Telem &operator=(const AP_Hott_Telem&) = delete;

    static AP_Hott_Telem *get_singleton(void) {
        return singleton;
    }

    // initialise
    void init(void);

private:
    static AP_Hott_Telem *singleton;

    AP_HAL::UARTDriver *uart;

    void loop(void);
    void send_EAM();
    void send_GPS();
    void send_Vario();
    void send_packet(const uint8_t *b, uint8_t len);
    void GPS_to_DDM(float decimal, uint8_t &sign, uint16_t &dm, uint16_t &sec) const;

    float min_alt, max_alt;
};

namespace AP {
    AP_Hott_Telem *hott_telem();
};
#endif // HAL_HOTT_TELEM_ENABLED
