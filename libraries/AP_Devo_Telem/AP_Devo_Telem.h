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

#ifndef AP_DEVO_TELEM_ENABLED
    #define AP_DEVO_TELEM_ENABLED   0
#endif

#if AP_DEVO_TELEM_ENABLED
class AP_DEVO_Telem {
public:
    //constructor
    AP_DEVO_Telem() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_DEVO_Telem);

    void init();

private:

    uint32_t gpsDdToDmsFormat(int32_t ddm);

    // tick - main call to send updates to transmitter
    void tick(void);

    // send_frames - sends updates down telemetry link
    void send_frames();

    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    uint32_t _last_frame_ms;

};
#endif
