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

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Doppler_Backend.h"

class AP_Doppler_Parameters;

class AP_Doppler_Telem {

public:
    AP_Doppler_Telem();

    ~AP_Doppler_Telem();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Doppler_Telem);

    // init - perform required initialisation
    bool init(const AP_SerialManager &serial_manager);
    void update();
    void send();

    static AP_Doppler_Telem *get_singleton(void) {
        return singleton;
    }

protected:
    

private:

    AP_Doppler_Parameters *_doppler_parameters;
    AP_HAL::UARTDriver *port;
    AP_Doppler_Backend *_backend;
    static AP_Doppler_Telem *singleton;

};

namespace AP {
    AP_Doppler_Telem *Doppler_telem();
};

