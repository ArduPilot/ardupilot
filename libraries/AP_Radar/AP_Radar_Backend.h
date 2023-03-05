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

/*
  AP_Radar backend class for ArduPilot
 */

#include "AP_Radar.h"

class Radar_backend
{
    friend class AP_Radar;

public:
    // constructor
    Radar_backend(AP_Radar &_frontend);
    virtual ~Radar_backend(void);

    CLASS_NO_COPY(Radar_backend);

    // init - initialise sensor
    virtual void init() {}

    // periodic update
    virtual void update() = 0;

    // handle mavlink messages
    virtual void handle_msg(const mavlink_message_t &msg) {}

#if HAL_MSP_RADAR_ENABLED
    // handle msp messages
    virtual void handle_msp(const MSP::msp_radar_pos_message_t &pkt) {}
#endif

protected:
    // access to frontend
    AP_Radar &frontend;

    // update the frontend
    void _update_frontend(const struct AP_Radar::Radar_state &state);

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;
};
