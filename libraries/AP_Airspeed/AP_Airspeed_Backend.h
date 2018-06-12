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
  backend driver class for airspeed
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Airspeed.h"

class AP_Airspeed_Backend {
public:
    AP_Airspeed_Backend(AP_Airspeed &frontend, uint8_t instance);
    virtual ~AP_Airspeed_Backend();
    
    // probe and initialise the sensor
    virtual bool init(void) = 0;

    // return the current differential_pressure in Pascal
    virtual bool get_differential_pressure(float &pressure) = 0;

    // return the current temperature in degrees C, if available
    virtual bool get_temperature(float &temperature) = 0;

protected:
    int8_t get_pin(void) const;
    float get_psi_range(void) const;
    uint8_t get_bus(void) const;

    AP_Airspeed::pitot_tube_order get_tube_order(void) const {
        return AP_Airspeed::pitot_tube_order(frontend.param[instance].tube_order.get());
    }
    
    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *sem;

    float get_airspeed_ratio(void) const {
        return frontend.get_airspeed_ratio(instance);
    }

    // some sensors use zero offsets
    void set_use_zero_offset(void) {
        frontend.state[instance].use_zero_offset = true;
    }

    // set to no zero cal, which makes sense for some sensors
    void set_skip_cal(void) {
        frontend.param[instance].skip_cal.set(1);
    }

    // set zero offset
    void set_offset(float ofs) {
        frontend.param[instance].offset.set(ofs);
    }
    
private:
    AP_Airspeed &frontend;
    uint8_t instance;
};
