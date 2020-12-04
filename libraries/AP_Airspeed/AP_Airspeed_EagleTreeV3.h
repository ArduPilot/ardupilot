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
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_EagleTreeV3 : public AP_Airspeed_Backend
{
public:

    AP_Airspeed_EagleTreeV3(AP_Airspeed &frontend, uint8_t _instance);
    ~AP_Airspeed_EagleTreeV3(void) {}

    // probe and initialise the sensor
    bool init() override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override {
        return false;
    }

    // true if sensor reads airspeed directly, not via pressue
    virtual bool has_airspeed() override {return true;}

    // return airspeed in m/s if available
    bool get_airspeed(float& airspeed) override;

private:

    // timer thread's data and methods:
    void timer();
    bool start_reading();
    bool get_reading(uint16_t &_reading_kph);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    // semaphore-protected shared data:
    HAL_Semaphore sem;
    uint16_t airspeed_kph;
    uint32_t last_sample_time_ms;

};
