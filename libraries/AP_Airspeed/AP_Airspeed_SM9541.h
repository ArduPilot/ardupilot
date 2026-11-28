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

// backend driver for the TE Connectivity SM9541 series differential
// pressure sensor

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SM9541_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_SM9541 : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_SM9541(AP_Airspeed &frontend, uint8_t _instance, float _range_cmH2O);

    ~AP_Airspeed_SM9541(void) {
        delete dev;
    }

    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void timer();

    float pressure;
    float temperature;
    float pressure_sum;
    float temperature_sum;
    uint32_t press_count;
    uint32_t temp_count;
    uint32_t last_sample_time_ms;

    // the +/- span of the part, in cmH2O
    const float range_cmH2O;

    AP_HAL::I2CDevice *dev;
};

#endif  // AP_AIRSPEED_SM9541_ENABLED
