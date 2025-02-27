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

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ASP5033_ENABLED

/*
  backend driver for airspeed from I2C
 */
#include "AP_Airspeed_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_Airspeed_ASP5033 : public AP_Airspeed_Backend
{
public:
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    ~AP_Airspeed_ASP5033(void) {
        delete dev;
    }

    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &_pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &_temperature) override;

private:
    void timer();
    bool confirm_sensor_id(void);
    float temp_sum;
    float press_sum;
    float last_pressure;
    float last_temperature;
    uint32_t press_count;
    uint32_t temp_count;
    uint32_t last_sample_ms;

    AP_HAL::I2CDevice *dev;
};

#endif  // AP_AIRSPEED_ASP5033_ENABLED
