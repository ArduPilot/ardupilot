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

#if AP_AIRSPEED_ND210_ENABLED

/*
  backend driver for airspeed from I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"
#include "AP_Airspeed.h"

/*
 * Thanks to PX4 for registers definitions
 */

class AP_Airspeed_ND210 : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_ND210(AP_Airspeed &frontend, uint8_t _instance);
    ~AP_Airspeed_ND210(void) {}

    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void _timer();
    bool _send_configuration_command();
    void _print_info();

    float _temp;  // temperature in Celsius
    float _press; // differential pressure in Pascal
    uint16_t _temp_count;
    uint16_t _press_count;
    float _temp_sum;
    float _press_sum; // pressure accumulator in Pascal
    uint32_t _last_sample_time_ms;
    float _selected_pressure_range;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};


#endif  // AP_AIRSPEED_SDP3X_ENABLED
