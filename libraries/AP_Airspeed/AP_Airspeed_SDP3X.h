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
  backend driver for airspeed from I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"
#include <AP_HAL/I2CDevice.h>
#include "AP_Airspeed.h"

/*
 * Thanks to PX4 for registers definitions
 */

#define SDP3X_SCALE_TEMPERATURE		200.0f

#define SDP3XD0_I2C_ADDR            	0x21
#define SDP3XD1_I2C_ADDR            	0x22
#define SDP3XD2_I2C_ADDR            	0x23

#define SDP3X_RESET_ADDR            	0x00
#define SDP3X_RESET_CMD             	0x06
#define SDP3X_CONT_MEAS_AVG_MODE    	0x3615

#define SPD3X_MEAS_RATE			100
#define SDP3X_MEAS_DRIVER_FILTER_FREQ	3.0f

#define SDP3X_SCALE_PRESSURE_SDP31	60
#define SDP3X_SCALE_PRESSURE_SDP32	240
#define SDP3X_SCALE_PRESSURE_SDP33	20

class AP_Airspeed_SDP3X : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_SDP3X(AP_Airspeed &frontend);
    ~AP_Airspeed_SDP3X(void) {}
    
    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void _collect();
    void _timer();
    float _get_pressure(int16_t dp_raw) const;
    float _get_temperature(int16_t dT_raw) const;
    bool _crc(const uint8_t data[], unsigned size, uint8_t checksum);

    float _temp;
    float _press;
    float _temperature;
    float _pressure;
    uint32_t _last_sample_time_ms;

    uint16_t _scale{0};

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
