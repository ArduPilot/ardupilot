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

#if AP_AIRSPEED_AUAV_ENABLED

/*
  backend driver for airspeed from I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

class AUAV_Pressure_sensor
{
public:
    enum class Type {
        Differential,
        Absolute,
    };

    AUAV_Pressure_sensor(AP_HAL::Device *&_dev, Type _type);

    // start a measurement
    // Return true if successful
    bool measure() WARN_IF_UNUSED;

    enum class Status {
        Normal, // Happy, valid reading
        Busy, // Busy, try reading again later
        Fault, // Something bad has happened
    };

    // read the values from the sensor
    // Return status
    // pressure corrected but not scaled to the correct units
    // temperature in degrees centigrade
    Status collect(float &PComp, float &temperature) WARN_IF_UNUSED;

    // Read coefficients in stages
    enum class CoefficientStage {
        A_high,
        A_low,
        B_high,
        B_low,
        C_high,
        C_low,
        D_high,
        D_low,
        E_high,
        E_low,
        Done,
    } stage;

    // Read extended compensation coefficients from sensor
    void read_coefficients();

    // Read 4 bytes compensation coefficient
    bool read_register(uint8_t cmd, uint16_t &result);

private:
    AP_HAL::Device *&dev;

    Type type;

    // Extended compensation coefficients
    // Theses are read from the sensor in read_coefficients
    float LIN_A;
    float LIN_B;
    float LIN_C;
    float LIN_D;
    float Es;
    float TC50H;
    float TC50L;

    // Step of reading coefficients, request must be made before read.
    enum class CoefficientStep {
        request,
        read,
    } coefficient_step;

};

class AP_Airspeed_AUAV : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_AUAV(AP_Airspeed &frontend, uint8_t _instance, const float _range_inH2O);
    ~AP_Airspeed_AUAV(void) {
        delete _dev;
    }

    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    bool probe(uint8_t bus, uint8_t address);
    void _timer();

    uint32_t last_sample_time_ms;
    bool measurement_requested;
    AP_HAL::Device *_dev;

    AUAV_Pressure_sensor sensor { _dev, AUAV_Pressure_sensor::Type::Differential };

    float pressure;
    float temp_C;
    const float range_inH2O;
};

#endif  // AP_Airspeed_AUAV_ENABLED
