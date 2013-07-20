/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Baro.h>
#include "AP_Baro_HIL.h"
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_HIL::init()
{
    BMP085_State=1;
    return true;
}

// Read the sensor. This is a state machine
// We read one time Temperature (state = 1) and then 4 times Pressure (states 2-5)
uint8_t AP_Baro_HIL::read()
{
    uint8_t result = 0;

    if (_count != 0) {
        result = 1;
        Press = ((float)_pressure_sum) / _count;
        Temp = ((float)_temperature_sum) / _count;
        _pressure_samples = _count;
        _count = 0;
        _pressure_sum = 0;
        _temperature_sum = 0;
    }

    return result;
}

void AP_Baro_HIL::setHIL(float altitude_msl)
{
    // approximate a barometer
    // see http://en.wikipedia.org/wiki/Atmospheric_pressure
    const float p0 = 101325;
    const float L = 0.0065;
    const float T0 = 288.15;
    const float M = 0.0289644;
    const float R = 8.31447;

    float p = p0 * pow(1.0 - (L*altitude_msl)/T0, (GRAVITY_MSS*M)/(R*L));
    float T = 31.2 - altitude_msl * L;

    _count++;
    _pressure_sum += p;
    _temperature_sum += T;

    if (_count == 128) {
        // we have summed 128 values. This only happens
        // when we stop reading the barometer for a long time
        // (more than 1.2 seconds)
        _count = 64;
        _pressure_sum /= 2;
        _temperature_sum /= 2;
    }

    healthy = true;
    _last_update = hal.scheduler->millis();
}

float AP_Baro_HIL::get_pressure() {
    return Press;
}

float AP_Baro_HIL::get_temperature() {
    return Temp;
}

int32_t AP_Baro_HIL::get_raw_pressure() {
    return Press;
}

int32_t AP_Baro_HIL::get_raw_temp() {
    return Temp;
}
