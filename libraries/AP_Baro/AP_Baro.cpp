/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       APM_Baro.cpp - barometer driver
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License
 *   as published by the Free Software Foundation; either version 2.1
 *   of the License, or (at your option) any later version.
 */

#include <math.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Baro::var_info[] PROGMEM = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure

    // @Param: ABS_PRESS
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure
    // @Increment: 1
    AP_GROUPINFO("ABS_PRESS", 2, AP_Baro, _ground_pressure, 0),

    // @Param: ABS_PRESS
    // @DisplayName: ground temperature
    // @Description: calibrated ground temperature
    // @Increment: 1
    AP_GROUPINFO("TEMP", 3, AP_Baro, _ground_temperature, 0),
    AP_GROUPEND
};

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Baro::calibrate()
{
    float ground_pressure = 0;
    float ground_temperature = 0;

    {
        uint32_t tstart = hal.scheduler->millis();
        while (ground_pressure == 0 || !healthy) {
            read();         // Get initial data from absolute pressure sensor
            if (hal.scheduler->millis() - tstart > 500) {
                hal.console->println_P(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [1]\r\n"));
                return;
            }
            ground_pressure         = get_pressure();
            ground_temperature      = get_temperature();
            hal.scheduler->delay(20);
        }
    }
    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    for (uint8_t i = 0; i < 10; i++) {
        uint32_t tstart = hal.scheduler->millis();
        do {
            read();
            if (hal.scheduler->millis() - tstart > 500) {
                hal.console->println_P(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [2]\r\n"));
                return;
            }
        } while (!healthy);
        ground_pressure     = get_pressure();
        ground_temperature  = get_temperature();

        hal.scheduler->delay(100);
    }

    // now average over 5 values for the ground pressure and
    // temperature settings
    for (uint16_t i = 0; i < 5; i++) {
        uint32_t tstart = hal.scheduler->millis();
        do {
            read();
            if (hal.scheduler->millis() - tstart > 500) {
                hal.console->println_P(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [3]\r\n"));
                return;
            }
        } while (!healthy);
        ground_pressure = (ground_pressure * 0.8) + (get_pressure() * 0.2);
        ground_temperature = (ground_temperature * 0.8) + 
            (get_temperature() * 0.2);

        hal.scheduler->delay(100);
    }

    _ground_pressure.set_and_save(ground_pressure);
    _ground_temperature.set_and_save(ground_temperature / 10.0f);
}

// return current altitude estimate relative to time that calibrate()
// was called. Returns altitude in meters
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_altitude(void)
{
    float scaling, temp;

    if (_last_altitude_t == _last_update) {
        // no new information
        return _altitude;
    }

    // this has no filtering of the pressure values, use a separate
    // filter if you want a smoothed value. The AHRS driver wants
    // unsmoothed values
    scaling                                 = (float)_ground_pressure / (float)get_pressure();
    temp                                    = ((float)_ground_temperature) + 273.15f;
    _altitude = log(scaling) * temp * 29.271267f;

    _last_altitude_t = _last_update;

    // ensure the climb rate filter is updated
    _climb_rate_filter.update(_altitude, _last_update);

    return _altitude;
}

// return current climb_rate estimeate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_climb_rate(void)
{
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
    return _climb_rate_filter.slope() * 1.0e3;
}

