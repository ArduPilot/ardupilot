/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	APM_Baro.cpp - barometer driver

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public License
    as published by the Free Software Foundation; either version 2.1
    of the License, or (at your option) any later version.
*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Baro.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_Baro::var_info[] PROGMEM = {
	// @Param: ABS_PRESS
	// @DisplayName: Absolute Pressure
	// @Description: calibrated ground pressure
	// @Increment: 1
    AP_GROUPINFO("ABS_PRESS", 0, AP_Baro, _ground_pressure),

	// @Param: ABS_PRESS
	// @DisplayName: ground temperature
	// @Description: calibrated ground temperature
	// @Increment: 1
    AP_GROUPINFO("TEMP", 1, AP_Baro, _ground_temperature),
    AP_GROUPEND
};

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Baro::calibrate(void (*callback)(unsigned long t))
{
	int32_t ground_pressure = 0;
	int16_t ground_temperature;

	while (ground_pressure == 0 || !healthy) {
		read(); // Get initial data from absolute pressure sensor
		ground_pressure 	= get_pressure();
		ground_temperature 	= get_temperature();
		callback(20);
	}

	for (int i = 0; i < 30; i++) {
		do {
			read();
		} while (!healthy);
		ground_pressure		= (ground_pressure * 9l   + get_pressure()) / 10l;
		ground_temperature	= (ground_temperature * 9 + get_temperature()) / 10;
		callback(20);
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
	scaling 				= (float)_ground_pressure / (float)get_pressure();
	temp 					= ((float)_ground_temperature) + 273.15f;
	_altitude = log(scaling) * temp * 29.271267f;

    _last_altitude_t = _last_update;
    return _altitude;
}

// return current climb_rate estimeate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_climb_rate(void)
{
    float alt, deltat;

    if (_last_climb_rate_t == _last_update) {
        // no new information
        return _climb_rate;
    }
    if (_last_climb_rate_t == 0) {
        // first call
        _last_altitude = get_altitude();
        _last_climb_rate_t = _last_update;
        _climb_rate = 0.0;
        return _climb_rate;
    }

    deltat = (_last_update - _last_climb_rate_t) * 1.0e-3;

    alt = get_altitude();

    // we use a 5 point average filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
    _climb_rate = _climb_rate_filter.apply((alt - _last_altitude) / deltat);
    _last_altitude = alt;
    _last_climb_rate_t = _last_update;

	return _climb_rate;
}
