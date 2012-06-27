/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_H__
#define __AP_BARO_H__

#include <AP_Param.h>
#include <Filter.h>
#include <AverageFilter.h>
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"

class AP_Baro
{
    public:
	bool healthy;
	AP_Baro() {}
	virtual bool    init(AP_PeriodicProcess *scheduler)=0;
	virtual uint8_t read() = 0;
	virtual int32_t get_pressure() = 0;
	virtual int16_t get_temperature() = 0;
	
	virtual int32_t get_raw_pressure() = 0;
	virtual int32_t get_raw_temp() = 0;

    // calibrate the barometer. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    // the callback is a delay() like routine
    void calibrate(void (*callback)(unsigned long t));

    // get current altitude in meters relative to altitude at the time
    // of the last calibrate() call
    float get_altitude(void);

    // get current climb rate in meters/s. A positive number means
    // going up
    float get_climb_rate(void);

    // the ground values are only valid after calibration
    int16_t get_ground_temperature(void) { return _ground_temperature.get(); }
    int32_t get_ground_pressure(void) { return _ground_pressure.get(); }

	static const struct AP_Param::GroupInfo var_info[];

protected:
    uint32_t _last_update;

private:
    AP_Int16    _ground_temperature;
    AP_Int32    _ground_pressure;
    float       _altitude;
    uint32_t    _last_altitude_t;
    float		_last_altitude;
    float       _climb_rate;
    uint32_t    _last_climb_rate_t;
    AverageFilterFloat_Size5 _climb_rate_filter;
};

#include "AP_Baro_MS5611.h"
#include "AP_Baro_BMP085.h"
#include "AP_Baro_BMP085_hil.h"

#endif // __AP_BARO_H__
