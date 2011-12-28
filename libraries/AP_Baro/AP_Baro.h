/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_H__
#define __AP_BARO_H__

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
	virtual float   get_altitude() = 0;
	
	virtual int32_t get_raw_pressure() = 0;
	virtual int32_t get_raw_temp() = 0;
};

#include "AP_Baro_MS5611.h"
#include "AP_Baro_BMP085.h"
#include "AP_Baro_BMP085_hil.h"

#endif // __AP_BARO_H__
