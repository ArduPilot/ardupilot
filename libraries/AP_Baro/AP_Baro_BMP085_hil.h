/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_BMP085_HIL_H__
#define __AP_BARO_BMP085_HIL_H__

#include "AP_Baro.h"

class AP_Baro_BMP085_HIL : public AP_Baro
{
private:
	uint8_t BMP085_State;
    float Temp;
    float Press;
    int32_t _pressure_sum;
    int32_t _temperature_sum;
    uint8_t _count;

public:
    AP_Baro_BMP085_HIL();  // Constructor

	bool init(AP_PeriodicProcess * scheduler);
	uint8_t read();
	float get_pressure();
	float get_temperature();
	int32_t get_raw_pressure();
	int32_t get_raw_temp();
	void setHIL(float Temp, float Press);
};

#endif //  __AP_BARO_BMP085_HIL_H__
