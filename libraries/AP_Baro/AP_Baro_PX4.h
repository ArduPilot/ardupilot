/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_PX4_H__
#define __AP_BARO_PX4_H__

#include "AP_Baro.h"

class AP_Baro_PX4 : public AP_Baro
{
public:
	virtual bool init() override;
	virtual uint8_t read() override;
	virtual float get_pressure() const override;
	virtual float get_temperature() const override;
	virtual int32_t get_raw_pressure() const override;
	virtual int32_t get_raw_temp() const override;

private:
    float _temperature;
    float _pressure;
    static float _pressure_sum;
    static float _temperature_sum;
    static uint32_t _sum_count;
    static void _accumulate(void);
    static void _baro_timer(uint32_t now);
    static uint64_t _last_timestamp;
    // baro driver handle
    static int _baro_fd;
    static uint32_t _last_timer;
};

#endif //  __AP_BARO_PX4_H__
