#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_Baro_Backend.h"

#ifndef HAL_BARO_BME280_I2C_ADDR
 #define HAL_BARO_BME280_I2C_ADDR  (0x76)
#endif
#ifndef HAL_BARO_BME280_I2C_ADDR2
 #define HAL_BARO_BME280_I2C_ADDR2 (0x77)
#endif

class AP_Baro_BME280 : public AP_Baro_Backend
{
public:
    AP_Baro_BME280(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:

    bool _init(void);
    void _timer(void);
    void _update_temperature(int32_t);
    void _update_pressure(int32_t);
    void _update_humidity(int32_t);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    bool _has_sample;
    uint8_t _instance;
    int32_t _t_fine;
    float _pressure;
    float _temperature;
    float _humidity;

    // Internal calibration registers
    int8_t _h6;
    int16_t _t2, _t3, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9, _h2, _h4, _h5;
    uint8_t _h1, _h3;
    uint16_t _t1, _p1;
};
