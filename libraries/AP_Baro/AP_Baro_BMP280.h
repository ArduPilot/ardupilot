#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_BMP280_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_BMP280_I2C_ADDR
 #define HAL_BARO_BMP280_I2C_ADDR  (0x76)
#endif
#ifndef HAL_BARO_BMP280_I2C_ADDR2
 #define HAL_BARO_BMP280_I2C_ADDR2 (0x77)
#endif

class AP_Baro_BMP280 : public AP_Baro_Backend
{
public:
    AP_Baro_BMP280(AP_Baro &baro, AP_HAL::Device &dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

private:

    bool _init(void);
    void _timer(void);
    void _update_temperature(int32_t);
    void _update_pressure(int32_t);

    AP_HAL::Device *_dev;

    uint8_t _instance;
    int32_t _t_fine;
    float _pressure_sum;
    uint32_t _pressure_count;
    float _temperature;

    // Internal calibration registers
    struct PACKED {
        uint16_t t1;
        int16_t  t2;
        int16_t  t3;
        uint16_t p1;
        int16_t  p2;
        int16_t  p3;
        int16_t  p4;
        int16_t  p5;
        int16_t  p6;
        int16_t  p7;
        int16_t  p8;
        int16_t  p9;
    } _calib;
};

#endif  // AP_BARO_BMP280_ENABLED
