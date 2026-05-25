#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_LPS22DF_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#define HAL_BARO_LPS22DF_I2C_BUS 0

#ifndef HAL_BARO_LPS22DF_I2C_ADDR
#define HAL_BARO_LPS22DF_I2C_ADDR 0x5D
#endif /* HAL_BARO_LPS22DF_I2C_ADDR */

class AP_Baro_LPS22DF : public AP_Baro_Backend
{
public:
    AP_Baro_LPS22DF(AP_Baro &baro, AP_HAL::Device &dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

private:
    virtual ~AP_Baro_LPS22DF(void) {};

    bool _init(void);
    void _timer(void);
    void _update_temperature(void);
    void _update_pressure(void);

    bool _check_whoami(void);

    AP_HAL::Device *_dev;

    uint8_t _instance;
    float _pressure_sum;
    uint32_t _pressure_count;
    float _temperature;

    uint32_t CallTime = 0;
};

#endif  // AP_BARO_LPS22DF_ENABLED
