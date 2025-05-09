#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_BMP581_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_BMP581_I2C_ADDR
 #define HAL_BARO_BMP581_I2C_ADDR  (0x46)
#endif
#ifndef HAL_BARO_BMP581_I2C_ADDR2
 #define HAL_BARO_BMP581_I2C_ADDR2 (0x47)
#endif

class AP_Baro_BMP581 : public AP_Baro_Backend
{
public:
    AP_Baro_BMP581(AP_Baro &baro, AP_HAL::Device &dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

private:

    bool init(void);
    void timer(void);

    AP_HAL::Device *_dev;

    uint8_t instance;
    float pressure_sum;
    uint32_t pressure_count;
    float temperature;
};

#endif  // AP_BARO_BMP581_ENABLED
