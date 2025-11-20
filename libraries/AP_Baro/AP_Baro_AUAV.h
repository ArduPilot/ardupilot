#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_AUAV_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

// Baro uses the airspeed AUAV Pressure sensor class from airspeed, airspeed must be enabled
#include <AP_Airspeed/AP_Airspeed_config.h>
#if !AP_AIRSPEED_AUAV_ENABLED
#error AUAV Baro requires AUAV Airspeed
#endif

#include <AP_Airspeed/AP_Airspeed_AUAV.h>


#ifndef HAL_BARO_AUAV_I2C_ADDR
 #define HAL_BARO_AUAV_I2C_ADDR 0x27
#endif

class AP_Baro_AUAV : public AP_Baro_Backend {
public:
    AP_Baro_AUAV(AP_Baro &baro, AP_HAL::Device *dev);

    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

protected:
    bool init();

    void timer();

    AP_HAL::Device *dev;

    AUAV_Pressure_sensor sensor { dev, AUAV_Pressure_sensor::Type::Absolute };

    uint8_t instance;

    uint32_t count;
    float pressure_sum;
    float temperature_sum;

    bool measurement_requested;
};

#endif  // AP_BARO_AUAV_ENABLED
