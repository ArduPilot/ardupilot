/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_L3G4200D : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_L3G4200D(AP_InertialSensor &imu);
    ~AP_InertialSensor_L3G4200D();

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    // return product ID
    int16_t product_id() const { return AP_PRODUCT_ID_L3G4200D; }

private:
    bool _init_sensor();
    void _accumulate();

    // gyro and accel instances
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
};
#endif
