/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include <AP_HAL_Linux/qflight/qflight_buffer.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_QFLIGHT : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_QFLIGHT(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool init_sensor(void);
    void timer_update();

    uint8_t gyro_instance;
    uint8_t accel_instance;
    DSPBuffer::IMU *imubuf;
};

#endif
