/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_FLYMAPLE_H__
#define __AP_INERTIAL_SENSOR_FLYMAPLE_H__

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "AP_InertialSensor.h"
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

class AP_InertialSensor_Flymaple : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_Flymaple(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // accumulate samples
    void accumulate(void) override;

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool        _init_sensor(void);

    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    uint32_t _last_gyro_timestamp;
    uint32_t _last_accel_timestamp;
};
#endif
#endif // __AP_INERTIAL_SENSOR_FLYMAPLE_H__
