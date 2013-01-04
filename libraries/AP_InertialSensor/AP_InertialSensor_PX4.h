/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_PX4_H__
#define __AP_INERTIAL_SENSOR_PX4_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

class AP_InertialSensor_PX4 : public AP_InertialSensor
{
public:

    AP_InertialSensor_PX4() {}

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    bool            new_data_available();
    float           temperature();
    uint32_t        get_delta_time_micros();
    uint32_t        get_last_sample_time_micros();
    float           get_gyro_drift_rate();
    uint16_t        num_samples_available();

private:
    uint16_t        _init_sensor( Sample_rate sample_rate );
    uint32_t        _last_update_usec;
    uint32_t        _delta_time_usec;

    // accelerometer ORB subscription handle
    int _accel_sub;

    // gyro ORB subscription handle
    int _gyro_sub;

    // raw sensor values, combined structure
    struct sensor_combined_s _raw_sensors;
    
};
#endif
#endif // __AP_INERTIAL_SENSOR_PX4_H__
