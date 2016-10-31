/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_PX4_H__
#define __AP_INERTIAL_SENSOR_PX4_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

class AP_InertialSensor_PX4 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_PX4(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    bool gyro_sample_available(void);
    bool accel_sample_available(void);

private:
    bool     _init_sensor(void);
    void     _get_sample(void);
    bool     _sample_available(void);
    Vector3f _accel_in[INS_MAX_INSTANCES];
    Vector3f _gyro_in[INS_MAX_INSTANCES];
    uint64_t _last_accel_timestamp[INS_MAX_INSTANCES];
    uint64_t _last_gyro_timestamp[INS_MAX_INSTANCES];
    uint64_t _last_accel_update_timestamp[INS_MAX_INSTANCES];
    uint64_t _last_gyro_update_timestamp[INS_MAX_INSTANCES];
    uint64_t _last_get_sample_timestamp;
    uint64_t _last_sample_timestamp;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;
    uint8_t _default_filter_hz;

    void _set_filter_frequency(uint8_t filter_hz);

    // accelerometer and gyro driver handles
    uint8_t _num_accel_instances;
    uint8_t _num_gyro_instances;

    int _accel_fd[INS_MAX_INSTANCES];
    int _gyro_fd[INS_MAX_INSTANCES];

    // indexes in frontend object. Note that these could be different
    // from the backend indexes
    uint8_t _accel_instance[INS_MAX_INSTANCES];
    uint8_t _gyro_instance[INS_MAX_INSTANCES];
};
#endif // CONFIG_HAL_BOARD
#endif // __AP_INERTIAL_SENSOR_PX4_H__
