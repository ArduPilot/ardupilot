#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

class AP_InertialSensor_PX4 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_PX4(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    // accumulate more samples
    void accumulate(void) override { _get_sample(); }

private:
    bool     _init_sensor(void);
    void     _get_sample(void);
    uint64_t _last_accel_timestamp[INS_MAX_INSTANCES];
    uint64_t _last_gyro_timestamp[INS_MAX_INSTANCES];
    float    _accel_sample_time[INS_MAX_INSTANCES];
    float    _gyro_sample_time[INS_MAX_INSTANCES];

    void _new_accel_sample(uint8_t i, accel_report &accel_report);
    void _new_gyro_sample(uint8_t i, gyro_report &gyro_report);

    bool _get_gyro_sample(uint8_t i, struct gyro_report &gyro_report);
    bool _get_accel_sample(uint8_t i, struct accel_report &accel_report);

    // calculate right queue depth for a sensor
    uint8_t _queue_depth(uint16_t sensor_sample_rate) const;

    // accelerometer and gyro driver handles
    uint8_t _num_accel_instances;
    uint8_t _num_gyro_instances;

    int _accel_fd[INS_MAX_INSTANCES];
    int _gyro_fd[INS_MAX_INSTANCES];

    // indexes in frontend object. Note that these could be different
    // from the backend indexes
    uint8_t _accel_instance[INS_MAX_INSTANCES];
    uint8_t _gyro_instance[INS_MAX_INSTANCES];

#ifdef AP_INERTIALSENSOR_PX4_DEBUG
    uint32_t _gyro_meas_count[INS_MAX_INSTANCES];
    uint32_t _accel_meas_count[INS_MAX_INSTANCES];

    uint32_t _gyro_meas_count_start_us[INS_MAX_INSTANCES];
    uint32_t _accel_meas_count_start_us[INS_MAX_INSTANCES];

    float _gyro_dt_max[INS_MAX_INSTANCES];
    float _accel_dt_max[INS_MAX_INSTANCES];
#endif // AP_INERTIALSENSOR_PX4_DEBUG
};
#endif
