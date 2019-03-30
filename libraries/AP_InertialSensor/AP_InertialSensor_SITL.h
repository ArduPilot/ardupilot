#pragma once

#include <SITL/SITL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#define INS_SITL_INSTANCES 2

class AP_InertialSensor_SITL : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_SITL(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update() override;

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool init_sensor(void);
    void timer_update();
    float gyro_drift(void);
    void generate_accel(uint8_t instance);
    void generate_gyro(uint8_t instance);

    SITL::SITL *sitl;

    // simulated sensor rates in Hz. This matches a pixhawk1
    const uint16_t gyro_sample_hz[INS_SITL_INSTANCES]  { 1000, 760 };
    const uint16_t accel_sample_hz[INS_SITL_INSTANCES] { 1000, 800 };

    uint8_t gyro_instance[INS_SITL_INSTANCES];
    uint8_t accel_instance[INS_SITL_INSTANCES];
    uint64_t next_gyro_sample[INS_SITL_INSTANCES];
    uint64_t next_accel_sample[INS_SITL_INSTANCES];
};
